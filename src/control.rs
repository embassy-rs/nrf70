//! The application's side: [`Control`], which hands the runner its requests and waits for the
//! answers, and what they return (scan results, the link status, power save, errors).

use defmt::warn;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::with_timeout;

#[cfg(feature = "ap")]
use crate::ap::{self, ApError};
use crate::station::{Bss, Credentials, Ssid};
use crate::{c, EVENT_TIMEOUT, SCAN_TIMEOUT};

/// Scan results buffered between the runner and a [`Scanner`].
const SCAN_RESULTS_DEPTH: usize = 32;

/// What [`Control`] asks of the runner.
// A join carries its credentials, a WPA3 one its password and PT: the channel holds one request.
#[allow(clippy::large_enum_variant)]
pub(crate) enum Request {
    Scan,
    Connect(Ssid, Credentials),
    LinkStatus,
    SetPowerSave(bool),
    PowerSave,
    PowerOff,
    PowerOn,
    #[cfg(feature = "ap")]
    StartAp(ap::Settings),
    #[cfg(feature = "ap")]
    StopAp,
}

/// What a scan hands its [`Scanner`].
pub(crate) enum ScanEvent {
    Bss(BssInfo),
    Done,
    Aborted,
}

/// What [`Control`] and the runner share: the requests, and a channel for each kind of answer.
pub(crate) struct Shared {
    pub(crate) requests: Channel<NoopRawMutex, Request, 1>,
    pub(crate) scan_results: Channel<NoopRawMutex, ScanEvent, SCAN_RESULTS_DEPTH>,
    pub(crate) connect_result: Channel<NoopRawMutex, Result<(), ConnectError>, 1>,
    pub(crate) link_status: Channel<NoopRawMutex, Option<LinkStatus>, 1>,
    pub(crate) power_save: Channel<NoopRawMutex, Option<PowerSave>, 1>,
    pub(crate) power_done: Channel<NoopRawMutex, (), 1>,
    #[cfg(feature = "ap")]
    pub(crate) ap_result: Channel<NoopRawMutex, Result<(), ApError>, 1>,
}

impl Shared {
    pub(crate) const fn new() -> Self {
        Self {
            requests: Channel::new(),
            scan_results: Channel::new(),
            connect_result: Channel::new(),
            link_status: Channel::new(),
            power_save: Channel::new(),
            power_done: Channel::new(),
            #[cfg(feature = "ap")]
            ap_result: Channel::new(),
        }
    }
}

/// Why joining a network failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[non_exhaustive]
pub enum ConnectError {
    /// The SSID is longer than 32 bytes.
    InvalidSsid,
    /// The passphrase of a WPA2-Personal network is not 8 to 63 bytes long, or the password of a
    /// WPA3-Personal one is empty or longer than 128 bytes, or its identifier longer than 32.
    InvalidPassphrase,
    /// A scan or another connection is in progress.
    Busy,
    /// No access point with this SSID answered.
    NotFound,
    /// Access points with this SSID answered, but none offers the security asked for: an open
    /// network for [`Control::join_open`], WPA2-Personal with CCMP for `Control::join_wpa2`, SAE
    /// for `Control::join_wpa3`.
    SecurityMismatch,
    /// The access point refused the authentication, with this IEEE 802.11 status code.
    AuthenticationRejected(u16),
    /// The access point refused the association, with this IEEE 802.11 status code.
    AssociationRejected(u16),
    /// A step did not complete in time.
    Timeout,
    /// The key handshake did not complete: the 4-way handshake, or with WPA3 the SAE confirm. Most
    /// often the passphrase or password is wrong: with WPA2 the access point then ignores the
    /// station and gives no reason, with WPA3 it refuses the confirm.
    HandshakeFailed,
    /// The connection was lost before it completed.
    Disconnected,
    /// The chip is off: see [`Control::power_off`].
    PoweredOff,
}

/// The application's handle on the driver. Each call hands the runner a request and, but for
/// [`Control::set_power_save`], waits for its answer.
pub struct Control<'a> {
    pub(crate) shared: &'a Shared,
}

impl<'a> Control<'a> {
    /// Joins the open (unencrypted) network `ssid`, picking its strongest access point. Returns
    /// once the link is up. embassy-net sees that on its next poll: wait for
    /// `Stack::wait_link_up` before relying on it.
    pub async fn join_open(&mut self, ssid: &[u8]) -> Result<(), ConnectError> {
        self.join(ssid, Credentials::Open).await
    }

    /// Hands the runner a join of `ssid` with `credentials`, and returns its outcome.
    pub(crate) async fn join(&mut self, ssid: &[u8], credentials: Credentials) -> Result<(), ConnectError> {
        let ssid = Ssid::new(ssid).ok_or(ConnectError::InvalidSsid)?;
        self.shared.connect_result.clear();
        self.shared.requests.send(Request::Connect(ssid, credentials)).await;
        self.shared.connect_result.receive().await
    }

    /// The state of the link to the access point: its signal strength and the rates in use, as
    /// the RPU reports them. `None` when the driver is not connected, or if the RPU does not
    /// answer.
    pub async fn link_status(&mut self) -> Option<LinkStatus> {
        self.shared.link_status.clear();
        self.shared.requests.send(Request::LinkStatus).await;
        with_timeout(EVENT_TIMEOUT, self.shared.link_status.receive())
            .await
            .ok()
            .flatten()
    }

    /// Turns 802.11 power save on or off. It is off until asked for.
    ///
    /// With it on, the RPU tells the access point that it dozes, sleeps between beacons, and
    /// collects the frames the access point kept for it when a beacon announces some. The radio
    /// then draws a fraction of what it does listening all the time, and a frame for the station
    /// may wait at the access point until the next beacon the RPU wakes for.
    pub async fn set_power_save(&mut self, enabled: bool) {
        self.shared.requests.send(Request::SetPowerSave(enabled)).await;
    }

    /// The power save settings as the RPU reports them, or `None` if it does not answer or is
    /// off.
    pub async fn power_save(&mut self) -> Option<PowerSave> {
        self.shared.power_save.clear();
        self.shared.requests.send(Request::PowerSave).await;
        with_timeout(EVENT_TIMEOUT, self.shared.power_save.receive())
            .await
            .ok()
            .flatten()
    }

    /// Turns the chip off: its shutdown state, where it draws the least (1.7 µA in the data
    /// sheet). A connection ends first, with a word to the access point, and the link goes down.
    ///
    /// Until [`Control::power_on`], a scan aborts, a join fails with
    /// [`ConnectError::PoweredOff`], and the power save setting is kept for later.
    pub async fn power_off(&mut self) {
        self.shared.power_done.clear();
        self.shared.requests.send(Request::PowerOff).await;
        self.shared.power_done.receive().await;
    }

    /// Turns the chip on again after [`Control::power_off`]: loads its firmware and brings the
    /// interface up, as [`new`](crate::new) does, and restores the power save setting. The network has to be
    /// joined again.
    pub async fn power_on(&mut self) {
        self.shared.power_done.clear();
        self.shared.requests.send(Request::PowerOn).await;
        self.shared.power_done.receive().await;
    }

    /// Starts an active scan of every channel. The results arrive through the returned [`Scanner`].
    pub async fn scan(&mut self) -> Scanner<'_> {
        self.shared.scan_results.clear();
        self.shared.requests.send(Request::Scan).await;
        Scanner {
            shared: self.shared,
            done: false,
        }
    }
}

/// The results of one scan, as the RPU reports them.
pub struct Scanner<'a> {
    shared: &'a Shared,
    done: bool,
}

impl Scanner<'_> {
    /// Returns the next BSS, or `None` once the scan has completed, been aborted or timed out.
    pub async fn next(&mut self) -> Option<BssInfo> {
        if self.done {
            return None;
        }
        let event = match with_timeout(SCAN_TIMEOUT, self.shared.scan_results.receive()).await {
            Ok(event) => event,
            Err(_) => {
                warn!("scan timed out");
                ScanEvent::Aborted
            }
        };
        match event {
            ScanEvent::Bss(bss) => Some(bss),
            ScanEvent::Done => {
                self.done = true;
                None
            }
            ScanEvent::Aborted => {
                warn!("scan aborted");
                self.done = true;
                None
            }
        }
    }
}

/// Frequency band of a BSS.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[non_exhaustive]
pub enum Band {
    /// 2.4 GHz.
    Band2_4GHz,
    /// 5 GHz.
    Band5GHz,
    /// A band the driver does not know, by the RPU's number.
    Unknown(u32),
}

/// Security of a BSS, as the RPU classifies it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[non_exhaustive]
pub enum Security {
    /// No encryption.
    Open,
    /// WEP.
    Wep,
    /// WPA, the version before WPA2.
    Wpa,
    /// WPA2-Personal.
    Wpa2,
    /// WPA2-Personal with PSK-SHA256 key management.
    Wpa2Sha256,
    /// WPA3-Personal (SAE), alone or beside WPA2.
    Wpa3,
    /// WAPI.
    Wapi,
    /// WPA2-Enterprise (802.1X).
    Eap,
    /// Another kind (other enterprise and SHA-384 variants), by the RPU's number.
    Unknown(u32),
}

/// One scan result.
#[derive(Clone, Debug, defmt::Format)]
pub struct BssInfo {
    ssid: [u8; 32],
    ssid_len: u8,
    /// The access point's MAC address.
    pub bssid: [u8; 6],
    /// Its band.
    pub band: Band,
    /// Its channel number.
    pub channel: u32,
    /// Signal strength in dBm, when the RPU reports it in mBm.
    pub rssi: Option<i32>,
    /// The security it announces.
    pub security: Security,
    /// Its beacon interval, in time units (1.024 ms).
    pub beacon_interval: u16,
}

impl BssInfo {
    /// The network's SSID, which may be empty (a hidden network).
    pub fn ssid(&self) -> &[u8] {
        &self.ssid[..self.ssid_len as usize]
    }

    pub(crate) fn from_display_result(r: &c::umac_display_results) -> Self {
        let ssid_len = r.ssid.ssid_len.min(32);
        let band = r.nwk_band as u32;
        let security = r.security_type as u32;
        let signal_type = r.signal.signal_type;
        let mbm = unsafe { r.signal.signal.mbm_signal } as i32;
        Self {
            ssid: r.ssid.ssid,
            ssid_len,
            bssid: r.mac_addr,
            band: match c::band::try_from(band) {
                Ok(c::band::BAND_2GHZ) => Band::Band2_4GHz,
                Ok(c::band::BAND_5GHZ) => Band::Band5GHz,
                _ => Band::Unknown(band),
            },
            channel: r.nwk_channel,
            rssi: (signal_type == c::SIGNAL_TYPE_MBM).then_some(mbm / 100),
            security: match c::security_type::try_from(security) {
                Ok(c::security_type::OPEN) => Security::Open,
                Ok(c::security_type::WEP) => Security::Wep,
                Ok(c::security_type::WPA) => Security::Wpa,
                Ok(c::security_type::WPA2) => Security::Wpa2,
                Ok(c::security_type::WPA2_256) => Security::Wpa2Sha256,
                Ok(
                    c::security_type::WPA3_HNP
                    | c::security_type::WPA3_H2E
                    | c::security_type::WPA3_AUTO
                    | c::security_type::WPA3_FT_SAE,
                ) => Security::Wpa3,
                Ok(c::security_type::WAPI) => Security::Wapi,
                Ok(c::security_type::EAP) => Security::Eap,
                // The other EAP and SHA-384 variants, which the SDK does not map either.
                _ => Security::Unknown(security),
            },
            beacon_interval: r.beacon_interval,
        }
    }
}

/// The link to the access point, as the RPU sees it: see [`Control::link_status`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[non_exhaustive]
pub struct LinkStatus {
    /// The access point.
    pub bssid: [u8; 6],
    /// Its channel's centre frequency, in MHz.
    pub frequency: u32,
    /// Signal strength of the last frame received from it, in dBm. `None` until the RPU has
    /// measured one, shortly after a join.
    pub rssi: Option<i32>,
    /// Rate of the last frame sent to it, in kbit/s.
    pub tx_rate: Option<u32>,
    /// Rate of the last frame received from it, in kbit/s.
    pub rx_rate: Option<u32>,
}

impl LinkStatus {
    /// From the RPU's station entry for the access point `bss`.
    pub(crate) fn from_station(bss: &Bss, info: &c::sta_info) -> Self {
        let valid = info.valid_fields;
        // The RPU gives rates in units of 100 kbit/s.
        let rate = |field: u32, rate: c::rate_info| {
            (valid & field != 0 && rate.valid_fields & c::RATE_INFO_BITRATE_VALID != 0).then_some(rate.bitrate * 100)
        };
        let signal = info.signal;
        Self {
            bssid: bss.bssid,
            frequency: bss.frequency,
            // Right after a join the RPU may mark the signal valid before it has measured one,
            // and gives 0 dBm.
            rssi: (valid & c::STA_INFO_SIGNAL_VALID != 0 && signal != 0).then_some(signal),
            tx_rate: rate(c::STA_INFO_TX_BITRATE_VALID, info.tx_bitrate),
            rx_rate: rate(c::STA_INFO_RX_BITRATE_VALID, info.rx_bitrate),
        }
    }
}

/// The 802.11 power save settings of the RPU: see [`Control::power_save`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[non_exhaustive]
pub struct PowerSave {
    /// Whether power save is on.
    pub enabled: bool,
    /// How long the RPU stays awake after the last frame before it dozes again, in milliseconds.
    pub timeout_ms: u32,
}

#[cfg(test)]
mod tests {
    use core::assert_eq;
    use core::mem::zeroed;

    use super::*;
    use crate::station::tests::bss;

    #[test]
    fn display_result_is_converted() {
        let mut r: c::umac_display_results = unsafe { zeroed() };
        let mut ssid = [0u8; 32];
        ssid[..4].copy_from_slice(b"test");
        r.ssid.ssid = ssid;
        r.ssid.ssid_len = 4;
        r.mac_addr = [1, 2, 3, 4, 5, 6];
        r.nwk_band = c::band::BAND_5GHZ as _;
        r.nwk_channel = 36;
        r.security_type = c::security_type::WPA3_H2E as _;
        r.signal.signal_type = c::SIGNAL_TYPE_MBM;
        r.signal.signal.mbm_signal = -7500i32 as u32;

        let bss = BssInfo::from_display_result(&r);
        assert_eq!(bss.ssid(), b"test");
        assert_eq!(bss.bssid, [1, 2, 3, 4, 5, 6]);
        assert_eq!(bss.band, Band::Band5GHz);
        assert_eq!(bss.channel, 36);
        assert_eq!(bss.rssi, Some(-75));
        assert_eq!(bss.security, Security::Wpa3);
    }

    #[test]
    fn link_status_is_read_from_the_station_entry() {
        let ap = Bss {
            bssid: [1, 2, 3, 4, 5, 6],
            ..bss(5200, -60)
        };
        let mut info: c::sta_info = unsafe { zeroed() };

        // Nothing marked valid: only what the driver knows by itself.
        let status = LinkStatus::from_station(&ap, &info);
        assert_eq!(status.bssid, [1, 2, 3, 4, 5, 6]);
        assert_eq!(status.frequency, 5200);
        assert_eq!((status.rssi, status.tx_rate, status.rx_rate), (None, None, None));

        // 65 Mbit/s out, 58.5 Mbit/s in, as the RPU gives them: in units of 100 kbit/s.
        info.valid_fields = c::STA_INFO_SIGNAL_VALID | c::STA_INFO_TX_BITRATE_VALID | c::STA_INFO_RX_BITRATE_VALID;
        info.signal = -58;
        info.tx_bitrate.valid_fields = c::RATE_INFO_BITRATE_VALID;
        info.tx_bitrate.bitrate = 650;
        info.rx_bitrate.valid_fields = c::RATE_INFO_BITRATE_VALID;
        info.rx_bitrate.bitrate = 585;
        let status = LinkStatus::from_station(&ap, &info);
        assert_eq!(status.rssi, Some(-58));
        assert_eq!(status.tx_rate, Some(65_000));
        assert_eq!(status.rx_rate, Some(58_500));

        // A rate the RPU marks valid in the station entry but not in the rate itself.
        info.rx_bitrate.valid_fields = 0;
        assert_eq!(LinkStatus::from_station(&ap, &info).rx_rate, None);

        // Right after a join: a signal marked valid before the RPU has measured one.
        info.signal = 0;
        let status = LinkStatus::from_station(&ap, &info);
        assert_eq!((status.rssi, status.tx_rate), (None, Some(65_000)));
    }
}
