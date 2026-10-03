//! Access point mode, behind the `ap` feature.
//!
//! The nRF70 firmware sends the beacons, keeps the links of the stations it is given and tells
//! when one of them dozes, but the rest of an access point is the host's, as hostapd is in the
//! nRF Connect SDK: this module answers probe requests, authenticates and associates stations and
//! adds them to the RPU, and keeps the frames for a station that sleeps until it asks for them.

use core::mem::zeroed;

use defmt::{debug, info, warn};
use embassy_time::Duration;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

use crate::{
    c, find_ie, slice8, slice8_mut, tx_priority, unsliceit, Bus, ConnState, Control, Request, Runner, Ssid,
    CAPABILITY_PRIVACY, EVENT_TIMEOUT, IE_SSID, MTU,
};

/// How many stations the access point takes (the SDK's `CONFIG_WIFI_MGMT_AP_MAX_NUM_STA`).
pub(crate) const MAX_STATIONS: usize = 4;
/// How many frames the driver keeps for stations that sleep, all of them together. Once they are
/// all taken, the runner takes no more frames from embassy-net until a station wakes up and gets
/// its own: TCP then slows down instead of losing segments.
const HELD_FRAMES: usize = 4;
/// Beacon interval, in TU of 1.024 ms (hostapd's default).
const BEACON_INTERVAL: u16 = 100;
/// Beacons between two that announce buffered group frames (hostapd's default).
const DTIM_PERIOD: u8 = 2;
/// How long the RPU gets to start or stop the access point.
const CARRIER_TIMEOUT: Duration = Duration::from_secs(2);

/// Management frame subtypes, as the first byte of the frame control field (type 0).
const ASSOC_REQ: u8 = 0x00;
const ASSOC_RESP: u8 = 0x10;
const REASSOC_REQ: u8 = 0x20;
const REASSOC_RESP: u8 = 0x30;
const PROBE_REQ: u8 = 0x40;
const PROBE_RESP: u8 = 0x50;
const BEACON: u8 = 0x80;
const DISASSOC: u8 = 0xA0;
const AUTH: u8 = 0xB0;
const DEAUTH: u8 = 0xC0;

/// The management frames the RPU hands over in AP mode, as hostapd subscribes to them in the SDK.
const SUBSCRIBED: [u8; 6] = [AUTH, ASSOC_REQ, REASSOC_REQ, DISASSOC, DEAUTH, PROBE_REQ];

const IE_RATES: u8 = 1;
const IE_DS_PARAMS: u8 = 3;
const IE_ERP: u8 = 42;
const IE_EXT_RATES: u8 = 50;

/// Capability information bits.
const CAPABILITY_ESS: u16 = 0x0001;
const CAPABILITY_SHORT_PREAMBLE: u16 = 0x0020;
const CAPABILITY_SHORT_SLOT: u16 = 0x0400;

/// IEEE 802.11 status codes.
const STATUS_SUCCESS: u16 = 0;
const STATUS_UNSPECIFIED: u16 = 1;
const STATUS_AUTH_ALGORITHM: u16 = 13;
const STATUS_AUTH_SEQUENCE: u16 = 14;
const STATUS_TOO_MANY_STATIONS: u16 = 17;
const STATUS_RATES: u16 = 18;

/// IEEE 802.11 reason codes.
const REASON_PREV_AUTH_NOT_VALID: u16 = 2;
const REASON_LEAVING: u16 = 3;
const REASON_CLASS2_FROM_UNAUTHENTICATED: u16 = 6;

/// Rates in 500 kb/s units, a basic rate with its top bit set: on 2.4 GHz the 802.11b ones are
/// basic (hostapd's `hw_mode=g`) and the 802.11g ones go on in the extended element, on 5 GHz 6,
/// 12 and 24 Mb/s are.
const RATES_2G: [u8; 8] = [0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24];
const EXT_RATES_2G: [u8; 4] = [0x30, 0x48, 0x60, 0x6C];
const RATES_5G: [u8; 8] = [0x8C, 0x12, 0x98, 0x24, 0xB0, 0x48, 0x60, 0x6C];

/// Most rates a station is taken with: an 802.11g one has 12.
const MAX_RATES: usize = 16;

/// Why an access point did not start.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum ApError {
    /// The SSID is empty or longer than 32 bytes.
    InvalidSsid,
    /// Not a channel the access point can use: 1 to 13 on 2.4 GHz, 36 to 48 or 149 to 177 on
    /// 5 GHz.
    InvalidChannel,
    /// A scan or a join is in progress.
    Busy,
    /// The chip is off: see [`Control::power_off`].
    PoweredOff,
    /// The chip did not start the access point: a command failed, or it did not answer.
    Refused,
}

/// What an access point is started with.
#[derive(Clone, Copy)]
pub(crate) struct Settings {
    ssid: Ssid,
    channel: u8,
    frequency: u32,
}

impl Settings {
    fn new(ssid: &[u8], channel: u8) -> Result<Self, ApError> {
        if ssid.is_empty() {
            return Err(ApError::InvalidSsid);
        }
        Ok(Self {
            ssid: Ssid::new(ssid).ok_or(ApError::InvalidSsid)?,
            channel,
            frequency: channel_frequency(channel).ok_or(ApError::InvalidChannel)?,
        })
    }

    fn band_2g(&self) -> bool {
        self.frequency < 3000
    }

    fn capability(&self) -> u16 {
        let mut capability = CAPABILITY_ESS | CAPABILITY_SHORT_PREAMBLE;
        if self.band_2g() {
            capability |= CAPABILITY_SHORT_SLOT;
        }
        if self.privacy() {
            capability |= CAPABILITY_PRIVACY;
        }
        capability
    }

    /// Whether data frames are encrypted.
    fn privacy(&self) -> bool {
        false
    }

    fn rates(&self) -> &'static [u8] {
        if self.band_2g() {
            &RATES_2G
        } else {
            &RATES_5G
        }
    }

    /// The fields and elements of a beacon or probe response up to the TIM, which the RPU inserts
    /// in a beacon: timestamp, beacon interval, capability, SSID, rates and channel.
    fn write_head_body(&self, w: &mut Writer) {
        w.put(&[0; 8]) // the timestamp, which the RPU fills in
            .le16(BEACON_INTERVAL)
            .le16(self.capability())
            .ie(IE_SSID, self.ssid.as_bytes())
            .ie(IE_RATES, self.rates());
        if self.band_2g() {
            w.ie(IE_DS_PARAMS, &[self.channel]);
        }
    }

    /// The elements of a beacon or probe response after the TIM.
    fn write_tail(&self, w: &mut Writer) {
        if self.band_2g() {
            // No 802.11b station in the BSS: no protection, long preambles allowed.
            w.ie(IE_ERP, &[0]).ie(IE_EXT_RATES, &EXT_RATES_2G);
        }
    }

    /// The part of a beacon before the TIM.
    fn beacon_head(&self, bssid: &[u8; 6], out: &mut [u8]) -> usize {
        let mut w = Writer::new(out);
        w.header(BEACON, &[0xFF; 6], bssid);
        self.write_head_body(&mut w);
        w.len
    }

    /// The part of a beacon after the TIM.
    fn beacon_tail(&self, out: &mut [u8]) -> usize {
        let mut w = Writer::new(out);
        self.write_tail(&mut w);
        w.len
    }

    fn probe_response(&self, bssid: &[u8; 6], to: &[u8; 6], out: &mut [u8]) -> usize {
        let mut w = Writer::new(out);
        w.header(PROBE_RESP, to, bssid);
        self.write_head_body(&mut w);
        self.write_tail(&mut w);
        w.len
    }

    /// Whether a probe request asks for this network: by its SSID or for any (wildcard SSID).
    fn probed(&self, body: &[u8]) -> bool {
        find_ie(body, IE_SSID).is_some_and(|ssid| ssid.is_empty() || ssid == self.ssid.as_bytes())
    }

    fn association_response(
        &self,
        bssid: &[u8; 6],
        to: &[u8; 6],
        reassoc: bool,
        status: u16,
        aid: u16,
        out: &mut [u8],
    ) -> usize {
        let mut w = Writer::new(out);
        w.header(if reassoc { REASSOC_RESP } else { ASSOC_RESP }, to, bssid)
            .le16(self.capability())
            .le16(status)
            .le16(if status == STATUS_SUCCESS { aid | 0xC000 } else { 0 })
            .ie(IE_RATES, self.rates());
        if self.band_2g() {
            w.ie(IE_EXT_RATES, &EXT_RATES_2G);
        }
        w.len
    }

    /// Checks an association request (its body after the current AP address of a reassociation
    /// request) and returns the station's parameters, or the status code to refuse it with.
    fn association_request(&self, body: &[u8]) -> Result<AssocRequest, u16> {
        let [c0, c1, l0, l1, ies @ ..] = body else {
            return Err(STATUS_UNSPECIFIED);
        };
        if find_ie(ies, IE_SSID) != Some(self.ssid.as_bytes()) {
            return Err(STATUS_UNSPECIFIED);
        }
        let mut request = AssocRequest {
            capability: u16::from_le_bytes([*c0, *c1]),
            listen_interval: u16::from_le_bytes([*l0, *l1]),
            rates: [0; MAX_RATES],
            rates_len: 0,
        };
        let rates = find_ie(ies, IE_RATES).unwrap_or(&[]).iter();
        for rate in rates.chain(find_ie(ies, IE_EXT_RATES).unwrap_or(&[])) {
            if let Some(slot) = request.rates.get_mut(request.rates_len) {
                *slot = rate & 0x7F;
                request.rates_len += 1;
            }
        }
        if request.rates_len == 0 {
            return Err(STATUS_RATES);
        }
        Ok(request)
    }
}

/// The centre frequency of a 20 MHz channel the access point can use, in MHz. 5 GHz channels that
/// need radar detection (52 to 144) are left out.
fn channel_frequency(channel: u8) -> Option<u32> {
    let channel = channel as u32;
    match channel {
        1..=13 => Some(2407 + 5 * channel),
        36 | 40 | 44 | 48 => Some(5000 + 5 * channel),
        149..=177 if (channel - 149).is_multiple_of(4) => Some(5000 + 5 * channel),
        _ => None,
    }
}

/// Builds a frame or a part of one. The buffers are sized for what goes in them: a write past the
/// end is a bug, and panics.
struct Writer<'a> {
    out: &'a mut [u8],
    len: usize,
}

impl<'a> Writer<'a> {
    fn new(out: &'a mut [u8]) -> Self {
        Self { out, len: 0 }
    }

    fn put(&mut self, bytes: &[u8]) -> &mut Self {
        self.out[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        self
    }

    fn le16(&mut self, value: u16) -> &mut Self {
        self.put(&value.to_le_bytes())
    }

    fn ie(&mut self, id: u8, body: &[u8]) -> &mut Self {
        self.put(&[id, body.len() as u8]).put(body)
    }

    /// The 24-byte header of a management frame from the access point.
    fn header(&mut self, subtype: u8, to: &[u8; 6], bssid: &[u8; 6]) -> &mut Self {
        self.put(&[subtype, 0, 0, 0]).put(to).put(bssid).put(bssid).put(&[0, 0])
    }
}

fn authentication_response(bssid: &[u8; 6], to: &[u8; 6], algorithm: u16, status: u16, out: &mut [u8]) -> usize {
    let mut w = Writer::new(out);
    w.header(AUTH, to, bssid).le16(algorithm).le16(2).le16(status);
    w.len
}

fn deauthentication(bssid: &[u8; 6], to: &[u8; 6], reason: u16, out: &mut [u8]) -> usize {
    let mut w = Writer::new(out);
    w.header(DEAUTH, to, bssid).le16(reason);
    w.len
}

/// A received management frame.
struct Mgmt<'a> {
    subtype: u8,
    to: [u8; 6],
    from: [u8; 6],
    bssid: [u8; 6],
    body: &'a [u8],
}

impl<'a> Mgmt<'a> {
    fn parse(frame: &'a [u8]) -> Option<Self> {
        let header = frame.get(..24)?;
        if header[0] & 0x0C != 0 {
            return None;
        }
        let addr = |n: usize| -> [u8; 6] { header[4 + 6 * n..10 + 6 * n].try_into().unwrap() };
        Some(Self {
            subtype: header[0] & 0xF0,
            to: addr(0),
            from: addr(1),
            bssid: addr(2),
            body: &frame[24..],
        })
    }

    fn le16(&self, offset: usize) -> Option<u16> {
        Some(u16::from_le_bytes(self.body.get(offset..offset + 2)?.try_into().ok()?))
    }
}

/// What an association request says about the station.
struct AssocRequest {
    capability: u16,
    listen_interval: u16,
    /// Its rates, in 500 kb/s units, without the basic rate bit.
    rates: [u8; MAX_RATES],
    rates_len: usize,
}

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Phase {
    /// Authenticated, not associated.
    Authenticated,
    /// The association response went out: the station is added once it is acknowledged.
    Responded,
    /// Added to the RPU: data frames flow.
    Associated,
}

#[derive(Clone, Copy)]
struct Station {
    addr: [u8; 6],
    phase: Phase,
    capability: u16,
    listen_interval: u16,
    rates: [u8; MAX_RATES],
    rates_len: u8,
    /// The station dozes: its frames are kept until it asks for them.
    asleep: bool,
    /// While asleep, how many frames it asked for (PS-Poll or U-APSD trigger) and still gets.
    service_period: u8,
}

impl Station {
    fn new(addr: [u8; 6]) -> Self {
        Self {
            addr,
            phase: Phase::Authenticated,
            capability: 0,
            listen_interval: 0,
            rates: [0; MAX_RATES],
            rates_len: 0,
            asleep: false,
            service_period: 0,
        }
    }
}

/// The stations, by slot. A station's association ID is its slot plus one, and its slot is its
/// place in the RPU's table of pending frames.
struct Stations([Option<Station>; MAX_STATIONS]);

impl Stations {
    const fn new() -> Self {
        Self([None; MAX_STATIONS])
    }

    fn find(&self, addr: &[u8; 6]) -> Option<usize> {
        self.0.iter().position(|s| s.is_some_and(|s| s.addr == *addr))
    }

    /// The slot of `addr`, a new one if it has none. `None` if every slot is taken.
    fn find_or_add(&mut self, addr: &[u8; 6]) -> Option<usize> {
        if let Some(slot) = self.find(addr) {
            return Some(slot);
        }
        let slot = self.0.iter().position(Option::is_none)?;
        self.0[slot] = Some(Station::new(*addr));
        Some(slot)
    }

    fn get(&mut self, slot: usize) -> Option<&mut Station> {
        self.0.get_mut(slot)?.as_mut()
    }
}

/// A frame kept for a station that sleeps.
struct Held {
    /// The station's slot, or `None` if this place is free.
    slot: Option<u8>,
    /// Order of arrival.
    seq: u32,
    len: u16,
    frame: [u32; MTU.div_ceil(4)],
}

/// The frames kept for stations that sleep. They live in the application's [`crate::State`], which
/// exists once, rather than in the runner, which is moved around.
pub(crate) struct HeldFrames {
    frames: [Held; HELD_FRAMES],
    next_seq: u32,
}

impl HeldFrames {
    const FREE: Held = Held {
        slot: None,
        seq: 0,
        len: 0,
        frame: [0; MTU.div_ceil(4)],
    };

    pub(crate) const fn new() -> Self {
        Self {
            frames: [Self::FREE; HELD_FRAMES],
            next_seq: 0,
        }
    }

    fn clear(&mut self) {
        for held in &mut self.frames {
            held.slot = None;
        }
    }

    /// The oldest frame of station `slot`.
    fn oldest(&self, slot: usize) -> Option<usize> {
        (0..HELD_FRAMES)
            .filter(|&i| self.frames[i].slot == Some(slot as u8))
            .min_by_key(|&i| self.frames[i].seq)
    }

    fn count(&self, slot: usize) -> usize {
        self.frames.iter().filter(|h| h.slot == Some(slot as u8)).count()
    }

    /// Whether a place is free.
    fn has_room(&self) -> bool {
        self.frames.iter().any(|h| h.slot.is_none())
    }

    /// Keeps a frame for station `slot`. Returns `false` if every place is taken.
    fn hold(&mut self, slot: usize, frame: &[u32], len: usize) -> bool {
        let Some(i) = self.frames.iter().position(|h| h.slot.is_none()) else {
            return false;
        };
        let held = &mut self.frames[i];
        held.slot = Some(slot as u8);
        held.seq = self.next_seq;
        held.len = len as u16;
        held.frame[..len.div_ceil(4)].copy_from_slice(&frame[..len.div_ceil(4)]);
        self.next_seq = self.next_seq.wrapping_add(1);
        true
    }

    /// Drops the frames of station `slot`.
    fn drop_station(&mut self, slot: usize) {
        for held in self.frames.iter_mut().filter(|h| h.slot == Some(slot as u8)) {
            held.slot = None;
        }
    }

    /// The access categories of station `slot`'s frames, one bit each, as the RPU's table of
    /// pending frames wants them (background 0, best effort 1, video 2, voice 3).
    fn categories(&self, slot: usize) -> u32 {
        self.frames
            .iter()
            .filter(|h| h.slot == Some(slot as u8))
            .map(|h| 1 << access_category(tx_priority(&slice8(&h.frame)[..h.len as usize])))
            .fold(0, |a, b| a | b)
    }
}

/// The access category of an 802.1D priority (IEEE 802.11-2020, table 10-1).
fn access_category(priority: u32) -> u32 {
    match priority {
        1 | 2 => 0,
        4 | 5 => 2,
        6 | 7 => 3,
        _ => 1,
    }
}

/// What the runner keeps for the access point.
pub(crate) struct State<'a> {
    /// The access point is up.
    pub(crate) running: bool,
    settings: Option<Settings>,
    mac_addr: [u8; 6],
    stations: Stations,
    held: &'a mut HeldFrames,
    /// The RPU's answer to the last interface type change.
    set_interface: Option<i32>,
    /// Identifies each management frame sent, for the RPU's TX status.
    cookie: u64,
}

impl<'a> State<'a> {
    pub(crate) fn new(held: &'a mut HeldFrames) -> Self {
        Self {
            running: false,
            settings: None,
            mac_addr: [0; 6],
            stations: Stations::new(),
            held,
            set_interface: None,
            cookie: 0,
        }
    }
}

impl Control<'_> {
    /// Starts an open (unencrypted) access point named `ssid` on `channel`: 1 to 13 on 2.4 GHz,
    /// or 36 to 48 or 149 to 177 on 5 GHz, where the country's rules allow. A network joined
    /// until then is left. Returns once the access point beacons and the link is up; up to four
    /// stations may then join it.
    ///
    /// Scans and joins are refused while the access point runs.
    pub async fn start_ap_open(&mut self, ssid: &[u8], channel: u8) -> Result<(), ApError> {
        let settings = Settings::new(ssid, channel)?;
        self.shared.ap_result.clear();
        self.shared.requests.send(Request::StartAp(settings)).await;
        self.shared.ap_result.receive().await
    }

    /// Stops the access point, with a word to each station, and takes the link down. Nothing
    /// happens if none runs.
    pub async fn stop_ap(&mut self) {
        self.shared.ap_result.clear();
        self.shared.requests.send(Request::StopAp).await;
        let _ = self.shared.ap_result.receive().await;
    }
}

/// The runner's side of AP mode. The first items are what `lib.rs` calls, each with an empty
/// counterpart there for a build without the feature.
impl<BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> Runner<'_, BUS, IN, OUT> {
    pub(super) fn ap_running(&self) -> bool {
        self.ap.running
    }

    /// Whether the runner may take a frame from embassy-net: not while the frames kept for
    /// sleeping stations fill every place.
    pub(super) fn ap_has_room(&self) -> bool {
        !self.ap.running || self.ap.held.has_room()
    }

    pub(super) fn set_ap_address(&mut self, mac_addr: [u8; 6]) {
        self.ap.mac_addr = mac_addr;
    }

    /// Starts the access point (hostapd's setup through the SDK's `nrf_wifi_wpa_supp_init_ap`,
    /// `register_mgmt_frames_ap` and `nrf_wifi_wpa_supp_start_ap`).
    pub(super) async fn start_ap(&mut self, settings: Settings) -> Result<(), ApError> {
        let connecting = !matches!(self.conn, ConnState::Idle | ConnState::Connected);
        let scanning = self
            .scan_deadline
            .is_some_and(|deadline| embassy_time::Instant::now() < deadline);
        if connecting || scanning {
            return Err(ApError::Busy);
        }
        if self.ap.running {
            self.stop_ap().await;
        }
        if self.conn == ConnState::Connected {
            self.leave().await;
        }
        if !self.set_interface_type(c::iftype::IFTYPE_AP).await {
            return Err(ApError::Refused);
        }
        for subtype in SUBSCRIBED {
            let mut cmd: c::umac_cmd_mgmt_frame_reg = unsafe { zeroed() };
            cmd.info.frame_type = subtype as u16;
            self.send_cmd(cmd).await;
        }

        let freq_params = freq_params(settings.frequency);
        let mut cmd: c::umac_cmd_set_wiphy = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_SET_WIPHY_FREQ_PARAMS_VALID;
        cmd.info.freq_params = freq_params;
        self.send_cmd(cmd).await;

        let mut cmd: c::umac_cmd_start_ap = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_BEACON_INFO_BEACON_INTERVAL_VALID
            | c::CMD_BEACON_INFO_VERSIONS_VALID
            | c::CMD_BEACON_INFO_CIPHER_SUITE_GROUP_VALID;
        let info = &mut cmd.info;
        info.beacon_interval = BEACON_INTERVAL;
        info.dtim_period = DTIM_PERIOD;
        info.auth_type = c::auth_type::AUTHTYPE_OPEN_SYSTEM as _;
        let beacon = &mut info.beacon_data;
        beacon.head_len = settings.beacon_head(&self.ap.mac_addr, &mut beacon.head) as u32;
        beacon.tail_len = settings.beacon_tail(&mut beacon.tail) as u32;
        info.ssid = settings.ssid.to_c();
        info.connect_common_info.valid_fields = c::CONNECT_COMMON_INFO_WPA_VERSIONS_VALID;
        info.freq_params = freq_params;
        self.carrier_on = false;
        self.send_cmd(cmd).await;
        if !self.wait_until(CARRIER_TIMEOUT, |r| r.carrier_on).await {
            warn!("the access point did not start");
            self.set_interface_type(c::iftype::IFTYPE_STATION).await;
            return Err(ApError::Refused);
        }

        let mut cmd: c::umac_cmd_set_bss = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_SET_BSS_CTS_VALID
            | c::CMD_SET_BSS_PREAMBLE_VALID
            | c::CMD_SET_BSS_SLOT_VALID
            | c::CMD_SET_BSS_HT_OPMODE_VALID
            | c::CMD_SET_BSS_AP_ISOLATE_VALID;
        cmd.bss_info.preamble = 1;
        cmd.bss_info.slot = settings.band_2g() as u8;
        self.send_cmd(cmd).await;

        self.ap.settings = Some(settings);
        self.ap.stations = Stations::new();
        self.ap.held.clear();
        for slot in 0..MAX_STATIONS {
            self.write_pending_entry(slot, &[0; 6]).await;
        }
        self.ap.running = true;
        // Stations that were associated with an earlier run of the access point think they still
        // are: hostapd tells them otherwise when it starts.
        let bssid = self.ap.mac_addr;
        let mut frame = [0; 26];
        let len = deauthentication(&bssid, &[0xFF; 6], REASON_PREV_AUTH_NOT_VALID, &mut frame);
        self.send_mgmt(&frame[..len], true).await;
        self.set_link(true);
        info!(
            "access point {=[u8]:a} up on channel {}",
            settings.ssid.as_bytes(),
            settings.channel
        );
        Ok(())
    }

    /// Stops the access point (the SDK's `nrf_wifi_wpa_supp_deinit_ap`), saying goodbye to each
    /// station.
    pub(super) async fn stop_ap(&mut self) {
        if !self.ap.running {
            return;
        }
        for slot in 0..MAX_STATIONS {
            if self.ap.stations.0[slot].is_some() {
                self.remove_station(slot, Some(REASON_LEAVING)).await;
            }
        }
        self.set_link(false);
        // Without settings no frame is answered any more; the access point counts as running until
        // the RPU has stopped it, so that the station removals it reports meanwhile land here.
        self.ap.settings = None;
        let cmd: c::umac_cmd_stop_ap = unsafe { zeroed() };
        self.send_cmd(cmd).await;
        if !self.wait_until(CARRIER_TIMEOUT, |r| !r.carrier_on).await {
            warn!("the access point did not report its carrier off");
            self.carrier_on = false;
        }
        self.ap.running = false;
        self.set_interface_type(c::iftype::IFTYPE_STATION).await;
        info!("access point down");
    }

    /// The chip was turned off: the access point is gone with it.
    pub(super) fn forget_ap(&mut self) {
        if self.ap.running {
            self.set_link(false);
        }
        self.ap.running = false;
        self.ap.stations = Stations::new();
        self.ap.held.clear();
    }

    /// Takes the interface down, changes its type and brings it up again (the SDK's
    /// `nrf_wifi_iftype_change`). Returns whether the RPU accepted the type.
    async fn set_interface_type(&mut self, iftype: c::iftype) -> bool {
        self.set_interface_state(false).await;
        let mut cmd: c::umac_cmd_chg_vif_attr = unsafe { zeroed() };
        cmd.valid_fields = c::SET_INTERFACE_IFTYPE_VALID | c::SET_INTERFACE_USE_4ADDR_VALID;
        cmd.info.iftype = iftype as _;
        self.ap.set_interface = None;
        self.send_cmd(cmd).await;
        let answered = self.wait_until(EVENT_TIMEOUT, |r| r.ap.set_interface.is_some()).await;
        self.set_interface_state(true).await;
        match self.ap.set_interface {
            Some(0) => true,
            status => {
                warn!(
                    "interface type {} refused: {}",
                    iftype as u32,
                    if answered { status } else { None }
                );
                false
            }
        }
    }

    /// The RPU answered an interface type change.
    pub(super) fn interface_type_set(&mut self, body: &[u8]) {
        self.ap.set_interface = Some(unsliceit::<c::umac_event_set_interface>(body).return_value);
    }

    /// Sends a management frame (the SDK's `nrf_wifi_nl80211_send_mlme`). With `noack`, the RPU
    /// does not wait for an acknowledgement, nor retry.
    async fn send_mgmt(&mut self, frame: &[u8], noack: bool) {
        let Some(settings) = self.ap.settings else {
            return;
        };
        let mut cmd: c::umac_cmd_mgmt_tx = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_FRAME_FREQ_VALID | c::CMD_FRAME_DURATION_VALID | c::CMD_SET_FRAME_FREQ_PARAMS_VALID;
        let info = &mut cmd.info;
        if noack {
            info.flags = c::CMD_FRAME_DONT_WAIT_FOR_ACK;
        }
        info.frequency = settings.frequency;
        info.freq_params = freq_params(settings.frequency);
        info.frame.frame_len = frame.len() as _;
        for (to, from) in info.frame.frame.iter_mut().zip(frame) {
            *to = *from as _;
        }
        self.ap.cookie += 1;
        info.host_cookie = self.ap.cookie;
        self.send_cmd(cmd).await;
    }

    /// A management frame for the access point (`UMAC_EVENT_FRAME`): what hostapd's
    /// `ieee802_11_mgmt` does with it.
    pub(super) async fn ap_frame(&mut self, body: &[u8]) {
        let event: &c::umac_event_mlme = unsliceit(body);
        let Some(settings) = self.ap.settings.filter(|_| self.ap.running) else {
            return;
        };
        let frame = mlme_frame(event);
        let Some(mgmt) = Mgmt::parse(frame) else {
            return;
        };
        let bssid = self.ap.mac_addr;
        if mgmt.subtype != PROBE_REQ && mgmt.bssid != bssid {
            return;
        }
        let mut out = [0u8; 400];
        match mgmt.subtype {
            PROBE_REQ => {
                if settings.probed(mgmt.body) {
                    let len = settings.probe_response(&bssid, &mgmt.from, &mut out);
                    // A probe request to everyone gets its answer once (hostapd's `noack`).
                    self.send_mgmt(&out[..len], mgmt.to == [0xFF; 6]).await;
                }
            }
            AUTH => {
                let (Some(algorithm), Some(sequence)) = (mgmt.le16(0), mgmt.le16(2)) else {
                    return;
                };
                let status = if algorithm != c::auth_type::AUTHTYPE_OPEN_SYSTEM as u16 {
                    STATUS_AUTH_ALGORITHM
                } else if sequence != 1 {
                    STATUS_AUTH_SEQUENCE
                } else {
                    // A station that authenticates again starts over.
                    if let Some(slot) = self.ap.stations.find(&mgmt.from) {
                        self.remove_station(slot, None).await;
                    }
                    match self.ap.stations.find_or_add(&mgmt.from) {
                        Some(_) => STATUS_SUCCESS,
                        None => STATUS_TOO_MANY_STATIONS,
                    }
                };
                debug!("authentication from {:02x}: status {}", mgmt.from, status);
                let len = authentication_response(&bssid, &mgmt.from, algorithm, status, &mut out);
                self.send_mgmt(&out[..len], false).await;
            }
            ASSOC_REQ | REASSOC_REQ => {
                let reassoc = mgmt.subtype == REASSOC_REQ;
                let Some(mut slot) = self.ap.stations.find(&mgmt.from) else {
                    debug!("association request from {:02x}, not authenticated", mgmt.from);
                    let len = deauthentication(&bssid, &mgmt.from, REASON_CLASS2_FROM_UNAUTHENTICATED, &mut out);
                    self.send_mgmt(&out[..len], false).await;
                    return;
                };
                if self.ap.stations.0[slot].is_some_and(|s| s.phase == Phase::Associated) {
                    // Associating again: the RPU's entry goes, the new one comes with the answer.
                    self.remove_station(slot, None).await;
                    slot = self.ap.stations.find_or_add(&mgmt.from).unwrap();
                }
                let body = mgmt.body.get(if reassoc { 6 } else { 0 }..).unwrap_or(&[]);
                let status = match settings.association_request(body) {
                    Ok(request) => {
                        let station = self.ap.stations.get(slot).unwrap();
                        station.capability = request.capability;
                        station.listen_interval = request.listen_interval;
                        station.rates = request.rates;
                        station.rates_len = request.rates_len as u8;
                        station.phase = Phase::Responded;
                        STATUS_SUCCESS
                    }
                    Err(status) => status,
                };
                debug!("association request from {:02x}: status {}", mgmt.from, status);
                let len = settings.association_response(&bssid, &mgmt.from, reassoc, status, slot as u16 + 1, &mut out);
                self.send_mgmt(&out[..len], false).await;
            }
            DISASSOC | DEAUTH => {
                if let Some(slot) = self.ap.stations.find(&mgmt.from) {
                    info!("station {:02x} left, reason {}", mgmt.from, mgmt.le16(0));
                    self.remove_station(slot, None).await;
                }
            }
            _ => {}
        }
    }

    /// The RPU reports a management frame sent (`UMAC_EVENT_FRAME_TX_STATUS`): an acknowledged
    /// association response adds its station (hostapd's `handle_assoc_cb`).
    pub(super) async fn ap_frame_sent(&mut self, body: &[u8]) {
        let event: &c::umac_event_mlme = unsliceit(body);
        let acked = event.flags & c::EVENT_MLME_ACK != 0;
        let Some(mgmt) = Mgmt::parse(mlme_frame(event)) else {
            return;
        };
        if !matches!(mgmt.subtype, ASSOC_RESP | REASSOC_RESP) || mgmt.le16(2) != Some(STATUS_SUCCESS) {
            return;
        }
        let Some(slot) = self.ap.stations.find(&mgmt.to) else {
            return;
        };
        let station = self.ap.stations.get(slot).unwrap();
        if station.phase != Phase::Responded {
            return;
        }
        if !acked {
            debug!("association response to {:02x} not acknowledged", mgmt.to);
            station.phase = Phase::Authenticated;
            return;
        }
        station.phase = Phase::Associated;
        let station = *station;
        self.add_station(slot, &station).await;
        info!("station {:02x} joined, association ID {}", station.addr, slot + 1);
    }

    /// Adds a station to the RPU and opens its port (the SDK's `nrf_wifi_wpa_supp_sta_add` and
    /// `nrf_wifi_wpa_supp_sta_set_flags`).
    async fn add_station(&mut self, slot: usize, station: &Station) {
        let mut cmd: c::umac_cmd_add_sta = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_NEW_STATION_AID_VALID
            | c::CMD_NEW_STATION_STA_CAPABILITY_VALID
            | c::CMD_NEW_STATION_LISTEN_INTERVAL_VALID
            | c::CMD_NEW_STATION_SUPP_RATES_VALID
            | c::CMD_NEW_STATION_STA_FLAGS2_VALID;
        let info = &mut cmd.info;
        info.listen_interval = station.listen_interval as _;
        info.aid = slot as u16 + 1;
        info.sta_capability = station.capability;
        info.supp_rates.num_rates = station.rates_len as _;
        info.supp_rates.rates[..station.rates_len as usize]
            .copy_from_slice(&station.rates[..station.rates_len as usize]);
        info.sta_flags2 = c::sta_flag_update {
            mask: c::STA_FLAG_AUTHORIZED | c::STA_FLAG_SHORT_PREAMBLE | c::STA_FLAG_WME | c::STA_FLAG_MFP,
            set: if station.capability & CAPABILITY_SHORT_PREAMBLE != 0 {
                c::STA_FLAG_SHORT_PREAMBLE
            } else {
                0
            },
        };
        info.mac_addr = station.addr;
        self.send_cmd(cmd).await;
        self.write_pending_entry(slot, &station.addr).await;

        let mut cmd: c::umac_cmd_chg_sta = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_SET_STATION_STA_FLAGS2_VALID;
        cmd.info.mac_addr = station.addr;
        cmd.info.sta_flags2 = c::sta_flag_update {
            mask: c::STA_FLAG_AUTHORIZED,
            set: c::STA_FLAG_AUTHORIZED,
        };
        self.send_cmd(cmd).await;
    }

    /// Forgets the station of `slot` and removes it from the RPU, after telling it why if there
    /// is a `reason` (hostapd's `ap_sta_deauthenticate` and `ap_free_sta`).
    async fn remove_station(&mut self, slot: usize, reason: Option<u16>) {
        let Some(station) = self.ap.stations.0[slot].take() else {
            return;
        };
        self.ap.held.drop_station(slot);
        if let Some(reason) = reason {
            let mut frame = [0; 26];
            let len = deauthentication(&self.ap.mac_addr, &station.addr, reason, &mut frame);
            self.send_mgmt(&frame[..len], false).await;
        }
        if station.phase == Phase::Associated {
            let mut cmd: c::umac_cmd_del_sta = unsafe { zeroed() };
            cmd.valid_fields = c::CMD_DEL_STATION_MAC_ADDR_VALID;
            cmd.info.mac_addr = station.addr;
            self.send_cmd(cmd).await;
            self.write_pending_entry(slot, &[0; 6]).await;
        }
    }

    /// The RPU removed a station on its own (`UMAC_EVENT_DEL_STATION`).
    pub(super) async fn ap_station_removed(&mut self, body: &[u8]) {
        let addr = unsliceit::<c::umac_event_new_station>(body).mac_addr;
        if let Some(slot) = self.ap.stations.find(&addr) {
            if self.ap.stations.0[slot].is_some_and(|s| s.phase == Phase::Associated) {
                info!("station {:02x} removed by the RPU", addr);
                self.ap.stations.0[slot] = None;
                self.ap.held.drop_station(slot);
                self.write_pending_entry(slot, &[0; 6]).await;
            }
        }
    }

    /// Writes station `slot`'s entry of the RPU's table of pending frames, which tells the RPU
    /// what to announce in the TIM (the SDK's `nrf_wifi_fmac_peer_add`): its address, and no
    /// frame pending.
    async fn write_pending_entry(&mut self, slot: usize, addr: &[u8; 6]) {
        let mut entry = [0u32; 3];
        slice8_mut(&mut entry)[..6].copy_from_slice(addr);
        self.write(pending_entry_addr(slot), None, &entry).await;
    }

    /// Updates the pending frame bits of station `slot` (the SDK's `update_pend_q_bmp`).
    async fn write_pending_bits(&mut self, slot: usize) {
        let bits = self.ap.held.categories(slot);
        self.write32(pending_entry_addr(slot) + 8, None, bits).await;
    }

    /// Takes a frame to send if it is for a station that sleeps, and says whether it did (the
    /// SDK's `tx_enqueue` while the station is in `NRF_WIFI_CLIENT_PS_MODE`).
    pub(super) async fn ap_hold(&mut self, frame: &[u32], len: usize) -> bool {
        if !self.ap.running {
            return false;
        }
        let to: [u8; 6] = slice8(frame)[..6].try_into().unwrap();
        let Some(slot) = self.ap.stations.find(&to) else {
            return false;
        };
        let station = self.ap.stations.0[slot].unwrap();
        // Frames go out directly to a station that is awake, or within what it asked for.
        if (!station.asleep || station.service_period > 0) && self.ap.held.count(slot) == 0 {
            return false;
        }
        if !self.ap.held.hold(slot, frame, len) {
            debug!("frame for sleeping station {:02x} dropped", to);
        }
        self.write_pending_bits(slot).await;
        true
    }

    /// A station dozes or wakes up (`CMD_PM_MODE`, the SDK's `sap_client_update_pmmode`).
    pub(super) async fn ap_power_save(&mut self, body: &[u8]) {
        let event: &c::sap_client_pwrsave = unsliceit(body);
        let (addr, state) = (event.mac_addr, event.sta_ps_state);
        let Some(slot) = self.ap.stations.find(&addr) else {
            return;
        };
        let station = self.ap.stations.get(slot).unwrap();
        station.asleep = state as u32 == c::CLIENT_PS_MODE;
        station.service_period = 0;
        debug!(
            "station {:02x} {}",
            addr,
            if station.asleep { "dozes" } else { "awake" }
        );
        self.ap_deliver().await;
    }

    /// A sleeping station asks for frames (`CMD_PS_GET_FRAMES`, the SDK's
    /// `sap_client_ps_get_frames`): a PS-Poll, or a U-APSD trigger.
    pub(super) async fn ap_get_frames(&mut self, body: &[u8]) {
        let event: &c::sap_ps_get_frames = unsliceit(body);
        let (addr, count) = (event.mac_addr, event.num_frames);
        let Some(slot) = self.ap.stations.find(&addr) else {
            return;
        };
        self.ap.stations.get(slot).unwrap().service_period = count.max(1) as u8;
        self.ap_deliver().await;
    }

    /// Sends what the stations may have of their kept frames: everything to one that is awake,
    /// what a sleeping one asked for. Frames that find no TX token free wait for the next call.
    pub(super) async fn ap_deliver(&mut self) {
        if !self.ap.running {
            return;
        }
        for slot in 0..MAX_STATIONS {
            while let Some(station) = self.ap.stations.0[slot] {
                if station.asleep && station.service_period == 0 {
                    break;
                }
                let Some(i) = self.ap.held.oldest(slot) else {
                    if let Some(station) = self.ap.stations.get(slot) {
                        station.service_period = 0;
                    }
                    break;
                };
                let more = self.ap.held.count(slot) > 1;
                let last = !station.asleep || station.service_period == 1 || !more;
                let len = self.ap.held.frames[i].len as usize;
                let mut frame = [0u32; MTU.div_ceil(4)];
                frame[..len.div_ceil(4)].copy_from_slice(&self.ap.held.frames[i].frame[..len.div_ceil(4)]);
                if self.send_frame_flags(&frame, len, more, last).await.is_none() {
                    return;
                }
                self.ap.held.frames[i].slot = None;
                if let Some(station) = self.ap.stations.get(slot) {
                    station.service_period = station.service_period.saturating_sub(1);
                }
                self.write_pending_bits(slot).await;
            }
        }
    }

    /// Handles events until `done` holds, or `timeout` passes. Returns whether `done` held.
    async fn wait_until(&mut self, timeout: Duration, done: impl Fn(&Self) -> bool) -> bool {
        let mut buf = [0u32; crate::MAX_EVENT_LEN / 4];
        embassy_time::with_timeout(timeout, async {
            while !done(self) {
                let len = self.next_event(&mut buf).await;
                self.handle_event(slice8(&buf), len).await;
            }
        })
        .await
        .is_ok()
    }
}

/// The frame of a management frame event or TX status.
fn mlme_frame(event: &c::umac_event_mlme) -> &[u8] {
    let len = { event.frame.frame_len }.clamp(0, 400) as usize;
    // The event is packed: the frame's bytes are reached through a raw pointer, and are C chars,
    // the same bytes.
    let bytes = core::ptr::addr_of!(event.frame.frame) as *const u8;
    unsafe { core::slice::from_raw_parts(bytes, len) }
}

/// Where station `slot`'s entry of the RPU's table of pending frames is.
fn pending_entry_addr(slot: usize) -> u32 {
    c::RPU_MEM_UMAC_PEND_Q_BMP + (slot * size_of::<c::sap_client_pend_frames_bitmap>()) as u32
}

/// A 20 MHz channel, HT allowed.
fn freq_params(frequency: u32) -> c::freq_params {
    c::freq_params {
        valid_fields: c::SET_FREQ_PARAMS_FREQ_VALID
            | c::SET_FREQ_PARAMS_CHANNEL_WIDTH_VALID
            | c::SET_FREQ_PARAMS_CENTER_FREQ1_VALID
            | c::SET_FREQ_PARAMS_CENTER_FREQ2_VALID
            | c::SET_FREQ_PARAMS_CHANNEL_TYPE_VALID,
        frequency: frequency as _,
        channel_width: c::chan_width::CHAN_WIDTH_20 as _,
        center_frequency1: frequency as _,
        center_frequency2: 0,
        channel_type: c::channel_type::CHAN_HT20 as _,
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::assert_eq;
    use std::vec::Vec;

    use super::*;

    const BSSID: [u8; 6] = [0xF4, 0xCE, 0x36, 0x00, 0x8B, 0x19];
    const STA: [u8; 6] = [0x98, 0x43, 0xFA, 0x23, 0x26, 0x25];

    fn hex(s: &str) -> Vec<u8> {
        let s: std::string::String = s.split_whitespace().collect();
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn channels_map_to_frequencies() {
        assert_eq!(channel_frequency(1), Some(2412));
        assert_eq!(channel_frequency(13), Some(2472));
        assert_eq!(channel_frequency(14), None);
        assert_eq!(channel_frequency(36), Some(5180));
        assert_eq!(channel_frequency(48), Some(5240));
        assert_eq!(channel_frequency(40), Some(5200));
        assert_eq!(channel_frequency(38), None);
        // Radar detection needed: left out.
        assert_eq!(channel_frequency(52), None);
        assert_eq!(channel_frequency(100), None);
        assert_eq!(channel_frequency(149), Some(5745));
        assert_eq!(channel_frequency(165), Some(5825));
        assert_eq!(channel_frequency(177), Some(5885));
        assert_eq!(channel_frequency(150), None);
        assert_eq!(channel_frequency(0), None);
    }

    #[test]
    fn settings_check_the_ssid_and_channel() {
        assert!(Settings::new(b"nrf70-ap", 6).is_ok());
        assert_eq!(Settings::new(b"", 6).err(), Some(ApError::InvalidSsid));
        assert_eq!(Settings::new(&[b'a'; 33], 6).err(), Some(ApError::InvalidSsid));
        assert_eq!(Settings::new(b"nrf70-ap", 52).err(), Some(ApError::InvalidChannel));
    }

    #[test]
    fn a_2g_beacon_is_split_around_the_tim() {
        let settings = Settings::new(b"nrf70-ap", 6).unwrap();
        let mut head = [0; 256];
        let len = settings.beacon_head(&BSSID, &mut head);
        assert_eq!(
            head[..len],
            hex("8000 0000 ffffffffffff f4ce36008b19 f4ce36008b19 0000
                 0000000000000000 6400 2104
                 0008 6e726637302d6170
                 0108 82848b960c121824
                 030106")[..]
        );
        let mut tail = [0; 512];
        let len = settings.beacon_tail(&mut tail);
        assert_eq!(tail[..len], hex("2a0100 3204 3048606c")[..]);
    }

    #[test]
    fn a_5g_beacon_has_ofdm_rates_only() {
        let settings = Settings::new(b"nrf70-ap", 36).unwrap();
        let mut head = [0; 256];
        let len = settings.beacon_head(&BSSID, &mut head);
        // ESS and short preamble, no short slot bit, no DS parameter set.
        assert_eq!(head[34..36], [0x21, 0x00]);
        assert_eq!(head[46..len], hex("0108 8c129824b048606c")[..]);
        let mut tail = [0; 512];
        assert_eq!(settings.beacon_tail(&mut tail), 0);
    }

    #[test]
    fn probe_requests_for_the_network_or_any_are_answered() {
        let settings = Settings::new(b"nrf70-ap", 6).unwrap();
        assert!(settings.probed(&hex("0000 0108 82848b960c121824")));
        assert!(settings.probed(&hex("0008 6e726637302d6170")));
        assert!(!settings.probed(&hex("0005 6f74686572")));
        assert!(!settings.probed(&hex("0108 82848b960c121824")));

        let mut out = [0; 400];
        let len = settings.probe_response(&BSSID, &STA, &mut out);
        assert_eq!(out[..4], [0x50, 0, 0, 0]);
        assert_eq!(out[4..10], STA);
        // The head of the beacon after the header, then its tail.
        let mut head = [0; 256];
        let head_len = settings.beacon_head(&BSSID, &mut head);
        assert_eq!(out[24..head_len], head[24..head_len]);
        assert_eq!(out[head_len..len], hex("2a0100 3204 3048606c")[..]);
    }

    #[test]
    fn authentication_and_association_responses() {
        let mut out = [0; 400];
        let len = authentication_response(&BSSID, &STA, 0, STATUS_SUCCESS, &mut out);
        assert_eq!(
            out[..len],
            hex("b000 0000 9843fa232625 f4ce36008b19 f4ce36008b19 0000 0000 0200 0000")[..]
        );

        let settings = Settings::new(b"nrf70-ap", 6).unwrap();
        let len = settings.association_response(&BSSID, &STA, false, STATUS_SUCCESS, 1, &mut out);
        assert_eq!(
            out[..len],
            hex("1000 0000 9843fa232625 f4ce36008b19 f4ce36008b19 0000
                 2104 0000 01c0 0108 82848b960c121824 3204 3048606c")[..]
        );
        let len = settings.association_response(&BSSID, &STA, true, STATUS_RATES, 1, &mut out);
        assert_eq!(out[0], REASSOC_RESP);
        assert_eq!(out[24..30], hex("2104 1200 0000")[..]);
        assert_eq!(len, 30 + 10 + 6);
    }

    #[test]
    fn association_requests_are_checked() {
        let settings = Settings::new(b"nrf70-ap", 6).unwrap();
        let request = settings
            .association_request(&hex(
                "3104 0a00 0008 6e726637302d6170 0108 02040b160c121824 3204 30486c60 dd07 0050f202000100",
            ))
            .ok()
            .unwrap();
        assert_eq!(request.capability, 0x0431);
        assert_eq!(request.listen_interval, 10);
        assert_eq!(
            request.rates[..request.rates_len],
            [0x02, 0x04, 0x0B, 0x16, 0x0C, 0x12, 0x18, 0x24, 0x30, 0x48, 0x6C, 0x60]
        );
        // Another network, no rates, cut short.
        assert_eq!(
            settings
                .association_request(&hex("3104 0a00 0005 6f74686572 0108 02040b160c121824"))
                .err(),
            Some(STATUS_UNSPECIFIED)
        );
        assert_eq!(
            settings
                .association_request(&hex("3104 0a00 0008 6e726637302d6170"))
                .err(),
            Some(STATUS_RATES)
        );
        assert_eq!(
            settings.association_request(&hex("3104 0a")).err(),
            Some(STATUS_UNSPECIFIED)
        );
    }

    #[test]
    fn management_frames_are_parsed() {
        let frame = hex("b000 3a01 f4ce36008b19 9843fa232625 f4ce36008b19 1000 0000 0100 0000");
        let mgmt = Mgmt::parse(&frame).unwrap();
        assert_eq!(mgmt.subtype, AUTH);
        assert_eq!(mgmt.to, BSSID);
        assert_eq!(mgmt.from, STA);
        assert_eq!(mgmt.bssid, BSSID);
        assert_eq!(
            (mgmt.le16(0), mgmt.le16(2), mgmt.le16(4), mgmt.le16(6)),
            (Some(0), Some(1), Some(0), None)
        );
        // A data frame, and one cut short.
        assert!(Mgmt::parse(&hex("0801 0000 f4ce36008b19 9843fa232625 f4ce36008b19 0000")).is_none());
        assert!(Mgmt::parse(&frame[..20]).is_none());
    }

    #[test]
    fn stations_get_the_first_free_slot() {
        let mut stations = Stations::new();
        let addr = |n: u8| [2, 0, 0, 0, 0, n];
        for n in 0..MAX_STATIONS as u8 {
            assert_eq!(stations.find_or_add(&addr(n)), Some(n as usize));
        }
        assert_eq!(stations.find_or_add(&addr(9)), None);
        assert_eq!(stations.find_or_add(&addr(2)), Some(2));
        stations.0[1] = None;
        assert_eq!(stations.find(&addr(1)), None);
        assert_eq!(stations.find_or_add(&addr(9)), Some(1));
    }

    fn ethernet(to: u8, tos: u8) -> ([u32; MTU.div_ceil(4)], usize) {
        let mut frame = [0u32; MTU.div_ceil(4)];
        let bytes = slice8_mut(&mut frame);
        bytes[..6].copy_from_slice(&[2, 0, 0, 0, 0, to]);
        bytes[12..16].copy_from_slice(&[0x08, 0x00, 0x45, tos]);
        (frame, 60)
    }

    #[test]
    fn frames_are_kept_per_station_oldest_first() {
        let mut held = HeldFrames::new();
        let (best_effort, len) = ethernet(1, 0x00);
        let (voice, _) = ethernet(1, 0xE0);
        assert!(held.hold(0, &best_effort, len));
        assert!(held.hold(1, &best_effort, len));
        assert!(held.hold(0, &voice, len));
        assert_eq!(held.count(0), 2);
        assert_eq!(held.categories(0), 0b1010);
        assert_eq!(held.categories(1), 0b0010);
        let first = held.oldest(0).unwrap();
        assert_eq!(held.frames[first].seq, 0);

        // Full: nothing is replaced, the frame is refused.
        assert!(held.has_room());
        assert!(held.hold(1, &voice, len));
        assert!(!held.has_room());
        assert!(!held.hold(1, &voice, len));
        assert_eq!(held.count(0), 2);
        assert_eq!(held.count(1), 2);
        assert_eq!(held.categories(1), 0b1010);

        held.drop_station(0);
        assert_eq!(held.count(0), 0);
        assert_eq!(held.categories(0), 0);
        assert!(held.has_room());
        assert!(held.hold(2, &voice, len));
    }

    #[test]
    fn priorities_map_to_access_categories() {
        assert_eq!([0, 1, 2, 3, 4, 5, 6, 7].map(access_category), [1, 0, 0, 1, 2, 2, 3, 3]);
    }
}
