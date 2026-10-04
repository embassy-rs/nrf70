//! Station mode: joining a network, from the connect scan through authentication and association to
//! the open port, with the access points of the network tried in turn and its channels remembered
//! for the next join.

use core::mem::zeroed;

use defmt::{debug, info, warn};
use embassy_time::{Duration, Instant};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

use crate::command::Command;
use crate::ieee80211::{find_ie, CAPABILITY_PRIVACY, IE_SSID, REASON_LEAVING};
#[cfg(feature = "wpa3")]
use crate::wpa3;
use crate::{c, unsliceit2, Bus, ConnectError, LinkStatus, Runner, EVENT_TIMEOUT, SCAN_TIMEOUT};
#[cfg(feature = "wpa2")]
use crate::{supplicant, wpa2};

/// How many access points of a network a join tries in turn, best first.
const CANDIDATES: usize = 4;

/// How many channels of a network are remembered for the next join.
const KNOWN_CHANNELS: usize = 4;

/// A scan of the remembered channels takes 50 ms per channel, 150 ms where the RPU may only
/// listen. Past this, the join falls back to the scan of every channel.
const KNOWN_SCAN_TIMEOUT: Duration = Duration::from_secs(2);

/// Authentication and association each get this long (wpa_supplicant's `SME_AUTH_TIMEOUT`).
pub(crate) const MLME_TIMEOUT: Duration = Duration::from_secs(5);

/// After association, how long the RPU gets to add the AP as a peer and turn the carrier on.
const LINK_TIMEOUT: Duration = Duration::from_secs(10);

/// How long the 4-way handshake may take once associated (wpa_supplicant gives it as long).
const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(10);

/// What a network is joined with.
// The runner keeps one, for the network it is on: a WPA3 one is its password, identifier and PT.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Copy)]
pub(crate) enum Credentials {
    /// Nothing: an open network.
    Open,
    /// The key of a WPA2-Personal network.
    #[cfg(feature = "wpa2")]
    Wpa2(wpa2::Wpa2),
    /// The password of a WPA3-Personal network.
    #[cfg(feature = "wpa3")]
    Wpa3(wpa3::Wpa3),
}

/// An SSID, up to 32 bytes.
#[derive(Clone, Copy, Default)]
pub(crate) struct Ssid {
    len: u8,
    bytes: [u8; 32],
}

impl Ssid {
    pub(crate) fn new(ssid: &[u8]) -> Option<Self> {
        let mut bytes = [0; 32];
        bytes.get_mut(..ssid.len())?.copy_from_slice(ssid);
        Some(Self {
            len: ssid.len() as u8,
            bytes,
        })
    }

    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }

    pub(crate) fn to_c(self) -> c::ssid {
        c::ssid {
            ssid_len: self.len,
            ssid: self.bytes,
        }
    }
}

/// A scan command followed by the channels to scan: `scan_params.center_frequency`, which the
/// bindings leave as an array of no length.
#[repr(C, packed)]
struct ScanChannelsCmd {
    cmd: c::umac_cmd_scan,
    center_frequency: [u32; KNOWN_CHANNELS],
}

impl Command for ScanChannelsCmd {
    const MESSAGE_TYPE: c::host_rpu_msg_type = c::host_rpu_msg_type::HOST_RPU_MSG_TYPE_UMAC;
    fn fill(&mut self) {
        self.cmd.fill();
    }
}

/// The status code in the frame of an authentication or association event, `offset` bytes in (after
/// the 24-byte header and the fields before the status), or `None` if the RPU reports a timeout.
fn mlme_status(event: &c::umac_event_mlme, offset: usize) -> Option<u16> {
    if event.flags & c::EVENT_MLME_TIMED_OUT != 0 {
        return None;
    }
    let frame = event.frame;
    if (frame.frame_len as usize) < offset + 2 {
        return None;
    }
    Some(u16::from_le_bytes([
        frame.frame[offset] as u8,
        frame.frame[offset + 1] as u8,
    ]))
}

/// The BSSID of the management frame in a MLME event: its third address.
fn mlme_bssid(event: &c::umac_event_mlme) -> Option<[u8; 6]> {
    let frame = event.frame;
    if (frame.frame_len as usize) < 22 {
        return None;
    }
    let mut bssid = [0; 6];
    for (to, from) in bssid.iter_mut().zip(&frame.frame[16..22]) {
        *to = *from as u8;
    }
    Some(bssid)
}

/// The access point chosen by a connect scan: what authentication and association need.
#[derive(Clone, Copy)]
pub(crate) struct Bss {
    pub(crate) bssid: [u8; 6],
    /// MHz.
    pub(crate) frequency: u32,
    pub(crate) capability: u16,
    pub(crate) beacon_interval: u16,
    pub(crate) tsf: u64,
    pub(crate) signal_dbm: i32,
    /// The RSN element it announces, if any.
    #[cfg(feature = "wpa2")]
    pub(crate) rsne: Option<supplicant::Rsne>,
    /// The RSN Extension element it announces, if any.
    #[cfg(feature = "wpa2")]
    pub(crate) rsnxe: Option<supplicant::Rsnxe>,
}

/// The access points of the network that a connect scan found, best first. A join tries the next
/// one when one refuses the station: an access point that steers its stations to its other band
/// does that.
#[derive(Clone, Copy)]
pub(crate) struct Candidates {
    list: [Option<Bss>; CANDIDATES],
}

impl Candidates {
    pub(crate) const fn new() -> Self {
        Self {
            list: [None; CANDIDATES],
        }
    }

    /// Puts `bss` in its place, keeping the best ones. An access point reported twice keeps its
    /// better report.
    fn add(&mut self, bss: Bss) {
        let same = |listed: &Option<Bss>| listed.is_some_and(|listed| listed.bssid == bss.bssid);
        if let Some(i) = self.list.iter().position(same) {
            if self.list[i].is_some_and(|listed| !better_bss(&bss, &listed)) {
                return;
            }
            self.list[i..].rotate_left(1);
            self.list[CANDIDATES - 1] = None;
        }
        let place = self
            .list
            .iter()
            .position(|listed| listed.is_none_or(|listed| better_bss(&bss, &listed)));
        if let Some(place) = place {
            self.list[place..].rotate_right(1);
            self.list[place] = Some(bss);
        }
    }

    /// The best access point left, which leaves the list.
    fn take_best(&mut self) -> Option<Bss> {
        let best = self.list[0].take();
        self.list.rotate_left(1);
        best
    }
}

/// The channels where a scan of every channel found a network, for the next join to scan only
/// these: a fraction of a second instead of several.
#[derive(Clone, Copy)]
pub(crate) struct KnownNetwork {
    pub(crate) ssid: Ssid,
    /// Centre frequency in MHz and rank of the best access point seen on it, best first.
    channels: [(u32, i32); KNOWN_CHANNELS],
    pub(crate) count: usize,
}

impl KnownNetwork {
    pub(crate) fn new(ssid: Ssid) -> Self {
        Self {
            ssid,
            channels: [(0, 0); KNOWN_CHANNELS],
            count: 0,
        }
    }

    /// Notes an access point of the network. When there are more channels than room, the ones
    /// with the best access points stay.
    fn note(&mut self, bss: &Bss) {
        let rank = bss_rank(bss);
        let seen = self.channels[..self.count]
            .iter()
            .position(|(frequency, _)| *frequency == bss.frequency);
        let slot = match seen {
            Some(slot) if self.channels[slot].1 >= rank => return,
            Some(slot) => slot,
            None if self.count < KNOWN_CHANNELS => {
                self.count += 1;
                self.count - 1
            }
            None if self.channels[KNOWN_CHANNELS - 1].1 >= rank => return,
            None => KNOWN_CHANNELS - 1,
        };
        self.channels[slot] = (bss.frequency, rank);
        self.channels[..self.count].sort_unstable_by_key(|(_, rank)| core::cmp::Reverse(*rank));
    }

    fn frequencies(&self) -> [u32; KNOWN_CHANNELS] {
        let mut frequencies = [0; KNOWN_CHANNELS];
        for (slot, (frequency, _)) in frequencies.iter_mut().zip(&self.channels[..self.count]) {
            *slot = *frequency;
        }
        frequencies
    }
}

/// How much weaker a 5 GHz access point may be than a 2.4 GHz one of the same network and still
/// be the one to join.
const BAND_5GHZ_BONUS_DB: i32 = 10;

/// Whether `candidate` is a better access point to join than `best`: the stronger one, a 5 GHz
/// one counting for [`BAND_5GHZ_BONUS_DB`] more than its signal. wpa_supplicant ranks by estimated
/// throughput, which favours 5 GHz in the same way: its channels are wider and less crowded.
fn better_bss(candidate: &Bss, best: &Bss) -> bool {
    bss_rank(candidate) > bss_rank(best)
}

/// What access points are compared by.
fn bss_rank(bss: &Bss) -> i32 {
    bss.signal_dbm + if bss.frequency >= 5000 { BAND_5GHZ_BONUS_DB } else { 0 }
}

/// Progress of joining a network (NCS leaves this to wpa_supplicant's SME).
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub(crate) enum ConnState {
    Idle,
    /// A connect scan for the SSID, keeping the strongest access point.
    Scanning,
    Authenticating,
    Associating,
    /// Associated: waiting for the RPU to add the AP as a peer and turn the carrier on.
    Associated,
    /// WPA2 and WPA3: the 4-way handshake, until both keys are in.
    Handshake,
    /// The port is being opened: waiting for the RPU to have done it.
    Authorizing,
    Connected,
}

impl<'a, BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> Runner<'a, BUS, IN, OUT> {
    /// Starts joining the network `ssid`, leaving the one joined first, if any: a scan of the
    /// channels where the network was last time, if this is the one joined then, or of every
    /// channel.
    pub(crate) async fn join(&mut self, ssid: Ssid, credentials: &Credentials) {
        if self.conn == ConnState::Connected {
            self.leave().await;
        }
        self.conn_ssid = ssid;
        self.conn_credentials = *credentials;
        self.conn_bss = None;
        self.conn_candidates = Candidates::new();
        self.conn_security_mismatch = false;
        match self.known {
            Some(known) if known.ssid.as_bytes() == ssid.as_bytes() && known.count > 0 => {
                debug!("scanning the {} channels the network was on", known.count);
                self.known_scan = true;
                self.set_conn(ConnState::Scanning, KNOWN_SCAN_TIMEOUT);
                self.trigger_connect_scan(Some(&known)).await;
            }
            _ => self.connect_scan_all().await,
        }
    }

    pub(crate) fn set_conn(&mut self, state: ConnState, timeout: Duration) {
        debug!("connection: {}", state);
        self.conn = state;
        self.conn_deadline = Some(Instant::now() + timeout);
    }

    /// Resets the connection state and reports the link down.
    fn reset_conn(&mut self) {
        self.conn = ConnState::Idle;
        self.conn_deadline = None;
        self.peer_known = false;
        self.carrier_on = false;
        self.conn_credentials = Credentials::Open;
        self.forget_keys();
        if self.link_status_requested {
            self.link_status_requested = false;
            let _ = self.shared.link_status.try_send(None);
        }
        self.set_link(false);
    }

    /// Gives up on joining: leaves the AP if already authenticated, and reports `error`. When the
    /// access point refused the station or did not answer, the next one of the network is tried
    /// first, if there is one.
    pub(crate) async fn connect_failed(&mut self, error: ConnectError) {
        warn!("connection failed: {}", error);
        let refused = matches!(
            error,
            ConnectError::AuthenticationRejected(_)
                | ConnectError::AssociationRejected(_)
                | ConnectError::Timeout
                | ConnectError::Disconnected
        ) && matches!(
            self.conn,
            ConnState::Authenticating | ConnState::Associating | ConnState::Associated
        );
        if matches!(
            self.conn,
            ConnState::Associating | ConnState::Associated | ConnState::Handshake | ConnState::Authorizing
        ) {
            self.deauthenticate().await;
        }
        // A PMK cached from an earlier SAE exchange that the access point does not take: SAE again.
        if let Some(bss) = self.pmksa_failed(&error) {
            self.peer_known = false;
            self.carrier_on = false;
            self.forget_keys();
            self.authenticate(bss).await;
            return;
        }
        if refused {
            if let Some(bss) = self.conn_candidates.take_best() {
                info!("trying the next access point of the network");
                self.peer_known = false;
                self.carrier_on = false;
                self.forget_keys();
                self.authenticate(bss).await;
                return;
            }
        }
        // The next try looks at every channel again.
        self.known = None;
        self.reset_conn();
        let _ = self.shared.connect_result.try_send(Err(error));
    }

    /// The RPU or the AP ended the association.
    pub(crate) async fn connection_lost(&mut self) {
        match self.conn {
            ConnState::Idle => {}
            // Nothing is associated yet: the end of the association that a new authentication with
            // the same access point follows (SAE after a cached PMK that did not serve), which the
            // RPU reports after the authentication has started. An authentication that fails ends
            // with its own event, or times out.
            ConnState::Authenticating => debug!("disconnection ignored: not associated"),
            ConnState::Connected => {
                warn!("disconnected");
                self.reset_conn();
            }
            // An AP that cannot verify the handshake gives up on the station after a few tries.
            ConnState::Handshake => self.connect_failed(ConnectError::HandshakeFailed).await,
            _ => self.connect_failed(ConnectError::Disconnected).await,
        }
    }

    /// Whether an event about `bssid` concerns the access point being joined. One left for the
    /// next access point of the network still reports its end, after the next one is in hand.
    fn is_conn_bss(&self, bssid: Option<[u8; 6]>) -> bool {
        let current = match (self.conn_bss, bssid) {
            (Some(bss), Some(bssid)) => bss.bssid == bssid,
            // Nothing being joined.
            (None, _) => false,
            // The event does not say: a timed out request.
            (Some(_), None) => true,
        };
        if !current {
            debug!("event ignored: not about the access point being joined");
        }
        current
    }

    /// Leaves the current network.
    pub(crate) async fn leave(&mut self) {
        self.deauthenticate().await;
        self.reset_conn();
    }

    pub(crate) async fn check_conn_timeout(&mut self) {
        if self.conn_deadline.is_some_and(|deadline| Instant::now() >= deadline) {
            if self.conn == ConnState::Scanning && self.known_scan {
                debug!("no answer on the channels the network was on: scanning every channel");
                return self.connect_scan_all().await;
            }
            let error = match (self.conn, self.conn_bss) {
                (ConnState::Scanning, None) => self.not_found(),
                (ConnState::Handshake, _) => ConnectError::HandshakeFailed,
                _ => ConnectError::Timeout,
            };
            self.connect_failed(error).await;
        }
    }

    /// Why the connect scan kept no access point.
    fn not_found(&self) -> ConnectError {
        if self.conn_security_mismatch {
            ConnectError::SecurityMismatch
        } else {
            ConnectError::NotFound
        }
    }

    /// Scans for the target SSID so that the RPU knows its access points before authentication
    /// (NCS `nrf_wifi_wpa_supp_scan2`), on the channels of `known` only if given.
    async fn trigger_connect_scan(&mut self, known: Option<&KnownNetwork>) {
        let mut cmd: c::umac_cmd_scan = unsafe { zeroed() };
        cmd.info.scan_reason = c::scan_reason::SCAN_CONNECT as _;
        cmd.info.scan_params.num_scan_ssids = 1;
        cmd.info.scan_params.scan_ssids[0] = self.conn_ssid.to_c();
        match known {
            Some(known) => {
                cmd.info.scan_params.num_scan_channels = known.count as _;
                let center_frequency = known.frequencies();
                self.rpu.send_cmd(&mut ScanChannelsCmd { cmd, center_frequency }).await;
            }
            None => self.rpu.send_cmd(&mut cmd).await,
        }
    }

    /// Starts the connect scan of every channel, which also notes where the network is.
    async fn connect_scan_all(&mut self) {
        self.known_scan = false;
        self.known = Some(KnownNetwork::new(self.conn_ssid));
        self.conn_bss = None;
        self.conn_candidates = Candidates::new();
        self.conn_security_mismatch = false;
        self.set_conn(ConnState::Scanning, SCAN_TIMEOUT);
        self.trigger_connect_scan(None).await;
    }

    /// Keeps the best access points of the target SSID, and authenticates with the best one after
    /// the last result (NCS `nrf_wifi_wpa_supp_event_proc_scan_res`).
    pub(crate) async fn handle_scan_result(&mut self, body: &[u8]) {
        if self.conn != ConnState::Scanning {
            return;
        }
        let (event, tail) = unsliceit2::<c::umac_event_new_scan_results>(body);
        let valid = event.valid_fields;
        let ies_len = if valid & c::EVENT_NEW_SCAN_RESULTS_IES_VALID != 0 {
            event.ies_len as usize
        } else {
            0
        };
        let beacon_ies_len = if valid & c::EVENT_NEW_SCAN_RESULTS_BEACON_IES_VALID != 0 {
            event.beacon_ies_len as usize
        } else {
            0
        };
        let ies = tail.get(..ies_len).unwrap_or(&[]);
        let beacon_ies = tail.get(ies_len..ies_len + beacon_ies_len).unwrap_or(&[]);
        let target = self.conn_ssid.as_bytes();
        let matches = find_ie(ies, IE_SSID) == Some(target) || find_ie(beacon_ies, IE_SSID) == Some(target);

        if matches {
            let signal = event.signal;
            let signal_dbm = if signal.signal_type == c::SIGNAL_TYPE_MBM {
                (unsafe { signal.signal.mbm_signal }) as i32 / 100
            } else {
                -100
            };
            let ies_tsf = if valid & c::EVENT_NEW_SCAN_RESULTS_IES_TSF_VALID != 0 {
                event.ies_tsf
            } else {
                0
            };
            let beacon_tsf = if valid & c::EVENT_NEW_SCAN_RESULTS_BEACON_IES_TSF_VALID != 0 {
                event.beacon_ies_tsf
            } else {
                0
            };
            let bss = Bss {
                bssid: event.mac_addr,
                frequency: event.frequency,
                capability: event.capability,
                beacon_interval: event.beacon_interval,
                tsf: ies_tsf.max(beacon_tsf),
                signal_dbm,
                #[cfg(feature = "wpa2")]
                rsne: wpa2::ap_rsne(ies, beacon_ies),
                #[cfg(feature = "wpa2")]
                rsnxe: wpa2::ap_rsnxe(ies, beacon_ies),
            };
            let suitable = match self.conn_credentials {
                Credentials::Open => bss.capability & CAPABILITY_PRIVACY == 0,
                #[cfg(feature = "wpa2")]
                Credentials::Wpa2(_) => bss.rsne.is_some_and(|rsne| rsne.negotiate(false).is_some()),
                #[cfg(feature = "wpa3")]
                Credentials::Wpa3(_) => bss.rsne.is_some_and(|rsne| rsne.negotiate(true).is_some()),
            };
            debug!(
                "found {:02x} at {} MHz, {} dBm, suitable: {}",
                bss.bssid, bss.frequency, bss.signal_dbm, suitable
            );
            if !suitable {
                self.conn_security_mismatch = true;
            } else {
                if let (false, Some(known)) = (self.known_scan, self.known.as_mut()) {
                    known.note(&bss);
                }
                self.conn_candidates.add(bss);
            }
        }

        // The last result has a zero sequence number.
        if event.umac_hdr.seq == 0 {
            match self.conn_candidates.take_best() {
                Some(bss) => self.authenticate(bss).await,
                None if self.known_scan => {
                    debug!("the network is not where it was: scanning every channel");
                    self.connect_scan_all().await;
                }
                None => self.connect_failed(self.not_found()).await,
            }
        }
    }

    /// Authenticates with `bss` (NCS `nrf_wifi_wpa_supp_authenticate`): open system
    /// authentication, or with WPA3 SAE, or an open system one for a cached PMK.
    pub(crate) async fn authenticate(&mut self, bss: Bss) {
        info!(
            "authenticating with {:02x} at {} MHz, {} dBm",
            bss.bssid, bss.frequency, bss.signal_dbm
        );
        let mut cmd = self.auth_cmd(&bss);
        self.conn_bss = Some(bss);
        // With WPA3, SAE in place of open system authentication.
        self.sae_start(&bss, &mut cmd);
        self.set_conn(ConnState::Authenticating, MLME_TIMEOUT);
        self.rpu.send_cmd(&mut cmd).await;
    }

    /// The authenticate command for `bss`: open system authentication.
    pub(crate) fn auth_cmd(&self, bss: &Bss) -> c::umac_cmd_auth {
        let mut cmd: c::umac_cmd_auth = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_AUTHENTICATE_FREQ_VALID | c::CMD_AUTHENTICATE_SSID_VALID;
        cmd.info.frequency = bss.frequency;
        cmd.info.auth_type = c::auth_type::AUTHTYPE_OPEN_SYSTEM as _;
        cmd.info.ssid = self.conn_ssid.to_c();
        cmd.info.bssid = bss.bssid;
        cmd.info.signal = bss.signal_dbm;
        cmd.info.capability = bss.capability;
        cmd.info.beacon_interval = bss.beacon_interval;
        cmd.info.tsf = bss.tsf;
        cmd
    }

    /// Associates with the authenticated AP (NCS `nrf_wifi_wpa_supp_associate`).
    pub(crate) async fn associate(&mut self) {
        let Some(bss) = self.conn_bss else {
            return;
        };
        let mut cmd: c::umac_cmd_assoc = unsafe { zeroed() };
        let info = &mut cmd.connect_common_info;
        info.valid_fields = c::CONNECT_COMMON_INFO_MAC_ADDR_VALID
            | c::CONNECT_COMMON_INFO_SSID_VALID
            | c::CONNECT_COMMON_INFO_FREQ_VALID
            | c::CONNECT_COMMON_INFO_USE_MFP_VALID;
        info.mac_addr = bss.bssid;
        info.ssid = self.conn_ssid.to_c();
        info.frequency = bss.frequency;
        info.use_mfp = 0;
        info.flags = c::CMD_CONNECT_COMMON_INFO_USE_RRM;
        // The RPU keeps the port closed until SET_STATION authorizes it.
        info.control_port = 1;
        // NCS CONFIG_WIFI_MGMT_BSS_MAX_IDLE_TIME, in seconds.
        info.maxidle_insec = 300;
        self.secure_association(&bss, info);
        self.set_conn(ConnState::Associating, MLME_TIMEOUT);
        self.rpu.send_cmd(&mut cmd).await;
    }

    /// Completes the connection once the RPU has added the AP as a peer and turned the carrier
    /// on, which it reports in no fixed order after the association.
    pub(crate) async fn check_associated(&mut self) {
        if self.conn != ConnState::Associated || !self.peer_known || !self.carrier_on {
            return;
        }
        if self.handshake_expected() {
            // The AP starts the 4-way handshake; the port opens once its keys are in.
            self.set_conn(ConnState::Handshake, HANDSHAKE_TIMEOUT);
        } else {
            // An open network has no keys to set up: open the port right away, as
            // wpa_supplicant's ForceAuthorized state does.
            self.open_port().await;
        }
    }

    /// Opens the port, and asks the RPU for the AP's station entry. Its answer is what
    /// [`Self::connected`] waits for.
    ///
    /// The RPU takes commands and frames from different queues. A frame handed over right after
    /// the commands that set the keys and open the port overtakes them and is lost, without a
    /// word from the RPU: a stack that sends its DHCP request the moment the link is up waits for
    /// its retry, 10 s later. The RPU answers commands in order, so once it has answered the
    /// query the port is open.
    pub(crate) async fn open_port(&mut self) {
        let Some(bss) = self.conn_bss else {
            return;
        };
        self.authorize().await;
        self.get_station(bss).await;
        self.set_conn(ConnState::Authorizing, EVENT_TIMEOUT);
    }

    /// Asks the RPU for its station entry of the AP. The answer is `UMAC_EVENT_GET_STATION`.
    pub(crate) async fn get_station(&mut self, bss: Bss) {
        let mut cmd: c::umac_cmd_get_sta = unsafe { zeroed() };
        cmd.info.mac_addr = bss.bssid;
        self.rpu.send_cmd(&mut cmd).await;
    }

    /// Reports the link up: the port is open.
    pub(crate) fn connected(&mut self) {
        self.conn = ConnState::Connected;
        self.conn_deadline = None;
        self.set_link(true);
        info!("connected");
        let _ = self.shared.connect_result.try_send(Ok(()));
    }

    /// Opens the port to the AP (NCS `nrf_wifi_wpa_set_supp_port`).
    async fn authorize(&mut self) {
        let Some(bss) = self.conn_bss else {
            return;
        };
        let mut cmd: c::umac_cmd_chg_sta = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_SET_STATION_STA_FLAGS2_VALID;
        cmd.info.mac_addr = bss.bssid;
        cmd.info.sta_flags2 = c::sta_flag_update {
            mask: c::STA_FLAG_AUTHORIZED,
            set: c::STA_FLAG_AUTHORIZED,
        };
        self.rpu.send_cmd(&mut cmd).await;
    }

    /// Deauthenticates from the AP, reason 3 "leaving" (NCS `nrf_wifi_wpa_supp_deauthenticate`).
    pub(crate) async fn deauthenticate(&mut self) {
        let Some(bss) = self.conn_bss else {
            return;
        };
        let mut cmd: c::umac_cmd_disconn = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_MLME_MAC_ADDR_VALID;
        cmd.info.reason_code = REASON_LEAVING;
        cmd.info.mac_addr = bss.bssid;
        self.rpu.send_cmd(&mut cmd).await;
    }

    /// The answer to the authentication (`UMAC_EVENT_AUTHENTICATE`): an SAE frame, or the status
    /// of an open system authentication, after which the station associates.
    pub(crate) async fn auth_event(&mut self, event: &c::umac_event_mlme) {
        if self.conn != ConnState::Authenticating || !self.is_conn_bss(mlme_bssid(event)) || self.sae_frame(event).await
        {
            return;
        }
        // Authentication frame: header, then algorithm, transaction and status.
        match mlme_status(event, 28) {
            Some(0) => self.associate().await,
            Some(status) => self.connect_failed(ConnectError::AuthenticationRejected(status)).await,
            None => self.connect_failed(ConnectError::Timeout).await,
        }
    }

    /// The answer to the association (`UMAC_EVENT_ASSOCIATE`).
    pub(crate) async fn assoc_event(&mut self, event: &c::umac_event_mlme) {
        if self.conn != ConnState::Associating || !self.is_conn_bss(mlme_bssid(event)) {
            return;
        }
        // Association response: header, then capabilities and status.
        match mlme_status(event, 26) {
            Some(0) => {
                debug!("associated");
                self.set_conn(ConnState::Associated, LINK_TIMEOUT);
                self.check_associated().await;
            }
            Some(status) => self.connect_failed(ConnectError::AssociationRejected(status)).await,
            None => self.connect_failed(ConnectError::Timeout).await,
        }
    }

    /// The RPU added `peer` as a peer (`UMAC_EVENT_NEW_STATION`).
    pub(crate) async fn peer_added(&mut self, peer: [u8; 6]) {
        debug!("AP {:02x} added as peer", peer);
        if self.is_conn_bss(Some(peer)) {
            self.peer_known = true;
            self.check_associated().await;
        }
    }

    /// The RPU removed `peer` as a peer (`UMAC_EVENT_DEL_STATION`).
    pub(crate) async fn peer_removed(&mut self, peer: [u8; 6]) {
        debug!("AP {:02x} removed as peer", peer);
        if self.is_conn_bss(Some(peer)) {
            self.peer_known = false;
            self.connection_lost().await;
        }
    }

    /// A deauthentication or disassociation, event `id`, from the AP or the RPU.
    pub(crate) async fn disconnect_event(&mut self, id: u32, event: &c::umac_event_mlme) {
        // The reason code follows the 24-byte header of the frame.
        let (reason, bssid) = (mlme_status(event, 24), mlme_bssid(event));
        debug!("disconnect event {} from {:02x}, reason {}", id, bssid, reason);
        if self.is_conn_bss(bssid) {
            self.connection_lost().await;
        }
    }

    /// The RPU's station entry of the AP (`UMAC_EVENT_GET_STATION`): the answer to the query that
    /// follows the opening of the port, or to [`Control::link_status`](crate::Control::link_status).
    pub(crate) fn station_info(&mut self, event: &c::umac_event_new_station) {
        if self.conn == ConnState::Authorizing {
            self.connected();
        } else if self.link_status_requested {
            self.link_status_requested = false;
            let info = event.sta_info;
            let status = self.conn_bss.map(|bss| LinkStatus::from_station(&bss, &info));
            let _ = self.shared.link_status.try_send(status);
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    extern crate std;

    use core::{assert, assert_eq};
    use std::vec::Vec;

    use super::*;

    pub(crate) fn bss(frequency: u32, signal_dbm: i32) -> Bss {
        Bss {
            bssid: [0; 6],
            frequency,
            capability: 0,
            beacon_interval: 100,
            tsf: 0,
            signal_dbm,
            #[cfg(feature = "wpa2")]
            rsne: None,
            #[cfg(feature = "wpa2")]
            rsnxe: None,
        }
    }

    #[test]
    fn the_stronger_access_point_is_better_and_5_ghz_counts_for_more() {
        // Same band: the stronger one.
        assert!(better_bss(&bss(2412, -50), &bss(2462, -60)));
        assert!(!better_bss(&bss(2412, -60), &bss(2462, -50)));
        assert!(!better_bss(&bss(5200, -60), &bss(5745, -60)));

        // 5 GHz wins when a little weaker, and loses when much weaker.
        assert!(better_bss(&bss(5200, -60), &bss(2412, -55)));
        assert!(!better_bss(&bss(2412, -55), &bss(5200, -60)));
        assert!(!better_bss(&bss(5200, -70), &bss(2412, -55)));
        assert!(better_bss(&bss(2412, -55), &bss(5200, -70)));
    }

    #[test]
    fn a_join_tries_the_best_access_points_in_turn() {
        let ap = |id: u8, frequency: u32, signal_dbm: i32| Bss {
            bssid: [id; 6],
            ..bss(frequency, signal_dbm)
        };
        let order = |candidates: &mut Candidates| {
            core::iter::from_fn(|| candidates.take_best())
                .map(|bss| bss.bssid[0])
                .collect::<Vec<_>>()
        };

        // Best first, a 5 GHz one counting for 10 dB more.
        let mut candidates = Candidates::new();
        candidates.add(ap(1, 2412, -50));
        candidates.add(ap(2, 5260, -58));
        candidates.add(ap(3, 2437, -70));
        assert_eq!(order(&mut candidates), [2, 1, 3]);
        assert!(candidates.take_best().is_none());

        // An access point reported twice keeps its better report, in its new place.
        let mut candidates = Candidates::new();
        candidates.add(ap(1, 2412, -50));
        candidates.add(ap(2, 2437, -60));
        candidates.add(ap(2, 2437, -40));
        candidates.add(ap(1, 2412, -70));
        assert_eq!(order(&mut candidates), [2, 1]);

        // With more than room, the weakest go.
        let mut candidates = Candidates::new();
        for (id, signal_dbm) in [(1, -80), (2, -50), (3, -70), (4, -60), (5, -90), (6, -40)] {
            candidates.add(ap(id, 2412, signal_dbm));
        }
        assert_eq!(order(&mut candidates), [6, 2, 4, 3]);
    }

    #[test]
    fn the_channels_of_the_best_access_points_are_remembered() {
        let mut known = KnownNetwork::new(Ssid::new(b"network").unwrap());
        assert_eq!((known.count, known.frequencies()), (0, [0; KNOWN_CHANNELS]));

        // Two access points on one channel count once, and the best channel comes first: 5 GHz
        // counts for 10 dB more.
        known.note(&bss(2412, -70));
        known.note(&bss(5200, -65));
        known.note(&bss(2412, -50));
        assert_eq!((known.count, known.frequencies()), (2, [2412, 5200, 0, 0]));
        // A weaker access point on a known channel changes nothing.
        known.note(&bss(5200, -80));
        assert_eq!((known.count, known.frequencies()), (2, [2412, 5200, 0, 0]));

        // With more channels than room, the weakest goes.
        known.note(&bss(2437, -80));
        known.note(&bss(5745, -40));
        assert_eq!((known.count, known.frequencies()), (4, [5745, 2412, 5200, 2437]));
        known.note(&bss(2462, -90));
        assert_eq!(known.frequencies(), [5745, 2412, 5200, 2437]);
        known.note(&bss(2462, -60));
        assert_eq!((known.count, known.frequencies()), (4, [5745, 2412, 5200, 2462]));
    }

    #[test]
    fn association_status_is_read_from_the_response_frame() {
        let mut event: c::umac_event_mlme = unsafe { zeroed() };
        let mut frame = [0i8; 400];
        frame[26] = 17; // status 17: the AP cannot take more stations
        event.frame = c::frame { frame_len: 30, frame };
        assert_eq!(mlme_status(&event, 26), Some(17));
        event.flags = c::EVENT_MLME_TIMED_OUT;
        assert_eq!(mlme_status(&event, 26), None);
    }
}
