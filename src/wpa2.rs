//! Joining WPA2-Personal networks: what ties the supplicant (`supplicant.rs`) to the driver. The
//! supplicant decides; this module hands it the AP's EAPOL frames, sends its answers and gives
//! the RPU the keys.
//!
//! All of it is behind the `wpa2` feature. Without it, the runner's hooks into this module are
//! the empty ones in `lib.rs`.

use core::mem::zeroed;

use defmt::{debug, warn};
use embassy_time::{Duration, Instant};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;
use rand_core::CryptoRng;

use crate::data::{write_ethernet, RxFrame};
use crate::ieee80211::{find_ie, IE_RSN, IE_RSNXE};
use crate::rpu::MAX_TX_TOKENS;
use crate::station::{Bss, ConnState, Credentials};
use crate::supplicant::{self, Gtk, Igtk, Outcome, Rsne, Rsnxe, Supplicant};
use crate::{c, slice8_mut, Bus, ConnectError, Control, Runner};

/// How long the last message of a handshake gets to leave before its keys go in anyway.
const KEY_INSTALL_TIMEOUT: Duration = Duration::from_millis(200);

/// How long message 1 of the association's first 4-way handshake waits before it is answered. A
/// newer message 1 meanwhile takes its place, and only that one is answered. An ISP router sends
/// message 1 a second time 15 to 18 ms after the first, with another ANonce, and once the first one
/// has been answered it takes no message 2 until its next try, 1 s later. wpa_supplicant, which
/// handles only the last EAPOL frame that came before it processed the association, answered the
/// second one only, 62 ms after the first on a laptop. hostapd sends message 1 again after 100 ms.
const MESSAGE_1_HOLD: Duration = Duration::from_millis(40);

/// Longest message 1 that waits: the EAPOL-Key frame, and key data up to a PMKID KDE.
const HELD_MESSAGE_1_MAX: usize = supplicant::KEY_FRAME_LEN + 32;

/// Message 1 waiting to be answered, from its 802.1X header on.
struct HeldMessage1 {
    len: u8,
    frame: [u8; HELD_MESSAGE_1_MAX],
    answer_at: Instant,
}

/// Longest EAPOL frame the handshakes are handed: the fixed part of an EAPOL-Key frame, and key
/// data with a few elements and keys.
const EAPOL_MAX: usize = 99 + 400;

/// A key handshake frame, copied out of the buffer it was received in.
pub(crate) struct RxEapol {
    src: [u8; 6],
    len: usize,
    bytes: [u8; EAPOL_MAX],
}

impl RxEapol {
    /// Copies the EAPOL frame `rx` carries. `None`, with a warning, if it is longer than the
    /// handshakes take.
    pub(crate) fn copy(rx: &RxFrame) -> Option<Self> {
        let mut eapol = Self {
            src: rx.src,
            len: rx.payload.len(),
            bytes: [0; EAPOL_MAX],
        };
        let Some(bytes) = eapol.bytes.get_mut(..rx.payload.len()) else {
            warn!("EAPOL frame of {} bytes dropped", rx.payload.len());
            return None;
        };
        bytes.copy_from_slice(rx.payload);
        Some(eapol)
    }
}

/// What a WPA2-Personal network is joined with.
#[derive(Clone, Copy)]
pub(crate) struct Wpa2 {
    psk: [u8; 32],
    /// What the supplicant draws its nonces from.
    nonce_seed: [u8; 32],
}

/// Keys that a completed handshake yielded, waiting for its last message to leave: that message
/// has to go out under the keys in use until then.
struct PendingKeys {
    /// The TX token of the last message.
    token: usize,
    tk: Option<[u8; 16]>,
    gtk: Option<Gtk>,
    igtk: Option<Igtk>,
    /// When the keys go in even if the RPU has not reported the message sent.
    deadline: Instant,
}

/// What the runner keeps for the key handshakes.
pub(crate) struct State {
    /// The station's address.
    mac_addr: [u8; 6],
    /// The key handshakes with the AP of a WPA2 network, from the association on.
    supplicant: Option<Supplicant>,
    pending_keys: Option<PendingKeys>,
    /// Message 1 of the first handshake, waiting [`MESSAGE_1_HOLD`] to be answered.
    held_message_1: Option<HeldMessage1>,
}

impl State {
    pub(crate) const fn new() -> Self {
        Self {
            mac_addr: [0; 6],
            supplicant: None,
            pending_keys: None,
            held_message_1: None,
        }
    }
}

/// What a default key is sent with.
#[derive(Clone, Copy)]
pub(crate) enum DefaultKey {
    /// Unicast data frames: a pairwise key.
    Unicast,
    /// Group data frames: a GTK.
    #[cfg_attr(not(feature = "ap"), allow(dead_code))]
    Multicast,
    /// Group management frames: an IGTK, with management frame protection.
    #[cfg_attr(not(feature = "ap"), allow(dead_code))]
    Management,
}

/// The pre-shared key of the WPA2-Personal network `ssid`, from its passphrase: PBKDF2-HMAC-SHA1
/// with 4096 iterations, which is slow on purpose. `None` if the passphrase is not 8 to 63 bytes
/// long. The key depends on nothing else, so it can be stored in place of the passphrase.
pub fn wpa2_psk(ssid: &[u8], passphrase: &[u8]) -> Option<[u8; 32]> {
    supplicant::psk_from_passphrase(passphrase, ssid)
}

impl Control<'_> {
    /// Joins the WPA2-Personal network `ssid` with its passphrase, picking its strongest access
    /// point that offers CCMP as pairwise cipher (the group cipher may be TKIP, in WPA/WPA2 mixed
    /// mode), with management frame protection where the access point offers it. Returns once
    /// the 4-way handshake is done, the keys are in and the link is up; see
    /// [`Control::join_open`].
    ///
    /// On a mixed mode network the chip does not check the Michael MIC of the group frames it
    /// receives, and reports no MIC failure: their integrity rests on TKIP's CRC alone.
    ///
    /// The nRF70 firmware has no supplicant, so the handshake runs here, on the host, and it needs
    /// random numbers for its nonces. They come from `rng`, any cryptographically secure generator
    /// the application has: a hardware one, or a software one seeded from real entropy. Each call
    /// takes 32 bytes from it, before joining.
    ///
    /// Deriving the key from the passphrase takes 16,384 SHA-1 compressions, during which this
    /// task does not yield. [`wpa2_psk`] and [`Control::join_wpa2_psk`] let an application do it
    /// once and keep the result.
    pub async fn join_wpa2(
        &mut self,
        ssid: &[u8],
        passphrase: &[u8],
        rng: &mut (impl CryptoRng + ?Sized),
    ) -> Result<(), ConnectError> {
        let psk = wpa2_psk(ssid, passphrase).ok_or(ConnectError::InvalidPassphrase)?;
        self.join_wpa2_psk(ssid, &psk, rng).await
    }

    /// [`Control::join_wpa2`] with the pre-shared key that [`wpa2_psk`] derives from the
    /// passphrase.
    pub async fn join_wpa2_psk(
        &mut self,
        ssid: &[u8],
        psk: &[u8; 32],
        rng: &mut (impl CryptoRng + ?Sized),
    ) -> Result<(), ConnectError> {
        // The runner does the handshake and has no generator of its own: it gets a seed, from
        // which the supplicant derives a nonce for each handshake of this association.
        let mut nonce_seed = [0; 32];
        rng.fill_bytes(&mut nonce_seed);
        self.join(ssid, Credentials::Wpa2(Wpa2 { psk: *psk, nonce_seed })).await
    }
}

/// The RSN element an access point announces: the one of its probe response, or else its
/// beacon's. Message 3 of the 4-way handshake has to repeat it.
pub(crate) fn ap_rsne(ies: &[u8], beacon_ies: &[u8]) -> Option<Rsne> {
    find_ie(ies, IE_RSN)
        .or_else(|| find_ie(beacon_ies, IE_RSN))
        .and_then(Rsne::from_body)
}

/// The RSN Extension element an access point announces, if any: in its probe response, or else
/// its beacon. Message 3 of the 4-way handshake has to repeat it.
pub(crate) fn ap_rsnxe(ies: &[u8], beacon_ies: &[u8]) -> Option<Rsnxe> {
    find_ie(ies, IE_RSNXE)
        .or_else(|| find_ie(beacon_ies, IE_RSNXE))
        .and_then(Rsnxe::from_body)
}

/// The runner's side of WPA2. The first items are what the rest of the runner calls, each
/// with an empty counterpart in `lib.rs` for a build without the feature.
impl<BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> Runner<'_, BUS, IN, OUT> {
    /// One TX token stays free for the supplicant, whose answers must not wait for traffic.
    pub(super) const EAPOL_TX_TOKENS: u32 = 1;

    pub(super) fn set_station_address(&mut self, mac_addr: [u8; 6]) {
        self.wpa2.mac_addr = mac_addr;
    }

    /// Prepares the association with `bss` when the network is a WPA2 one: the association
    /// request carries the RSN element the supplicant will repeat in the handshake, and the RPU
    /// keeps the port closed until the keys are in.
    pub(super) fn secure_association(&mut self, bss: &Bss, info: &mut c::connect_common_info) {
        self.wpa2.supplicant = match (self.conn_credentials, bss.rsne) {
            (Credentials::Wpa2(wpa2), Some(ap_rsne)) => {
                Supplicant::new(wpa2.psk, bss.bssid, self.wpa2.mac_addr, ap_rsne, wpa2.nonce_seed, false)
            }
            // WPA3: the PMK of the SAE exchange that just went through, or of an earlier one.
            #[cfg(feature = "wpa3")]
            (Credentials::Wpa3(wpa3), Some(ap_rsne)) => self
                .wpa3
                .pmk
                .and_then(|pmk| Supplicant::new(pmk, bss.bssid, self.wpa2.mac_addr, ap_rsne, wpa3.seed, true))
                .map(|supplicant| supplicant.with_pmkid(self.wpa3.pmkid)),
            _ => None,
        }
        .map(|supplicant| supplicant.with_rsnxe(self.association_rsnxe(), bss.rsnxe));
        if let Some(supplicant) = &self.wpa2.supplicant {
            let suite = supplicant.suite();
            debug!("WPA2 association: {}", suite);
            if suite.mfp {
                info.use_mfp = c::mfp::MFP_REQUIRED as _;
            }
            let rsne = supplicant.rsne().as_bytes();
            let rsnxe = self.association_rsnxe();
            let more = rsnxe.as_ref().map_or(&[][..], Rsnxe::as_bytes);
            info.valid_fields |= c::CONNECT_COMMON_INFO_WPA_IE_VALID;
            info.flags |= c::CONNECT_COMMON_INFO_SECURITY;
            info.wpa_ie.ie_len = (rsne.len() + more.len()) as u16;
            for (to, from) in info.wpa_ie.ie.iter_mut().zip(rsne.iter().chain(more)) {
                *to = *from as _;
            }
        }
    }

    /// The RSNXE the association request carries beside the RSNE: with WPA3 and hash to
    /// element.
    fn association_rsnxe(&self) -> Option<Rsnxe> {
        #[cfg(feature = "wpa3")]
        return self.sae_rsnxe();
        #[cfg(not(feature = "wpa3"))]
        return None;
    }

    /// The station's address.
    #[cfg(feature = "wpa3")]
    pub(super) fn wpa2_mac_addr(&self) -> [u8; 6] {
        self.wpa2.mac_addr
    }

    /// Whether the association is followed by a 4-way handshake, which the AP starts.
    pub(super) fn handshake_expected(&self) -> bool {
        self.wpa2.supplicant.is_some()
    }

    /// Hands a key handshake frame to the supplicant, or with an access point running, to its
    /// authenticator.
    pub(super) async fn rx_eapol(&mut self, eapol: RxEapol) {
        let payload = &eapol.bytes[..eapol.len];
        #[cfg(feature = "ap")]
        if self.ap_running() {
            return self.ap_eapol(&eapol.src, payload).await;
        }
        self.handle_eapol(&eapol.src, payload).await;
    }

    /// The RPU reported the frame of TX token `token` sent. If that was the last message of a
    /// handshake, its keys go in.
    pub(super) async fn frame_sent(&mut self, token: usize) {
        if self.wpa2.pending_keys.as_ref().is_some_and(|keys| keys.token == token) {
            self.install_pending_keys().await;
        }
    }

    /// Answers message 1 once it has waited, and installs the keys of a handshake whose last
    /// message the RPU did not report sent.
    pub(super) async fn check_handshake_timers(&mut self) {
        let now = Instant::now();
        if self
            .wpa2
            .held_message_1
            .as_ref()
            .is_some_and(|held| now >= held.answer_at)
        {
            if let Some(held) = self.wpa2.held_message_1.take() {
                self.answer_eapol(&held.frame[..held.len as usize]).await;
            }
        }
        if self
            .wpa2
            .pending_keys
            .as_ref()
            .is_some_and(|keys| Instant::now() >= keys.deadline)
        {
            debug!("the last handshake message was not reported sent");
            self.install_pending_keys().await;
        }
    }

    /// When [`Self::check_handshake_timers`] has to run at the latest.
    pub(super) fn handshake_deadline(&self) -> Option<Instant> {
        let held = self.wpa2.held_message_1.as_ref().map(|held| held.answer_at);
        let keys = self.wpa2.pending_keys.as_ref().map(|keys| keys.deadline);
        held.into_iter().chain(keys).min()
    }

    /// The association is over: its keys and its supplicant go with it.
    pub(super) fn forget_keys(&mut self) {
        self.wpa2.supplicant = None;
        self.wpa2.pending_keys = None;
        self.wpa2.held_message_1 = None;
    }

    /// Takes an EAPOL frame from `src`, the AP of the association: message 1 of the first handshake
    /// waits, the rest goes to [`Self::answer_eapol`].
    async fn handle_eapol(&mut self, src: &[u8; 6], eapol: &[u8]) {
        let (Some(bss), true) = (self.conn_bss, self.wpa2.supplicant.is_some()) else {
            debug!("EAPOL frame ignored: no WPA2 association");
            return;
        };
        if !matches!(
            self.conn,
            ConnState::Handshake | ConnState::Authorizing | ConnState::Connected
        ) || *src != bss.bssid
        {
            debug!("EAPOL frame ignored: not from the AP of an association");
            return;
        }
        // Message 1 of the first handshake waits a moment, in case a newer one follows (see
        // MESSAGE_1_HOLD).
        if self.conn == ConnState::Handshake && supplicant::is_message_1(eapol) && eapol.len() <= HELD_MESSAGE_1_MAX {
            let answer_at =
                (self.wpa2.held_message_1.as_ref()).map_or(Instant::now() + MESSAGE_1_HOLD, |held| held.answer_at);
            let mut held = HeldMessage1 {
                len: eapol.len() as u8,
                frame: [0; HELD_MESSAGE_1_MAX],
                answer_at,
            };
            held.frame[..eapol.len()].copy_from_slice(eapol);
            if self.wpa2.held_message_1.replace(held).is_some() {
                debug!("4-way handshake: a newer message 1 takes the place of the one waiting");
            }
            return;
        }
        self.answer_eapol(eapol).await;
    }

    /// Hands an EAPOL frame from the AP, from its 802.1X header on, to the supplicant, and does
    /// what it says.
    async fn answer_eapol(&mut self, eapol: &[u8]) {
        let Some(supplicant) = self.wpa2.supplicant.as_mut() else {
            return;
        };
        let mut reply = [0; supplicant::REPLY_MAX];
        match supplicant.handle(eapol, &mut reply) {
            Outcome::Ignored => {}
            Outcome::Reply(len) => {
                self.send_eapol(&reply[..len]).await;
            }
            Outcome::Keys {
                reply: len,
                tk,
                gtk,
                igtk,
            } => {
                // The keys of a handshake go in once its last message has left. If one is
                // still waiting, its keys go in now.
                self.install_pending_keys().await;
                let token = self.send_eapol(&reply[..len]).await;
                self.wpa2.pending_keys = Some(PendingKeys {
                    // Without a token the message was not sent, and the AP will repeat its own.
                    token: token.unwrap_or(MAX_TX_TOKENS),
                    tk,
                    gtk,
                    igtk,
                    deadline: Instant::now() + KEY_INSTALL_TIMEOUT,
                });
                if token.is_none() {
                    self.install_pending_keys().await;
                }
            }
            Outcome::Abort => {
                warn!("the AP's handshake contradicts what it announced, leaving");
                if self.conn == ConnState::Connected {
                    self.leave().await;
                } else {
                    self.connect_failed(ConnectError::HandshakeFailed).await;
                }
            }
        }
    }

    /// Sends an EAPOL frame to the AP. Returns its TX token, or `None` if none was free.
    async fn send_eapol(&mut self, eapol: &[u8]) -> Option<usize> {
        let bss = self.conn_bss?;
        let mut frame = [0u32; (14 + supplicant::REPLY_MAX).div_ceil(4)];
        let len = write_ethernet(
            slice8_mut(&mut frame),
            &bss.bssid,
            &self.wpa2.mac_addr,
            supplicant::ETHERTYPE_EAPOL,
            eapol,
        )?;
        let token = self.send_frame(&frame, len).await;
        if token.is_none() {
            warn!("EAPOL frame not sent: no TX token free");
        }
        token
    }

    /// Gives the RPU the keys of a completed handshake (wpa_supplicant's
    /// `wpa_supplicant_install_ptk` and `wpa_supplicant_install_gtk`), and opens the port if it
    /// was the first one.
    async fn install_pending_keys(&mut self) {
        let (Some(keys), Some(bss)) = (self.wpa2.pending_keys.take(), self.conn_bss) else {
            return;
        };
        if let Some(tk) = keys.tk {
            // A new pairwise key starts counting packets from zero.
            self.add_key(Some(bss.bssid), supplicant::CIPHER_SUITE_CCMP, 0, &tk, &[0; 6])
                .await;
            self.set_default_key(0, DefaultKey::Unicast).await;
            debug!("pairwise key installed");
        }
        if let Some(gtk) = keys.gtk {
            self.add_key(None, gtk.cipher.cipher_suite(), gtk.index, gtk.key(), &gtk.rsc)
                .await;
            debug!("group key {} installed", gtk.index);
        }
        if let Some(igtk) = keys.igtk {
            self.add_key(
                None,
                supplicant::CIPHER_SUITE_BIP_CMAC_128,
                igtk.index,
                &igtk.key,
                &igtk.ipn,
            )
            .await;
            debug!("management group key {} installed", igtk.index);
        }
        if self.conn == ConnState::Handshake {
            self.open_port().await;
        }
    }

    /// Adds a key of `cipher_suite`: the pairwise key of `peer`, or without one a group key (NCS
    /// `nrf_wifi_wpa_supp_set_key` and `nrf_wifi_sys_fmac_add_key`). `seq` is the packet number
    /// reception starts from, low byte first.
    pub(super) async fn add_key(
        &mut self,
        peer: Option<[u8; 6]>,
        cipher_suite: u32,
        index: u8,
        key: &[u8],
        seq: &[u8; 6],
    ) {
        let mut cmd: c::umac_cmd_key = unsafe { zeroed() };
        let info = &mut cmd.key_info;
        info.valid_fields = c::CIPHER_SUITE_VALID | c::KEY_VALID | c::SEQ_VALID | c::KEY_TYPE_VALID | c::KEY_IDX_VALID;
        info.cipher_suite = cipher_suite;
        info.key.key_len = key.len() as _;
        info.key.key[..key.len()].copy_from_slice(key);
        info.seq.seq_len = seq.len() as _;
        info.seq.seq[..seq.len()].copy_from_slice(seq);
        info.key_idx = index;
        match peer {
            Some(peer) => {
                info.key_type = c::key_type::KEYTYPE_PAIRWISE as _;
                cmd.mac_addr = peer;
                cmd.valid_fields = c::CMD_KEY_MAC_ADDR_VALID;
            }
            None => {
                info.key_type = c::key_type::KEYTYPE_GROUP as _;
                info.flags = c::KEY_DEFAULT_TYPE_MULTICAST as _;
            }
        }
        self.rpu.send_cmd(&mut cmd).await;
    }

    /// Makes key `index` the one `kind` frames are sent with (NCS `nrf_wifi_wpa_supp_set_key` with
    /// `set_tx`, and `nrf_wifi_sys_fmac_set_key`).
    pub(super) async fn set_default_key(&mut self, index: u8, kind: DefaultKey) {
        let mut cmd: c::umac_cmd_set_key = unsafe { zeroed() };
        cmd.key_info.valid_fields = c::KEY_IDX_VALID;
        cmd.key_info.key_idx = index;
        let flags = match kind {
            DefaultKey::Unicast => c::KEY_DEFAULT | c::KEY_DEFAULT_TYPE_UNICAST,
            DefaultKey::Multicast => c::KEY_DEFAULT | c::KEY_DEFAULT_TYPE_MULTICAST,
            DefaultKey::Management => c::KEY_DEFAULT_MGMT | c::KEY_DEFAULT_TYPE_MULTICAST,
        };
        cmd.key_info.flags = flags as _;
        self.rpu.send_cmd(&mut cmd).await;
    }
}
