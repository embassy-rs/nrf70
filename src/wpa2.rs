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

use crate::supplicant::{self, Gtk, Outcome, Rsne, Supplicant};
use crate::{
    c, find_ie, llc_ethertype, rx_to_ethernet, slice8_mut, write_ethernet, Bss, Bus, ConnState, ConnectError, Control,
    Credentials, Runner, MAX_TX_TOKENS,
};

/// How long the last message of a handshake gets to leave before its keys go in anyway.
const KEY_INSTALL_TIMEOUT: Duration = Duration::from_millis(200);

/// Longest EAPOL frame the supplicant is handed, as an Ethernet frame: the fixed part of an
/// EAPOL-Key frame, and key data with a few elements and keys.
const EAPOL_RX_MAX: usize = 14 + 99 + 400;

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
}

impl State {
    pub(crate) const fn new() -> Self {
        Self {
            mac_addr: [0; 6],
            supplicant: None,
            pending_keys: None,
        }
    }
}

/// The pre-shared key of the WPA2-Personal network `ssid`, from its passphrase: PBKDF2-HMAC-SHA1
/// with 4096 iterations, which is slow on purpose. `None` if the passphrase is not 8 to 63 bytes
/// long. The key depends on nothing else, so it can be stored in place of the passphrase.
pub fn wpa2_psk(ssid: &[u8], passphrase: &[u8]) -> Option<[u8; 32]> {
    supplicant::psk_from_passphrase(passphrase, ssid)
}

impl Control<'_> {
    /// Joins the WPA2-Personal network `ssid` with its passphrase, picking its strongest access
    /// point that offers CCMP and does not require management frame protection. Returns once the
    /// 4-way handshake is done, the keys are in and the link is up; see [`Control::join_open`].
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
    find_ie(ies, supplicant::IE_RSN)
        .or_else(|| find_ie(beacon_ies, supplicant::IE_RSN))
        .and_then(Rsne::from_body)
}

/// The ethertype of a received frame, as [`rx_to_ethernet`] would write it, without converting
/// the frame.
fn rx_ethertype(frame: &[u8], pkt_type: u32, mac_header_len: usize) -> Option<u16> {
    match pkt_type {
        c::PKT_TYPE_MPDU => llc_ethertype(frame.get(mac_header_len..)?),
        c::PKT_TYPE_MSDU_WITH_MAC => rx_ethertype(frame.get(mac_header_len..)?, c::PKT_TYPE_MSDU, 0),
        c::PKT_TYPE_MSDU => llc_ethertype(frame.get(14..)?),
        _ => None,
    }
}

/// The runner's side of WPA2. The first items are what `lib.rs` calls, each with an empty
/// counterpart there for a build without the feature.
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
            (Credentials::Wpa2(wpa2), Some(ap_rsne)) => Some(Supplicant::new(
                wpa2.psk,
                bss.bssid,
                self.wpa2.mac_addr,
                ap_rsne,
                wpa2.nonce_seed,
            )),
            _ => None,
        };
        if let Some(supplicant) = &self.wpa2.supplicant {
            let rsne = supplicant.rsne().as_bytes();
            info.valid_fields |= c::CONNECT_COMMON_INFO_WPA_IE_VALID;
            info.flags |= c::CONNECT_COMMON_INFO_SECURITY;
            info.wpa_ie.ie_len = rsne.len() as u16;
            for (to, from) in info.wpa_ie.ie.iter_mut().zip(rsne) {
                *to = *from as _;
            }
        }
    }

    /// Whether the association is followed by a 4-way handshake, which the AP starts.
    pub(super) fn handshake_expected(&self) -> bool {
        self.wpa2.supplicant.is_some()
    }

    /// Takes a received frame if it is an EAPOL one, and says whether it did: key handshake
    /// frames are the supplicant's, not embassy-net's.
    pub(super) async fn rx_eapol(&mut self, frame: &[u8], pkt_type: u32, mac_header_len: usize) -> bool {
        if rx_ethertype(frame, pkt_type, mac_header_len) != Some(supplicant::ETHERTYPE_EAPOL) {
            return false;
        }
        let mut eapol = [0; EAPOL_RX_MAX];
        match rx_to_ethernet(frame, pkt_type, mac_header_len, &mut eapol) {
            Some(n) => self.handle_eapol(&eapol[..n]).await,
            None => warn!("EAPOL frame of {} bytes dropped", frame.len()),
        }
        true
    }

    /// The RPU reported the frame of TX token `token` sent. If that was the last message of a
    /// handshake, its keys go in.
    pub(super) async fn frame_sent(&mut self, token: usize) {
        if self.wpa2.pending_keys.as_ref().is_some_and(|keys| keys.token == token) {
            self.install_pending_keys().await;
        }
    }

    pub(super) async fn check_pending_keys(&mut self) {
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

    /// The association is over: its keys and its supplicant go with it.
    pub(super) fn forget_keys(&mut self) {
        self.wpa2.supplicant = None;
        self.wpa2.pending_keys = None;
    }

    /// Hands an EAPOL frame from the AP, as an Ethernet frame, to the supplicant, and does what
    /// it says.
    async fn handle_eapol(&mut self, frame: &[u8]) {
        let (Some(bss), Some(supplicant)) = (self.conn_bss, self.wpa2.supplicant.as_mut()) else {
            debug!("EAPOL frame ignored: no WPA2 association");
            return;
        };
        if !matches!(self.conn, ConnState::Handshake | ConnState::Connected) || frame[6..12] != bss.bssid {
            debug!("EAPOL frame ignored: not from the AP of an association");
            return;
        }
        let mut reply = [0; supplicant::REPLY_MAX];
        match supplicant.handle(&frame[14..], &mut reply) {
            Outcome::Ignored => {}
            Outcome::Reply(len) => {
                self.send_eapol(&reply[..len]).await;
            }
            Outcome::Keys { reply: len, tk, gtk } => {
                // The keys of a handshake go in once its last message has left. If one is
                // still waiting, its keys go in now.
                self.install_pending_keys().await;
                let token = self.send_eapol(&reply[..len]).await;
                self.wpa2.pending_keys = Some(PendingKeys {
                    // Without a token the message was not sent, and the AP will repeat its own.
                    token: token.unwrap_or(MAX_TX_TOKENS),
                    tk,
                    gtk,
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
            self.add_key(Some(bss.bssid), 0, &tk, &[0; 6]).await;
            self.set_default_key(0).await;
            debug!("pairwise key installed");
        }
        if let Some(gtk) = keys.gtk {
            self.add_key(None, gtk.index, &gtk.key, &gtk.rsc).await;
            debug!("group key {} installed", gtk.index);
        }
        if self.conn == ConnState::Handshake {
            self.connected().await;
        }
    }

    /// Adds a CCMP key: the pairwise key of `peer`, or without one a group key (NCS
    /// `nrf_wifi_wpa_supp_set_key` and `nrf_wifi_sys_fmac_add_key`). `seq` is the packet number
    /// reception starts from, low byte first.
    async fn add_key(&mut self, peer: Option<[u8; 6]>, index: u8, key: &[u8; 16], seq: &[u8; 6]) {
        let mut cmd: c::umac_cmd_key = unsafe { zeroed() };
        let info = &mut cmd.key_info;
        info.valid_fields = c::CIPHER_SUITE_VALID | c::KEY_VALID | c::SEQ_VALID | c::KEY_TYPE_VALID | c::KEY_IDX_VALID;
        info.cipher_suite = supplicant::CIPHER_SUITE_CCMP;
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
        self.send_cmd(cmd).await;
    }

    /// Makes pairwise key `index` the one unicast frames are sent with (NCS
    /// `nrf_wifi_sys_fmac_set_key`).
    async fn set_default_key(&mut self, index: u8) {
        let mut cmd: c::umac_cmd_set_key = unsafe { zeroed() };
        cmd.key_info.valid_fields = c::KEY_IDX_VALID;
        cmd.key_info.key_idx = index;
        cmd.key_info.flags = (c::KEY_DEFAULT | c::KEY_DEFAULT_TYPE_UNICAST) as _;
        self.send_cmd(cmd).await;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::assert_eq;
    use std::vec;

    use super::*;

    #[test]
    fn ethertype_is_read_without_converting_the_frame() {
        let mut mpdu = vec![
            0x08, 0x02, // frame control: data, FromDS
            0x00, 0x00, // duration
            1, 1, 1, 1, 1, 1, // addr1: our MAC, the destination
            2, 2, 2, 2, 2, 2, // addr2: BSSID
            3, 3, 3, 3, 3, 3, // addr3: the source
            0x00, 0x00, // sequence control
            0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, // LLC/SNAP (RFC 1042)
            0x08, 0x00, // IPv4
            1, 3, 0, 95,
        ];
        assert_eq!(rx_ethertype(&mpdu, c::PKT_TYPE_MPDU, 24), Some(0x0800));
        mpdu[30..32].copy_from_slice(&supplicant::ETHERTYPE_EAPOL.to_be_bytes());
        assert_eq!(
            rx_ethertype(&mpdu, c::PKT_TYPE_MPDU, 24),
            Some(supplicant::ETHERTYPE_EAPOL)
        );

        let mut subframe = vec![1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 0, 12];
        subframe.extend_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E, 1, 3, 0, 95]);
        assert_eq!(
            rx_ethertype(&subframe, c::PKT_TYPE_MSDU, 0),
            Some(supplicant::ETHERTYPE_EAPOL)
        );
        let mut with_mac = vec![0; 24];
        with_mac.extend_from_slice(&subframe);
        assert_eq!(
            rx_ethertype(&with_mac, c::PKT_TYPE_MSDU_WITH_MAC, 24),
            Some(supplicant::ETHERTYPE_EAPOL)
        );

        // Cut before the ethertype, and a packet type the RPU does not use.
        assert_eq!(rx_ethertype(&mpdu[..31], c::PKT_TYPE_MPDU, 24), None);
        assert_eq!(rx_ethertype(&mpdu, 7, 24), None);
    }
}
