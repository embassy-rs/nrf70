//! The authenticator of the WPA2-Personal access point: the access point's side of the 4-way
//! handshake (IEEE 802.11-2020, 12.7.6) with each station that joins it. It shares the
//! supplicant's key derivation and frame format, and the tests run it against the supplicant.
//!
//! Behind the `ap` and `wpa2` features.

use defmt::debug;

use crate::supplicant::{
    prf, wrap_key_data, KeyData, KeyFrame, KeyHeader, Ptk, Rsne, Rsnxe, Suite, INFO_ACK, INFO_ENCRYPTED_KEY_DATA,
    INFO_ERROR, INFO_INSTALL, INFO_MIC, INFO_PAIRWISE, INFO_REQUEST, INFO_SECURE, INFO_VERSION_MASK, KDE_GTK, KDE_IGTK,
    KEY_FRAME_LEN, RSNE_MAX, RSNXE_MAX, TK_LEN,
};

/// How many times message 1, then message 3, goes out before the handshake fails (hostapd's
/// `wpa_pairwise_update_count`).
pub(crate) const TRIES: u8 = 4;

/// Length of a GTK KDE, and of an IGTK KDE.
const GTK_KDE_LEN: usize = 2 + 4 + 2 + TK_LEN;
const IGTK_KDE_LEN: usize = 2 + 4 + 2 + 6 + 16;
/// Key data of message 3 at most: the RSNE, the RSNXE, the GTK and IGTK KDEs, padding, and the 8 bytes of
/// the key wrap.
const KEY_DATA_MAX: usize = RSNE_MAX + RSNXE_MAX + GTK_KDE_LEN + IGTK_KDE_LEN + 8 + 8;
/// Longest frame the authenticator sends: message 3.
pub(crate) const MESSAGE_MAX: usize = KEY_FRAME_LEN + KEY_DATA_MAX;

/// A group key of the access point: a GTK, or an IGTK.
#[derive(Clone, Copy)]
pub(crate) struct GroupKey {
    /// Key ID: 1 or 2 for a GTK, 4 or 5 for an IGTK.
    pub(crate) index: u8,
    pub(crate) key: [u8; TK_LEN],
}

impl GroupKey {
    /// A key from the access point's seed: PRF-128(seed, `label`, AA || counter), after IEEE
    /// 802.11-2020, 12.7.1.4, with a counter in place of the GNonce.
    fn derive(seed: &[u8; 32], label: &[u8], aa: &[u8; 6], counter: u64, index: u8) -> Self {
        let mut data = [0; 14];
        data[0..6].copy_from_slice(aa);
        data[6..14].copy_from_slice(&counter.to_be_bytes());
        let mut key = [0; TK_LEN];
        prf(seed, label, &data, &mut key);
        Self { index, key }
    }
}

/// The group keys of the access point, which message 3 and the group key handshakes hand over:
/// the GTK, and the IGTK of the stations that use management frame protection.
#[derive(Clone, Copy)]
pub(crate) struct GroupKeys {
    pub(crate) gtk: GroupKey,
    pub(crate) igtk: GroupKey,
}

impl GroupKeys {
    /// The keys numbered `counter` from the access point's seed, under key IDs 1 and 4, then 2 and
    /// 5, by turns.
    pub(crate) fn derive(seed: &[u8; 32], aa: &[u8; 6], counter: u64) -> Self {
        let odd = (counter % 2) as u8;
        Self {
            gtk: GroupKey::derive(seed, b"Group key expansion", aa, counter, 1 + odd),
            igtk: GroupKey::derive(seed, b"IGTK key expansion", aa, counter, 4 + odd),
        }
    }
}

/// The GTK KDE of `gtk` (IEEE 802.11-2020, 12.7.2): its key ID with the Tx bit clear (the key is
/// for reception), a reserved byte, then the key.
fn gtk_kde(gtk: &GroupKey) -> [u8; GTK_KDE_LEN] {
    let mut kde = [0; GTK_KDE_LEN];
    kde[..2].copy_from_slice(&[0xDD, GTK_KDE_LEN as u8 - 2]);
    kde[2..6].copy_from_slice(&KDE_GTK);
    kde[6..8].copy_from_slice(&[gtk.index, 0]);
    kde[8..].copy_from_slice(&gtk.key);
    kde
}

/// The IGTK KDE of `igtk`: its key ID, the packet number reception starts from (zero: the RPU
/// starts counting with a new key), then the key.
fn igtk_kde(igtk: &GroupKey) -> [u8; IGTK_KDE_LEN] {
    let mut kde = [0; IGTK_KDE_LEN];
    kde[..2].copy_from_slice(&[0xDD, IGTK_KDE_LEN as u8 - 2]);
    kde[2..6].copy_from_slice(&KDE_IGTK);
    kde[6..8].copy_from_slice(&(igtk.index as u16).to_le_bytes());
    kde[14..].copy_from_slice(&igtk.key);
    kde
}

/// An ANonce from the access point's seed: PRF-256(seed, "Init Counter", AA || SPA || counter),
/// after IEEE 802.11-2020, 12.7.5.
pub(crate) fn anonce(seed: &[u8; 32], aa: &[u8; 6], spa: &[u8; 6], counter: u64) -> [u8; 32] {
    let mut data = [0; 20];
    data[0..6].copy_from_slice(aa);
    data[6..12].copy_from_slice(spa);
    data[12..20].copy_from_slice(&counter.to_be_bytes());
    let mut nonce = [0; 32];
    prf(seed, b"Init Counter", &data, &mut nonce);
    nonce
}

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Stage {
    /// Message 1 went out: waiting for message 2.
    Message1,
    /// Message 3 went out: waiting for message 4.
    Message3,
    /// Message 1 of a group key handshake went out: waiting for its message 2.
    GroupMessage1,
    /// The station has the keys.
    Done,
}

/// What to do with a frame from the station.
pub(crate) enum Outcome {
    /// Nothing: not the message expected now, a replay, or a wrong MIC (most often a wrong
    /// passphrase on the station's side, which shows as nothing more).
    Ignored,
    /// Send the first `usize` bytes of the output buffer: message 3.
    Send(usize),
    /// The handshake is done: install the pairwise temporal key and open the station's port.
    Done { tk: [u8; TK_LEN] },
    /// The group key handshake is done: the station has the new group key.
    GroupDone,
    /// Message 2 does not repeat the RSNE or the RSNXE of the association request: deauthenticate the
    /// station (reason 17).
    Refused,
}

/// The handshake with one station.
pub(crate) struct Authenticator {
    pmk: [u8; 32],
    /// The authenticator's address: the BSSID.
    aa: [u8; 6],
    /// The station's.
    spa: [u8; 6],
    /// The access point's RSNE, which message 3 carries.
    ap_rsne: Rsne,
    /// The station's RSNE, from its association request, which message 2 must repeat.
    sta_rsne: Rsne,
    /// What the station chose.
    suite: Suite,
    /// The access point's RSNXE, if it announces one, which message 3 carries after the RSNE.
    ap_rsnxe: Option<Rsnxe>,
    /// The station's RSNXE, if its association request had one, which message 2 must repeat.
    sta_rsnxe: Option<Rsnxe>,
    anonce: [u8; 32],
    /// The replay counter of the last message sent.
    replay_counter: u64,
    ptk: Option<Ptk>,
    stage: Stage,
    /// The group keys a group key handshake hands over.
    group: Option<GroupKeys>,
    /// Messages sent of the current stage.
    sent: u8,
}

impl Authenticator {
    /// The handshake with the station `spa`, whose association request carried `sta_rsne` and
    /// chose `suite` of what `ap_rsne` offers.
    pub(crate) fn new(
        pmk: [u8; 32],
        aa: [u8; 6],
        spa: [u8; 6],
        ap_rsne: Rsne,
        sta_rsne: Rsne,
        suite: Suite,
        anonce: [u8; 32],
    ) -> Self {
        Self {
            pmk,
            aa,
            spa,
            ap_rsne,
            sta_rsne,
            suite,
            ap_rsnxe: None,
            sta_rsnxe: None,
            anonce,
            replay_counter: 0,
            ptk: None,
            stage: Stage::Message1,
            group: None,
            sent: 0,
        }
    }

    /// The RSNXEs of the association: the access point's, which message 3 carries, and the
    /// station's, which message 2 must repeat.
    pub(crate) fn with_rsnxe(mut self, ap_rsnxe: Option<Rsnxe>, sta_rsnxe: Option<Rsnxe>) -> Self {
        self.ap_rsnxe = ap_rsnxe;
        self.sta_rsnxe = sta_rsnxe;
        self
    }

    /// Starts a group key handshake to hand over `keys`, once the 4-way handshake is done. The
    /// messages come from [`Self::next_message`]. Returns whether it started.
    pub(crate) fn start_group(&mut self, keys: GroupKeys) -> bool {
        if self.stage != Stage::Done {
            return false;
        }
        self.stage = Stage::GroupMessage1;
        self.group = Some(keys);
        self.sent = 0;
        true
    }

    /// Whether the handshake under way is a group key one.
    pub(crate) fn in_group_handshake(&self) -> bool {
        self.stage == Stage::GroupMessage1
    }

    #[cfg(test)]
    pub(crate) fn done(&self) -> bool {
        self.stage == Stage::Done
    }

    /// Writes the group key KDEs into `out` and returns their length: the GTK, and the IGTK if the
    /// station uses management frame protection.
    fn write_group_keys(&self, keys: &GroupKeys, out: &mut [u8]) -> usize {
        out[..GTK_KDE_LEN].copy_from_slice(&gtk_kde(&keys.gtk));
        if !self.suite.mfp {
            return GTK_KDE_LEN;
        }
        out[GTK_KDE_LEN..GTK_KDE_LEN + IGTK_KDE_LEN].copy_from_slice(&igtk_kde(&keys.igtk));
        GTK_KDE_LEN + IGTK_KDE_LEN
    }

    /// Writes the message to send now into `out` and returns its length: message 1, or message 3
    /// once message 2 is in, the first time or again (with the next replay counter). `None` once
    /// the handshake is done, or after [`TRIES`] sends of the same message.
    pub(crate) fn next_message(&mut self, keys: &GroupKeys, out: &mut [u8; MESSAGE_MAX]) -> Option<usize> {
        if self.stage == Stage::Done || self.sent >= TRIES {
            return None;
        }
        self.sent += 1;
        self.replay_counter += 1;
        let version = self.suite.akm.key_version();
        match (self.stage, &self.ptk) {
            (Stage::Message1, _) => {
                let header = KeyHeader {
                    info: version | INFO_PAIRWISE | INFO_ACK,
                    key_len: TK_LEN as u16,
                    replay_counter: self.replay_counter,
                    nonce: self.anonce,
                    rsc: [0; 6],
                };
                Some(header.write(out, &[], None))
            }
            (Stage::Message3, Some(ptk)) => {
                // The access point's RSNE and RSNXE, then the group keys.
                let mut data = [0; KEY_DATA_MAX];
                let rsne = self.ap_rsne.as_bytes();
                let rsnxe = self.ap_rsnxe.as_ref().map_or(&[][..], Rsnxe::as_bytes);
                data[..rsne.len()].copy_from_slice(rsne);
                data[rsne.len()..rsne.len() + rsnxe.len()].copy_from_slice(rsnxe);
                let at = rsne.len() + rsnxe.len();
                let len = at + self.write_group_keys(keys, &mut data[at..]);
                let mut wrapped = [0; KEY_DATA_MAX];
                let wrapped_len = wrap_key_data(&ptk.kek, &data[..len], &mut wrapped)?;
                let header = KeyHeader {
                    info: version
                        | INFO_PAIRWISE
                        | INFO_INSTALL
                        | INFO_ACK
                        | INFO_MIC
                        | INFO_SECURE
                        | INFO_ENCRYPTED_KEY_DATA,
                    key_len: TK_LEN as u16,
                    replay_counter: self.replay_counter,
                    nonce: self.anonce,
                    // The group key's packet number: the RPU starts it from zero.
                    rsc: [0; 6],
                };
                Some(header.write(out, &wrapped[..wrapped_len], Some(&ptk.kck)))
            }
            (Stage::GroupMessage1, Some(ptk)) => {
                let mut data = [0; KEY_DATA_MAX];
                let len = self.write_group_keys(self.group.as_ref()?, &mut data);
                let mut wrapped = [0; KEY_DATA_MAX];
                let wrapped_len = wrap_key_data(&ptk.kek, &data[..len], &mut wrapped)?;
                // Key length, nonce and RSC zero, as hostapd sends it with RSN.
                let header = KeyHeader {
                    info: version | INFO_ACK | INFO_MIC | INFO_SECURE | INFO_ENCRYPTED_KEY_DATA,
                    key_len: 0,
                    replay_counter: self.replay_counter,
                    nonce: [0; 32],
                    rsc: [0; 6],
                };
                Some(header.write(out, &wrapped[..wrapped_len], Some(&ptk.kck)))
            }
            _ => None,
        }
    }

    /// Handles an EAPOL frame from the station, from its 802.1X header on. A frame to send, if
    /// any, is written at the start of `out`.
    pub(crate) fn handle(&mut self, frame: &[u8], keys: &GroupKeys, out: &mut [u8; MESSAGE_MAX]) -> Outcome {
        let Some(key) = KeyFrame::parse(frame) else {
            return Outcome::Ignored;
        };
        // Every answer has a MIC and the replay counter of the last message sent.
        if key.info & INFO_VERSION_MASK != self.suite.akm.key_version()
            || key.info & INFO_MIC == 0
            || key.info & (INFO_ACK | INFO_REQUEST | INFO_ERROR) != 0
            || key.replay_counter != self.replay_counter
        {
            debug!(
                "EAPOL-Key frame from the station ignored: key information {:04x}, replay counter {}",
                key.info, key.replay_counter
            );
            return Outcome::Ignored;
        }
        let pairwise = key.info & INFO_PAIRWISE != 0;
        let secure = key.info & INFO_SECURE != 0;
        match self.stage {
            Stage::Message1 if pairwise && !secure => {
                let ptk = Ptk::derive(self.suite.akm, &self.pmk, &self.aa, &self.spa, &self.anonce, &key.nonce);
                if !key.mic_is_valid(&ptk.kck) {
                    debug!("4-way handshake: message 2 ignored, invalid MIC");
                    return Outcome::Ignored;
                }
                let data = KeyData::parse(key.key_data);
                if data.rsne != Some(self.sta_rsne.as_bytes()) {
                    debug!("4-way handshake: the RSNE of message 2 is not the association request's");
                    return Outcome::Refused;
                }
                if data.rsnxe != self.sta_rsnxe.as_ref().map(Rsnxe::as_bytes) {
                    debug!("4-way handshake: the RSNXE of message 2 is not the association request's");
                    return Outcome::Refused;
                }
                self.ptk = Some(ptk);
                self.stage = Stage::Message3;
                self.sent = 0;
                match self.next_message(keys, out) {
                    Some(len) => Outcome::Send(len),
                    None => Outcome::Ignored,
                }
            }
            Stage::Message3 if pairwise && secure => {
                let Some(ptk) = &self.ptk else {
                    return Outcome::Ignored;
                };
                if !key.mic_is_valid(&ptk.kck) {
                    debug!("4-way handshake: message 4 ignored, invalid MIC");
                    return Outcome::Ignored;
                }
                self.stage = Stage::Done;
                Outcome::Done { tk: ptk.tk }
            }
            Stage::GroupMessage1 if !pairwise && secure => {
                let Some(ptk) = &self.ptk else {
                    return Outcome::Ignored;
                };
                if !key.mic_is_valid(&ptk.kck) {
                    debug!("group key handshake: message 2 ignored, invalid MIC");
                    return Outcome::Ignored;
                }
                self.stage = Stage::Done;
                self.group = None;
                Outcome::GroupDone
            }
            _ => Outcome::Ignored,
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::assert_eq;

    use super::*;
    use crate::supplicant::{psk_from_passphrase, Outcome as StationOutcome, Supplicant, REPLY_MAX};
    use crate::supplicant::{Offer, Rsnxe};

    const WPA2: Offer = Offer { psk: true, sae: false };
    const WPA3: Offer = Offer { psk: false, sae: true };
    const TRANSITION: Offer = Offer { psk: true, sae: true };

    const AA: [u8; 6] = [0xF4, 0xCE, 0x36, 0x00, 0x8B, 0x19];
    const SPA: [u8; 6] = [0x98, 0x43, 0xFA, 0x23, 0x26, 0x25];

    fn ap_rsne() -> Rsne {
        Rsne::access_point(WPA2)
    }

    fn pair(passphrase: &[u8]) -> (Authenticator, Supplicant, GroupKeys) {
        let pmk = psk_from_passphrase(b"correct horse", b"nrf70-ap").unwrap();
        let station_pmk = psk_from_passphrase(passphrase, b"nrf70-ap").unwrap();
        let station = Supplicant::new(station_pmk, AA, SPA, ap_rsne(), [7; 32], false).unwrap();
        let sta_rsne = *station.rsne();
        let anonce = anonce(&[3; 32], &AA, &SPA, 0);
        let suite = sta_rsne.check_station(WPA2).unwrap();
        let authenticator = Authenticator::new(pmk, AA, SPA, ap_rsne(), sta_rsne, suite, anonce);
        (authenticator, station, GroupKeys::derive(&[3; 32], &AA, 0))
    }

    #[test]
    fn a_station_gets_the_keys_through_the_handshake() {
        let (mut ap, mut station, gtk) = pair(b"correct horse");
        let mut out = [0; MESSAGE_MAX];
        let mut reply = [0; REPLY_MAX];

        let len = ap.next_message(&gtk, &mut out).unwrap();
        let StationOutcome::Reply(reply_len) = station.handle(&out[..len], &mut reply) else {
            panic!("no message 2");
        };
        let Outcome::Send(len) = ap.handle(&reply[..reply_len], &gtk, &mut out) else {
            panic!("no message 3");
        };
        let StationOutcome::Keys {
            reply: reply_len,
            tk: Some(station_tk),
            gtk: Some(station_gtk),
            igtk: Some(station_igtk),
        } = station.handle(&out[..len], &mut reply)
        else {
            panic!("no keys from message 3");
        };
        // The station chose PSK-SHA256 with management frame protection: it got the IGTK too.
        assert_eq!(ap.suite.akm, crate::supplicant::Akm::PskSha256);
        assert_eq!(station_gtk.index, 1);
        assert_eq!(station_gtk.key(), &gtk.gtk.key);
        assert_eq!(
            (station_igtk.index, station_igtk.key, station_igtk.ipn),
            (4, gtk.igtk.key, [0; 6])
        );
        let Outcome::Done { tk } = ap.handle(&reply[..reply_len], &gtk, &mut out) else {
            panic!("message 4 not taken");
        };
        assert_eq!(tk, station_tk);
        assert!(ap.done());
        assert!(ap.next_message(&gtk, &mut out).is_none());
        // A replayed message 4 changes nothing.
        assert!(matches!(
            ap.handle(&reply[..reply_len], &gtk, &mut out),
            Outcome::Ignored
        ));
    }

    /// Runs the 4-way handshake to its end.
    fn handshake(ap: &mut Authenticator, station: &mut Supplicant, gtk: &GroupKeys) {
        let mut out = [0; MESSAGE_MAX];
        let mut reply = [0; REPLY_MAX];
        let len = ap.next_message(gtk, &mut out).unwrap();
        let StationOutcome::Reply(reply_len) = station.handle(&out[..len], &mut reply) else {
            panic!("no message 2");
        };
        let Outcome::Send(len) = ap.handle(&reply[..reply_len], gtk, &mut out) else {
            panic!("no message 3");
        };
        let StationOutcome::Keys { reply: reply_len, .. } = station.handle(&out[..len], &mut reply) else {
            panic!("no message 4");
        };
        assert!(matches!(
            ap.handle(&reply[..reply_len], gtk, &mut out),
            Outcome::Done { .. }
        ));
    }

    #[test]
    fn the_group_key_is_renewed_with_a_group_key_handshake() {
        let (mut ap, mut station, gtk) = pair(b"correct horse");
        let mut out = [0; MESSAGE_MAX];
        let mut reply = [0; REPLY_MAX];
        let next = GroupKeys::derive(&[3; 32], &AA, 1);
        // Not before the 4-way handshake is done.
        assert!(!ap.start_group(next));
        handshake(&mut ap, &mut station, &gtk);

        assert!(ap.start_group(next));
        assert!(ap.in_group_handshake());
        let len = ap.next_message(&gtk, &mut out).unwrap();
        let StationOutcome::Keys {
            reply: reply_len,
            tk: None,
            gtk: Some(station_gtk),
            igtk: Some(station_igtk),
        } = station.handle(&out[..len], &mut reply)
        else {
            panic!("no new group keys from group message 1");
        };
        assert_eq!(station_gtk.index, 2);
        assert_eq!(station_gtk.key(), &next.gtk.key);
        assert_eq!((station_igtk.index, station_igtk.key), (5, next.igtk.key));
        assert!(matches!(
            ap.handle(&reply[..reply_len], &gtk, &mut out),
            Outcome::GroupDone
        ));
        assert!(!ap.in_group_handshake());
        assert!(ap.next_message(&gtk, &mut out).is_none());

        // A station that does not answer gets the message four times, then the handshake fails.
        assert!(ap.start_group(GroupKeys::derive(&[3; 32], &AA, 2)));
        for _ in 0..TRIES {
            assert!(ap.next_message(&gtk, &mut out).is_some());
        }
        assert!(ap.next_message(&gtk, &mut out).is_none());
        assert!(ap.in_group_handshake());
    }

    #[test]
    fn a_wpa3_station_gets_the_keys_with_the_sae_pmk_and_the_rsnxe() {
        // The PMK of an SAE exchange, here any.
        let pmk = [0x5A; 32];
        let ap_rsne = Rsne::access_point(WPA3);
        let rsnxe = Some(Rsnxe::sae_h2e());
        let mut station = Supplicant::new(pmk, AA, SPA, ap_rsne, [7; 32], true)
            .unwrap()
            .with_rsnxe(rsnxe, rsnxe);
        let sta_rsne = *station.rsne();
        let suite = sta_rsne.check_station(WPA3).unwrap();
        assert_eq!((suite.akm, suite.mfp), (crate::supplicant::Akm::Sae, true));
        let mut ap = Authenticator::new(pmk, AA, SPA, ap_rsne, sta_rsne, suite, anonce(&[3; 32], &AA, &SPA, 0))
            .with_rsnxe(rsnxe, rsnxe);
        let keys = GroupKeys::derive(&[3; 32], &AA, 0);
        handshake(&mut ap, &mut station, &keys);
        assert!(ap.done());

        // A station whose message 2 leaves out the RSNXE of its association request is refused.
        let mut station = Supplicant::new(pmk, AA, SPA, ap_rsne, [7; 32], true)
            .unwrap()
            .with_rsnxe(None, rsnxe);
        let mut ap = Authenticator::new(pmk, AA, SPA, ap_rsne, sta_rsne, suite, anonce(&[3; 32], &AA, &SPA, 1))
            .with_rsnxe(rsnxe, rsnxe);
        let mut out = [0; MESSAGE_MAX];
        let mut reply = [0; REPLY_MAX];
        let len = ap.next_message(&keys, &mut out).unwrap();
        let StationOutcome::Reply(reply_len) = station.handle(&out[..len], &mut reply) else {
            panic!("no message 2");
        };
        assert!(matches!(
            ap.handle(&reply[..reply_len], &keys, &mut out),
            Outcome::Refused
        ));
    }

    #[test]
    fn a_wrong_passphrase_gets_no_message_3_and_the_handshake_gives_up() {
        let (mut ap, mut station, gtk) = pair(b"wrong horse");
        let mut out = [0; MESSAGE_MAX];
        let mut reply = [0; REPLY_MAX];
        for _ in 0..TRIES {
            let len = ap.next_message(&gtk, &mut out).unwrap();
            let StationOutcome::Reply(reply_len) = station.handle(&out[..len], &mut reply) else {
                panic!("no message 2");
            };
            assert!(matches!(
                ap.handle(&reply[..reply_len], &gtk, &mut out),
                Outcome::Ignored
            ));
        }
        assert!(ap.next_message(&gtk, &mut out).is_none());
        assert!(!ap.done());
    }

    #[test]
    fn message_1_is_sent_again_with_the_next_replay_counter() {
        let (mut ap, mut station, gtk) = pair(b"correct horse");
        let mut out = [0; MESSAGE_MAX];
        let mut reply = [0; REPLY_MAX];
        let len = ap.next_message(&gtk, &mut out).unwrap();
        let first = out[..len].to_vec();
        let len = ap.next_message(&gtk, &mut out).unwrap();
        let second = out[..len].to_vec();
        // Same ANonce, replay counter 2.
        assert_eq!(second[17..49], first[17..49]);
        assert_eq!(second[9..17], 2u64.to_be_bytes());
        // An answer to the first one is a replay now; one to the second goes on.
        let StationOutcome::Reply(len) = station.handle(&first, &mut reply) else {
            panic!("no message 2");
        };
        assert!(matches!(ap.handle(&reply[..len], &gtk, &mut out), Outcome::Ignored));
        let StationOutcome::Reply(len) = station.handle(&second, &mut reply) else {
            panic!("no message 2");
        };
        assert!(matches!(ap.handle(&reply[..len], &gtk, &mut out), Outcome::Send(_)));
    }

    #[test]
    fn message_2_must_repeat_the_association_rsne() {
        let (mut ap, mut station, gtk) = pair(b"correct horse");
        // The association request offered TKIP as pairwise cipher too: message 2 does not.
        let mut body = ap_rsne().as_bytes()[2..].to_vec();
        body.splice(6..12, [2, 0, 0x00, 0x0F, 0xAC, 2, 0x00, 0x0F, 0xAC, 4]);
        ap.sta_rsne = Rsne::from_body(&body).unwrap();
        let mut out = [0; MESSAGE_MAX];
        let mut reply = [0; REPLY_MAX];
        let len = ap.next_message(&gtk, &mut out).unwrap();
        let StationOutcome::Reply(reply_len) = station.handle(&out[..len], &mut reply) else {
            panic!("no message 2");
        };
        assert!(matches!(
            ap.handle(&reply[..reply_len], &gtk, &mut out),
            Outcome::Refused
        ));
    }

    #[test]
    fn stations_offering_what_the_ap_does_are_taken() {
        let rsne = |hex: &[u8]| Rsne::from_body(hex).unwrap();
        let ccmp = [0x00, 0x0F, 0xAC, 4];
        let tkip = [0x00, 0x0F, 0xAC, 2];
        let psk = [0x00, 0x0F, 0xAC, 2];
        let sae = [0x00, 0x0F, 0xAC, 8];
        let body = |group: [u8; 4], pairwise: [u8; 4], akm: [u8; 4], caps: u16| {
            let mut b = std::vec![1, 0];
            b.extend_from_slice(&group);
            b.extend_from_slice(&[1, 0]);
            b.extend_from_slice(&pairwise);
            b.extend_from_slice(&[1, 0]);
            b.extend_from_slice(&akm);
            b.extend_from_slice(&caps.to_le_bytes());
            b
        };
        let suite = |akm, mfp| {
            Ok(Suite {
                akm,
                mfp,
                group: crate::supplicant::GroupCipher::Ccmp,
            })
        };
        use crate::supplicant::Akm::{Psk, PskSha256};
        let psk_sha256 = [0x00, 0x0F, 0xAC, 6];
        assert_eq!(rsne(&body(ccmp, ccmp, psk, 0)).check_station(WPA2), suite(Psk, false));
        // Management frame protection when the station is capable of it, or requires it.
        assert_eq!(
            rsne(&body(ccmp, ccmp, psk, 0x0080)).check_station(WPA2),
            suite(Psk, true)
        );
        assert_eq!(
            rsne(&body(ccmp, ccmp, psk, 0x00C0)).check_station(WPA2),
            suite(Psk, true)
        );
        assert_eq!(
            rsne(&body(ccmp, ccmp, psk_sha256, 0x0080)).check_station(WPA2),
            suite(PskSha256, true)
        );
        assert_eq!(
            rsne(&body(ccmp, ccmp, psk_sha256, 0)).check_station(WPA2),
            suite(PskSha256, false)
        );
        // Protection with BIP-GMAC-128 (no PMKIDs, then the group management cipher).
        let mut gmac = body(ccmp, ccmp, psk, 0x0080);
        gmac.extend_from_slice(&[0, 0, 0x00, 0x0F, 0xAC, 11]);
        assert_eq!(rsne(&gmac).check_station(WPA2), Err(46));
        assert_eq!(rsne(&body(tkip, ccmp, psk, 0)).check_station(WPA2), Err(41));
        assert_eq!(rsne(&body(ccmp, tkip, psk, 0)).check_station(WPA2), Err(42));
        assert_eq!(rsne(&body(ccmp, ccmp, sae, 0)).check_station(WPA2), Err(43));
        assert_eq!(rsne(&[2, 0]).check_station(WPA2), Err(40));

        // WPA3: SAE only, and protection required.
        use crate::supplicant::Akm::Sae;
        assert_eq!(
            rsne(&body(ccmp, ccmp, sae, 0x0080)).check_station(WPA3),
            suite(Sae, true)
        );
        assert_eq!(rsne(&body(ccmp, ccmp, sae, 0)).check_station(WPA3), Err(31));
        assert_eq!(rsne(&body(ccmp, ccmp, psk, 0x0080)).check_station(WPA3), Err(43));
        // Transition: both, protection for SAE only.
        assert_eq!(
            rsne(&body(ccmp, ccmp, psk, 0)).check_station(TRANSITION),
            suite(Psk, false)
        );
        assert_eq!(
            rsne(&body(ccmp, ccmp, sae, 0x0080)).check_station(TRANSITION),
            suite(Sae, true)
        );
        assert_eq!(rsne(&body(ccmp, ccmp, sae, 0)).check_station(TRANSITION), Err(31));
    }

    #[test]
    fn the_access_point_rsne_follows_the_offer() {
        let bytes = |rsne: Rsne| rsne.as_bytes().to_vec();
        let hex = |s: &str| {
            let s: std::string::String = s.split_whitespace().collect();
            (0..s.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
                .collect::<std::vec::Vec<u8>>()
        };
        // CCMP; PSK and PSK-SHA256; MFPC.
        assert_eq!(
            bytes(Rsne::access_point(WPA2)),
            hex("30 18 0100 000fac04 0100 000fac04 0200 000fac02 000fac06 8000")
        );
        // SAE; MFPC and MFPR.
        assert_eq!(
            bytes(Rsne::access_point(WPA3)),
            hex("30 14 0100 000fac04 0100 000fac04 0100 000fac08 c000")
        );
        // All three; MFPC.
        assert_eq!(
            bytes(Rsne::access_point(TRANSITION)),
            hex("30 1c 0100 000fac04 0100 000fac04 0300 000fac02 000fac06 000fac08 8000")
        );
    }
}
