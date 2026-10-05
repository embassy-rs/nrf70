//! Entry points for fuzzing what the driver reads from the air: received frames, elements, scan
//! results, management frames to the access point, the key handshakes and SAE. Each takes the
//! fuzzer's bytes, and panics on what must not happen.
//!
//! This module exists for `cargo fuzz` (which builds with `--cfg fuzzing`; the targets are in
//! `fuzz/`) and for the tests, which run every entry point on random inputs so that a regression
//! shows in `cargo test` too. It is not part of the API.

extern crate std;

#[cfg(feature = "ap")]
pub use crate::ap::fuzz::management;
pub use crate::data::fuzz::rx_frame;
#[cfg(all(feature = "ap", feature = "wpa2"))]
pub use crate::eapol::fuzz::wrap;
#[cfg(feature = "wpa2")]
pub use crate::eapol::fuzz::{key_data, key_frame};
use crate::ieee80211;
#[cfg(feature = "wpa2")]
pub use crate::rsn::fuzz::rsn;
pub use crate::station::fuzz::scan_result;

/// The elements of a frame body.
pub fn elements(data: &[u8]) {
    let mut len = 0;
    for (id, body) in ieee80211::elements(data) {
        len += 2 + body.len();
        assert!(ieee80211::find_ie(data, id).is_some());
    }
    assert!(len <= data.len());
    for extension in [
        ieee80211::IE_EXT_PASSWORD_IDENTIFIER,
        ieee80211::IE_EXT_ANTI_CLOGGING_TOKEN,
    ] {
        let _ = ieee80211::find_extension(data, extension);
    }
}

/// The fuzzer's bytes, taken a few at a time: zeros once they run out.
#[cfg_attr(not(all(feature = "ap", feature = "wpa3")), allow(dead_code))]
struct Input<'a>(&'a [u8]);

#[cfg_attr(not(all(feature = "ap", feature = "wpa3")), allow(dead_code))]
impl<'a> Input<'a> {
    fn byte(&mut self) -> u8 {
        let Some((first, rest)) = self.0.split_first() else {
            return 0;
        };
        self.0 = rest;
        *first
    }

    fn u16(&mut self) -> u16 {
        u16::from_le_bytes([self.byte(), self.byte()])
    }

    fn bytes(&mut self, n: usize) -> &'a [u8] {
        let (taken, rest) = self.0.split_at(n.min(self.0.len()));
        self.0 = rest;
        taken
    }

    fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Changes `bytes` as the input says: up to four changes, each a flipped byte, a truncation,
    /// an inserted byte or overwritten ones.
    fn mutate(&mut self, bytes: &mut std::vec::Vec<u8>) {
        for _ in 0..1 + self.byte() % 4 {
            let at = usize::from(self.u16()) % (bytes.len() + 1);
            match self.byte() % 4 {
                0 if at < bytes.len() => bytes[at] ^= self.byte() | 1,
                1 => bytes.truncate(at),
                2 => bytes.insert(at, self.byte()),
                _ => {
                    let len = usize::from(self.byte() % 16);
                    let run = self.bytes(len);
                    let end = (at + run.len()).min(bytes.len());
                    bytes[at..end].copy_from_slice(&run[..end - at]);
                }
            }
        }
    }
}

/// The 4-way handshake and group key handshakes between the supplicant and the authenticator,
/// with what passes between them in the fuzzer's hands. The first byte picks what the access
/// point offers and what the station uses; then each step delivers the access point's next
/// message, one of its messages again, or the station's last reply, renews the group keys, or
/// hands either side bytes of the input. A frame on its way may be changed, then signed again
/// with the key confirmation key, and the key data of the access point's frames changed under
/// their encryption.
///
/// It panics if a side hands a key over twice (the reinstallation that KRACK forces), if the
/// access point refuses the station's association request, or if, with nothing changed, the two
/// sides end with different pairwise keys.
#[cfg(all(feature = "ap", feature = "wpa2"))]
pub fn handshake(data: &[u8]) {
    let _ = handshake_keys(data);
}

/// [`handshake`], which returns the pairwise keys the access point and the station ended with,
/// unless a frame was changed on its way.
#[cfg(all(feature = "ap", feature = "wpa2"))]
fn handshake_keys(data: &[u8]) -> Option<([u8; 16], [u8; 16])> {
    use std::vec::Vec;

    use crate::authenticator::{self, Authenticator, GroupKeys, MESSAGE_MAX};
    use crate::crypto::{Ptk, TK_LEN};
    use crate::eapol::{
        mic, unwrap_key_data, wrap_key_data, Gtk, Igtk, KeyFrame, KeyHeader, INFO_ENCRYPTED_KEY_DATA, KEY_DATA_MAX,
        KEY_FRAME_LEN, MIC_LEN, OFFSET_KEY_INFO, OFFSET_MIC, OFFSET_NONCE, REPLY_MAX,
    };
    use crate::rsn::{Akm, Offer, Rsne};
    use crate::supplicant::{self, Supplicant};

    const AA: [u8; 6] = [0x02, 0xAA, 0, 0, 0, 1];
    const SPA: [u8; 6] = [0x02, 0x55, 0, 0, 0, 2];
    const PMK: [u8; 32] = [0x5A; 32];
    const ANONCE: [u8; 32] = [0xA5; 32];

    /// The keys of the handshake with `anonce` and `snonce`.
    fn ptk(akm: Akm, anonce: &[u8; 32], snonce: &[u8; 32]) -> Ptk {
        Ptk::derive(akm, &PMK, &AA, &SPA, anonce, snonce)
    }

    /// The nonce field of an EAPOL-Key frame, if it has one that is not zero.
    fn nonce(frame: &[u8]) -> Option<[u8; 32]> {
        let nonce: [u8; 32] = frame.get(OFFSET_NONCE..OFFSET_NONCE + 32)?.try_into().ok()?;
        (nonce != [0; 32]).then_some(nonce)
    }

    /// Writes into `frame` the MIC that `kck` gives over its EAPOL PDU, as the version in its key
    /// information asks.
    fn sign(frame: &mut [u8], kck: &[u8; 16]) {
        if frame.len() < KEY_FRAME_LEN {
            return;
        }
        let pdu = (4 + usize::from(u16::from_be_bytes([frame[2], frame[3]]))).min(frame.len());
        if pdu < OFFSET_MIC + MIC_LEN {
            return;
        }
        frame[OFFSET_MIC..OFFSET_MIC + MIC_LEN].fill(0);
        let info = u16::from_be_bytes([frame[OFFSET_KEY_INFO], frame[OFFSET_KEY_INFO + 1]]);
        let mic = mic(info, kck, &[&frame[..pdu]]);
        frame[OFFSET_MIC..OFFSET_MIC + MIC_LEN].copy_from_slice(&mic);
    }

    /// Changes the plaintext of the key data of `frame`, then wraps and signs it again.
    fn change_key_data(frame: &mut Vec<u8>, ptk: &Ptk, input: &mut Input) {
        let Some(key) = KeyFrame::parse(frame) else {
            return;
        };
        let mut buf = [0; KEY_DATA_MAX];
        let Some(plain) = unwrap_key_data(&key, &ptk.kek, &mut buf) else {
            return;
        };
        let mut plain = plain.to_vec();
        input.mutate(&mut plain);
        let mut wrapped = [0; KEY_DATA_MAX + 8];
        let Some(len) = wrap_key_data(&ptk.kek, &plain, &mut wrapped) else {
            return;
        };
        let header = KeyHeader {
            info: key.info | INFO_ENCRYPTED_KEY_DATA,
            key_len: u16::from_be_bytes([frame[OFFSET_KEY_INFO + 2], frame[OFFSET_KEY_INFO + 3]]),
            replay_counter: key.replay_counter,
            nonce: key.nonce,
            rsc: key.rsc,
        };
        let mut out = std::vec![0; KEY_FRAME_LEN + len];
        header.write(&mut out, &wrapped[..len], Some(&ptk.kck));
        *frame = out;
    }

    /// What the station has handed over, to check that it never does so twice.
    #[derive(Default)]
    struct Station {
        tks: Vec<[u8; TK_LEN]>,
        gtk: Option<Gtk>,
        igtk: Option<Igtk>,
        snonce: Option<[u8; 32]>,
        reply: Vec<u8>,
    }

    impl Station {
        fn receive(&mut self, supplicant: &mut Supplicant, frame: &[u8]) {
            let mut reply = [0; REPLY_MAX];
            match supplicant.handle(frame, &mut reply) {
                supplicant::Outcome::Ignored | supplicant::Outcome::Abort => {}
                supplicant::Outcome::Reply(len) => {
                    self.reply = reply[..len].to_vec();
                    self.snonce = nonce(&self.reply);
                }
                supplicant::Outcome::Keys {
                    reply: len,
                    tk,
                    gtk,
                    igtk,
                } => {
                    self.reply = reply[..len].to_vec();
                    if let Some(tk) = tk {
                        assert!(!self.tks.contains(&tk), "the station took a pairwise key twice");
                        self.tks.push(tk);
                    }
                    if let Some(gtk) = gtk {
                        assert!(self.gtk.as_ref() != Some(&gtk), "the station took a group key twice");
                        self.gtk = Some(gtk);
                    }
                    if let Some(igtk) = igtk {
                        assert!(self.igtk.as_ref() != Some(&igtk), "the station took an IGTK twice");
                        self.igtk = Some(igtk);
                    }
                }
            }
        }
    }

    let mut input = Input(data);
    let config = input.byte();
    let offer = match config % 3 {
        0 => Offer { psk: true, sae: false },
        1 => Offer { psk: false, sae: true },
        _ => Offer { psk: true, sae: true },
    };
    // On a transition network, a WPA3 station uses SAE.
    let sae = offer.sae && (!offer.psk || config & 4 != 0);
    let ap_rsne = Rsne::access_point(offer);
    let Some(supplicant) = Supplicant::new(PMK, AA, SPA, ap_rsne, [config; 32], sae) else {
        panic!("the station takes nothing that the access point offers");
    };
    #[cfg(feature = "wpa3")]
    let rsnxe = (sae && config & 8 != 0).then(crate::rsn::Rsnxe::sae_h2e);
    #[cfg(not(feature = "wpa3"))]
    let rsnxe = None;
    #[cfg(feature = "wpa3")]
    let pmkid = (sae && config & 16 != 0).then_some([0x77; 16]);
    #[cfg(feature = "wpa3")]
    let supplicant = supplicant.with_pmkid(pmkid);
    let mut supplicant = supplicant.with_rsnxe(rsnxe, rsnxe);
    let suite = supplicant.suite();
    assert_eq!(
        supplicant.rsne().check_station(offer),
        Ok(suite),
        "the access point refuses the station's association request"
    );
    let authenticator =
        Authenticator::new(PMK, AA, SPA, ap_rsne, *supplicant.rsne(), suite, ANONCE).with_rsnxe(rsnxe, rsnxe);
    #[cfg(feature = "wpa3")]
    let authenticator = authenticator.with_pmkid(pmkid);
    let mut authenticator = authenticator;
    let mut counter = 0;
    let mut keys = GroupKeys::derive(&[7; 32], &AA, counter);

    let mut station = Station::default();
    let mut from_ap = Vec::new();
    let mut ap_tk = None;
    let mut changed = false;
    for _ in 0..64 {
        if input.is_empty() {
            break;
        }
        match input.byte() % 6 {
            step @ (0 | 1) => {
                if step == 0 {
                    let mut out = [0; MESSAGE_MAX];
                    let Some(len) = authenticator.next_message(&keys, &mut out) else {
                        continue;
                    };
                    from_ap = out[..len].to_vec();
                }
                let mut frame = from_ap.clone();
                // The station's keys: from the ANonce of the frame, and its own SNonce.
                let station_ptk = station
                    .snonce
                    .map(|snonce| ptk(suite.akm, &nonce(&frame).unwrap_or(ANONCE), &snonce));
                match input.byte() % 4 {
                    0 => {}
                    1 => {
                        input.mutate(&mut frame);
                        changed = true;
                    }
                    2 => {
                        input.mutate(&mut frame);
                        if let Some(ptk) = &station_ptk {
                            sign(&mut frame, &ptk.kck);
                        }
                        changed = true;
                    }
                    _ => {
                        if let Some(ptk) = &station_ptk {
                            change_key_data(&mut frame, ptk, &mut input);
                            changed = true;
                        }
                    }
                }
                station.receive(&mut supplicant, &frame);
            }
            2 => {
                let mut frame = station.reply.clone();
                match input.byte() % 3 {
                    0 => {}
                    1 => {
                        input.mutate(&mut frame);
                        changed = true;
                    }
                    _ => {
                        input.mutate(&mut frame);
                        // The access point's keys: from its ANonce, and the frame's SNonce.
                        if let Some(snonce) = nonce(&frame).or(station.snonce) {
                            sign(&mut frame, &ptk(suite.akm, &ANONCE, &snonce).kck);
                        }
                        changed = true;
                    }
                }
                let mut out = [0; MESSAGE_MAX];
                match authenticator.handle(&frame, &keys, &mut out) {
                    authenticator::Outcome::Send(len) => from_ap = out[..len].to_vec(),
                    authenticator::Outcome::Done { tk } => {
                        assert!(ap_tk.is_none(), "the access point installed a pairwise key twice");
                        ap_tk = Some(tk);
                    }
                    _ => {}
                }
            }
            3 => {
                counter += 1;
                keys = GroupKeys::derive(&[7; 32], &AA, counter);
                let _ = authenticator.start_group(keys);
            }
            4 => {
                let len = usize::from(input.byte());
                let frame = input.bytes(len);
                station.receive(&mut supplicant, frame);
                changed = true;
            }
            _ => {
                let len = usize::from(input.byte());
                let frame = input.bytes(len);
                let mut out = [0; MESSAGE_MAX];
                let _ = authenticator.handle(frame, &keys, &mut out);
                changed = true;
            }
        }
    }
    let (false, Some(ap_tk), Some(&station_tk)) = (changed, ap_tk, station.tks.last()) else {
        return None;
    };
    assert_eq!(ap_tk, station_tk, "the two sides ended with different pairwise keys");
    Some((ap_tk, station_tk))
}

/// The address of our side of an SAE exchange, and of the peer's.
#[cfg(feature = "wpa3")]
const SAE_ADDRS: ([u8; 6], [u8; 6]) = ([0x02, 0, 0, 0, 0, 1], [0x02, 0, 0, 0, 0, 2]);

/// An SAE exchange with a peer whose commit and confirm the fuzzer may change, or replace with
/// its own bytes, our own commit (a reflection), or one whose element makes the shared secret the
/// identity. With nothing changed, both sides must agree on the PMK.
#[cfg(feature = "wpa3")]
pub fn sae(data: &[u8]) {
    use std::sync::OnceLock;

    use p256::ProjectivePoint;

    use crate::sae::{Pt, Sae, COMMIT_LEN, CONFIRM_LEN};
    use crate::wpa3::scalars;

    static PWE: OnceLock<ProjectivePoint> = OnceLock::new();
    let pwe = *PWE.get_or_init(|| Pt::derive(b"nrf70", b"password", None).pwe(&SAE_ADDRS.0, &SAE_ADDRS.1));

    let mut input = Input(data);
    let mode = input.byte();
    let (rand, mask) = scalars(&[1; 32], 1);
    let Some(mut ours) = Sae::new(pwe, rand, mask) else {
        return;
    };
    let (rand, mask) = scalars(&[2; 32], u32::from(mode >> 4));
    let Some(mut peer) = Sae::new(pwe, rand, mask) else {
        return;
    };
    let mut changed = false;
    // A reflected commit, and one whose shared secret is the identity, must be refused.
    let mut must_refuse = false;

    let mut commit = [0; COMMIT_LEN];
    let len = peer.write_commit(&[], &mut commit);
    let mut commit = commit[..len].to_vec();
    if mode & 1 != 0 {
        input.mutate(&mut commit);
        changed = true;
    }
    if mode & 2 != 0 {
        let len = usize::from(input.byte());
        commit = input.bytes(len).to_vec();
        changed = true;
    }
    if mode & 8 != 0 {
        let mut ours_commit = [0; COMMIT_LEN];
        let len = ours.write_commit(&[], &mut ours_commit);
        commit = ours_commit[..len].to_vec();
        changed = true;
        must_refuse = true;
    }
    if mode & 0x80 != 0 {
        let mut wide = [0; 48];
        let bytes = input.bytes(48);
        wide[..bytes.len()].copy_from_slice(bytes);
        if let Some(scalar) = Sae::scalar_from(&wide) {
            commit = crate::sae::fuzz::identity_commit(pwe, scalar).to_vec();
            changed = true;
            must_refuse = true;
        }
    }
    let accepted = ours.process_commit(&commit).is_ok();
    assert!(
        !(accepted && must_refuse),
        "a reflected commit, or one whose secret is the identity, was taken"
    );
    assert!(accepted || changed, "an honest commit was refused");
    if !accepted {
        return;
    }

    let mut our_commit = [0; COMMIT_LEN];
    let len = ours.write_commit(&[], &mut our_commit);
    if peer.process_commit(&our_commit[..len]).is_err() {
        assert!(changed, "the peer refused our commit");
        return;
    }
    let mut confirm = [0; CONFIRM_LEN];
    let len = peer.write_confirm(&mut confirm);
    let mut confirm = confirm[..len].to_vec();
    if mode & 4 != 0 {
        input.mutate(&mut confirm);
        changed = true;
    }
    let confirmed = ours.check_confirm(&confirm).is_ok();
    if !changed {
        assert!(confirmed, "an honest confirm was refused");
        let mut our_confirm = [0; CONFIRM_LEN];
        let len = ours.write_confirm(&mut our_confirm);
        assert!(peer.check_confirm(&our_confirm[..len]).is_ok());
        assert_eq!(ours.pmksa().pmk, peer.pmksa().pmk);
    }
}

/// A WPA3 password, its identifier and the SSID: the PT of hash to element, and the PWE of
/// hunting and pecking.
#[cfg(feature = "wpa3")]
pub fn sae_password(data: &[u8]) {
    let mut input = Input(data);
    let ssid_len = usize::from(input.byte() % 33);
    let ssid = input.bytes(ssid_len);
    let identifier_len = input.byte();
    let identifier = (identifier_len & 0x80 != 0).then(|| input.bytes(usize::from(identifier_len % 33)));
    let password = input.bytes(129);
    let Some(wpa3) = crate::wpa3::Wpa3::new(ssid, password, identifier, [0; 32]) else {
        return;
    };
    let _ = wpa3.pt.pwe(&SAE_ADDRS.0, &SAE_ADDRS.1);
    let _ = crate::sae::pwe_hunting_and_pecking(password, identifier.unwrap_or(&[]), &SAE_ADDRS.0, &SAE_ADDRS.1);
}

/// What a fuzz target needs of defmt, which the tests have in `lib.rs`.
#[cfg(not(test))]
mod defmt_sink {
    #[defmt::global_logger]
    struct NullLogger;

    unsafe impl defmt::Logger for NullLogger {
        fn acquire() {}
        unsafe fn flush() {}
        unsafe fn release() {}
        unsafe fn write(_bytes: &[u8]) {}
    }

    #[defmt::panic_handler]
    fn defmt_panic() -> ! {
        core::panic!("defmt panic")
    }

    defmt::timestamp!("");
}

#[cfg(test)]
mod tests {
    use std::vec::Vec;

    use super::*;

    /// Runs `target` `runs` times on inputs of up to `max_len` random bytes, and on random changes
    /// of `seeds`: what `cargo test` can do without a fuzzer, so that a regression shows there.
    fn exercise(target: fn(&[u8]), runs: usize, max_len: usize, seeds: &[&[u8]]) {
        // xorshift64, from a fixed seed: the same inputs on every run.
        let mut state = 0x2545_F491_4F6C_DD1D_u64;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        for run in 0..runs {
            let mut data: Vec<u8> = match seeds.get(run % (seeds.len() + 1)) {
                Some(seed) => seed.to_vec(),
                None => (0..next() as usize % (max_len + 1)).map(|_| next() as u8).collect(),
            };
            for _ in 0..next() % 4 {
                if !data.is_empty() {
                    let at = next() as usize % data.len();
                    data[at] ^= next() as u8 | 1;
                }
            }
            target(&data);
        }
    }

    #[test]
    fn received_frames_are_read_without_panicking() {
        // An MPDU from the access point with an RFC 1042 header, and an A-MSDU subframe.
        let mut mpdu = std::vec![0, 24, 0x08, 0x02];
        mpdu.extend_from_slice(&[0; 22]);
        mpdu.extend_from_slice(&[0xAA, 0xAA, 0x03, 0, 0, 0, 0x08, 0x00, 0x45]);
        exercise(rx_frame, 2000, 128, &[&mpdu]);
        exercise(elements, 2000, 64, &[&[0, 3, b'a', b'b', b'c', 255, 2, 33, 7]]);
        exercise(scan_result, 500, 512, &[]);
    }

    #[cfg(feature = "wpa2")]
    #[test]
    fn rsn_elements_and_key_frames_are_read_without_panicking() {
        let rsne = crate::tests::hex("00 0100 000fac04 0100 000fac04 0200 000fac02 000fac08 c000");
        exercise(rsn, 2000, 64, &[&rsne]);
        exercise(key_frame, 2000, 160, &[]);
        exercise(key_data, 2000, 96, &[]);
    }

    #[cfg(all(feature = "ap", feature = "wpa2"))]
    #[test]
    fn key_data_comes_back_as_it_was_wrapped() {
        exercise(wrap, 500, 300, &[]);
    }

    #[cfg(feature = "ap")]
    #[test]
    fn management_frames_to_the_access_point_are_read_without_panicking() {
        // An association request with the SSID, rates, HT capabilities and an RSNE.
        let mut request = std::vec![1, 0x00, 0x00, 0x00, 0x00];
        request.extend_from_slice(&[0x02, 0x70, 0x02, 0, 0, 1]);
        request.extend_from_slice(&[0x02, 0x55, 0, 0, 0, 2]);
        request.extend_from_slice(&[0x02, 0x70, 0x02, 0, 0, 1, 0, 0]);
        request.extend_from_slice(&[0x31, 0x04, 0x0A, 0x00]);
        request.extend_from_slice(&[0, 8]);
        request.extend_from_slice(b"nrf70-ap");
        request.extend_from_slice(&[1, 8, 0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24]);
        request.extend_from_slice(&[45, 26]);
        request.extend_from_slice(&[0; 26]);
        request.extend_from_slice(&crate::tests::hex(
            "30 14 0100 000fac04 0100 000fac04 0100 000fac02 8000",
        ));
        for kind in 0..4 {
            request[0] = kind;
            exercise(management, 500, 128, &[&request]);
        }
    }

    /// The steps of an honest handshake: message 1, message 2, message 3, message 4, then a
    /// group key handshake.
    #[cfg(all(feature = "ap", feature = "wpa2"))]
    const HONEST: [u8; 14] = [0, 0, 0, 2, 0, 1, 0, 2, 0, 3, 0, 0, 2, 0];

    #[cfg(all(feature = "ap", feature = "wpa2"))]
    #[test]
    fn an_honest_handshake_ends_with_the_same_keys_on_both_sides() {
        for config in 0..32 {
            let mut steps = HONEST;
            steps[0] = config;
            assert!(handshake_keys(&steps).is_some(), "configuration {config}");
        }
    }

    #[cfg(all(feature = "ap", feature = "wpa2"))]
    #[test]
    fn handshakes_with_changed_frames_never_reinstall_a_key() {
        exercise(handshake, 300, 256, &[&HONEST]);
    }

    #[cfg(feature = "wpa3")]
    #[test]
    fn sae_refuses_a_reflected_commit_and_one_whose_secret_is_the_identity() {
        sae(&[8]);
        sae(&[0x80, 1, 2, 3, 4, 5]);
        sae(&[0x88, 0x42]);
    }

    #[cfg(feature = "wpa3")]
    #[test]
    fn sae_exchanges_with_changed_messages_do_not_panic() {
        exercise(sae, 8, 200, &[&[0], &[1, 2, 0, 5, 0], &[4, 0, 1, 0, 0, 0x42]]);
        exercise(sae_password, 4, 64, &[]);
    }
}
