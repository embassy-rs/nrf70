//! EAPOL-Key frames (IEEE 802.11-2020, 12.7.2), from the IEEE 802.1X header on: their layout and
//! MIC, their key data and its encryption, and the group keys it carries. The supplicant and the
//! authenticator both read and write them.
//!
//! The AES-128 block cipher and AES-128-CMAC come from `embassy-crypto`, as HMAC does (see
//! `crypto.rs`); the AES key wrap and unwrap (RFC 3394) are here.

use embassy_crypto::{Aes128, Aes128Cmac};

use crate::crypto::{hmac_sha1, TK_LEN};
use crate::ieee80211::{IE_RSN, IE_RSNXE};
use crate::rsn::{GroupCipher, RSNE_MAX, RSNXE_MAX};

/// Ethertype of EAPOL frames (IEEE 802.1X).
pub(crate) const ETHERTYPE_EAPOL: u16 = 0x888E;

/// The GTK key data encapsulation (KDE): OUI 00-0F-AC, data type 1.
pub(crate) const KDE_GTK: [u8; 4] = [0x00, 0x0F, 0xAC, 1];
/// The IGTK KDE: data type 9.
pub(crate) const KDE_IGTK: [u8; 4] = [0x00, 0x0F, 0xAC, 9];

/// IEEE 802.1X-2001, which wpa_supplicant sends too: some APs refuse a newer version.
pub(crate) const EAPOL_VERSION: u8 = 1;
pub(crate) const EAPOL_TYPE_KEY: u8 = 3;
/// Key descriptor type of RSN (WPA2). WPA's own, 254, is not supported.
pub(crate) const DESCRIPTOR_RSN: u8 = 2;

pub(crate) const OFFSET_KEY_INFO: usize = 5;
pub(crate) const OFFSET_REPLAY_COUNTER: usize = 9;
pub(crate) const OFFSET_NONCE: usize = 17;
pub(crate) const OFFSET_RSC: usize = 65;
pub(crate) const OFFSET_MIC: usize = 81;
pub(crate) const OFFSET_KEY_DATA_LEN: usize = 97;
/// Length of an EAPOL-Key frame without key data.
pub(crate) const KEY_FRAME_LEN: usize = 99;
pub(crate) const MIC_LEN: usize = 16;

/// Key descriptor version 2: HMAC-SHA1-128 as MIC, AES key wrap for the key data. It is the one
/// that goes with CCMP and PSK key management.
pub(crate) const INFO_VERSION_HMAC_SHA1_AES: u16 = 2;
/// Key descriptor version 3: AES-128-CMAC as MIC, AES key wrap for the key data, with PSK-SHA256.
pub(crate) const INFO_VERSION_AES_CMAC: u16 = 3;
/// Key descriptor version 0: the AKM says. With SAE (AKM 8): AES-128-CMAC as MIC, AES key wrap
/// for the key data, as with version 3 (HMAC-SHA256 is SAE-EXT-KEY's, AKM 24).
pub(crate) const INFO_VERSION_AKM_DEFINED: u16 = 0;
pub(crate) const INFO_VERSION_MASK: u16 = 0x0007;
pub(crate) const INFO_PAIRWISE: u16 = 1 << 3;
pub(crate) const INFO_INSTALL: u16 = 1 << 6;
pub(crate) const INFO_ACK: u16 = 1 << 7;
pub(crate) const INFO_MIC: u16 = 1 << 8;
pub(crate) const INFO_SECURE: u16 = 1 << 9;
pub(crate) const INFO_ERROR: u16 = 1 << 10;
pub(crate) const INFO_REQUEST: u16 = 1 << 11;
pub(crate) const INFO_ENCRYPTED_KEY_DATA: u16 = 1 << 12;

/// Whether `frame`, from its 802.1X header on, is message 1 of a 4-way handshake: pairwise, asking
/// for an answer, without a MIC.
pub(crate) fn is_message_1(frame: &[u8]) -> bool {
    KeyFrame::parse(frame)
        .is_some_and(|key| key.info & (INFO_PAIRWISE | INFO_ACK | INFO_MIC) == INFO_PAIRWISE | INFO_ACK)
}

/// Longest reply: message 2, which carries the RSNE and the RSNXE.
pub(crate) const REPLY_MAX: usize = KEY_FRAME_LEN + RSNE_MAX + RSNXE_MAX;

/// Longest key data the driver unwraps. Message 3 carries the RSNE and the GTK, and sometimes a
/// second RSNE or keys the driver ignores.
pub(crate) const KEY_DATA_MAX: usize = 256;

/// A received EAPOL-Key frame.
pub(crate) struct KeyFrame<'a> {
    /// The frame from the 802.1X header to the end of the key data: what the MIC covers.
    pub(crate) pdu: &'a [u8],
    pub(crate) info: u16,
    pub(crate) replay_counter: u64,
    pub(crate) nonce: [u8; 32],
    /// The receive sequence counter of the GTK in the key data, low byte first.
    pub(crate) rsc: [u8; 6],
    pub(crate) key_data: &'a [u8],
}

impl<'a> KeyFrame<'a> {
    /// Parses an RSN EAPOL-Key frame. `frame` starts at the 802.1X header and may be followed by
    /// padding.
    pub(crate) fn parse(frame: &'a [u8]) -> Option<Self> {
        let be16 = |at: usize| Some(u16::from_be_bytes(frame.get(at..at + 2)?.try_into().ok()?));
        if frame.get(1) != Some(&EAPOL_TYPE_KEY) {
            return None;
        }
        let pdu = frame.get(..4 + be16(2)? as usize)?;
        if pdu.len() < KEY_FRAME_LEN || pdu[4] != DESCRIPTOR_RSN {
            return None;
        }
        let key_data = pdu.get(KEY_FRAME_LEN..KEY_FRAME_LEN + be16(OFFSET_KEY_DATA_LEN)? as usize)?;
        Some(Self {
            pdu,
            info: be16(OFFSET_KEY_INFO)?,
            replay_counter: u64::from_be_bytes(pdu[OFFSET_REPLAY_COUNTER..OFFSET_NONCE].try_into().ok()?),
            nonce: pdu[OFFSET_NONCE..OFFSET_NONCE + 32].try_into().ok()?,
            rsc: pdu[OFFSET_RSC..OFFSET_RSC + 6].try_into().ok()?,
            key_data,
        })
    }

    /// Whether the frame's MIC is the one `kck` gives over the frame with its MIC field zeroed,
    /// for the key descriptor version of the frame.
    pub(crate) fn mic_is_valid(&self, kck: &[u8; 16]) -> bool {
        let expected = mic(
            self.info,
            kck,
            &[
                &self.pdu[..OFFSET_MIC],
                &[0; MIC_LEN],
                &self.pdu[OFFSET_MIC + MIC_LEN..],
            ],
        );
        // Compared in constant time.
        let received = &self.pdu[OFFSET_MIC..OFFSET_MIC + MIC_LEN];
        expected.iter().zip(received).fold(0, |diff, (a, b)| diff | (a ^ b)) == 0
    }
}

/// The MIC of the EAPOL-Key frame made of `parts`, for the key descriptor version in `info`:
/// HMAC-SHA1 truncated to 128 bits, or AES-128-CMAC (version 3, and version 0 with SAE, the only
/// AKM-defined one the driver knows).
pub(crate) fn mic(info: u16, kck: &[u8; 16], parts: &[&[u8]]) -> [u8; MIC_LEN] {
    let mut out = [0; MIC_LEN];
    if matches!(
        info & INFO_VERSION_MASK,
        INFO_VERSION_AES_CMAC | INFO_VERSION_AKM_DEFINED
    ) {
        let mut mac = Aes128Cmac::new(kck);
        for part in parts {
            mac.update(part);
        }
        out = mac.finalize();
    } else {
        out.copy_from_slice(&hmac_sha1(kck, parts)[..MIC_LEN]);
    }
    out
}

/// Writes an EAPOL-Key frame with its MIC into `out`, and returns its length. The key length, IV,
/// RSC and reserved fields are zero, as in every frame a supplicant sends.
pub(crate) fn write_key_frame(
    out: &mut [u8],
    info: u16,
    replay_counter: u64,
    nonce: &[u8; 32],
    key_data: &[u8],
    kck: &[u8; 16],
) -> usize {
    let header = KeyHeader {
        info,
        key_len: 0,
        replay_counter,
        nonce: *nonce,
        rsc: [0; 6],
    };
    header.write(out, key_data, Some(kck))
}

/// The fields of an EAPOL-Key frame that change from one to the next.
pub(crate) struct KeyHeader {
    pub(crate) info: u16,
    /// The length of the pairwise key, which the authenticator gives and the supplicant leaves at
    /// zero.
    pub(crate) key_len: u16,
    pub(crate) replay_counter: u64,
    pub(crate) nonce: [u8; 32],
    /// The packet number of the group key in the key data, low byte first.
    pub(crate) rsc: [u8; 6],
}

impl KeyHeader {
    /// Writes the frame with `key_data` into `out`, and its MIC if there is a `kck`, and returns
    /// its length. The IV and reserved fields are zero.
    pub(crate) fn write(&self, out: &mut [u8], key_data: &[u8], kck: Option<&[u8; 16]>) -> usize {
        let len = KEY_FRAME_LEN + key_data.len();
        let frame = &mut out[..len];
        frame.fill(0);
        frame[0] = EAPOL_VERSION;
        frame[1] = EAPOL_TYPE_KEY;
        frame[2..4].copy_from_slice(&((len - 4) as u16).to_be_bytes());
        frame[4] = DESCRIPTOR_RSN;
        frame[OFFSET_KEY_INFO..OFFSET_KEY_INFO + 2].copy_from_slice(&self.info.to_be_bytes());
        frame[OFFSET_KEY_INFO + 2..OFFSET_REPLAY_COUNTER].copy_from_slice(&self.key_len.to_be_bytes());
        frame[OFFSET_REPLAY_COUNTER..OFFSET_NONCE].copy_from_slice(&self.replay_counter.to_be_bytes());
        frame[OFFSET_NONCE..OFFSET_NONCE + 32].copy_from_slice(&self.nonce);
        frame[OFFSET_RSC..OFFSET_RSC + 6].copy_from_slice(&self.rsc);
        frame[OFFSET_KEY_DATA_LEN..KEY_FRAME_LEN].copy_from_slice(&(key_data.len() as u16).to_be_bytes());
        frame[KEY_FRAME_LEN..].copy_from_slice(key_data);

        if let Some(kck) = kck {
            let mic = mic(self.info, kck, &[frame]);
            frame[OFFSET_MIC..OFFSET_MIC + MIC_LEN].copy_from_slice(&mic);
        }
        len
    }
}

/// Pads `data` as IEEE 802.11-2020, 12.7.2 asks (0xDD, then zeros, up to a multiple of 8 bytes
/// and at least 16) and wraps it with the key encryption key into `out`. Returns the wrapped
/// length, 8 bytes more than the padded data, or `None` if `out` is too short.
#[cfg(feature = "ap")]
pub(crate) fn wrap_key_data(kek: &[u8; 16], data: &[u8], out: &mut [u8]) -> Option<usize> {
    let mut padded = [0; KEY_DATA_MAX];
    let mut len = data.len();
    padded.get_mut(..len)?.copy_from_slice(data);
    if !len.is_multiple_of(8) || len < 16 {
        *padded.get_mut(len)? = 0xDD;
        len = (len + 1).next_multiple_of(8).max(16);
    }
    aes_key_wrap(kek, padded.get(..len)?, out)
}

/// A group temporal key, as the RPU takes it.
#[derive(Clone, PartialEq, Eq)]
pub(crate) struct Gtk {
    /// Key ID, 0 to 3.
    pub(crate) index: u8,
    pub(crate) cipher: GroupCipher,
    pub(crate) key: [u8; TK_LEN + 16],
    /// The packet number the AP has reached with this key, low byte first.
    pub(crate) rsc: [u8; 6],
}

impl Gtk {
    /// The key: 16 bytes for CCMP; for TKIP, 32, the Michael MIC keys being in the station's
    /// order (the RX one first), as wpa_supplicant swaps the AP's before installing it.
    pub(crate) fn key(&self) -> &[u8] {
        &self.key[..self.cipher.key_len()]
    }
}

/// An integrity group temporal key, for BIP-CMAC-128, as the AP hands it over.
#[derive(Clone, PartialEq, Eq)]
pub(crate) struct Igtk {
    /// Key ID, 4 or 5.
    pub(crate) index: u8,
    pub(crate) key: [u8; 16],
    /// The packet number the AP has reached with this key, low byte first.
    pub(crate) ipn: [u8; 6],
}

/// What the key data of message 3 or of a group key message holds, as far as the driver uses it.
pub(crate) struct KeyData<'a> {
    /// The first RSNE, with its header.
    pub(crate) rsne: Option<&'a [u8]>,
    /// The first RSNXE, with its header.
    pub(crate) rsnxe: Option<&'a [u8]>,
    /// The GTK KDE's body: key ID and flags, a reserved byte, then the key.
    gtk: Option<&'a [u8]>,
    /// The IGTK KDE's body: key ID, IPN, then the key.
    igtk: Option<&'a [u8]>,
}

impl<'a> KeyData<'a> {
    /// Walks the elements and KDEs of unwrapped key data (IEEE 802.11-2020, 12.7.2).
    pub(crate) fn parse(mut data: &'a [u8]) -> Self {
        const KDE: u8 = 0xDD;
        let mut found = Self {
            rsne: None,
            rsnxe: None,
            gtk: None,
            igtk: None,
        };
        while let [id, len, rest @ ..] = data {
            // Padding: a KDE of length zero, then zeros.
            if *id == KDE && *len == 0 {
                break;
            }
            let Some(body) = rest.get(..*len as usize) else {
                break;
            };
            match *id {
                IE_RSN if found.rsne.is_none() => found.rsne = Some(&data[..2 + body.len()]),
                IE_RSNXE if found.rsnxe.is_none() => found.rsnxe = Some(&data[..2 + body.len()]),
                KDE if body.len() >= 4 && body[..4] == KDE_GTK => found.gtk = Some(&body[4..]),
                KDE if body.len() >= 4 && body[..4] == KDE_IGTK => found.igtk = Some(&body[4..]),
                _ => {}
            }
            data = &rest[body.len()..];
        }
        found
    }

    pub(crate) fn gtk(&self, rsc: [u8; 6], cipher: GroupCipher) -> Option<Gtk> {
        let [flags, _reserved, key @ ..] = self.gtk? else {
            return None;
        };
        if key.len() != cipher.key_len() {
            return None;
        }
        let mut gtk = Gtk {
            index: flags & 0x03,
            cipher,
            key: [0; TK_LEN + 16],
            rsc,
        };
        gtk.key[..TK_LEN].copy_from_slice(&key[..TK_LEN]);
        if cipher == GroupCipher::Tkip {
            // The AP's TX Michael key is the station's RX one, and the other way round.
            gtk.key[TK_LEN..TK_LEN + 8].copy_from_slice(&key[TK_LEN + 8..]);
            gtk.key[TK_LEN + 8..].copy_from_slice(&key[TK_LEN..TK_LEN + 8]);
        }
        Some(gtk)
    }

    pub(crate) fn igtk(&self) -> Option<Igtk> {
        let body = self.igtk?;
        let index = u16::from_le_bytes(body.get(0..2)?.try_into().ok()?);
        if !(4..=5).contains(&index) || body.len() != 24 {
            return None;
        }
        Some(Igtk {
            index: index as u8,
            ipn: body[2..8].try_into().ok()?,
            key: body[8..24].try_into().ok()?,
        })
    }
}

/// Unwraps the key data of `key` with the key encryption key (NIST AES key wrap, RFC 3394), which
/// also proves its integrity. `None` if the key data is not encrypted or does not unwrap.
pub(crate) fn unwrap_key_data<'a>(key: &KeyFrame, kek: &[u8; 16], buf: &'a mut [u8; KEY_DATA_MAX]) -> Option<&'a [u8]> {
    if key.info & INFO_ENCRYPTED_KEY_DATA == 0 {
        return None;
    }
    aes_key_unwrap(kek, key.key_data, buf)
}

/// The initial value a wrapped key carries, which is how unwrapping proves its integrity (RFC
/// 3394, 2.2.3.1).
const KEY_WRAP_IV: [u8; 8] = [0xA6; 8];

/// AES key unwrap (RFC 3394, 2.2.2) of `wrapped`, 64-bit blocks of which the first holds the
/// integrity check, into `out`. Returns the unwrapped key, or `None` if `wrapped` is not at least
/// three whole blocks, does not fit `out`, or fails the integrity check.
fn aes_key_unwrap<'a>(kek: &[u8; 16], wrapped: &[u8], out: &'a mut [u8]) -> Option<&'a [u8]> {
    if wrapped.len() < 24 || !wrapped.len().is_multiple_of(8) {
        return None;
    }
    let n = wrapped.len() / 8 - 1;
    let out = out.get_mut(..8 * n)?;
    out.copy_from_slice(&wrapped[8..]);
    let mut a = [0; 8];
    a.copy_from_slice(&wrapped[..8]);

    let aes = Aes128::new(kek);
    for j in (0..6).rev() {
        for i in (0..n).rev() {
            let t = (n * j + i + 1) as u64;
            let mut block = [0; 16];
            block[..8].copy_from_slice(&(u64::from_be_bytes(a) ^ t).to_be_bytes());
            block[8..].copy_from_slice(&out[8 * i..8 * i + 8]);
            aes.decrypt_block(&mut block);
            a.copy_from_slice(&block[..8]);
            out[8 * i..8 * i + 8].copy_from_slice(&block[8..]);
        }
    }

    // Every byte is looked at, whatever the first ones are.
    let difference = a.iter().zip(KEY_WRAP_IV).fold(0, |acc, (a, iv)| acc | (a ^ iv));
    (difference == 0).then_some(out)
}

/// AES key wrap (RFC 3394, 2.2.1) of `data`, whole 64-bit blocks, at least two, into `out`: the
/// access point's side of [`aes_key_unwrap`]. Returns the wrapped length, 8 bytes more, or `None`
/// if `data` is not whole blocks or `out` is too short.
#[cfg(any(feature = "ap", test))]
pub(crate) fn aes_key_wrap(kek: &[u8; 16], data: &[u8], out: &mut [u8]) -> Option<usize> {
    if data.len() < 16 || !data.len().is_multiple_of(8) {
        return None;
    }
    let n = data.len() / 8;
    let out = out.get_mut(..8 * (n + 1))?;
    out[8..].copy_from_slice(data);
    let mut a = KEY_WRAP_IV;
    let aes = Aes128::new(kek);
    for j in 0..6 {
        for i in 1..=n {
            let mut block = [0; 16];
            block[..8].copy_from_slice(&a);
            block[8..].copy_from_slice(&out[8 * i..8 * i + 8]);
            aes.encrypt_block(&mut block);
            let t = (n * j + i) as u64;
            a = (u64::from_be_bytes(block[..8].try_into().ok()?) ^ t).to_be_bytes();
            out[8 * i..8 * i + 8].copy_from_slice(&block[8..]);
        }
    }
    out[..8].copy_from_slice(&a);
    Some(8 * (n + 1))
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::assert_eq;
    use std::vec::Vec;

    use super::*;
    use crate::tests::hex;

    /// [`aes_key_wrap`] into a vector.
    fn wrap(kek: &[u8; 16], data: &[u8]) -> Vec<u8> {
        let mut wrapped = std::vec![0; data.len() + 8];
        let len = aes_key_wrap(kek, data, &mut wrapped).unwrap();
        wrapped.truncate(len);
        wrapped
    }

    // RFC 3394, 4.1: 128 bits of key data wrapped with a 128-bit KEK.
    #[test]
    fn key_unwrap_matches_the_rfc_test_vector() {
        let kek: [u8; 16] = hex("000102030405060708090A0B0C0D0E0F").try_into().unwrap();
        let key = hex("00112233445566778899AABBCCDDEEFF");
        let wrapped = hex("1FA68B0A8112B447AEF34BD8FB5A7B829D3E862371D2CFE5");
        assert_eq!(wrap(&kek, &key), wrapped);

        let mut out = [0; 32];
        assert_eq!(aes_key_unwrap(&kek, &wrapped, &mut out), Some(key.as_slice()));
    }

    #[test]
    fn key_unwrap_refuses_what_was_not_wrapped_with_the_key() {
        let kek = [0x42; 16];
        let wrapped = wrap(&kek, &[0x11; 24]);
        let mut out = [0; 32];
        assert_eq!(aes_key_unwrap(&kek, &wrapped, &mut out), Some([0x11; 24].as_slice()));

        // A flipped bit, another key, a length that is not whole blocks, too little to hold a
        // key, and more than the buffer takes.
        let mut tampered = wrapped.clone();
        tampered[20] ^= 1;
        assert_eq!(aes_key_unwrap(&kek, &tampered, &mut out), None);
        assert_eq!(aes_key_unwrap(&[0x43; 16], &wrapped, &mut out), None);
        assert_eq!(aes_key_unwrap(&kek, &wrapped[..31], &mut out), None);
        assert_eq!(aes_key_unwrap(&kek, &wrapped[..16], &mut out), None);
        assert_eq!(aes_key_unwrap(&kek, &wrapped, &mut out[..16]), None);
    }
}

/// What the fuzz targets reach of the EAPOL-Key frames: see `fuzz.rs`.
#[cfg(any(fuzzing, test))]
pub(crate) mod fuzz {
    use super::*;

    /// An EAPOL frame, from its 802.1X header on.
    pub fn key_frame(data: &[u8]) {
        let _ = is_message_1(data);
        if let Some(key) = KeyFrame::parse(data) {
            assert!(key.pdu.len() >= KEY_FRAME_LEN && key.pdu.len() <= data.len());
            assert!(KEY_FRAME_LEN + key.key_data.len() <= key.pdu.len());
            let _ = key.mic_is_valid(&[0x42; 16]);
            let mut buf = [0; KEY_DATA_MAX];
            let _ = unwrap_key_data(&key, &[0x42; 16], &mut buf);
        }
        key_data(data);
    }

    /// Unwrapped key data: elements and KDEs.
    pub fn key_data(data: &[u8]) {
        let found = KeyData::parse(data);
        for element in [found.rsne, found.rsnxe].into_iter().flatten() {
            assert_eq!(element.len(), 2 + usize::from(element[1]));
        }
        for cipher in [GroupCipher::Ccmp, GroupCipher::Tkip] {
            if let Some(gtk) = found.gtk([1; 6], cipher) {
                assert_eq!(gtk.key().len(), cipher.key_len());
            }
        }
        let _ = found.igtk();
    }

    /// Key data that the access point wraps into a frame comes back from the frame as it was,
    /// padded: `data[0]` makes the keys, the rest is the key data.
    #[cfg(feature = "ap")]
    pub fn wrap(data: &[u8]) {
        let [key, data @ ..] = data else {
            return;
        };
        let kek = [*key; 16];
        let mut wrapped = [0; KEY_DATA_MAX + 8];
        let Some(len) = wrap_key_data(&kek, data, &mut wrapped) else {
            assert!(data.len() + 1 > KEY_DATA_MAX);
            return;
        };
        let header = KeyHeader {
            info: INFO_VERSION_HMAC_SHA1_AES | INFO_ENCRYPTED_KEY_DATA,
            key_len: 0,
            replay_counter: 1,
            nonce: [0; 32],
            rsc: [0; 6],
        };
        let mut frame = [0; KEY_FRAME_LEN + KEY_DATA_MAX + 8];
        let frame_len = header.write(&mut frame, &wrapped[..len], Some(&kek));
        let key = KeyFrame::parse(&frame[..frame_len]).unwrap();
        assert!(key.mic_is_valid(&kek));
        let mut buf = [0; KEY_DATA_MAX];
        let unwrapped = unwrap_key_data(&key, &kek, &mut buf).unwrap();
        assert_eq!(unwrapped[..data.len()], *data);
    }
}
