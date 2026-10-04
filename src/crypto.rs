//! The key derivations of WPA2 and WPA3 (IEEE 802.11-2020, 12.7.1): HMAC, the PRF and the KDF
//! built on it, the PSK from a passphrase, and the pairwise transient key.

use hmac::{Hmac, KeyInit, Mac};
use sha1::Sha1;
use sha2::Sha256;

use crate::rsn::Akm;

/// Length of a CCMP-128 temporal key.
pub(crate) const TK_LEN: usize = 16;

/// Longest passphrase and shortest one (IEEE 802.11-2020, J.4.1).
const PASSPHRASE_LEN: core::ops::RangeInclusive<usize> = 8..=63;

/// The pre-shared key of a WPA2-Personal network from its passphrase: PBKDF2-HMAC-SHA1 with the
/// SSID as salt, 4096 iterations (IEEE 802.11-2020, J.4.1). `None` if the passphrase is not 8 to
/// 63 bytes long.
///
/// This is slow by design: 16,384 SHA-1 compressions.
pub(crate) fn psk_from_passphrase(passphrase: &[u8], ssid: &[u8]) -> Option<[u8; 32]> {
    if !PASSPHRASE_LEN.contains(&passphrase.len()) {
        return None;
    }
    let mut psk = [0; 32];
    pbkdf2::pbkdf2_hmac::<Sha1>(passphrase, ssid, 4096, &mut psk);
    Some(psk)
}

/// HMAC-SHA1(`key`, the concatenation of `parts`).
pub(crate) fn hmac_sha1(key: &[u8], parts: &[&[u8]]) -> [u8; 20] {
    hmac::<Hmac<Sha1>>(key, parts).into()
}

/// HMAC-SHA256(`key`, the concatenation of `parts`).
pub(crate) fn hmac_sha256(key: &[u8], parts: &[&[u8]]) -> [u8; 32] {
    hmac::<Hmac<Sha256>>(key, parts).into()
}

fn hmac<M: Mac + KeyInit>(key: &[u8], parts: &[&[u8]]) -> hmac::digest::Output<M> {
    let mut mac = match M::new_from_slice(key) {
        Ok(mac) => mac,
        // HMAC takes a key of any length.
        Err(_) => defmt::unreachable!(),
    };
    for part in parts {
        mac.update(part);
    }
    mac.finalize().into_bytes()
}

/// The 802.11 key derivation function over HMAC-SHA256 (IEEE 802.11-2020, 12.7.1.6.2): fills
/// `out` with HMAC-SHA256(`key`, i || `label` || `context` || length) for i = 1, 2, and so on, i
/// and the length of `out` in bits as 16-bit little-endian numbers.
pub(crate) fn kdf_sha256(key: &[u8], label: &[u8], context: &[u8], out: &mut [u8]) {
    let bits = (out.len() * 8) as u16;
    for (i, chunk) in out.chunks_mut(32).enumerate() {
        let block = hmac_sha256(
            key,
            &[&(i as u16 + 1).to_le_bytes(), label, context, &bits.to_le_bytes()],
        );
        chunk.copy_from_slice(&block[..chunk.len()]);
    }
}

/// The 802.11 pseudo-random function over HMAC-SHA1 (IEEE 802.11-2020, 12.7.1.2): fills `out`
/// with HMAC-SHA1(`key`, `label` || 0 || `data` || i) for i = 0, 1, and so on.
pub(crate) fn prf(key: &[u8], label: &[u8], data: &[u8], out: &mut [u8]) {
    for (i, chunk) in out.chunks_mut(20).enumerate() {
        let block = hmac_sha1(key, &[label, &[0], data, &[i as u8]]);
        chunk.copy_from_slice(&block[..chunk.len()]);
    }
}

/// The pairwise transient key for CCMP: the EAPOL-Key confirmation and encryption keys, and the
/// temporal key.
#[derive(Clone)]
pub(crate) struct Ptk {
    pub(crate) kck: [u8; 16],
    pub(crate) kek: [u8; 16],
    pub(crate) tk: [u8; TK_LEN],
}

impl Ptk {
    /// PRF-384(PMK, "Pairwise key expansion", Min(AA, SPA) || Max(AA, SPA) || Min(ANonce, SNonce)
    /// || Max(ANonce, SNonce)) (IEEE 802.11-2020, 12.7.1.3), with the KDF over HMAC-SHA256 in
    /// place of the PRF for PSK-SHA256 and SAE. `aa` is the authenticator's address (the BSSID)
    /// and `spa` the supplicant's.
    pub(crate) fn derive(
        akm: Akm,
        pmk: &[u8; 32],
        aa: &[u8; 6],
        spa: &[u8; 6],
        anonce: &[u8; 32],
        snonce: &[u8; 32],
    ) -> Self {
        let mut data = [0; 76];
        let (min, max) = if aa < spa { (aa, spa) } else { (spa, aa) };
        data[0..6].copy_from_slice(min);
        data[6..12].copy_from_slice(max);
        let (min, max) = if anonce < snonce {
            (anonce, snonce)
        } else {
            (snonce, anonce)
        };
        data[12..44].copy_from_slice(min);
        data[44..76].copy_from_slice(max);

        let mut out = [0; 48];
        match akm {
            Akm::Psk => prf(pmk, b"Pairwise key expansion", &data, &mut out),
            Akm::PskSha256 | Akm::Sae => kdf_sha256(pmk, b"Pairwise key expansion", &data, &mut out),
        }
        let mut ptk = Ptk {
            kck: [0; 16],
            kek: [0; 16],
            tk: [0; TK_LEN],
        };
        ptk.kck.copy_from_slice(&out[0..16]);
        ptk.kek.copy_from_slice(&out[16..32]);
        ptk.tk.copy_from_slice(&out[32..48]);
        ptk
    }
}

#[cfg(test)]
mod tests {
    use core::{assert, assert_eq};

    use super::*;
    use crate::tests::hex;

    // IEEE 802.11-2020, J.4.2: test vectors for the passphrase to PSK mapping.
    #[test]
    fn psk_matches_the_standard_test_vectors() {
        assert_eq!(
            psk_from_passphrase(b"password", b"IEEE").unwrap().as_slice(),
            hex("f42c6fc52df0ebef9ebb4b90b38a5f902e83fe1b135a70e23aed762e9710a12e")
        );
        assert_eq!(
            psk_from_passphrase(b"ThisIsAPassword", b"ThisIsASSID")
                .unwrap()
                .as_slice(),
            hex("0dc0d6eb90555ed6419756b9a15ec3e3209b63df707dd508d14581f8982721af")
        );
    }

    #[test]
    fn passphrase_must_be_8_to_63_bytes() {
        assert!(psk_from_passphrase(b"1234567", b"net").is_none());
        assert!(psk_from_passphrase(&[b'a'; 64], b"net").is_none());
        assert!(psk_from_passphrase(&[b'a'; 63], b"net").is_some());
    }

    // IEEE 802.11-2020, J.3.2: test vectors for the PRF.
    #[test]
    fn prf_matches_the_standard_test_vectors() {
        let mut out = [0; 64];
        prf(&[0x0b; 20], b"prefix", b"Hi There", &mut out);
        assert_eq!(
            out.as_slice(),
            hex("bcd4c650b30b9684951829e0d75f9d54b862175ed9f00606e17d8da35402ffee
                 75df78c3d31e0f889f012120c0862beb67753e7439ae242edb8373698356cf5a")
        );
        prf(b"Jefe", b"prefix", b"what do ya want for nothing?", &mut out);
        assert_eq!(
            out.as_slice(),
            hex("51f4de5b33f249adf81aeb713a3c20f4fe631446fabdfa58244759ae58ef9009
                 a99abf4eac2ca5fa87e692c440eb40023e7babb206d61de7b92f41529092b8fc")
        );
    }
}
