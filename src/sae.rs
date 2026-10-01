//! SAE (simultaneous authentication of equals), the password authenticated key exchange of
//! WPA3-Personal (IEEE 802.11-2020, 12.4), over group 19 (the NIST P-256 curve), which every WPA3
//! access point offers. The nRF70 firmware carries the authentication frames, and leaves the
//! exchange to the host, as the nRF Connect SDK leaves it to wpa_supplicant.
//!
//! Both ways to turn the password into the password element (PWE) are here: hunting and pecking,
//! which depends on the two MAC addresses and is computed for each access point, and hash to
//! element, where the part that depends on the password (PT) is computed once per SSID.
//!
//! Nothing here touches the RPU: [`Sae`] writes and checks the bodies of the commit and confirm
//! messages, so that it can be tested on its own.

use crate::supplicant::kdf_sha256;
use embassy_crypto::HmacSha256;
use p256::elliptic_curve::array::Array;
use p256::elliptic_curve::bigint::{ArrayEncoding, NonZero, U256};
use p256::elliptic_curve::consts::U48;
use p256::elliptic_curve::ff::{Field, PrimeField};
use p256::elliptic_curve::ops::Reduce;
use p256::elliptic_curve::point::{AffineCoordinates, DecompressPoint};
use p256::elliptic_curve::subtle::{Choice, ConditionallySelectable, ConstantTimeEq};
use p256::elliptic_curve::Curve;
use p256::hash2curve::MapToCurve;
use p256::{AffinePoint, FieldBytes, NistP256, ProjectivePoint, Scalar};

type FieldElement = <NistP256 as MapToCurve>::FieldElement;

/// The finite cyclic group: 19, NIST P-256.
pub(crate) const GROUP: u16 = 19;
/// Length of a scalar or of a coordinate.
const LEN: usize = 32;
/// A commit message without anti-clogging token: group, scalar, element.
pub(crate) const COMMIT_LEN: usize = 2 + 3 * LEN;
/// A confirm message: send-confirm and the confirm.
pub(crate) const CONFIRM_LEN: usize = 2 + LEN;

/// The prime p of P-256, big-endian.
const PRIME: [u8; LEN] = [
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
];

/// Hunting and pecking tries at least this many counters, whether or not it has found the PWE,
/// so that its duration says nothing about the password (wpa_supplicant's
/// `dragonfly_min_pwe_loop_iter`).
const HUNTING_AND_PECKING_ROUNDS: u8 = 40;

fn hmac_sha256(key: &[u8], parts: &[&[u8]]) -> [u8; LEN] {
    let mut mac = HmacSha256::new(key);
    for part in parts {
        mac.update(part);
    }
    mac.finalize()
}

/// HKDF-Expand with SHA-256 (RFC 5869), as far as SAE needs it: up to 64 bytes.
fn hkdf_expand(prk: &[u8; LEN], info: &[u8], out: &mut [u8]) {
    let mut block = [0; LEN];
    for (i, chunk) in out.chunks_mut(LEN).enumerate() {
        let counter = [i as u8 + 1];
        block = if i == 0 {
            hmac_sha256(prk, &[info, &counter])
        } else {
            hmac_sha256(prk, &[&block, info, &counter])
        };
        chunk.copy_from_slice(&block[..chunk.len()]);
    }
}

/// MAX(addr1, addr2) || MIN(addr1, addr2).
fn max_min(addr1: &[u8; 6], addr2: &[u8; 6]) -> [u8; 12] {
    let (max, min) = if addr1 > addr2 { (addr1, addr2) } else { (addr2, addr1) };
    let mut both = [0; 12];
    both[..6].copy_from_slice(max);
    both[6..].copy_from_slice(min);
    both
}

/// The PWE by hunting and pecking (IEEE 802.11-2020, 12.4.4.2.2): for counter = 1, 2, ...,
/// pwd-seed = HMAC-SHA256(MAX(addr) || MIN(addr), password || counter), and the first
/// pwd-value = KDF-256(pwd-seed, "SAE Hunting and Pecking", p) that is the x of a point gives the
/// PWE, with the y whose parity is that of pwd-seed. `None` if no counter up to 255 does, which has
/// a probability of about 2^-255.
pub(crate) fn pwe_hunting_and_pecking(password: &[u8], addr1: &[u8; 6], addr2: &[u8; 6]) -> Option<ProjectivePoint> {
    let key = max_min(addr1, addr2);
    let mut found = Choice::from(0);
    let mut pwe = AffinePoint::IDENTITY;
    let mut counter: u8 = 1;
    loop {
        let pwd_seed = hmac_sha256(&key, &[password, &[counter]]);
        let mut pwd_value = [0; LEN];
        kdf_sha256(&pwd_seed, b"SAE Hunting and Pecking", &PRIME, &mut pwd_value);
        // pwd-value < p, and x^3 + ax + b a quadratic residue: decompress answers both, and
        // picks the y with the parity of pwd-seed.
        let candidate = AffinePoint::decompress(&FieldBytes::from(pwd_value), Choice::from(pwd_seed[LEN - 1] & 1));
        let is_new = candidate.is_some() & !found;
        pwe = AffinePoint::conditional_select(&pwe, &candidate.unwrap_or(AffinePoint::IDENTITY), is_new);
        found |= candidate.is_some();
        if counter >= HUNTING_AND_PECKING_ROUNDS && bool::from(found) {
            return Some(pwe.into());
        }
        counter = counter.checked_add(1)?;
    }
}

/// The base of the PWE for hash to element (IEEE 802.11-2020, 12.4.4.2.3): it depends on the SSID
/// and the password only, so it is derived once. An optional password identifier goes into it too.
#[derive(Clone, Copy)]
pub struct Pt(AffinePoint);

impl Pt {
    /// pwd-seed = HKDF-Extract(SSID, password [|| identifier]), then two points by the simplified
    /// SWU map from u1 and u2 = HKDF-Expand(pwd-seed, "SAE Hash to Element u1 P1" or "... u2 P2",
    /// 48) modulo p, and PT is their sum.
    pub(crate) fn derive(ssid: &[u8], password: &[u8], identifier: Option<&[u8]>) -> Self {
        let pwd_seed = hmac_sha256(ssid, &[password, identifier.unwrap_or(&[])]);
        let point = |label: &[u8]| {
            let mut pwd_value = Array::<u8, _>::default();
            hkdf_expand(&pwd_seed, label, &mut pwd_value);
            let u = FieldElement::reduce(&pwd_value);
            NistP256::map_to_curve(u)
        };
        Self((point(b"SAE Hash to Element u1 P1") + point(b"SAE Hash to Element u2 P2")).into())
    }

    /// The PWE for the exchange between `addr1` and `addr2`: val = HKDF-Extract(0^32, MAX(addr)
    /// || MIN(addr)), reduced to 1 .. q - 1, times PT.
    pub(crate) fn pwe(&self, addr1: &[u8; 6], addr2: &[u8; 6]) -> ProjectivePoint {
        let val = U256::from_be_slice(&hmac_sha256(&[0; LEN], &[&max_min(addr1, addr2)]));
        let order_minus_1 = NistP256::ORDER.get().wrapping_sub(&U256::ONE);
        let val = val.rem(&NonZero::new(order_minus_1).unwrap()).wrapping_add(&U256::ONE);
        let val = Scalar::from_repr(val.to_be_byte_array()).unwrap();
        ProjectivePoint::from(self.0) * val
    }
}

/// Why a peer's message was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub(crate) enum Refusal {
    /// Another group than 19.
    UnsupportedGroup(u16),
    /// Cut short, a scalar out of range, or an element that is not on the curve.
    Malformed,
    /// The peer sent back our own scalar and element: a reflection attack.
    Reflected,
    /// Its confirm is not the one the shared key gives: the passwords differ.
    WrongConfirm,
}

/// One SAE exchange with an access point.
pub(crate) struct Sae {
    pwe: ProjectivePoint,
    rand: Scalar,
    scalar: Scalar,
    element: AffinePoint,
    peer: Option<(Scalar, AffinePoint)>,
    kck: [u8; LEN],
    pmk: [u8; LEN],
    #[cfg_attr(not(test), allow(dead_code))]
    pmkid: [u8; 16],
    send_confirm: u16,
}

/// 32 bytes as the curve crate takes them.
fn field_bytes(bytes: &[u8]) -> FieldBytes {
    let mut field = FieldBytes::default();
    field.copy_from_slice(bytes);
    field
}

/// A scalar encoded as SAE carries it: 32 bytes, big-endian.
fn scalar_bytes(scalar: &Scalar) -> [u8; LEN] {
    scalar.to_repr().into()
}

/// An element as SAE carries it: x then y, 32 bytes each, big-endian.
fn element_bytes(element: &AffinePoint) -> [u8; 2 * LEN] {
    let mut bytes = [0; 2 * LEN];
    bytes[..LEN].copy_from_slice(&element.x());
    bytes[LEN..].copy_from_slice(&element.y());
    bytes
}

impl Sae {
    /// Starts an exchange with the PWE, and `rand` and `mask`, two random scalars in 2 .. r - 1.
    /// `None` if commit-scalar = rand + mask modulo r is 0 or 1: draw again.
    pub(crate) fn new(pwe: ProjectivePoint, rand: Scalar, mask: Scalar) -> Option<Self> {
        let scalar = rand + mask;
        if bool::from(scalar.is_zero() | scalar.ct_eq(&Scalar::ONE)) {
            return None;
        }
        // COMMIT-ELEMENT = inverse(scalar-op(mask, PWE)).
        let element = (-(pwe * mask)).into();
        Some(Self {
            pwe,
            rand,
            scalar,
            element,
            peer: None,
            kck: [0; LEN],
            pmk: [0; LEN],
            pmkid: [0; 16],
            send_confirm: 0,
        })
    }

    /// A scalar in 2 .. r - 1 from 48 random bytes, reduced modulo r, or `None` if they give 0
    /// or 1: draw again.
    pub(crate) fn scalar_from(random: &[u8; 48]) -> Option<Scalar> {
        let wide: Array<u8, U48> = Array::from(*random);
        let scalar = Scalar::reduce(&wide);
        (!bool::from(scalar.is_zero() | scalar.ct_eq(&Scalar::ONE))).then_some(scalar)
    }

    /// Writes the commit message's body into `out`: the group, the anti-clogging token if the
    /// access point asked for one, the scalar and the element. Returns its length.
    pub(crate) fn write_commit(&self, token: &[u8], out: &mut [u8]) -> usize {
        let len = COMMIT_LEN + token.len();
        out[..2].copy_from_slice(&GROUP.to_le_bytes());
        out[2..2 + token.len()].copy_from_slice(token);
        let at = 2 + token.len();
        out[at..at + LEN].copy_from_slice(&scalar_bytes(&self.scalar));
        out[at + LEN..len].copy_from_slice(&element_bytes(&self.element));
        len
    }

    /// Takes the access point's commit message: checks its group, scalar and element, then
    /// derives the shared keys (IEEE 802.11-2020, 12.4.5.4).
    pub(crate) fn process_commit(&mut self, body: &[u8]) -> Result<(), Refusal> {
        let group = u16::from_le_bytes(body.get(..2).ok_or(Refusal::Malformed)?.try_into().unwrap());
        if group != GROUP {
            return Err(Refusal::UnsupportedGroup(group));
        }
        let fields = body.get(2..COMMIT_LEN).ok_or(Refusal::Malformed)?;
        // 1 < scalar < r.
        let scalar = Option::<Scalar>::from(Scalar::from_repr(field_bytes(&fields[..LEN])))
            .filter(|scalar| !bool::from(scalar.is_zero() | scalar.ct_eq(&Scalar::ONE)))
            .ok_or(Refusal::Malformed)?;
        // On the curve (which also means x and y below p), and not the identity.
        let element = Option::<AffinePoint>::from(AffinePoint::from_coordinates(
            &field_bytes(&fields[LEN..2 * LEN]),
            &field_bytes(&fields[2 * LEN..]),
        ))
        .filter(|element| !bool::from(element.is_identity()))
        .ok_or(Refusal::Malformed)?;
        if bool::from(scalar.ct_eq(&self.scalar)) && element == self.element {
            return Err(Refusal::Reflected);
        }

        // K = scalar-op(rand, elem-op(scalar-op(peer-commit-scalar, PWE), PEER-COMMIT-ELEMENT)),
        // and k its x.
        let k = AffinePoint::from((self.pwe * scalar + ProjectivePoint::from(element)) * self.rand);
        if bool::from(k.is_identity()) {
            return Err(Refusal::Malformed);
        }
        // keyseed = H(0^32, k); KCK || PMK = KDF-512(keyseed, "SAE KCK and PMK",
        // (commit-scalar + peer-commit-scalar) mod r); PMKID is that sum's first 16 bytes.
        let keyseed = hmac_sha256(&[0; LEN], &[&k.x()]);
        let context = scalar_bytes(&(self.scalar + scalar));
        let mut keys = [0; 2 * LEN];
        kdf_sha256(&keyseed, b"SAE KCK and PMK", &context, &mut keys);
        self.kck.copy_from_slice(&keys[..LEN]);
        self.pmk.copy_from_slice(&keys[LEN..]);
        self.pmkid.copy_from_slice(&context[..16]);
        self.peer = Some((scalar, element));
        Ok(())
    }

    /// CN(KCK, send-confirm, scalar1, element1, scalar2, element2).
    fn confirm(&self, send_confirm: u16, first: (&Scalar, &AffinePoint), second: (&Scalar, &AffinePoint)) -> [u8; LEN] {
        hmac_sha256(
            &self.kck,
            &[
                &send_confirm.to_le_bytes(),
                &scalar_bytes(first.0),
                &element_bytes(first.1),
                &scalar_bytes(second.0),
                &element_bytes(second.1),
            ],
        )
    }

    /// Writes the confirm message's body into `out`: send-confirm, then the confirm over our
    /// commit and the access point's. Returns its length. Only after [`Sae::process_commit`].
    pub(crate) fn write_confirm(&mut self, out: &mut [u8]) -> usize {
        let Some((peer_scalar, peer_element)) = self.peer else {
            return 0;
        };
        self.send_confirm = self.send_confirm.saturating_add(1);
        out[..2].copy_from_slice(&self.send_confirm.to_le_bytes());
        out[2..CONFIRM_LEN].copy_from_slice(&self.confirm(
            self.send_confirm,
            (&self.scalar, &self.element),
            (&peer_scalar, &peer_element),
        ));
        CONFIRM_LEN
    }

    /// Checks the access point's confirm message: its confirm is over its commit then ours.
    pub(crate) fn check_confirm(&self, body: &[u8]) -> Result<(), Refusal> {
        let (Some((peer_scalar, peer_element)), Some(received)) = (self.peer, body.get(..CONFIRM_LEN)) else {
            return Err(Refusal::Malformed);
        };
        let send_confirm = u16::from_le_bytes([received[0], received[1]]);
        let expected = self.confirm(
            send_confirm,
            (&peer_scalar, &peer_element),
            (&self.scalar, &self.element),
        );
        if bool::from(expected.ct_eq(&received[2..])) {
            Ok(())
        } else {
            Err(Refusal::WrongConfirm)
        }
    }

    /// The PMK of the 4-way handshake that follows, once the exchange is through.
    pub(crate) fn pmk(&self) -> [u8; LEN] {
        self.pmk
    }

    /// Names the PMK (for PMK caching, which the driver does not do).
    #[cfg(test)]
    pub(crate) fn pmkid(&self) -> [u8; 16] {
        self.pmkid
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::assert_eq;
    use std::vec::Vec;

    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        let s: Vec<u8> = s.bytes().filter(|b| !b.is_ascii_whitespace()).collect();
        s.chunks(2)
            .map(|pair| u8::from_str_radix(core::str::from_utf8(pair).unwrap(), 16).unwrap())
            .collect()
    }

    fn scalar(s: &str) -> Scalar {
        Scalar::from_repr(field_bytes(&hex(s))).unwrap()
    }

    // IEEE 802.11-2020, J.10: SAE test vectors for group 19, as hostapd's module tests use them.
    const ADDR1: [u8; 6] = [0x4d, 0x3f, 0x2f, 0xff, 0xe3, 0x87];
    const ADDR2: [u8; 6] = [0xa5, 0xd8, 0xaa, 0x95, 0x8e, 0x3c];
    const PASSWORD: &[u8] = b"mekmitasdigoat";

    #[test]
    fn hunting_and_pecking_matches_the_standard_test_vectors() {
        let pwe = pwe_hunting_and_pecking(PASSWORD, &ADDR1, &ADDR2).unwrap();
        let rand = scalar("992465fd3daa3c60aa6565b7f62a2a7f2e12dd12f198faf4fbed89d7ff1ace94");
        let mask = scalar("9507a90f777a044d6a0830b91ea3d5dd70bece44e1acffb86983b5e1bf9fb322");
        let mut sae = Sae::new(pwe, rand, mask).unwrap();

        let mut commit = [0; COMMIT_LEN];
        assert_eq!(sae.write_commit(&[], &mut commit), COMMIT_LEN);
        assert_eq!(
            commit.as_slice(),
            hex("1300 2e2c0f0db52440ad146d967114ce005ce1eab0aa2c2e5c2871b774f6c2575c65
                 d5ad9e00829707aa36ba8b859738fc961d08243505f47c035376d7ac4bc8d7b9
                 5083bf43827d0fc31ed778dd3671fd21a46d1091d64b6f9a1e1272621325dbe1")
        );

        let peer_commit = hex("1300 591b96f3397fb945100848e7b550543b6720d88337ee93fc49fd6df7e08b5223
             e71b9bb048d3873f20556953a96c91536fd8ee6ca9b4a68a148b056a909be03e
             83ae208f60f8ef5537858074db06687032399862999b511e0a1552a5fea317c2");
        assert_eq!(peer_commit.len(), COMMIT_LEN);
        sae.process_commit(&peer_commit).unwrap();
        assert_eq!(
            sae.kck.as_slice(),
            hex("1e733f6d9bd53256287304338831b09a39406d121017073a5c30db36f36cb81a")
        );
        assert_eq!(
            sae.pmk().as_slice(),
            hex("4e4dfab1a2dd8ac1a91790f953faaa452ae5c6873ab75b63605ba663f8a7fe59")
        );
        assert_eq!(sae.pmkid().as_slice(), hex("8747a600eea3f9f22475df58ca1e5498"));
    }

    #[test]
    fn hash_to_element_matches_the_standard_test_vectors() {
        let pt = Pt::derive(b"byteme", PASSWORD, Some(b"psk4internet"));
        let pwe = AffinePoint::from(pt.pwe(
            &[0x00, 0x09, 0x5b, 0x66, 0xec, 0x1e],
            &[0x00, 0x0b, 0x6b, 0xd9, 0x02, 0x46],
        ));
        assert_eq!(
            pwe.x().as_slice(),
            hex("c93049b9e64000f848201649e999f2b5c22dea69b5632c9df4d633b8aa1f6c1e")
        );
        assert_eq!(
            pwe.y().as_slice(),
            hex("73634e94b53d82e7383a8d258199d9dc1a5ee8269d060382ccbf33e614ff59a0")
        );
    }

    /// Two stations that share a password agree on the keys and accept each other's confirm.
    #[test]
    fn two_peers_agree() {
        let pwe = pwe_hunting_and_pecking(b"correct horse", &ADDR1, &ADDR2).unwrap();
        let random = |seed: u8| Sae::scalar_from(&[seed; 48]).unwrap();
        let mut a = Sae::new(pwe, random(1), random(2)).unwrap();
        let mut b = Sae::new(pwe, random(3), random(4)).unwrap();
        let (mut commit_a, mut commit_b) = ([0; COMMIT_LEN], [0; COMMIT_LEN]);
        a.write_commit(&[], &mut commit_a);
        b.write_commit(&[], &mut commit_b);
        a.process_commit(&commit_b).unwrap();
        b.process_commit(&commit_a).unwrap();
        assert_eq!(a.pmk(), b.pmk());
        assert_eq!(a.pmkid(), b.pmkid());

        let (mut confirm_a, mut confirm_b) = ([0; CONFIRM_LEN], [0; CONFIRM_LEN]);
        a.write_confirm(&mut confirm_a);
        b.write_confirm(&mut confirm_b);
        assert_eq!(a.check_confirm(&confirm_b), Ok(()));
        assert_eq!(b.check_confirm(&confirm_a), Ok(()));

        // Another password: the keys differ, and so does the confirm.
        let other = pwe_hunting_and_pecking(b"wrong horse", &ADDR1, &ADDR2).unwrap();
        let mut c = Sae::new(other, random(5), random(6)).unwrap();
        let mut commit_c = [0; COMMIT_LEN];
        c.write_commit(&[], &mut commit_c);
        let mut a = Sae::new(pwe, random(1), random(2)).unwrap();
        a.process_commit(&commit_c).unwrap();
        c.process_commit(&commit_a).unwrap();
        let mut confirm_c = [0; CONFIRM_LEN];
        c.write_confirm(&mut confirm_c);
        assert_eq!(a.check_confirm(&confirm_c), Err(Refusal::WrongConfirm));
    }

    #[test]
    fn bad_commits_are_refused() {
        let pwe = pwe_hunting_and_pecking(PASSWORD, &ADDR1, &ADDR2).unwrap();
        let random = |seed: u8| Sae::scalar_from(&[seed; 48]).unwrap();
        let mut sae = Sae::new(pwe, random(1), random(2)).unwrap();
        let mut commit = [0; COMMIT_LEN];
        sae.write_commit(&[], &mut commit);

        // Our own commit sent back.
        assert_eq!(sae.process_commit(&commit), Err(Refusal::Reflected));
        // Group 20.
        let mut other_group = commit;
        other_group[0] = 20;
        assert_eq!(sae.process_commit(&other_group), Err(Refusal::UnsupportedGroup(20)));
        // Cut short, a scalar of 1, a scalar of r, an element off the curve.
        assert_eq!(sae.process_commit(&commit[..COMMIT_LEN - 1]), Err(Refusal::Malformed));
        let mut one = commit;
        one[2..2 + LEN].copy_from_slice(&scalar_bytes(&Scalar::ONE));
        assert_eq!(sae.process_commit(&one), Err(Refusal::Malformed));
        let mut order = commit;
        order[2..2 + LEN].copy_from_slice(&NistP256::ORDER.get().to_be_byte_array());
        assert_eq!(sae.process_commit(&order), Err(Refusal::Malformed));
        let mut off_curve = commit;
        off_curve[COMMIT_LEN - 1] ^= 1;
        assert_eq!(sae.process_commit(&off_curve), Err(Refusal::Malformed));
    }
}
