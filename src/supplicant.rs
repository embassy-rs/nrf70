//! WPA2-Personal key management: the supplicant's side of the 4-way handshake and of the group
//! key handshake (IEEE 802.11-2020, 12.7), which the nRF70 firmware leaves to the host. The nRF
//! Connect SDK runs wpa_supplicant for this; the checks here follow its `wpa.c`.
//!
//! Only what a WPA2-Personal network with CCMP needs: the PSK key management suite (00-0F-AC:2),
//! or PSK-SHA256 (00-0F-AC:6) where the AP offers it with management frame protection, CCMP-128
//! as pairwise and group cipher, and management frame protection with BIP-CMAC-128 when the AP
//! offers or requires it. The RPU does the encryption itself: the host only derives the keys and
//! hands them over.
//!
//! Nothing here touches the RPU. [`Supplicant::handle`] takes an EAPOL frame and says what to
//! send back and which keys to install, so that it can be tested on its own.

use aes::Aes128;
use aes_kw::KwAes128;
use cmac::Cmac;
use defmt::debug;
use hmac::{Hmac, KeyInit, Mac};
use sha1::Sha1;
use sha2::Sha256;

type HmacSha1 = Hmac<Sha1>;
type HmacSha256 = Hmac<Sha256>;
type CmacAes128 = Cmac<Aes128>;

/// Ethertype of EAPOL frames (IEEE 802.1X).
pub(crate) const ETHERTYPE_EAPOL: u16 = 0x888E;

/// Element ID of the RSN element (RSNE).
pub(crate) const IE_RSN: u8 = 48;
/// Element ID of the RSN Extension element (RSNXE).
pub(crate) const IE_RSNXE: u8 = 244;
/// RSNXE capability: SAE hash to element (IEEE 802.11-2020, 9.4.2.241).
#[cfg(any(feature = "wpa3", test))]
const RSNXE_SAE_H2E: u8 = 1 << 5;

/// CCMP-128 as the RPU's key commands take a cipher suite: the selector's OUI then its type.
pub(crate) const CIPHER_SUITE_CCMP: u32 = 0x000F_AC04;
/// BIP-CMAC-128, the group management cipher, in the same form.
pub(crate) const CIPHER_SUITE_BIP_CMAC_128: u32 = 0x000F_AC06;

/// Suite selectors as they appear in an RSNE and in key data: OUI 00-0F-AC, then the type.
const SUITE_CCMP: [u8; 4] = [0x00, 0x0F, 0xAC, 4];
const AKM_PSK: [u8; 4] = [0x00, 0x0F, 0xAC, 2];
const AKM_PSK_SHA256: [u8; 4] = [0x00, 0x0F, 0xAC, 6];
const AKM_SAE: [u8; 4] = [0x00, 0x0F, 0xAC, 8];
const SUITE_BIP_CMAC_128: [u8; 4] = [0x00, 0x0F, 0xAC, 6];
/// The GTK key data encapsulation (KDE): OUI 00-0F-AC, data type 1.
const KDE_GTK: [u8; 4] = [0x00, 0x0F, 0xAC, 1];
/// The IGTK KDE: data type 9.
const KDE_IGTK: [u8; 4] = [0x00, 0x0F, 0xAC, 9];

/// RSN capabilities bit: management frame protection required.
const RSN_CAP_MFPR: u16 = 1 << 6;
/// RSN capabilities bit: management frame protection capable.
const RSN_CAP_MFPC: u16 = 1 << 7;

/// Length of a CCMP-128 temporal key.
const TK_LEN: usize = 16;

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

fn hmac_sha1(key: &[u8]) -> HmacSha1 {
    match HmacSha1::new_from_slice(key) {
        Ok(mac) => mac,
        // HMAC takes a key of any length.
        Err(_) => defmt::unreachable!(),
    }
}

fn hmac_sha256(key: &[u8]) -> HmacSha256 {
    match HmacSha256::new_from_slice(key) {
        Ok(mac) => mac,
        Err(_) => defmt::unreachable!(),
    }
}

/// The 802.11 key derivation function over HMAC-SHA256 (IEEE 802.11-2020, 12.7.1.6.2): fills
/// `out` with HMAC-SHA256(`key`, i || `label` || `context` || length) for i = 1, 2, and so on, i
/// and the length of `out` in bits as 16-bit little-endian numbers.
pub(crate) fn kdf_sha256(key: &[u8], label: &[u8], context: &[u8], out: &mut [u8]) {
    let bits = (out.len() * 8) as u16;
    for (i, chunk) in out.chunks_mut(32).enumerate() {
        let mut mac = hmac_sha256(key);
        mac.update(&(i as u16 + 1).to_le_bytes());
        mac.update(label);
        mac.update(context);
        mac.update(&bits.to_le_bytes());
        chunk.copy_from_slice(&mac.finalize().into_bytes()[..chunk.len()]);
    }
}

/// The key management suites the driver handles.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub(crate) enum Akm {
    /// PSK (00-0F-AC:2): keys from the PRF over HMAC-SHA1, HMAC-SHA1 MICs.
    Psk,
    /// PSK-SHA256 (00-0F-AC:6): keys from the KDF over HMAC-SHA256, AES-128-CMAC MICs.
    PskSha256,
    /// SAE (00-0F-AC:8), WPA3-Personal: the PMK comes from the SAE exchange, keys from the KDF
    /// over HMAC-SHA256, AES-128-CMAC MICs.
    Sae,
}

impl Akm {
    fn selector(self) -> [u8; 4] {
        match self {
            Akm::Psk => AKM_PSK,
            Akm::PskSha256 => AKM_PSK_SHA256,
            Akm::Sae => AKM_SAE,
        }
    }

    /// The key descriptor version of its EAPOL-Key frames, which says how their MIC is made.
    fn key_version(self) -> u16 {
        match self {
            Akm::Psk => INFO_VERSION_HMAC_SHA1_AES,
            Akm::PskSha256 => INFO_VERSION_AES_CMAC,
            Akm::Sae => INFO_VERSION_AKM_DEFINED,
        }
    }
}

/// What an association with an AP uses, as negotiated from its RSNE.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub(crate) struct Suite {
    pub(crate) akm: Akm,
    /// Management frame protection, with BIP-CMAC-128.
    pub(crate) mfp: bool,
}

/// The 802.11 pseudo-random function over HMAC-SHA1 (IEEE 802.11-2020, 12.7.1.2): fills `out`
/// with HMAC-SHA1(`key`, `label` || 0 || `data` || i) for i = 0, 1, and so on.
fn prf(key: &[u8], label: &[u8], data: &[u8], out: &mut [u8]) {
    for (i, chunk) in out.chunks_mut(20).enumerate() {
        let mut mac = hmac_sha1(key);
        mac.update(label);
        mac.update(&[0]);
        mac.update(data);
        mac.update(&[i as u8]);
        chunk.copy_from_slice(&mac.finalize().into_bytes()[..chunk.len()]);
    }
}

/// The pairwise transient key for CCMP: the EAPOL-Key confirmation and encryption keys, and the
/// temporal key.
#[derive(Clone)]
struct Ptk {
    kck: [u8; 16],
    kek: [u8; 16],
    tk: [u8; TK_LEN],
}

impl Ptk {
    /// PRF-384(PMK, "Pairwise key expansion", Min(AA, SPA) || Max(AA, SPA) || Min(ANonce, SNonce)
    /// || Max(ANonce, SNonce)) (IEEE 802.11-2020, 12.7.1.3), with the KDF over HMAC-SHA256 in
    /// place of the PRF for PSK-SHA256 and SAE. `aa` is the authenticator's address (the BSSID) and `spa`
    /// the supplicant's.
    fn derive(akm: Akm, pmk: &[u8; 32], aa: &[u8; 6], spa: &[u8; 6], anonce: &[u8; 32], snonce: &[u8; 32]) -> Self {
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

/// Longest RSN element the driver keeps, with its two-byte header. An AP offering several
/// pairwise ciphers and key management suites stays well under this.
const RSNE_MAX: usize = 64;

/// An RSN element, with its ID and length.
#[derive(Clone, Copy)]
pub(crate) struct Rsne {
    len: u8,
    bytes: [u8; RSNE_MAX],
}

impl Rsne {
    /// The element whose body (what follows the ID and length) is `body`, or `None` if it is
    /// longer than the driver keeps.
    pub(crate) fn from_body(body: &[u8]) -> Option<Self> {
        let mut bytes = [0; RSNE_MAX];
        bytes[0] = IE_RSN;
        bytes[1] = body.len() as u8;
        bytes.get_mut(2..2 + body.len())?.copy_from_slice(body);
        Some(Self {
            len: 2 + body.len() as u8,
            bytes,
        })
    }

    /// What the driver asks for in its association request and repeats in message 2: RSN version
    /// 1, CCMP-128 as group and pairwise cipher, the suite's key management, and management frame
    /// protection capable if the suite has it, and required with SAE. BIP-CMAC-128, the default
    /// group management cipher, goes without saying.
    pub(crate) fn for_suite(suite: Suite) -> Self {
        let mut body = [0; 20];
        body[0..2].copy_from_slice(&1u16.to_le_bytes());
        body[2..6].copy_from_slice(&SUITE_CCMP);
        body[6..8].copy_from_slice(&1u16.to_le_bytes());
        body[8..12].copy_from_slice(&SUITE_CCMP);
        body[12..14].copy_from_slice(&1u16.to_le_bytes());
        body[14..18].copy_from_slice(&suite.akm.selector());
        let capabilities = match (suite.akm, suite.mfp) {
            (Akm::Sae, _) => RSN_CAP_MFPC | RSN_CAP_MFPR,
            (_, true) => RSN_CAP_MFPC,
            (_, false) => 0,
        };
        body[18..20].copy_from_slice(&capabilities.to_le_bytes());
        match Self::from_body(&body) {
            Some(rsne) => rsne,
            None => defmt::unreachable!(),
        }
    }

    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }

    /// What an association with an AP announcing this element uses, if the driver can join it:
    /// CCMP-128 as the group cipher and among the pairwise ciphers, and with a pre-shared key, PSK
    /// or PSK-SHA256 key management; with `sae`, SAE. Management frame protection is used when
    /// the AP is capable of it with BIP-CMAC-128, and PSK-SHA256 then if the AP offers it; SAE
    /// requires it. An AP that requires management frame protection with another cipher cannot be
    /// joined.
    pub(crate) fn negotiate(&self, sae: bool) -> Option<Suite> {
        let body = &self.as_bytes()[2..];
        let u16_at = |at: usize| Some(u16::from_le_bytes(body.get(at..at + 2)?.try_into().ok()?));
        // A list of suite selectors: its count, then its entries.
        let list = |at: usize| {
            let count = u16_at(at)? as usize;
            Some((body.get(at + 2..at + 2 + 4 * count)?, at + 2 + 4 * count))
        };

        if u16_at(0)? != 1 {
            return None;
        }
        let group = body.get(2..6)?;
        let (pairwise, at) = list(6)?;
        let (akms, at) = list(at)?;
        // The capabilities are optional: without them, nothing is required.
        let capabilities = u16_at(at).unwrap_or(0);
        // Then the PMKIDs (16 bytes each) and the group management cipher, BIP-CMAC-128 when it
        // is not there.
        let group_management = u16_at(at + 2)
            .and_then(|pmkids| {
                let at = at + 4 + 16 * pmkids as usize;
                body.get(at..at + 4)
            })
            .unwrap_or(&SUITE_BIP_CMAC_128);
        if group != SUITE_CCMP || !pairwise.chunks(4).any(|suite| suite == SUITE_CCMP) {
            return None;
        }

        let mfp = capabilities & RSN_CAP_MFPC != 0 && group_management == SUITE_BIP_CMAC_128;
        if capabilities & RSN_CAP_MFPR != 0 && !mfp {
            return None;
        }
        let offers = |akm: [u8; 4]| akms.chunks(4).any(|suite| suite == akm);
        if sae {
            return (mfp && offers(AKM_SAE)).then_some(Suite { akm: Akm::Sae, mfp });
        }
        let akm = if mfp && offers(AKM_PSK_SHA256) {
            Akm::PskSha256
        } else if offers(AKM_PSK) {
            Akm::Psk
        } else {
            return None;
        };
        Some(Suite { akm, mfp })
    }
}

/// Longest RSNXE the driver keeps, with its header: its capabilities fit in a few bytes.
const RSNXE_MAX: usize = 18;

/// An RSN Extension element, with its ID and length.
#[derive(Clone, Copy)]
pub(crate) struct Rsnxe {
    len: u8,
    bytes: [u8; RSNXE_MAX],
}

impl Rsnxe {
    /// The element whose body is `body`, or `None` if it is longer than the driver keeps.
    pub(crate) fn from_body(body: &[u8]) -> Option<Self> {
        let mut bytes = [0; RSNXE_MAX];
        bytes[0] = IE_RSNXE;
        bytes[1] = body.len() as u8;
        bytes.get_mut(2..2 + body.len())?.copy_from_slice(body);
        Some(Self {
            len: 2 + body.len() as u8,
            bytes,
        })
    }

    /// What a station that uses SAE hash to element says in its association request.
    #[cfg(any(feature = "wpa3", test))]
    pub(crate) fn sae_h2e() -> Self {
        match Self::from_body(&[RSNXE_SAE_H2E]) {
            Some(rsnxe) => rsnxe,
            None => defmt::unreachable!(),
        }
    }

    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }

    /// Whether it announces SAE hash to element.
    #[cfg(any(feature = "wpa3", test))]
    pub(crate) fn offers_sae_h2e(&self) -> bool {
        self.as_bytes()
            .get(2)
            .is_some_and(|capabilities| capabilities & RSNXE_SAE_H2E != 0)
    }
}

// EAPOL-Key frames (IEEE 802.11-2020, 12.7.2), from the IEEE 802.1X header on.

/// IEEE 802.1X-2001, which wpa_supplicant sends too: some APs refuse a newer version.
const EAPOL_VERSION: u8 = 1;
const EAPOL_TYPE_KEY: u8 = 3;
/// Key descriptor type of RSN (WPA2). WPA's own, 254, is not supported.
const DESCRIPTOR_RSN: u8 = 2;

const OFFSET_KEY_INFO: usize = 5;
const OFFSET_REPLAY_COUNTER: usize = 9;
const OFFSET_NONCE: usize = 17;
const OFFSET_RSC: usize = 65;
const OFFSET_MIC: usize = 81;
const OFFSET_KEY_DATA_LEN: usize = 97;
/// Length of an EAPOL-Key frame without key data.
const KEY_FRAME_LEN: usize = 99;
const MIC_LEN: usize = 16;

/// Key descriptor version 2: HMAC-SHA1-128 as MIC, AES key wrap for the key data. It is the one
/// that goes with CCMP and PSK key management.
const INFO_VERSION_HMAC_SHA1_AES: u16 = 2;
/// Key descriptor version 3: AES-128-CMAC as MIC, AES key wrap for the key data, with PSK-SHA256.
const INFO_VERSION_AES_CMAC: u16 = 3;
/// Key descriptor version 0: the AKM says. With SAE (AKM 8): AES-128-CMAC as MIC, AES key wrap
/// for the key data, as with version 3 (HMAC-SHA256 is SAE-EXT-KEY's, AKM 24).
const INFO_VERSION_AKM_DEFINED: u16 = 0;
const INFO_VERSION_MASK: u16 = 0x0007;
const INFO_PAIRWISE: u16 = 1 << 3;
const INFO_INSTALL: u16 = 1 << 6;
const INFO_ACK: u16 = 1 << 7;
const INFO_MIC: u16 = 1 << 8;
const INFO_SECURE: u16 = 1 << 9;
const INFO_ERROR: u16 = 1 << 10;
const INFO_REQUEST: u16 = 1 << 11;
const INFO_ENCRYPTED_KEY_DATA: u16 = 1 << 12;

/// Longest reply: message 2, which carries the RSNE and the RSNXE.
pub(crate) const REPLY_MAX: usize = KEY_FRAME_LEN + RSNE_MAX + RSNXE_MAX;

/// Longest key data the driver unwraps. Message 3 carries the RSNE and the GTK, and sometimes a
/// second RSNE or keys the driver ignores.
const KEY_DATA_MAX: usize = 256;

/// A received EAPOL-Key frame.
struct KeyFrame<'a> {
    /// The frame from the 802.1X header to the end of the key data: what the MIC covers.
    pdu: &'a [u8],
    info: u16,
    replay_counter: u64,
    nonce: [u8; 32],
    /// The receive sequence counter of the GTK in the key data, low byte first.
    rsc: [u8; 6],
    key_data: &'a [u8],
}

impl<'a> KeyFrame<'a> {
    /// Parses an RSN EAPOL-Key frame. `frame` starts at the 802.1X header and may be followed by
    /// padding.
    fn parse(frame: &'a [u8]) -> Option<Self> {
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
    fn mic_is_valid(&self, kck: &[u8; 16]) -> bool {
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
fn mic(info: u16, kck: &[u8; 16], parts: &[&[u8]]) -> [u8; MIC_LEN] {
    let mut out = [0; MIC_LEN];
    if matches!(
        info & INFO_VERSION_MASK,
        INFO_VERSION_AES_CMAC | INFO_VERSION_AKM_DEFINED
    ) {
        let mut mac = match CmacAes128::new_from_slice(kck) {
            Ok(mac) => mac,
            Err(_) => defmt::unreachable!(),
        };
        for part in parts {
            mac.update(part);
        }
        out.copy_from_slice(&mac.finalize().into_bytes());
    } else {
        let mut mac = hmac_sha1(kck);
        for part in parts {
            mac.update(part);
        }
        out.copy_from_slice(&mac.finalize().into_bytes()[..MIC_LEN]);
    }
    out
}

/// Writes an EAPOL-Key frame with its MIC into `out`, and returns its length. The key length, IV,
/// RSC and reserved fields are zero, as in every frame a supplicant sends.
fn write_key_frame(
    out: &mut [u8],
    info: u16,
    replay_counter: u64,
    nonce: &[u8; 32],
    key_data: &[u8],
    kck: &[u8; 16],
) -> usize {
    let len = KEY_FRAME_LEN + key_data.len();
    let frame = &mut out[..len];
    frame.fill(0);
    frame[0] = EAPOL_VERSION;
    frame[1] = EAPOL_TYPE_KEY;
    frame[2..4].copy_from_slice(&((len - 4) as u16).to_be_bytes());
    frame[4] = DESCRIPTOR_RSN;
    frame[OFFSET_KEY_INFO..OFFSET_KEY_INFO + 2].copy_from_slice(&info.to_be_bytes());
    frame[OFFSET_REPLAY_COUNTER..OFFSET_NONCE].copy_from_slice(&replay_counter.to_be_bytes());
    frame[OFFSET_NONCE..OFFSET_NONCE + 32].copy_from_slice(nonce);
    frame[OFFSET_KEY_DATA_LEN..KEY_FRAME_LEN].copy_from_slice(&(key_data.len() as u16).to_be_bytes());
    frame[KEY_FRAME_LEN..].copy_from_slice(key_data);

    let mic = mic(info, kck, &[frame]);
    frame[OFFSET_MIC..OFFSET_MIC + MIC_LEN].copy_from_slice(&mic);
    len
}

/// A group temporal key, as the AP hands it over.
#[derive(Clone, PartialEq, Eq)]
pub(crate) struct Gtk {
    /// Key ID, 0 to 3.
    pub(crate) index: u8,
    pub(crate) key: [u8; TK_LEN],
    /// The packet number the AP has reached with this key, low byte first.
    pub(crate) rsc: [u8; 6],
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
struct KeyData<'a> {
    /// The first RSNE, with its header.
    rsne: Option<&'a [u8]>,
    /// The first RSNXE, with its header.
    rsnxe: Option<&'a [u8]>,
    /// The GTK KDE's body: key ID and flags, a reserved byte, then the key.
    gtk: Option<&'a [u8]>,
    /// The IGTK KDE's body: key ID, IPN, then the key.
    igtk: Option<&'a [u8]>,
}

impl<'a> KeyData<'a> {
    /// Walks the elements and KDEs of unwrapped key data (IEEE 802.11-2020, 12.7.2).
    fn parse(mut data: &'a [u8]) -> Self {
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

    fn gtk(&self, rsc: [u8; 6]) -> Option<Gtk> {
        let [flags, _reserved, key @ ..] = self.gtk? else {
            return None;
        };
        Some(Gtk {
            index: flags & 0x03,
            key: (*key).try_into().ok()?,
            rsc,
        })
    }

    fn igtk(&self) -> Option<Igtk> {
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

/// What to do with a received EAPOL frame.
pub(crate) enum Outcome {
    /// Nothing: the frame is not an RSN EAPOL-Key frame from the AP's side of a handshake, is a
    /// replay, or fails its MIC.
    Ignored,
    /// Send the first `usize` bytes of the reply buffer: message 2 of the 4-way handshake.
    Reply(usize),
    /// A handshake is complete. Send the first `reply` bytes of the reply buffer (message 4, or
    /// message 2 of the group key handshake), and only once that frame has left, install the keys:
    /// the reply has to go out under the keys in use until now.
    Keys {
        reply: usize,
        /// The pairwise temporal key, when it is a new one.
        tk: Option<[u8; TK_LEN]>,
        /// The group temporal key, when it is a new one.
        gtk: Option<Gtk>,
        /// The integrity group temporal key, when it is a new one: with management frame
        /// protection only.
        igtk: Option<Igtk>,
    },
    /// The AP's message 3 contradicts what it announced (a downgrade attack looks like this), or
    /// lacks a usable group key: leave the network.
    Abort,
}

/// The supplicant of one association.
pub(crate) struct Supplicant {
    pmk: [u8; 32],
    /// The authenticator's address: the BSSID.
    aa: [u8; 6],
    /// The supplicant's address: the station's.
    spa: [u8; 6],
    /// What the association uses.
    suite: Suite,
    /// The RSNE of the association request, which message 2 repeats.
    rsne: Rsne,
    /// The RSNE of the AP's beacon or probe response, which message 3 must repeat.
    ap_rsne: Rsne,
    /// The RSNXE of the association request, if any, which message 2 repeats after the RSNE.
    rsnxe: Option<Rsnxe>,
    /// The RSNXE of the AP's beacon or probe response, if any: message 3 must repeat it, and
    /// carry none if the AP announced none.
    ap_rsnxe: Option<Rsnxe>,

    nonce_seed: [u8; 32],
    nonce_counter: u64,
    snonce: [u8; 32],
    /// The next message 1 starts a new handshake and gets a new SNonce. A repeated message 1
    /// keeps the SNonce, so that the PTK does not change under a message 3 already on its way.
    renew_snonce: bool,
    anonce: [u8; 32],

    /// The PTK derived from message 1, until a MIC it verifies proves that the AP has it too.
    tptk: Option<Ptk>,
    ptk: Option<Ptk>,
    /// The TK of `ptk` has been handed over for installation. It is never handed over twice: a
    /// replayed message 3 must not reset the packet numbers of a key in use.
    tk_installed: bool,
    /// The last GTK handed over, for the same reason.
    gtk: Option<Gtk>,
    /// The last IGTK handed over.
    igtk: Option<Igtk>,
    /// Replay counter of the last frame whose MIC was valid.
    replay_counter: Option<u64>,
}

impl Supplicant {
    /// `seed` is what the SNonces are drawn from: 32 random bytes. With `sae`, the PMK is the
    /// one of an SAE exchange. `None` if the AP's RSNE offers nothing the driver can use.
    pub(crate) fn new(
        pmk: [u8; 32],
        aa: [u8; 6],
        spa: [u8; 6],
        ap_rsne: Rsne,
        seed: [u8; 32],
        sae: bool,
    ) -> Option<Self> {
        let suite = ap_rsne.negotiate(sae)?;
        Some(Self {
            pmk,
            aa,
            spa,
            suite,
            rsne: Rsne::for_suite(suite),
            ap_rsne,
            rsnxe: None,
            ap_rsnxe: None,
            nonce_seed: seed,
            nonce_counter: 0,
            snonce: [0; 32],
            renew_snonce: true,
            anonce: [0; 32],
            tptk: None,
            ptk: None,
            tk_installed: false,
            gtk: None,
            igtk: None,
            replay_counter: None,
        })
    }

    /// The RSNXEs of the association: the station's, which message 2 repeats, and the AP's,
    /// which message 3 must repeat.
    pub(crate) fn with_rsnxe(mut self, rsnxe: Option<Rsnxe>, ap_rsnxe: Option<Rsnxe>) -> Self {
        self.rsnxe = rsnxe;
        self.ap_rsnxe = ap_rsnxe;
        self
    }

    /// What the association uses.
    pub(crate) fn suite(&self) -> Suite {
        self.suite
    }

    /// The RSNE to put in the association request.
    pub(crate) fn rsne(&self) -> &Rsne {
        &self.rsne
    }

    /// Handles an EAPOL frame from the AP. `frame` starts at the 802.1X header. A reply, if any,
    /// is written at the start of `reply`.
    pub(crate) fn handle(&mut self, frame: &[u8], reply: &mut [u8; REPLY_MAX]) -> Outcome {
        let Some(key) = KeyFrame::parse(frame) else {
            debug!("EAPOL frame ignored: not an RSN EAPOL-Key frame");
            return Outcome::Ignored;
        };
        // Every frame of the authenticator asks for an answer, and none is a request or an error
        // report: those go the other way.
        if key.info & INFO_VERSION_MASK != self.suite.akm.key_version()
            || key.info & INFO_ACK == 0
            || key.info & (INFO_REQUEST | INFO_ERROR) != 0
        {
            debug!("EAPOL-Key frame ignored: key information {:04x}", key.info);
            return Outcome::Ignored;
        }
        if self.replay_counter.is_some_and(|last| key.replay_counter <= last) {
            debug!("EAPOL-Key frame ignored: replay counter {}", key.replay_counter);
            return Outcome::Ignored;
        }

        if key.info & INFO_MIC == 0 {
            return if key.info & INFO_PAIRWISE != 0 {
                self.message_1(&key, reply)
            } else {
                Outcome::Ignored
            };
        }

        // A MIC that the PTK of the last message 1 verifies proves the AP derived the same one.
        if self.tptk.as_ref().is_some_and(|tptk| key.mic_is_valid(&tptk.kck)) {
            self.ptk = self.tptk.take();
            self.tk_installed = false;
        } else if !self.ptk.as_ref().is_some_and(|ptk| key.mic_is_valid(&ptk.kck)) {
            debug!("EAPOL-Key frame ignored: invalid MIC");
            return Outcome::Ignored;
        }
        self.replay_counter = Some(key.replay_counter);

        if key.info & INFO_PAIRWISE != 0 {
            self.message_3(&key, reply)
        } else {
            self.group_message_1(&key, reply)
        }
    }

    /// Message 1 of the 4-way handshake brings the ANonce: derive the PTK, and answer with the
    /// SNonce and the RSNE in message 2.
    fn message_1(&mut self, key: &KeyFrame, reply: &mut [u8; REPLY_MAX]) -> Outcome {
        if self.renew_snonce {
            self.snonce = self.next_nonce();
            self.renew_snonce = false;
        }
        self.anonce = key.nonce;
        let tptk = Ptk::derive(
            self.suite.akm,
            &self.pmk,
            &self.aa,
            &self.spa,
            &self.anonce,
            &self.snonce,
        );
        // The key data: the RSNE and the RSNXE of the association request.
        let mut key_data = [0; RSNE_MAX + RSNXE_MAX];
        let rsne = self.rsne.as_bytes();
        let rsnxe = self.rsnxe.as_ref().map_or(&[][..], Rsnxe::as_bytes);
        key_data[..rsne.len()].copy_from_slice(rsne);
        key_data[rsne.len()..rsne.len() + rsnxe.len()].copy_from_slice(rsnxe);
        let len = write_key_frame(
            reply,
            self.suite.akm.key_version() | INFO_PAIRWISE | INFO_MIC,
            key.replay_counter,
            &self.snonce,
            &key_data[..rsne.len() + rsnxe.len()],
            &tptk.kck,
        );
        self.tptk = Some(tptk);
        debug!(
            "4-way handshake: message 1 (replay counter {}), sending message 2",
            key.replay_counter
        );
        Outcome::Reply(len)
    }

    /// Message 3 proves that the AP has the PTK, repeats its RSNE and brings the GTK: answer with
    /// message 4, then the keys go in.
    fn message_3(&mut self, key: &KeyFrame, reply: &mut [u8; REPLY_MAX]) -> Outcome {
        if key.nonce != self.anonce {
            debug!("4-way handshake: message 3 ignored, its ANonce is not message 1's");
            return Outcome::Ignored;
        }
        let Some(ptk) = self.ptk.clone() else {
            return Outcome::Ignored;
        };
        let mut buf = [0; KEY_DATA_MAX];
        let Some(data) = unwrap_key_data(key, &ptk.kek, &mut buf) else {
            debug!("4-way handshake: message 3 ignored, its key data does not unwrap");
            return Outcome::Ignored;
        };
        let data = KeyData::parse(data);
        if data.rsne != Some(self.ap_rsne.as_bytes()) {
            debug!("4-way handshake: the RSNE of message 3 is not the one the AP announced");
            return Outcome::Abort;
        }
        if data.rsnxe != self.ap_rsnxe.as_ref().map(Rsnxe::as_bytes) {
            debug!("4-way handshake: the RSNXE of message 3 is not the one the AP announced");
            return Outcome::Abort;
        }
        let Some(gtk) = data.gtk(key.rsc) else {
            debug!("4-way handshake: message 3 has no usable GTK");
            return Outcome::Abort;
        };
        let igtk = self.suite.mfp.then(|| data.igtk()).flatten();
        if self.suite.mfp && igtk.is_none() {
            debug!("4-way handshake: message 3 has no usable IGTK");
        }

        let len = write_key_frame(
            reply,
            self.suite.akm.key_version() | INFO_PAIRWISE | INFO_MIC | (key.info & INFO_SECURE),
            key.replay_counter,
            &[0; 32],
            &[],
            &ptk.kck,
        );
        // The SNonce has served: the next message 1 starts a new handshake.
        self.renew_snonce = true;
        let tk = (key.info & INFO_INSTALL != 0 && !self.tk_installed).then_some(ptk.tk);
        self.tk_installed |= tk.is_some();
        debug!(
            "4-way handshake: message 3 (replay counter {}), sending message 4",
            key.replay_counter
        );
        Outcome::Keys {
            reply: len,
            tk,
            gtk: self.new_gtk(gtk),
            igtk: igtk.and_then(|igtk| self.new_igtk(igtk)),
        }
    }

    /// Message 1 of the group key handshake brings a new GTK: answer with message 2.
    fn group_message_1(&mut self, key: &KeyFrame, reply: &mut [u8; REPLY_MAX]) -> Outcome {
        let Some(ptk) = self.ptk.clone() else {
            return Outcome::Ignored;
        };
        let mut buf = [0; KEY_DATA_MAX];
        let Some(data) = unwrap_key_data(key, &ptk.kek, &mut buf).map(KeyData::parse) else {
            debug!("group key handshake: message 1 ignored, its key data does not unwrap");
            return Outcome::Ignored;
        };
        let Some(gtk) = data.gtk(key.rsc) else {
            debug!("group key handshake: message 1 ignored, no usable GTK");
            return Outcome::Ignored;
        };
        let igtk = self.suite.mfp.then(|| data.igtk()).flatten();
        let len = write_key_frame(
            reply,
            self.suite.akm.key_version() | INFO_MIC | INFO_SECURE,
            key.replay_counter,
            &[0; 32],
            &[],
            &ptk.kck,
        );
        debug!("group key handshake: message 1, sending message 2");
        Outcome::Keys {
            reply: len,
            tk: None,
            gtk: self.new_gtk(gtk),
            igtk: igtk.and_then(|igtk| self.new_igtk(igtk)),
        }
    }

    /// `gtk` if it differs from the one in use, which it then replaces.
    fn new_gtk(&mut self, gtk: Gtk) -> Option<Gtk> {
        let current = self
            .gtk
            .as_ref()
            .is_some_and(|current| current.index == gtk.index && current.key == gtk.key);
        if current {
            return None;
        }
        self.gtk = Some(gtk.clone());
        Some(gtk)
    }

    /// `igtk` if it differs from the one in use, which it then replaces.
    fn new_igtk(&mut self, igtk: Igtk) -> Option<Igtk> {
        let current = self
            .igtk
            .as_ref()
            .is_some_and(|current| current.index == igtk.index && current.key == igtk.key);
        if current {
            return None;
        }
        self.igtk = Some(igtk.clone());
        Some(igtk)
    }

    /// A nonce that this supplicant has not used: PRF-256(seed, "Init Counter", address ||
    /// counter), after the construction of IEEE 802.11-2020, 12.7.5, with a counter in place of
    /// the time.
    fn next_nonce(&mut self) -> [u8; 32] {
        let mut data = [0; 14];
        data[0..6].copy_from_slice(&self.spa);
        data[6..14].copy_from_slice(&self.nonce_counter.to_be_bytes());
        self.nonce_counter += 1;
        let mut nonce = [0; 32];
        prf(&self.nonce_seed, b"Init Counter", &data, &mut nonce);
        nonce
    }
}

/// Unwraps the key data of `key` with the key encryption key (NIST AES key wrap, RFC 3394), which
/// also proves its integrity. `None` if the key data is not encrypted or does not unwrap.
fn unwrap_key_data<'a>(key: &KeyFrame, kek: &[u8; 16], buf: &'a mut [u8; KEY_DATA_MAX]) -> Option<&'a [u8]> {
    if key.info & INFO_ENCRYPTED_KEY_DATA == 0 {
        return None;
    }
    let kw = KwAes128::new_from_slice(kek).ok()?;
    kw.unwrap_key(key.key_data, buf).ok()
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::{assert, assert_eq};
    use std::vec::Vec;

    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        let s: Vec<u8> = s.bytes().filter(|b| !b.is_ascii_whitespace()).collect();
        s.chunks(2)
            .map(|pair| u8::from_str_radix(core::str::from_utf8(pair).unwrap(), 16).unwrap())
            .collect()
    }

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

    /// The RSNE of a WPA2-Personal AP: CCMP, PSK, 16 replay counters.
    const AP_RSNE: &str = "30 14 0100 000fac04 0100 000fac04 0100 000fac02 0c00";

    fn rsne(element: &str) -> Rsne {
        Rsne::from_body(&hex(element)[2..]).unwrap()
    }

    // A whole 4-way handshake computed outside this crate, with OpenSSL (`openssl kdf` for the
    // PSK, `openssl dgst -sha1 -mac HMAC` for the PRF and the MICs, `openssl enc
    // -id-aes128-wrap` for the key data of message 3): passphrase "nrf70-test-passphrase", SSID
    // "nrf70-wpa2", AA 02:00:00:00:00:01, SPA 02:00:00:00:00:02, ANonce 00 01 .. 1f, nonce seed
    // 32 times 5e, GTK 10 11 .. 1f with key ID 1 and RSC 2a 01 00 00 00 00, the AP's RSNE as
    // above. The frames below are what an AP sends and expects, byte for byte.
    #[test]
    fn handshake_matches_vectors_computed_with_openssl() {
        let psk = psk_from_passphrase(b"nrf70-test-passphrase", b"nrf70-wpa2").unwrap();
        assert_eq!(
            psk.as_slice(),
            hex("b856f849794c9b63a715cfd1ef15809adeb2aca6a49f8693a6fae8d678caa3ca")
        );
        let mut sta = Supplicant::new(
            psk,
            [2, 0, 0, 0, 0, 1],
            [2, 0, 0, 0, 0, 2],
            rsne(AP_RSNE),
            [0x5E; 32],
            false,
        )
        .unwrap();
        let mut reply = [0; REPLY_MAX];

        let message_1 = hex("0203005f02008a00100000000000000001
             000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f
             00000000000000000000000000000000 0000000000000000 0000000000000000
             00000000000000000000000000000000 0000");
        let Outcome::Reply(len) = sta.handle(&message_1, &mut reply) else {
            core::panic!("no message 2");
        };
        assert_eq!(
            &reply[..len],
            hex("0103007502010a00000000000000000001
                 3c0eb2aac9a32be8c80969dfe8090faae17974c16d88517404f3515054fb9e8f
                 00000000000000000000000000000000 0000000000000000 0000000000000000
                 056eaa0ff38fa67c584435748882ca96 0016
                 30140100000fac040100000fac040100000fac020000")
        );

        let message_3 = hex("020300970213ca00100000000000000002
             000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f
             00000000000000000000000000000000 2a01000000000000 0000000000000000
             432bbde140c111c82ab45560f8c1a61d 0038
             dc1e61b4588d9816247ae5c33c5fa2729353afcf6b2f4579927b02b251fb5b11
             c0faed4de8abcc5013b23ed8fdadfce1e8161b0cc5fa4eee");
        let Outcome::Keys {
            reply: len,
            tk,
            gtk,
            igtk,
        } = sta.handle(&message_3, &mut reply)
        else {
            core::panic!("no message 4");
        };
        assert!(igtk.is_none());
        assert_eq!(
            &reply[..len],
            hex("0103005f02030a00000000000000000002
                 0000000000000000000000000000000000000000000000000000000000000000
                 00000000000000000000000000000000 0000000000000000 0000000000000000
                 97e0039a4c0ccd40e9fb29a0ce07beca 0000")
        );
        assert_eq!(tk.unwrap().as_slice(), hex("486844c8d22bccc24053a97171b0279e"));
        let gtk = gtk.unwrap();
        assert_eq!(gtk.index, 1);
        assert_eq!(gtk.key.as_slice(), hex("101112131415161718191a1b1c1d1e1f"));
        assert_eq!(gtk.rsc, [0x2A, 1, 0, 0, 0, 0]);
    }

    const PSK: Suite = Suite {
        akm: Akm::Psk,
        mfp: false,
    };
    const PSK_MFP: Suite = Suite {
        akm: Akm::Psk,
        mfp: true,
    };
    const PSK_SHA256_MFP: Suite = Suite {
        akm: Akm::PskSha256,
        mfp: true,
    };

    const SAE: Suite = Suite {
        akm: Akm::Sae,
        mfp: true,
    };

    #[test]
    fn sae_is_negotiated_where_offered() {
        let negotiate = |element: &str| rsne(element).negotiate(true);
        // WPA3 only, and WPA2/WPA3 transition.
        assert_eq!(
            negotiate("30 14 0100 000fac04 0100 000fac04 0100 000fac08 cc00"),
            Some(SAE)
        );
        assert_eq!(
            negotiate("30 18 0100 000fac04 0100 000fac04 0200 000fac02 000fac08 8000"),
            Some(SAE)
        );
        // Not without management frame protection, nor where SAE is not offered.
        assert_eq!(negotiate("30 14 0100 000fac04 0100 000fac04 0100 000fac08 0000"), None);
        assert_eq!(negotiate("30 14 0100 000fac04 0100 000fac04 0100 000fac02 8000"), None);
        // And a pre-shared key does not join a WPA3-only network.
        assert_eq!(
            rsne("30 14 0100 000fac04 0100 000fac04 0100 000fac08 cc00").negotiate(false),
            None
        );
        assert_eq!(
            Rsne::for_suite(SAE).as_bytes(),
            hex("30 14 0100 000fac04 0100 000fac04 0100 000fac08 c000")
        );
    }

    // The 4-way handshake after SAE (key descriptor version 0: AES-128-CMAC MICs, as hostapd's
    // `wpa_eapol_key_mic` makes them for AKM 8), computed with OpenSSL as the one above, from the PMK of the SAE test vector of
    // IEEE 802.11-2020, J.10, and the same addresses, nonces and keys.
    #[test]
    fn sae_handshake_matches_vectors_computed_with_openssl() {
        let pmk: [u8; 32] = hex("4e4dfab1a2dd8ac1a91790f953faaa452ae5c6873ab75b63605ba663f8a7fe59")
            .try_into()
            .unwrap();
        let ap_rsne = rsne("30 14 0100 000fac04 0100 000fac04 0100 000fac08 cc00");
        let mut sta = Supplicant::new(pmk, [2, 0, 0, 0, 0, 1], [2, 0, 0, 0, 0, 2], ap_rsne, [0x5E; 32], true).unwrap();
        assert_eq!(sta.suite(), SAE);
        let mut reply = [0; REPLY_MAX];

        let message_1 = hex("0203005f02008800100000000000000001
             000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f
             00000000000000000000000000000000 0000000000000000 0000000000000000
             00000000000000000000000000000000 0000");
        let Outcome::Reply(len) = sta.handle(&message_1, &mut reply) else {
            core::panic!("no message 2");
        };
        assert_eq!(
            &reply[..len],
            hex("01030075020108000000000000000000013c0eb2aac9a32be8c80969dfe8090f
                 aae17974c16d88517404f3515054fb9e8f000000000000000000000000000000
                 0000000000000000000000000000000000cd2e7c6bd64b891381afce7a374ffb
                 e0001630140100000fac040100000fac040100000fac08c000")
        );

        let message_3 = hex("020300b70213c800100000000000000002000102030405060708090a0b0c0d0e
                 0f101112131415161718191a1b1c1d1e1f000000000000000000000000000000
                 002a0100000000000000000000000000002e1cdbd414bfd67a653140097b6e30
                 1900587e2c5119cc618e37191e5b8fdb820eca15888ebeda91865c34576dc009
                 fd4c4036dc5360f1f982472b829a88c0c82c55d63bf32b977131efec48567dab
                 050219b9cced4578f6b183ff15cb83c653eb5b6c24db4a699be9ed");
        let Outcome::Keys {
            reply: len,
            tk,
            gtk,
            igtk,
        } = sta.handle(&message_3, &mut reply)
        else {
            core::panic!("no message 4");
        };
        assert_eq!(
            &reply[..len],
            hex("0103005f02030800000000000000000002000000000000000000000000000000
                 0000000000000000000000000000000000000000000000000000000000000000
                 0000000000000000000000000000000000ca2d247cf9c1c29df7bdde82141a9f
                 330000")
        );
        assert_eq!(tk.unwrap().as_slice(), hex("aeb8aa4780e0bfd7497414469dd273a6"));
        assert_eq!(gtk.unwrap().index, 1);
        assert_eq!(igtk.unwrap().index, 4);
    }

    #[test]
    fn association_rsne_follows_the_suite() {
        assert_eq!(
            Rsne::for_suite(PSK).as_bytes(),
            hex("30 14 0100 000fac04 0100 000fac04 0100 000fac02 0000")
        );
        assert_eq!(
            Rsne::for_suite(PSK_MFP).as_bytes(),
            hex("30 14 0100 000fac04 0100 000fac04 0100 000fac02 8000")
        );
        assert_eq!(
            Rsne::for_suite(PSK_SHA256_MFP).as_bytes(),
            hex("30 14 0100 000fac04 0100 000fac04 0100 000fac06 8000")
        );
    }

    #[test]
    fn the_suite_is_negotiated_from_the_aps_rsne() {
        let negotiate = |element: &str| rsne(element).negotiate(false);
        assert_eq!(negotiate(AP_RSNE), Some(PSK));
        // CCMP among several pairwise ciphers.
        assert_eq!(
            negotiate("30 18 0100 000fac04 0200 000fac02 000fac04 0100 000fac02 0000"),
            Some(PSK)
        );
        // No capabilities field.
        assert_eq!(negotiate("30 12 0100 000fac04 0100 000fac04 0100 000fac02"), Some(PSK));

        // Management frame protection capable or required: used, with PSK-SHA256 if offered.
        assert_eq!(
            negotiate("30 14 0100 000fac04 0100 000fac04 0100 000fac02 8000"),
            Some(PSK_MFP)
        );
        assert_eq!(
            negotiate("30 14 0100 000fac04 0100 000fac04 0100 000fac02 c000"),
            Some(PSK_MFP)
        );
        assert_eq!(
            negotiate("30 18 0100 000fac04 0100 000fac04 0200 000fac02 000fac06 8000"),
            Some(PSK_SHA256_MFP)
        );
        assert_eq!(
            negotiate("30 14 0100 000fac04 0100 000fac04 0100 000fac06 cc00"),
            Some(PSK_SHA256_MFP)
        );
        // WPA2/WPA3 transition: PSK and SAE, management frame protection capable.
        assert_eq!(
            negotiate("30 18 0100 000fac04 0100 000fac04 0200 000fac02 000fac08 8000"),
            Some(PSK_MFP)
        );
        // BIP-CMAC-128 named, after an empty PMKID list.
        assert_eq!(
            negotiate("30 1a 0100 000fac04 0100 000fac04 0100 000fac02 8000 0000 000fac06"),
            Some(PSK_MFP)
        );
        // Another group management cipher (BIP-GMAC-256): without protection if it is optional,
        // not at all if it is required.
        assert_eq!(
            negotiate("30 1a 0100 000fac04 0100 000fac04 0100 000fac02 8000 0000 000fac0c"),
            Some(PSK)
        );
        assert_eq!(
            negotiate("30 1a 0100 000fac04 0100 000fac04 0100 000fac02 c000 0000 000fac0c"),
            None
        );
        // PSK-SHA256 without management frame protection is not used.
        assert_eq!(negotiate("30 14 0100 000fac04 0100 000fac04 0100 000fac06 0000"), None);

        // WPA3 only: SAE.
        assert_eq!(negotiate("30 14 0100 000fac04 0100 000fac04 0100 000fac08 c000"), None);
        // WPA/WPA2 mixed mode: TKIP as group cipher.
        assert_eq!(negotiate("30 14 0100 000fac02 0100 000fac04 0100 000fac02 0000"), None);
        // WPA2-Enterprise: 802.1X.
        assert_eq!(negotiate("30 14 0100 000fac04 0100 000fac04 0100 000fac01 0000"), None);
        // Another version, and elements cut short.
        assert_eq!(negotiate("30 14 0200 000fac04 0100 000fac04 0100 000fac02 0000"), None);
        assert_eq!(negotiate("30 0e 0100 000fac04 0100 000fac04 0100"), None);
        assert_eq!(negotiate("30 02 0100"), None);
    }

    // A PSK-SHA256 4-way handshake with management frame protection, computed outside this crate
    // with OpenSSL (`openssl mac HMAC` with SHA-256 for the KDF, `openssl mac CMAC` with
    // AES-128-CBC for the MICs, `openssl enc -id-aes128-wrap` for the key data), from the same
    // PMK, addresses, ANonce and nonce seed as the test above. The AP's RSNE requires management
    // frame protection; message 3 carries the GTK 10 11 .. 1f with key ID 1, and the IGTK 20 21
    // .. 2f with key ID 4 and IPN 3.
    #[test]
    fn psk_sha256_handshake_matches_vectors_computed_with_openssl() {
        let psk = psk_from_passphrase(b"nrf70-test-passphrase", b"nrf70-wpa2").unwrap();
        let ap_rsne = rsne("30 14 0100 000fac04 0100 000fac04 0100 000fac06 cc00");
        let mut sta = Supplicant::new(psk, [2, 0, 0, 0, 0, 1], [2, 0, 0, 0, 0, 2], ap_rsne, [0x5E; 32], false).unwrap();
        assert_eq!(sta.suite(), PSK_SHA256_MFP);
        let mut reply = [0; REPLY_MAX];

        let message_1 = hex("0203005f02008b00100000000000000001
             000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f
             00000000000000000000000000000000 0000000000000000 0000000000000000
             00000000000000000000000000000000 0000");
        let Outcome::Reply(len) = sta.handle(&message_1, &mut reply) else {
            core::panic!("no message 2");
        };
        assert_eq!(
            &reply[..len],
            hex("0103007502010b00000000000000000001
                 3c0eb2aac9a32be8c80969dfe8090faae17974c16d88517404f3515054fb9e8f
                 00000000000000000000000000000000 0000000000000000 0000000000000000
                 8d46ca6fbe91cdc686d2f151e1b10ee6 0016
                 30140100000fac040100000fac040100000fac068000")
        );

        let message_3 = hex("020300b70213cb00100000000000000002
             000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f
             00000000000000000000000000000000 2a01000000000000 0000000000000000
             fe443de843324f4d1d0a2149cfa6efd6 0058
             cc89ea94b80fa324ae68a57e86653d7d05f0092a6af547b4f99fad25cb8ab5de
             ce57626890f09e0c26d5719ff3569727f9de18015135d0fb79d560e763c5fe7b
             20f4a87644aa4db30bfbf70efc3465c88e3e2ca22a3d15f1");
        let Outcome::Keys {
            reply: len,
            tk,
            gtk,
            igtk,
        } = sta.handle(&message_3, &mut reply)
        else {
            core::panic!("no message 4");
        };
        assert_eq!(
            &reply[..len],
            hex("0103005f02030b00000000000000000002
                 0000000000000000000000000000000000000000000000000000000000000000
                 00000000000000000000000000000000 0000000000000000 0000000000000000
                 06c8a3713108ab7abb6fb6da7fe0a75d 0000")
        );
        assert_eq!(tk.unwrap().as_slice(), hex("54c0f30d8bad6e2b38b9d5df22d1c632"));
        let gtk = gtk.unwrap();
        assert_eq!(
            (gtk.index, gtk.key.as_slice()),
            (1, hex("101112131415161718191a1b1c1d1e1f").as_slice())
        );
        let igtk = igtk.unwrap();
        assert_eq!(igtk.index, 4);
        assert_eq!(igtk.key.as_slice(), hex("202122232425262728292a2b2c2d2e2f"));
        assert_eq!(igtk.ipn, [3, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn rsne_longer_than_kept_is_refused() {
        assert!(Rsne::from_body(&[0; RSNE_MAX - 2]).is_some());
        assert!(Rsne::from_body(&[0; RSNE_MAX - 1]).is_none());
    }

    const AA: [u8; 6] = [0x02, 0xAA, 0, 0, 0, 1];
    const SPA: [u8; 6] = [0xF4, 0xCE, 0x36, 0, 0, 2];
    const ANONCE: [u8; 32] = [0xA5; 32];
    const GTK: [u8; 16] = [0x61; 16];
    const RSC: [u8; 6] = [9, 8, 7, 6, 5, 4];

    const IGTK: [u8; 16] = [0x71; 16];
    const IPN: [u8; 6] = [1, 2, 3, 0, 0, 0];

    /// The authenticator's side, as far as the tests need it.
    struct Ap {
        pmk: [u8; 32],
        rsne: Vec<u8>,
        suite: Suite,
        replay_counter: u64,
        /// From the SNonce of message 2.
        ptk: Option<Ptk>,
    }

    impl Ap {
        fn new() -> Self {
            Self::with_rsne(AP_RSNE)
        }

        fn with_rsne(element: &str) -> Self {
            Self {
                pmk: psk_from_passphrase(b"correct horse", b"net").unwrap(),
                rsne: hex(element),
                suite: rsne(element).negotiate(false).unwrap(),
                replay_counter: 0,
                ptk: None,
            }
        }

        fn version(&self) -> u16 {
            self.suite.akm.key_version()
        }

        /// An EAPOL-Key frame with the next replay counter, and a MIC if `info` says so.
        fn frame(&mut self, info: u16, nonce: &[u8; 32], key_data: &[u8]) -> Vec<u8> {
            self.replay_counter += 1;
            let len = KEY_FRAME_LEN + key_data.len();
            let mut frame = std::vec![0; len];
            frame[0] = 2;
            frame[1] = EAPOL_TYPE_KEY;
            frame[2..4].copy_from_slice(&((len - 4) as u16).to_be_bytes());
            frame[4] = DESCRIPTOR_RSN;
            frame[OFFSET_KEY_INFO..OFFSET_KEY_INFO + 2].copy_from_slice(&info.to_be_bytes());
            frame[7..9].copy_from_slice(&16u16.to_be_bytes());
            frame[OFFSET_REPLAY_COUNTER..OFFSET_NONCE].copy_from_slice(&self.replay_counter.to_be_bytes());
            frame[OFFSET_NONCE..OFFSET_NONCE + 32].copy_from_slice(nonce);
            frame[OFFSET_RSC..OFFSET_RSC + 6].copy_from_slice(&RSC);
            frame[OFFSET_KEY_DATA_LEN..KEY_FRAME_LEN].copy_from_slice(&(key_data.len() as u16).to_be_bytes());
            frame[KEY_FRAME_LEN..].copy_from_slice(key_data);
            if info & INFO_MIC != 0 {
                let mic = mic(info, &self.ptk.as_ref().unwrap().kck, &[&frame]);
                frame[OFFSET_MIC..OFFSET_MIC + MIC_LEN].copy_from_slice(&mic);
            }
            frame
        }

        /// Key data padded and wrapped with the KEK.
        fn wrap(&self, mut data: Vec<u8>) -> Vec<u8> {
            if !data.len().is_multiple_of(8) {
                data.push(0xDD);
                data.resize(data.len().next_multiple_of(8), 0);
            }
            let kw = KwAes128::new_from_slice(&self.ptk.as_ref().unwrap().kek).unwrap();
            let mut wrapped = std::vec![0; data.len() + 8];
            kw.wrap_key(&data, &mut wrapped).unwrap();
            wrapped
        }

        fn gtk_kde(gtk: &[u8], index: u8) -> Vec<u8> {
            let mut kde = std::vec![0xDD, (6 + gtk.len()) as u8];
            kde.extend_from_slice(&KDE_GTK);
            kde.extend_from_slice(&[index, 0]);
            kde.extend_from_slice(gtk);
            kde
        }

        fn igtk_kde(igtk: &[u8], index: u16) -> Vec<u8> {
            let mut kde = std::vec![0xDD, (12 + igtk.len()) as u8];
            kde.extend_from_slice(&KDE_IGTK);
            kde.extend_from_slice(&index.to_le_bytes());
            kde.extend_from_slice(&IPN);
            kde.extend_from_slice(igtk);
            kde
        }

        fn message_1(&mut self) -> Vec<u8> {
            self.frame(self.version() | INFO_PAIRWISE | INFO_ACK, &ANONCE, &[])
        }

        /// Takes message 2: derives the PTK from its SNonce and checks its MIC.
        fn take_message_2(&mut self, frame: &[u8]) -> KeyFrame<'static> {
            let frame: &'static [u8] = frame.to_vec().leak();
            let key = KeyFrame::parse(frame).unwrap();
            let ptk = Ptk::derive(self.suite.akm, &self.pmk, &AA, &SPA, &ANONCE, &key.nonce);
            assert!(key.mic_is_valid(&ptk.kck));
            self.ptk = Some(ptk);
            key
        }

        fn message_3_with(&mut self, rsne: &[u8], gtk: &[u8]) -> Vec<u8> {
            let mut data = rsne.to_vec();
            data.extend_from_slice(&Self::gtk_kde(gtk, 1));
            if self.suite.mfp {
                data.extend_from_slice(&Self::igtk_kde(&IGTK, 4));
            }
            let data = self.wrap(data);
            self.frame(
                self.version()
                    | INFO_PAIRWISE
                    | INFO_INSTALL
                    | INFO_ACK
                    | INFO_MIC
                    | INFO_SECURE
                    | INFO_ENCRYPTED_KEY_DATA,
                &ANONCE,
                &data,
            )
        }

        fn message_3(&mut self) -> Vec<u8> {
            let rsne = self.rsne.clone();
            self.message_3_with(&rsne, &GTK)
        }

        fn group_message_1(&mut self, gtk: &[u8], index: u8) -> Vec<u8> {
            let mut data = Self::gtk_kde(gtk, index);
            if self.suite.mfp {
                data.extend_from_slice(&Self::igtk_kde(&[index + 0x70; 16], 4 + (index as u16 & 1)));
            }
            let data = self.wrap(data);
            self.frame(
                self.version() | INFO_ACK | INFO_MIC | INFO_SECURE | INFO_ENCRYPTED_KEY_DATA,
                &[0; 32],
                &data,
            )
        }
    }

    fn supplicant(passphrase: &[u8]) -> Supplicant {
        supplicant_for(passphrase, AP_RSNE)
    }

    fn supplicant_for(passphrase: &[u8], ap_rsne: &str) -> Supplicant {
        Supplicant::new(
            psk_from_passphrase(passphrase, b"net").unwrap(),
            AA,
            SPA,
            rsne(ap_rsne),
            [0x5E; 32],
            false,
        )
        .unwrap()
    }

    /// Runs the 4-way handshake to its end.
    fn connected() -> (Ap, Supplicant) {
        let mut ap = Ap::new();
        let mut sta = supplicant(b"correct horse");
        let mut reply = [0; REPLY_MAX];
        let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
            core::panic!("no message 2");
        };
        ap.take_message_2(&reply[..len]);
        let Outcome::Keys { .. } = sta.handle(&ap.message_3(), &mut reply) else {
            core::panic!("no message 4");
        };
        (ap, sta)
    }

    #[test]
    fn four_way_handshake_yields_both_keys() {
        let mut ap = Ap::new();
        let mut sta = supplicant(b"correct horse");
        let mut reply = [0; REPLY_MAX];

        let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
            core::panic!("no message 2");
        };
        let message_2 = ap.take_message_2(&reply[..len]);
        assert_eq!(reply[0], EAPOL_VERSION);
        assert_eq!(message_2.info, INFO_VERSION_HMAC_SHA1_AES | INFO_PAIRWISE | INFO_MIC);
        assert_eq!(message_2.replay_counter, 1);
        assert_eq!(message_2.key_data, Rsne::for_suite(PSK).as_bytes());

        let Outcome::Keys {
            reply: len,
            tk,
            gtk,
            igtk,
        } = sta.handle(&ap.message_3(), &mut reply)
        else {
            core::panic!("no message 4");
        };
        let ptk = ap.ptk.as_ref().unwrap();
        let message_4 = KeyFrame::parse(&reply[..len]).unwrap();
        assert!(message_4.mic_is_valid(&ptk.kck));
        assert_eq!(
            message_4.info,
            INFO_VERSION_HMAC_SHA1_AES | INFO_PAIRWISE | INFO_MIC | INFO_SECURE
        );
        assert_eq!(message_4.replay_counter, 2);
        assert_eq!(message_4.nonce, [0; 32]);
        assert!(message_4.key_data.is_empty());
        assert_eq!(tk, Some(ptk.tk));
        assert!(
            gtk == Some(Gtk {
                index: 1,
                key: GTK,
                rsc: RSC
            })
        );
        assert!(igtk.is_none());
    }

    #[test]
    fn psk_sha256_handshake_yields_the_igtk_and_rekeys_it() {
        for element in [
            "30 14 0100 000fac04 0100 000fac04 0100 000fac06 cc00",
            "30 14 0100 000fac04 0100 000fac04 0100 000fac02 8000",
        ] {
            let mut ap = Ap::with_rsne(element);
            let mut sta = supplicant_for(b"correct horse", element);
            let mut reply = [0; REPLY_MAX];
            let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
                core::panic!("no message 2");
            };
            let message_2 = ap.take_message_2(&reply[..len]);
            assert_eq!(message_2.info, ap.version() | INFO_PAIRWISE | INFO_MIC);
            assert_eq!(message_2.key_data, Rsne::for_suite(ap.suite).as_bytes());

            let Outcome::Keys {
                reply: len,
                tk,
                gtk,
                igtk,
            } = sta.handle(&ap.message_3(), &mut reply)
            else {
                core::panic!("no message 4");
            };
            assert!(KeyFrame::parse(&reply[..len])
                .unwrap()
                .mic_is_valid(&ap.ptk.as_ref().unwrap().kck));
            assert_eq!(tk, Some(ap.ptk.as_ref().unwrap().tk));
            assert!(gtk.is_some());
            assert!(
                igtk == Some(Igtk {
                    index: 4,
                    key: IGTK,
                    ipn: IPN
                })
            );

            // A group rekey brings a new GTK and a new IGTK, once.
            let Outcome::Keys { igtk, .. } = sta.handle(&ap.group_message_1(&[0x62; 16], 1), &mut reply) else {
                core::panic!("no group message 2");
            };
            assert!(
                igtk == Some(Igtk {
                    index: 5,
                    key: [0x71; 16],
                    ipn: IPN
                })
            );
            let Outcome::Keys { gtk, igtk, .. } = sta.handle(&ap.group_message_1(&[0x62; 16], 1), &mut reply) else {
                core::panic!("no group message 2");
            };
            assert!(gtk.is_none() && igtk.is_none());
        }
    }

    #[test]
    fn wrong_passphrase_never_gets_past_message_2() {
        let mut ap = Ap::new();
        let mut sta = supplicant(b"wrong horse");
        let mut reply = [0; REPLY_MAX];
        let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
            core::panic!("no message 2");
        };
        // The AP cannot verify message 2. Should it send message 3 anyway, its MIC is refused.
        let key = KeyFrame::parse(&reply[..len]).unwrap();
        let ap_ptk = Ptk::derive(Akm::Psk, &ap.pmk, &AA, &SPA, &ANONCE, &key.nonce);
        assert!(!key.mic_is_valid(&ap_ptk.kck));
        ap.ptk = Some(ap_ptk);
        assert!(matches!(sta.handle(&ap.message_3(), &mut reply), Outcome::Ignored));
    }

    #[test]
    fn repeated_message_1_keeps_the_snonce_and_a_new_handshake_renews_it() {
        let mut ap = Ap::new();
        let mut sta = supplicant(b"correct horse");
        let mut reply = [0; REPLY_MAX];
        let mut snonce = |sta: &mut Supplicant, ap: &mut Ap| {
            let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
                core::panic!("no message 2");
            };
            ap.take_message_2(&reply[..len]).nonce
        };
        let first = snonce(&mut sta, &mut ap);
        assert_eq!(snonce(&mut sta, &mut ap), first);
        assert!(first != [0; 32]);

        let mut reply = [0; REPLY_MAX];
        assert!(matches!(sta.handle(&ap.message_3(), &mut reply), Outcome::Keys { .. }));
        let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
            core::panic!("no message 2");
        };
        assert!(ap.take_message_2(&reply[..len]).nonce != first);
    }

    #[test]
    fn replayed_frames_are_ignored() {
        let mut ap = Ap::new();
        let mut sta = supplicant(b"correct horse");
        let mut reply = [0; REPLY_MAX];
        let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
            core::panic!("no message 2");
        };
        ap.take_message_2(&reply[..len]);
        let message_3 = ap.message_3();
        assert!(matches!(sta.handle(&message_3, &mut reply), Outcome::Keys { .. }));
        // The same frame again, then a message 1 with a replay counter already seen.
        assert!(matches!(sta.handle(&message_3, &mut reply), Outcome::Ignored));
        ap.replay_counter = 0;
        assert!(matches!(sta.handle(&ap.message_1(), &mut reply), Outcome::Ignored));
    }

    #[test]
    fn retransmitted_message_3_is_answered_without_handing_the_keys_over_again() {
        let (mut ap, mut sta) = connected();
        let mut reply = [0; REPLY_MAX];
        // The AP did not get message 4: it sends message 3 again, with the next replay counter.
        let Outcome::Keys {
            reply: len, tk, gtk, ..
        } = sta.handle(&ap.message_3(), &mut reply)
        else {
            core::panic!("no message 4");
        };
        assert!(KeyFrame::parse(&reply[..len])
            .unwrap()
            .mic_is_valid(&ap.ptk.as_ref().unwrap().kck));
        assert!(tk.is_none());
        assert!(gtk.is_none());
    }

    #[test]
    fn message_3_with_another_rsne_aborts() {
        let mut ap = Ap::new();
        let mut sta = supplicant(b"correct horse");
        let mut reply = [0; REPLY_MAX];
        let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
            core::panic!("no message 2");
        };
        ap.take_message_2(&reply[..len]);
        // The beacon announced CCMP; message 3 says TKIP.
        let downgraded = hex("30 14 0100 000fac02 0100 000fac02 0100 000fac02 0c00");
        let message_3 = ap.message_3_with(&downgraded, &GTK);
        assert!(matches!(sta.handle(&message_3, &mut reply), Outcome::Abort));
    }

    #[test]
    fn the_rsnxes_are_repeated_and_checked() {
        let rsnxe = Rsnxe::sae_h2e();
        assert_eq!(rsnxe.as_bytes(), hex("f4 01 20"));
        assert!(rsnxe.offers_sae_h2e());
        assert!(!Rsnxe::from_body(&[0]).unwrap().offers_sae_h2e());
        assert!(!Rsnxe::from_body(&[]).unwrap().offers_sae_h2e());

        // Message 2 repeats the station's RSNXE after its RSNE.
        let mut ap = Ap::new();
        let mut sta = supplicant(b"correct horse").with_rsnxe(Some(rsnxe), Some(rsnxe));
        let mut reply = [0; REPLY_MAX];
        let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
            core::panic!("no message 2");
        };
        let message_2 = ap.take_message_2(&reply[..len]);
        assert_eq!(
            message_2.key_data,
            [Rsne::for_suite(PSK).as_bytes(), rsnxe.as_bytes()].concat()
        );

        // Message 3 has to repeat the AP's RSNXE: without it, or with another, the station leaves.
        let rsne = ap.rsne.clone();
        let message_3 = ap.message_3_with(&rsne, &GTK);
        assert!(matches!(sta.handle(&message_3, &mut reply), Outcome::Abort));
        let other = [rsne.as_slice(), &hex("f4 01 00")].concat();
        let message_3 = ap.message_3_with(&other, &GTK);
        assert!(matches!(sta.handle(&message_3, &mut reply), Outcome::Abort));
        let same = [rsne.as_slice(), rsnxe.as_bytes()].concat();
        let message_3 = ap.message_3_with(&same, &GTK);
        assert!(matches!(sta.handle(&message_3, &mut reply), Outcome::Keys { .. }));

        // An AP that announced none must send none.
        let mut ap = Ap::new();
        let mut sta = supplicant(b"correct horse");
        let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
            core::panic!("no message 2");
        };
        ap.take_message_2(&reply[..len]);
        let message_3 = ap.message_3_with(&same, &GTK);
        assert!(matches!(sta.handle(&message_3, &mut reply), Outcome::Abort));
    }

    #[test]
    fn message_3_without_a_ccmp_gtk_aborts() {
        let mut ap = Ap::new();
        let mut sta = supplicant(b"correct horse");
        let mut reply = [0; REPLY_MAX];
        let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
            core::panic!("no message 2");
        };
        ap.take_message_2(&reply[..len]);
        // A 32-byte GTK is TKIP's.
        let rsne = ap.rsne.clone();
        let message_3 = ap.message_3_with(&rsne, &[0x61; 32]);
        assert!(matches!(sta.handle(&message_3, &mut reply), Outcome::Abort));
    }

    #[test]
    fn message_3_with_another_anonce_or_a_bad_mic_is_ignored() {
        let mut ap = Ap::new();
        let mut sta = supplicant(b"correct horse");
        let mut reply = [0; REPLY_MAX];
        let Outcome::Reply(len) = sta.handle(&ap.message_1(), &mut reply) else {
            core::panic!("no message 2");
        };
        ap.take_message_2(&reply[..len]);

        let mut tampered = ap.message_3();
        *tampered.last_mut().unwrap() ^= 1;
        assert!(matches!(sta.handle(&tampered, &mut reply), Outcome::Ignored));

        // A valid MIC over another ANonce.
        let data = ap.wrap(ap.rsne.clone());
        let other = ap.frame(
            ap.version() | INFO_PAIRWISE | INFO_INSTALL | INFO_ACK | INFO_MIC | INFO_SECURE | INFO_ENCRYPTED_KEY_DATA,
            &[0x11; 32],
            &data,
        );
        assert!(matches!(sta.handle(&other, &mut reply), Outcome::Ignored));
    }

    #[test]
    fn group_key_handshake_brings_a_new_gtk() {
        let (mut ap, mut sta) = connected();
        let mut reply = [0; REPLY_MAX];
        let new_gtk = [0x62; 16];
        let Outcome::Keys {
            reply: len, tk, gtk, ..
        } = sta.handle(&ap.group_message_1(&new_gtk, 2), &mut reply)
        else {
            core::panic!("no group message 2");
        };
        let message_2 = KeyFrame::parse(&reply[..len]).unwrap();
        assert!(message_2.mic_is_valid(&ap.ptk.as_ref().unwrap().kck));
        assert_eq!(message_2.info, INFO_VERSION_HMAC_SHA1_AES | INFO_MIC | INFO_SECURE);
        assert_eq!(message_2.replay_counter, ap.replay_counter);
        assert!(tk.is_none());
        assert!(
            gtk == Some(Gtk {
                index: 2,
                key: new_gtk,
                rsc: RSC
            })
        );

        // The same GTK again is acknowledged, not handed over again.
        let Outcome::Keys { gtk, .. } = sta.handle(&ap.group_message_1(&new_gtk, 2), &mut reply) else {
            core::panic!("no group message 2");
        };
        assert!(gtk.is_none());
    }

    #[test]
    fn frames_that_are_not_the_authenticators_are_ignored() {
        let (mut ap, mut sta) = connected();
        let mut reply = [0; REPLY_MAX];
        let ok = INFO_VERSION_HMAC_SHA1_AES | INFO_PAIRWISE | INFO_ACK;

        // Another descriptor version, no ACK, a request, and WPA's own descriptor.
        for info in [1 | INFO_PAIRWISE | INFO_ACK, ok & !INFO_ACK, ok | INFO_REQUEST] {
            let frame = ap.frame(info, &ANONCE, &[]);
            assert!(matches!(sta.handle(&frame, &mut reply), Outcome::Ignored));
        }
        let mut wpa1 = ap.frame(ok, &ANONCE, &[]);
        wpa1[4] = 254;
        assert!(matches!(sta.handle(&wpa1, &mut reply), Outcome::Ignored));

        // Cut short, another EAPOL type, and a key data length beyond the frame.
        let frame = ap.frame(ok, &ANONCE, &[]);
        assert!(matches!(
            sta.handle(&frame[..KEY_FRAME_LEN - 1], &mut reply),
            Outcome::Ignored
        ));
        let mut start = frame.clone();
        start[1] = 1;
        assert!(matches!(sta.handle(&start, &mut reply), Outcome::Ignored));
        let mut long = frame.clone();
        long[OFFSET_KEY_DATA_LEN + 1] = 8;
        assert!(matches!(sta.handle(&long, &mut reply), Outcome::Ignored));

        // Padding after the frame, as Ethernet adds it, does not matter.
        let mut padded = ap.message_1();
        padded.extend_from_slice(&[0; 20]);
        assert!(matches!(sta.handle(&padded, &mut reply), Outcome::Reply(_)));
    }
}
