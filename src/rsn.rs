//! RSN elements and cipher suites (IEEE 802.11-2020, 9.4.2.24 and 9.4.2.241): what a network
//! offers, what a station asks for, and the suite an association settles on.

use crate::crypto::TK_LEN;
use crate::eapol::{INFO_VERSION_AES_CMAC, INFO_VERSION_AKM_DEFINED, INFO_VERSION_HMAC_SHA1_AES};
use crate::ieee80211::{IE_RSN, IE_RSNXE};

/// RSNXE capability: SAE hash to element (IEEE 802.11-2020, 9.4.2.241).
#[cfg(any(feature = "wpa3", test))]
const RSNXE_SAE_H2E: u8 = 1 << 5;

/// CCMP-128 as the RPU's key commands take a cipher suite: the selector's OUI then its type.
pub(crate) const CIPHER_SUITE_CCMP: u32 = 0x000F_AC04;
/// TKIP, in the same form.
const CIPHER_SUITE_TKIP: u32 = 0x000F_AC02;
/// BIP-CMAC-128, the group management cipher, in the same form.
pub(crate) const CIPHER_SUITE_BIP_CMAC_128: u32 = 0x000F_AC06;

/// Suite selectors as they appear in an RSNE and in key data: OUI 00-0F-AC, then the type.
const SUITE_CCMP: [u8; 4] = [0x00, 0x0F, 0xAC, 4];
const SUITE_TKIP: [u8; 4] = [0x00, 0x0F, 0xAC, 2];
const AKM_PSK: [u8; 4] = [0x00, 0x0F, 0xAC, 2];
const AKM_PSK_SHA256: [u8; 4] = [0x00, 0x0F, 0xAC, 6];
const AKM_SAE: [u8; 4] = [0x00, 0x0F, 0xAC, 8];
const SUITE_BIP_CMAC_128: [u8; 4] = [0x00, 0x0F, 0xAC, 6];

/// RSN capabilities bit: management frame protection required.
const RSN_CAP_MFPR: u16 = 1 << 6;
/// RSN capabilities bit: management frame protection capable.
const RSN_CAP_MFPC: u16 = 1 << 7;

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
    pub(crate) fn key_version(self) -> u16 {
        match self {
            Akm::Psk => INFO_VERSION_HMAC_SHA1_AES,
            Akm::PskSha256 => INFO_VERSION_AES_CMAC,
            Akm::Sae => INFO_VERSION_AKM_DEFINED,
        }
    }
}

/// The group cipher of a network. The pairwise cipher is always CCMP-128.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub(crate) enum GroupCipher {
    Ccmp,
    /// TKIP, which WPA/WPA2 mixed mode networks keep for their WPA stations.
    Tkip,
}

impl GroupCipher {
    fn selector(self) -> [u8; 4] {
        match self {
            GroupCipher::Ccmp => SUITE_CCMP,
            GroupCipher::Tkip => SUITE_TKIP,
        }
    }

    /// As the RPU's key commands take it.
    pub(crate) fn cipher_suite(self) -> u32 {
        match self {
            GroupCipher::Ccmp => CIPHER_SUITE_CCMP,
            GroupCipher::Tkip => CIPHER_SUITE_TKIP,
        }
    }

    /// The length of its group key: TKIP's carries two Michael MIC keys after its temporal key.
    pub(crate) fn key_len(self) -> usize {
        match self {
            GroupCipher::Ccmp => TK_LEN,
            GroupCipher::Tkip => TK_LEN + 16,
        }
    }
}

/// What an association with an AP uses, as negotiated from its RSNE.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub(crate) struct Suite {
    pub(crate) akm: Akm,
    /// Management frame protection, with BIP-CMAC-128.
    pub(crate) mfp: bool,
    pub(crate) group: GroupCipher,
}

/// Longest RSN element the driver keeps, with its two-byte header. An AP offering several
/// pairwise ciphers and key management suites stays well under this.
pub(crate) const RSNE_MAX: usize = 64;

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
        body[2..6].copy_from_slice(&suite.group.selector());
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

    /// The element of [`Self::for_suite`], which ends with the capabilities, naming `pmkid` after
    /// them: the association request of a station that uses the PMK of an earlier SAE exchange.
    #[cfg(feature = "wpa3")]
    pub(crate) fn with_pmkid(&self, pmkid: &[u8; 16]) -> Self {
        let mut rsne = *self;
        let at = self.len as usize;
        rsne.bytes[at..at + 2].copy_from_slice(&1u16.to_le_bytes());
        rsne.bytes[at + 2..at + 18].copy_from_slice(pmkid);
        rsne.len += 18;
        rsne.bytes[1] += 18;
        rsne
    }

    /// The fields of the element, if it is an RSN version 1 one with a group cipher and lists of
    /// pairwise ciphers and key management suites.
    fn fields(&self) -> Option<RsneFields<'_>> {
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
        let pmkids = u16_at(at + 2).and_then(|count| body.get(at + 4..at + 4 + 16 * count as usize));
        let group_management = pmkids
            .and_then(|pmkids| body.get(at + 4 + pmkids.len()..at + 8 + pmkids.len()))
            .unwrap_or(&SUITE_BIP_CMAC_128);
        Some(RsneFields {
            group,
            pairwise,
            akms,
            capabilities,
            pmkids: pmkids.unwrap_or(&[]),
            group_management,
        })
    }

    /// What an association with an AP announcing this element uses, if the driver can join it:
    /// CCMP-128 as the group cipher and among the pairwise ciphers, and with a pre-shared key, PSK
    /// or PSK-SHA256 key management; with `sae`, SAE. Management frame protection is used when
    /// the AP is capable of it with BIP-CMAC-128, and PSK-SHA256 then if the AP offers it; SAE
    /// requires it. An AP that requires management frame protection with another cipher cannot be
    /// joined.
    pub(crate) fn negotiate(&self, sae: bool) -> Option<Suite> {
        let RsneFields {
            group,
            pairwise,
            akms,
            capabilities,
            group_management,
            ..
        } = self.fields()?;
        let group = match group {
            selector if selector == SUITE_CCMP => GroupCipher::Ccmp,
            selector if selector == SUITE_TKIP => GroupCipher::Tkip,
            _ => return None,
        };
        if !pairwise.chunks(4).any(|suite| suite == SUITE_CCMP) {
            return None;
        }

        // Management frame protection does not go with a TKIP group key, nor does SAE.
        let mfp =
            capabilities & RSN_CAP_MFPC != 0 && group_management == SUITE_BIP_CMAC_128 && group == GroupCipher::Ccmp;
        if capabilities & RSN_CAP_MFPR != 0 && !mfp {
            return None;
        }
        let offers = |akm: [u8; 4]| akms.chunks(4).any(|suite| suite == akm);
        if sae {
            return (mfp && offers(AKM_SAE)).then_some(Suite {
                akm: Akm::Sae,
                mfp,
                group,
            });
        }
        let akm = if mfp && offers(AKM_PSK_SHA256) {
            Akm::PskSha256
        } else if offers(AKM_PSK) {
            Akm::Psk
        } else {
            return None;
        };
        Some(Suite { akm, mfp, group })
    }

    /// The RSNE of an access point of the driver: CCMP-128 as group and pairwise cipher, PSK and
    /// PSK-SHA256 key management if `offer` has PSK, SAE if it has SAE, and management frame
    /// protection with BIP-CMAC-128 (the default group management cipher, which goes without
    /// saying): capable, and required with SAE alone (WPA3-Personal).
    #[cfg(feature = "ap")]
    pub(crate) fn access_point(offer: Offer) -> Self {
        let mut body = [0; 28];
        body[0..2].copy_from_slice(&1u16.to_le_bytes());
        body[2..6].copy_from_slice(&SUITE_CCMP);
        body[6..8].copy_from_slice(&1u16.to_le_bytes());
        body[8..12].copy_from_slice(&SUITE_CCMP);
        let mut akms = 0;
        for (offered, akm) in [(offer.psk, AKM_PSK), (offer.psk, AKM_PSK_SHA256), (offer.sae, AKM_SAE)] {
            if offered {
                body[14 + 4 * akms..18 + 4 * akms].copy_from_slice(&akm);
                akms += 1;
            }
        }
        body[12..14].copy_from_slice(&(akms as u16).to_le_bytes());
        let at = 14 + 4 * akms;
        let capabilities = if offer.sae && !offer.psk {
            RSN_CAP_MFPC | RSN_CAP_MFPR
        } else {
            RSN_CAP_MFPC
        };
        body[at..at + 2].copy_from_slice(&capabilities.to_le_bytes());
        match Self::from_body(&body[..at + 2]) {
            Some(rsne) => rsne,
            None => defmt::unreachable!(),
        }
    }

    /// Checks the RSNE of a station's association request against what [`Self::access_point`]
    /// offers, and returns what the station chose: SAE if it names it, else PSK-SHA256, else PSK,
    /// and management frame protection if it is capable of it, which SAE requires. Returns the
    /// IEEE 802.11 status code to refuse it with otherwise, as hostapd's `wpa_validate_wpa_ie`
    /// chooses it.
    #[cfg(feature = "ap")]
    pub(crate) fn check_station(&self, offer: Offer) -> Result<Suite, u16> {
        const INVALID_ELEMENT: u16 = 40;
        const INVALID_GROUP_CIPHER: u16 = 41;
        const INVALID_PAIRWISE_CIPHER: u16 = 42;
        const INVALID_AKMP: u16 = 43;
        const CIPHER_REJECTED_PER_POLICY: u16 = 46;
        const ROBUST_MANAGEMENT_POLICY_VIOLATION: u16 = 31;
        let fields = self.fields().ok_or(INVALID_ELEMENT)?;
        let names = |list: &[u8], suite: [u8; 4]| list.chunks(4).any(|s| s == suite);
        if fields.group != SUITE_CCMP {
            return Err(INVALID_GROUP_CIPHER);
        }
        if !names(fields.pairwise, SUITE_CCMP) {
            return Err(INVALID_PAIRWISE_CIPHER);
        }
        let akm = if offer.sae && names(fields.akms, AKM_SAE) {
            Akm::Sae
        } else if offer.psk && names(fields.akms, AKM_PSK_SHA256) {
            Akm::PskSha256
        } else if offer.psk && names(fields.akms, AKM_PSK) {
            Akm::Psk
        } else {
            return Err(INVALID_AKMP);
        };
        let mfp = fields.capabilities & RSN_CAP_MFPC != 0;
        if !mfp && (akm == Akm::Sae || !offer.psk) {
            return Err(ROBUST_MANAGEMENT_POLICY_VIOLATION);
        }
        if mfp && fields.group_management != SUITE_BIP_CMAC_128 {
            return Err(CIPHER_REJECTED_PER_POLICY);
        }
        Ok(Suite {
            akm,
            mfp,
            group: GroupCipher::Ccmp,
        })
    }

    /// Whether the element names `pmkid` among its PMKIDs: a station that asks to use a PMK
    /// cached from an earlier SAE exchange names it in its association request.
    #[cfg(all(feature = "ap", feature = "wpa3"))]
    pub(crate) fn names_pmkid(&self, pmkid: &[u8; 16]) -> bool {
        self.fields()
            .is_some_and(|fields| fields.pmkids.chunks(16).any(|named| named == pmkid))
    }
}

/// The key management an access point of the driver offers.
#[cfg(feature = "ap")]
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct Offer {
    /// PSK and PSK-SHA256: WPA2-Personal.
    pub(crate) psk: bool,
    /// SAE: WPA3-Personal.
    pub(crate) sae: bool,
}

/// The fields of an RSN element (IEEE 802.11-2020, 9.4.2.24), from the group cipher on.
struct RsneFields<'a> {
    group: &'a [u8],
    /// The pairwise cipher suite selectors, 4 bytes each.
    pairwise: &'a [u8],
    /// The key management suite selectors, 4 bytes each.
    akms: &'a [u8],
    capabilities: u16,
    /// The PMKIDs, 16 bytes each.
    #[cfg_attr(not(all(feature = "ap", feature = "wpa3")), allow(dead_code))]
    pmkids: &'a [u8],
    group_management: &'a [u8],
}

/// Longest RSNXE the driver keeps, with its header: its capabilities fit in a few bytes.
pub(crate) const RSNXE_MAX: usize = 18;

/// Writes `rsne`, then `rsnxe` if there is one, into `out`, as association requests and the key
/// data of messages 2 and 3 carry them. Returns their length.
pub(crate) fn write_elements(out: &mut [u8], rsne: &Rsne, rsnxe: Option<&Rsnxe>) -> usize {
    let rsne = rsne.as_bytes();
    let rsnxe = rsnxe.map_or(&[][..], Rsnxe::as_bytes);
    out[..rsne.len()].copy_from_slice(rsne);
    out[rsne.len()..rsne.len() + rsnxe.len()].copy_from_slice(rsnxe);
    rsne.len() + rsnxe.len()
}

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

#[cfg(test)]
pub(crate) mod tests {
    use core::{assert, assert_eq};

    use super::*;
    use crate::tests::hex;

    /// The RSNE of a WPA2-Personal AP: CCMP, PSK, 16 replay counters.
    pub(crate) const AP_RSNE: &str = "30 14 0100 000fac04 0100 000fac04 0100 000fac02 0c00";

    pub(crate) fn rsne(element: &str) -> Rsne {
        Rsne::from_body(&hex(element)[2..]).unwrap()
    }

    pub(crate) const PSK: Suite = Suite {
        akm: Akm::Psk,
        mfp: false,
        group: GroupCipher::Ccmp,
    };
    const PSK_MFP: Suite = Suite {
        akm: Akm::Psk,
        mfp: true,
        group: GroupCipher::Ccmp,
    };
    pub(crate) const PSK_SHA256_MFP: Suite = Suite {
        akm: Akm::PskSha256,
        mfp: true,
        group: GroupCipher::Ccmp,
    };

    pub(crate) const SAE: Suite = Suite {
        akm: Akm::Sae,
        mfp: true,
        group: GroupCipher::Ccmp,
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
        // WPA/WPA2 mixed mode: TKIP as group cipher, with or without an offer of protection (it
        // does not go with TKIP), but not if protection is required.
        let mixed = Suite {
            group: GroupCipher::Tkip,
            ..PSK
        };
        assert_eq!(
            negotiate("30 14 0100 000fac02 0100 000fac04 0100 000fac02 0000"),
            Some(mixed)
        );
        assert_eq!(
            negotiate("30 14 0100 000fac02 0100 000fac04 0100 000fac02 8000"),
            Some(mixed)
        );
        assert_eq!(negotiate("30 14 0100 000fac02 0100 000fac04 0100 000fac02 c000"), None);
        // TKIP as pairwise cipher only.
        assert_eq!(negotiate("30 14 0100 000fac02 0100 000fac02 0100 000fac02 0000"), None);
        // WPA2-Enterprise: 802.1X.
        assert_eq!(negotiate("30 14 0100 000fac04 0100 000fac04 0100 000fac01 0000"), None);
        // Another version, and elements cut short.
        assert_eq!(negotiate("30 14 0200 000fac04 0100 000fac04 0100 000fac02 0000"), None);
        assert_eq!(negotiate("30 0e 0100 000fac04 0100 000fac04 0100"), None);
        assert_eq!(negotiate("30 02 0100"), None);
    }

    #[test]
    fn rsne_longer_than_kept_is_refused() {
        assert!(Rsne::from_body(&[0; RSNE_MAX - 2]).is_some());
        assert!(Rsne::from_body(&[0; RSNE_MAX - 1]).is_none());
    }
}
