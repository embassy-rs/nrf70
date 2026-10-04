//! The IEEE 802.11 vocabulary the station and the access point share: element IDs, status and
//! reason codes, capability bits, and the walk over the elements of a frame body (IEEE
//! 802.11-2020, 9.4).

// Each feature uses its part: a build with all of them uses everything.
#![cfg_attr(not(all(feature = "ap", feature = "wpa3")), allow(dead_code))]

/// Element IDs (9.4.2.1).
pub(crate) const IE_SSID: u8 = 0;
pub(crate) const IE_RATES: u8 = 1;
pub(crate) const IE_DS_PARAMS: u8 = 3;
pub(crate) const IE_ERP: u8 = 42;
pub(crate) const IE_HT_CAPABILITIES: u8 = 45;
pub(crate) const IE_RSN: u8 = 48;
pub(crate) const IE_EXT_RATES: u8 = 50;
pub(crate) const IE_TIMEOUT_INTERVAL: u8 = 56;
pub(crate) const IE_HT_OPERATION: u8 = 61;
pub(crate) const IE_VENDOR: u8 = 221;
pub(crate) const IE_RSNXE: u8 = 244;
/// Element ID Extension: the element's first byte is its extension ID.
pub(crate) const IE_EXTENSION: u8 = 255;

/// Element ID extensions.
pub(crate) const IE_EXT_PASSWORD_IDENTIFIER: u8 = 33;
pub(crate) const IE_EXT_ANTI_CLOGGING_TOKEN: u8 = 93;

/// Capability information bits (9.4.1.4).
pub(crate) const CAPABILITY_ESS: u16 = 0x0001;
/// Data frames are encrypted.
pub(crate) const CAPABILITY_PRIVACY: u16 = 0x0010;
pub(crate) const CAPABILITY_SHORT_PREAMBLE: u16 = 0x0020;
pub(crate) const CAPABILITY_SHORT_SLOT: u16 = 0x0400;

/// The authentication algorithm number of SAE (9.4.1.1).
pub(crate) const AUTH_ALGORITHM_SAE: u16 = 3;

/// Status codes (9.4.1.9).
pub(crate) const STATUS_SUCCESS: u16 = 0;
pub(crate) const STATUS_UNSPECIFIED: u16 = 1;
pub(crate) const STATUS_AUTH_ALGORITHM: u16 = 13;
pub(crate) const STATUS_AUTH_SEQUENCE: u16 = 14;
/// "Challenge failure": for SAE, the peer found the confirm wrong, the passwords differ.
pub(crate) const STATUS_CHALLENGE_FAILURE: u16 = 15;
pub(crate) const STATUS_TOO_MANY_STATIONS: u16 = 17;
pub(crate) const STATUS_RATES: u16 = 18;
pub(crate) const STATUS_TRY_LATER: u16 = 30;
pub(crate) const STATUS_INVALID_ELEMENT: u16 = 40;
pub(crate) const STATUS_INVALID_PMKID: u16 = 53;
pub(crate) const STATUS_ANTI_CLOGGING_TOKEN_REQUIRED: u16 = 76;
pub(crate) const STATUS_UNSUPPORTED_GROUP: u16 = 77;
/// The access point knows no password under the identifier of the commit.
pub(crate) const STATUS_UNKNOWN_PASSWORD_IDENTIFIER: u16 = 123;
/// An SAE commit with hash to element.
pub(crate) const STATUS_SAE_HASH_TO_ELEMENT: u16 = 126;

/// Reason codes (9.4.1.7).
pub(crate) const REASON_PREV_AUTH_NOT_VALID: u16 = 2;
pub(crate) const REASON_LEAVING: u16 = 3;
pub(crate) const REASON_INACTIVITY: u16 = 4;
pub(crate) const REASON_CLASS2_FROM_UNAUTHENTICATED: u16 = 6;
pub(crate) const REASON_4WAY_HANDSHAKE_TIMEOUT: u16 = 15;
pub(crate) const REASON_GROUP_KEY_UPDATE_TIMEOUT: u16 = 16;
pub(crate) const REASON_IE_IN_4WAY_DIFFERS: u16 = 17;

/// The elements of `body`, as their ID and body, up to its end or to the first truncated one.
pub(crate) fn elements(mut body: &[u8]) -> impl Iterator<Item = (u8, &[u8])> {
    core::iter::from_fn(move || {
        let [id, len, rest @ ..] = body else {
            return None;
        };
        let element = rest.get(..*len as usize)?;
        body = &rest[element.len()..];
        Some((*id, element))
    })
}

/// The body of the first element `id` in `body`.
pub(crate) fn find_ie(body: &[u8], id: u8) -> Option<&[u8]> {
    elements(body).find_map(|(i, element)| (i == id).then_some(element))
}

/// The body of the first extension element `extension` in `body`, after its extension ID.
pub(crate) fn find_extension(body: &[u8], extension: u8) -> Option<&[u8]> {
    elements(body).find_map(|(id, element)| match element {
        [ext, rest @ ..] if id == IE_EXTENSION && *ext == extension => Some(rest),
        _ => None,
    })
}

#[cfg(test)]
mod tests {
    use core::assert_eq;

    use super::*;

    #[test]
    fn ies_are_searched_by_id() {
        let ies = [0x01, 0x02, 0x82, 0x84, 0x00, 0x03, b'a', b'b', b'c'];
        assert_eq!(find_ie(&ies, IE_SSID), Some(&b"abc"[..]));
        assert_eq!(find_ie(&ies, IE_RSN), None);
        // A truncated element ends the search.
        assert_eq!(find_ie(&[0x00, 0x05, b'a'], IE_SSID), None);
    }

    #[test]
    fn extension_elements_are_searched_by_extension_id() {
        let ies = [IE_EXTENSION, 0x02, 0x21, b'x', IE_EXTENSION, 0x03, 0x5D, 0xAA, 0xBB];
        assert_eq!(
            find_extension(&ies, IE_EXT_ANTI_CLOGGING_TOKEN),
            Some(&[0xAA, 0xBB][..])
        );
        assert_eq!(find_extension(&ies, IE_EXT_PASSWORD_IDENTIFIER), Some(&b"x"[..]));
        // An empty extension element has no extension ID.
        assert_eq!(find_extension(&[IE_EXTENSION, 0x00], IE_EXT_PASSWORD_IDENTIFIER), None);
    }
}
