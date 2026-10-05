//! WPA3-Personal on the access point: its side of SAE (IEEE 802.11-2020, 12.4) with each station,
//! which gives the PMK of the 4-way handshake that follows. The exchange is the one of `sae.rs`,
//! which is symmetric: the access point is the station's peer, as hostapd's `handle_auth_sae` has
//! it.
//!
//! Behind the `ap` and `wpa3` features.

use defmt::{debug, info};
use embassy_time::Instant;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

use super::{Mgmt, Security, Writer, AUTH, STATUS_SUCCESS, STATUS_TOO_MANY_STATIONS, STATUS_UNSPECIFIED};
use crate::ieee80211::{
    find_extension, AUTH_ALGORITHM_SAE, IE_EXTENSION, IE_EXT_ANTI_CLOGGING_TOKEN, IE_EXT_PASSWORD_IDENTIFIER,
    STATUS_ANTI_CLOGGING_TOKEN_REQUIRED, STATUS_CHALLENGE_FAILURE, STATUS_SAE_HASH_TO_ELEMENT,
    STATUS_UNKNOWN_PASSWORD_IDENTIFIER, STATUS_UNSUPPORTED_GROUP,
};
use crate::pmksa::Pmksa;
use crate::rsn::Rsne;
use crate::sae::{self, Refusal, Sae};
use crate::wpa3::{scalars, Wpa3};
use crate::{Bus, Runner};

/// How many exchanges with other stations may be under way before a commit needs an anti-clogging
/// token: one, with four station places (hostapd's `sae_anti_clogging_threshold` is 5).
const ANTI_CLOGGING_THRESHOLD: usize = 1;
/// Length of the access point's anti-clogging tokens.
const TOKEN_LEN: usize = 32;
/// How many PMKs of earlier SAE exchanges the access point keeps, one per station address.
const PMKSA_CACHE: usize = 2 * super::MAX_STATIONS;
/// The access point's cache knows one network, its own, and is cleared when it starts.
const NETWORK: u64 = 0;

/// The PMKs of earlier SAE exchanges.
pub(super) type PmksaCache = crate::pmksa::Cache<PMKSA_CACHE>;

/// The PMKSA of the station `addr` that `rsne` names, if the access point still has it: a station
/// that comes back within its lifetime names its PMKID in its association request, after an open
/// system authentication, and skips SAE.
pub(super) fn cached_pmksa(cache: &PmksaCache, addr: &[u8; 6], rsne: &Rsne, now: Instant) -> Option<Pmksa> {
    cache
        .get(addr, NETWORK, now)
        .filter(|pmksa| rsne.names_pmkid(&pmksa.pmkid))
}

/// An SAE exchange with a station.
pub(super) struct Exchange {
    sae: Sae,
    /// The station's commit, to tell a repeated one, which gets the same answer, from a new one.
    peer_commit: [u8; sae::COMMIT_LEN],
    /// The commit status: hash to element (126), or hunting and pecking (0).
    status: u16,
    /// The station's confirm checked out: the exchange is through.
    confirmed: bool,
}

/// The anti-clogging token of the station `from`: HMAC-SHA256(seed, label || address), which only
/// a station that receives frames at that address learns (IEEE 802.11-2020, 12.4.6).
fn anti_clogging_token(seed: &[u8; 32], from: &[u8; 6]) -> [u8; TOKEN_LEN] {
    crate::crypto::hmac_sha256(seed, &[b"nrf70 SAE anti-clogging", from])
}

/// The anti-clogging token in a station's commit, if any: after the group with hunting and
/// pecking, in a container element after the commit with hash to element.
fn commit_token(body: &[u8], h2e: bool) -> Option<&[u8]> {
    if !h2e {
        return (body.len() == sae::COMMIT_LEN + TOKEN_LEN).then(|| &body[2..2 + TOKEN_LEN]);
    }
    find_extension(body.get(sae::COMMIT_LEN..)?, IE_EXT_ANTI_CLOGGING_TOKEN)
}

/// Whether a station's commit names a password identifier, in its element after the commit (and
/// after the anti-clogging token, with hunting and pecking, which puts it inside).
fn names_identifier(body: &[u8]) -> bool {
    [sae::COMMIT_LEN, sae::COMMIT_LEN + TOKEN_LEN].iter().any(|&at| {
        body.get(at..)
            .and_then(|elements| find_extension(elements, IE_EXT_PASSWORD_IDENTIFIER))
            .is_some()
    })
}

/// An SAE authentication frame from the access point: `sequence` 1 for a commit, 2 for a
/// confirm, then `status` and `body`.
fn sae_authentication(bssid: &[u8; 6], to: &[u8; 6], sequence: u16, status: u16, body: &[u8], out: &mut [u8]) -> usize {
    let mut w = Writer::new(out);
    w.header(AUTH, to, bssid)
        .le16(AUTH_ALGORITHM_SAE)
        .le16(sequence)
        .le16(status)
        .put(body);
    w.len
}

impl<BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> Runner<'_, BUS, IN, OUT> {
    /// The password and PT of a WPA3 access point.
    fn ap_wpa3(&self) -> Option<&Wpa3> {
        match self.ap.settings.as_ref().map(|settings| &settings.security) {
            Some(Security::Wpa3 { wpa3, .. }) => Some(wpa3),
            _ => None,
        }
    }

    /// An SAE authentication frame from a station: a commit, or a confirm.
    pub(super) async fn ap_sae_frame(&mut self, mgmt: &Mgmt<'_>) {
        let (Some(sequence), Some(status)) = (mgmt.le16(2), mgmt.le16(4)) else {
            return;
        };
        let body = mgmt.body.get(6..).unwrap_or(&[]);
        match sequence {
            1 => self.ap_sae_commit(&mgmt.from, status, body).await,
            2 => self.ap_sae_confirm(&mgmt.from, body).await,
            _ => {}
        }
    }

    async fn ap_sae_reply(&mut self, to: &[u8; 6], sequence: u16, status: u16, body: &[u8]) {
        let mut frame = [0; 24 + 6 + sae::COMMIT_LEN];
        let len = sae_authentication(&self.ap.mac_addr, to, sequence, status, body, &mut frame);
        self.send_mgmt(&frame[..len], false).await;
    }

    /// A station's commit: a new exchange answers it with the access point's commit. The PWE is
    /// derived by hash to element if the station uses it (status 126), else by hunting and
    /// pecking, which takes 0.39 s on an nRF5340 at 128 MHz.
    async fn ap_sae_commit(&mut self, from: &[u8; 6], status: u16, body: &[u8]) {
        let Some(seed) = self.ap_wpa3().map(|wpa3| wpa3.seed) else {
            return;
        };
        let h2e = match status {
            STATUS_SUCCESS => false,
            STATUS_SAE_HASH_TO_ELEMENT => true,
            _ => return,
        };
        // The access point has one password, without an identifier (hostapd's answer to an unknown
        // one).
        if names_identifier(body) {
            debug!("SAE with {:02x}: commit with a password identifier refused", from);
            return self
                .ap_sae_reply(from, 1, STATUS_UNKNOWN_PASSWORD_IDENTIFIER, &[])
                .await;
        }
        // The commit without its token: with hunting and pecking, the token sits between the group
        // and the scalar.
        let token = commit_token(body, h2e);
        let mut stripped = [0; sae::COMMIT_LEN];
        let body = match token {
            Some(_) if !h2e => {
                stripped[..2].copy_from_slice(&body[..2]);
                stripped[2..].copy_from_slice(&body[2 + TOKEN_LEN..]);
                &stripped[..]
            }
            _ => body,
        };
        let known = self.ap.storage.stations.find(from);
        // A repeated commit gets the same answer: the station did not hear it.
        if let Some(exchange) = known.and_then(|slot| self.ap.storage.exchanges[slot].as_ref()) {
            if body.get(..sae::COMMIT_LEN) == Some(&exchange.peer_commit[..]) {
                let mut commit = [0; sae::COMMIT_LEN];
                let len = exchange.sae.write_commit(&[], &mut commit);
                let status = exchange.status;
                return self.ap_sae_reply(from, 1, status, &commit[..len]).await;
            }
        }
        // While another exchange is under way, a commit needs the station's anti-clogging token: a
        // flood of commits from forged addresses then takes neither station places nor curve
        // computations (hostapd's `use_anti_clogging` and `auth_build_token_req`).
        let under_way = (self.ap.storage.exchanges.iter().enumerate())
            .filter(|(slot, exchange)| Some(*slot) != known && exchange.as_ref().is_some_and(|e| !e.confirmed))
            .count();
        let expected = anti_clogging_token(&seed, from);
        if under_way >= ANTI_CLOGGING_THRESHOLD && token != Some(&expected[..]) {
            debug!("SAE with {:02x}: anti-clogging token asked for", from);
            let mut reply = [0; 2 + 3 + TOKEN_LEN];
            reply[..2].copy_from_slice(&sae::GROUP.to_le_bytes());
            let len = if h2e {
                reply[2..5].copy_from_slice(&[IE_EXTENSION, 1 + TOKEN_LEN as u8, IE_EXT_ANTI_CLOGGING_TOKEN]);
                reply[5..].copy_from_slice(&expected);
                5 + TOKEN_LEN
            } else {
                reply[2..2 + TOKEN_LEN].copy_from_slice(&expected);
                2 + TOKEN_LEN
            };
            return self
                .ap_sae_reply(from, 1, STATUS_ANTI_CLOGGING_TOKEN_REQUIRED, &reply[..len])
                .await;
        }
        let slot = match known.or_else(|| self.ap.storage.stations.find_or_add(from)) {
            Some(slot) => slot,
            None => return self.ap_sae_reply(from, 1, STATUS_TOO_MANY_STATIONS, &[]).await,
        };

        let bssid = self.ap.mac_addr;
        let pwe = match self.ap_wpa3() {
            Some(wpa3) if h2e => Some(wpa3.pt.pwe(&bssid, from)),
            Some(wpa3) => sae::pwe_hunting_and_pecking(wpa3.password(), &[], &bssid, from),
            None => None,
        };
        let Some(pwe) = pwe else {
            return self.ap_sae_reply(from, 1, STATUS_UNSPECIFIED, &[]).await;
        };
        let mut sae = loop {
            self.ap.rsn.sae_attempts += 1;
            let (rand, mask) = scalars(&seed, self.ap.rsn.sae_attempts);
            if let Some(sae) = Sae::new(pwe, rand, mask) {
                break sae;
            }
        };
        match sae.process_commit(body) {
            Ok(()) => {
                let mut commit = [0; sae::COMMIT_LEN];
                let len = sae.write_commit(&[], &mut commit);
                let mut peer_commit = [0; sae::COMMIT_LEN];
                peer_commit.copy_from_slice(&body[..sae::COMMIT_LEN]);
                self.ap.storage.exchanges[slot] = Some(Exchange {
                    sae,
                    peer_commit,
                    status,
                    confirmed: false,
                });
                debug!(
                    "SAE with {:02x}: commit, {}",
                    from,
                    if h2e { "hash to element" } else { "hunting and pecking" }
                );
                self.ap_sae_reply(from, 1, status, &commit[..len]).await;
            }
            // The group the access point takes instead.
            Err(Refusal::UnsupportedGroup(group)) => {
                debug!("SAE with {:02x}: group {} refused", from, group);
                self.ap_sae_reply(from, 1, STATUS_UNSUPPORTED_GROUP, &sae::GROUP.to_le_bytes())
                    .await;
            }
            Err(Refusal::Reflected) => debug!("SAE with {:02x}: reflected commit dropped", from),
            Err(refusal) => {
                debug!("SAE with {:02x}: commit refused, {}", from, refusal);
                self.ap_sae_reply(from, 1, STATUS_UNSPECIFIED, &[]).await;
            }
        }
    }

    /// A station's confirm: right, it gets the access point's confirm, and the station the PMK of
    /// the exchange, which the access point keeps for its next associations; wrong (another
    /// password), it gets status 15.
    async fn ap_sae_confirm(&mut self, from: &[u8; 6], body: &[u8]) {
        let Some(slot) = self.ap.storage.stations.find(from) else {
            return;
        };
        let Some(exchange) = self.ap.storage.exchanges[slot].as_mut() else {
            return;
        };
        if exchange.sae.check_confirm(body).is_err() {
            info!("station {:02x}: wrong SAE confirm (another password)", from);
            self.ap.storage.exchanges[slot] = None;
            return self.ap_sae_reply(from, 2, STATUS_CHALLENGE_FAILURE, &[]).await;
        }
        exchange.confirmed = true;
        let mut confirm = [0; sae::CONFIRM_LEN];
        let len = exchange.sae.write_confirm(&mut confirm);
        let pmksa = exchange.sae.pmksa();
        if let Some(station) = self.ap.storage.stations.get(slot) {
            station.rsn.pmksa = Some(pmksa);
        }
        self.ap.storage.pmksa_cache.insert(from, NETWORK, pmksa, Instant::now());
        debug!("SAE with {:02x}: confirmed", from);
        self.ap_sae_reply(from, 2, STATUS_SUCCESS, &confirm[..len]).await;
    }
}

/// What the fuzz targets reach of the access point's SAE: see `fuzz.rs`.
#[cfg(any(fuzzing, test))]
pub(crate) mod fuzz {
    use super::*;

    /// The body of a station's SAE commit, after its status: where its anti-clogging token and its
    /// password identifier are.
    pub fn commit(body: &[u8]) {
        for h2e in [false, true] {
            if let Some(token) = commit_token(body, h2e) {
                let start = token.as_ptr() as usize - body.as_ptr() as usize;
                assert!(start + token.len() <= body.len());
            }
        }
        let _ = names_identifier(body);
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::assert_eq;
    use std::vec::Vec;

    use super::*;

    #[test]
    fn the_anti_clogging_token_is_found_in_either_place() {
        let token = anti_clogging_token(&[1; 32], &[2; 6]);
        let commit = [7u8; sae::COMMIT_LEN];
        // Hunting and pecking: none, then between the group and the scalar.
        assert_eq!(commit_token(&commit, false), None);
        let mut with_token: Vec<u8> = commit[..2].to_vec();
        with_token.extend_from_slice(&token);
        with_token.extend_from_slice(&commit[2..]);
        assert_eq!(commit_token(&with_token, false), Some(&token[..]));
        // Hash to element: in its container, here after a Rejected Groups element (extension 92).
        let mut h2e = commit.to_vec();
        assert_eq!(commit_token(&h2e, true), None);
        h2e.extend_from_slice(&[IE_EXTENSION, 3, 92, 20, 0]);
        h2e.extend_from_slice(&[IE_EXTENSION, 1 + TOKEN_LEN as u8, IE_EXT_ANTI_CLOGGING_TOKEN]);
        h2e.extend_from_slice(&token);
        assert_eq!(commit_token(&h2e, true), Some(&token[..]));
        // Each address has its own token.
        assert!(anti_clogging_token(&[1; 32], &[3; 6]) != token);
    }

    #[test]
    fn a_cached_pmk_serves_the_station_that_names_it() {
        let pmksa = Pmksa {
            pmk: [1; 32],
            pmkid: [1; 16],
        };
        // An association request's RSNE naming `pmkids`.
        let rsne = |pmkids: &[[u8; 16]]| {
            let offer = crate::rsn::Offer { psk: false, sae: true };
            let mut body = Rsne::access_point(offer).as_bytes()[2..].to_vec();
            body.extend_from_slice(&(pmkids.len() as u16).to_le_bytes());
            pmkids.iter().for_each(|pmkid| body.extend_from_slice(pmkid));
            Rsne::from_body(&body).unwrap()
        };
        let (a, b) = ([0xA; 6], [0xB; 6]);
        let now = Instant::from_secs(1000);
        let found = |cache: &PmksaCache, addr, pmkids: &[[u8; 16]]| {
            cached_pmksa(cache, addr, &rsne(pmkids), now).map(|p| p.pmk)
        };
        let mut cache = PmksaCache::new();
        cache.insert(&a, NETWORK, pmksa, now);
        assert_eq!(found(&cache, &a, &[[1; 16]]), Some([1; 32]));
        assert_eq!(found(&cache, &a, &[[9; 16], [1; 16]]), Some([1; 32]));
        // Not without its PMKID, nor for another station.
        assert_eq!(found(&cache, &a, &[]), None);
        assert_eq!(found(&cache, &a, &[[9; 16]]), None);
        assert_eq!(found(&cache, &b, &[[1; 16]]), None);
    }

    #[test]
    fn the_access_point_finds_what_a_station_commit_carries() {
        let pwe = sae::pwe_hunting_and_pecking(b"password", &[], &[1; 6], &[2; 6]).unwrap();
        let (rand, mask) = scalars(&[5; 32], 1);
        let sae = Sae::new(pwe, rand, mask).unwrap();
        let token = [9; TOKEN_LEN];
        let mut data = [0; 256];
        for h2e in [false, true] {
            // The commit as the station sends it, from the group on: the token where it goes.
            let len = crate::wpa3::commit_data(&sae, h2e, &token, b"", &mut data);
            let body = &data[4..len];
            assert!(!names_identifier(body));
            assert_eq!(commit_token(body, h2e), Some(&token[..]));
            // With a password identifier, after the commit (and the token with hunting and
            // pecking), before the token's container with hash to element.
            let len = crate::wpa3::commit_data(&sae, h2e, &token, b"nrf70", &mut data);
            let body = &data[4..len];
            assert!(names_identifier(body));
            let at = if h2e {
                sae::COMMIT_LEN
            } else {
                sae::COMMIT_LEN + TOKEN_LEN
            };
            assert_eq!(&body[at..at + 8], b"\xFF\x06\x21nrf70");
            if h2e {
                assert_eq!(commit_token(body, h2e), Some(&token[..]));
            }
        }
    }
}
