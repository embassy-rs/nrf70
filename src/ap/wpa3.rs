//! WPA3-Personal on the access point: its side of SAE (IEEE 802.11-2020, 12.4) with each station,
//! which gives the PMK of the 4-way handshake that follows. The exchange is the one of `sae.rs`,
//! which is symmetric: the access point is the station's peer, as hostapd's `handle_auth_sae` has
//! it.
//!
//! Behind the `ap` and `wpa3` features.

use defmt::{debug, info};
use embassy_time::{Duration, Instant};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

use super::{Mgmt, Security, Writer, AUTH, STATUS_SUCCESS, STATUS_TOO_MANY_STATIONS, STATUS_UNSPECIFIED};
use crate::sae::{self, Refusal, Sae};
use crate::supplicant::Rsne;
use crate::wpa3::{scalars, Wpa3};
use crate::{Bus, Runner};

/// The authentication algorithm number of SAE.
pub(super) const AUTH_ALGORITHM_SAE: u16 = 3;
const STATUS_CHALLENGE_FAILURE: u16 = 15;
const STATUS_ANTI_CLOGGING_TOKEN_REQUIRED: u16 = 76;
const STATUS_UNSUPPORTED_GROUP: u16 = 77;
const STATUS_SAE_HASH_TO_ELEMENT: u16 = 126;

/// How many exchanges with other stations may be under way before a commit needs an anti-clogging
/// token: one, with four station places (hostapd's `sae_anti_clogging_threshold` is 5).
const ANTI_CLOGGING_THRESHOLD: usize = 1;
/// Length of the access point's anti-clogging tokens.
const TOKEN_LEN: usize = 32;
/// The element that carries an anti-clogging token with hash to element: Element ID Extension
/// (255), then extension 93.
const IE_EXTENSION: u8 = 255;
const IE_EXT_ANTI_CLOGGING_TOKEN: u8 = 93;
/// How many PMKs of earlier SAE exchanges the access point keeps, one per station address.
const PMKSA_CACHE: usize = 2 * super::MAX_STATIONS;
/// How long a PMK of an SAE exchange serves (hostapd's and wpa_supplicant's
/// `dot11RSNAConfigPMKLifetime`: 12 hours).
const PMK_LIFETIME: Duration = Duration::from_secs(12 * 60 * 60);

/// The PMK of an SAE exchange, and its PMKID: a PMK security association.
#[derive(Clone, Copy)]
pub(super) struct Pmksa {
    pub(super) pmk: [u8; 32],
    pub(super) pmkid: [u8; 16],
}

#[derive(Clone, Copy)]
struct CachedPmksa {
    addr: [u8; 6],
    pmksa: Pmksa,
    expires: Instant,
}

/// The PMKs of earlier SAE exchanges (hostapd's `pmksa_cache_auth`): a station that comes back
/// within their lifetime names its PMKID in its association request, after an open system
/// authentication, and skips SAE.
pub(super) struct PmksaCache([Option<CachedPmksa>; PMKSA_CACHE]);

impl PmksaCache {
    pub(super) const fn new() -> Self {
        Self([None; PMKSA_CACHE])
    }

    pub(super) fn clear(&mut self) {
        self.0 = [None; PMKSA_CACHE];
    }

    /// Keeps the PMKSA of the station `addr`, in place of the one it had, or of the one that
    /// expires first if the cache is full.
    fn insert(&mut self, addr: &[u8; 6], pmksa: Pmksa, now: Instant) {
        let entry = CachedPmksa {
            addr: *addr,
            pmksa,
            expires: now + PMK_LIFETIME,
        };
        let place = (self.0.iter())
            .position(|cached| cached.is_some_and(|cached| cached.addr == *addr))
            .or_else(|| self.0.iter().position(Option::is_none))
            .or_else(|| (0..PMKSA_CACHE).min_by_key(|&i| self.0[i].map_or(Instant::MIN, |cached| cached.expires)));
        if let Some(place) = place {
            self.0[place] = Some(entry);
        }
    }

    /// The PMKSA of the station `addr` that `rsne` names, if it has not expired.
    pub(super) fn find(&self, addr: &[u8; 6], rsne: &Rsne, now: Instant) -> Option<Pmksa> {
        (self.0.iter().flatten())
            .find(|cached| cached.addr == *addr && cached.expires > now && rsne.names_pmkid(&cached.pmksa.pmkid))
            .map(|cached| cached.pmksa)
    }

    /// Forgets the PMKSA of the station `addr`: its 4-way handshake failed with it.
    pub(super) fn remove(&mut self, addr: &[u8; 6]) {
        for cached in &mut self.0 {
            if cached.is_some_and(|cached| cached.addr == *addr) {
                *cached = None;
            }
        }
    }
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
    sae::hmac_sha256(seed, &[b"nrf70 SAE anti-clogging", from])
}

/// The anti-clogging token in a station's commit, if any: after the group with hunting and
/// pecking, in a container element after the commit with hash to element.
fn commit_token(body: &[u8], h2e: bool) -> Option<&[u8]> {
    if !h2e {
        return (body.len() == sae::COMMIT_LEN + TOKEN_LEN).then(|| &body[2..2 + TOKEN_LEN]);
    }
    let mut elements = body.get(sae::COMMIT_LEN..)?;
    while let [id, len, rest @ ..] = elements {
        let element = rest.get(..*len as usize)?;
        if let [IE_EXT_ANTI_CLOGGING_TOKEN, token @ ..] = element {
            if *id == IE_EXTENSION {
                return Some(token);
            }
        }
        elements = &rest[element.len()..];
    }
    None
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
    /// An SAE authentication frame from a station: a commit, or a confirm.
    pub(super) async fn ap_sae_frame(&mut self, mgmt: &Mgmt<'_>) {
        let Some(Security::Wpa3 { wpa3, .. }) = self.ap.settings.map(|settings| settings.security) else {
            return;
        };
        let (Some(sequence), Some(status)) = (mgmt.le16(2), mgmt.le16(4)) else {
            return;
        };
        let body = mgmt.body.get(6..).unwrap_or(&[]);
        match sequence {
            1 => self.ap_sae_commit(&mgmt.from, status, body, &wpa3).await,
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
    async fn ap_sae_commit(&mut self, from: &[u8; 6], status: u16, body: &[u8], wpa3: &Wpa3) {
        let h2e = match status {
            STATUS_SUCCESS => false,
            STATUS_SAE_HASH_TO_ELEMENT => true,
            _ => return,
        };
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
        let expected = anti_clogging_token(&wpa3.seed, from);
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
        let pwe = if h2e {
            Some(wpa3.pt.pwe(&bssid, from))
        } else {
            sae::pwe_hunting_and_pecking(wpa3.password(), &bssid, from)
        };
        let Some(pwe) = pwe else {
            return self.ap_sae_reply(from, 1, STATUS_UNSPECIFIED, &[]).await;
        };
        let mut sae = loop {
            self.ap.sae_attempts += 1;
            let (rand, mask) = scalars(&wpa3.seed, self.ap.sae_attempts);
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
        let pmksa = Pmksa {
            pmk: exchange.sae.pmk(),
            pmkid: exchange.sae.pmkid(),
        };
        if let Some(station) = self.ap.storage.stations.get(slot) {
            station.pmksa = Some(pmksa);
        }
        self.ap.storage.pmksa_cache.insert(from, pmksa, Instant::now());
        debug!("SAE with {:02x}: confirmed", from);
        self.ap_sae_reply(from, 2, STATUS_SUCCESS, &confirm[..len]).await;
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
    fn a_cached_pmk_serves_the_station_that_names_it_until_it_expires() {
        let pmksa = |n: u8| Pmksa {
            pmk: [n; 32],
            pmkid: [n; 16],
        };
        // An association request's RSNE naming `pmkids`.
        let rsne = |pmkids: &[[u8; 16]]| {
            let offer = crate::supplicant::Offer { psk: false, sae: true };
            let mut body = Rsne::access_point(offer).as_bytes()[2..].to_vec();
            body.extend_from_slice(&(pmkids.len() as u16).to_le_bytes());
            pmkids.iter().for_each(|pmkid| body.extend_from_slice(pmkid));
            Rsne::from_body(&body).unwrap()
        };
        let (a, b) = ([0xA; 6], [0xB; 6]);
        let start = Instant::from_secs(1000);
        let found =
            |cache: &PmksaCache, addr, pmkids: &[[u8; 16]], at| cache.find(addr, &rsne(pmkids), at).map(|p| p.pmk);
        let mut cache = PmksaCache::new();
        cache.insert(&a, pmksa(1), start);
        assert_eq!(found(&cache, &a, &[[1; 16]], start), Some([1; 32]));
        assert_eq!(found(&cache, &a, &[[9; 16], [1; 16]], start), Some([1; 32]));
        // Not without its PMKID, nor for another station, nor once expired.
        assert_eq!(found(&cache, &a, &[], start), None);
        assert_eq!(found(&cache, &b, &[[1; 16]], start), None);
        let expiry = start + PMK_LIFETIME;
        assert_eq!(
            found(&cache, &a, &[[1; 16]], expiry - Duration::from_secs(1)),
            Some([1; 32])
        );
        assert_eq!(found(&cache, &a, &[[1; 16]], expiry), None);
        // A new SAE exchange takes the place of the station's last one.
        cache.insert(&a, pmksa(2), start);
        assert_eq!(found(&cache, &a, &[[1; 16]], start), None);
        assert_eq!(found(&cache, &a, &[[2; 16]], start), Some([2; 32]));
        // Full, the cache drops the one that expires first: here the station's.
        for n in 1..PMKSA_CACHE as u8 {
            cache.insert(&[n; 6], pmksa(n), start + Duration::from_secs(n as u64));
        }
        assert_eq!(found(&cache, &a, &[[2; 16]], start), Some([2; 32]));
        cache.insert(&b, pmksa(3), start);
        assert_eq!(found(&cache, &a, &[[2; 16]], start), None);
        assert_eq!(found(&cache, &b, &[[3; 16]], start), Some([3; 32]));
        cache.remove(&b);
        assert_eq!(found(&cache, &b, &[[3; 16]], start), None);
    }
}
