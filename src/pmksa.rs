//! PMK caching (IEEE 802.11-2020, 12.6.10.2): the PMK of an SAE exchange, kept with its PMKID for
//! the next associations between the same two parties, which then skip SAE. The station keeps the
//! PMKs of the access points it joined, the access point those of its stations.
//!
//! Behind the `wpa3` feature.

use embassy_time::{Duration, Instant};

/// How long a PMK serves (hostapd's and wpa_supplicant's `dot11RSNAConfigPMKLifetime`: 12 hours).
pub(crate) const LIFETIME: Duration = Duration::from_secs(12 * 60 * 60);

/// The PMK of an SAE exchange, and its PMKID: a PMK security association.
#[derive(Clone, Copy)]
pub(crate) struct Pmksa {
    pub(crate) pmk: [u8; 32],
    pub(crate) pmkid: [u8; 16],
}

#[derive(Clone, Copy)]
struct Entry {
    /// The other party's address.
    peer: [u8; 6],
    /// Which network it was on: see [`network_id`].
    network: u64,
    pmksa: Pmksa,
    expires: Instant,
}

/// Up to `N` PMKSAs, one per peer (hostapd's `pmksa_cache_auth`, wpa_supplicant's
/// `pmksa_cache`).
pub(crate) struct Cache<const N: usize>([Option<Entry>; N]);

impl<const N: usize> Cache<N> {
    pub(crate) const fn new() -> Self {
        Self([None; N])
    }

    /// Forgets every PMKSA: the access point starts.
    #[cfg_attr(not(feature = "ap"), allow(dead_code))]
    pub(crate) fn clear(&mut self) {
        self.0 = [None; N];
    }

    /// Keeps the PMKSA of `peer` on `network`, in place of the one it had, or of the one that
    /// expires first if the cache is full.
    pub(crate) fn insert(&mut self, peer: &[u8; 6], network: u64, pmksa: Pmksa, now: Instant) {
        let entry = Entry {
            peer: *peer,
            network,
            pmksa,
            expires: now + LIFETIME,
        };
        let place = (self.0.iter())
            .position(|cached| cached.is_some_and(|cached| cached.peer == *peer))
            .or_else(|| self.0.iter().position(Option::is_none))
            .or_else(|| (0..N).min_by_key(|&i| self.0[i].map_or(Instant::MIN, |cached| cached.expires)));
        if let Some(place) = place {
            self.0[place] = Some(entry);
        }
    }

    /// The PMKSA of `peer` on `network`, if it has not expired.
    pub(crate) fn get(&self, peer: &[u8; 6], network: u64, now: Instant) -> Option<Pmksa> {
        (self.0.iter().flatten())
            .find(|cached| cached.peer == *peer && cached.network == network && cached.expires > now)
            .map(|cached| cached.pmksa)
    }

    /// Forgets the PMKSA of `peer`: it did not serve.
    pub(crate) fn remove(&mut self, peer: &[u8; 6]) {
        for cached in &mut self.0 {
            if cached.is_some_and(|cached| cached.peer == *peer) {
                *cached = None;
            }
        }
    }
}

/// What tells networks apart in the station's cache: the first 8 bytes of HMAC-SHA256(password,
/// SSID length || SSID || password identifier). A PMK found under another password would only fail the 4-way handshake.
pub(crate) fn network_id(ssid: &[u8], password: &[u8], identifier: &[u8]) -> u64 {
    let mac = crate::crypto::hmac_sha256(password, &[&[ssid.len() as u8], ssid, identifier]);
    u64::from_le_bytes(mac[..8].try_into().unwrap())
}

#[cfg(test)]
mod tests {
    use core::assert_eq;

    use super::*;

    #[test]
    fn a_pmksa_serves_its_peer_on_its_network_until_it_expires() {
        let pmksa = |n: u8| Pmksa {
            pmk: [n; 32],
            pmkid: [n; 16],
        };
        let (a, b) = ([0xA; 6], [0xB; 6]);
        let start = Instant::from_secs(1000);
        let found = |cache: &Cache<4>, peer, network, at| cache.get(peer, network, at).map(|p| p.pmk);
        let mut cache = Cache::<4>::new();
        cache.insert(&a, 7, pmksa(1), start);
        assert_eq!(found(&cache, &a, 7, start), Some([1; 32]));
        // Not for another peer, nor on another network, nor once expired.
        assert_eq!(found(&cache, &b, 7, start), None);
        assert_eq!(found(&cache, &a, 8, start), None);
        let expiry = start + LIFETIME;
        assert_eq!(found(&cache, &a, 7, expiry - Duration::from_secs(1)), Some([1; 32]));
        assert_eq!(found(&cache, &a, 7, expiry), None);
        // A new exchange with the peer takes the place of the last one.
        cache.insert(&a, 7, pmksa(2), start);
        assert_eq!(found(&cache, &a, 7, start), Some([2; 32]));
        // Full, the cache drops the one that expires first: here a's.
        for n in 1..4u8 {
            cache.insert(&[n; 6], 7, pmksa(n), start + Duration::from_secs(n as u64));
        }
        assert_eq!(found(&cache, &a, 7, start), Some([2; 32]));
        cache.insert(&b, 7, pmksa(3), start);
        assert_eq!(found(&cache, &a, 7, start), None);
        assert_eq!(found(&cache, &b, 7, start), Some([3; 32]));
        cache.remove(&b);
        assert_eq!(found(&cache, &b, 7, start), None);
        cache.clear();
        assert_eq!(found(&cache, &[1; 6], 7, start), None);
    }

    #[test]
    fn networks_differ_by_ssid_password_and_identifier() {
        let id = network_id(b"nrf70", b"password", b"");
        assert_eq!(id, network_id(b"nrf70", b"password", b""));
        assert!(id != network_id(b"nrf70", b"passwort", b""));
        assert!(id != network_id(b"nrf71", b"password", b""));
        assert!(id != network_id(b"nrf70", b"password", b"id"));
    }
}
