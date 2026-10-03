//! WPA3-Personal on the access point: its side of SAE (IEEE 802.11-2020, 12.4) with each station,
//! which gives the PMK of the 4-way handshake that follows. The exchange is the one of `sae.rs`,
//! which is symmetric: the access point is the station's peer, as hostapd's `handle_auth_sae` has
//! it.
//!
//! Behind the `ap` and `wpa3` features.

use defmt::{debug, info};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

use super::{Mgmt, Security, Writer, AUTH, STATUS_SUCCESS, STATUS_TOO_MANY_STATIONS, STATUS_UNSPECIFIED};
use crate::sae::{self, Refusal, Sae};
use crate::wpa3::{scalars, Wpa3};
use crate::{Bus, Runner};

/// The authentication algorithm number of SAE.
pub(super) const AUTH_ALGORITHM_SAE: u16 = 3;
const STATUS_CHALLENGE_FAILURE: u16 = 15;
const STATUS_UNSUPPORTED_GROUP: u16 = 77;
const STATUS_SAE_HASH_TO_ELEMENT: u16 = 126;

/// An SAE exchange with a station.
pub(super) struct Exchange {
    sae: Sae,
    /// The station's commit, to tell a repeated one, which gets the same answer, from a new one.
    peer_commit: [u8; sae::COMMIT_LEN],
    /// The commit status: hash to element (126), or hunting and pecking (0).
    status: u16,
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
        let slot = match self.ap.storage.stations.find(from) {
            Some(slot) => slot,
            None => match self.ap.storage.stations.find_or_add(from) {
                Some(slot) => slot,
                None => return self.ap_sae_reply(from, 1, STATUS_TOO_MANY_STATIONS, &[]).await,
            },
        };
        // A repeated commit gets the same answer: the station did not hear it.
        if let Some(exchange) = &self.ap.storage.exchanges[slot] {
            if body.get(..sae::COMMIT_LEN) == Some(&exchange.peer_commit[..]) {
                let mut commit = [0; sae::COMMIT_LEN];
                let len = exchange.sae.write_commit(&[], &mut commit);
                let status = exchange.status;
                return self.ap_sae_reply(from, 1, status, &commit[..len]).await;
            }
        }

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

    /// A station's confirm: right, it gets the access point's confirm and the station the PMK
    /// of the exchange; wrong (another password), it gets status 15.
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
        let mut confirm = [0; sae::CONFIRM_LEN];
        let len = exchange.sae.write_confirm(&mut confirm);
        let pmk = exchange.sae.pmk();
        if let Some(station) = self.ap.storage.stations.get(slot) {
            station.pmk = Some(pmk);
        }
        debug!("SAE with {:02x}: confirmed", from);
        self.ap_sae_reply(from, 2, STATUS_SUCCESS, &confirm[..len]).await;
    }
}
