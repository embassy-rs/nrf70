//! What the runner needs for WPA3-Personal networks, behind the `wpa3` feature: the SAE exchange,
//! which takes the place of open system authentication, and the PMK it leaves for the 4-way
//! handshake of `wpa2.rs`. The exchange itself is in `sae.rs`. The PMK is kept (`pmksa.rs`): the
//! next join of the same access point skips SAE.
//!
//! The nRF70 firmware sends the SAE commit and confirm messages as authentication frames, from the
//! authenticate command (`AUTHTYPE_SAE`, with the frame's body from its transaction sequence
//! number on), and reports the access point's as authentication events, as wpa_supplicant's SME
//! drives it in the nRF Connect SDK.

use core::mem::zeroed;

use defmt::{debug, info, warn};
use embassy_time::Instant;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;
use rand_core::CryptoRng;

use crate::crypto::hmac_sha256;
use crate::ieee80211::{
    AUTH_ALGORITHM_SAE, IE_EXTENSION, IE_EXT_ANTI_CLOGGING_TOKEN, IE_EXT_PASSWORD_IDENTIFIER,
    STATUS_ANTI_CLOGGING_TOKEN_REQUIRED, STATUS_CHALLENGE_FAILURE, STATUS_SAE_HASH_TO_ELEMENT, STATUS_SUCCESS,
    STATUS_UNKNOWN_PASSWORD_IDENTIFIER,
};
use crate::pmksa;
use crate::rsn::Rsnxe;
use crate::sae::{self, Pt, Sae};
use crate::station::{Bss, ConnState, Credentials, MLME_TIMEOUT};
use crate::{c, Bus, ConnectError, Control, Runner};

/// The longest password the driver takes for SAE.
const PASSWORD_MAX: usize = 128;

/// The longest password identifier the driver takes.
const IDENTIFIER_MAX: usize = 32;

/// The longest anti-clogging token the driver keeps.
const TOKEN_MAX: usize = 64;

/// How many access points' PMKs the station keeps.
const PMKSA_CACHE: usize = 4;

/// The PMKs of the access points joined.
pub(crate) type PmksaCache = pmksa::Cache<PMKSA_CACHE>;

/// What joining a WPA3-Personal network needs.
#[derive(Clone, Copy)]
pub(crate) struct Wpa3 {
    password: [u8; PASSWORD_MAX],
    password_len: u8,
    /// The password identifier, which tells the access point which of its passwords is ours.
    identifier: [u8; IDENTIFIER_MAX],
    identifier_len: u8,
    /// The base of the PWE for hash to element, derived at the join.
    pub(crate) pt: Pt,
    /// What the SAE scalars and the supplicant's nonces are drawn from.
    pub(crate) seed: [u8; 32],
    /// Which network the password is for, for the PMKs kept.
    network: u64,
}

impl Wpa3 {
    /// What `password`, under `identifier` if given, gives on the network `ssid`, with `seed` to
    /// draw the SAE scalars and the nonces from. `None` if the password is empty or longer than
    /// the driver keeps, or the identifier is. Deriving the PT for hash to element takes a moment
    /// (22 ms on an nRF5340 at 128 MHz).
    pub(crate) fn new(ssid: &[u8], password: &[u8], identifier: Option<&[u8]>, seed: [u8; 32]) -> Option<Self> {
        let identifier = identifier.unwrap_or(&[]);
        if password.is_empty() || password.len() > PASSWORD_MAX || identifier.len() > IDENTIFIER_MAX {
            return None;
        }
        let mut wpa3 = Self {
            password: [0; PASSWORD_MAX],
            password_len: password.len() as u8,
            identifier: [0; IDENTIFIER_MAX],
            identifier_len: identifier.len() as u8,
            pt: Pt::derive(ssid, password, Some(identifier).filter(|id| !id.is_empty())),
            seed,
            network: pmksa::network_id(ssid, password, identifier),
        };
        wpa3.password[..password.len()].copy_from_slice(password);
        wpa3.identifier[..identifier.len()].copy_from_slice(identifier);
        Some(wpa3)
    }

    pub(crate) fn password(&self) -> &[u8] {
        &self.password[..self.password_len as usize]
    }

    /// The password identifier, empty if there is none.
    pub(crate) fn identifier(&self) -> &[u8] {
        &self.identifier[..self.identifier_len as usize]
    }
}

/// What the runner keeps of the SAE exchange of a join, and the PMKs of earlier ones.
pub(crate) struct State<'a> {
    sae: Option<Sae>,
    /// The exchange uses hash to element: the access point announces it.
    h2e: bool,
    /// Exchanges started for this join: each one draws other scalars.
    attempts: u32,
    /// Our confirm is out.
    confirm_sent: bool,
    /// The anti-clogging token the access point asked for, if any.
    token: [u8; TOKEN_MAX],
    token_len: usize,
    /// The PMK of the exchange that went through, or of an earlier one, for the association.
    pub(crate) pmk: Option<[u8; 32]>,
    /// The PMKID of the earlier exchange whose PMK the association uses, if it does: the
    /// association request names it.
    pub(crate) pmkid: Option<[u8; 16]>,
    cache: &'a mut PmksaCache,
}

impl<'a> State<'a> {
    pub(crate) fn new(cache: &'a mut PmksaCache) -> Self {
        Self {
            sae: None,
            h2e: false,
            attempts: 0,
            confirm_sent: false,
            token: [0; TOKEN_MAX],
            token_len: 0,
            pmk: None,
            pmkid: None,
            cache,
        }
    }
}

impl Control<'_> {
    /// Joins the WPA3-Personal network `ssid` with its password: SAE, then the 4-way handshake,
    /// with management frame protection. It picks the strongest access point that offers SAE,
    /// among the WPA3 ones and the WPA2/WPA3 transition ones, and uses hash to element where the
    /// access point announces it, hunting and pecking elsewhere. Returns once the keys are in and
    /// the link is up; see [`Control::join_open`].
    ///
    /// The exchange runs here, on the host, and needs random numbers: 32 bytes from `rng`, any
    /// cryptographically secure generator the application has, taken before joining.
    ///
    /// The PMK of the exchange is kept for 12 hours, for the four access points joined last: the
    /// next join of one of them with the same password skips SAE (an open system authentication,
    /// and the association names the PMK), and goes through SAE after all if the access point no
    /// longer has it.
    pub async fn join_wpa3(
        &mut self,
        ssid: &[u8],
        password: &[u8],
        rng: &mut (impl CryptoRng + ?Sized),
    ) -> Result<(), ConnectError> {
        self.join_sae(ssid, password, None, rng).await
    }

    /// Joins the WPA3-Personal network `ssid` as [`Control::join_wpa3`] does, with the password
    /// that `identifier` names there: an access point may have several passwords, one per
    /// identifier (hostapd's `sae_password=<password>|id=<identifier>`). The identifier goes in
    /// the SAE commit, in the clear. An access point that knows no password under it refuses the
    /// commit with `ConnectError::AuthenticationRejected(123)`. Up to 32 bytes.
    pub async fn join_wpa3_with_identifier(
        &mut self,
        ssid: &[u8],
        password: &[u8],
        identifier: &[u8],
        rng: &mut (impl CryptoRng + ?Sized),
    ) -> Result<(), ConnectError> {
        if identifier.is_empty() {
            return Err(ConnectError::InvalidPassphrase);
        }
        self.join_sae(ssid, password, Some(identifier), rng).await
    }

    async fn join_sae(
        &mut self,
        ssid: &[u8],
        password: &[u8],
        identifier: Option<&[u8]>,
        rng: &mut (impl CryptoRng + ?Sized),
    ) -> Result<(), ConnectError> {
        let mut seed = [0; 32];
        rng.fill_bytes(&mut seed);
        let start = Instant::now();
        let wpa3 = Wpa3::new(ssid, password, identifier, seed).ok_or(ConnectError::InvalidPassphrase)?;
        debug!("SAE: PT derived in {} ms", start.elapsed().as_millis());
        self.join(ssid, Credentials::Wpa3(wpa3)).await
    }
}

/// Two scalars for the SAE exchange `attempt` of a join, drawn from the join's seed:
/// HMAC-SHA256(seed, "nrf70 SAE scalars" || attempt || counter), 48 bytes each, reduced modulo r.
pub(crate) fn scalars(seed: &[u8; 32], attempt: u32) -> (p256::Scalar, p256::Scalar) {
    let mut counter = 0u32;
    let mut next = || loop {
        let mut wide = [0; 48];
        for (i, chunk) in wide.chunks_mut(32).enumerate() {
            let parts: [&[u8]; 4] = [
                b"nrf70 SAE scalars",
                &attempt.to_le_bytes(),
                &counter.to_le_bytes(),
                &[i as u8],
            ];
            chunk.copy_from_slice(&hmac_sha256(seed, &parts)[..chunk.len()]);
        }
        counter += 1;
        if let Some(scalar) = Sae::scalar_from(&wide) {
            return scalar;
        }
    };
    (next(), next())
}

impl<BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> Runner<'_, BUS, IN, OUT> {
    /// For a WPA3 join, starts SAE with `bss` in place of open system authentication: a new
    /// exchange, whose commit goes in the authenticate command `cmd`. With the PMK of an earlier
    /// exchange with `bss`, `cmd` stays an open system authentication, and the association names
    /// its PMKID (wpa_supplicant's `sae_pmksa_caching`).
    pub(super) fn sae_start(&mut self, bss: &Bss, cmd: &mut c::umac_cmd_auth) {
        let Credentials::Wpa3(wpa3) = self.conn_credentials else {
            return;
        };
        let h2e = bss.rsnxe.is_some_and(|rsnxe| rsnxe.offers_sae_h2e());
        self.wpa3.h2e = h2e;
        self.wpa3.sae = None;
        self.wpa3.pmk = None;
        self.wpa3.pmkid = None;
        if let Some(pmksa) = self.wpa3.cache.get(&bss.bssid, wpa3.network, Instant::now()) {
            debug!("SAE: the PMK of an earlier exchange, open system authentication");
            self.wpa3.pmk = Some(pmksa.pmk);
            self.wpa3.pmkid = Some(pmksa.pmkid);
            return;
        }
        let pwe = if h2e {
            Some(wpa3.pt.pwe(&self.wpa2_mac_addr(), &bss.bssid))
        } else {
            sae::pwe_hunting_and_pecking(wpa3.password(), wpa3.identifier(), &self.wpa2_mac_addr(), &bss.bssid)
        };
        self.wpa3.attempts += 1;
        let (rand, mask) = scalars(&wpa3.seed, self.wpa3.attempts);
        self.wpa3.sae = pwe.and_then(|pwe| Sae::new(pwe, rand, mask));
        self.wpa3.confirm_sent = false;
        self.wpa3.token_len = 0;
        debug!(
            "SAE with {}",
            if h2e { "hash to element" } else { "hunting and pecking" }
        );
        self.sae_commit(cmd);
    }

    /// Puts our commit, with the password identifier if there is one and the anti-clogging token
    /// if the access point asked for one, in the authenticate command `cmd`.
    fn sae_commit(&mut self, cmd: &mut c::umac_cmd_auth) {
        let (Some(sae), Credentials::Wpa3(wpa3)) = (&self.wpa3.sae, &self.conn_credentials) else {
            return;
        };
        let token = &self.wpa3.token[..self.wpa3.token_len];
        let mut data = [0; c::MAX_SAE_DATA_LENGTH as usize];
        let len = commit_data(sae, self.wpa3.h2e, token, wpa3.identifier(), &mut data);
        sae_data(cmd, &data[..len]);
    }

    /// Takes an authentication event of an SAE exchange: the access point's commit or confirm.
    /// Returns whether it was one.
    pub(super) async fn sae_frame(&mut self, event: &c::umac_event_mlme) -> bool {
        if !matches!(self.conn_credentials, Credentials::Wpa3(_)) {
            return false;
        }
        if event.flags & c::EVENT_MLME_TIMED_OUT != 0 {
            // No answer: after our confirm, most often a wrong password, which the access point
            // does not answer.
            let error = if self.wpa3.confirm_sent {
                ConnectError::HandshakeFailed
            } else {
                ConnectError::Timeout
            };
            self.connect_failed(error).await;
            return true;
        }
        let len = (event.frame.frame_len as usize).min(event.frame.frame.len());
        let mut frame = [0u8; 512];
        let len = len.min(frame.len());
        for (to, from) in frame.iter_mut().zip(&event.frame.frame[..len]) {
            *to = *from as u8;
        }
        let frame = &frame[..len];
        // Header, then algorithm, transaction sequence number and status.
        let Some(fields) = frame.get(24..30) else {
            return false;
        };
        let le16 = |at: usize| u16::from_le_bytes([fields[at], fields[at + 1]]);
        let (algorithm, sequence, status) = (le16(0), le16(2), le16(4));
        if algorithm != AUTH_ALGORITHM_SAE {
            return false;
        }
        let body = &frame[30..];
        match (sequence, status) {
            (1, STATUS_SUCCESS | STATUS_SAE_HASH_TO_ELEMENT) => self.sae_peer_commit(body).await,
            (1, STATUS_ANTI_CLOGGING_TOKEN_REQUIRED) => self.sae_token(body).await,
            (2, STATUS_SUCCESS) => self.sae_peer_confirm(body).await,
            (2, STATUS_CHALLENGE_FAILURE) => {
                warn!("SAE: the access point refused our confirm: the password is wrong");
                self.connect_failed(ConnectError::HandshakeFailed).await;
            }
            (1, STATUS_UNKNOWN_PASSWORD_IDENTIFIER) => {
                warn!("SAE: the access point knows no password under our identifier");
                self.connect_failed(ConnectError::AuthenticationRejected(STATUS_UNKNOWN_PASSWORD_IDENTIFIER))
                    .await;
            }
            (_, status) => {
                warn!(
                    "SAE: the access point refused message {} with status {}",
                    sequence, status
                );
                self.connect_failed(ConnectError::AuthenticationRejected(status)).await;
            }
        }
        true
    }

    /// The access point's commit: derive the keys, and send our confirm.
    async fn sae_peer_commit(&mut self, body: &[u8]) {
        let Some(sae) = self.wpa3.sae.as_mut() else {
            return;
        };
        if let Err(refusal) = sae.process_commit(body) {
            warn!("SAE: commit refused: {}", refusal);
            self.connect_failed(ConnectError::AuthenticationRejected(1)).await;
            return;
        }
        let mut data = [0; 4 + sae::CONFIRM_LEN];
        data[0..2].copy_from_slice(&2u16.to_le_bytes());
        data[2..4].copy_from_slice(&STATUS_SUCCESS.to_le_bytes());
        sae.write_confirm(&mut data[4..]);
        let Some(bss) = self.conn_bss else {
            return;
        };
        let mut cmd = self.auth_cmd(&bss);
        sae_data(&mut cmd, &data);
        self.wpa3.confirm_sent = true;
        self.set_conn(ConnState::Authenticating, MLME_TIMEOUT);
        self.rpu.send_cmd(&mut cmd).await;
        debug!("SAE: commit taken, confirm sent");
    }

    /// The access point wants an anti-clogging token in our commit: send it again with it.
    async fn sae_token(&mut self, body: &[u8]) {
        // The group, then the token, in a container element with hash to element.
        let token = match body.get(2..) {
            Some([IE_EXTENSION, len, IE_EXT_ANTI_CLOGGING_TOKEN, rest @ ..]) if self.wpa3.h2e => {
                rest.get(..(*len as usize).saturating_sub(1))
            }
            Some(token) if !self.wpa3.h2e => Some(token),
            _ => None,
        };
        let (Some(token), Some(bss)) = (token.filter(|token| token.len() <= TOKEN_MAX), self.conn_bss) else {
            self.connect_failed(ConnectError::AuthenticationRejected(
                STATUS_ANTI_CLOGGING_TOKEN_REQUIRED,
            ))
            .await;
            return;
        };
        debug!("SAE: the access point asks for an anti-clogging token");
        self.wpa3.token[..token.len()].copy_from_slice(token);
        self.wpa3.token_len = token.len();
        let mut cmd = self.auth_cmd(&bss);
        self.sae_commit(&mut cmd);
        self.set_conn(ConnState::Authenticating, MLME_TIMEOUT);
        self.rpu.send_cmd(&mut cmd).await;
    }

    /// The access point's confirm: if it is the one the shared key gives, the PMK is good and
    /// the association follows.
    async fn sae_peer_confirm(&mut self, body: &[u8]) {
        let Some(sae) = self.wpa3.sae.take() else {
            return;
        };
        match sae.check_confirm(body) {
            Ok(()) => {
                debug!("SAE: done");
                let pmksa = sae.pmksa();
                self.wpa3.pmk = Some(pmksa.pmk);
                if let (Some(bss), Credentials::Wpa3(wpa3)) = (self.conn_bss, self.conn_credentials) {
                    self.wpa3.cache.insert(&bss.bssid, wpa3.network, pmksa, Instant::now());
                }
                self.associate().await;
            }
            Err(refusal) => {
                warn!("SAE: confirm refused: {}", refusal);
                self.connect_failed(ConnectError::HandshakeFailed).await;
            }
        }
    }

    /// After `error`, whether a join that used the PMK of an earlier exchange goes on with SAE:
    /// the access point no longer has it (hostapd refuses the association with status 53, which the
    /// RPU reports as the removal of its peer) or does not take it (the 4-way handshake fails). The
    /// PMK is forgotten, and the access point to try again comes back.
    pub(super) fn pmksa_failed(&mut self, error: &ConnectError) -> Option<Bss> {
        let retry = matches!(
            error,
            ConnectError::AuthenticationRejected(_)
                | ConnectError::AssociationRejected(_)
                | ConnectError::Timeout
                | ConnectError::Disconnected
                | ConnectError::HandshakeFailed
        );
        let bss = self.conn_bss.filter(|_| retry && self.wpa3.pmkid.is_some())?;
        info!("the access point did not take the PMK of the last SAE exchange: SAE again");
        self.wpa3.cache.remove(&bss.bssid);
        self.wpa3.pmkid = None;
        Some(bss)
    }

    /// The RSNXE the association request carries for WPA3: with hash to element.
    pub(super) fn sae_rsnxe(&self) -> Option<Rsnxe> {
        (matches!(self.conn_credentials, Credentials::Wpa3(_)) && self.wpa3.h2e).then(Rsnxe::sae_h2e)
    }
}

/// Writes the body of our commit from its transaction sequence number on into `data`, and returns
/// its length. With hunting and pecking, the anti-clogging token goes between the group and the
/// scalar; with hash to element, in a container element at the end. The password identifier's
/// element comes before that container (hostapd's `sae_write_commit`).
pub(crate) fn commit_data(sae: &Sae, h2e: bool, token: &[u8], identifier: &[u8], data: &mut [u8]) -> usize {
    let status = if h2e {
        STATUS_SAE_HASH_TO_ELEMENT
    } else {
        STATUS_SUCCESS
    };
    data[0..2].copy_from_slice(&1u16.to_le_bytes());
    data[2..4].copy_from_slice(&status.to_le_bytes());
    let mut len = 4 + sae.write_commit(if h2e { &[] } else { token }, &mut data[4..]);
    let mut element = |extension: u8, body: &[u8]| {
        data[len..len + 3].copy_from_slice(&[IE_EXTENSION, 1 + body.len() as u8, extension]);
        data[len + 3..len + 3 + body.len()].copy_from_slice(body);
        len += 3 + body.len();
    };
    if !identifier.is_empty() {
        element(IE_EXT_PASSWORD_IDENTIFIER, identifier);
    }
    if h2e && !token.is_empty() {
        element(IE_EXT_ANTI_CLOGGING_TOKEN, token);
    }
    len
}

/// Makes `cmd` an SAE authentication with `data`: the frame's body from its transaction sequence
/// number on.
fn sae_data(cmd: &mut c::umac_cmd_auth, data: &[u8]) {
    cmd.info.auth_type = c::auth_type::AUTHTYPE_SAE as _;
    cmd.info.sae = unsafe { zeroed() };
    cmd.info.sae.sae_data_len = data.len() as _;
    cmd.info.sae.sae_data[..data.len()].copy_from_slice(data);
    cmd.valid_fields |= c::CMD_AUTHENTICATE_SAE_VALID;
}
