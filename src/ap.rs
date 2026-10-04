//! Access point mode, behind the `ap` feature.
//!
//! The nRF70 firmware sends the beacons, keeps the links of the stations it is given and tells
//! when one of them dozes, but the rest of an access point is the host's, as hostapd is in the
//! nRF Connect SDK: this module answers probe requests, authenticates and associates stations and
//! adds them to the RPU, and keeps the frames for a station that sleeps until it asks for them.
//! With WPA2, it runs the 4-way handshake with each station (the authenticator of
//! `authenticator.rs`) and gives the RPU the keys.

#[cfg(feature = "wpa3")]
mod wpa3;

use core::mem::zeroed;

use defmt::{debug, info, warn};
use embassy_time::{Duration, Instant};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

#[cfg(feature = "wpa2")]
use crate::{
    authenticator::{self, Authenticator, GroupKeys},
    supplicant::{self, Offer, Rsne, Rsnxe, Suite},
    wpa2::DefaultKey,
};
use crate::{
    c, find_ie, slice8, slice8_mut, tx_priority, unsliceit, Bus, ConnState, Control, Request, Runner, Ssid,
    CAPABILITY_PRIVACY, EVENT_TIMEOUT, IE_SSID, MTU,
};

/// How many stations the access point takes (the SDK's `CONFIG_WIFI_MGMT_AP_MAX_NUM_STA`).
pub(crate) const MAX_STATIONS: usize = 4;
/// How many frames the driver keeps for stations that sleep, all of them together. Once they are
/// all taken, the runner takes no more frames from embassy-net until a station wakes up and gets
/// its own: TCP then slows down instead of losing segments.
const HELD_FRAMES: usize = 4;
/// Beacon interval, in TU of 1.024 ms (hostapd's default).
const BEACON_INTERVAL: u16 = 100;
/// Beacons between two that announce buffered group frames (hostapd's default).
const DTIM_PERIOD: u8 = 2;
/// How long the RPU gets to start or stop the access point.
const CARRIER_TIMEOUT: Duration = Duration::from_secs(2);
/// A station heard from for this long is polled (hostapd's `ap_max_inactivity`).
const INACTIVITY: Duration = Duration::from_secs(300);
/// How long a poll waits for its acknowledgement before the station is let go. A dozing station
/// gets it when the beacon tells it a frame waits.
const POLL_TIMEOUT: Duration = Duration::from_secs(10);
/// Ethertype of the poll: IEEE 802's first local experimental one, which a station's stack drops.
const ETHERTYPE_POLL: u16 = 0x88B5;
/// How long a station that is let go may take to acknowledge its deauthentication, which waits
/// for it to wake up if it dozes, before it goes all the same.
const LEAVE_TIMEOUT: Duration = Duration::from_secs(2);
/// How many times the deauthentication goes out before the station goes all the same.
const DEAUTH_TRIES: u8 = 3;

/// Management frame subtypes, as the first byte of the frame control field (type 0).
const ASSOC_REQ: u8 = 0x00;
const ASSOC_RESP: u8 = 0x10;
const REASSOC_REQ: u8 = 0x20;
const REASSOC_RESP: u8 = 0x30;
const PROBE_REQ: u8 = 0x40;
const PROBE_RESP: u8 = 0x50;
const BEACON: u8 = 0x80;
const DISASSOC: u8 = 0xA0;
const AUTH: u8 = 0xB0;
const DEAUTH: u8 = 0xC0;
const ACTION: u8 = 0xD0;

/// The management frames the RPU hands over in AP mode, as hostapd subscribes to them in the SDK,
/// and the SA Query action frames (category 8), which management frame protection needs: with the
/// first byte of the body each must start with.
const SUBSCRIBED: [(u8, &[u8]); 7] = [
    (AUTH, &[]),
    (ASSOC_REQ, &[]),
    (REASSOC_REQ, &[]),
    (DISASSOC, &[]),
    (DEAUTH, &[]),
    (PROBE_REQ, &[]),
    (ACTION, &[CATEGORY_SA_QUERY]),
];

/// SA Query action frames (IEEE 802.11-2020, 9.6.9).
const CATEGORY_SA_QUERY: u8 = 8;
#[cfg(feature = "wpa2")]
const SA_QUERY_REQUEST: u8 = 0;
#[cfg(feature = "wpa2")]
const SA_QUERY_RESPONSE: u8 = 1;
/// How long an SA Query waits for its answer, and between its requests (hostapd's
/// `assoc_sa_query_max_timeout` and `assoc_sa_query_retry_timeout`, in TUs).
#[cfg(feature = "wpa2")]
const SA_QUERY_TIMEOUT_TU: u32 = 1000;
#[cfg(feature = "wpa2")]
const SA_QUERY_RETRY: Duration = Duration::from_micros(201 * 1024);
/// Timeout Interval element, of the association comeback time type.
#[cfg(feature = "wpa2")]
const IE_TIMEOUT_INTERVAL: u8 = 56;
#[cfg(feature = "wpa2")]
const TIMEOUT_ASSOC_COMEBACK: u8 = 3;

const IE_RATES: u8 = 1;
const IE_DS_PARAMS: u8 = 3;
const IE_ERP: u8 = 42;
const IE_HT_CAPABILITIES: u8 = 45;
const IE_HT_OPERATION: u8 = 61;
const IE_VENDOR: u8 = 221;
const IE_EXT_RATES: u8 = 50;

/// The access point's HT Capabilities (IEEE 802.11-2020, 9.4.2.55): 20 MHz only, short guard
/// interval, SM power save disabled; A-MPDUs up to 64 KiB (what the RPU is set up to receive) with
/// MPDUs 4 µs apart; MCS 0 to 7, the nRF70's one spatial stream.
const HT_CAPABILITIES: [u8; 26] = [
    0x2C, 0x00, // HT capability information
    0x17, // A-MPDU parameters
    0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, // RX MCS bitmask: MCS 0 to 7
    0, 0, // highest supported data rate: from the MCS
    0x01, 0, 0, 0, // TX MCS set defined, the same as the RX one
    0, 0, // HT extended capabilities
    0, 0, 0, 0, // transmit beamforming
    0, // antenna selection
];

/// The WMM Parameter element's body (WMM specification, 2.2.2): hostapd's default EDCA parameters
/// for an access point, without U-APSD.
const WMM_PARAMETERS: [u8; 24] = [
    0x00, 0x50, 0xF2, 0x02, 0x01, 0x01, // WMM, parameter element, version 1
    0x00, 0x00, // QoS info: parameter set 0, no U-APSD; reserved
    0x03, 0xA4, 0x00, 0x00, // best effort: AIFSN 3, CWmin 15, CWmax 1023
    0x27, 0xA4, 0x00, 0x00, // background: AIFSN 7, CWmin 15, CWmax 1023
    0x42, 0x43, 0x5E, 0x00, // video: AIFSN 2, CWmin 7, CWmax 15, TXOP 3 ms
    0x62, 0x32, 0x2F, 0x00, // voice: AIFSN 2, CWmin 3, CWmax 7, TXOP 1.5 ms
];
/// What a station's WMM Information element starts with.
const WMM_INFORMATION: [u8; 5] = [0x00, 0x50, 0xF2, 0x02, 0x00];

/// Capability information bits.
const CAPABILITY_ESS: u16 = 0x0001;
const CAPABILITY_SHORT_PREAMBLE: u16 = 0x0020;
const CAPABILITY_SHORT_SLOT: u16 = 0x0400;

/// IEEE 802.11 status codes.
const STATUS_SUCCESS: u16 = 0;
const STATUS_UNSPECIFIED: u16 = 1;
const STATUS_AUTH_ALGORITHM: u16 = 13;
const STATUS_AUTH_SEQUENCE: u16 = 14;
const STATUS_TOO_MANY_STATIONS: u16 = 17;
const STATUS_RATES: u16 = 18;
#[cfg(feature = "wpa2")]
const STATUS_TRY_LATER: u16 = 30;
#[cfg(feature = "wpa3")]
const STATUS_INVALID_PMKID: u16 = 53;
#[cfg(feature = "wpa2")]
const STATUS_INVALID_ELEMENT: u16 = 40;

/// IEEE 802.11 reason codes.
const REASON_PREV_AUTH_NOT_VALID: u16 = 2;
const REASON_INACTIVITY: u16 = 4;
const REASON_LEAVING: u16 = 3;
const REASON_CLASS2_FROM_UNAUTHENTICATED: u16 = 6;
#[cfg(feature = "wpa2")]
const REASON_4WAY_HANDSHAKE_TIMEOUT: u16 = 15;
#[cfg(feature = "wpa2")]
const REASON_IE_IN_4WAY_DIFFERS: u16 = 17;
#[cfg(feature = "wpa2")]
const REASON_GROUP_KEY_UPDATE_TIMEOUT: u16 = 16;

/// Rates in 500 kb/s units, a basic rate with its top bit set: on 2.4 GHz the 802.11b ones are
/// basic (hostapd's `hw_mode=g`) and the 802.11g ones go on in the extended element, on 5 GHz 6,
/// 12 and 24 Mb/s are.
const RATES_2G: [u8; 8] = [0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24];
const EXT_RATES_2G: [u8; 4] = [0x30, 0x48, 0x60, 0x6C];
const RATES_5G: [u8; 8] = [0x8C, 0x12, 0x98, 0x24, 0xB0, 0x48, 0x60, 0x6C];

/// Most rates a station is taken with: an 802.11g one has 12.
const MAX_RATES: usize = 16;

/// Why an access point did not start.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum ApError {
    /// The SSID is empty or longer than 32 bytes.
    InvalidSsid,
    /// Not a channel the access point can use: 1 to 13 on 2.4 GHz, 36 to 48 or 149 to 177 on
    /// 5 GHz.
    InvalidChannel,
    /// The passphrase of a WPA2-Personal access point is not 8 to 63 bytes long.
    InvalidPassphrase,
    /// A scan or a join is in progress.
    Busy,
    /// The chip is off: see [`Control::power_off`].
    PoweredOff,
    /// The chip did not start the access point: a command failed, or it did not answer.
    Refused,
}

/// What an access point is started with.
#[derive(Clone, Copy)]
pub(crate) struct Settings {
    ssid: Ssid,
    channel: u8,
    frequency: u32,
    security: Security,
}

/// How stations join the access point.
#[derive(Clone, Copy)]
// The WPA3 variant carries the password and the PT of hash to element; settings are made once
// for each start of the access point.
#[allow(clippy::large_enum_variant)]
enum Security {
    Open,
    /// WPA2-Personal: the pre-shared key, and the seed that the ANonces and the group key are
    /// drawn from.
    #[cfg(feature = "wpa2")]
    Wpa2 {
        psk: [u8; 32],
        seed: [u8; 32],
    },
    /// WPA3-Personal (SAE), and with a pre-shared key WPA2/WPA3 transition, which takes WPA2
    /// stations too. The seed is the password's.
    #[cfg(feature = "wpa3")]
    Wpa3 {
        wpa3: crate::wpa3::Wpa3,
        psk: Option<[u8; 32]>,
    },
}

#[cfg(feature = "wpa2")]
impl Security {
    /// What the ANonces and the group keys are drawn from.
    fn seed(&self) -> Option<[u8; 32]> {
        match self {
            Security::Open => None,
            Security::Wpa2 { seed, .. } => Some(*seed),
            #[cfg(feature = "wpa3")]
            Security::Wpa3 { wpa3, .. } => Some(wpa3.seed),
        }
    }

    /// The pre-shared key of the WPA2 stations.
    fn psk(&self) -> Option<[u8; 32]> {
        match self {
            Security::Open => None,
            Security::Wpa2 { psk, .. } => Some(*psk),
            #[cfg(feature = "wpa3")]
            Security::Wpa3 { psk, .. } => *psk,
        }
    }

    /// The key management the RSNE offers.
    fn offer(&self) -> Option<Offer> {
        match self {
            Security::Open => None,
            Security::Wpa2 { .. } => Some(Offer { psk: true, sae: false }),
            #[cfg(feature = "wpa3")]
            Security::Wpa3 { psk, .. } => Some(Offer {
                psk: psk.is_some(),
                sae: true,
            }),
        }
    }

    /// The RSNXE the access point announces: with SAE, that it does hash to element.
    fn rsnxe(&self) -> Option<Rsnxe> {
        #[cfg(feature = "wpa3")]
        if let Security::Wpa3 { .. } = self {
            return Some(Rsnxe::sae_h2e());
        }
        None
    }
}

impl Settings {
    fn new(ssid: &[u8], channel: u8) -> Result<Self, ApError> {
        if ssid.is_empty() {
            return Err(ApError::InvalidSsid);
        }
        Ok(Self {
            ssid: Ssid::new(ssid).ok_or(ApError::InvalidSsid)?,
            channel,
            frequency: channel_frequency(channel).ok_or(ApError::InvalidChannel)?,
            security: Security::Open,
        })
    }

    fn band_2g(&self) -> bool {
        self.frequency < 3000
    }

    fn capability(&self) -> u16 {
        let mut capability = CAPABILITY_ESS | CAPABILITY_SHORT_PREAMBLE;
        if self.band_2g() {
            capability |= CAPABILITY_SHORT_SLOT;
        }
        if self.privacy() {
            capability |= CAPABILITY_PRIVACY;
        }
        capability
    }

    /// Whether data frames are encrypted.
    fn privacy(&self) -> bool {
        !matches!(self.security, Security::Open)
    }

    fn rates(&self) -> &'static [u8] {
        if self.band_2g() {
            &RATES_2G
        } else {
            &RATES_5G
        }
    }

    /// The fields and elements of a beacon or probe response up to the TIM, which the RPU inserts
    /// in a beacon: timestamp, beacon interval, capability, SSID, rates and channel.
    fn write_head_body(&self, w: &mut Writer) {
        w.put(&[0; 8]) // the timestamp, which the RPU fills in
            .le16(BEACON_INTERVAL)
            .le16(self.capability())
            .ie(IE_SSID, self.ssid.as_bytes())
            .ie(IE_RATES, self.rates());
        if self.band_2g() {
            w.ie(IE_DS_PARAMS, &[self.channel]);
        }
    }

    /// The elements of a beacon or probe response after the TIM.
    fn write_tail(&self, w: &mut Writer) {
        if self.band_2g() {
            // No 802.11b station in the BSS: no protection, long preambles allowed.
            w.ie(IE_ERP, &[0]).ie(IE_EXT_RATES, &EXT_RATES_2G);
        }
        #[cfg(feature = "wpa2")]
        if let Some(offer) = self.security.offer() {
            w.put(Rsne::access_point(offer).as_bytes());
        }
        #[cfg(feature = "wpa2")]
        if let Some(rsnxe) = self.security.rsnxe() {
            w.put(rsnxe.as_bytes());
        }
        w.ie(IE_HT_CAPABILITIES, &HT_CAPABILITIES);
        self.write_ht_operation(w);
        w.ie(IE_VENDOR, &WMM_PARAMETERS);
    }

    /// The HT Operation element (IEEE 802.11-2020, 9.4.2.56): the channel, 20 MHz, no protection.
    fn write_ht_operation(&self, w: &mut Writer) {
        let mut body = [0; 22];
        body[0] = self.channel;
        w.ie(IE_HT_OPERATION, &body);
    }

    /// The part of a beacon before the TIM.
    fn beacon_head(&self, bssid: &[u8; 6], out: &mut [u8]) -> usize {
        let mut w = Writer::new(out);
        w.header(BEACON, &[0xFF; 6], bssid);
        self.write_head_body(&mut w);
        w.len
    }

    /// The part of a beacon after the TIM.
    fn beacon_tail(&self, out: &mut [u8]) -> usize {
        let mut w = Writer::new(out);
        self.write_tail(&mut w);
        w.len
    }

    fn probe_response(&self, bssid: &[u8; 6], to: &[u8; 6], out: &mut [u8]) -> usize {
        let mut w = Writer::new(out);
        w.header(PROBE_RESP, to, bssid);
        self.write_head_body(&mut w);
        self.write_tail(&mut w);
        w.len
    }

    /// Whether a probe request asks for this network: by its SSID or for any (wildcard SSID).
    fn probed(&self, body: &[u8]) -> bool {
        find_ie(body, IE_SSID).is_some_and(|ssid| ssid.is_empty() || ssid == self.ssid.as_bytes())
    }

    /// The answer to an association request: with `Ok`, the association ID and what the request
    /// says, the HT and WMM elements going to a station that has them; with `Err`, the status code
    /// of the refusal.
    fn association_response(
        &self,
        bssid: &[u8; 6],
        to: &[u8; 6],
        reassoc: bool,
        answer: Result<(u16, &AssocRequest), u16>,
        out: &mut [u8],
    ) -> usize {
        let (status, aid) = match answer {
            Ok((aid, _)) => (STATUS_SUCCESS, aid | 0xC000),
            Err(status) => (status, 0),
        };
        let mut w = Writer::new(out);
        w.header(if reassoc { REASSOC_RESP } else { ASSOC_RESP }, to, bssid)
            .le16(self.capability())
            .le16(status)
            .le16(aid)
            .ie(IE_RATES, self.rates());
        if self.band_2g() {
            w.ie(IE_EXT_RATES, &EXT_RATES_2G);
        }
        // Refused for now while an SA Query checks the association in place: when to come back.
        #[cfg(feature = "wpa2")]
        if matches!(answer, Err(STATUS_TRY_LATER)) {
            let mut comeback = [TIMEOUT_ASSOC_COMEBACK, 0, 0, 0, 0];
            comeback[1..].copy_from_slice(&SA_QUERY_TIMEOUT_TU.to_le_bytes());
            w.ie(IE_TIMEOUT_INTERVAL, &comeback);
        }
        if let Ok((_, request)) = answer {
            if request.ht_capabilities.is_some() {
                w.ie(IE_HT_CAPABILITIES, &HT_CAPABILITIES);
                self.write_ht_operation(&mut w);
            }
            if request.wmm {
                w.ie(IE_VENDOR, &WMM_PARAMETERS);
            }
        }
        w.len
    }

    /// Checks an association request (its body after the current AP address of a reassociation
    /// request) and returns the station's parameters, or the status code to refuse it with.
    fn association_request(&self, body: &[u8]) -> Result<AssocRequest, u16> {
        let [c0, c1, l0, l1, ies @ ..] = body else {
            return Err(STATUS_UNSPECIFIED);
        };
        if find_ie(ies, IE_SSID) != Some(self.ssid.as_bytes()) {
            return Err(STATUS_UNSPECIFIED);
        }
        let mut request = AssocRequest {
            capability: u16::from_le_bytes([*c0, *c1]),
            listen_interval: u16::from_le_bytes([*l0, *l1]),
            rates: [0; MAX_RATES],
            rates_len: 0,
            ht_capabilities: find_ie(ies, IE_HT_CAPABILITIES).and_then(|ht| ht.try_into().ok()),
            wmm: vendor_ies(ies).any(|ie| ie.starts_with(&WMM_INFORMATION)),
            #[cfg(feature = "wpa2")]
            rsne: None,
            #[cfg(feature = "wpa2")]
            rsnxe: find_ie(ies, supplicant::IE_RSNXE).and_then(Rsnxe::from_body),
        };
        let rates = find_ie(ies, IE_RATES).unwrap_or(&[]).iter();
        for rate in rates.chain(find_ie(ies, IE_EXT_RATES).unwrap_or(&[])) {
            if let Some(slot) = request.rates.get_mut(request.rates_len) {
                *slot = rate & 0x7F;
                request.rates_len += 1;
            }
        }
        if request.rates_len == 0 {
            return Err(STATUS_RATES);
        }
        // A WPA2 station names what it chooses of what the access point offers.
        #[cfg(feature = "wpa2")]
        if self.privacy() {
            let rsne = find_ie(ies, supplicant::IE_RSN)
                .and_then(Rsne::from_body)
                .ok_or(STATUS_INVALID_ELEMENT)?;
            let suite = rsne.check_station(self.security.offer().unwrap_or(Offer { psk: true, sae: false }))?;
            request.rsne = Some((rsne, suite));
        }
        Ok(request)
    }
}

/// The bodies of the vendor specific elements in `ies`.
fn vendor_ies(mut ies: &[u8]) -> impl Iterator<Item = &[u8]> {
    core::iter::from_fn(move || {
        while let [id, len, rest @ ..] = ies {
            let body = rest.get(..*len as usize)?;
            ies = &rest[body.len()..];
            if *id == IE_VENDOR {
                return Some(body);
            }
        }
        None
    })
}

/// The centre frequency of a 20 MHz channel the access point can use, in MHz. 5 GHz channels that
/// need radar detection (52 to 144) are left out.
fn channel_frequency(channel: u8) -> Option<u32> {
    let channel = channel as u32;
    match channel {
        1..=13 => Some(2407 + 5 * channel),
        36 | 40 | 44 | 48 => Some(5000 + 5 * channel),
        149..=177 if (channel - 149).is_multiple_of(4) => Some(5000 + 5 * channel),
        _ => None,
    }
}

/// Builds a frame or a part of one. The buffers are sized for what goes in them: a write past the
/// end is a bug, and panics.
struct Writer<'a> {
    out: &'a mut [u8],
    len: usize,
}

impl<'a> Writer<'a> {
    fn new(out: &'a mut [u8]) -> Self {
        Self { out, len: 0 }
    }

    fn put(&mut self, bytes: &[u8]) -> &mut Self {
        self.out[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        self
    }

    fn le16(&mut self, value: u16) -> &mut Self {
        self.put(&value.to_le_bytes())
    }

    fn ie(&mut self, id: u8, body: &[u8]) -> &mut Self {
        self.put(&[id, body.len() as u8]).put(body)
    }

    /// The 24-byte header of a management frame from the access point.
    fn header(&mut self, subtype: u8, to: &[u8; 6], bssid: &[u8; 6]) -> &mut Self {
        self.put(&[subtype, 0, 0, 0]).put(to).put(bssid).put(bssid).put(&[0, 0])
    }
}

fn authentication_response(bssid: &[u8; 6], to: &[u8; 6], algorithm: u16, status: u16, out: &mut [u8]) -> usize {
    let mut w = Writer::new(out);
    w.header(AUTH, to, bssid).le16(algorithm).le16(2).le16(status);
    w.len
}

/// An SA Query action frame: a request, or the response to one with the same transaction ID.
#[cfg(feature = "wpa2")]
fn sa_query(bssid: &[u8; 6], to: &[u8; 6], action: u8, id: [u8; 2], out: &mut [u8]) -> usize {
    let mut w = Writer::new(out);
    w.header(ACTION, to, bssid).put(&[CATEGORY_SA_QUERY, action]).put(&id);
    w.len
}

/// An SA Query of a station's association.
#[cfg(feature = "wpa2")]
#[derive(Clone, Copy)]
struct SaQuery {
    id: [u8; 2],
    retry_at: Instant,
    deadline: Instant,
}

fn deauthentication(bssid: &[u8; 6], to: &[u8; 6], reason: u16, out: &mut [u8]) -> usize {
    let mut w = Writer::new(out);
    w.header(DEAUTH, to, bssid).le16(reason);
    w.len
}

/// A received management frame.
struct Mgmt<'a> {
    subtype: u8,
    /// The frame control's Protected bit.
    protected: bool,
    to: [u8; 6],
    from: [u8; 6],
    bssid: [u8; 6],
    body: &'a [u8],
}

impl<'a> Mgmt<'a> {
    fn parse(frame: &'a [u8]) -> Option<Self> {
        let header = frame.get(..24)?;
        if header[0] & 0x0C != 0 {
            return None;
        }
        let addr = |n: usize| -> [u8; 6] { header[4 + 6 * n..10 + 6 * n].try_into().unwrap() };
        Some(Self {
            subtype: header[0] & 0xF0,
            protected: header[1] & 0x40 != 0,
            to: addr(0),
            from: addr(1),
            bssid: addr(2),
            body: &frame[24..],
        })
    }

    fn le16(&self, offset: usize) -> Option<u16> {
        Some(u16::from_le_bytes(self.body.get(offset..offset + 2)?.try_into().ok()?))
    }
}

/// What an association request says about the station.
struct AssocRequest {
    capability: u16,
    listen_interval: u16,
    /// Its rates, in 500 kb/s units, without the basic rate bit.
    rates: [u8; MAX_RATES],
    rates_len: usize,
    /// Its HT Capabilities, if it is an 802.11n station.
    ht_capabilities: Option<[u8; 26]>,
    /// It has a WMM Information element: it takes QoS data frames.
    wmm: bool,
    /// Its RSNE, with WPA2.
    #[cfg(feature = "wpa2")]
    rsne: Option<(Rsne, Suite)>,
    /// Its RSNXE, if it has one, with WPA2.
    #[cfg(feature = "wpa2")]
    rsnxe: Option<Rsnxe>,
}

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Phase {
    /// Authenticated, not associated.
    Authenticated,
    /// The association response went out: the station is added once it is acknowledged.
    Responded,
    /// Added to the RPU: data frames flow.
    Associated,
}

/// What a station that is let go waits for: to hear why. It stays in the RPU until it
/// acknowledges its deauthentication, as hostapd's stations stay until its TX status
/// (`ap_sta_deauth_cb`).
#[derive(Clone, Copy)]
struct Leaving {
    reason: u16,
    /// The deauthentication went out, and its TX status is awaited.
    sent: bool,
    /// How many times it went out.
    tries: u8,
    /// When the station goes, told or not.
    deadline: Instant,
}

#[derive(Clone, Copy)]
struct Station {
    addr: [u8; 6],
    phase: Phase,
    capability: u16,
    listen_interval: u16,
    rates: [u8; MAX_RATES],
    rates_len: u8,
    /// Its HT Capabilities, if it is an 802.11n station.
    ht_capabilities: Option<[u8; 26]>,
    /// It takes QoS data frames (WMM).
    wmm: bool,
    /// Its port is open: it gets data frames, group ones included.
    authorized: bool,
    /// The station dozes: its frames are kept until it asks for them.
    asleep: bool,
    /// While asleep, how many frames it asked for (PS-Poll or U-APSD trigger) and still gets.
    service_period: u8,
    /// When a frame last came from it, or one to it was acknowledged. Set when it is associated.
    last_seen: Instant,
    /// When a poll that went unanswered lets it go.
    poll_deadline: Option<Instant>,
    /// It is being let go: it keeps its place in the RPU until it hears why.
    leaving: Option<Leaving>,
    /// With WPA2, the RSNE of its association request, which message 2 must repeat.
    #[cfg(feature = "wpa2")]
    rsne: Option<(Rsne, Suite)>,
    /// With WPA2, the RSNXE of its association request, if any, which message 2 must repeat.
    #[cfg(feature = "wpa2")]
    rsnxe: Option<Rsnxe>,
    /// With WPA3, the PMK of the SAE exchange that authenticated it, or of an earlier one that its
    /// association request named.
    #[cfg(feature = "wpa3")]
    pmksa: Option<crate::pmksa::Pmksa>,
    /// With WPA2, when the last handshake message goes out again if unanswered.
    #[cfg(feature = "wpa2")]
    retry_at: Option<Instant>,
    /// With management frame protection, the SA Query under way, which checks the association of
    /// a station that asks for a new one.
    #[cfg(feature = "wpa2")]
    sa_query: Option<SaQuery>,
    /// The last SA Query went unanswered: the station's next association request is taken.
    #[cfg(feature = "wpa2")]
    sa_query_timed_out: bool,
}

impl Station {
    fn new(addr: [u8; 6]) -> Self {
        Self {
            addr,
            phase: Phase::Authenticated,
            authorized: false,
            capability: 0,
            listen_interval: 0,
            rates: [0; MAX_RATES],
            rates_len: 0,
            ht_capabilities: None,
            wmm: false,
            asleep: false,
            service_period: 0,
            last_seen: Instant::MIN,
            poll_deadline: None,
            leaving: None,
            #[cfg(feature = "wpa2")]
            rsne: None,
            #[cfg(feature = "wpa2")]
            rsnxe: None,
            #[cfg(feature = "wpa3")]
            pmksa: None,
            #[cfg(feature = "wpa2")]
            retry_at: None,
            #[cfg(feature = "wpa2")]
            sa_query: None,
            #[cfg(feature = "wpa2")]
            sa_query_timed_out: false,
        }
    }
}

/// The stations, by slot. A station's association ID is its slot plus one, and its slot is its
/// place in the RPU's table of pending frames.
struct Stations([Option<Station>; MAX_STATIONS]);

impl Stations {
    const fn new() -> Self {
        Self([None; MAX_STATIONS])
    }

    fn find(&self, addr: &[u8; 6]) -> Option<usize> {
        self.0.iter().position(|s| s.is_some_and(|s| s.addr == *addr))
    }

    /// The slot of `addr`, a new one if it has none. `None` if every slot is taken.
    fn find_or_add(&mut self, addr: &[u8; 6]) -> Option<usize> {
        if let Some(slot) = self.find(addr) {
            return Some(slot);
        }
        let slot = self.0.iter().position(Option::is_none)?;
        self.0[slot] = Some(Station::new(*addr));
        Some(slot)
    }

    fn get(&mut self, slot: usize) -> Option<&mut Station> {
        self.0.get_mut(slot)?.as_mut()
    }
}

/// A frame kept for a station that sleeps.
struct Held {
    /// The station's slot, or `None` if this place is free.
    slot: Option<u8>,
    /// Order of arrival.
    seq: u32,
    len: u16,
    frame: [u32; MTU.div_ceil(4)],
}

/// The frames kept for stations that sleep.
struct HeldFrames {
    frames: [Held; HELD_FRAMES],
    next_seq: u32,
}

impl HeldFrames {
    const FREE: Held = Held {
        slot: None,
        seq: 0,
        len: 0,
        frame: [0; MTU.div_ceil(4)],
    };

    const fn new() -> Self {
        Self {
            frames: [Self::FREE; HELD_FRAMES],
            next_seq: 0,
        }
    }

    fn clear(&mut self) {
        for held in &mut self.frames {
            held.slot = None;
        }
    }

    /// The oldest frame of station `slot`.
    fn oldest(&self, slot: usize) -> Option<usize> {
        (0..HELD_FRAMES)
            .filter(|&i| self.frames[i].slot == Some(slot as u8))
            .min_by_key(|&i| self.frames[i].seq)
    }

    fn count(&self, slot: usize) -> usize {
        self.frames.iter().filter(|h| h.slot == Some(slot as u8)).count()
    }

    /// Whether a place is free.
    fn has_room(&self) -> bool {
        self.frames.iter().any(|h| h.slot.is_none())
    }

    /// Keeps a frame for station `slot`. Returns `false` if every place is taken.
    fn hold(&mut self, slot: usize, frame: &[u32], len: usize) -> bool {
        let Some(i) = self.frames.iter().position(|h| h.slot.is_none()) else {
            return false;
        };
        let held = &mut self.frames[i];
        held.slot = Some(slot as u8);
        held.seq = self.next_seq;
        held.len = len as u16;
        held.frame[..len.div_ceil(4)].copy_from_slice(&frame[..len.div_ceil(4)]);
        self.next_seq = self.next_seq.wrapping_add(1);
        true
    }

    /// Drops the frames of station `slot`.
    fn drop_station(&mut self, slot: usize) {
        for held in self.frames.iter_mut().filter(|h| h.slot == Some(slot as u8)) {
            held.slot = None;
        }
    }

    /// The access categories of station `slot`'s frames, one bit each, as the RPU's table of
    /// pending frames wants them (background 0, best effort 1, video 2, voice 3).
    fn categories(&self, slot: usize) -> u32 {
        self.frames
            .iter()
            .filter(|h| h.slot == Some(slot as u8))
            .map(|h| 1 << access_category(tx_priority(&slice8(&h.frame)[..h.len as usize])))
            .fold(0, |a, b| a | b)
    }
}

/// The access category of an 802.1D priority (IEEE 802.11-2020, table 10-1).
fn access_category(priority: u32) -> u32 {
    match priority {
        1 | 2 => 0,
        4 | 5 => 2,
        6 | 7 => 3,
        _ => 1,
    }
}

/// What the access point keeps for its stations. It lives in the application's [`crate::State`],
/// which exists once, rather than in the runner, which is moved around.
pub(crate) struct Storage {
    stations: Stations,
    held: HeldFrames,
    /// The 4-way handshake of each station, with WPA2.
    #[cfg(feature = "wpa2")]
    handshakes: [Option<Authenticator>; MAX_STATIONS],
    /// The SAE exchange of each station, with WPA3.
    #[cfg(feature = "wpa3")]
    exchanges: [Option<wpa3::Exchange>; MAX_STATIONS],
    /// The PMKs of earlier SAE exchanges, with WPA3.
    #[cfg(feature = "wpa3")]
    pmksa_cache: wpa3::PmksaCache,
}

impl Storage {
    #[cfg(feature = "wpa2")]
    const NO_HANDSHAKE: Option<Authenticator> = None;
    #[cfg(feature = "wpa3")]
    const NO_EXCHANGE: Option<wpa3::Exchange> = None;

    pub(crate) const fn new() -> Self {
        Self {
            stations: Stations::new(),
            held: HeldFrames::new(),
            #[cfg(feature = "wpa2")]
            handshakes: [Self::NO_HANDSHAKE; MAX_STATIONS],
            #[cfg(feature = "wpa3")]
            exchanges: [Self::NO_EXCHANGE; MAX_STATIONS],
            #[cfg(feature = "wpa3")]
            pmksa_cache: wpa3::PmksaCache::new(),
        }
    }

    /// Forgets every station.
    fn clear(&mut self) {
        self.stations = Stations::new();
        self.held.clear();
        #[cfg(feature = "wpa2")]
        self.handshakes.iter_mut().for_each(|handshake| *handshake = None);
        #[cfg(feature = "wpa3")]
        self.exchanges.iter_mut().for_each(|exchange| *exchange = None);
        #[cfg(feature = "wpa3")]
        self.pmksa_cache.clear();
    }
}

/// What the runner keeps for the access point.
pub(crate) struct State<'a> {
    /// The access point is up.
    pub(crate) running: bool,
    settings: Option<Settings>,
    mac_addr: [u8; 6],
    storage: &'a mut Storage,
    /// With WPA2, the group key in use.
    #[cfg(feature = "wpa2")]
    gtk: Option<GroupKeys>,
    /// With WPA2, while the group key is renewed: the next one, which becomes the one in use once
    /// every station has it.
    #[cfg(feature = "wpa2")]
    next_gtk: Option<GroupKeys>,
    /// With WPA2, when the group key is renewed next.
    #[cfg(feature = "wpa2")]
    rekey_at: Option<Instant>,
    /// With WPA2, how many group keys were drawn.
    #[cfg(feature = "wpa2")]
    gtks: u64,
    /// With WPA2, how many ANonces were drawn.
    #[cfg(feature = "wpa2")]
    nonces: u64,
    /// With WPA3, how many SAE exchanges drew their scalars.
    #[cfg(feature = "wpa3")]
    sae_attempts: u32,
    /// The RPU's answer to the last interface type change.
    set_interface: Option<i32>,
    /// Identifies each management frame sent, for the RPU's TX status.
    cookie: u64,
    /// The station each TX token carries frames to, to note it seen when they are acknowledged.
    token_stations: [Option<u8>; crate::MAX_TX_TOKENS],
}

impl<'a> State<'a> {
    pub(crate) fn new(storage: &'a mut Storage) -> Self {
        Self {
            running: false,
            settings: None,
            mac_addr: [0; 6],
            storage,
            #[cfg(feature = "wpa2")]
            gtk: None,
            #[cfg(feature = "wpa2")]
            next_gtk: None,
            #[cfg(feature = "wpa2")]
            rekey_at: None,
            #[cfg(feature = "wpa2")]
            gtks: 0,
            #[cfg(feature = "wpa2")]
            nonces: 0,
            #[cfg(feature = "wpa3")]
            sae_attempts: 0,
            set_interface: None,
            cookie: 0,
            token_stations: [None; crate::MAX_TX_TOKENS],
        }
    }
}

impl Control<'_> {
    /// Starts an open (unencrypted) access point named `ssid` on `channel`: 1 to 13 on 2.4 GHz,
    /// or 36 to 48 or 149 to 177 on 5 GHz, where the country's rules allow. A network joined
    /// until then is left. Returns once the access point beacons and the link is up; up to four
    /// stations may then join it.
    ///
    /// Scans and joins are refused while the access point runs.
    pub async fn start_ap_open(&mut self, ssid: &[u8], channel: u8) -> Result<(), ApError> {
        let settings = Settings::new(ssid, channel)?;
        self.request_ap(settings).await
    }

    /// Hands the access point to the runner, and returns its answer.
    async fn request_ap(&mut self, settings: Settings) -> Result<(), ApError> {
        self.shared.ap_result.clear();
        self.shared.requests.send(Request::StartAp(settings)).await;
        self.shared.ap_result.receive().await
    }

    /// Stops the access point, with a word to each station, and takes the link down. Nothing
    /// happens if none runs.
    pub async fn stop_ap(&mut self) {
        self.shared.ap_result.clear();
        self.shared.requests.send(Request::StopAp).await;
        let _ = self.shared.ap_result.receive().await;
    }
}

/// The runner's side of AP mode. The first items are what `lib.rs` calls, each with an empty
/// counterpart there for a build without the feature.
impl<BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> Runner<'_, BUS, IN, OUT> {
    pub(super) fn ap_running(&self) -> bool {
        self.ap.running
    }

    /// Whether the runner may take a frame from embassy-net: not while the frames kept for
    /// sleeping stations fill every place.
    pub(super) fn ap_has_room(&self) -> bool {
        !self.ap.running || self.ap.storage.held.has_room()
    }

    pub(super) fn set_ap_address(&mut self, mac_addr: [u8; 6]) {
        self.ap.mac_addr = mac_addr;
    }

    /// Starts the access point (hostapd's setup through the SDK's `nrf_wifi_wpa_supp_init_ap`,
    /// `register_mgmt_frames_ap` and `nrf_wifi_wpa_supp_start_ap`).
    pub(super) async fn start_ap(&mut self, settings: Settings) -> Result<(), ApError> {
        let connecting = !matches!(self.conn, ConnState::Idle | ConnState::Connected);
        let scanning = self
            .scan_deadline
            .is_some_and(|deadline| embassy_time::Instant::now() < deadline);
        if connecting || scanning {
            return Err(ApError::Busy);
        }
        if self.ap.running {
            self.stop_ap().await;
        }
        if self.conn == ConnState::Connected {
            self.leave().await;
        }
        if !self.set_interface_type(c::iftype::IFTYPE_AP).await {
            return Err(ApError::Refused);
        }
        for (subtype, prefix) in SUBSCRIBED {
            let mut cmd: c::umac_cmd_mgmt_frame_reg = unsafe { zeroed() };
            cmd.info.frame_type = subtype as u16;
            cmd.info.frame_match.frame_match_len = prefix.len() as _;
            cmd.info.frame_match.frame_match[..prefix.len()].copy_from_slice(prefix);
            self.send_cmd(&mut cmd).await;
        }

        let freq_params = freq_params(settings.frequency);
        let mut cmd: c::umac_cmd_set_wiphy = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_SET_WIPHY_FREQ_PARAMS_VALID;
        cmd.info.freq_params = freq_params;
        self.send_cmd(&mut cmd).await;

        let mut cmd: c::umac_cmd_start_ap = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_BEACON_INFO_BEACON_INTERVAL_VALID
            | c::CMD_BEACON_INFO_VERSIONS_VALID
            | c::CMD_BEACON_INFO_CIPHER_SUITE_GROUP_VALID;
        let info = &mut cmd.info;
        info.beacon_interval = BEACON_INTERVAL;
        info.dtim_period = DTIM_PERIOD;
        info.auth_type = c::auth_type::AUTHTYPE_OPEN_SYSTEM as _;
        let beacon = &mut info.beacon_data;
        beacon.head_len = settings.beacon_head(&self.ap.mac_addr, &mut beacon.head) as u32;
        beacon.tail_len = settings.beacon_tail(&mut beacon.tail) as u32;
        info.ssid = settings.ssid.to_c();
        info.connect_common_info.valid_fields = c::CONNECT_COMMON_INFO_WPA_VERSIONS_VALID;
        #[cfg(feature = "wpa2")]
        if settings.privacy() {
            let common = &mut info.connect_common_info;
            common.valid_fields |= c::CONNECT_COMMON_INFO_CIPHER_SUITES_PAIRWISE_VALID;
            common.num_cipher_suites_pairwise = 1;
            // A packed field: copied out, set, and copied back.
            let mut suites = common.cipher_suites_pairwise;
            suites[0] = supplicant::CIPHER_SUITE_CCMP;
            common.cipher_suites_pairwise = suites;
        }
        info.freq_params = freq_params;
        self.carrier_on = false;
        self.send_cmd(&mut cmd).await;
        if !self.wait_until(CARRIER_TIMEOUT, |r| r.carrier_on).await {
            warn!("the access point did not start");
            self.set_interface_type(c::iftype::IFTYPE_STATION).await;
            return Err(ApError::Refused);
        }

        let mut cmd: c::umac_cmd_set_bss = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_SET_BSS_CTS_VALID
            | c::CMD_SET_BSS_PREAMBLE_VALID
            | c::CMD_SET_BSS_SLOT_VALID
            | c::CMD_SET_BSS_HT_OPMODE_VALID
            | c::CMD_SET_BSS_AP_ISOLATE_VALID;
        cmd.bss_info.preamble = 1;
        cmd.bss_info.slot = settings.band_2g() as u8;
        self.send_cmd(&mut cmd).await;
        // With WPA2, the group keys the access point sends broadcasts with: the GTK, and the IGTK
        // that protects its broadcast management frames.
        #[cfg(feature = "wpa2")]
        if let Some(seed) = settings.security.seed() {
            let keys = GroupKeys::derive(&seed, &self.ap.mac_addr, 0);
            self.install_group_keys(&keys).await;
            self.set_default_key(keys.gtk.index, DefaultKey::Multicast).await;
            self.set_default_key(keys.igtk.index, DefaultKey::Management).await;
            self.ap.gtk = Some(keys);
            self.ap.next_gtk = None;
            self.ap.gtks = 0;
            self.ap.rekey_at = Some(Instant::now() + GROUP_REKEY);
        }

        self.ap.settings = Some(settings);
        self.ap.storage.clear();
        for slot in 0..MAX_STATIONS {
            self.write_pending_entry(slot, &[0; 6]).await;
        }
        self.ap.running = true;
        // Stations that were associated with an earlier run of the access point think they still
        // are: hostapd tells them otherwise when it starts.
        let bssid = self.ap.mac_addr;
        let mut frame = [0; 26];
        let len = deauthentication(&bssid, &[0xFF; 6], REASON_PREV_AUTH_NOT_VALID, &mut frame);
        self.send_mgmt(&frame[..len], true).await;
        self.set_link(true);
        info!(
            "access point {=[u8]:a} up on channel {}",
            settings.ssid.as_bytes(),
            settings.channel
        );
        Ok(())
    }

    /// Stops the access point (the SDK's `nrf_wifi_wpa_supp_deinit_ap`), saying goodbye to each
    /// station.
    pub(super) async fn stop_ap(&mut self) {
        if !self.ap.running {
            return;
        }
        for slot in 0..MAX_STATIONS {
            if self.ap.storage.stations.0[slot].is_some() {
                self.remove_station(slot, Some(REASON_LEAVING)).await;
            }
        }
        // Each one hears it before the access point goes, or the time it gets runs out.
        let told = |r: &Self| r.ap.storage.stations.0.iter().flatten().all(|s| s.leaving.is_none());
        if !self.wait_until(LEAVE_TIMEOUT, told).await {
            for slot in 0..MAX_STATIONS {
                self.remove_station(slot, None).await;
            }
        }
        self.set_link(false);
        // Without settings no frame is answered any more; the access point counts as running until
        // the RPU has stopped it, so that the station removals it reports meanwhile land here.
        self.ap.settings = None;
        let mut cmd: c::umac_cmd_stop_ap = unsafe { zeroed() };
        self.send_cmd(&mut cmd).await;
        if !self.wait_until(CARRIER_TIMEOUT, |r| !r.carrier_on).await {
            warn!("the access point did not report its carrier off");
            self.carrier_on = false;
        }
        self.ap.running = false;
        #[cfg(feature = "wpa2")]
        {
            self.ap.next_gtk = None;
            self.ap.rekey_at = None;
        }
        self.set_interface_type(c::iftype::IFTYPE_STATION).await;
        info!("access point down");
    }

    /// The chip was turned off: the access point is gone with it.
    pub(super) fn forget_ap(&mut self) {
        if self.ap.running {
            self.set_link(false);
        }
        self.ap.running = false;
        self.ap.storage.clear();
        #[cfg(feature = "wpa2")]
        {
            self.ap.next_gtk = None;
            self.ap.rekey_at = None;
        }
    }

    /// Takes the interface down, changes its type and brings it up again (the SDK's
    /// `nrf_wifi_iftype_change`). Returns whether the RPU accepted the type.
    async fn set_interface_type(&mut self, iftype: c::iftype) -> bool {
        self.set_interface_state(false).await;
        let mut cmd: c::umac_cmd_chg_vif_attr = unsafe { zeroed() };
        cmd.valid_fields = c::SET_INTERFACE_IFTYPE_VALID | c::SET_INTERFACE_USE_4ADDR_VALID;
        cmd.info.iftype = iftype as _;
        self.ap.set_interface = None;
        self.send_cmd(&mut cmd).await;
        let answered = self.wait_until(EVENT_TIMEOUT, |r| r.ap.set_interface.is_some()).await;
        self.set_interface_state(true).await;
        match self.ap.set_interface {
            Some(0) => true,
            status => {
                warn!(
                    "interface type {} refused: {}",
                    iftype as u32,
                    if answered { status } else { None }
                );
                false
            }
        }
    }

    /// The RPU answered an interface type change.
    pub(super) fn interface_type_set(&mut self, body: &[u8]) {
        self.ap.set_interface = Some(unsliceit::<c::umac_event_set_interface>(body).return_value);
    }

    /// Sends a management frame (the SDK's `nrf_wifi_nl80211_send_mlme`). With `noack`, the RPU
    /// does not wait for an acknowledgement, nor retry.
    async fn send_mgmt(&mut self, frame: &[u8], noack: bool) {
        let Some(settings) = self.ap.settings else {
            return;
        };
        let mut cmd: c::umac_cmd_mgmt_tx = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_FRAME_FREQ_VALID | c::CMD_FRAME_DURATION_VALID | c::CMD_SET_FRAME_FREQ_PARAMS_VALID;
        let info = &mut cmd.info;
        if noack {
            info.flags = c::CMD_FRAME_DONT_WAIT_FOR_ACK;
        }
        info.frequency = settings.frequency;
        info.freq_params = freq_params(settings.frequency);
        info.frame.frame_len = frame.len() as _;
        for (to, from) in info.frame.frame.iter_mut().zip(frame) {
            *to = *from as _;
        }
        self.ap.cookie += 1;
        info.host_cookie = self.ap.cookie;
        self.send_cmd(&mut cmd).await;
    }

    /// A management frame for the access point (`UMAC_EVENT_FRAME`): what hostapd's
    /// `ieee802_11_mgmt` does with it.
    pub(super) async fn ap_frame(&mut self, body: &[u8]) {
        let event: &c::umac_event_mlme = unsliceit(body);
        let Some(settings) = self.ap.settings.filter(|_| self.ap.running) else {
            return;
        };
        let frame = mlme_frame(event);
        let Some(mgmt) = Mgmt::parse(frame) else {
            return;
        };
        let bssid = self.ap.mac_addr;
        if mgmt.subtype != PROBE_REQ && mgmt.bssid != bssid {
            return;
        }
        let mut out = [0u8; 400];
        match mgmt.subtype {
            PROBE_REQ => {
                if settings.probed(mgmt.body) {
                    let len = settings.probe_response(&bssid, &mgmt.from, &mut out);
                    // A probe request to everyone gets its answer once (hostapd's `noack`).
                    self.send_mgmt(&out[..len], mgmt.to == [0xFF; 6]).await;
                }
            }
            AUTH => {
                let (Some(algorithm), Some(sequence)) = (mgmt.le16(0), mgmt.le16(2)) else {
                    return;
                };
                // WPA3: SAE, in place of open system authentication.
                #[cfg(feature = "wpa3")]
                if algorithm == wpa3::AUTH_ALGORITHM_SAE && matches!(settings.security, Security::Wpa3 { .. }) {
                    return self.ap_sae_frame(&mgmt).await;
                }
                let status = if algorithm != c::auth_type::AUTHTYPE_OPEN_SYSTEM as u16 {
                    STATUS_AUTH_ALGORITHM
                } else if sequence != 1 {
                    STATUS_AUTH_SEQUENCE
                } else {
                    match self.ap.storage.stations.find(&mgmt.from) {
                        // A protected association does not end on an unprotected frame that anyone
                        // could forge: it stays until a new association is checked (SA Query).
                        Some(slot) if self.protected_association(slot) => STATUS_SUCCESS,
                        // Otherwise a station that authenticates again starts over.
                        found => {
                            if let Some(slot) = found {
                                self.remove_station(slot, None).await;
                            }
                            match self.ap.storage.stations.find_or_add(&mgmt.from) {
                                Some(_) => STATUS_SUCCESS,
                                None => STATUS_TOO_MANY_STATIONS,
                            }
                        }
                    }
                };
                debug!("authentication from {:02x}: status {}", mgmt.from, status);
                let len = authentication_response(&bssid, &mgmt.from, algorithm, status, &mut out);
                self.send_mgmt(&out[..len], false).await;
            }
            ASSOC_REQ | REASSOC_REQ => {
                let reassoc = mgmt.subtype == REASSOC_REQ;
                let Some(mut slot) = self.ap.storage.stations.find(&mgmt.from) else {
                    debug!("association request from {:02x}, not authenticated", mgmt.from);
                    let len = deauthentication(&bssid, &mgmt.from, REASON_CLASS2_FROM_UNAUTHENTICATED, &mut out);
                    self.send_mgmt(&out[..len], false).await;
                    return;
                };
                // A station with a protected association asks for a new one: maybe it lost its keys, maybe
                // the request is forged. It is refused for now, and an SA Query checks the association
                // in place; unanswered, the next request is taken (hostapd's `check_assoc_ies`).
                #[cfg(feature = "wpa2")]
                if self.protected_association(slot)
                    && !self.ap.storage.stations.0[slot].is_some_and(|s| s.sa_query_timed_out)
                {
                    debug!(
                        "association request from {:02x}, associated with protection: SA Query",
                        mgmt.from
                    );
                    self.start_sa_query(slot).await;
                    let len =
                        settings.association_response(&bssid, &mgmt.from, reassoc, Err(STATUS_TRY_LATER), &mut out);
                    self.send_mgmt(&out[..len], false).await;
                    return;
                }
                if self.ap.storage.stations.0[slot].is_some_and(|s| s.phase == Phase::Associated) {
                    // Associating again: the RPU's entry goes, the new one comes with the answer.
                    // The PMK of the SAE exchange just before stays.
                    #[cfg(feature = "wpa3")]
                    let pmksa = self.ap.storage.stations.0[slot].and_then(|s| s.pmksa);
                    self.remove_station(slot, None).await;
                    slot = self.ap.storage.stations.find_or_add(&mgmt.from).unwrap();
                    #[cfg(feature = "wpa3")]
                    if let Some(station) = self.ap.storage.stations.get(slot) {
                        station.pmksa = pmksa;
                    }
                }
                let body = mgmt.body.get(if reassoc { 6 } else { 0 }..).unwrap_or(&[]);
                let request = settings.association_request(body);
                // SAE needs the PMK of an SAE exchange: the one just before, or after an open system
                // authentication, a cached one that the request names (hostapd's `check_assoc_ies`).
                #[cfg(feature = "wpa3")]
                let request = request.and_then(|request| {
                    let Some((rsne, suite)) = request.rsne.filter(|(_, suite)| suite.akm == supplicant::Akm::Sae)
                    else {
                        return Ok(request);
                    };
                    let station = self.ap.storage.stations.get(slot).unwrap();
                    if station.pmksa.is_none() {
                        station.pmksa =
                            wpa3::cached_pmksa(&self.ap.storage.pmksa_cache, &mgmt.from, &rsne, Instant::now());
                        if station.pmksa.is_some() {
                            debug!("association request from {:02x}: cached PMK, {}", mgmt.from, suite);
                        }
                    }
                    match station.pmksa {
                        Some(_) => Ok(request),
                        None => Err(STATUS_INVALID_PMKID),
                    }
                });
                let status = match &request {
                    Ok(request) => {
                        let station = self.ap.storage.stations.get(slot).unwrap();
                        station.capability = request.capability;
                        station.listen_interval = request.listen_interval;
                        station.rates = request.rates;
                        station.rates_len = request.rates_len as u8;
                        station.ht_capabilities = request.ht_capabilities;
                        station.wmm = request.wmm;
                        #[cfg(feature = "wpa2")]
                        {
                            station.rsne = request.rsne;
                            station.rsnxe = request.rsnxe;
                        }
                        station.phase = Phase::Responded;
                        STATUS_SUCCESS
                    }
                    Err(status) => *status,
                };
                debug!("association request from {:02x}: status {}", mgmt.from, status);
                let answer = request
                    .as_ref()
                    .map(|request| (slot as u16 + 1, request))
                    .map_err(|status| *status);
                let len = settings.association_response(&bssid, &mgmt.from, reassoc, answer, &mut out);
                self.send_mgmt(&out[..len], false).await;
            }
            #[cfg(feature = "wpa2")]
            ACTION => self.sa_query_frame(&mgmt).await,
            DISASSOC | DEAUTH => {
                if let Some(slot) = self.ap.storage.stations.find(&mgmt.from) {
                    if self.protected_association(slot) && !mgmt.protected {
                        debug!(
                            "unprotected disassociation or deauthentication from {:02x} ignored",
                            mgmt.from
                        );
                        return;
                    }
                    info!(
                        "station {:02x} left, reason {} (protected {})",
                        mgmt.from,
                        mgmt.le16(0),
                        mgmt.protected
                    );
                    self.remove_station(slot, None).await;
                }
            }
            _ => {}
        }
    }

    /// The RPU reports a management frame sent (`UMAC_EVENT_FRAME_TX_STATUS`): an acknowledged
    /// association response adds its station (hostapd's `handle_assoc_cb`).
    pub(super) async fn ap_frame_sent(&mut self, body: &[u8]) {
        let event: &c::umac_event_mlme = unsliceit(body);
        let acked = event.flags & c::EVENT_MLME_ACK != 0;
        let Some(mgmt) = Mgmt::parse(mlme_frame(event)) else {
            return;
        };
        if mgmt.subtype == DEAUTH {
            return self.deauthentication_sent(&mgmt.to, acked).await;
        }
        if !matches!(mgmt.subtype, ASSOC_RESP | REASSOC_RESP) || mgmt.le16(2) != Some(STATUS_SUCCESS) {
            return;
        }
        let Some(slot) = self.ap.storage.stations.find(&mgmt.to) else {
            return;
        };
        let station = self.ap.storage.stations.get(slot).unwrap();
        if station.phase != Phase::Responded {
            return;
        }
        if !acked {
            debug!("association response to {:02x} not acknowledged", mgmt.to);
            station.phase = Phase::Authenticated;
            return;
        }
        station.phase = Phase::Associated;
        station.last_seen = Instant::now();
        let station = *station;
        self.add_station(slot, &station).await;
        info!("station {:02x} joined, association ID {}", station.addr, slot + 1);
        if self.ap.settings.is_some_and(|settings| settings.privacy()) {
            // WPA2: the port opens once the 4-way handshake is done.
            #[cfg(feature = "wpa2")]
            self.start_handshake(slot).await;
        } else {
            self.authorize_station(&station.addr).await;
        }
    }

    /// Adds a station to the RPU, its port closed (the SDK's `nrf_wifi_wpa_supp_sta_add`).
    async fn add_station(&mut self, slot: usize, station: &Station) {
        let mut cmd: c::umac_cmd_add_sta = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_NEW_STATION_AID_VALID
            | c::CMD_NEW_STATION_STA_CAPABILITY_VALID
            | c::CMD_NEW_STATION_LISTEN_INTERVAL_VALID
            | c::CMD_NEW_STATION_SUPP_RATES_VALID
            | c::CMD_NEW_STATION_STA_FLAGS2_VALID;
        let info = &mut cmd.info;
        info.listen_interval = station.listen_interval as _;
        info.aid = slot as u16 + 1;
        info.sta_capability = station.capability;
        info.supp_rates.num_rates = station.rates_len as _;
        info.supp_rates.rates[..station.rates_len as usize]
            .copy_from_slice(&station.rates[..station.rates_len as usize]);
        let mut flags = 0;
        if station.capability & CAPABILITY_SHORT_PREAMBLE != 0 {
            flags |= c::STA_FLAG_SHORT_PREAMBLE;
        }
        if station.wmm {
            flags |= c::STA_FLAG_WME;
        }
        // Management frame protection, which the RPU does for the frames to and from the station.
        #[cfg(feature = "wpa2")]
        if station.rsne.is_some_and(|(_, suite)| suite.mfp) {
            flags |= c::STA_FLAG_MFP;
        }
        info.sta_flags2 = c::sta_flag_update {
            mask: c::STA_FLAG_AUTHORIZED | c::STA_FLAG_SHORT_PREAMBLE | c::STA_FLAG_WME | c::STA_FLAG_MFP,
            set: flags,
        };
        if let Some(ht) = station.ht_capabilities {
            cmd.valid_fields |= c::CMD_NEW_STATION_HT_CAPABILITY_VALID;
            info.ht_capability[..ht.len()].copy_from_slice(&ht);
        }
        info.mac_addr = station.addr;
        self.send_cmd(&mut cmd).await;
        self.write_pending_entry(slot, &station.addr).await;
    }

    /// Opens a station's port (the SDK's `nrf_wifi_wpa_supp_sta_set_flags`).
    async fn authorize_station(&mut self, addr: &[u8; 6]) {
        let mut cmd: c::umac_cmd_chg_sta = unsafe { zeroed() };
        cmd.valid_fields = c::CMD_SET_STATION_STA_FLAGS2_VALID;
        cmd.info.mac_addr = *addr;
        cmd.info.sta_flags2 = c::sta_flag_update {
            mask: c::STA_FLAG_AUTHORIZED,
            set: c::STA_FLAG_AUTHORIZED,
        };
        self.send_cmd(&mut cmd).await;
        if let Some(slot) = self.ap.storage.stations.find(addr) {
            self.ap.storage.stations.0[slot].as_mut().unwrap().authorized = true;
        }
    }

    /// Whether the station of `slot` has an association with management frame protection, its keys
    /// in place: frames that would end it have to be protected.
    fn protected_association(&self, slot: usize) -> bool {
        #[cfg(feature = "wpa2")]
        return self.ap.storage.stations.0[slot]
            .is_some_and(|s| s.authorized && s.rsne.is_some_and(|(_, suite)| suite.mfp));
        #[cfg(not(feature = "wpa2"))]
        {
            let _ = slot;
            false
        }
    }

    /// Forgets the station of `slot` and removes it from the RPU, after telling it why if there
    /// is a `reason` (hostapd's `ap_sta_deauthenticate` and `ap_free_sta`). An associated station
    /// is let go first, and removed once it has heard why (see [`Self::let_go`]).
    async fn remove_station(&mut self, slot: usize, reason: Option<u16>) {
        let Some(station) = self.ap.storage.stations.0[slot] else {
            return;
        };
        if let (Some(reason), Phase::Associated, None) = (reason, station.phase, station.leaving) {
            return self.let_go(slot, reason).await;
        }
        self.ap.storage.stations.0[slot] = None;
        self.forget_frames(slot);
        if let Some(reason) = reason {
            let mut frame = [0; 26];
            let len = deauthentication(&self.ap.mac_addr, &station.addr, reason, &mut frame);
            self.send_mgmt(&frame[..len], false).await;
        }
        if station.phase == Phase::Associated {
            let mut cmd: c::umac_cmd_del_sta = unsafe { zeroed() };
            cmd.valid_fields = c::CMD_DEL_STATION_MAC_ADDR_VALID;
            cmd.info.mac_addr = station.addr;
            self.send_cmd(&mut cmd).await;
            self.write_pending_entry(slot, &[0; 6]).await;
        }
        // It may have been the last one a group key renewal waited for.
        #[cfg(feature = "wpa2")]
        self.finish_rekey().await;
    }

    /// Drops what the station of `slot` has under way: its kept frames, its handshake, its SAE
    /// exchange.
    fn forget_frames(&mut self, slot: usize) {
        self.ap.storage.held.drop_station(slot);
        #[cfg(feature = "wpa2")]
        {
            self.ap.storage.handshakes[slot] = None;
            #[cfg(feature = "wpa3")]
            {
                self.ap.storage.exchanges[slot] = None;
            }
        }
    }

    /// Lets the associated station of `slot` go: it loses its port and its frames, and gets a
    /// deauthentication, which it has to hear before it is removed. The RPU sends a management
    /// frame right away, and of two deauthentications to a dozing laptop, one went unheard (the
    /// laptop then stays associated in its own eyes, and the RPU tells nothing of the frames it
    /// still sends). So a dozing station gets its TIM bit set, and the deauthentication once it
    /// wakes up or polls, as mac80211 keeps deauthentications for dozing stations; it is removed on
    /// the acknowledgement, after [`DEAUTH_TRIES`] tries, or after [`LEAVE_TIMEOUT`].
    async fn let_go(&mut self, slot: usize, reason: u16) {
        self.forget_frames(slot);
        let Some(station) = self.ap.storage.stations.get(slot) else {
            return;
        };
        station.authorized = false;
        station.poll_deadline = None;
        station.leaving = Some(Leaving {
            reason,
            sent: false,
            tries: 0,
            deadline: Instant::now() + LEAVE_TIMEOUT,
        });
        #[cfg(feature = "wpa2")]
        {
            station.retry_at = None;
            station.sa_query = None;
        }
        self.tell_leaving(slot).await;
        // It may have been the last one a group key renewal waited for.
        #[cfg(feature = "wpa2")]
        self.finish_rekey().await;
    }

    /// Sends the station of `slot`, being let go, its deauthentication if it is awake or polls;
    /// raises its TIM bit if it dozes.
    async fn tell_leaving(&mut self, slot: usize) {
        let Some(station) = self.ap.storage.stations.get(slot) else {
            return;
        };
        let Some(leaving) = station.leaving.as_mut().filter(|leaving| !leaving.sent) else {
            return;
        };
        if station.asleep && station.service_period == 0 {
            debug!("station {:02x} dozes: its deauthentication waits", station.addr);
            return self.write_pending_bits(slot).await;
        }
        leaving.sent = true;
        leaving.tries += 1;
        station.service_period = 0;
        let (to, reason) = (station.addr, leaving.reason);
        let mut frame = [0; 26];
        let len = deauthentication(&self.ap.mac_addr, &to, reason, &mut frame);
        self.send_mgmt(&frame[..len], false).await;
        self.write_pending_bits(slot).await;
    }

    /// The TX status of a deauthentication: the station being let go goes once it is
    /// acknowledged, or tried often enough; until then it gets it again, when awake.
    async fn deauthentication_sent(&mut self, to: &[u8; 6], acked: bool) {
        let Some(slot) = self.ap.storage.stations.find(to) else {
            return;
        };
        let Some(leaving) = self.ap.storage.stations.0[slot]
            .and_then(|s| s.leaving)
            .filter(|l| l.sent)
        else {
            return;
        };
        if acked || leaving.tries >= DEAUTH_TRIES {
            if acked {
                debug!("station {:02x} heard its deauthentication", to);
            } else {
                info!("station {:02x} did not acknowledge its deauthentication", to);
            }
            return self.remove_station(slot, None).await;
        }
        if let Some(leaving) = self.ap.storage.stations.get(slot).and_then(|s| s.leaving.as_mut()) {
            leaving.sent = false;
        }
        self.tell_leaving(slot).await;
    }

    /// The RPU removed a station on its own (`UMAC_EVENT_DEL_STATION`).
    pub(super) async fn ap_station_removed(&mut self, body: &[u8]) {
        let addr = unsliceit::<c::umac_event_new_station>(body).mac_addr;
        if let Some(slot) = self.ap.storage.stations.find(&addr) {
            if self.ap.storage.stations.0[slot].is_some_and(|s| s.phase == Phase::Associated) {
                info!("station {:02x} removed by the RPU", addr);
                self.ap.storage.stations.0[slot] = None;
                self.forget_frames(slot);
                self.write_pending_entry(slot, &[0; 6]).await;
                #[cfg(feature = "wpa2")]
                self.finish_rekey().await;
            }
        }
    }

    /// Writes station `slot`'s entry of the RPU's table of pending frames, which tells the RPU
    /// what to announce in the TIM (the SDK's `nrf_wifi_fmac_peer_add`): its address, and no
    /// frame pending.
    async fn write_pending_entry(&mut self, slot: usize, addr: &[u8; 6]) {
        let mut entry = [0u32; 3];
        slice8_mut(&mut entry)[..6].copy_from_slice(addr);
        self.write(pending_entry_addr(slot), None, &entry).await;
    }

    /// Updates the pending frame bits of station `slot` (the SDK's `update_pend_q_bmp`): its kept
    /// frames, and its deauthentication if it waits.
    async fn write_pending_bits(&mut self, slot: usize) {
        let mut bits = self.ap.storage.held.categories(slot);
        let leaving = self.ap.storage.stations.0[slot].and_then(|s| s.leaving);
        if leaving.is_some_and(|leaving| !leaving.sent) {
            bits |= 1 << access_category(0);
        }
        self.write32(pending_entry_addr(slot) + 8, None, bits).await;
    }

    /// Takes a frame to send if it is for a station that sleeps, and says whether it did (the
    /// SDK's `tx_enqueue` while the station is in `NRF_WIFI_CLIENT_PS_MODE`).
    ///
    /// The RPU sends group frames right away, and a sleeping station misses them (13 broadcasts
    /// out of 30 reached a dozing laptop). So while a station sleeps, a group frame goes to each
    /// station with keys as a unicast frame instead, its destination replaced by the station's
    /// address (hostapd's `multicast_to_unicast`): a sleeping one gets it when it wakes up.
    pub(super) async fn ap_hold(&mut self, frame: &mut [u32], len: usize) -> bool {
        if !self.ap.running {
            return false;
        }
        let to: [u8; 6] = slice8(frame)[..6].try_into().unwrap();
        if to[0] & 1 != 0 {
            return self.ap_group_frame(frame, len).await;
        }
        let Some(slot) = self.ap.storage.stations.find(&to) else {
            return false;
        };
        // A station being let go gets no more frames.
        if self.ap.storage.stations.0[slot].is_some_and(|s| s.leaving.is_some()) {
            return true;
        }
        self.hold_for(slot, frame, len).await
    }

    /// Sends a group frame as unicast frames, if a station with keys sleeps. Says whether it did.
    async fn ap_group_frame(&mut self, frame: &mut [u32], len: usize) -> bool {
        let stations = self.ap.storage.stations.0;
        if !stations.iter().flatten().any(|s| s.authorized && s.asleep) {
            return false;
        }
        for (slot, station) in stations.iter().enumerate() {
            let Some(station) = station.filter(|s| s.authorized) else {
                continue;
            };
            slice8_mut(frame)[..6].copy_from_slice(&station.addr);
            if !self.hold_for(slot, frame, len).await && self.send_frame(frame, len).await.is_none() {
                debug!("group frame for {:02x} dropped: no TX token free", station.addr);
            }
        }
        true
    }

    /// Keeps a frame for the station of `slot` if it sleeps, or if frames wait for it already.
    /// Says whether it did.
    async fn hold_for(&mut self, slot: usize, frame: &[u32], len: usize) -> bool {
        let Some(station) = self.ap.storage.stations.0[slot] else {
            return false;
        };
        // Frames go out directly to a station that is awake, or within what it asked for.
        if (!station.asleep || station.service_period > 0) && self.ap.storage.held.count(slot) == 0 {
            return false;
        }
        if !self.ap.storage.held.hold(slot, frame, len) {
            debug!("frame for sleeping station {:02x} dropped", station.addr);
        }
        self.write_pending_bits(slot).await;
        true
    }

    /// A station dozes or wakes up (`CMD_PM_MODE`, the SDK's `sap_client_update_pmmode`).
    pub(super) async fn ap_power_save(&mut self, body: &[u8]) {
        let event: &c::sap_client_pwrsave = unsliceit(body);
        let (addr, state) = (event.mac_addr, event.sta_ps_state);
        let Some(slot) = self.ap.storage.stations.find(&addr) else {
            return;
        };
        let station = self.ap.storage.stations.get(slot).unwrap();
        station.asleep = state as u32 == c::CLIENT_PS_MODE;
        station.service_period = 0;
        debug!(
            "station {:02x} {}",
            addr,
            if station.asleep { "dozes" } else { "awake" }
        );
        // The RPU saw a frame of the station's go by: it is there.
        self.station_seen(slot);
        self.tell_leaving(slot).await;
        self.ap_deliver().await;
    }

    /// A sleeping station asks for frames (`CMD_PS_GET_FRAMES`, the SDK's
    /// `sap_client_ps_get_frames`): a PS-Poll, or a U-APSD trigger.
    pub(super) async fn ap_get_frames(&mut self, body: &[u8]) {
        let event: &c::sap_ps_get_frames = unsliceit(body);
        let (addr, count) = (event.mac_addr, event.num_frames);
        let Some(slot) = self.ap.storage.stations.find(&addr) else {
            return;
        };
        self.ap.storage.stations.get(slot).unwrap().service_period = count.max(1) as u8;
        self.station_seen(slot);
        self.tell_leaving(slot).await;
        self.ap_deliver().await;
    }

    /// Sends what the stations may have of their kept frames: everything to one that is awake,
    /// what a sleeping one asked for. Frames that find no TX token free wait for the next call.
    pub(super) async fn ap_deliver(&mut self) {
        if !self.ap.running {
            return;
        }
        for slot in 0..MAX_STATIONS {
            while let Some(station) = self.ap.storage.stations.0[slot] {
                if station.asleep && station.service_period == 0 {
                    break;
                }
                let Some(i) = self.ap.storage.held.oldest(slot) else {
                    if let Some(station) = self.ap.storage.stations.get(slot) {
                        station.service_period = 0;
                    }
                    break;
                };
                let more = self.ap.storage.held.count(slot) > 1;
                let last = !station.asleep || station.service_period == 1 || !more;
                let len = self.ap.storage.held.frames[i].len as usize;
                let mut frame = [0u32; MTU.div_ceil(4)];
                frame[..len.div_ceil(4)].copy_from_slice(&self.ap.storage.held.frames[i].frame[..len.div_ceil(4)]);
                if self.send_frame_flags(&frame, len, more, last).await.is_none() {
                    return;
                }
                self.ap.storage.held.frames[i].slot = None;
                if let Some(station) = self.ap.storage.stations.get(slot) {
                    station.service_period = station.service_period.saturating_sub(1);
                }
                self.write_pending_bits(slot).await;
            }
        }
    }

    /// When [`Self::ap_check_timeouts`] has to run at the latest: the next handshake message to
    /// send again, station to poll, or poll to give up on.
    pub(super) fn ap_deadline(&self) -> Option<Instant> {
        if !self.ap.running {
            return None;
        }
        #[cfg(feature = "wpa2")]
        let rekey = self.ap.rekey_at.filter(|_| self.ap.next_gtk.is_none());
        #[cfg(not(feature = "wpa2"))]
        let rekey = None;
        let stations = self
            .ap
            .storage
            .stations
            .0
            .iter()
            .flatten()
            .filter(|s| s.phase == Phase::Associated);
        stations
            .flat_map(|s| {
                if let Some(leaving) = s.leaving {
                    return [Some(leaving.deadline), None, None];
                }
                #[cfg(feature = "wpa2")]
                let retry = s.retry_at;
                #[cfg(not(feature = "wpa2"))]
                let retry = None;
                #[cfg(feature = "wpa2")]
                let query = s.sa_query.map(|query| query.retry_at.min(query.deadline));
                #[cfg(not(feature = "wpa2"))]
                let query = None;
                [retry, query, Some(s.poll_deadline.unwrap_or(s.last_seen + INACTIVITY))]
            })
            .chain([rekey])
            .flatten()
            .min()
    }

    /// Sends again the handshake messages that went unanswered for too long, and lets go of the
    /// stations that never answered; polls the stations not heard from for [`INACTIVITY`], and
    /// lets go of those that do not acknowledge the poll (hostapd's `ap_handle_timer`).
    pub(super) async fn ap_check_timeouts(&mut self) {
        if !self.ap.running {
            return;
        }
        let now = Instant::now();
        #[cfg(feature = "wpa2")]
        if self.ap.next_gtk.is_none() && self.ap.rekey_at.is_some_and(|at| now >= at) {
            self.start_rekey().await;
        }
        #[cfg(feature = "wpa2")]
        self.check_sa_queries(now).await;
        for slot in 0..MAX_STATIONS {
            let Some(station) = self.ap.storage.stations.0[slot].filter(|s| s.phase == Phase::Associated) else {
                continue;
            };
            if let Some(leaving) = station.leaving {
                if now >= leaving.deadline {
                    info!("station {:02x} did not hear its deauthentication in time", station.addr);
                    self.remove_station(slot, None).await;
                }
                continue;
            }
            #[cfg(feature = "wpa2")]
            if station.retry_at.is_some_and(|at| now >= at) {
                self.send_handshake_message(slot).await;
                continue;
            }
            match station.poll_deadline {
                Some(deadline) if now >= deadline => {
                    info!("station {:02x} did not answer its poll, letting it go", station.addr);
                    self.remove_station(slot, Some(REASON_INACTIVITY)).await;
                }
                None if station.authorized && now >= station.last_seen + INACTIVITY => {
                    debug!(
                        "station {:02x} not heard from for {} s, polling it",
                        station.addr,
                        INACTIVITY.as_secs()
                    );
                    self.poll_station(slot).await;
                }
                _ => {}
            }
        }
    }

    /// Sends a station a frame it has to acknowledge (hostapd's `poll_client`, which the SDK's
    /// driver does not have): an empty frame of an experimental ethertype, which its stack drops.
    /// It goes through the frames kept for sleeping stations, so a dozing one gets it too.
    async fn poll_station(&mut self, slot: usize) {
        let Some(station) = self.ap.storage.stations.get(slot) else {
            return;
        };
        station.poll_deadline = Some(Instant::now() + POLL_TIMEOUT);
        let to = station.addr;
        let mut frame = [0u32; 4];
        let bssid = self.ap.mac_addr;
        let Some(len) = crate::write_ethernet(slice8_mut(&mut frame), &to, &bssid, ETHERTYPE_POLL, &[]) else {
            return;
        };
        if !self.hold_for(slot, &frame, len).await && self.send_frame(&frame, len).await.is_none() {
            debug!("poll of {:02x} not sent: no TX token free", to);
        }
    }

    /// Notes the station a frame goes to with TX token `token`.
    pub(super) fn ap_frame_queued(&mut self, token: usize, to: &[u8]) {
        if !self.ap.running {
            return;
        }
        let to: [u8; 6] = to.try_into().unwrap_or([0xFF; 6]);
        if let Some(entry) = self.ap.token_stations.get_mut(token) {
            *entry = self.ap.storage.stations.find(&to).map(|slot| slot as u8);
        }
    }

    /// The frames of TX token `token` are done: if acknowledged, their station is there.
    pub(super) fn ap_frame_done(&mut self, token: usize, acked: bool) {
        let slot = self.ap.token_stations.get_mut(token).and_then(Option::take);
        if let (Some(slot), true) = (slot, acked) {
            self.station_seen(slot as usize);
        }
    }

    /// A frame came in: its transmitter, if a station, is there. `frame` is as the RPU hands it
    /// over, with the 802.11 header (whose second address is the transmitter) or as an A-MSDU
    /// subframe (whose source is the station).
    pub(super) fn ap_seen(&mut self, frame: &[u8], pkt_type: u32) {
        if !self.ap.running {
            return;
        }
        let from = match pkt_type {
            c::PKT_TYPE_MPDU | c::PKT_TYPE_MSDU_WITH_MAC => frame.get(10..16),
            c::PKT_TYPE_MSDU => frame.get(6..12),
            _ => None,
        };
        let slot = from
            .and_then(|from| from.try_into().ok())
            .and_then(|from: [u8; 6]| self.ap.storage.stations.find(&from));
        if let Some(slot) = slot {
            self.station_seen(slot);
        }
    }

    fn station_seen(&mut self, slot: usize) {
        if let Some(station) = self.ap.storage.stations.get(slot) {
            station.last_seen = Instant::now();
            station.poll_deadline = None;
        }
    }

    /// Handles events until `done` holds, or `timeout` passes. Returns whether `done` held.
    async fn wait_until(&mut self, timeout: Duration, done: impl Fn(&Self) -> bool) -> bool {
        let mut buf = [0u32; crate::MAX_EVENT_LEN / 4];
        embassy_time::with_timeout(timeout, async {
            while !done(self) {
                let len = self.next_event(&mut buf).await;
                self.handle_event(slice8(&buf), len).await;
            }
        })
        .await
        .is_ok()
    }
}

/// How long a handshake message waits for its answer before it goes out again (hostapd's
/// `eapol_key_timeout_subseq`).
#[cfg(feature = "wpa2")]
const HANDSHAKE_RETRY: Duration = Duration::from_secs(1);

/// How often the group key is renewed (hostapd's `wpa_group_rekey` for CCMP).
#[cfg(feature = "wpa2")]
const GROUP_REKEY: Duration = Duration::from_secs(24 * 3600);

#[cfg(feature = "wpa2")]
impl Control<'_> {
    /// Starts a WPA2-Personal access point named `ssid` on `channel`, with the passphrase its
    /// stations join with: PSK key management, CCMP as group and pairwise cipher. Otherwise as
    /// [`Control::start_ap_open`].
    ///
    /// The 4-way handshake with each station runs in the driver, which needs random numbers for
    /// its ANonces and the group key. They come from `rng`, as for [`Control::join_wpa2`]: 32
    /// bytes, before starting. Deriving the key from the passphrase takes as long as for a join.
    pub async fn start_ap_wpa2(
        &mut self,
        ssid: &[u8],
        channel: u8,
        passphrase: &[u8],
        rng: &mut (impl rand_core::CryptoRng + ?Sized),
    ) -> Result<(), ApError> {
        let settings = Settings::new(ssid, channel)?;
        let psk = crate::wpa2_psk(ssid, passphrase).ok_or(ApError::InvalidPassphrase)?;
        let mut seed = [0; 32];
        rng.fill_bytes(&mut seed);
        let settings = Settings {
            security: Security::Wpa2 { psk, seed },
            ..settings
        };
        self.request_ap(settings).await
    }
}

#[cfg(feature = "wpa3")]
impl Control<'_> {
    /// Starts a WPA3-Personal access point named `ssid` on `channel`, with the password its
    /// stations join with: SAE on the P-256 curve, by hash to element or hunting and pecking as each
    /// station chooses, then the 4-way handshake, management frame protection required. Otherwise
    /// as [`Control::start_ap_wpa2`], whose random numbers this takes from `rng` too.
    ///
    /// Hunting and pecking takes 0.39 s for each station on an nRF5340 at 128 MHz, during which
    /// the runner does nothing else; hash to element takes a few milliseconds.
    pub async fn start_ap_wpa3(
        &mut self,
        ssid: &[u8],
        channel: u8,
        password: &[u8],
        rng: &mut (impl rand_core::CryptoRng + ?Sized),
    ) -> Result<(), ApError> {
        self.start_ap_sae(ssid, channel, password, false, rng).await
    }

    /// Starts a WPA2/WPA3 transition access point: WPA3 stations join with SAE, as with
    /// [`Control::start_ap_wpa3`], and WPA2 ones with the same passphrase (PSK or PSK-SHA256),
    /// management frame protection for those that are capable of it.
    pub async fn start_ap_wpa2_wpa3(
        &mut self,
        ssid: &[u8],
        channel: u8,
        passphrase: &[u8],
        rng: &mut (impl rand_core::CryptoRng + ?Sized),
    ) -> Result<(), ApError> {
        self.start_ap_sae(ssid, channel, passphrase, true, rng).await
    }

    async fn start_ap_sae(
        &mut self,
        ssid: &[u8],
        channel: u8,
        password: &[u8],
        transition: bool,
        rng: &mut (impl rand_core::CryptoRng + ?Sized),
    ) -> Result<(), ApError> {
        let settings = Settings::new(ssid, channel)?;
        let psk = match transition {
            true => Some(crate::wpa2_psk(ssid, password).ok_or(ApError::InvalidPassphrase)?),
            false => None,
        };
        let mut seed = [0; 32];
        rng.fill_bytes(&mut seed);
        let wpa3 = crate::wpa3::Wpa3::new(ssid, password, None, seed).ok_or(ApError::InvalidPassphrase)?;
        let settings = Settings {
            security: Security::Wpa3 { wpa3, psk },
            ..settings
        };
        self.request_ap(settings).await
    }
}

/// The runner's side of a WPA2 access point: the 4-way handshake with each station.
#[cfg(feature = "wpa2")]
impl<BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> Runner<'_, BUS, IN, OUT> {
    /// Starts the 4-way handshake with the station of `slot`, just added to the RPU (hostapd's
    /// `wpa_auth_sm_event(WPA_ASSOC)`).
    async fn start_handshake(&mut self, slot: usize) {
        let (Some(security), Some(station)) = (
            self.ap.settings.map(|settings| settings.security),
            self.ap.storage.stations.0[slot],
        ) else {
            return;
        };
        let (Some(seed), Some(offer), Some((sta_rsne, suite))) = (security.seed(), security.offer(), station.rsne)
        else {
            return;
        };
        // The PMK: the SAE exchange's with SAE, which message 1 names, else the pre-shared key.
        #[cfg(feature = "wpa3")]
        let (pmk, pmkid) = match station.pmksa {
            Some(pmksa) if suite.akm == supplicant::Akm::Sae => (Some(pmksa.pmk), Some(pmksa.pmkid)),
            _ => (security.psk(), None),
        };
        #[cfg(not(feature = "wpa3"))]
        let (pmk, pmkid) = (security.psk(), None);
        let Some(pmk) = pmk else {
            return;
        };
        self.ap.nonces += 1;
        let aa = self.ap.mac_addr;
        let anonce = authenticator::anonce(&seed, &aa, &station.addr, self.ap.nonces);
        let ap_rsne = Rsne::access_point(offer);
        debug!("4-way handshake with {:02x}: {}", station.addr, suite);
        let handshake = Authenticator::new(pmk, aa, station.addr, ap_rsne, sta_rsne, suite, anonce)
            .with_rsnxe(security.rsnxe(), station.rsnxe)
            .with_pmkid(pmkid);
        self.ap.storage.handshakes[slot] = Some(handshake);
        self.send_handshake_message(slot).await;
    }

    /// Sends the handshake message the station of `slot` is due, the first time or again, or
    /// lets the station go after [`authenticator::TRIES`] unanswered ones (reason 15, or 16 for a
    /// group key handshake).
    async fn send_handshake_message(&mut self, slot: usize) {
        let (Some(gtk), Some(station)) = (self.ap.gtk, self.ap.storage.stations.0[slot]) else {
            return;
        };
        let Some(handshake) = self.ap.storage.handshakes[slot].as_mut() else {
            return;
        };
        let group = handshake.in_group_handshake();
        let mut out = [0; authenticator::MESSAGE_MAX];
        let Some(len) = handshake.next_message(&gtk, &mut out) else {
            if group {
                info!("station {:02x} did not take the new group key", station.addr);
                self.remove_station(slot, Some(REASON_GROUP_KEY_UPDATE_TIMEOUT)).await;
            } else {
                info!("station {:02x} did not complete the 4-way handshake", station.addr);
                // Maybe the PMK it named is not the one it has.
                #[cfg(feature = "wpa3")]
                self.ap.storage.pmksa_cache.remove(&station.addr);
                self.remove_station(slot, Some(REASON_4WAY_HANDSHAKE_TIMEOUT)).await;
            }
            return;
        };
        if let Some(station) = self.ap.storage.stations.get(slot) {
            station.retry_at = Some(Instant::now() + HANDSHAKE_RETRY);
        }
        self.send_eapol_to(&station.addr, &out[..len]).await;
    }

    /// Takes an EAPOL frame from a station, as an Ethernet frame (hostapd's `wpa_receive`).
    pub(super) async fn ap_eapol(&mut self, frame: &[u8]) {
        let from: [u8; 6] = frame[6..12].try_into().unwrap();
        let (Some(slot), Some(gtk)) = (self.ap.storage.stations.find(&from), self.ap.gtk) else {
            debug!(
                "EAPOL frame from {:02x} ignored: not a station of the access point",
                from
            );
            return;
        };
        let mut out = [0; authenticator::MESSAGE_MAX];
        let Some(handshake) = self.ap.storage.handshakes[slot].as_mut() else {
            return;
        };
        match handshake.handle(&frame[14..], &gtk, &mut out) {
            authenticator::Outcome::Ignored => {}
            authenticator::Outcome::Send(len) => {
                debug!("4-way handshake with {:02x}: message 2, sending message 3", from);
                if let Some(station) = self.ap.storage.stations.get(slot) {
                    station.retry_at = Some(Instant::now() + HANDSHAKE_RETRY);
                }
                self.send_eapol_to(&from, &out[..len]).await;
            }
            authenticator::Outcome::Done { tk } => {
                if let Some(station) = self.ap.storage.stations.get(slot) {
                    station.retry_at = None;
                }
                // The pairwise key, then the port (hostapd's PTKINITDONE).
                self.add_key(Some(from), supplicant::CIPHER_SUITE_CCMP, 0, &tk, &[0; 6])
                    .await;
                self.set_default_key(0, DefaultKey::Unicast).await;
                self.authorize_station(&from).await;
                info!("station {:02x} has its keys", from);
                // The authenticator stays, for the group key handshakes. One that joined while the
                // group key is renewed got the one in use, and gets the next one now.
                if let Some(next) = self.ap.next_gtk {
                    self.start_group_handshake(slot, next).await;
                }
            }
            authenticator::Outcome::GroupDone => {
                if let Some(station) = self.ap.storage.stations.get(slot) {
                    station.retry_at = None;
                }
                debug!("station {:02x} has the new group key", from);
                self.finish_rekey().await;
            }
            authenticator::Outcome::Refused => {
                warn!(
                    "station {:02x} contradicts its association request, letting it go",
                    from
                );
                #[cfg(feature = "wpa3")]
                self.ap.storage.pmksa_cache.remove(&from);
                self.remove_station(slot, Some(REASON_IE_IN_4WAY_DIFFERS)).await;
            }
        }
    }

    /// Starts renewing the group key (hostapd's `wpa_group_setkeys`): the next one goes to the RPU,
    /// for reception until it is in use, and to each station with its keys in a group key
    /// handshake.
    async fn start_rekey(&mut self) {
        let (Some(seed), Some(_)) = (self.ap.settings.and_then(|s| s.security.seed()), self.ap.gtk) else {
            return;
        };
        self.ap.gtks += 1;
        let next = GroupKeys::derive(&seed, &self.ap.mac_addr, self.ap.gtks);
        self.install_group_keys(&next).await;
        self.ap.next_gtk = Some(next);
        debug!(
            "renewing the group keys: keys {} and {}",
            next.gtk.index, next.igtk.index
        );
        for slot in 0..MAX_STATIONS {
            if self.ap.storage.stations.0[slot].is_some_and(|s| s.authorized) {
                self.start_group_handshake(slot, next).await;
            }
        }
        self.finish_rekey().await;
    }

    async fn start_group_handshake(&mut self, slot: usize, next: GroupKeys) {
        let started = self.ap.storage.handshakes[slot]
            .as_mut()
            .is_some_and(|handshake| handshake.start_group(next));
        if started {
            self.send_handshake_message(slot).await;
        }
    }

    /// Puts the next group key in use once no station is still being given it (hostapd's
    /// `wpa_group_setkeysdone`): broadcasts go out with it from then on.
    async fn finish_rekey(&mut self) {
        let Some(next) = self.ap.next_gtk else {
            return;
        };
        let pending = self
            .ap
            .storage
            .handshakes
            .iter()
            .flatten()
            .any(Authenticator::in_group_handshake);
        if pending {
            return;
        }
        self.set_default_key(next.gtk.index, DefaultKey::Multicast).await;
        self.set_default_key(next.igtk.index, DefaultKey::Management).await;
        self.ap.gtk = Some(next);
        self.ap.next_gtk = None;
        self.ap.rekey_at = Some(Instant::now() + GROUP_REKEY);
        info!("group keys {} and {} in use", next.gtk.index, next.igtk.index);
    }

    /// Gives the RPU group keys, which it receives with until one is made the default: the GTK,
    /// and the IGTK for BIP-CMAC-128.
    async fn install_group_keys(&mut self, keys: &GroupKeys) {
        let (gtk, igtk) = (keys.gtk, keys.igtk);
        self.add_key(None, supplicant::CIPHER_SUITE_CCMP, gtk.index, &gtk.key, &[0; 6])
            .await;
        self.add_key(
            None,
            supplicant::CIPHER_SUITE_BIP_CMAC_128,
            igtk.index,
            &igtk.key,
            &[0; 6],
        )
        .await;
    }

    /// Starts an SA Query of the association of the station of `slot`, if none is under way: a
    /// protected request, which only the station that has the keys can answer.
    async fn start_sa_query(&mut self, slot: usize) {
        let now = Instant::now();
        let id = (self.ap.cookie as u16).to_le_bytes();
        let Some(station) = self.ap.storage.stations.get(slot).filter(|s| s.sa_query.is_none()) else {
            return;
        };
        station.sa_query = Some(SaQuery {
            id,
            retry_at: now + SA_QUERY_RETRY,
            deadline: now + Duration::from_micros(SA_QUERY_TIMEOUT_TU as u64 * 1024),
        });
        let to = station.addr;
        self.send_sa_query(&to, SA_QUERY_REQUEST, id).await;
    }

    async fn send_sa_query(&mut self, to: &[u8; 6], action: u8, id: [u8; 2]) {
        let mut frame = [0; 28];
        let len = sa_query(&self.ap.mac_addr, to, action, id, &mut frame);
        self.send_mgmt(&frame[..len], false).await;
    }

    /// Sends the requests of the SA Queries again, and ends the ones that went unanswered: those
    /// stations' next association request is taken.
    async fn check_sa_queries(&mut self, now: Instant) {
        for slot in 0..MAX_STATIONS {
            let Some(station) = self.ap.storage.stations.get(slot) else {
                continue;
            };
            let Some(query) = station.sa_query.as_mut() else {
                continue;
            };
            if now >= query.deadline {
                station.sa_query = None;
                station.sa_query_timed_out = true;
                info!(
                    "station {:02x} did not answer the SA Query: its next association request is taken",
                    station.addr
                );
            } else if now >= query.retry_at {
                query.retry_at = now + SA_QUERY_RETRY;
                let (to, id) = (station.addr, query.id);
                self.send_sa_query(&to, SA_QUERY_REQUEST, id).await;
            }
        }
    }

    /// An SA Query action frame from a station (hostapd's `ieee802_11_sa_query_action`): a request,
    /// which gets its response, or the response to the access point's request, which keeps the
    /// station's association. Only protected ones count.
    async fn sa_query_frame(&mut self, mgmt: &Mgmt<'_>) {
        let [CATEGORY_SA_QUERY, action, id0, id1, ..] = *mgmt.body else {
            return;
        };
        let Some(slot) = self.ap.storage.stations.find(&mgmt.from) else {
            return;
        };
        if !self.protected_association(slot) || !mgmt.protected {
            debug!("SA Query from {:02x} ignored: not protected", mgmt.from);
            return;
        }
        match action {
            SA_QUERY_REQUEST => self.send_sa_query(&mgmt.from, SA_QUERY_RESPONSE, [id0, id1]).await,
            SA_QUERY_RESPONSE => {
                let Some(station) = self.ap.storage.stations.get(slot) else {
                    return;
                };
                if station.sa_query.is_some_and(|query| query.id == [id0, id1]) {
                    station.sa_query = None;
                    info!(
                        "station {:02x} answered the SA Query: it keeps its association",
                        mgmt.from
                    );
                }
            }
            _ => {}
        }
    }

    /// Sends an EAPOL frame to a station, from the access point, through the frames kept for
    /// sleeping stations: a dozing one gets it when the TIM wakes it up (sent right away, the
    /// group key handshakes of a laptop in power save got lost). A frame that finds no TX token
    /// free is lost, and goes out again on the handshake's timeout.
    async fn send_eapol_to(&mut self, to: &[u8; 6], eapol: &[u8]) {
        let mut frame = [0u32; (14 + authenticator::MESSAGE_MAX).div_ceil(4)];
        let bssid = self.ap.mac_addr;
        let Some(len) = crate::write_ethernet(slice8_mut(&mut frame), to, &bssid, supplicant::ETHERTYPE_EAPOL, eapol)
        else {
            return;
        };
        let held = match self.ap.storage.stations.find(to) {
            Some(slot) => self.hold_for(slot, &frame, len).await,
            None => false,
        };
        if !held && self.send_frame(&frame, len).await.is_none() {
            warn!("EAPOL frame for {:02x} not sent: no TX token free", to);
        }
    }
}

/// The frame of a management frame event or TX status.
fn mlme_frame(event: &c::umac_event_mlme) -> &[u8] {
    let len = { event.frame.frame_len }.clamp(0, 400) as usize;
    // The event is packed: the frame's bytes are reached through a raw pointer, and are C chars,
    // the same bytes.
    let bytes = core::ptr::addr_of!(event.frame.frame) as *const u8;
    unsafe { core::slice::from_raw_parts(bytes, len) }
}

/// Where station `slot`'s entry of the RPU's table of pending frames is.
fn pending_entry_addr(slot: usize) -> u32 {
    c::RPU_MEM_UMAC_PEND_Q_BMP + (slot * size_of::<c::sap_client_pend_frames_bitmap>()) as u32
}

/// A 20 MHz channel, HT allowed.
fn freq_params(frequency: u32) -> c::freq_params {
    c::freq_params {
        valid_fields: c::SET_FREQ_PARAMS_FREQ_VALID
            | c::SET_FREQ_PARAMS_CHANNEL_WIDTH_VALID
            | c::SET_FREQ_PARAMS_CENTER_FREQ1_VALID
            | c::SET_FREQ_PARAMS_CENTER_FREQ2_VALID
            | c::SET_FREQ_PARAMS_CHANNEL_TYPE_VALID,
        frequency: frequency as _,
        channel_width: c::chan_width::CHAN_WIDTH_20 as _,
        center_frequency1: frequency as _,
        center_frequency2: 0,
        channel_type: c::channel_type::CHAN_HT20 as _,
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::assert_eq;
    use std::vec::Vec;

    use super::*;

    const BSSID: [u8; 6] = [0xF4, 0xCE, 0x36, 0x00, 0x8B, 0x19];
    const STA: [u8; 6] = [0x98, 0x43, 0xFA, 0x23, 0x26, 0x25];

    // The elements every beacon and probe response ends with.
    const HT_CAPABILITIES_IE: &str = "2d1a 2c00 17 ff000000000000000000 0000 01000000 0000 00000000 00";
    const WMM_IE: &str = "dd18 0050f2020101 0000 03a40000 27a40000 42435e00 62322f00";

    fn ht_operation_ie(channel: u8) -> std::string::String {
        std::format!("3d16 {channel:02x} 00 0000 0000 00000000000000000000000000000000")
    }

    fn tail_2g(channel: u8) -> Vec<u8> {
        hex(&std::format!(
            "2a0100 3204 3048606c {HT_CAPABILITIES_IE} {} {WMM_IE}",
            ht_operation_ie(channel)
        ))
    }

    fn hex(s: &str) -> Vec<u8> {
        let s: std::string::String = s.split_whitespace().collect();
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn channels_map_to_frequencies() {
        assert_eq!(channel_frequency(1), Some(2412));
        assert_eq!(channel_frequency(13), Some(2472));
        assert_eq!(channel_frequency(14), None);
        assert_eq!(channel_frequency(36), Some(5180));
        assert_eq!(channel_frequency(48), Some(5240));
        assert_eq!(channel_frequency(40), Some(5200));
        assert_eq!(channel_frequency(38), None);
        // Radar detection needed: left out.
        assert_eq!(channel_frequency(52), None);
        assert_eq!(channel_frequency(100), None);
        assert_eq!(channel_frequency(149), Some(5745));
        assert_eq!(channel_frequency(165), Some(5825));
        assert_eq!(channel_frequency(177), Some(5885));
        assert_eq!(channel_frequency(150), None);
        assert_eq!(channel_frequency(0), None);
    }

    #[test]
    fn settings_check_the_ssid_and_channel() {
        assert!(Settings::new(b"nrf70-ap", 6).is_ok());
        assert_eq!(Settings::new(b"", 6).err(), Some(ApError::InvalidSsid));
        assert_eq!(Settings::new(&[b'a'; 33], 6).err(), Some(ApError::InvalidSsid));
        assert_eq!(Settings::new(b"nrf70-ap", 52).err(), Some(ApError::InvalidChannel));
    }

    #[test]
    fn a_2g_beacon_is_split_around_the_tim() {
        let settings = Settings::new(b"nrf70-ap", 6).unwrap();
        let mut head = [0; 256];
        let len = settings.beacon_head(&BSSID, &mut head);
        assert_eq!(
            head[..len],
            hex("8000 0000 ffffffffffff f4ce36008b19 f4ce36008b19 0000
                 0000000000000000 6400 2104
                 0008 6e726637302d6170
                 0108 82848b960c121824
                 030106")[..]
        );
        let mut tail = [0; 512];
        let len = settings.beacon_tail(&mut tail);
        assert_eq!(tail[..len], tail_2g(6)[..]);
    }

    #[test]
    fn a_5g_beacon_has_ofdm_rates_only() {
        let settings = Settings::new(b"nrf70-ap", 36).unwrap();
        let mut head = [0; 256];
        let len = settings.beacon_head(&BSSID, &mut head);
        // ESS and short preamble, no short slot bit, no DS parameter set.
        assert_eq!(head[34..36], [0x21, 0x00]);
        assert_eq!(head[46..len], hex("0108 8c129824b048606c")[..]);
        // No ERP nor extended rates, 802.11n and WMM all the same.
        let mut tail = [0; 512];
        let len = settings.beacon_tail(&mut tail);
        assert_eq!(
            tail[..len],
            hex(&std::format!("{HT_CAPABILITIES_IE} {} {WMM_IE}", ht_operation_ie(36)))[..]
        );
    }

    #[test]
    fn probe_requests_for_the_network_or_any_are_answered() {
        let settings = Settings::new(b"nrf70-ap", 6).unwrap();
        assert!(settings.probed(&hex("0000 0108 82848b960c121824")));
        assert!(settings.probed(&hex("0008 6e726637302d6170")));
        assert!(!settings.probed(&hex("0005 6f74686572")));
        assert!(!settings.probed(&hex("0108 82848b960c121824")));

        let mut out = [0; 400];
        let len = settings.probe_response(&BSSID, &STA, &mut out);
        assert_eq!(out[..4], [0x50, 0, 0, 0]);
        assert_eq!(out[4..10], STA);
        // The head of the beacon after the header, then its tail.
        let mut head = [0; 256];
        let head_len = settings.beacon_head(&BSSID, &mut head);
        assert_eq!(out[24..head_len], head[24..head_len]);
        assert_eq!(out[head_len..len], tail_2g(6)[..]);
    }

    #[test]
    fn authentication_and_association_responses() {
        let mut out = [0; 400];
        let len = authentication_response(&BSSID, &STA, 0, STATUS_SUCCESS, &mut out);
        assert_eq!(
            out[..len],
            hex("b000 0000 9843fa232625 f4ce36008b19 f4ce36008b19 0000 0000 0200 0000")[..]
        );

        let settings = Settings::new(b"nrf70-ap", 6).unwrap();
        // An 802.11g station without WMM.
        let legacy = settings
            .association_request(&hex("3104 0a00 0008 6e726637302d6170 0108 02040b160c121824"))
            .ok()
            .unwrap();
        let len = settings.association_response(&BSSID, &STA, false, Ok((1, &legacy)), &mut out);
        assert_eq!(
            out[..len],
            hex("1000 0000 9843fa232625 f4ce36008b19 f4ce36008b19 0000
                 2104 0000 01c0 0108 82848b960c121824 3204 3048606c")[..]
        );
        // An 802.11n station with WMM gets the HT and WMM elements.
        let modern = settings
            .association_request(&hex(&std::format!(
                "3104 0a00 0008 6e726637302d6170 0108 02040b160c121824 {HT_CAPABILITIES_IE} dd07 0050f202000100"
            )))
            .ok()
            .unwrap();
        let len = settings.association_response(&BSSID, &STA, false, Ok((2, &modern)), &mut out);
        assert_eq!(
            out[..len],
            hex(&std::format!(
                "1000 0000 9843fa232625 f4ce36008b19 f4ce36008b19 0000
                 2104 0000 02c0 0108 82848b960c121824 3204 3048606c {HT_CAPABILITIES_IE} {} {WMM_IE}",
                ht_operation_ie(6)
            ))[..]
        );
        let len = settings.association_response(&BSSID, &STA, true, Err(STATUS_RATES), &mut out);
        assert_eq!(out[0], REASSOC_RESP);
        assert_eq!(out[24..30], hex("2104 1200 0000")[..]);
        assert_eq!(len, 30 + 10 + 6);
    }

    #[test]
    fn association_requests_are_checked() {
        let settings = Settings::new(b"nrf70-ap", 6).unwrap();
        let request = settings
            .association_request(&hex(
                "3104 0a00 0008 6e726637302d6170 0108 02040b160c121824 3204 30486c60 dd07 0050f202000100",
            ))
            .ok()
            .unwrap();
        assert_eq!(request.capability, 0x0431);
        assert_eq!(request.listen_interval, 10);
        assert!(request.wmm);
        assert!(request.ht_capabilities.is_none());
        assert_eq!(
            request.rates[..request.rates_len],
            [0x02, 0x04, 0x0B, 0x16, 0x0C, 0x12, 0x18, 0x24, 0x30, 0x48, 0x6C, 0x60]
        );
        // HT capabilities, and WMM after another vendor element (WPS).
        let request = settings
            .association_request(&hex(&std::format!(
                "3104 0a00 0008 6e726637302d6170 0108 02040b160c121824 {HT_CAPABILITIES_IE} dd05 0050f20410 dd07 0050f202000100"
            )))
            .ok()
            .unwrap();
        assert_eq!(request.ht_capabilities, Some(HT_CAPABILITIES));
        assert!(request.wmm);
        // A WMM parameter element is not a station's information element.
        let request = settings
            .association_request(&hex(&std::format!(
                "3104 0a00 0008 6e726637302d6170 0108 02040b160c121824 {WMM_IE}"
            )))
            .ok()
            .unwrap();
        assert!(!request.wmm);
        // Another network, no rates, cut short.
        assert_eq!(
            settings
                .association_request(&hex("3104 0a00 0005 6f74686572 0108 02040b160c121824"))
                .err(),
            Some(STATUS_UNSPECIFIED)
        );
        assert_eq!(
            settings
                .association_request(&hex("3104 0a00 0008 6e726637302d6170"))
                .err(),
            Some(STATUS_RATES)
        );
        assert_eq!(
            settings.association_request(&hex("3104 0a")).err(),
            Some(STATUS_UNSPECIFIED)
        );
    }

    #[test]
    fn management_frames_are_parsed() {
        let frame = hex("b000 3a01 f4ce36008b19 9843fa232625 f4ce36008b19 1000 0000 0100 0000");
        let mgmt = Mgmt::parse(&frame).unwrap();
        assert_eq!(mgmt.subtype, AUTH);
        assert_eq!(mgmt.to, BSSID);
        assert_eq!(mgmt.from, STA);
        assert_eq!(mgmt.bssid, BSSID);
        assert_eq!(
            (mgmt.le16(0), mgmt.le16(2), mgmt.le16(4), mgmt.le16(6)),
            (Some(0), Some(1), Some(0), None)
        );
        // A data frame, and one cut short.
        assert!(Mgmt::parse(&hex("0801 0000 f4ce36008b19 9843fa232625 f4ce36008b19 0000")).is_none());
        assert!(Mgmt::parse(&frame[..20]).is_none());
    }

    #[test]
    fn stations_get_the_first_free_slot() {
        let mut stations = Stations::new();
        let addr = |n: u8| [2, 0, 0, 0, 0, n];
        for n in 0..MAX_STATIONS as u8 {
            assert_eq!(stations.find_or_add(&addr(n)), Some(n as usize));
        }
        assert_eq!(stations.find_or_add(&addr(9)), None);
        assert_eq!(stations.find_or_add(&addr(2)), Some(2));
        stations.0[1] = None;
        assert_eq!(stations.find(&addr(1)), None);
        assert_eq!(stations.find_or_add(&addr(9)), Some(1));
    }

    fn ethernet(to: u8, tos: u8) -> ([u32; MTU.div_ceil(4)], usize) {
        let mut frame = [0u32; MTU.div_ceil(4)];
        let bytes = slice8_mut(&mut frame);
        bytes[..6].copy_from_slice(&[2, 0, 0, 0, 0, to]);
        bytes[12..16].copy_from_slice(&[0x08, 0x00, 0x45, tos]);
        (frame, 60)
    }

    #[test]
    fn frames_are_kept_per_station_oldest_first() {
        let mut held = HeldFrames::new();
        let (best_effort, len) = ethernet(1, 0x00);
        let (voice, _) = ethernet(1, 0xE0);
        assert!(held.hold(0, &best_effort, len));
        assert!(held.hold(1, &best_effort, len));
        assert!(held.hold(0, &voice, len));
        assert_eq!(held.count(0), 2);
        assert_eq!(held.categories(0), 0b1010);
        assert_eq!(held.categories(1), 0b0010);
        let first = held.oldest(0).unwrap();
        assert_eq!(held.frames[first].seq, 0);

        // Full: nothing is replaced, the frame is refused.
        assert!(held.has_room());
        assert!(held.hold(1, &voice, len));
        assert!(!held.has_room());
        assert!(!held.hold(1, &voice, len));
        assert_eq!(held.count(0), 2);
        assert_eq!(held.count(1), 2);
        assert_eq!(held.categories(1), 0b1010);

        held.drop_station(0);
        assert_eq!(held.count(0), 0);
        assert_eq!(held.categories(0), 0);
        assert!(held.has_room());
        assert!(held.hold(2, &voice, len));
    }

    #[test]
    fn priorities_map_to_access_categories() {
        assert_eq!([0, 1, 2, 3, 4, 5, 6, 7].map(access_category), [1, 0, 0, 1, 2, 2, 3, 3]);
    }
}
