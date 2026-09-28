#![no_std]
#![deny(unused_must_use)]
#![allow(async_fn_in_trait)]

use core::mem::{align_of, size_of, zeroed};
use core::slice;

use align_data::{include_aligned, Align16};
use defmt::{assert, panic, todo, unwrap, *};
use embassy_futures::select::{select, Either};
use embassy_net_driver_channel as ch;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::Operation;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;
use regions::*;

#[allow(unused)]
#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
mod c {
    include!("../fw/bindings.rs");
    pub const RPU_MCU_CORE_INDIRECT_BASE: u32 = 0xC0000000;
    pub const RX_BUF_HEADROOM: u32 = 4;
}

const MTU: usize = 1514;

/// How long the RPU gets to answer the init command and interface changes.
const EVENT_TIMEOUT: Duration = Duration::from_secs(5);
/// A scan covers every 2.4 GHz and 5 GHz channel, which takes a few seconds.
const SCAN_TIMEOUT: Duration = Duration::from_secs(30);
/// The RPU pulses HOST_IRQ once per event. Poll this often as well, in case an edge is missed.
const IRQ_POLL_PERIOD: Duration = Duration::from_millis(50);
/// Scan results buffered between the runner and a [`Scanner`].
const SCAN_RESULTS_DEPTH: usize = 32;

enum Request {
    Scan,
}

enum ScanEvent {
    Bss(BssInfo),
    Done,
    Aborted,
}

struct Shared {
    requests: Channel<NoopRawMutex, Request, 1>,
    scan_results: Channel<NoopRawMutex, ScanEvent, SCAN_RESULTS_DEPTH>,
}

pub struct State {
    shared: Shared,
    ch: ch::State<MTU, 4, 4>,
}

impl State {
    pub fn new() -> Self {
        Self {
            ch: ch::State::new(),
            shared: Shared {
                requests: Channel::new(),
                scan_results: Channel::new(),
            },
        }
    }
}

pub type NetDriver<'a> = ch::Device<'a, MTU>;

pub async fn new<'a, BUS, IN, OUT>(
    state: &'a mut State,
    bus: BUS,
    bucken: OUT,
    iovdd_ctl: OUT,
    host_irq: IN,
) -> (NetDriver<'a>, Control<'a>, Runner<'a, BUS, IN, OUT>)
where
    BUS: Bus,
    IN: InputPin + Wait,
    OUT: OutputPin,
{
    let (ch_runner, device) = ch::new(&mut state.ch, ch::driver::HardwareAddress::Ethernet([0; 6]));
    let state_ch = ch_runner.state_runner();

    let mut runner = Runner {
        ch: ch_runner,
        state_ch,
        shared: &state.shared,
        bus,
        bucken,
        iovdd_ctl,
        host_irq,
        rpu_info: None,
        num_commands: c::RPU_CMD_START_MAGIC,
        scan_deadline: None,
    };
    runner.init().await;

    let control = Control {
        shared: &state.shared,
        state_ch,
    };

    (device, control, runner)
}

pub struct Control<'a> {
    shared: &'a Shared,
    #[allow(unused)]
    state_ch: ch::StateRunner<'a>,
}

impl<'a> Control<'a> {
    /// Starts an active scan of every channel. The results arrive through the returned [`Scanner`].
    pub async fn scan(&mut self) -> Scanner<'_> {
        self.shared.scan_results.clear();
        self.shared.requests.send(Request::Scan).await;
        Scanner {
            shared: self.shared,
            done: false,
        }
    }
}

/// The results of one scan, as the RPU reports them.
pub struct Scanner<'a> {
    shared: &'a Shared,
    done: bool,
}

impl Scanner<'_> {
    /// Returns the next BSS, or `None` once the scan has completed, been aborted or timed out.
    pub async fn next(&mut self) -> Option<BssInfo> {
        if self.done {
            return None;
        }
        let event = match with_timeout(SCAN_TIMEOUT, self.shared.scan_results.receive()).await {
            Ok(event) => event,
            Err(_) => {
                warn!("scan timed out");
                ScanEvent::Aborted
            }
        };
        match event {
            ScanEvent::Bss(bss) => Some(bss),
            ScanEvent::Done => {
                self.done = true;
                None
            }
            ScanEvent::Aborted => {
                warn!("scan aborted");
                self.done = true;
                None
            }
        }
    }
}

/// Frequency band of a BSS.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum Band {
    Band2_4GHz,
    Band5GHz,
    Unknown(u32),
}

/// Security of a BSS, as the RPU classifies it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum Security {
    Open,
    Wep,
    Wpa,
    Wpa2,
    Wpa2Sha256,
    Wpa3,
    Wapi,
    Eap,
    Unknown(u32),
}

/// One scan result.
#[derive(Clone, Debug, defmt::Format)]
pub struct BssInfo {
    ssid: [u8; 32],
    ssid_len: u8,
    pub bssid: [u8; 6],
    pub band: Band,
    pub channel: u32,
    /// Signal strength in dBm, when the RPU reports it in mBm.
    pub rssi: Option<i32>,
    pub security: Security,
    pub beacon_interval: u16,
}

impl BssInfo {
    pub fn ssid(&self) -> &[u8] {
        &self.ssid[..self.ssid_len as usize]
    }

    fn from_display_result(r: &c::umac_display_results) -> Self {
        let ssid_len = r.ssid.ssid_len.min(32);
        let band = r.nwk_band as u32;
        let security = r.security_type as u32;
        let signal_type = r.signal.signal_type;
        let mbm = unsafe { r.signal.signal.mbm_signal } as i32;
        Self {
            ssid: r.ssid.ssid,
            ssid_len,
            bssid: r.mac_addr,
            band: match c::band::try_from(band) {
                Ok(c::band::BAND_2GHZ) => Band::Band2_4GHz,
                Ok(c::band::BAND_5GHZ) => Band::Band5GHz,
                _ => Band::Unknown(band),
            },
            channel: r.nwk_channel,
            rssi: (signal_type == c::SIGNAL_TYPE_MBM).then_some(mbm / 100),
            security: match c::security_type::try_from(security) {
                Ok(c::security_type::OPEN) => Security::Open,
                Ok(c::security_type::WEP) => Security::Wep,
                Ok(c::security_type::WPA) => Security::Wpa,
                Ok(c::security_type::WPA2) => Security::Wpa2,
                Ok(c::security_type::WPA2_256) => Security::Wpa2Sha256,
                Ok(c::security_type::WPA3) => Security::Wpa3,
                Ok(c::security_type::WAPI) => Security::Wapi,
                Ok(c::security_type::EAP) => Security::Eap,
                Err(_) => Security::Unknown(security),
            },
            beacon_interval: r.beacon_interval,
        }
    }
}

trait Command {
    const MESSAGE_TYPE: c::host_rpu_msg_type;
    fn fill(&mut self);
}

macro_rules! impl_cmd {
    (sys, $cmd:path, $num:expr) => {
        impl Command for $cmd {
            const MESSAGE_TYPE: c::host_rpu_msg_type = c::host_rpu_msg_type::HOST_RPU_MSG_TYPE_SYSTEM;
            fn fill(&mut self) {
                self.sys_head = c::sys_head {
                    cmd_event: $num as _,
                    len: size_of::<Self>() as _,
                };
            }
        }
    };
    (umac, $cmd:path, $num:expr) => {
        impl Command for $cmd {
            const MESSAGE_TYPE: c::host_rpu_msg_type = c::host_rpu_msg_type::HOST_RPU_MSG_TYPE_UMAC;
            fn fill(&mut self) {
                // Every UMAC command addresses the default interface, wdev 0.
                self.umac_hdr = c::umac_hdr {
                    cmd_evnt: $num as _,
                    ids: c::index_ids {
                        valid_fields: c::INDEX_IDS_WDEV_ID_VALID,
                        wdev_id: 0,
                        ..unsafe { zeroed() }
                    },
                    ..unsafe { zeroed() }
                };
            }
        }
    };
}

impl_cmd!(sys, c::cmd_sys_init, c::sys_commands::CMD_INIT);
impl_cmd!(
    umac,
    c::umac_cmd_change_macaddr,
    c::umac_commands::UMAC_CMD_CHANGE_MACADDR
);
impl_cmd!(umac, c::umac_cmd_chg_vif_state, c::umac_commands::UMAC_CMD_SET_IFFLAGS);
impl_cmd!(umac, c::umac_cmd_scan, c::umac_commands::UMAC_CMD_TRIGGER_SCAN);
impl_cmd!(
    umac,
    c::umac_cmd_get_scan_results,
    c::umac_commands::UMAC_CMD_GET_SCAN_RESULTS
);

fn sliceit<T>(t: &T) -> &[u8] {
    unsafe { slice::from_raw_parts(t as *const _ as _, size_of::<T>()) }
}

fn unsliceit2<T>(t: &[u8]) -> (&T, &[u8]) {
    assert!(t.len() > size_of::<T>());
    assert!(t.as_ptr() as usize % align_of::<T>() == 0);
    (unsafe { &*(t.as_ptr() as *const T) }, &t[size_of::<T>()..])
}

fn unsliceit<T>(t: &[u8]) -> &T {
    unsliceit2(t).0
}

fn slice8(x: &[u32]) -> &[u8] {
    let len = x.len() * 4;
    unsafe { slice::from_raw_parts(x.as_ptr() as _, len) }
}

fn slice8_mut(x: &mut [u32]) -> &mut [u8] {
    let len = x.len() * 4;
    unsafe { slice::from_raw_parts_mut(x.as_mut_ptr() as _, len) }
}

fn slice32(x: &[u8]) -> &[u32] {
    assert!(x.len() % 4 == 0);
    assert!(x.as_ptr() as usize % 4 == 0);
    let len = x.len() / 4;
    unsafe { slice::from_raw_parts(x.as_ptr() as _, len) }
}

#[derive(Copy, Clone, Debug, defmt::Format)]
struct MemoryRegion {
    start: u32,
    end: u32,

    /// Number of dummy 32bit words
    latency: u32,

    rpu_mem_start: u32,
    rpu_mem_end: u32,
    processor_restriction: Option<Processor>,
}

#[rustfmt::skip]
pub(crate) mod regions {
    use super::*;
	pub(crate) const SYSBUS       : &MemoryRegion = &MemoryRegion { start: 0x000000, end: 0x008FFF, latency: 1, rpu_mem_start: 0xA4000000, rpu_mem_end: 0xA4FFFFFF, processor_restriction: None };
	pub(crate) const EXT_SYS_BUS  : &MemoryRegion = &MemoryRegion { start: 0x009000, end: 0x03FFFF, latency: 2, rpu_mem_start: 0,          rpu_mem_end: 0,          processor_restriction: None };
	pub(crate) const PBUS         : &MemoryRegion = &MemoryRegion { start: 0x040000, end: 0x07FFFF, latency: 1, rpu_mem_start: 0xA5000000, rpu_mem_end: 0xA5FFFFFF, processor_restriction: None };
	pub(crate) const PKTRAM       : &MemoryRegion = &MemoryRegion { start: 0x0C0000, end: 0x0F0FFF, latency: 0, rpu_mem_start: 0xB0000000, rpu_mem_end: 0xB0FFFFFF, processor_restriction: None };
	pub(crate) const GRAM         : &MemoryRegion = &MemoryRegion { start: 0x080000, end: 0x092000, latency: 1, rpu_mem_start: 0xB7000000, rpu_mem_end: 0xB7FFFFFF, processor_restriction: None };
	pub(crate) const LMAC_ROM     : &MemoryRegion = &MemoryRegion { start: 0x100000, end: 0x134000, latency: 1, rpu_mem_start: 0x80000000, rpu_mem_end: 0x80033FFF, processor_restriction: Some(Processor::LMAC) }; // ROM
	pub(crate) const LMAC_RET_RAM : &MemoryRegion = &MemoryRegion { start: 0x140000, end: 0x14C000, latency: 1, rpu_mem_start: 0x80040000, rpu_mem_end: 0x8004BFFF, processor_restriction: Some(Processor::LMAC) }; // retained RAM
	pub(crate) const LMAC_SRC_RAM : &MemoryRegion = &MemoryRegion { start: 0x180000, end: 0x190000, latency: 1, rpu_mem_start: 0x80080000, rpu_mem_end: 0x8008FFFF, processor_restriction: Some(Processor::LMAC) }; // scratch RAM
	pub(crate) const UMAC_ROM     : &MemoryRegion = &MemoryRegion { start: 0x200000, end: 0x261800, latency: 1, rpu_mem_start: 0x80000000, rpu_mem_end: 0x800617FF, processor_restriction: Some(Processor::UMAC) }; // ROM
	pub(crate) const UMAC_RET_RAM : &MemoryRegion = &MemoryRegion { start: 0x280000, end: 0x2A4000, latency: 1, rpu_mem_start: 0x80080000, rpu_mem_end: 0x800A3FFF, processor_restriction: Some(Processor::UMAC) }; // retained RAM
	pub(crate) const UMAC_SRC_RAM : &MemoryRegion = &MemoryRegion { start: 0x300000, end: 0x338000, latency: 1, rpu_mem_start: 0x80100000, rpu_mem_end: 0x80137FFF, processor_restriction: Some(Processor::UMAC) }; // scratch RAM

    pub(crate) const REGIONS: [&MemoryRegion; 11] = [
        SYSBUS, EXT_SYS_BUS, PBUS, PKTRAM, GRAM, LMAC_ROM, LMAC_RET_RAM, LMAC_SRC_RAM, UMAC_ROM, UMAC_RET_RAM, UMAC_SRC_RAM
    ];

    #[doc(alias = "pal_rpu_addr_offset_get")]
    pub(crate) fn remap_global_addr_to_region_and_offset(rpu_addr: u32, processor: Option<Processor>) -> (&'static MemoryRegion, u32) {
        defmt::unwrap!(
            REGIONS
                .into_iter()
                .filter(|region| region.processor_restriction.is_none() || region.processor_restriction == processor)
                .find(|region| rpu_addr >= region.rpu_mem_start && rpu_addr <= region.rpu_mem_end)
                .map(|region| (region, rpu_addr - region.rpu_mem_start))
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub(crate) enum Processor {
    LMAC,
    UMAC,
}

static FW_LMAC_PATCH_PRI: &[u8] = include_aligned!(Align16, "../fw/lmac_patch_pri_bimg.bin");
static FW_LMAC_PATCH_SEC: &[u8] = include_aligned!(Align16, "../fw/lmac_patch_sec_bin.bin");
static FW_UMAC_PATCH_PRI: &[u8] = include_aligned!(Align16, "../fw/umac_patch_pri_bimg.bin");
static FW_UMAC_PATCH_SEC: &[u8] = include_aligned!(Align16, "../fw/umac_patch_sec_bin.bin");

const SR0_WRITE_IN_PROGRESS: u8 = 0x01;

const SR1_RPU_AWAKE: u8 = 0x02;
const SR1_RPU_READY: u8 = 0x04;

const SR2_RPU_WAKEUP_REQ: u8 = 0x01;

const MAX_EVENT_POOL_LEN: usize = 1000;

/// Largest command the RPU takes in one buffer; longer ones are sent in fragments.
const MAX_CMD_SIZE: usize = c::MAX_UMAC_CMD_SIZE as usize;

// ========= RF parameters

const RF_PARAMS_SIZE: usize = c::RF_PARAMS_SIZE as usize;

/// Default RF parameters for the nRF7002, from the nRF Connect SDK
/// (`NRF_WIFI_DEF_RF_PARAMS` in `phy_rf_params.h`, v2.4). Bytes past the string stay 0xFF.
const DEF_RF_PARAMS: &str = "0000000000002A00000000030303035440403838383838380000000050EC000000FCFCF8FCF800000000007077003F032424001000002800323500000CF008087D8105010071630300EED501001F6F00003B350100F52E0000E35E0000B7B6000066EFFEFFB5F60000896200007A840200E28FFCFF080808080408120100000000A1A10178000000080050003B020726181818181A120A140E0600";

const fn hex_nibble(c: u8) -> u8 {
    match c {
        b'0'..=b'9' => c - b'0',
        b'A'..=b'F' => c - b'A' + 10,
        b'a'..=b'f' => c - b'a' + 10,
        _ => core::panic!("invalid hex digit"),
    }
}

const DEF_RF_PARAMS_BYTES: [u8; RF_PARAMS_SIZE] = {
    let s = DEF_RF_PARAMS.as_bytes();
    core::assert!(s.len() % 2 == 0 && s.len() / 2 <= RF_PARAMS_SIZE);
    let mut out = [0xFF; RF_PARAMS_SIZE];
    let mut i = 0;
    while i < s.len() / 2 {
        out[i] = hex_nibble(s[2 * i]) << 4 | hex_nibble(s[2 * i + 1]);
        i += 1;
    }
    out
};

/// Builds the RF parameters from the defaults and the chip's OTP calibration, as the nRF Connect
/// SDK does (`wifi_nrf_fmac_rf_params_get`). A calibration value is used only when its "not
/// programmed" flag is clear.
fn rf_params(otp: &c::host_rpu_umac_info, otp_flags: u32) -> [u8; RF_PARAMS_SIZE] {
    let mut params = DEF_RF_PARAMS_BYTES;

    let mut calib = [0u8; 36];
    let calib_words = otp.calib;
    for (dst, word) in calib.chunks_exact_mut(4).zip(calib_words) {
        dst.copy_from_slice(&word.to_le_bytes());
    }

    let programmed = |mask: i32| otp_flags & !(mask as u32) == 0;
    let mut copy = |param_off: u32, otp_off: u32, len: u32| {
        let (p, o, n) = (param_off as usize, otp_off as usize, len as usize);
        params[p..p + n].copy_from_slice(&calib[o..o + n]);
    };

    if programmed(c::CALIB_XO_FLAG_MASK) {
        copy(c::RF_PARAMS_OFF_CALIB_X0, c::OTP_OFF_CALIB_XO, c::OTP_SZ_CALIB_XO);
    }
    if programmed(c::CALIB_PDADJM7_FLAG_MASK) {
        copy(
            c::RF_PARAMS_OFF_CALIB_PDADJM7,
            c::OTP_OFF_CALIB_PDADJM7,
            c::OTP_SZ_CALIB_PDADJM7,
        );
    }
    if programmed(c::CALIB_PDADJM0_FLAG_MASK) {
        copy(
            c::RF_PARAMS_OFF_CALIB_PDADJM0,
            c::OTP_OFF_CALIB_PDADJM0,
            c::OTP_SZ_CALIB_PDADJM0,
        );
    }
    if programmed(c::CALIB_PWR2G_FLAG_MASK) {
        copy(
            c::RF_PARAMS_OFF_CALIB_PWR2G,
            c::OTP_OFF_CALIB_PWR2G,
            c::OTP_SZ_CALIB_PWR2G,
        );
        copy(
            c::RF_PARAMS_OFF_CALIB_PWR2GM0M7,
            c::OTP_OFF_CALIB_PWR2GM0M7,
            c::OTP_SZ_CALIB_PWR2GM0M7,
        );
    }
    if programmed(c::CALIB_PWR5GM7_FLAG_MASK) {
        copy(
            c::RF_PARAMS_OFF_CALIB_PWR5GM7,
            c::OTP_OFF_CALIB_PWR5GM7,
            c::OTP_SZ_CALIB_PWR5GM7,
        );
    }
    if programmed(c::CALIB_PWR5GM0_FLAG_MASK) {
        copy(
            c::RF_PARAMS_OFF_CALIB_PWR5GM0,
            c::OTP_OFF_CALIB_PWR5GM0,
            c::OTP_SZ_CALIB_PWR5GM0,
        );
    }
    if programmed(c::CALIB_RXGNOFF_FLAG_MASK) {
        copy(
            c::RF_PARAMS_OFF_CALIB_RXGNOFF,
            c::OTP_OFF_CALIB_RXGNOFF,
            c::OTP_SZ_CALIB_RXGNOFF,
        );
    }
    if programmed(c::CALIB_TXPOWBACKOFFT_FLAG_MASK) {
        copy(
            c::RF_PARAMS_OFF_CALIB_TXP_BOFF_2GH,
            c::OTP_OFF_CALIB_TXP_BOFF_2GH,
            c::OTP_SZ_CALIB_TXP_BOFF_2GH,
        );
        copy(
            c::RF_PARAMS_OFF_CALIB_TXP_BOFF_2GL,
            c::OTP_OFF_CALIB_TXP_BOFF_2GL,
            c::OTP_SZ_CALIB_TXP_BOFF_2GL,
        );
        copy(
            c::RF_PARAMS_OFF_CALIB_TXP_BOFF_5GH,
            c::OTP_OFF_CALIB_TXP_BOFF_5GH,
            c::OTP_SZ_CALIB_TXP_BOFF_5GH,
        );
        copy(
            c::RF_PARAMS_OFF_CALIB_TXP_BOFF_5GL,
            c::OTP_OFF_CALIB_TXP_BOFF_5GL,
            c::OTP_SZ_CALIB_TXP_BOFF_5GL,
        );
    }
    if programmed(c::CALIB_TXPOWBACKOFFV_FLAG_MASK) {
        copy(
            c::RF_PARAMS_OFF_CALIB_TXP_BOFF_V,
            c::OTP_OFF_CALIB_TXP_BOFF_V,
            c::OTP_SZ_CALIB_TXP_BOFF_V,
        );
    }

    params
}

/// The MAC address programmed in OTP (`MAC0`), if it is a valid unicast address.
fn otp_mac_address(otp: &c::host_rpu_umac_info) -> Option<[u8; 6]> {
    let words = otp.mac_address0;
    let mut bytes = [0u8; 8];
    bytes[..4].copy_from_slice(&words[0].to_le_bytes());
    bytes[4..].copy_from_slice(&words[1].to_le_bytes());
    let mac: [u8; 6] = unwrap!(bytes[..6].try_into());
    let unset = mac == [0; 6] || mac == [0xFF; 6];
    let multicast = mac[0] & 0x01 != 0;
    (!unset && !multicast).then_some(mac)
}

/// Locally administered address used when the OTP holds none.
const FALLBACK_MAC_ADDRESS: [u8; 6] = [0x02, 0x70, 0x02, 0x00, 0x00, 0x01];

// ========= config
/*
pktram: 0xB0000000 - 0xB0030FFF -- 196kb
usable for mcu-rpu comms: 0xB0005000 - 0xB0030FFF -- 176kb

First we allocate N tx buffers, which consist of
- Header of 52 bytes
- Data of N bytes

Then we allocate rx buffers.
- 3 queues of
  - N buffers each, which consist of
    - Header of 4 bytes
    - Data of N bytes (default 1600)

Each RX buffer has a "descriptor ID" which is assigned across all queues starting from 0
- queue 0 is descriptors 0..N-1
- queue 1 is descriptors N..2N-1
- queue 2 is descriptors 2N..3N-1
*/

// configurable by user
const MAX_TX_TOKENS: usize = 10;
const MAX_TX_AGGREGATION: usize = 6;
const TX_MAX_DATA_SIZE: usize = 1600;
const RX_MAX_DATA_SIZE: usize = 1600;
const RX_BUFS_PER_QUEUE: usize = 16;

// fixed

const TX_BUFS: usize = MAX_TX_TOKENS * MAX_TX_AGGREGATION;
const TX_BUF_SIZE: usize = c::TX_BUF_HEADROOM as usize + TX_MAX_DATA_SIZE;
const TX_TOTAL_SIZE: usize = TX_BUFS * TX_BUF_SIZE;

const RX_BUFS: usize = RX_BUFS_PER_QUEUE * c::MAX_NUM_OF_RX_QUEUES as usize;
const RX_BUF_SIZE: usize = c::RX_BUF_HEADROOM as usize + RX_MAX_DATA_SIZE;
const RX_TOTAL_SIZE: usize = RX_BUFS * RX_BUF_SIZE;

const _: () = {
    use core::assert;
    assert!(MAX_TX_TOKENS >= 1, "At least one TX token is required");
    assert!(MAX_TX_AGGREGATION <= 16, "Max TX aggregation is 16");
    assert!(RX_BUFS_PER_QUEUE >= 1, "At least one RX buffer per queue is required");
    assert!(
        (TX_TOTAL_SIZE + RX_TOTAL_SIZE) as u32 <= c::RPU_PKTRAM_SIZE,
        "Packet RAM overflow"
    );
};

/// An event from the RPU, as far as the runner needs to tell them apart.
enum Event<'b> {
    /// A system event, by `sys_events` number.
    Sys(u32),
    /// A UMAC event, by `umac_events` number, with the whole message (which starts with its
    /// `umac_hdr`).
    Umac(u32, &'b [u8]),
    /// A data path event, by `umac_data_commands` number, with the whole message (which starts
    /// with its `umac_head`).
    Data(u32, &'b [u8]),
    /// Any other message type.
    Other(u32),
}

fn parse_event(buf: &[u8]) -> Event<'_> {
    let (msg, body) = unsliceit2::<c::host_rpu_msg>(buf);
    let msg_type = msg.type_ as u32;
    match c::host_rpu_msg_type::try_from(msg_type) {
        Ok(c::host_rpu_msg_type::HOST_RPU_MSG_TYPE_SYSTEM) => Event::Sys(unsliceit::<c::sys_head>(body).cmd_event),
        Ok(c::host_rpu_msg_type::HOST_RPU_MSG_TYPE_UMAC) => Event::Umac(unsliceit::<c::umac_hdr>(body).cmd_evnt, body),
        Ok(c::host_rpu_msg_type::HOST_RPU_MSG_TYPE_DATA) => Event::Data(unsliceit::<c::umac_head>(body).cmd, body),
        _ => Event::Other(msg_type),
    }
}

pub struct Runner<'a, BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> {
    #[allow(unused)]
    ch: ch::Runner<'a, MTU>,
    state_ch: ch::StateRunner<'a>,
    shared: &'a Shared,

    bus: BUS,
    bucken: OUT,
    iovdd_ctl: OUT,
    host_irq: IN,

    rpu_info: Option<RpuInfo>,

    num_commands: u32,

    /// Set while a display scan is running, so a lost one does not block the next.
    scan_deadline: Option<Instant>,
}

impl<'a, BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> Runner<'a, BUS, IN, OUT> {
    async fn init(&mut self) {
        info!("power on...");
        Timer::after(Duration::from_millis(10)).await;
        self.bucken.set_high().unwrap();
        Timer::after(Duration::from_millis(10)).await;
        self.iovdd_ctl.set_high().unwrap();
        Timer::after(Duration::from_millis(10)).await;

        info!("wakeup...");
        self.rpu_wakeup().await;

        info!("enable clocks...");
        self.raw_write32(PBUS, 0x8C20, 0x0100).await;

        info!("enable interrupt...");
        // First enable the blockwise interrupt for the relevant block in the master register
        let mut val = self.raw_read32(SYSBUS, 0x400).await;
        val |= 1 << 17;
        self.raw_write32(SYSBUS, 0x400, val).await;

        // Now enable the relevant MCU interrupt line
        self.raw_write32(SYSBUS, 0x494, 1 << 31).await;

        info!("load LMAC firmware patches...");
        self.raw_write32(SYSBUS, 0x000, 0x01).await; // reset
        while self.raw_read32(SYSBUS, 0x0000).await & 0x01 != 0 {}
        while self.raw_read32(SYSBUS, 0x0018).await & 0x01 != 1 {}
        self.load_fw(LMAC_RET_RAM, 0x9000, FW_LMAC_PATCH_PRI).await;
        self.load_fw(LMAC_RET_RAM, 0x4000, FW_LMAC_PATCH_SEC).await;

        self.raw_write32(GRAM, 0xD50, 0).await;
        self.raw_write32(SYSBUS, 0x50, 0x3c1a8000).await;
        self.raw_write32(SYSBUS, 0x54, 0x275a0000).await;
        self.raw_write32(SYSBUS, 0x58, 0x03400008).await;
        self.raw_write32(SYSBUS, 0x5c, 0x00000000).await;
        self.raw_write32(SYSBUS, 0x2C2c, 0x9000).await;

        info!("booting LMAC...");
        self.raw_write32(SYSBUS, 0x000, 0x01).await; // reset
        while self.raw_read32(GRAM, 0xD50).await != 0x5A5A5A5A {}

        info!("load UMAC firmware patches...");
        self.raw_write32(SYSBUS, 0x100, 0x01).await; // reset
        while self.raw_read32(SYSBUS, 0x0100).await & 0x01 != 0 {}
        while self.raw_read32(SYSBUS, 0x0118).await & 0x01 != 1 {}
        self.load_fw(UMAC_RET_RAM, 0x14400, FW_UMAC_PATCH_PRI).await;
        self.load_fw(UMAC_RET_RAM, 0xC000, FW_UMAC_PATCH_SEC).await;

        self.raw_write32(PKTRAM, 0, 0).await;
        self.raw_write32(SYSBUS, 0x150, 0x3c1a8000).await;
        self.raw_write32(SYSBUS, 0x154, 0x275a0000).await;
        self.raw_write32(SYSBUS, 0x158, 0x03400008).await;
        self.raw_write32(SYSBUS, 0x15c, 0x00000000).await;
        self.raw_write32(SYSBUS, 0x2C30, 0x14400).await;

        info!("booting UMAC...");
        self.raw_write32(SYSBUS, 0x100, 0x01).await; // reset
        while self.raw_read32(PKTRAM, 0).await != 0x5A5A5A5A {}

        let umac_ver = self.read32(c::RPU_MEM_UMAC_VER, None).await.to_be_bytes();
        let lmac_ver = self.read32(c::RPU_MEM_LMAC_VER, None).await.to_be_bytes();
        info!(
            "firmware booted: UMAC {}.{}.{}.{}, LMAC {}.{}.{}.{}",
            umac_ver[0], umac_ver[1], umac_ver[2], umac_ver[3], lmac_ver[0], lmac_ver[1], lmac_ver[2], lmac_ver[3]
        );

        info!("Initializing rpu info...");
        self.init_rpu_info().await;

        info!("Reading OTP...");
        let (otp, otp_flags) = self.read_otp().await;
        let rf_params = rf_params(&otp, otp_flags);
        let mac_addr = match otp_mac_address(&otp) {
            Some(mac) => mac,
            None => {
                warn!("no MAC address in OTP, using a locally administered one");
                FALLBACK_MAC_ADDRESS
            }
        };
        info!("OTP flags {:08x}, MAC address {:02x}", otp_flags, mac_addr);

        info!("Enabling interrupts...");
        self.rpu_irq_enable().await;

        info!("Initializing TX...");
        self.init_tx().await;

        info!("Initializing RX...");
        self.init_rx().await;

        info!("Initializing umac...");
        self.init_umac(&rf_params).await;
        self.wait_for_event(
            "INIT_DONE",
            |event| matches!(event, Event::Sys(id) if *id == c::sys_events::EVENT_INIT_DONE as u32),
        )
        .await;
        info!("======== INIT DONE!! ==========");

        info!("Bringing the interface up...");
        self.set_mac_address(mac_addr).await;
        self.state_ch
            .set_hardware_address(ch::driver::HardwareAddress::Ethernet(mac_addr));
        self.set_interface_up().await;
    }

    pub async fn run(&mut self) -> ! {
        info!("running...");

        let mut buf = [0u32; MAX_EVENT_POOL_LEN / 4];

        loop {
            while self.rpu_event_next(&mut buf).await {
                self.handle_event(slice8(&buf)).await;
            }
            self.rpu_irq_service_end().await;

            let shared = self.shared;
            let irq = with_timeout(IRQ_POLL_PERIOD, self.host_irq.wait_for_rising_edge());
            match select(irq, shared.requests.receive()).await {
                Either::First(_) => {}
                Either::Second(request) => self.handle_request(request).await,
            }
        }
    }

    async fn handle_request(&mut self, request: Request) {
        match request {
            Request::Scan => {
                if let Some(deadline) = self.scan_deadline {
                    if Instant::now() < deadline {
                        warn!("scan already in progress");
                        self.push_scan_event(ScanEvent::Aborted);
                        return;
                    }
                }
                self.scan_deadline = Some(Instant::now() + SCAN_TIMEOUT);
                self.trigger_scan().await;
            }
        }
    }

    async fn handle_event(&mut self, buf: &[u8]) {
        match parse_event(buf) {
            Event::Sys(id) => match c::sys_events::try_from(id) {
                Ok(event) => debug!("sys event {}", event as u32),
                Err(_) => warn!("unknown sys event type {:08x}", id),
            },
            Event::Umac(id, body) => self.handle_umac_event(id, body).await,
            Event::Data(id, body) => self.handle_data_event(id, body).await,
            Event::Other(msg_type) => warn!("unknown event type {:08x}", msg_type),
        }
    }

    async fn handle_data_event(&mut self, id: u32, body: &[u8]) {
        match c::umac_data_commands::try_from(id) {
            Ok(c::umac_data_commands::CMD_RX_BUFF) => {
                let (rx, infos) = unsliceit2::<c::rx_buff>(body);
                let count = rx.rx_pkt_cnt as usize;
                for i in 0..count {
                    let info: &c::rx_buff_info = unsliceit(&infos[i * size_of::<c::rx_buff_info>()..]);
                    let desc_id = info.descriptor_id as usize;
                    if desc_id >= RX_BUFS {
                        warn!("RX event for invalid descriptor {}", desc_id);
                        continue;
                    }
                    // Frames are not passed to a network stack yet, so the buffer goes straight
                    // back to the RPU. Without this the RX pool drains.
                    self.rx_buf_post(desc_id).await;
                }
            }
            _ => debug!("unhandled data event {}", id),
        }
    }

    async fn handle_umac_event(&mut self, id: u32, body: &[u8]) {
        match c::umac_events::try_from(id) {
            Ok(c::umac_events::UMAC_EVENT_TRIGGER_SCAN_START) => debug!("scan started"),
            Ok(c::umac_events::UMAC_EVENT_SCAN_DONE) => {
                if self.scan_deadline.is_some() {
                    // A display scan hands its results over only when asked for them.
                    self.get_scan_results().await;
                }
            }
            Ok(c::umac_events::UMAC_EVENT_SCAN_ABORTED) => {
                self.scan_deadline = None;
                self.push_scan_event(ScanEvent::Aborted);
            }
            Ok(c::umac_events::UMAC_EVENT_SCAN_DISPLAY_RESULT) => {
                let event: &c::umac_event_new_scan_display_results = unsliceit(body);
                let count = (event.event_bss_count as usize).min(event.display_results.len());
                let seq = event.umac_hdr.seq;
                debug!("scan display results: seq {}, {} BSS", seq, count);
                for result in &event.display_results[..count] {
                    self.push_scan_event(ScanEvent::Bss(BssInfo::from_display_result(result)));
                }
                // The last batch of results has a zero sequence number.
                if seq == 0 {
                    self.scan_deadline = None;
                    self.push_scan_event(ScanEvent::Done);
                }
            }
            Ok(c::umac_events::UMAC_EVENT_IFFLAGS_STATUS) => {
                let status = unsliceit::<c::umac_event_vif_state>(body).status;
                debug!("interface flags status {}", status);
            }
            _ => debug!("unhandled UMAC event {}", id),
        }
    }

    fn push_scan_event(&mut self, event: ScanEvent) {
        if self.shared.scan_results.try_send(event).is_err() {
            warn!("scan result dropped, the scanner is not keeping up");
        }
    }

    /// Waits for an event that `matches` accepts, handling the others as usual.
    async fn wait_for_event(&mut self, what: &str, matches: impl Fn(&Event) -> bool) {
        let mut buf = [0u32; MAX_EVENT_POOL_LEN / 4];
        let found = with_timeout(EVENT_TIMEOUT, async {
            loop {
                self.next_event(&mut buf).await;
                if matches(&parse_event(slice8(&buf))) {
                    return;
                }
                self.handle_event(slice8(&buf)).await;
            }
        })
        .await;
        if found.is_err() {
            panic!("timed out waiting for {}", what);
        }
    }

    /// Reads the next event into `buf`, waiting for HOST_IRQ while the queue is empty.
    async fn next_event(&mut self, buf: &mut [u32]) {
        loop {
            if self.rpu_event_next(buf).await {
                return;
            }
            self.rpu_irq_service_end().await;
            let _ = with_timeout(IRQ_POLL_PERIOD, self.host_irq.wait_for_rising_edge()).await;
        }
    }

    /// Reads the next queued event into `buf`. Returns false if the queue is empty.
    async fn rpu_event_next(&mut self, buf: &mut [u32]) -> bool {
        let event_address = self
            .rpu_hpq_dequeue(self.rpu_info.as_ref().unwrap().hpqm_info.event_busy_queue)
            .await;

        match event_address {
            // No more events to read. Sometimes when low power mode is enabled
            // we see a wrong address, but it work after a while, so, add a
            // check for that.
            None | Some(0xAAAAAAAA) => false,
            Some(event_address) => {
                self.rpu_event_read(event_address, buf).await;
                true
            }
        }
    }

    /// Ends an interrupt once the event queue is empty (NCS `hal_rpu_irq_process`).
    async fn rpu_irq_service_end(&mut self) {
        if self.rpu_irq_watchdog_check().await {
            debug!("RPU watchdog interrupt");
            self.rpu_irq_watchdog_ack().await;
        }
        self.rpu_irq_ack().await;
    }

    async fn rpu_irq_enable(&mut self) {
        // First enable the blockwise interrupt for the relevant block in the master register
        let mut val = self.read32(c::RPU_REG_INT_FROM_RPU_CTRL, None).await;

        val |= 1 << c::RPU_REG_BIT_INT_FROM_RPU_CTRL;

        self.write32(c::RPU_REG_INT_FROM_RPU_CTRL, None, val).await;

        // Now enable the relevant MCU interrupt line
        self.write32(
            c::RPU_REG_INT_FROM_MCU_CTRL,
            None,
            1 << c::RPU_REG_BIT_INT_FROM_MCU_CTRL,
        )
        .await;
    }

    #[allow(unused)]
    async fn rpu_irq_disable(&mut self) {
        let mut val = self.read32(c::RPU_REG_INT_FROM_RPU_CTRL, None).await;
        val &= !(1 << c::RPU_REG_BIT_INT_FROM_RPU_CTRL);
        self.write32(c::RPU_REG_INT_FROM_RPU_CTRL, None, val).await;

        self.write32(
            c::RPU_REG_INT_FROM_MCU_CTRL,
            None,
            !(1 << c::RPU_REG_BIT_INT_FROM_MCU_CTRL),
        )
        .await;
    }

    async fn rpu_irq_ack(&mut self) {
        // Guess: I think this clears the interrupt flag
        self.write32(c::RPU_REG_INT_FROM_MCU_ACK, None, 1 << c::RPU_REG_BIT_INT_FROM_MCU_ACK)
            .await;
    }

    /// Checks if the watchdog was the source of the interrupt
    async fn rpu_irq_watchdog_check(&mut self) -> bool {
        let val = self.read32(c::RPU_REG_MIPS_MCU_UCCP_INT_STATUS, None).await;
        (val & (1 << c::RPU_REG_BIT_MIPS_WATCHDOG_INT_STATUS)) > 0
    }

    async fn rpu_irq_watchdog_ack(&mut self) {
        self.write32(c::RPU_REG_MIPS_MCU_TIMER_CONTROL, None, 0).await;
    }

    async fn rpu_event_read(&mut self, event_address: u32, buf: &mut [u32]) {
        self.read(
            event_address,
            None,
            &mut buf[..c::RPU_EVENT_COMMON_SIZE_MAX as usize / 4],
        )
        .await;

        // Get the header from the front of the event data
        let message_header: &c::host_rpu_msg_hdr = unsliceit(slice8(buf));
        let len = message_header.len as usize;
        let resubmit = message_header.resubmit;

        if len > MAX_EVENT_POOL_LEN {
            todo!("Fragmented event read is not yet implemented");
        } else if len > c::RPU_EVENT_COMMON_SIZE_MAX as usize {
            // This is a longer than usual event. We gotta read it again
            self.read(event_address, None, &mut buf[..len.div_ceil(4)]).await;
        }

        // Hand the event back to the RPU only once it has been read in full.
        if resubmit > 0 {
            self.rpu_event_free(event_address).await;
        }
    }

    async fn rpu_event_free(&mut self, event_address: u32) {
        self.rpu_hpq_enqueue(self.rpu_info.as_ref().unwrap().hpqm_info.event_avl_queue, event_address)
            .await;
    }

    /// Writes one command buffer (at most [`MAX_CMD_SIZE`] bytes) and hands it to the RPU.
    async fn rpu_cmd_ctrl_send(&mut self, message: &[u32]) {
        assert!(message.len() * 4 <= MAX_CMD_SIZE);

        // Wait until we get an address to write to
        // This queue might already be full with other messages, so we'll just have to wait a bit
        let message_address = loop {
            if let Some(message_address) = self
                .rpu_hpq_dequeue(self.rpu_info.as_ref().unwrap().hpqm_info.cmd_avl_queue)
                .await
            {
                break message_address;
            }
        };

        // Write the message to the suggested address
        self.write(message_address, None, message).await;

        // Post the updated information to the RPU
        self.rpu_hpq_enqueue(
            self.rpu_info.as_ref().unwrap().hpqm_info.cmd_busy_queue,
            message_address,
        )
        .await;

        self.rpu_msg_trigger().await;
    }

    async fn rpu_hpq_enqueue(&mut self, hpq: HostRpuHPQ, value: u32) {
        self.write32(hpq.enqueue_addr, None, value).await;
    }

    async fn rpu_hpq_dequeue(&mut self, hpq: HostRpuHPQ) -> Option<u32> {
        let value = self.read32(hpq.dequeue_addr, None).await;

        // Pop element only if it is valid
        if value != 0 {
            self.write32(hpq.dequeue_addr, None, value).await;
            Some(value)
        } else {
            None
        }
    }

    async fn init_rpu_info(&mut self) {
        // Based on 'wifi_nrf_hal_dev_init'

        let mut hpqm_info = [0u32; size_of::<HostRpuHPQMInfo>() / 4];
        self.read(c::RPU_MEM_HPQ_INFO, None, &mut hpqm_info).await;

        let rx_cmd_base = self.read32(c::RPU_MEM_RX_CMD_BASE, None).await;

        self.rpu_info = Some(RpuInfo {
            hpqm_info: unsafe { core::mem::transmute_copy(&hpqm_info) },
            rx_cmd_base,
            tx_cmd_base: c::RPU_MEM_TX_CMD_BASE,
        });
    }

    /// Reads the UMAC's copy of the OTP and its "not programmed" flags (NCS
    /// `wifi_nrf_hal_otp_info_get`).
    async fn read_otp(&mut self) -> (c::host_rpu_umac_info, u32) {
        const WORDS: usize = size_of::<c::host_rpu_umac_info>().div_ceil(4);
        let mut words = [0u32; WORDS];
        self.read(c::RPU_MEM_UMAC_BOOT_SIG, None, &mut words).await;
        let info = unsafe { core::ptr::read_unaligned(words.as_ptr() as *const c::host_rpu_umac_info) };
        let flags = self.read32(c::RPU_MEM_OTP_INFO_FLAGS, None).await;
        (info, flags)
    }

    async fn send_cmd<T: Command>(&mut self, mut cmd: T) {
        cmd.fill();

        #[repr(C, packed)]
        struct Msg<T> {
            header: c::host_rpu_msg,
            cmd: T,
        }

        let mut msg = Msg {
            header: unsafe { zeroed() },
            cmd,
        };
        msg.header.hdr.len = size_of::<Msg<T>>() as _;
        msg.header.type_ = T::MESSAGE_TYPE as _;

        // A command longer than one buffer goes out in fragments, which the RPU reassembles
        // using the total length in the header (NCS `hal_rpu_cmd_queue`).
        for fragment in sliceit(&msg).chunks(MAX_CMD_SIZE) {
            let mut buf = [0u32; MAX_CMD_SIZE / 4];
            slice8_mut(&mut buf)[..fragment.len()].copy_from_slice(fragment);
            let words = &buf[..fragment.len().div_ceil(4)];
            if with_timeout(Duration::from_secs(1), self.rpu_cmd_ctrl_send(words))
                .await
                .is_err()
            {
                panic!("timed out waiting for a free command buffer");
            }
        }
    }

    async fn init_umac(&mut self, rf_params: &[u8; RF_PARAMS_SIZE]) {
        let cmd = c::cmd_sys_init {
            sys_head: unsafe { zeroed() },
            wdev_id: 0,
            sys_params: c::sys_params {
                sleep_enable: 0, // TODO for low power
                hw_bringup_time: c::HW_DELAY,
                sw_bringup_time: c::SW_DELAY,
                bcn_time_out: c::BCN_TIMEOUT,
                calib_sleep_clk: c::CALIB_SLEEP_CLOCK_ENABLE,
                phy_calib: c::DEF_PHY_CALIB,
                mac_addr: [0; 6],
                rf_params: *rf_params,
                rf_params_valid: 1,
            },
            rx_buf_pools: [
                c::rx_buf_pool_params {
                    buf_sz: RX_MAX_DATA_SIZE as _, // the RPU adds the headroom itself
                    num_bufs: RX_BUFS_PER_QUEUE as _,
                },
                c::rx_buf_pool_params {
                    buf_sz: RX_MAX_DATA_SIZE as _,
                    num_bufs: RX_BUFS_PER_QUEUE as _,
                },
                c::rx_buf_pool_params {
                    buf_sz: RX_MAX_DATA_SIZE as _,
                    num_bufs: RX_BUFS_PER_QUEUE as _,
                },
            ],
            data_config_params: c::data_config_params {
                rate_protection_type: 0,
                aggregation: 1,
                wmm: 1,
                max_num_tx_agg_sessions: 4,
                max_num_rx_agg_sessions: 8,
                max_tx_aggregation: MAX_TX_AGGREGATION as _,
                reorder_buf_size: 64,
                max_rxampdu_size: 3,
            },
            temp_vbat_config_params: c::temp_vbat_config {
                temp_based_calib_en: c::TEMP_CALIB_ENABLE,
                temp_calib_bitmap: c::DEF_PHY_TEMP_CALIB,
                vbat_calibp_bitmap: c::DEF_PHY_VBAT_CALIB,
                temp_vbat_mon_period: c::TEMP_CALIB_PERIOD,
                vth_very_low: c::VBAT_VERYLOW as _,
                vth_low: c::VBAT_LOW as _,
                vth_hi: c::VBAT_HIGH as _,
                temp_threshold: c::TEMP_CALIB_THRESHOLD as _,
                vbat_threshold: 0,
            },
            // The world regulatory domain, "00" (NCS CONFIG_NRF700X_REG_DOMAIN).
            country_code: *b"00",
            mgmt_buff_offload: 0,
            op_band: 0,
            tcp_ip_checksum_offload: 0,
            tx_pwr_ctrl_params: c::tx_pwr_ctrl_params {
                ant_gain_2g: 0,
                ant_gain_5g_band1: 0,
                ant_gain_5g_band2: 0,
                ant_gain_5g_band3: 0,
                band_edge_2g_lo: 0,
                band_edge_2g_hi: 0,
                band_edge_5g_unii_1_lo: 0,
                band_edge_5g_unii_1_hi: 0,
                band_edge_5g_unii_2a_lo: 0,
                band_edge_5g_unii_2a_hi: 0,
                band_edge_5g_unii_2c_lo: 0,
                band_edge_5g_unii_2c_hi: 0,
                band_edge_5g_unii_3_lo: 0,
                band_edge_5g_unii_3_hi: 0,
                band_edge_5g_unii_4_lo: 0,
                band_edge_5g_unii_4_hi: 0,
            },
        };
        self.send_cmd(cmd).await;
    }

    /// Sets the MAC address of the default interface (NCS `wifi_nrf_fmac_set_vif_macaddr`).
    async fn set_mac_address(&mut self, mac_addr: [u8; 6]) {
        let mut cmd: c::umac_cmd_change_macaddr = unsafe { zeroed() };
        cmd.macaddr_info.mac_addr = mac_addr;
        self.send_cmd(cmd).await;
    }

    /// Brings the default interface up and waits for the RPU to confirm (NCS
    /// `wifi_nrf_fmac_chg_vif_state`).
    async fn set_interface_up(&mut self) {
        let mut ifacename = [0; 16];
        for (dst, src) in ifacename.iter_mut().zip(b"wlan0") {
            *dst = *src as _;
        }
        let mut cmd: c::umac_cmd_chg_vif_state = unsafe { zeroed() };
        cmd.info.state = 1;
        cmd.info.ifacename = ifacename;
        self.send_cmd(cmd).await;

        self.wait_for_event(
            "IFFLAGS_STATUS",
            |event| matches!(event, Event::Umac(id, _) if *id == c::umac_events::UMAC_EVENT_IFFLAGS_STATUS as u32),
        )
        .await;
    }

    /// Starts an active display scan of every channel (NCS `wifi_nrf_disp_scan_zep`).
    async fn trigger_scan(&mut self) {
        let mut cmd: c::umac_cmd_scan = unsafe { zeroed() };
        cmd.info.scan_mode = c::scan_mode::AUTO_SCAN as _;
        cmd.info.scan_reason = c::scan_reason::SCAN_DISPLAY as _;
        // One zero-length (wildcard) SSID makes the scan active.
        cmd.info.scan_params.num_scan_ssids = 1;
        self.send_cmd(cmd).await;
    }

    /// Asks for the results of a finished display scan (NCS `wifi_nrf_fmac_scan_res_get`).
    async fn get_scan_results(&mut self) {
        let mut cmd: c::umac_cmd_get_scan_results = unsafe { zeroed() };
        cmd.scan_reason = c::scan_reason::SCAN_DISPLAY as _;
        self.send_cmd(cmd).await;
    }

    async fn init_tx(&mut self) {}

    async fn init_rx(&mut self) {
        for desc_id in 0..RX_BUFS {
            self.rx_buf_post(desc_id).await;
        }
    }

    /// Hands RX buffer `desc_id` to the RPU (NCS `wifi_nrf_fmac_rx_cmd_send`).
    async fn rx_buf_post(&mut self, desc_id: usize) {
        let queue_id = desc_id / RX_BUFS_PER_QUEUE;
        let rpu_addr = c::RPU_MEM_PKT_BASE + (TX_TOTAL_SIZE + RX_BUF_SIZE * desc_id) as u32;

        // write rx buffer header
        self.write32(rpu_addr, None, desc_id as u32).await;

        // Create host_rpu_rx_buf_info (it's just one word of the address)
        let command = [rpu_addr + c::RX_BUF_HEADROOM as u32];

        // Call wifi_nrf_hal_data_cmd_send with the command
        self.rpu_rx_cmd_send(&command, desc_id as u32, queue_id).await;
    }

    async fn rpu_rx_cmd_send(&mut self, command: &[u32], desc_id: u32, pool_id: usize) {
        let addr_base = self.rpu_info.as_ref().unwrap().rx_cmd_base;
        let max_cmd_size = c::RPU_DATA_CMD_SIZE_MAX_RX;

        let addr = addr_base + max_cmd_size * desc_id;
        let host_addr = addr & c::RPU_ADDR_MASK_OFFSET | c::RPU_MCU_CORE_INDIRECT_BASE;

        // Write the command to the core. NCS writes it through the current processor, which is
        // the LMAC once the firmware has booted.
        self.rpu_write_core(host_addr, command, Processor::LMAC).await;

        // Post the updated information to the RPU
        self.rpu_hpq_enqueue(
            self.rpu_info.as_ref().unwrap().hpqm_info.rx_buf_busy_queue[pool_id],
            addr,
        )
        .await;
    }

    async fn rpu_msg_trigger(&mut self) {
        // Indicate to the RPU that the information has been posted
        self.write32(
            c::RPU_REG_INT_TO_MCU_CTRL,
            Some(Processor::UMAC),
            self.num_commands | 0x7fff0000,
        )
        .await;
        self.num_commands = self.num_commands.wrapping_add(1);
    }

    async fn load_fw(&mut self, mem: &MemoryRegion, addr: u32, fw: &[u8]) {
        const FW_CHUNK_SIZE: usize = 1024;
        for (i, chunk) in fw.chunks(FW_CHUNK_SIZE).enumerate() {
            let offs = addr + (FW_CHUNK_SIZE * i) as u32;
            self.raw_write(mem, offs, slice32(chunk)).await;
        }
    }

    #[allow(unused)]
    async fn rpu_wait_until_write_done(&mut self) {
        while self.bus.read_sr0().await & SR0_WRITE_IN_PROGRESS != 0 {}
    }

    async fn rpu_wait_until_awake(&mut self) {
        for _ in 0..10 {
            if self.bus.read_sr1().await & SR1_RPU_AWAKE != 0 {
                return;
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        panic!("awakening never came")
    }

    #[allow(unused)]
    async fn rpu_wait_until_ready(&mut self) {
        for _ in 0..10 {
            if self.bus.read_sr1().await == SR1_RPU_AWAKE | SR1_RPU_READY {
                return;
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        panic!("readyning never came")
    }

    async fn rpu_wait_until_wakeup_req(&mut self) {
        for _ in 0..10 {
            if self.bus.read_sr2().await == SR2_RPU_WAKEUP_REQ {
                return;
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        panic!("wakeup_req never came")
    }

    async fn rpu_wakeup(&mut self) {
        self.bus.write_sr2(SR2_RPU_WAKEUP_REQ).await;
        self.rpu_wait_until_wakeup_req().await;
        self.rpu_wait_until_awake().await;
    }

    #[allow(unused)]
    async fn rpu_sleep(&mut self) {
        self.bus.write_sr2(0).await;
    }

    #[allow(unused)]
    async fn rpu_sleep_status(&mut self) -> u8 {
        self.bus.read_sr1().await
    }

    async fn raw_read32_inner(&mut self, mem: &MemoryRegion, offs: u32) -> u32 {
        assert!(mem.start + offs + 4 <= mem.end);
        let lat = mem.latency as usize;

        let mut buf = [0u32; 3];
        self.bus.read(mem.start + offs, &mut buf[..lat + 1]).await;
        buf[lat]
    }

    async fn raw_read32(&mut self, mem: &MemoryRegion, offs: u32) -> u32 {
        let res = self.raw_read32_inner(mem, offs).await;
        trace!("read32 {:08x} {:08x}", mem.start + offs, res);
        res
    }

    async fn raw_read(&mut self, mem: &MemoryRegion, offs: u32, buf: &mut [u32]) {
        assert!(mem.start + offs + (buf.len() as u32 * 4) <= mem.end);

        // latency=0 optimization doesn't seem to be working, we read the first word repeatedly.
        // Read word by word instead.
        for (i, val) in buf.iter_mut().enumerate() {
            *val = self.raw_read32_inner(mem, offs + i as u32 * 4).await;
        }
        trace!(
            "read addr={:08x} len={:08x} buf={:02x}",
            mem.start + offs,
            buf.len() * 4,
            slice8(buf)
        );
    }
    async fn raw_write32(&mut self, mem: &MemoryRegion, offs: u32, val: u32) {
        self.raw_write(mem, offs, &[val]).await
    }

    async fn raw_write(&mut self, mem: &MemoryRegion, offs: u32, buf: &[u32]) {
        assert!(mem.start + offs + (buf.len() as u32 * 4) <= mem.end);
        trace!(
            "write addr={:08x} len={:08x} buf={:02x}",
            mem.start + offs,
            buf.len() * 4,
            slice8(buf)
        );
        self.bus.write(mem.start + offs, buf).await;
    }

    async fn read32(&mut self, rpu_addr: u32, processor: Option<Processor>) -> u32 {
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(rpu_addr, processor);
        self.raw_read32(mem, offs).await
    }

    async fn read(&mut self, rpu_addr: u32, processor: Option<Processor>, buf: &mut [u32]) {
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(rpu_addr, processor);
        self.raw_read(mem, offs, buf).await
    }

    async fn write32(&mut self, rpu_addr: u32, processor: Option<Processor>, val: u32) {
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(rpu_addr, processor);
        self.raw_write32(mem, offs, val).await
    }

    async fn write(&mut self, rpu_addr: u32, processor: Option<Processor>, buf: &[u32]) {
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(rpu_addr, processor);
        self.raw_write(mem, offs, buf).await
    }

    async fn rpu_write_core(&mut self, core_address: u32, buf: &[u32], processor: Processor) {
        // We receive the address as a byte address, while we need to write it as a word address
        let addr = (core_address & c::RPU_ADDR_MASK_OFFSET) / 4;

        let (addr_reg, data_reg) = match processor {
            Processor::LMAC => (
                c::RPU_REG_MIPS_MCU_SYS_CORE_MEM_CTRL,
                c::RPU_REG_MIPS_MCU_SYS_CORE_MEM_WDATA,
            ),
            Processor::UMAC => (
                c::RPU_REG_MIPS_MCU2_SYS_CORE_MEM_CTRL,
                c::RPU_REG_MIPS_MCU2_SYS_CORE_MEM_WDATA,
            ),
        };

        // Write the processor address register
        self.write32(addr_reg, Some(processor), addr).await;

        // Write to the data register one by one
        for data in buf {
            self.write32(data_reg, Some(processor), *data).await;
        }
    }
}

pub trait Bus {
    async fn read(&mut self, addr: u32, buf: &mut [u32]);
    async fn write(&mut self, addr: u32, buf: &[u32]);
    async fn read_sr0(&mut self) -> u8;
    async fn read_sr1(&mut self) -> u8;
    async fn read_sr2(&mut self) -> u8;
    async fn write_sr2(&mut self, val: u8);
}

pub struct SpiBus<T> {
    spi: T,
}

impl<T> SpiBus<T> {
    pub fn new(spi: T) -> Self {
        Self { spi }
    }
}

impl<T: SpiDevice> Bus for SpiBus<T> {
    async fn read(&mut self, addr: u32, buf: &mut [u32]) {
        self.spi
            .transaction(&mut [
                Operation::Write(&[0x0B, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8, 0x00]),
                Operation::Read(slice8_mut(buf)),
            ])
            .await
            .unwrap()
    }

    async fn write(&mut self, addr: u32, buf: &[u32]) {
        self.spi
            .transaction(&mut [
                Operation::Write(&[0x02, (addr >> 16) as u8 | 0x80, (addr >> 8) as u8, addr as u8]),
                Operation::Write(slice8(buf)),
            ])
            .await
            .unwrap()
    }

    async fn read_sr0(&mut self) -> u8 {
        let mut buf = [0; 2];
        self.spi.transfer(&mut buf, &[0x05]).await.unwrap();
        let val = buf[1];
        defmt::trace!("read sr0 = {:02x}", val);
        val
    }

    async fn read_sr1(&mut self) -> u8 {
        let mut buf = [0; 2];
        self.spi.transfer(&mut buf, &[0x1f]).await.unwrap();
        let val = buf[1];
        defmt::trace!("read sr1 = {:02x}", val);
        val
    }

    async fn read_sr2(&mut self) -> u8 {
        let mut buf = [0; 2];
        self.spi.transfer(&mut buf, &[0x2f]).await.unwrap();
        let val = buf[1];
        defmt::trace!("read sr2 = {:02x}", val);
        val
    }

    async fn write_sr2(&mut self, val: u8) {
        defmt::trace!("write sr2 = {:02x}", val);
        self.spi.write(&[0x3f, val]).await.unwrap();
    }
}

/*
pub struct QspiBus<'a> {
    qspi: Qspi<'a, QSPI>,
}

impl<'a> QspiBus<'a> {}

impl<'a> Bus for QspiBus<'a> {
    async fn read(&mut self, addr: u32, buf: &mut [u32]) {
        self.qspi.read(addr, slice8_mut(buf)).await.unwrap();
    }

    async fn write(&mut self, addr: u32, buf: &[u32]) {
        self.qspi.write(addr, slice8(buf)).await.unwrap();
    }

    async fn read_sr0(&mut self) -> u8 {
        let mut status = [4; 1];
        unwrap!(self.qspi.custom_instruction(0x05, &[0x00], &mut status).await);
        defmt::trace!("read sr0 = {:02x}", status[0]);
        status[0]
    }

    async fn read_sr1(&mut self) -> u8 {
        let mut status = [4; 1];
        unwrap!(self.qspi.custom_instruction(0x1f, &[0x00], &mut status).await);
        defmt::trace!("read sr1 = {:02x}", status[0]);
        status[0]
    }

    async fn read_sr2(&mut self) -> u8 {
        let mut status = [4; 1];
        unwrap!(self.qspi.custom_instruction(0x2f, &[0x00], &mut status).await);
        defmt::trace!("read sr2 = {:02x}", status[0]);
        status[0]
    }

    async fn write_sr2(&mut self, val: u8) {
        defmt::trace!("write sr2 = {:02x}", val);
        unwrap!(self.qspi.custom_instruction(0x3f, &[val], &mut []).await);
    }
}
 */

/// This structure encapsulates the information which represents a HPQ.
#[repr(C)]
#[derive(Debug, defmt::Format, Clone, Copy)]
pub(crate) struct HostRpuHPQ {
    /// HPQ address where the host can post the address of a
    /// message intended for the RPU.
    enqueue_addr: u32,
    /// HPQ address where the host can get the address of a
    /// message intended for the host.
    dequeue_addr: u32,
}

/// Hostport queue information passed by the RPU to the host, which the host can
/// use, to communicate with the RPU.
#[repr(C)]
#[derive(Debug, defmt::Format, Clone, Copy)]
pub(crate) struct HostRpuHPQMInfo {
    /// Queue which the RPU uses to inform the host about events.
    event_busy_queue: HostRpuHPQ,
    /// Queue on which the consumed events are pushed so that RPU can reuse them.
    event_avl_queue: HostRpuHPQ,
    /// Queue used by the host to push commands to the RPU.
    cmd_busy_queue: HostRpuHPQ,
    /// Queue which RPU uses to inform host about command buffers which can be used to push commands to the RPU.
    cmd_avl_queue: HostRpuHPQ,
    rx_buf_busy_queue: [HostRpuHPQ; c::MAX_NUM_OF_RX_QUEUES as usize],
}

#[derive(Debug, defmt::Format)]
pub(crate) struct RpuInfo {
    hpqm_info: HostRpuHPQMInfo,
    /// The base address for posting RX commands.
    rx_cmd_base: u32,
    /// The base address for posting TX commands.
    #[allow(unused)]
    tx_cmd_base: u32,
}
