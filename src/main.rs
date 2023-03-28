#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![deny(unused_must_use)]
#![allow(incomplete_features)]
#![feature(async_fn_in_trait)]
#![feature(impl_trait_projections)]

use core::future::Future;
use core::mem::{size_of, size_of_val};
use core::slice;

use align_data::{include_aligned, Align16};
use bbqueue::BBBuffer;
use config::Config;
use defmt::{assert, info, panic, unwrap};
use defmt_rtt as _; // global logger
use embassy_executor::Spawner;
use embassy_nrf::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::gpiote::{InputChannel, InputChannelPolarity};
use embassy_nrf::peripherals::QSPI;
use embassy_nrf::qspi::Qspi;
use embassy_nrf::spim::Spim;
use embassy_nrf::{bind_interrupts, spim};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal::spi::Operation;
use embedded_hal_async::spi::{ExclusiveDevice, SpiDevice};
use heapless::Vec;
use messages::{commands, RpuMessage};
use regions::*;
use {embassy_nrf as _, panic_probe as _};

use crate::config::RX_BUF_HEADROOM;
use crate::messages::commands::sys::structures::TempVbatConfig;
use crate::messages::RpuMessageHeader;

pub mod config;
pub(crate) mod messages;

bind_interrupts!(struct Irqs {
    SERIAL0 => spim::InterruptHandler<embassy_nrf::peripherals::SERIAL0>;
});

#[embassy_executor::task]
async fn blink_task(led: AnyPin) -> ! {
    let mut led = Output::new(led, Level::High, OutputDrive::Standard);
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(100)).await;
        led.set_low();
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");
    let config: embassy_nrf::config::Config = Default::default();
    let p = embassy_nrf::init(config);
    spawner.spawn(blink_task(p.P1_06.degrade())).unwrap();

    let sck = p.P0_17;
    let csn = p.P0_18;
    let dio0 = p.P0_13;
    let dio1 = p.P0_14;
    let _dio2 = p.P0_15;
    let _dio3 = p.P0_16;
    //let coex_req = Output::new(p.P0_28, Level::High, OutputDrive::Standard);
    //let coex_status0 = Output::new(p.P0_30, Level::High, OutputDrive::Standard);
    //let coex_status1 = Output::new(p.P0_29, Level::High, OutputDrive::Standard);
    //let coex_grant = Output::new(p.P0_24, Level::High, OutputDrive::Standard);
    let bucken = Output::new(p.P0_12.degrade(), Level::Low, OutputDrive::HighDrive);
    let iovdd_ctl = Output::new(p.P0_31.degrade(), Level::Low, OutputDrive::Standard);
    let host_irq = InputChannel::new(
        p.GPIOTE_CH0,
        Input::new(p.P0_23, Pull::None),
        InputChannelPolarity::LoToHi,
    );

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M8;
    let spim = Spim::new(p.SERIAL0, Irqs, sck, dio1, dio0, config);
    let csn = Output::new(csn, Level::High, OutputDrive::HighDrive);
    let spi = ExclusiveDevice::new(spim, csn, Delay);
    let bus = SpiBus { spi };

    /*
    // QSPI is not working well yet.
    let mut config = qspi::Config::default();
    config.read_opcode = qspi::ReadOpcode::READ4IO;
    config.write_opcode = qspi::WriteOpcode::PP4IO;
    config.write_page_size = qspi::WritePageSize::_256BYTES;
    config.frequency = qspi::Frequency::M8; // NOTE: Waking RPU works reliably only with lowest frequency (8MHz)

    let irq = interrupt::take!(QSPI);
    let qspi: qspi::Qspi<_> = qspi::Qspi::new(p.QSPI, irq, sck, csn, dio0, dio1, dio2, dio3, config);
    let bus = QspiBus { qspi };
    */

    let mut nrf70 = Nrf70::new(bus, bucken, iovdd_ctl, Config::new_only_scan());

    info!("Size of nrf70: {}", size_of_val(&nrf70));

    nrf70.start().await;

    loop {
        host_irq.wait().await;
        nrf70
            .rpu_irq_process(|_data| async { defmt::info!("Received an event") })
            .await;
    }
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

fn slice32_mut(x: &mut [u8]) -> &mut [u32] {
    assert!(x.len() % 4 == 0);
    assert!(x.as_ptr() as usize % 4 == 0);
    let len = x.len() / 4;
    unsafe { slice::from_raw_parts_mut(x.as_ptr() as _, len) }
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

static FW_LMAC_PATCH_PRI: &[u8] = include_aligned!(Align16, "../fw/lmac_patch_pri.bin");
static FW_LMAC_PATCH_SEC: &[u8] = include_aligned!(Align16, "../fw/lmac_patch_sec.bin");
static FW_UMAC_PATCH_PRI: &[u8] = include_aligned!(Align16, "../fw/umac_patch_pri.bin");
static FW_UMAC_PATCH_SEC: &[u8] = include_aligned!(Align16, "../fw/umac_patch_sec.bin");

const SR0_WRITE_IN_PROGRESS: u8 = 0x01;

const SR1_RPU_AWAKE: u8 = 0x02;
const SR1_RPU_READY: u8 = 0x04;

const SR2_RPU_WAKEUP_REQ: u8 = 0x01;

const MAX_EVENT_POOL_LEN: usize = 1000;

struct Nrf70<
    'a,
    B: Bus,
    const TX_BUFFERS: usize,
    const TX_BUFFER_SIZE_MAX: usize,
    const RX_BUFFERS: usize,
    const RX_BUFFER_SIZE_MAX: usize,
> {
    bus: B,
    bucken: Output<'a, AnyPin>,
    iovdd_ctl: Output<'a, AnyPin>,

    config: Config<TX_BUFFERS, TX_BUFFER_SIZE_MAX, RX_BUFFERS, RX_BUFFER_SIZE_MAX>,
    rpu_info: Option<RpuInfo>,

    tx_buffers: [BBBuffer<TX_BUFFER_SIZE_MAX>; TX_BUFFERS],
    rx_buffers: [BBBuffer<RX_BUFFER_SIZE_MAX>; RX_BUFFERS],

    num_rx_commands: u32,
}

impl<
        'a,
        B: Bus,
        const TX_BUFFERS: usize,
        const TX_BUFFER_SIZE_MAX: usize,
        const RX_BUFFERS: usize,
        const RX_BUFFER_SIZE_MAX: usize,
    > Nrf70<'a, B, TX_BUFFERS, TX_BUFFER_SIZE_MAX, RX_BUFFERS, RX_BUFFER_SIZE_MAX>
{
    const DEFAULT_TX_BBBUFFER: BBBuffer<{ TX_BUFFER_SIZE_MAX }> = BBBuffer::new();
    const DEFAULT_RX_BBBUFFER: BBBuffer<{ RX_BUFFER_SIZE_MAX }> = BBBuffer::new();

    const RPU_CMD_START_MAGIC: u32 = 0xDEAD;

    const fn new(
        bus: B,
        bucken: Output<'a, AnyPin>,
        iovdd_ctl: Output<'a, AnyPin>,
        config: Config<TX_BUFFERS, TX_BUFFER_SIZE_MAX, RX_BUFFERS, RX_BUFFER_SIZE_MAX>,
    ) -> Self {
        Self {
            bus,
            bucken,
            iovdd_ctl,
            config,
            rpu_info: None,
            tx_buffers: [Self::DEFAULT_TX_BBBUFFER; TX_BUFFERS],
            rx_buffers: [Self::DEFAULT_RX_BBBUFFER; RX_BUFFERS],
            num_rx_commands: Self::RPU_CMD_START_MAGIC,
        }
    }

    pub async fn start(&mut self) {
        info!("power on...");
        Timer::after(Duration::from_millis(10)).await;
        self.bucken.set_high();
        Timer::after(Duration::from_millis(10)).await;
        self.iovdd_ctl.set_high();
        Timer::after(Duration::from_millis(10)).await;

        info!("wakeup...");
        self.rpu_wakeup().await;

        info!("enable clocks...");
        self.rpu_write32(PBUS, 0x8C20, 0x0100).await;

        info!("enable interrupt...");
        // First enable the blockwise interrupt for the relevant block in the master register
        let mut val = self.rpu_read32(SYSBUS, 0x400).await;
        val |= 1 << 17;
        self.rpu_write32(SYSBUS, 0x400, val).await;

        // Now enable the relevant MCU interrupt line
        self.rpu_write32(SYSBUS, 0x494, 1 << 31).await;

        info!("load LMAC firmware patches...");
        self.rpu_write32(SYSBUS, 0x000, 0x01).await; // reset
        while self.rpu_read32(SYSBUS, 0x0000).await & 0x01 != 0 {}
        while self.rpu_read32(SYSBUS, 0x0018).await & 0x01 != 1 {}
        self.load_fw(LMAC_RET_RAM, 0x9000, FW_LMAC_PATCH_PRI).await;
        self.load_fw(LMAC_RET_RAM, 0x4000, FW_LMAC_PATCH_SEC).await;

        self.rpu_write32(GRAM, 0xD50, 0).await;
        self.rpu_write32(SYSBUS, 0x50, 0x3c1a8000).await;
        self.rpu_write32(SYSBUS, 0x54, 0x275a0000).await;
        self.rpu_write32(SYSBUS, 0x58, 0x03400008).await;
        self.rpu_write32(SYSBUS, 0x5c, 0x00000000).await;
        self.rpu_write32(SYSBUS, 0x2C2c, 0x9000).await;

        info!("booting LMAC...");
        self.rpu_write32(SYSBUS, 0x000, 0x01).await; // reset
        while self.rpu_read32(GRAM, 0xD50).await != 0x5A5A5A5A {}

        info!("load UMAC firmware patches...");
        self.rpu_write32(SYSBUS, 0x100, 0x01).await; // reset
        while self.rpu_read32(SYSBUS, 0x0100).await & 0x01 != 0 {}
        while self.rpu_read32(SYSBUS, 0x0118).await & 0x01 != 1 {}
        self.load_fw(UMAC_RET_RAM, 0x14400, FW_UMAC_PATCH_PRI).await;
        self.load_fw(UMAC_RET_RAM, 0xC000, FW_UMAC_PATCH_SEC).await;

        self.rpu_write32(PKTRAM, 0, 0).await;
        self.rpu_write32(SYSBUS, 0x150, 0x3c1a8000).await;
        self.rpu_write32(SYSBUS, 0x154, 0x275a0000).await;
        self.rpu_write32(SYSBUS, 0x158, 0x03400008).await;
        self.rpu_write32(SYSBUS, 0x15c, 0x00000000).await;
        self.rpu_write32(SYSBUS, 0x2C30, 0x14400).await;

        info!("booting UMAC...");
        self.rpu_write32(SYSBUS, 0x100, 0x01).await; // reset
        while self.rpu_read32(PKTRAM, 0).await != 0x5A5A5A5A {}

        info!("Initializing rpu info...");
        self.init_rpu_info().await;

        info!("Enabling interrupts...");
        self.rpu_irq_enable().await;

        info!("Initializing TX...");
        self.init_tx().await;

        info!("Initializing RX...");
        self.init_rx().await;

        info!("Initializing umac...");
        self.init_umac().await;

        info!("done!");
    }

    const RPU_REG_INT_FROM_RPU_CTRL: u32 = 0xA4000400;
    const RPU_REG_BIT_INT_FROM_RPU_CTRL: u32 = 17;
    const RPU_REG_INT_FROM_MCU_CTRL: u32 = 0xA4000494;
    const RPU_REG_BIT_INT_FROM_MCU_CTRL: u32 = 31;
    const RPU_REG_INT_FROM_MCU_ACK: u32 = 0xA4000488;
    const RPU_REG_BIT_INT_FROM_MCU_ACK: u32 = 31;
    const RPU_REG_MIPS_MCU_UCCP_INT_STATUS: u32 = 0xA4000004;
    const RPU_REG_BIT_MIPS_WATCHDOG_INT_STATUS: u32 = 1;
    const RPU_REG_MIPS_MCU_TIMER_CONTROL: u32 = 0xA4000048;

    async fn rpu_irq_enable(&mut self) {
        // First enable the blockwise interrupt for the relevant block in the master register

        let (mem, offs) = remap_global_addr_to_region_and_offset(Self::RPU_REG_INT_FROM_RPU_CTRL, None);
        let mut val = self.rpu_read32(mem, offs).await;

        val |= 1 << Self::RPU_REG_BIT_INT_FROM_RPU_CTRL;

        self.rpu_write32(mem, offs, val).await;

        // Now enable the relevant MCU interrupt line
        let (mem, offs) = remap_global_addr_to_region_and_offset(Self::RPU_REG_INT_FROM_MCU_CTRL, None);
        self.rpu_write32(mem, offs, 1 << Self::RPU_REG_BIT_INT_FROM_MCU_CTRL)
            .await;
    }

    async fn rpu_irq_disable(&mut self) {
        let (mem, offs) = remap_global_addr_to_region_and_offset(Self::RPU_REG_INT_FROM_RPU_CTRL, None);
        let mut val = self.rpu_read32(mem, offs).await;

        val &= !(1 << Self::RPU_REG_BIT_INT_FROM_RPU_CTRL);

        self.rpu_write32(mem, offs, val).await;

        let (mem, offs) = remap_global_addr_to_region_and_offset(Self::RPU_REG_INT_FROM_MCU_CTRL, None);
        self.rpu_write32(mem, offs, !(1 << Self::RPU_REG_BIT_INT_FROM_MCU_CTRL))
            .await;
    }

    async fn rpu_irq_ack(&mut self) {
        // Guess: I think this clears the interrupt flag
        let (mem, offs) = remap_global_addr_to_region_and_offset(Self::RPU_REG_INT_FROM_MCU_ACK, None);
        self.rpu_write32(mem, offs, 1 << Self::RPU_REG_BIT_INT_FROM_MCU_ACK)
            .await;
    }

    /// Checks if the watchdog was the source of the interrupt
    async fn rpu_irq_watchdog_check(&mut self) -> bool {
        let (mem, offs) = remap_global_addr_to_region_and_offset(Self::RPU_REG_MIPS_MCU_UCCP_INT_STATUS, None);
        let val = self.rpu_read32(mem, offs).await;
        (val & (1 << Self::RPU_REG_BIT_MIPS_WATCHDOG_INT_STATUS)) > 0
    }

    async fn rpu_irq_watchdog_ack(&mut self) {
        let (mem, offs) = remap_global_addr_to_region_and_offset(Self::RPU_REG_MIPS_MCU_TIMER_CONTROL, None);
        self.rpu_write32(mem, offs, 0).await;
    }

    /// Must be called when the interrupt pin is triggered
    pub async fn rpu_irq_process<F, FR>(&mut self, mut callback: F)
    where
        F: FnMut(Vec<u8, MAX_EVENT_POOL_LEN>) -> FR,
        FR: Future,
    {
        let event_count = self.rpu_event_process_all(&mut callback).await;

        if event_count == 0 && self.rpu_irq_watchdog_check().await {
            self.rpu_irq_watchdog_ack().await;
        }

        self.rpu_irq_ack().await;
    }

    /// Get all events. Every event is processed by the given callback.
    /// The total amount of events that were processed is returned.
    async fn rpu_event_process_all<F, FR>(&mut self, mut callback: F) -> u32
    where
        F: FnMut(Vec<u8, MAX_EVENT_POOL_LEN>) -> FR,
        FR: Future,
    {
        let mut event_count = 0;

        loop {
            let event_address = self
                .rpu_hpq_dequeue(self.rpu_info.as_ref().unwrap().hpqm_info.event_busy_queue)
                .await;

            let event_address = match event_address {
                None | Some(0xAAAAAAAA) => {
                    // No more events to read. Sometimes when low power mode is enabled
                    // we see a wrong address, but it work after a while, so, add a
                    // check for that.
                    break;
                }
                Some(event_address) => event_address,
            };

            self.rpu_event_process(event_address, &mut callback).await;

            event_count += 1;
        }

        event_count
    }

    async fn rpu_event_process<F, FR>(&mut self, event_address: u32, callback: &mut F)
    where
        F: FnMut(Vec<u8, MAX_EVENT_POOL_LEN>) -> FR,
        FR: Future,
    {
        const RPU_EVENT_COMMON_SIZE_MAX: usize = 128;
        let mut event_data = Vec::new();
        event_data.resize_default(RPU_EVENT_COMMON_SIZE_MAX).unwrap();

        let (mem, offs) = remap_global_addr_to_region_and_offset(event_address, None);
        self.rpu_read(mem, offs, slice32_mut(&mut event_data)).await;

        // Get the header from the front of the event data
        let message_header: RpuMessageHeader =
            unsafe { core::mem::transmute_copy(event_data.as_ptr().as_ref().unwrap()) };

        if message_header.length <= RPU_EVENT_COMMON_SIZE_MAX as u32 {
            event_data.truncate(message_header.length as usize);
        } else if message_header.length as usize <= MAX_EVENT_POOL_LEN {
            // This is a longer than usual event. We gotta read it again
            let Ok(_) = event_data.resize_default(message_header.length as usize) else {
                defmt::panic!("Event is too big ({} bytes)! Either the buffer has to be increased or we need to implement fragmented event reading which is something the C lib does", message_header.length);
            };
            self.rpu_read(mem, offs, slice32_mut(&mut event_data)).await;
        } else {
            todo!("Fragmented event read is not yet implemented");
        }

        callback(event_data).await;

        if message_header.resubmit > 0 {
            self.rpu_event_free(event_address).await;
        }
    }

    async fn rpu_event_free(&mut self, event_address: u32) {
        self.rpu_hpq_enqueue(self.rpu_info.as_ref().unwrap().hpqm_info.event_avl_queue, event_address)
            .await;
    }

    pub async fn rpu_cmd_ctrl_send(&mut self, message: &[u8]) {
        const MAX_NRF_WIFI_UMAC_CMD_SIZE: usize = 400;

        if message.len() > MAX_NRF_WIFI_UMAC_CMD_SIZE {
            todo!("Fragmenting commands is not yet implemented");
        } else {
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
            let (mem, offs) = remap_global_addr_to_region_and_offset(message_address, None);
            self.rpu_write(mem, offs, slice32(message)).await;

            // Post the updated information to the RPU
            self.rpu_hpq_enqueue(
                self.rpu_info.as_ref().unwrap().hpqm_info.cmd_busy_queue,
                message_address,
            )
            .await;
        }
    }

    async fn rpu_hpq_enqueue(&mut self, hpq: HostRpuHPQ, value: u32) {
        let (mem, offs) = remap_global_addr_to_region_and_offset(hpq.enqueue_addr, None);
        self.rpu_write32(mem, offs, value).await;
    }

    async fn rpu_hpq_dequeue(&mut self, hpq: HostRpuHPQ) -> Option<u32> {
        let (mem, offs) = remap_global_addr_to_region_and_offset(hpq.dequeue_addr, None);
        let value = self.rpu_read32(mem, offs).await;

        // Pop element only if it is valid
        if value != 0 {
            self.rpu_write32(mem, offs, value).await;

            Some(value)
        } else {
            None
        }
    }

    const RPU_MEM_HPQ_INFO: u32 = 0xB0000024;
    const RPU_MEM_RX_CMD_BASE: u32 = 0xB7000D58;
    const RPU_MEM_TX_CMD_BASE: u32 = 0xB00000B8;

    async fn init_rpu_info(&mut self) {
        // Based on 'wifi_nrf_hal_dev_init'

        let mut hpqm_info = [0; size_of::<HostRpuHPQMInfo>()];
        let (mem, offs) = remap_global_addr_to_region_and_offset(Self::RPU_MEM_HPQ_INFO, None);
        self.rpu_read(mem, offs, slice32_mut(&mut hpqm_info)).await;

        let (mem, offs) = remap_global_addr_to_region_and_offset(Self::RPU_MEM_RX_CMD_BASE, None);
        let rx_cmd_base = self.rpu_read32(mem, offs).await;

        self.rpu_info = Some(RpuInfo {
            hpqm_info: unsafe { core::mem::transmute_copy(&hpqm_info) },
            rx_cmd_base,
            tx_cmd_base: Self::RPU_MEM_TX_CMD_BASE,
        });
    }

    async fn init_umac(&mut self) {
        const HW_DELAY: u32 = 7300;
        const SW_DELAY: u32 = 5000;
        const BCN_TIMEOUT: u32 = 40000;
        const CALIB_SLEEP_CLOCK_ENABLE: u32 = 1;

        const NRF_WIFI_DEF_PHY_CALIB: u32 = 196667;

        const NRF_WIFI_TEMP_CALIB_ENABLE: u32 = 1;
        const NRF_WIFI_DEF_PHY_TEMP_CALIB: u32 = 1 | 2 | 16 | 8 | 0 | 32;
        const NRF_WIFI_DEF_PHY_VBAT_CALIB: u32 = 32;
        const NRF_WIFI_TEMP_CALIB_PERIOD: u32 = 1024 * 1024;
        const NRF_WIFI_VBAT_LOW: i32 = 6;
        const NRF_WIFI_VBAT_HIGH: i32 = 12;
        const NRF_WIFI_TEMP_CALIB_THRESHOLD: i32 = 40;

        let umac_cmd = RpuMessage::new(commands::sys::SysCommand::new(commands::sys::Init {
            wdev_id: 0,
            sys_params: commands::sys::structures::SysParams {
                sleep_enable: 0, // TODO for low power
                hw_bringup_time: HW_DELAY,
                sw_bringup_time: SW_DELAY,
                bcn_time_out: BCN_TIMEOUT,
                calib_sleep_clk: CALIB_SLEEP_CLOCK_ENABLE,
                phy_calib: NRF_WIFI_DEF_PHY_CALIB,
                mac_addr: [0; 6],
                rf_params: [0; 200],
                rf_params_valid: 0,
            },
            rx_buf_pools: self.config.rx_buf_pools,
            data_config_params: self.config.data_config,
            temp_vbat_config_params: TempVbatConfig {
                temp_based_calib_en: NRF_WIFI_TEMP_CALIB_ENABLE,
                temp_calib_bitmap: NRF_WIFI_DEF_PHY_TEMP_CALIB,
                vbat_calibp_bitmap: NRF_WIFI_DEF_PHY_VBAT_CALIB,
                temp_vbat_mon_period: NRF_WIFI_TEMP_CALIB_PERIOD,
                vth_very_low: 0,
                vth_low: NRF_WIFI_VBAT_LOW,
                vth_hi: NRF_WIFI_VBAT_HIGH,
                temp_threshold: NRF_WIFI_TEMP_CALIB_THRESHOLD,
                vbat_threshold: 0,
            },
        }));

        let cmd_bytes = unsafe {
            core::slice::from_raw_parts(
                &umac_cmd as *const RpuMessage<commands::sys::SysCommand<commands::sys::Init>> as *const u8,
                size_of::<RpuMessage<commands::sys::SysCommand<commands::sys::Init>>>(),
            )
        };

        unwrap!(embassy_time::with_timeout(Duration::from_secs(1), self.rpu_cmd_ctrl_send(cmd_bytes)).await);
    }

    async fn init_tx(&mut self) {
        if TX_BUFFERS == 0 {
            return;
        }

        todo!("Implement `tx_init`");
    }

    async fn init_rx(&mut self) {
        const WIFI_NRF_FMAC_RX_CMD_TYPE_INIT: u32 = 0;

        for desc_id in 0..RX_BUFFERS {
            let pool_info = self.map_desc_to_pool(desc_id as u32);

            let (pool_rpu_region, pool_rpu_base) = self.config.rpu_rx_buffer_base[pool_info.pool_id as usize];
            let bounce_buffer_address = pool_rpu_base + pool_info.buf_id * RX_BUFFER_SIZE_MAX as u32;

            let mut initial_buffer = [0; RX_BUFFER_SIZE_MAX];
            let initial_buffer = slice32_mut(&mut initial_buffer);

            initial_buffer[0] = desc_id as u32;

            // Reset the RPU buffer to 0
            self.rpu_write(pool_rpu_region, bounce_buffer_address, &initial_buffer)
                .await;

            // Create host_rpu_rx_buf_info (it's just one word of the address)
            let command = [pool_rpu_region.rpu_mem_start + bounce_buffer_address + RX_BUF_HEADROOM as u32];

            // Call wifi_nrf_hal_data_cmd_send with the command
            self.rpu_rx_cmd_send(&command, desc_id as u32, pool_info.pool_id as usize)
                .await;
        }
    }

    const RPU_ADDR_MASK_OFFSET: u32 = 0x00FFFFFF;
    const RPU_MCU_CORE_INDIRECT_BASE: u32 = 0xC0000000;
    const RPU_REG_INT_TO_MCU_CTRL: u32 = 0xA4000480;

    async fn rpu_rx_cmd_send(&mut self, command: &[u32], desc_id: u32, pool_id: usize) {
        const RPU_DATA_CMD_SIZE_MAX_RX: u32 = 8;

        let addr_base = self.rpu_info.as_ref().unwrap().rx_cmd_base;
        let max_cmd_size = RPU_DATA_CMD_SIZE_MAX_RX;

        let addr = addr_base + max_cmd_size * desc_id;
        let host_addr = addr & Self::RPU_ADDR_MASK_OFFSET | Self::RPU_MCU_CORE_INDIRECT_BASE;

        // Write the command to the core
        self.rpu_write_core(host_addr, command, Processor::UMAC).await; // UMAC is a guess here

        // Post the updated information to the RPU
        self.rpu_hpq_enqueue(
            self.rpu_info.as_ref().unwrap().hpqm_info.rx_buf_busy_queue[pool_id],
            addr,
        )
        .await;

        // Indicate to the RPU that the information has been posted
        let (mem, offs) =
            regions::remap_global_addr_to_region_and_offset(Self::RPU_REG_INT_TO_MCU_CTRL, Some(Processor::UMAC));
        self.rpu_write32(mem, offs, self.num_rx_commands | 0x7fff0000).await;
        self.num_rx_commands = self.num_rx_commands.wrapping_add(1);
    }

    fn map_desc_to_pool(&mut self, desc_id: u32) -> RxPoolMapInfo {
        for pool_id in 0..MAX_NUM_OF_RX_QUEUES {
            if desc_id >= self.config.rx_desc[pool_id]
                && desc_id < self.config.rx_desc[pool_id] + self.config.rx_buf_pools[pool_id].num_bufs as u32
            {
                return RxPoolMapInfo {
                    pool_id: pool_id as u32,
                    buf_id: desc_id - self.config.rx_desc[pool_id],
                };
            }
        }

        defmt::panic!("desc_id is too high: {}", desc_id);
    }

    async fn load_fw(&mut self, mem: &MemoryRegion, addr: u32, fw: &[u8]) {
        const FW_CHUNK_SIZE: usize = 1024;
        for (i, chunk) in fw.chunks(FW_CHUNK_SIZE).enumerate() {
            let offs = addr + (FW_CHUNK_SIZE * i) as u32;
            self.rpu_write(mem, offs, slice32(chunk)).await;
        }
    }

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

    async fn rpu_sleep(&mut self) {
        self.bus.write_sr2(0).await;
    }

    async fn rpu_sleep_status(&mut self) -> u8 {
        self.bus.read_sr1().await
    }

    async fn rpu_read32(&mut self, mem: &MemoryRegion, offs: u32) -> u32 {
        assert!(mem.start + offs + 4 <= mem.end);
        let lat = mem.latency as usize;

        let mut buf = [0u32; 3];
        self.bus.read(mem.start + offs, &mut buf[..lat + 1]).await;
        buf[lat]
    }

    async fn rpu_read(&mut self, mem: &MemoryRegion, offs: u32, buf: &mut [u32]) {
        assert!(mem.start + offs + (buf.len() as u32 * 4) <= mem.end);

        if mem.latency == 0 {
            // No latency, we can do a big read directly.
            self.bus.read(mem.start + offs, buf).await;
        } else {
            // Otherwise, read word by word.
            for (i, val) in buf.iter_mut().enumerate() {
                *val = self.rpu_read32(mem, offs + i as u32 * 4).await;
            }
        }
    }
    async fn rpu_write32(&mut self, mem: &MemoryRegion, offs: u32, val: u32) {
        self.rpu_write(mem, offs, &[val]).await
    }

    async fn rpu_write(&mut self, mem: &MemoryRegion, offs: u32, buf: &[u32]) {
        assert!(mem.start + offs + (buf.len() as u32 * 4) <= mem.end);
        self.bus.write(mem.start + offs, buf).await;
    }

    async fn rpu_write_core(&mut self, core_address: u32, buf: &[u32], processor: Processor) {
        // We receive the address as a byte address, while we need to write it as a word address
        let addr = (core_address & Self::RPU_ADDR_MASK_OFFSET) / 4;

        const RPU_REG_MIPS_MCU_SYS_CORE_MEM_CTRL: u32 = 0xA4000030;
        const RPU_REG_MIPS_MCU_SYS_CORE_MEM_WDATA: u32 = 0xA4000034;
        const RPU_REG_MIPS_MCU2_SYS_CORE_MEM_CTRL: u32 = 0xA4000130;
        const RPU_REG_MIPS_MCU2_SYS_CORE_MEM_WDATA: u32 = 0xA4000134;

        let (addr_reg, data_reg) = match processor {
            Processor::LMAC => (RPU_REG_MIPS_MCU_SYS_CORE_MEM_CTRL, RPU_REG_MIPS_MCU_SYS_CORE_MEM_WDATA),
            Processor::UMAC => (
                RPU_REG_MIPS_MCU2_SYS_CORE_MEM_CTRL,
                RPU_REG_MIPS_MCU2_SYS_CORE_MEM_WDATA,
            ),
        };

        // Write the processor address register
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(addr_reg, Some(processor));
        self.rpu_write32(mem, offs, addr).await;

        // Write to the data register one by one
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(data_reg, Some(processor));
        for data in buf {
            self.rpu_write32(mem, offs, *data).await;
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

struct SpiBus<T> {
    spi: T,
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

struct QspiBus<'a> {
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

const MAX_NUM_OF_RX_QUEUES: usize = 3;

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
    rx_buf_busy_queue: [HostRpuHPQ; MAX_NUM_OF_RX_QUEUES],
}

#[derive(Debug, defmt::Format)]
pub(crate) struct RpuInfo {
    hpqm_info: HostRpuHPQMInfo,
    /// The base address for posting RX commands.
    rx_cmd_base: u32,
    /// The base address for posting TX commands.
    tx_cmd_base: u32,
}

struct RxPoolMapInfo {
    pool_id: u32,
    buf_id: u32,
}
