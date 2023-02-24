#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![deny(unused_must_use)]
#![feature(async_fn_in_trait)]
#![feature(impl_trait_projections)]

use core::slice;

use defmt::{assert, assert_eq, info, panic, todo, *};
use defmt_rtt as _; // global logger
use embassy_executor::Spawner;
use embassy_nrf::config::Config;
use embassy_nrf::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::peripherals::QSPI;
use embassy_nrf::qspi::Qspi;
use embassy_nrf::spim::Spim;
use embassy_nrf::{interrupt, qspi, spim};
use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::{ExclusiveDevice, SpiBus as _, SpiBusRead as _, SpiBusWrite as _, SpiDevice};
use embedded_hal_async::spi_transaction;
use {embassy_nrf as _, panic_probe as _};

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
    let config: Config = Default::default();
    let p = embassy_nrf::init(config);
    spawner.spawn(blink_task(p.P1_06.degrade())).unwrap();

    let sck = p.P0_17;
    let csn = p.P0_18;
    let dio0 = p.P0_13;
    let dio1 = p.P0_14;
    let dio2 = p.P0_15;
    let dio3 = p.P0_16;
    //let coex_req = Output::new(p.P0_28, Level::High, OutputDrive::Standard);
    //let coex_status0 = Output::new(p.P0_30, Level::High, OutputDrive::Standard);
    //let coex_status1 = Output::new(p.P0_29, Level::High, OutputDrive::Standard);
    //let coex_grant = Output::new(p.P0_24, Level::High, OutputDrive::Standard);
    let mut bucken = Output::new(p.P0_12.degrade(), Level::Low, OutputDrive::HighDrive);
    let mut iovdd_ctl = Output::new(p.P0_31.degrade(), Level::Low, OutputDrive::Standard);
    let host_irq = Input::new(p.P0_23.degrade(), Pull::None);

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M8;
    let spim = Spim::new(p.SERIAL0, interrupt::take!(SERIAL0), sck, dio1, dio0, config);
    let csn = Output::new(csn, Level::High, OutputDrive::HighDrive);
    let spi = ExclusiveDevice::new(spim, csn);
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

    let mut nrf70 = Nrf70 {
        bucken,
        iovdd_ctl,
        host_irq,
        bus,
    };

    nrf70.run().await;
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
}

#[rustfmt::skip]
mod regions {
    use super::*;
	pub(crate) const SYSBUS       : &MemoryRegion = &MemoryRegion{ start: 0x000000, end: 0x008FFF, latency: 1 }; // 0xA4xxxxxx
	pub(crate) const EXT_SYS_BUS  : &MemoryRegion = &MemoryRegion{ start: 0x009000, end: 0x03FFFF, latency: 2 };
	pub(crate) const PBUS         : &MemoryRegion = &MemoryRegion{ start: 0x040000, end: 0x07FFFF, latency: 1 }; // 0xA5xxxxxx
	pub(crate) const PKTRAM       : &MemoryRegion = &MemoryRegion{ start: 0x0C0000, end: 0x0F0FFF, latency: 0 }; // 0xB0xxxxxx
	pub(crate) const GRAM         : &MemoryRegion = &MemoryRegion{ start: 0x080000, end: 0x092000, latency: 1 }; // 0xB7xxxxxx
	pub(crate) const LMAC_ROM     : &MemoryRegion = &MemoryRegion{ start: 0x100000, end: 0x134000, latency: 1 }; // 0x80000000 - 0x80033FFF - ROM
	pub(crate) const LMAC_RET_RAM : &MemoryRegion = &MemoryRegion{ start: 0x140000, end: 0x14C000, latency: 1 }; // 0x80040000 - 0x8004BFFF - retained RAM
	pub(crate) const LMAC_SRC_RAM : &MemoryRegion = &MemoryRegion{ start: 0x180000, end: 0x190000, latency: 1 }; // 0x80080000 - 0x8008FFFF - scratch RAM
	pub(crate) const UMAC_ROM     : &MemoryRegion = &MemoryRegion{ start: 0x200000, end: 0x261800, latency: 1 }; // 0x80000000 - 0x800617FF - ROM
	pub(crate) const UMAC_RET_RAM : &MemoryRegion = &MemoryRegion{ start: 0x280000, end: 0x2A4000, latency: 1 }; // 0x80080000 - 0x800A3FFF - retained RAM
	pub(crate) const UMAC_SRC_RAM : &MemoryRegion = &MemoryRegion{ start: 0x300000, end: 0x338000, latency: 1 }; // 0x80100000 - 0x80137FFF - scratch RAM
}
use regions::*;

static FW_LMAC_PATCH_PRI: &[u8] = include_bytes!("../fw/lmac_patch_pri.bin");
static FW_LMAC_PATCH_SEC: &[u8] = include_bytes!("../fw/lmac_patch_sec.bin");
static FW_UMAC_PATCH_PRI: &[u8] = include_bytes!("../fw/umac_patch_pri.bin");
static FW_UMAC_PATCH_SEC: &[u8] = include_bytes!("../fw/umac_patch_sec.bin");

const SR0_WRITE_IN_PROGRESS: u8 = 0x01;

const SR1_RPU_AWAKE: u8 = 0x02;
const SR1_RPU_READY: u8 = 0x04;

const SR2_RPU_WAKEUP_REQ: u8 = 0x01;

struct Nrf70<'a, B: Bus> {
    bus: B,
    bucken: Output<'a, AnyPin>,
    iovdd_ctl: Output<'a, AnyPin>,
    host_irq: Input<'a, AnyPin>,
}

impl<'a, B: Bus> Nrf70<'a, B> {
    pub async fn run(&mut self) {
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

        info!("done!");
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

impl<T: SpiDevice> Bus for SpiBus<T>
where
    T::Bus: embedded_hal_async::spi::SpiBus,
{
    async fn read(&mut self, addr: u32, buf: &mut [u32]) {
        self.spi
            .transaction(move |bus| {
                let bus = unsafe { &mut *bus };
                async move {
                    bus.write(&[0x0B, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8, 0x00])
                        .await?;
                    bus.read(slice8_mut(buf)).await?;
                    Ok(())
                }
            })
            .await
            .unwrap()
    }

    async fn write(&mut self, addr: u32, buf: &[u32]) {
        self.spi
            .transaction(move |bus| {
                let bus = unsafe { &mut *bus };
                async move {
                    bus.write(&[0x02, (addr >> 16) as u8 | 0x80, (addr >> 8) as u8, addr as u8])
                        .await?;
                    bus.write(slice8(buf)).await?;
                    Ok(())
                }
            })
            .await
            .unwrap()
    }

    async fn read_sr0(&mut self) -> u8 {
        let val = self
            .spi
            .transaction(move |bus| {
                let bus = unsafe { &mut *bus };
                async move {
                    let mut buf = [0; 2];
                    bus.transfer(&mut buf, &[0x05]).await?;
                    Ok(buf[1])
                }
            })
            .await
            .unwrap();
        trace!("read sr0 = {:02x}", val);
        val
    }

    async fn read_sr1(&mut self) -> u8 {
        let val = self
            .spi
            .transaction(move |bus| {
                let bus = unsafe { &mut *bus };
                async move {
                    let mut buf = [0; 2];
                    bus.transfer(&mut buf, &[0x1f]).await?;
                    Ok(buf[1])
                }
            })
            .await
            .unwrap();
        trace!("read sr1 = {:02x}", val);
        val
    }

    async fn read_sr2(&mut self) -> u8 {
        let val = self
            .spi
            .transaction(move |bus| {
                let bus = unsafe { &mut *bus };
                async move {
                    let mut buf = [0; 2];
                    bus.transfer(&mut buf, &[0x2f]).await?;
                    Ok(buf[1])
                }
            })
            .await
            .unwrap();
        trace!("read sr2 = {:02x}", val);
        val
    }

    async fn write_sr2(&mut self, val: u8) {
        trace!("write sr2 = {:02x}", val);
        self.spi
            .transaction(move |bus| {
                let bus = unsafe { &mut *bus };
                async move {
                    bus.write(&[0x3f, val]).await?;
                    Ok(())
                }
            })
            .await
            .unwrap()
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
        trace!("read sr0 = {:02x}", status[0]);
        status[0]
    }

    async fn read_sr1(&mut self) -> u8 {
        let mut status = [4; 1];
        unwrap!(self.qspi.custom_instruction(0x1f, &[0x00], &mut status).await);
        trace!("read sr1 = {:02x}", status[0]);
        status[0]
    }

    async fn read_sr2(&mut self) -> u8 {
        let mut status = [4; 1];
        unwrap!(self.qspi.custom_instruction(0x2f, &[0x00], &mut status).await);
        trace!("read sr2 = {:02x}", status[0]);
        status[0]
    }

    async fn write_sr2(&mut self, val: u8) {
        trace!("write sr2 = {:02x}", val);
        unwrap!(self.qspi.custom_instruction(0x3f, &[val], &mut []).await);
    }
}
