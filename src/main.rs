#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![deny(unused_must_use)]

use core::slice;

use defmt::{assert, assert_eq, info, panic, *};
use defmt_rtt as _; // global logger
use embassy_executor::Spawner;
use embassy_nrf::config::Config;
use embassy_nrf::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::peripherals::QSPI;
use embassy_nrf::qspi::Qspi;
use embassy_nrf::{interrupt, qspi};
use embassy_time::{Duration, Timer};
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

    let mut config = qspi::Config::default();
    config.read_opcode = qspi::ReadOpcode::READ4IO;
    config.write_opcode = qspi::WriteOpcode::PP4IO;
    config.write_page_size = qspi::WritePageSize::_256BYTES;
    config.frequency = qspi::Frequency::M8; // NOTE: Waking RPU works reliably only with lowest frequency (8MHz)

    let irq = interrupt::take!(QSPI);
    let qspi: qspi::Qspi<_> = qspi::Qspi::new(
        p.QSPI, irq, p.P0_17, p.P0_18, p.P0_13, p.P0_14, p.P0_15, p.P0_16, config,
    );

    let mut bucken = Output::new(p.P0_12.degrade(), Level::Low, OutputDrive::HighDrive);
    let mut iovdd_ctl = Output::new(p.P0_31.degrade(), Level::Low, OutputDrive::Standard);
    let host_irq = Input::new(p.P0_23.degrade(), Pull::None);

    let mut nrf70 = Nrf70 {
        bucken,
        iovdd_ctl,
        host_irq,
        qspi,
    };

    nrf70.run().await;

    //let coex_req = Output::new(p.P0_28, Level::High, OutputDrive::Standard);
    //let coex_status0 = Output::new(p.P0_30, Level::High, OutputDrive::Standard);
    //let coex_status1 = Output::new(p.P0_29, Level::High, OutputDrive::Standard);
    //let coex_grant = Output::new(p.P0_24, Level::High, OutputDrive::Standard);
}

fn slice8(x: &[u32]) -> &[u8] {
    let len = x.len() * 4;
    unsafe { slice::from_raw_parts(x.as_ptr() as _, len) }
}

fn slice8_mut(x: &mut [u32]) -> &mut [u8] {
    let len = x.len() * 4;
    unsafe { slice::from_raw_parts_mut(x.as_mut_ptr() as _, len) }
}

#[derive(Copy, Clone, Debug, defmt::Format)]
struct MemoryRegion {
    start: u32,
    end: u32,

    /// Number of dummy 32bit words
    latency: u32,
}

#[rustfmt::skip]
impl MemoryRegion {
	const SYSBUS       : Self = Self{ start: 0x000000, end: 0x008FFF, latency: 1 };
	const EXT_SYS_BUS  : Self = Self{ start: 0x009000, end: 0x03FFFF, latency: 2 };
	const PBUS         : Self = Self{ start: 0x040000, end: 0x07FFFF, latency: 1 };
	const PKTRAM       : Self = Self{ start: 0x0C0000, end: 0x0F0FFF, latency: 0 };
	const GRAM         : Self = Self{ start: 0x080000, end: 0x092000, latency: 1 };
	const LMAC_ROM     : Self = Self{ start: 0x100000, end: 0x134000, latency: 1 };
	const LMAC_RET_RAM : Self = Self{ start: 0x140000, end: 0x14C000, latency: 1 };
	const LMAC_SRC_RAM : Self = Self{ start: 0x180000, end: 0x190000, latency: 1 };
	const UMAC_ROM     : Self = Self{ start: 0x200000, end: 0x261800, latency: 1 };
	const UMAC_RET_RAM : Self = Self{ start: 0x280000, end: 0x2A4000, latency: 1 };
	const UMAC_SRC_RAM : Self = Self{ start: 0x300000, end: 0x338000, latency: 1 };
}

const SR0_WRITE_IN_PROGRESS: u8 = 0x01;

const SR1_RPU_AWAKE: u8 = 0x02;
const SR1_RPU_READY: u8 = 0x04;

const SR2_RPU_WAKEUP_REQ: u8 = 0x01;

struct Nrf70<'a> {
    qspi: Qspi<'a, QSPI>,
    bucken: Output<'a, AnyPin>,
    iovdd_ctl: Output<'a, AnyPin>,
    host_irq: Input<'a, AnyPin>,
}

impl<'a> Nrf70<'a> {
    pub async fn run(&mut self) {
        // Power on
        Timer::after(Duration::from_millis(10)).await;
        self.bucken.set_high();
        Timer::after(Duration::from_millis(10)).await;
        self.iovdd_ctl.set_high();
        Timer::after(Duration::from_millis(10)).await;

        // Wakeup
        self.rpu_wakeup().await;

        info!("status reg 0: {:?}", self.read_sr0().await);
        info!("status reg 1: {:?}", self.read_sr1().await);
        info!("status reg 2: {:?}", self.read_sr2().await);

        // buf[0] = 0x42424242;
        // q.write(0, slice8_mut(&mut buf)).await.unwrap();
        //
        // q.read(0, slice8_mut(&mut buf)).await.unwrap();
        // info!("value: {:08x}", buf[0]);
    }

    async fn rpu_wait_until_write_done(&mut self) {
        while self.read_sr0().await & SR0_WRITE_IN_PROGRESS != 0 {}
    }

    async fn rpu_wait_until_awake(&mut self) {
        for _ in 0..10 {
            if self.read_sr1().await & SR1_RPU_AWAKE != 0 {
                return;
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        panic!("awakening never came")
    }

    async fn rpu_wait_until_ready(&mut self) {
        for _ in 0..10 {
            if self.read_sr1().await == SR1_RPU_AWAKE | SR1_RPU_READY {
                return;
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        panic!("readyning never came")
    }

    async fn rpu_wait_until_wakeup_req(&mut self) {
        for _ in 0..10 {
            if self.read_sr2().await == SR2_RPU_WAKEUP_REQ {
                return;
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        panic!("wakeup_req never came")
    }

    async fn rpu_wakeup(&mut self) {
        self.write_sr2(SR2_RPU_WAKEUP_REQ).await;
        self.rpu_wait_until_wakeup_req().await;
        self.rpu_wait_until_awake().await;
    }

    async fn rpu_sleep(&mut self) {
        self.write_sr2(0).await;
    }

    async fn rpu_sleep_status(&mut self) -> u8 {
        self.read_sr1().await
    }

    async fn rpu_read_word(&mut self, mem: &MemoryRegion, offs: u32) -> u32 {
        assert!(mem.start + offs + 4 <= mem.end);
        let lat = mem.latency as usize;

        let mut buf = [0u32; 4];
        self.qspi
            .read(mem.start + offs, slice8_mut(&mut buf[..lat + 1]))
            .await
            .unwrap();

        buf[lat]
    }

    async fn rpu_read(&mut self, mem: &MemoryRegion, offs: u32, buf: &mut [u32]) {
        assert!(mem.start + offs + (buf.len() as u32 * 4) <= mem.end);

        if mem.latency == 0 {
            // No latency, we can do a big read directly.
            self.qspi.read(mem.start + offs, slice8_mut(buf)).await.unwrap();
        } else {
            // Otherwise, read word by word.
            for (i, val) in buf.iter_mut().enumerate() {
                *val = self.rpu_read_word(mem, offs + i as u32 * 4).await;
            }
        }
    }

    async fn rpu_write(&mut self, mem: &MemoryRegion, offs: u32, buf: &[u32]) {
        assert!(mem.start + offs + (buf.len() as u32 * 4) <= mem.end);
        self.qspi.write(mem.start + offs, slice8(buf)).await.unwrap();
    }

    async fn read_sr0(&mut self) -> u8 {
        let mut status = [4; 1];
        unwrap!(self.qspi.custom_instruction(0x05, &[], &mut status).await);
        trace!("read sr0 = {:02x}", status[0]);
        status[0]
    }

    async fn read_sr1(&mut self) -> u8 {
        let mut status = [4; 1];
        unwrap!(self.qspi.custom_instruction(0x1f, &[], &mut status).await);
        trace!("read sr1 = {:02x}", status[0]);
        status[0]
    }

    async fn read_sr2(&mut self) -> u8 {
        let mut status = [4; 1];
        unwrap!(self.qspi.custom_instruction(0x2f, &[], &mut status).await);
        trace!("read sr2 = {:02x}", status[0]);
        status[0]
    }

    async fn write_sr2(&mut self, val: u8) {
        trace!("write sr2 = {:02x}", val);
        unwrap!(self.qspi.custom_instruction(0x3f, &[val], &mut []).await);
    }
}
