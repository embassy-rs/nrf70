//! Scans for Wi-Fi networks like the main example, with the nRF7002 on the nRF5340's QSPI
//! peripheral in quad mode instead of SPI.
//!
//! The nRF7002-DK wires the nRF7002 to the nRF5340's dedicated QSPI pins (P0.13 to P0.18). The bus
//! is set up as the nRF Connect SDK 3.4.0 sets it for this board (`qspi_if.c` in its nRF70 driver,
//! the board's devicetree, and nrfx underneath):
//!
//! - READ4IO (0xEB) and PP4IO (0x38), 24-bit addresses, SPI mode 0, SCKDELAY 0, 24 MHz.
//! - IFTIMING bits 7:4 set to 0xA: ten dummy cycles for READ4IO (`RDC4IO` in the SDK). The field
//!   is not in the nRF5340 Product Specification.
//! - The status registers through custom instructions without write enable or write-in-progress
//!   polling, which embassy-nrf's `Qspi::custom_instruction` always adds.
//! - The wake-up (WRSR2) at 8 MHz, as the SDK sends it; the bus switches to its own frequency once
//!   RDSR1 reports the RPU awake.
//! - Writes set address bit 23, which makes the RPU increment the address (the SDK's `addrmask`).
//!
//! It also works around three nRF5340 anomalies, as nrfx does:
//!
//! - 43: writing IFCONFIG1, IFTIMING or CINSTRCONF needs an ACTIVATE with the pins disconnected
//!   first.
//! - 121 (all but the first engineering revision): IFCONFIG0 bit 16 set and RXDELAY 6.
//! - 159 (later revisions): transfers need HFCLK192M undivided, with the CPU at 64 MHz. The bus sets
//!   the divider to 1 for as long as it lives (its reset value, 4, is only allowed while the
//!   peripheral is idle anyway).
//!
//! The PAC accesses need embassy-nrf's `unstable-pac` feature.

#![no_std]
#![no_main]
#![deny(unused_must_use)]

use core::slice;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::pac::clock::vals::Hclk192m;
use embassy_nrf::pac::qspi::vals::Length;
use embassy_nrf::pac::shared::regs::Psel;
use embassy_nrf::qspi::{self, AddressMode, Qspi, ReadOpcode, SpiMode, WriteOpcode, WritePageSize};
use embassy_nrf::{bind_interrupts, interrupt, pac, peripherals, Peri};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    QSPI => qspi::InterruptHandler<peripherals::QSPI>;
});

/// The nRF7002-DK's TX power limits, from the nRF Connect SDK's devicetree for it
/// (`wifi-max-tx-pwr-*`), and the world regulatory domain.
const WIFI_CONFIG: nrf70::Config = nrf70::Config {
    max_tx_power: nrf70::TxPowerCeiling {
        dsss_2g: 21,
        mcs0_2g: 16,
        mcs7_2g: 16,
        mcs0_5g_low: 9,
        mcs7_5g_low: 9,
        mcs0_5g_mid: 11,
        mcs7_5g_mid: 11,
        mcs0_5g_high: 13,
        mcs7_5g_high: 13,
    },
    country_code: *b"00",
    low_power: false,
};

/// SCK = 96 MHz / (SCKFREQ + 1) with HFCLK192M undivided: 3 gives 24 MHz, the SDK's frequency for
/// this board, and 2 gives 32 MHz, the nRF7002's maximum.
const SCKFREQ: u8 = 3;

#[embassy_executor::task]
async fn blink_task(led: Peri<'static, peripherals::P1_06>) -> ! {
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
    let p = embassy_nrf::init(Default::default());
    spawner.spawn(unwrap!(blink_task(p.P1_06)));

    // BTRF_SWITCH: low selects separate antennas for the nRF5340 and the nRF7002, as the nRF
    // Connect SDK sets it.
    let _btrf_switch = Output::new(p.P1_10, Level::Low, OutputDrive::Standard);
    let bucken = Output::new(p.P0_12, Level::Low, OutputDrive::HighDrive);
    let iovdd_ctl = Output::new(p.P0_31, Level::Low, OutputDrive::Standard);
    let host_irq = Input::new(p.P0_23, Pull::None);

    let bus = QspiBus::new(
        p.QSPI, Irqs, p.P0_17, p.P0_18, p.P0_13, p.P0_14, p.P0_15, p.P0_16, SCKFREQ,
    );

    let mut state = nrf70::State::new();
    let (_device, mut control, mut runner) =
        nrf70::new(&mut state, bus, bucken, iovdd_ctl, host_irq, WIFI_CONFIG).await;

    let scan = async {
        loop {
            let mut scanner = control.scan().await;
            while let Some(bss) = scanner.next().await {
                let ssid = core::str::from_utf8(bss.ssid()).unwrap_or("<not UTF-8>");
                info!(
                    "{:02x} {} ch {} rssi {} {} {=str}",
                    bss.bssid, bss.band, bss.channel, bss.rssi, bss.security, ssid
                );
            }
            info!("scan done");
            Timer::after(Duration::from_secs(10)).await;
        }
    };

    join(runner.run(), scan).await;
}

/// Write address bit that makes the RPU increment the address (the SDK's `addrmask`).
const ADDR_INCREMENT: u32 = 0x80_0000;

const RDSR0: u8 = 0x05;
const RDSR1: u8 = 0x1F;
const RDSR2: u8 = 0x2F;
const WRSR2: u8 = 0x3F;
const SR1_RPU_AWAKE: u8 = 0x02;

/// SCKFREQ for the 8 MHz of the wake-up.
const SCKFREQ_WAKE: u8 = 11;

/// IFTIMING bits 7:4: ten dummy cycles for READ4IO.
const RDC4IO: u32 = 0xA0;

/// Transfers up to this size wait for the peripheral by polling: an interrupt and a task wake-up
/// cost more than such a transfer takes.
const BLOCKING_MAX: usize = 64;

/// Words of the buffer that copies writes whose data is not in RAM (EasyDMA only reads RAM).
const BOUNCE_WORDS: usize = 64;

/// The application core's RAM, which EasyDMA can reach.
const RAM: core::ops::Range<usize> = 0x2000_0000..0x2008_0000;

/// The nRF7002 on the QSPI peripheral, as an [`nrf70::Bus`].
struct QspiBus<'d> {
    qspi: Qspi<'d>,
    sckfreq: u8,
    /// Set from the wake-up until RDSR1 reports the RPU awake.
    waking: bool,
    bounce: [u32; BOUNCE_WORDS],
}

impl<'d> QspiBus<'d> {
    #[allow(clippy::too_many_arguments)]
    fn new(
        qspi: Peri<'d, peripherals::QSPI>,
        irq: impl interrupt::typelevel::Binding<interrupt::typelevel::QSPI, qspi::InterruptHandler<peripherals::QSPI>> + 'd,
        sck: Peri<'d, impl Pin>,
        csn: Peri<'d, impl Pin>,
        io0: Peri<'d, impl Pin>,
        io1: Peri<'d, impl Pin>,
        io2: Peri<'d, impl Pin>,
        io3: Peri<'d, impl Pin>,
        sckfreq: u8,
    ) -> Self {
        // The FICR words nrfx reads to tell which anomalies apply (`nrf53_errata_121`).
        let (part, variant) = unsafe {
            (
                core::ptr::read_volatile(0x00FF_0130 as *const u32),
                core::ptr::read_volatile(0x00FF_0134 as *const u32),
            )
        };
        let anomaly_121 = part == 0x07 && variant != 0x02;

        pac::CLOCK.hfclk192mctrl().write(|w| w.set_hclk192m(Hclk192m::Div1));

        let mut config = qspi::Config::default();
        config.read_opcode = ReadOpcode::Read4io;
        config.write_opcode = WriteOpcode::Pp4io;
        config.write_page_size = WritePageSize::_256bytes;
        config.address_mode = AddressMode::_24bit;
        config.spi_mode = SpiMode::Mode0;
        config.sck_delay = 0;
        config.rx_delay = if anomaly_121 {
            6
        } else {
            pac::QSPI.iftiming().read().rxdelay()
        };
        // The activation runs at the wake-up frequency, SCKFREQ 11.
        config.frequency = qspi::Frequency::M2_7;
        let qspi = Qspi::new(qspi, irq, sck, csn, io0, io1, io2, io3, config);

        let r = pac::QSPI;
        anomaly_43_workaround();
        if anomaly_121 {
            // Bit 17 would be set too at SCKFREQ 0, which this bus does not use.
            r.ifconfig0().modify(|w| w.0 = (w.0 & !(1 << 17)) | (1 << 16));
        }
        r.iftiming().modify(|w| w.0 |= RDC4IO);
        info!(
            "QSPI: SCKFREQ {}, IFCONFIG0 {:08x}, IFTIMING {:08x}",
            sckfreq,
            r.ifconfig0().read().0,
            r.iftiming().read().0
        );
        Self {
            qspi,
            sckfreq,
            waking: false,
            bounce: [0; BOUNCE_WORDS],
        }
    }

    fn set_sckfreq(&mut self, sckfreq: u8) {
        anomaly_43_workaround();
        pac::QSPI.ifconfig1().modify(|w| w.set_sckfreq(sckfreq));
    }

    /// A custom instruction of the opcode and one byte, out or in.
    fn instruction(&mut self, opcode: u8, out: u8) -> u8 {
        let r = pac::QSPI;
        anomaly_43_workaround();
        r.cinstrdat0().write(|w| w.0 = out as u32);
        r.events_ready().write_value(0);
        r.cinstrconf().write(|w| {
            w.set_opcode(opcode);
            w.set_length(Length::_2b);
            w.set_lio2(true);
            w.set_lio3(true);
            w.set_wipwait(false);
            w.set_wren(false);
            w.set_lfen(false);
        });
        while r.events_ready().read() == 0 {}
        r.cinstrdat0().read().0 as u8
    }

    fn read_status(&mut self, opcode: u8) -> u8 {
        let val = self.instruction(opcode, 0);
        trace!("read sr {:02x} = {:02x}", opcode, val);
        val
    }
}

/// Anomaly 43: an ACTIVATE with the pins disconnected, so that it does not wait on the device,
/// before writing IFCONFIG1, IFTIMING or CINSTRCONF (nrfx `qspi_workaround_215_43_apply`).
fn anomaly_43_workaround() {
    let r = pac::QSPI;
    let psel = r.psel();
    let regs = [psel.sck(), psel.csn(), psel.io0(), psel.io1(), psel.io2(), psel.io3()];
    let pins = regs.map(|reg| reg.read());
    for reg in regs {
        reg.write_value(Psel(0xFFFF_FFFF));
    }
    r.events_ready().write_value(0);
    r.tasks_activate().write_value(1);
    while r.events_ready().read() == 0 {}
    for (reg, pin) in regs.into_iter().zip(pins) {
        reg.write_value(pin);
    }
}

impl nrf70::Bus for QspiBus<'_> {
    async fn read(&mut self, addr: u32, buf: &mut [u32]) {
        let bytes = unsafe { slice::from_raw_parts_mut(buf.as_mut_ptr() as *mut u8, buf.len() * 4) };
        if bytes.len() <= BLOCKING_MAX {
            unwrap!(self.qspi.blocking_read_raw(addr, bytes));
        } else {
            unwrap!(self.qspi.read_raw(addr, bytes).await);
        }
    }

    async fn write(&mut self, addr: u32, buf: &[u32]) {
        let addr = addr | ADDR_INCREMENT;
        if RAM.contains(&(buf.as_ptr() as usize)) {
            let bytes = unsafe { slice::from_raw_parts(buf.as_ptr() as *const u8, buf.len() * 4) };
            if bytes.len() <= BLOCKING_MAX {
                unwrap!(self.qspi.blocking_write_raw(addr, bytes));
            } else {
                unwrap!(self.qspi.write_raw(addr, bytes).await);
            }
            return;
        }
        for (i, chunk) in buf.chunks(BOUNCE_WORDS).enumerate() {
            self.bounce[..chunk.len()].copy_from_slice(chunk);
            let bytes = unsafe { slice::from_raw_parts(self.bounce.as_ptr() as *const u8, chunk.len() * 4) };
            let chunk_addr = addr + (i * BOUNCE_WORDS * 4) as u32;
            unwrap!(self.qspi.write_raw(chunk_addr, bytes).await);
        }
    }

    async fn read_sr0(&mut self) -> u8 {
        self.read_status(RDSR0)
    }

    async fn read_sr1(&mut self) -> u8 {
        let val = self.read_status(RDSR1);
        if self.waking && val & SR1_RPU_AWAKE != 0 {
            self.waking = false;
            self.set_sckfreq(self.sckfreq);
        }
        val
    }

    async fn read_sr2(&mut self) -> u8 {
        self.read_status(RDSR2)
    }

    async fn write_sr2(&mut self, val: u8) {
        trace!("write sr2 = {:02x}", val);
        // The RPU wakes reliably only at 8 MHz (the SDK's `qspi_cmd_wakeup_rpu`).
        if !self.waking {
            self.waking = true;
            self.set_sckfreq(SCKFREQ_WAKE);
        }
        self.instruction(WRSR2, val);
    }
}
