#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![deny(unused_must_use)]
#![feature(async_fn_in_trait)]
#![feature(impl_trait_projections)]

use defmt::*;
use defmt_rtt as _; // global logger
use embassy_executor::Spawner;
use embassy_nrf::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::spim::Spim;
use embassy_nrf::{bind_interrupts, spim};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use nrf70::SpiBus;
use {embassy_nrf as _, panic_probe as _};

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
    let host_irq = Input::new(p.P0_23.degrade(), Pull::None);

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M8;
    let spim = Spim::new(p.SERIAL0, Irqs, sck, dio1, dio0, config);
    let csn = Output::new(csn, Level::High, OutputDrive::HighDrive);
    let spi = ExclusiveDevice::new(spim, csn, Delay);
    let bus = SpiBus::new(spi);

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

    let mut state = nrf70::State::new();
    let (device, control, mut runner) = nrf70::new(&mut state, bus, bucken, iovdd_ctl, host_irq).await;

    runner.run().await;
}
