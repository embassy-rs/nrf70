#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::slice;

use defmt::{info, *};
use defmt_rtt as _; // global logger
use embassy_executor::Spawner;
use embassy_nrf::config::Config;
use embassy_nrf::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull};
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

    // Config for the MX25R64 present in the nRF52840 DK
    let mut config = qspi::Config::default();
    config.read_opcode = qspi::ReadOpcode::READ4IO;
    config.write_opcode = qspi::WriteOpcode::PP4IO;
    config.write_page_size = qspi::WritePageSize::_256BYTES;

    let irq = interrupt::take!(QSPI);
    let mut q: qspi::Qspi<_> = qspi::Qspi::new(
        p.QSPI, irq, p.P0_17, p.P0_18, p.P0_13, p.P0_14, p.P0_15, p.P0_16, config,
    );

    let mut bucken = Output::new(p.P0_12, Level::Low, OutputDrive::HighDrive);
    let mut iovdd_control = Output::new(p.P0_31, Level::Low, OutputDrive::Standard);
    let host_irq = Input::new(p.P0_23, Pull::None);
    //let coex_req = Output::new(p.P0_28, Level::High, OutputDrive::Standard);
    //let coex_status0 = Output::new(p.P0_30, Level::High, OutputDrive::Standard);
    //let coex_status1 = Output::new(p.P0_29, Level::High, OutputDrive::Standard);
    //let coex_grant = Output::new(p.P0_24, Level::High, OutputDrive::Standard);

    Timer::after(Duration::from_millis(10)).await;
    bucken.set_high();
    Timer::after(Duration::from_millis(10)).await;
    iovdd_control.set_high();
    Timer::after(Duration::from_millis(10)).await;

    // Read status register
    let mut status = [4; 1];
    unwrap!(q.custom_instruction(0x05, &[], &mut status).await);
    info!("status reg 0: {:?}", status[0]);
    unwrap!(q.custom_instruction(0x1f, &[], &mut status).await);
    info!("status reg 1: {:?}", status[0]);
    unwrap!(q.custom_instruction(0x2f, &[], &mut status).await);
    info!("status reg 2: {:?}", status[0]);

    let mut buf = [0u32; 1];
    q.read(0, slice8_mut(&mut buf)).await.unwrap();
    info!("value: {:08x}", buf[0]);

    // buf[0] = 0x42424242;
    // q.write(0, slice8_mut(&mut buf)).await.unwrap();
    //
    // q.read(0, slice8_mut(&mut buf)).await.unwrap();
    // info!("value: {:08x}", buf[0]);
}

fn slice8_mut(x: &mut [u32]) -> &mut [u8] {
    let len = x.len() * 4;
    unsafe { slice::from_raw_parts_mut(x.as_mut_ptr() as _, len) }
}
