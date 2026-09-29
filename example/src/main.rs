#![no_std]
#![no_main]
#![deny(unused_must_use)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::spim::Spim;
use embassy_nrf::{bind_interrupts, peripherals, spim, Peri};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use nrf70::SpiBus;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SERIAL0 => spim::InterruptHandler<embassy_nrf::peripherals::SERIAL0>;
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
    let config: embassy_nrf::config::Config = Default::default();
    let p = embassy_nrf::init(config);
    spawner.spawn(unwrap!(blink_task(p.P1_06)));

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
    // BTRF_SWITCH: low selects separate antennas for the nRF5340 and the nRF7002, as the nRF
    // Connect SDK sets it.
    let _btrf_switch = Output::new(p.P1_10, Level::Low, OutputDrive::Standard);
    let bucken = Output::new(p.P0_12, Level::Low, OutputDrive::HighDrive);
    let iovdd_ctl = Output::new(p.P0_31, Level::Low, OutputDrive::Standard);
    let host_irq = Input::new(p.P0_23, Pull::None);

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M8;
    let spim = Spim::new(p.SERIAL0, sck, dio0, dio1, Irqs, config);
    let csn = Output::new(csn, Level::High, OutputDrive::HighDrive);
    let spi = unwrap!(ExclusiveDevice::new(spim, csn, Delay));
    // `src/bin/scan_qspi.rs` drives the same pins with the QSPI peripheral instead.
    let bus = SpiBus::new(spi);

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
