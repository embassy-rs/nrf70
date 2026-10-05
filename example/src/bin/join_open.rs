//! Joins an open (unencrypted) Wi-Fi network, gets an address over DHCP, answers ping, and runs a
//! TCP echo server on port 1234. It joins again whenever the link drops.
//!
//! The network is `WIFI_SSID` at build time:
//!
//! ```text
//! WIFI_SSID=MyOpenNetwork cargo run --release --bin join_open
//! ```
//!
//! then, from a machine on the same network, `ping <address>` and `nc <address> 1234`.
//!
//! The driver's runner and embassy-net run as tasks of their own. Joined into one task, every bus
//! wake-up would poll the whole network stack too, which costs throughput.

#![no_std]
#![no_main]
#![deny(unused_must_use)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_net::iface::Iface;
use embassy_net::tcp::{TcpListener, TcpSocket};
use embassy_net::{Stack, StackStorage};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::{bind_interrupts, mode, peripherals};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_io_async::Write;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SERIAL0 => spim::InterruptHandler<peripherals::SERIAL0>;
});

/// The open network to join.
const WIFI_SSID: &str = match option_env!("WIFI_SSID") {
    Some(ssid) => ssid,
    None => "MyOpenNetwork",
};

/// TCP port of the echo server.
const ECHO_PORT: u16 = 1234;

/// The nRF7002-DK's TX power limits, from the nRF Connect SDK's devicetree for it
/// (`wifi-max-tx-pwr-*`), and the world regulatory domain.
const WIFI_CONFIG: nrf70::Config = nrf70::Config::new(nrf70::TxPowerCeiling {
    dsss_2g: 21,
    mcs0_2g: 16,
    mcs7_2g: 16,
    mcs0_5g_low: 9,
    mcs7_5g_low: 9,
    mcs0_5g_mid: 11,
    mcs7_5g_mid: 11,
    mcs0_5g_high: 13,
    mcs7_5g_high: 13,
});

type Bus = nrf70::SpiBus<ExclusiveDevice<Spim<'static, mode::Async>, Output<'static>, Delay>>;

#[embassy_executor::task]
async fn wifi_task(mut runner: nrf70::Runner<'static, Bus, Input<'static>, Output<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static>) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    info!("joining \"{=str}\"", WIFI_SSID);

    // BTRF_SWITCH: low selects separate antennas for the nRF5340 and the nRF7002, as the nRF
    // Connect SDK sets it.
    let _btrf_switch = Output::new(p.P1_10, Level::Low, OutputDrive::Standard);
    let bucken = Output::new(p.P0_12, Level::Low, OutputDrive::HighDrive);
    let iovdd_ctl = Output::new(p.P0_31, Level::Low, OutputDrive::Standard);
    let host_irq = Input::new(p.P0_23, Pull::None);

    // The nRF7002-DK's QSPI pins used as SPI: SCK P0.17, CS P0.18, MOSI DIO0 P0.13, MISO DIO1 P0.14.
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M8;
    let spim = Spim::new(p.SERIAL0, p.P0_17, p.P0_13, p.P0_14, Irqs, config);
    let csn = Output::new(p.P0_18, Level::High, OutputDrive::HighDrive);
    let bus = nrf70::SpiBus::new(unwrap!(ExclusiveDevice::new(spim, csn, Delay)));

    static STATE: StaticCell<nrf70::State> = StaticCell::new();
    let (device, mut control, runner) = nrf70::new(
        STATE.init(nrf70::State::new()),
        bus,
        bucken,
        iovdd_ctl,
        host_irq,
        WIFI_CONFIG,
    );
    spawner.spawn(unwrap!(wifi_task(runner)));
    // The chip starts off. Turned on before the network stack is made, it gives the stack its MAC
    // address from the start.
    unwrap!(control.power_on().await);

    // The CryptoCell's random number generator, the `embassy-crypto` driver of this example.
    let mut seed = [0; 8];
    embassy_crypto::rng_fill_bytes(&mut seed);
    static STACK: StaticCell<StackStorage> = StaticCell::new();
    let (stack, net_runner) = Stack::new(STACK.init(StackStorage::new()), u64::from_le_bytes(seed));
    static DEVICE: StaticCell<nrf70::NetDriver<'static>> = StaticCell::new();
    let iface = unwrap!(stack.add_iface_borrowed(DEVICE.init(device)));
    unwrap!(iface.set_dhcpv4(Some(Default::default())));
    spawner.spawn(unwrap!(net_task(net_runner)));

    loop {
        if let Err(error) = control.join_open(WIFI_SSID.as_bytes()).await {
            warn!("joining failed: {}", error);
            if let nrf70::ConnectError::PoweredOff | nrf70::ConnectError::Fault(_) = error {
                // The chip failed, and is off: start it again.
                if let Err(error) = control.power_on().await {
                    warn!("the chip did not start: {}", error);
                }
            }
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }
        info!("connected");
        if let Some(link) = control.link_status().await {
            info!("link: {}", link);
        }
        // join_open returns when the driver has the link up; embassy-net sees it on its next
        // poll.
        iface.wait_link_up().await;
        if let Either::Second(()) = select(echo_server(stack, iface), iface.wait_link_down()).await {
            warn!("link down, joining again");
        }
    }
}

/// Waits for an address, then echoes what each TCP client sends, one client at a time.
async fn echo_server(stack: Stack<'_>, iface: Iface<'_>) {
    iface.wait_config_v4_up().await;
    for addr in iface.ip_addrs() {
        info!("address {}, echo server on TCP port {}", addr.cidr, ECHO_PORT);
    }
    let mut listener = unwrap!(TcpListener::new(stack));
    unwrap!(listener.listen(ECHO_PORT));

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 1024];
    loop {
        let token = match listener.accept().await {
            Ok(token) => token,
            Err(error) => {
                warn!("accept failed: {}", error);
                continue;
            }
        };
        let mut socket = unwrap!(TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer));
        socket.set_timeout(Some(Duration::from_secs(30)));
        if let Err(error) = socket.accept(token).await {
            warn!("accept failed: {}", error);
            continue;
        }
        info!("client {} connected", socket.remote_addr());
        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => break,
                Ok(n) => n,
                Err(error) => {
                    warn!("read failed: {}", error);
                    break;
                }
            };
            if let Err(error) = socket.write_all(&buf[..n]).await {
                warn!("write failed: {}", error);
                break;
            }
        }
        info!("client disconnected");
        socket.close();
        let _ = socket.flush().await;
    }
}
