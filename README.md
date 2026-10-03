# nrf70

Rust driver for the Nordic nRF7002 Wi-Fi 6 companion chip. Implementation based on Nordic's host
driver, [zephyrproject-rtos/nrf_wifi](https://github.com/zephyrproject-rtos/nrf_wifi), as the nRF
Connect SDK uses it.

Works on the following boards:

- nRF7002-DK (PCA10143), with the nRF5340 application core as the host.
- Any board with the nRF7002 wired for SPI, and possibly the 2.4 GHz only nRF7001, which runs the
  same firmware (untested).

## Features

Working:

- Boots the nRF7002 with the nRF Connect SDK 3.4.0 firmware (1.2.14.9), bundled in `fw/`.
- RF parameters as the SDK computes them: the defaults for the chip's package, the crystal
  calibration from OTP, and the board's TX power ceilings, given in `Config`.
- The MAC address from OTP.
- Station interface up, and scanning: active scans of every 2.4 GHz and 5 GHz channel, with SSID,
  BSSID, band, channel, signal strength and security for each network found.
- Joining open networks with `Control::join_open`, as the nRF Connect SDK's supplicant does it: a
  scan for the SSID, open system authentication with its strongest access point, association and
  port authorisation. If that access point refuses the station or does not answer (a dual band
  one may refuse on one band to steer it to the other), the next best one of the same scan is
  tried, up to four. `ConnectError` says which step failed. The driver reports a lost connection
  as link down, and the application joins again.
  The driver remembers the channels where that scan found the network (up to four, the best
  access points' first), and the next join of the same network scans those only: 0.1 s instead
  of 4.6 s on an nRF7002-DK. If the network is no longer there, the scan of every channel
  follows, and a join that fails forgets the channels.
- Joining WPA2-Personal networks with `Control::join_wpa2`, with the `wpa2`
  [cargo feature](#cargo-features). The nRF70 firmware has no supplicant, so the driver runs the
  4-way handshake and the group key handshake itself, and hands the keys to the chip, which does
  the encryption (CCMP), so throughput is that of an open network. The handshake's HMAC-SHA1,
  HMAC-SHA256, AES-128, AES-128-CMAC and random numbers come from the application, as
  `embassy-crypto` drivers: the microcontroller's accelerator or software.
  `wpa2_psk` derives the key from the passphrase once, for `Control::join_wpa2_psk`: 0.7 s on an
  nRF5340 at 64 MHz with its CryptoCell, 2.2 s in software built for size. A wrong passphrase
  shows as `ConnectError::HandshakeFailed` after about 3 s. The group key handshake was checked
  on an nRF7002-DK too, against an access point that changes its group key every 30 s.
  Management frame protection (IEEE 802.11w) is used when the access point offers it, required
  or not, with BIP-CMAC-128: the driver then takes PSK-SHA256 key management if the access point
  offers it (AES-128-CMAC MICs, keys derived with SHA-256), and hands the chip the management
  group key (IGTK) too. The handshakes are checked against vectors computed with OpenSSL.
  WPA/WPA2 mixed mode networks, whose group key is TKIP's, are joined too, with CCMP for the
  station's own traffic. The chip does not check the Michael MIC of the group frames it receives
  (on an nRF7002-DK, broadcasts kept arriving with the Michael keys in either order) and reports
  no MIC failure, so there are no TKIP countermeasures: broadcasts on such a network are
  encrypted, but their integrity rests on TKIP's CRC alone. Checked on an nRF7002-DK against
  hostapd, with group key renewals every 30 s.
- Joining WPA3-Personal networks with `Control::join_wpa3`, with the `wpa3` cargo feature: SAE on
  the P-256 curve (group 19), then the 4-way handshake, with management frame protection, on
  WPA3 and WPA2/WPA3 transition networks. The chip sends the SAE messages as authentication
  frames; the driver computes them. It uses hash to element where the access point announces it
  in its RSNXE (22 ms on an nRF5340 at 128 MHz, once per join), and hunting and pecking
  elsewhere (0.39 s, for each access point). A wrong password shows as
  `ConnectError::HandshakeFailed` once the access point refuses the confirm. SAE is checked
  against the test vectors of IEEE 802.11-2020, J.10, and the handshake after it against vectors
  computed with OpenSSL. Both PWE derivations were checked on an nRF7002-DK against hostapd, with
  group key renewals every 30 s.
- Access point mode with `Control::start_ap_open`, with the `ap` cargo feature: an open network on
  a 2.4 GHz channel (1 to 13) or a 5 GHz one that needs no radar detection (36 to 48, 149 to
  177), for up to four stations. The firmware sends the beacons; the driver answers probe,
  authentication and association requests and adds the stations to the chip, as hostapd does in
  the nRF Connect SDK. It keeps the frames for a station that dozes (802.11 power save) and hands
  them over when the station wakes up; while they fill its four places it takes no more frames
  from embassy-net, so TCP slows down instead of losing segments. `Control::stop_ap` ends it.
  On an nRF7002-DK with a laptop as station: 9.7 Mbit/s up and 12.2 Mbit/s down on channel 36,
  as in station mode, with the laptop's power save on or off. On 2.4 GHz that laptop announced
  itself away 80 ms out of every 100 even with its power save off (its Bluetooth coexistence,
  most likely), and got 3 Mbit/s.
  With the `wpa2` feature too, `Control::start_ap_wpa2` starts a WPA2-Personal access point (PSK,
  CCMP): the driver runs the access point's side of the 4-way handshake with each station,
  sending its messages again every second up to four times, and gives the chip the group key and
  each station's pairwise key. A laptop joins it 0.1 s after its association, at the same
  throughput as on the open one; with a wrong passphrase it is let go after 4 s. The handshake is
  checked against the driver's own supplicant.
  The chip sends group frames right away, and a dozing station misses them: 13 broadcasts out of
  30 reached a dozing laptop. While a station sleeps, the driver sends each group frame to every
  station with keys as a unicast frame instead (hostapd's `multicast_to_unicast`), and the
  laptop got 30 out of 30.
- The state of the link with `Control::link_status`: the access point, its channel, the signal
  strength and the rates in use, as the chip reports them.
- 802.11 power save with `Control::set_power_save`, off until asked for: the chip sleeps between
  the access point's beacons and fetches what was kept for it. Traffic towards an idle station
  then waits for a beacon (a ping to an nRF7002-DK is answered in 50 to 130 ms on average instead
  of 4 ms), and throughput is unchanged, as the chip stays awake while frames flow.
  `Control::power_save` reads the setting back.
- Low power mode with `Config::low_power`, as in the nRF Connect SDK: the chip may sleep whenever
  it has nothing to do, and the driver wakes it up before each bus access and leaves it alone
  once the bus has been idle for 10 ms. Together with power save, an idle nRF7002-DK on a 5 GHz
  network sleeps 93 to 96% of the time (read from the chip's status register every 7 ms), and
  the driver does not touch the bus between two interrupts. A wake-up from sleep takes 7 ms, and
  throughput stays within 5% of the normal mode's.
- Turning the chip off with `Control::power_off` (its shutdown state, after leaving the network)
  and on again with `Control::power_on`, which loads the firmware and brings the interface up in
  0.1 to 0.2 s on an nRF7002-DK, and restores the power save setting. The network is then joined
  again, on the channels remembered from before.
- Ethernet frames to and from [`embassy-net`](https://embassy.dev) through the `NetDriver`: TCP,
  UDP, DHCP and ICMP work on top of it.
- SPI bus through any [`embedded-hal-async`](https://crates.io/crates/embedded-hal-async)
  `SpiDevice`.
- Any other bus through the `Bus` trait. The `scan_qspi` example implements it on the nRF5340's
  QSPI peripheral in quad mode, as the nRF Connect SDK drives the nRF7002-DK: it loads the
  firmware in 19 ms, against 103 ms over SPI at 8 MHz.
- Using the host IRQ for device events, with a slow poll as a fallback (not in low power mode,
  where it would wake the chip up).

Not yet:

- Beyond WPA2-Personal and WPA3-Personal with CCMP (see above): TKIP as pairwise cipher (WPA
  networks without WPA2), SAE on other groups than 19 and SAE-EXT-KEY, SAE password identifiers,
  and PMK caching. An access point that requires one of them is reported as
  `ConnectError::SecurityMismatch`.
- Target wake time, and the current drawn in power save and low power mode, which is not
  measured yet.
- In AP mode: WPA3, management frame protection and group key renewals, 802.11n and WMM (the
  access point announces the 802.11a/b/g rates only), and letting go of stations that vanish
  without a word (they keep their place until they come back). Delivery to sleeping stations was
  checked with a laptop that wakes up when a beacon says frames wait for it; PS-Poll and U-APSD
  are not tested.

## Cargo features

- `wpa2` (off by default): `Control::join_wpa2`, `Control::join_wpa2_psk` and `wpa2_psk`. Without
  it the driver joins open networks only.
- `wpa3` (off by default, implies `wpa2`): `Control::join_wpa3`. SAE runs on the P-256
  arithmetic of the `p256` crate, in software, and its hashes on `embassy-crypto`'s HMAC-SHA256.
- `ap` (off by default): `Control::start_ap_open` and `Control::stop_ap`, and with `wpa2`
  `Control::start_ap_wpa2`. It adds about 15 KB of flash and 9 KB of RAM on an nRF5340, 6 KB of
  which in `State` for the frames kept for stations that sleep; with `wpa2`, 23 KB and 11 KB.

The handshakes of WPA2 need HMAC-SHA1 (for the key derivation from the passphrase, the pairwise
key and the frames' integrity codes), the AES-128 block cipher (to unwrap the group keys) and
random numbers (for the nonces), and with management frame protection HMAC-SHA256 and
AES-128-CMAC (the keys and integrity codes of PSK-SHA256). The driver has none of them: it calls
[`embassy-crypto`](https://github.com/embassy-rs/embassy/tree/main/embassy-crypto), and the
application says in its own `Cargo.toml` who answers, with one feature per operation. Without a
driver for each of the five, the application does not link.

The microcontroller's accelerator, where the HAL has `embassy-crypto` drivers: here the
CryptoCell of an nRF5340, nRF52840 or nRF91, or the CRACEN of an nRF54L.

```toml
nrf70 = { version = "0.2", features = ["wpa2"] }
embassy-nrf = { version = "...", features = ["embassy-crypto-aes128-cmac", "embassy-crypto-aes128-ecb", "embassy-crypto-hmac-sha1", "embassy-crypto-hmac-sha256", "embassy-crypto-rng"] }
```

`embassy-crypto-rng` takes the generator over: the application then gets its own random numbers
from `embassy_crypto::rng_fill_bytes` too.

Or HMAC-SHA1 and AES-128 in software, from the RustCrypto crates, on any microcontroller:

```toml
nrf70 = { version = "0.2", features = ["wpa2"] }
embassy-crypto-rustcrypto = { version = "0.1", features = ["embassy-crypto-aes128-cmac", "embassy-crypto-aes128-ecb", "embassy-crypto-hmac-sha1", "embassy-crypto-hmac-sha256"] }
```

with `use embassy_crypto_rustcrypto as _;` in the application, so that the crate is linked. The
random numbers still have to come from hardware: the HAL's `embassy-crypto-rng`, or a generator
of the application's own registered with `embassy_crypto::rng_impl!`.

On an nRF5340 at 64 MHz, built for size, with the `join_wpa2` example:

| | CryptoCell | Software |
| --- | --- | --- |
| Flash, more than without `wpa2` | 17 KB | 29 KB |
| RAM | 2 KB | 2 KB |
| Key from the passphrase (`wpa2_psk`) | 0.7 s | 2.2 s |

## Running the example

- Install `probe-rs` following the instructions at <https://probe.rs>.
- On the nRF7002-DK, fit the P22 (nRF7002 VDD) and P23 (VBAT) jumper caps, or the chip does not
  answer.
- `cd example`
- `cargo run --release` (over SPI at 8 MHz), or `cargo run --release --bin scan_qspi` (over QSPI at
  24 MHz)

The examples scan every 10 s and log what they find:

```
0.161621 [INFO ] firmware booted: UMAC 1.2.14.9, LMAC 1.1.7.0
0.187103 [INFO ] ======== INIT DONE!! ==========
4.887054 [INFO ] [10, 13, 31, 6a, 3a, 15] Band2_4GHz ch 11 rssi Some(-72) Wpa2 MyNetwork
4.887420 [INFO ] [12, 13, 31, 6a, 3a, 1d] Band5GHz ch 112 rssi Some(-89) Wpa2 MyNetwork-5G
4.887786 [INFO ] scan done
```

`join_open` joins an open network, gets an address over DHCP, answers ping and runs a TCP echo
server on port 1234:

```
WIFI_SSID=MyOpenNetwork cargo run --release --bin join_open
```

```
5.350860 [INFO ] connected
5.604736 [INFO ] address 10.42.0.65/24, echo server on TCP port 1234
```

Then `ping 10.42.0.65` and `nc 10.42.0.65 1234` from the same network.

`join_wpa2` does the same on a WPA2-Personal network. It needs the example's `wpa2` feature, which
turns on the driver's and makes the nRF5340's CryptoCell the `embassy-crypto` driver of all it
needs:

```
WIFI_SSID=MyNetwork WIFI_PASSPHRASE=MyPassphrase cargo run --release --features wpa2 --bin join_wpa2
```

```
1.000732 [INFO ] pre-shared key derived in 732 ms
6.119781 [INFO ] connected
6.369903 [INFO ] address 10.42.0.65/24, echo server on TCP port 1234
```

If probe-rs reports the core as locked, the DK's application core has APPROTECT enabled: add
`--allow-erase-all` to the runner in `example/.cargo/config.toml`.

## Firmware and bindings

`fw/nrf70.bin` is the SDK's firmware file (`nrf_wifi/bin/zephyr/default/nrf70.bin` in
[sdk-nrfxlib](https://github.com/nrfconnect/sdk-nrfxlib)), and `fw/bindings.rs` is generated from
the `nrf_wifi` headers of the same SDK release. The two must match: the driver checks the firmware
version against the bindings at boot. To move to another release, with `bindgen`, `python3` and
`rustfmt` installed:

```
./gen.py <nrf_wifi checkout at the SDK's revision> <that release's nrf70.bin>
```

## Interoperability

This crate can run on any executor, with an [`embassy-time`](https://crates.io/crates/embassy-time)
time driver.

## License

This work is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

The firmware in `fw/nrf70.bin` is Nordic's, under the Nordic 5-Clause license in
[fw/LICENSE](fw/LICENSE): it may only be used with a Nordic Semiconductor integrated circuit.
`fw/bindings.rs` is generated from the `nrf_wifi` headers, which are BSD-3-Clause.
