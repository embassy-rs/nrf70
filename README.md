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
- SPI bus through any [`embedded-hal-async`](https://crates.io/crates/embedded-hal-async)
  `SpiDevice`.
- Using the host IRQ for device events, with a slow poll as a fallback.

Not yet:

- Joining networks and sending and receiving Ethernet frames (the `NetDriver` for
  [`embassy-net`](https://embassy.dev) exists, the data path does not).
- WPA2/WPA3 (the firmware has no supplicant: the host runs the 4-way handshake), AP mode, low power
  mode, QSPI.

## Running the example

- Install `probe-rs` following the instructions at <https://probe.rs>.
- On the nRF7002-DK, fit the P22 (nRF7002 VDD) and P23 (VBAT) jumper caps, or the chip does not
  answer.
- `cd example`
- `cargo run --release`

The example scans every 10 s and logs what it finds:

```
0.161621 [INFO ] firmware booted: UMAC 1.2.14.9, LMAC 1.1.7.0
0.187103 [INFO ] ======== INIT DONE!! ==========
4.887054 [INFO ] [10, 13, 31, 6a, 3a, 15] Band2_4GHz ch 11 rssi Some(-72) Wpa2 MyNetwork
4.887420 [INFO ] [12, 13, 31, 6a, 3a, 1d] Band5GHz ch 112 rssi Some(-89) Wpa2 MyNetwork-5G
4.887786 [INFO ] scan done
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
