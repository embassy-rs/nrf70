#![no_main]

// The software HMAC, AES-128 and AES-128-CMAC that the driver runs on here.
use embassy_crypto_rustcrypto as _;

libfuzzer_sys::fuzz_target!(|data: &[u8]| nrf70::fuzz::key_data(data));
