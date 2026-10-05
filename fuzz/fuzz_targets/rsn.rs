#![no_main]

libfuzzer_sys::fuzz_target!(|data: &[u8]| nrf70::fuzz::rsn(data));
