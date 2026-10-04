//! Turning the chip on and off: the firmware patches and their load, the OTP and the RF parameters
//! it gives, and the UMAC's initialisation (NCS `nrf_wifi_fmac_dev_add` and `nrf_wifi_fmac_dev_init`).

use core::mem::{size_of, zeroed};

use align_data::{include_aligned, Align16};
use defmt::{assert, info, panic, unwrap, warn};
use embassy_time::{Duration, Timer};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

use crate::control::ScanEvent;
use crate::rpu::regions::*;
use crate::rpu::{Processor, MAX_TX_AGGREGATION, RX_BUFS, RX_BUFS_PER_QUEUE, RX_MAX_DATA_SIZE};
use crate::station::ConnState;
use crate::{c, ch, sliceit, Bus, Config, ConnectError, Runner, TxPowerCeiling, EVENT_TIMEOUT};

/// How long a deauthentication frame gets to leave before the chip is turned off.
const POWER_OFF_DELAY: Duration = Duration::from_millis(20);

/// The nRF70 firmware, in the nRF Connect SDK's `nrf70.bin` format: a header, then the UMAC and
/// LMAC patches. `gen.py` takes it from the SDK release that `fw/bindings.rs` comes from.
static FIRMWARE: &[u8] = include_aligned!(Align16, "../fw/nrf70.bin");

/// The four patch images of an `nrf70.bin`.
struct Firmware<'a> {
    umac_pri: &'a [u8],
    umac_sec: &'a [u8],
    lmac_pri: &'a [u8],
    lmac_sec: &'a [u8],
}

/// Splits an `nrf70.bin` into its images, after checking that it is a system (station) mode
/// firmware of the version the bindings come from (NCS `nrf_wifi_fmac_fw_parse`).
fn parse_firmware(fw: &[u8]) -> Firmware<'_> {
    let word = |offset: usize| u32::from_le_bytes(unwrap!(fw[offset..offset + 4].try_into()));

    assert!(word(0) == c::PATCH_SIGNATURE, "not an nRF70 firmware file");
    assert!(word(4) == c::PATCH_NUM_IMAGES, "unexpected number of firmware images");
    let version = word(8);
    let expected = c::RPU_FAMILY << 24 | c::RPU_MAJOR_VERSION << 16 | c::RPU_MINOR_VERSION << 8 | c::RPU_PATCH_VERSION;
    assert!(
        version == expected,
        "firmware version {:08x} does not match the bindings ({:08x})",
        version,
        expected
    );
    let system_mode = c::nrf70_feature_flags::NRF70_FEAT_SYSTEM_MODE as u32;
    assert!(word(12) & system_mode != 0, "not a system mode firmware");

    // Each image is a type and a length, then the data, in `nrf70_image_ids` order. The data is
    // not word aligned: one image has an odd length.
    let mut images = [&fw[..0]; 4];
    let mut offset = size_of::<c::nrf70_fw_image_info>();
    for image in &mut images {
        let len = word(offset + 4) as usize;
        let start = offset + size_of::<c::nrf70_fw_image>();
        *image = &fw[start..start + len];
        offset = start + len;
    }
    let [umac_pri, umac_sec, lmac_pri, lmac_sec] = images;
    Firmware {
        umac_pri,
        umac_sec,
        lmac_pri,
        lmac_sec,
    }
}

// ========= RF parameters

const _: () = core::assert!(size_of::<c::phy_rf_params>() == c::RF_PARAMS_SIZE as usize);

/// What the UMAC reads out of the chip's OTP at boot.
pub(crate) struct Otp {
    pub(crate) info: c::host_rpu_umac_info,
    /// One "not programmed" bit per field.
    pub(crate) flags: u32,
    /// Version of the production test program that wrote the calibration.
    pub(crate) ft_prog_ver: u32,
    /// `QFN_PACKAGE_INFO` or `CSP_PACKAGE_INFO`, or 0xFFFFFFFF if not programmed.
    pub(crate) package_info: u32,
}

/// The value of hex digit `c`.
const fn hex_nibble(c: u8) -> u8 {
    match c {
        b'0'..=b'9' => c - b'0',
        b'A'..=b'F' => c - b'A' + 10,
        b'a'..=b'f' => c - b'a' + 10,
        _ => core::panic!("invalid hex digit"),
    }
}

/// A byte of `phy_rf_params_common.h`, which writes signed values as unsigned hex.
const fn byte(value: u32) -> i8 {
    value as u8 as i8
}

/// Builds the RF parameters as the nRF Connect SDK does (`nrf_wifi_sys_fmac_rf_params_get`): the
/// defaults for the chip's package, the crystal calibration from OTP, and TX power ceilings that
/// are the lower of the board's and the package's, less a backoff for the chip's test program.
fn rf_params(otp: &Otp, board: &TxPowerCeiling) -> c::phy_rf_params {
    let mut p: c::phy_rf_params = unsafe { zeroed() };

    p.pd_adjust_val = c::pd_adst_val {
        pd_adjt_lb_chan: byte(c::PD_ADJUST_VAL),
        pd_adjt_hb_low_chan: byte(c::PD_ADJUST_VAL),
        pd_adjt_hb_mid_chan: byte(c::PD_ADJUST_VAL),
        pd_adjt_hb_high_chan: byte(c::PD_ADJUST_VAL),
    };
    p.rx_gain_offset = c::rx_gain_offset {
        rx_gain_lb_chan: byte(c::CTRL_PWR_OPTIMIZATIONS),
        rx_gain_hb_low_chan: byte(c::RX_GAIN_OFFSET_HB_LOW_CHAN),
        rx_gain_hb_mid_chan: byte(c::RX_GAIN_OFFSET_HB_MID_CHAN),
        rx_gain_hb_high_chan: byte(c::RX_GAIN_OFFSET_HB_HIGH_CHAN),
    };

    // A chip without package information in OTP is taken to be a QFN.
    if otp.package_info == c::CSP_PACKAGE_INFO {
        p.xo_offset.xo_freq_offset = c::CSP_XO_VAL as u8;
        p.syst_tx_pwr_offset = c::tx_pwr_systm_offset {
            syst_off_lb_chan: byte(c::CSP_SYSTEM_OFFSET_LB),
            syst_off_hb_low_chan: byte(c::CSP_SYSTEM_OFFSET_HB_CHAN_LOW),
            syst_off_hb_mid_chan: byte(c::CSP_SYSTEM_OFFSET_HB_CHAN_MID),
            syst_off_hb_high_chan: byte(c::CSP_SYSTEM_OFFSET_HB_CHAN_HIGH),
        };
        p.max_pwr_ceil = c::tx_pwr_ceil {
            max_dsss_pwr: byte(c::CSP_MAX_TX_PWR_DSSS),
            max_lb_mcs7_pwr: byte(c::CSP_MAX_TX_PWR_LB_MCS7),
            max_lb_mcs0_pwr: byte(c::CSP_MAX_TX_PWR_LB_MCS0),
            max_hb_low_chan_mcs7_pwr: byte(c::CSP_MAX_TX_PWR_HB_LOW_CHAN_MCS7),
            max_hb_mid_chan_mcs7_pwr: byte(c::CSP_MAX_TX_PWR_HB_MID_CHAN_MCS7),
            max_hb_high_chan_mcs7_pwr: byte(c::CSP_MAX_TX_PWR_HB_HIGH_CHAN_MCS7),
            max_hb_low_chan_mcs0_pwr: byte(c::CSP_MAX_TX_PWR_HB_LOW_CHAN_MCS0),
            max_hb_mid_chan_mcs0_pwr: byte(c::CSP_MAX_TX_PWR_HB_MID_CHAN_MCS0),
            max_hb_high_chan_mcs0_pwr: byte(c::CSP_MAX_TX_PWR_HB_HIGH_CHAN_MCS0),
        };
        p.temp_volt_backoff = c::temp_volt_depend_params {
            max_chip_temp: byte(c::CSP_MAX_CHIP_TEMP),
            min_chip_temp: byte(c::CSP_MIN_CHIP_TEMP),
            lb_max_pwr_bkf_hi_temp: byte(c::CSP_LB_MAX_PWR_BKF_HI_TEMP),
            lb_max_pwr_bkf_low_temp: byte(c::CSP_LB_MAX_PWR_BKF_LOW_TEMP),
            hb_max_pwr_bkf_hi_temp: byte(c::CSP_HB_MAX_PWR_BKF_HI_TEMP),
            hb_max_pwr_bkf_low_temp: byte(c::CSP_HB_MAX_PWR_BKF_LOW_TEMP),
            lb_vbt_lt_vlow: byte(c::CSP_LB_VBT_LT_VLOW),
            hb_vbt_lt_vlow: byte(c::CSP_HB_VBT_LT_VLOW),
            lb_vbt_lt_low: byte(c::CSP_LB_VBT_LT_LOW),
            hb_vbt_lt_low: byte(c::CSP_HB_VBT_LT_LOW),
            reserved: [0; 4],
        };
    } else {
        p.xo_offset.xo_freq_offset = c::QFN_XO_VAL as u8;
        p.syst_tx_pwr_offset = c::tx_pwr_systm_offset {
            syst_off_lb_chan: byte(c::QFN_SYSTEM_OFFSET_LB),
            syst_off_hb_low_chan: byte(c::QFN_SYSTEM_OFFSET_HB_CHAN_LOW),
            syst_off_hb_mid_chan: byte(c::QFN_SYSTEM_OFFSET_HB_CHAN_MID),
            syst_off_hb_high_chan: byte(c::QFN_SYSTEM_OFFSET_HB_CHAN_HIGH),
        };
        p.max_pwr_ceil = c::tx_pwr_ceil {
            max_dsss_pwr: byte(c::QFN_MAX_TX_PWR_DSSS),
            max_lb_mcs7_pwr: byte(c::QFN_MAX_TX_PWR_LB_MCS7),
            max_lb_mcs0_pwr: byte(c::QFN_MAX_TX_PWR_LB_MCS0),
            max_hb_low_chan_mcs7_pwr: byte(c::QFN_MAX_TX_PWR_HB_LOW_CHAN_MCS7),
            max_hb_mid_chan_mcs7_pwr: byte(c::QFN_MAX_TX_PWR_HB_MID_CHAN_MCS7),
            max_hb_high_chan_mcs7_pwr: byte(c::QFN_MAX_TX_PWR_HB_HIGH_CHAN_MCS7),
            max_hb_low_chan_mcs0_pwr: byte(c::QFN_MAX_TX_PWR_HB_LOW_CHAN_MCS0),
            max_hb_mid_chan_mcs0_pwr: byte(c::QFN_MAX_TX_PWR_HB_MID_CHAN_MCS0),
            max_hb_high_chan_mcs0_pwr: byte(c::QFN_MAX_TX_PWR_HB_HIGH_CHAN_MCS0),
        };
        p.temp_volt_backoff = c::temp_volt_depend_params {
            max_chip_temp: byte(c::QFN_MAX_CHIP_TEMP),
            min_chip_temp: byte(c::QFN_MIN_CHIP_TEMP),
            lb_max_pwr_bkf_hi_temp: byte(c::QFN_LB_MAX_PWR_BKF_HI_TEMP),
            lb_max_pwr_bkf_low_temp: byte(c::QFN_LB_MAX_PWR_BKF_LOW_TEMP),
            hb_max_pwr_bkf_hi_temp: byte(c::QFN_HB_MAX_PWR_BKF_HI_TEMP),
            hb_max_pwr_bkf_low_temp: byte(c::QFN_HB_MAX_PWR_BKF_LOW_TEMP),
            lb_vbt_lt_vlow: byte(c::QFN_LB_VBT_LT_VLOW),
            hb_vbt_lt_vlow: byte(c::QFN_HB_VBT_LT_VLOW),
            lb_vbt_lt_low: byte(c::QFN_LB_VBT_LT_LOW),
            hb_vbt_lt_low: byte(c::QFN_HB_VBT_LT_LOW),
            reserved: [0; 4],
        };
    }

    // The PHY defaults. The band edge backoffs, antenna gains and PCB losses that follow them stay
    // 0, the SDK's Kconfig defaults.
    let hex = &c::SYS_DEF_RF_PARAMS[..c::SYS_DEF_RF_PARAMS.len() - 1];
    for (dst, [high, low]) in p.phy_params.iter_mut().zip(hex.as_chunks::<2>().0) {
        *dst = hex_nibble(*high) << 4 | hex_nibble(*low);
    }

    if otp.flags & !(c::CALIB_XO_FLAG_MASK as u32) == 0 {
        let calib = otp.info.calib;
        let offset = c::OTP_OFF_CALIB_XO as usize;
        p.xo_offset.xo_freq_offset = calib[offset / 4].to_le_bytes()[offset % 4];
    }

    let backoffs = match c::ft_prog_ver::try_from((otp.ft_prog_ver & c::FT_PROG_VER_MASK) >> 16) {
        Ok(c::ft_prog_ver::FT_PROG_VER1) => [
            c::FT_PROG_VER1_2G_DSSS_TXCEIL_BKOFF,
            c::FT_PROG_VER1_2G_OFDM_TXCEIL_BKOFF,
            c::FT_PROG_VER1_5G_LOW_OFDM_TXCEIL_BKOFF,
            c::FT_PROG_VER1_5G_MID_OFDM_TXCEIL_BKOFF,
            c::FT_PROG_VER1_5G_HIGH_OFDM_TXCEIL_BKOFF,
        ],
        Ok(c::ft_prog_ver::FT_PROG_VER2) => [
            c::FT_PROG_VER2_2G_DSSS_TXCEIL_BKOFF,
            c::FT_PROG_VER2_2G_OFDM_TXCEIL_BKOFF,
            c::FT_PROG_VER2_5G_LOW_OFDM_TXCEIL_BKOFF,
            c::FT_PROG_VER2_5G_MID_OFDM_TXCEIL_BKOFF,
            c::FT_PROG_VER2_5G_HIGH_OFDM_TXCEIL_BKOFF,
        ],
        Ok(c::ft_prog_ver::FT_PROG_VER3) => [
            c::FT_PROG_VER3_2G_DSSS_TXCEIL_BKOFF,
            c::FT_PROG_VER3_2G_OFDM_TXCEIL_BKOFF,
            c::FT_PROG_VER3_5G_LOW_OFDM_TXCEIL_BKOFF,
            c::FT_PROG_VER3_5G_MID_OFDM_TXCEIL_BKOFF,
            c::FT_PROG_VER3_5G_HIGH_OFDM_TXCEIL_BKOFF,
        ],
        Err(_) => [0; 5],
    };
    let [dsss_2g, ofdm_2g, ofdm_5g_low, ofdm_5g_mid, ofdm_5g_high] = backoffs.map(|b| b as i32);

    // Both ceilings are in quarter dB.
    let ceiling = |board_dbm: u8, package: i8, backoff: i32| -> i8 {
        ((board_dbm as i32 * 4).min(package as i32) - backoff) as i8
    };
    let pkg = p.max_pwr_ceil;
    p.max_pwr_ceil = c::tx_pwr_ceil {
        max_dsss_pwr: ceiling(board.dsss_2g, pkg.max_dsss_pwr, dsss_2g),
        max_lb_mcs7_pwr: ceiling(board.mcs7_2g, pkg.max_lb_mcs7_pwr, ofdm_2g),
        max_lb_mcs0_pwr: ceiling(board.mcs0_2g, pkg.max_lb_mcs0_pwr, ofdm_2g),
        max_hb_low_chan_mcs7_pwr: ceiling(board.mcs7_5g_low, pkg.max_hb_low_chan_mcs7_pwr, ofdm_5g_low),
        max_hb_mid_chan_mcs7_pwr: ceiling(board.mcs7_5g_mid, pkg.max_hb_mid_chan_mcs7_pwr, ofdm_5g_mid),
        max_hb_high_chan_mcs7_pwr: ceiling(board.mcs7_5g_high, pkg.max_hb_high_chan_mcs7_pwr, ofdm_5g_high),
        max_hb_low_chan_mcs0_pwr: ceiling(board.mcs0_5g_low, pkg.max_hb_low_chan_mcs0_pwr, ofdm_5g_low),
        max_hb_mid_chan_mcs0_pwr: ceiling(board.mcs0_5g_mid, pkg.max_hb_mid_chan_mcs0_pwr, ofdm_5g_mid),
        max_hb_high_chan_mcs0_pwr: ceiling(board.mcs0_5g_high, pkg.max_hb_high_chan_mcs0_pwr, ofdm_5g_high),
    };

    p
}

/// The MAC address programmed in OTP (`MAC0`), if it is a valid unicast address.
fn otp_mac_address(otp: &c::host_rpu_umac_info) -> Option<[u8; 6]> {
    let words = otp.mac_address0;
    let mut bytes = [0u8; 8];
    bytes[..4].copy_from_slice(&words[0].to_le_bytes());
    bytes[4..].copy_from_slice(&words[1].to_le_bytes());
    let mac: [u8; 6] = unwrap!(bytes[..6].try_into());
    let unset = mac == [0; 6] || mac == [0xFF; 6];
    let multicast = mac[0] & 0x01 != 0;
    (!unset && !multicast).then_some(mac)
}

/// Locally administered address used when the OTP holds none.
const FALLBACK_MAC_ADDRESS: [u8; 6] = [0x02, 0x70, 0x02, 0x00, 0x00, 0x01];

impl<'a, BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> Runner<'a, BUS, IN, OUT> {
    /// Turns the chip on, loads its firmware and brings the interface up.
    pub(crate) async fn init(&mut self) {
        let config = self.config;
        self.rpu.reset();
        self.tx_tokens_busy = 0;

        info!("power on...");
        Timer::after(Duration::from_millis(10)).await;
        self.bucken.set_high().unwrap();
        Timer::after(Duration::from_millis(10)).await;
        self.iovdd_ctl.set_high().unwrap();
        Timer::after(Duration::from_millis(10)).await;

        info!("wakeup...");
        self.rpu.wakeup().await;

        info!("enable clocks...");
        self.rpu.raw_write32(PBUS, 0x8C20, 0x0100).await;

        info!("enable interrupt...");
        self.rpu.irq_enable().await;

        // Based on 'nrf_wifi_fmac_fw_load'
        let fw = parse_firmware(FIRMWARE);

        info!("reset processors...");
        self.rpu.proc_reset(Processor::Lmac).await;
        self.rpu.proc_reset(Processor::Umac).await;

        info!("load firmware patches...");
        self.rpu
            .load_fw(Processor::Umac, c::RPU_MEM_UMAC_PATCH_BIMG, fw.umac_pri)
            .await;
        self.rpu
            .load_fw(Processor::Umac, c::RPU_MEM_UMAC_PATCH_BIN, fw.umac_sec)
            .await;
        self.rpu
            .load_fw(Processor::Lmac, c::RPU_MEM_LMAC_PATCH_BIMG, fw.lmac_pri)
            .await;
        self.rpu
            .load_fw(Processor::Lmac, c::RPU_MEM_LMAC_PATCH_BIN, fw.lmac_sec)
            .await;

        info!("booting LMAC...");
        self.rpu.proc_boot(Processor::Lmac).await;

        info!("booting UMAC...");
        self.rpu.proc_boot(Processor::Umac).await;

        let umac_ver = self.rpu.read32(c::RPU_MEM_UMAC_VER, None).await.to_be_bytes();
        let lmac_ver = self.rpu.read32(c::RPU_MEM_LMAC_VER, None).await.to_be_bytes();
        info!(
            "firmware booted: UMAC {}.{}.{}.{}, LMAC {}.{}.{}.{}",
            umac_ver[0], umac_ver[1], umac_ver[2], umac_ver[3], lmac_ver[0], lmac_ver[1], lmac_ver[2], lmac_ver[3]
        );

        info!("Initializing rpu info...");
        self.rpu.init_info().await;

        info!("Reading OTP...");
        let otp = self.rpu.read_otp().await;
        let rf_params = rf_params(&otp, &config.max_tx_power);
        let mac_addr = match otp_mac_address(&otp.info) {
            Some(mac) => mac,
            None => {
                warn!("no MAC address in OTP, using a locally administered one");
                FALLBACK_MAC_ADDRESS
            }
        };
        info!(
            "OTP flags {:08x}, package {:08x}, test program {:08x}, MAC address {:02x}",
            otp.flags, otp.package_info, otp.ft_prog_ver, mac_addr
        );

        info!("Enabling interrupts...");
        self.rpu.irq_enable().await;

        info!("Initializing RX...");
        self.rpu.init_rx().await;

        info!("Initializing umac...");
        self.init_umac(&rf_params, &config).await;
        self.init_done = false;
        if !self.wait_until(EVENT_TIMEOUT, |r| r.init_done).await {
            panic!("timed out waiting for INIT_DONE");
        }
        info!("======== INIT DONE!! ==========");

        info!("Bringing the interface up...");
        self.set_mac_address(mac_addr).await;
        self.set_station_address(mac_addr);
        self.set_ap_address(mac_addr);
        self.state_ch
            .set_hardware_address(ch::driver::HardwareAddress::Ethernet(mac_addr));
        self.set_interface_state(true).await;
        self.powered = true;
        if self.power_save {
            self.set_power_save(true).await;
        }
    }

    /// Puts the chip in its shutdown state (NCS `rpu_pwroff`), after leaving the network.
    pub(crate) async fn power_off(&mut self) {
        match self.sta.conn {
            ConnState::Idle => {}
            ConnState::Connected => self.leave().await,
            _ => self.connect_failed(ConnectError::PoweredOff).await,
        }
        if self.scan_deadline.take().is_some() {
            self.push_scan_event(ScanEvent::Aborted);
        }
        // Time for the deauthentication frame to leave.
        Timer::after(POWER_OFF_DELAY).await;
        self.iovdd_ctl.set_low().unwrap();
        self.bucken.set_low().unwrap();
        self.powered = false;
        self.rpu.awake = false;
        self.rpu.irq_pending_ack = false;
        self.forget_ap();
        self.tx_tokens_busy = 0;
        info!("powered off");
    }

    /// Sends the system init command (NCS `umac_cmd_sys_init`), with the SDK's defaults for a
    /// station.
    async fn init_umac(&mut self, rf_params: &c::phy_rf_params, config: &Config) {
        let mut rf_params_bytes = [0u8; c::RF_PARAMS_SIZE as usize];
        rf_params_bytes.copy_from_slice(sliceit(rf_params));

        let rx_buf_pool = c::rx_buf_pool_params {
            buf_sz: RX_MAX_DATA_SIZE as _, // the RPU adds the headroom itself
            num_bufs: RX_BUFS_PER_QUEUE as _,
        };
        let mut cmd = c::cmd_sys_init {
            sys_head: unsafe { zeroed() },
            wdev_id: 0,
            sys_params: c::sys_params {
                sleep_enable: if config.low_power {
                    c::HW_SLEEP_ENABLE
                } else {
                    c::SLEEP_DISABLE
                },
                hw_bringup_time: c::HW_DELAY,
                sw_bringup_time: c::SW_DELAY,
                bcn_time_out: c::BCN_TIMEOUT,
                calib_sleep_clk: c::CALIB_SLEEP_CLOCK_ENABLE,
                phy_calib: c::DEF_PHY_CALIB,
                mac_addr: [0; 6],
                rf_params: rf_params_bytes,
                rf_params_valid: 1,
            },
            rx_buf_pools: [rx_buf_pool; c::MAX_NUM_OF_RX_QUEUES as usize],
            data_config_params: c::data_config_params {
                rate_protection_type: 0,
                // No A-MPDU aggregation. The driver hands the RPU one frame per token, so there
                // is nothing to aggregate, and with it on the RPU gives up on a frame after a few
                // tries: 0.2% of them lost on a 5 GHz link, 1.5% on a busy 2.4 GHz one, each of
                // which stalls a TCP sender.
                aggregation: 0,
                wmm: 1,
                max_num_tx_agg_sessions: 4,
                max_num_rx_agg_sessions: 8,
                max_tx_aggregation: MAX_TX_AGGREGATION as _,
                // NCS uses half the RX buffers (CONFIG_NRF70_RX_NUM_BUFS / 2).
                reorder_buf_size: (RX_BUFS / 2) as u8,
                max_rxampdu_size: 3,
            },
            temp_vbat_config_params: c::temp_vbat_config {
                temp_based_calib_en: c::TEMP_CALIB_ENABLE,
                temp_calib_bitmap: c::DEF_PHY_TEMP_CALIB,
                vbat_calibp_bitmap: c::DEF_PHY_VBAT_CALIB,
                temp_vbat_mon_period: c::TEMP_CALIB_PERIOD,
                vth_very_low: c::VBAT_VERYLOW as _,
                vth_low: c::VBAT_LOW as _,
                vth_hi: c::VBAT_HIGH as _,
                temp_threshold: c::TEMP_CALIB_THRESHOLD as _,
                vbat_threshold: 0,
            },
            tcp_ip_checksum_offload: 0,
            country_code: config.country_code,
            op_band: c::op_band::BAND_ALL as _,
            // Management frames stay in the RPU instead of taking RX buffers.
            mgmt_buff_offload: 1,
            feature_flags: 0,
            disable_beamforming: 0,
            // NCS CONFIG_NRF_WIFI_AP_DEAD_DETECT_TIMEOUT, in seconds.
            discon_timeout: 20,
            ps_exit_strategy: c::ps_exit_strategy::EVERY_TIM as _,
            // The watchdog serves the SDK's RPU recovery, which the driver does not have: off.
            watchdog_timer_val: 0xFFFFFF,
            keep_alive_enable: 0,
            keep_alive_period: 0,
            // NCS CONFIG_NRF_WIFI_DISPLAY_SCAN_BSS_LIMIT.
            display_scan_bss_limit: 150,
            coex_disable_ptiwin_for_wifi_scan: 0,
            raw_scan_enable: 0,
            // NCS CONFIG_NRF_WIFI_MAX_PS_POLL_FAIL_CNT.
            max_ps_poll_fail_cnt: 10,
            // NCS CONFIG_NRF_WIFI_RX_STBC_HT.
            stbc_enable_in_ht: 1,
            dbs_war_ctrl: 0,
            dynamic_ed: 0,
            // No coexistence with a short-range radio.
            bt_slot_time_in_ms: 0,
            bt_coex_disable: 1,
            display_scan_abort_on_bss_limit: 0,
        };
        self.rpu.send_cmd(&mut cmd).await;
    }

    /// Sets the MAC address of the default interface (NCS `wifi_nrf_fmac_set_vif_macaddr`).
    async fn set_mac_address(&mut self, mac_addr: [u8; 6]) {
        let mut cmd: c::umac_cmd_change_macaddr = unsafe { zeroed() };
        cmd.macaddr_info.mac_addr = mac_addr;
        self.rpu.send_cmd(&mut cmd).await;
    }

    /// Brings the default interface up or down and waits for the RPU to confirm (NCS
    /// `nrf_wifi_sys_fmac_chg_vif_state`).
    pub(crate) async fn set_interface_state(&mut self, up: bool) {
        let mut cmd: c::umac_cmd_chg_vif_state = unsafe { zeroed() };
        cmd.info.state = up as _;
        cmd.info.if_index = 0;
        self.interface_state_set = false;
        self.rpu.send_cmd(&mut cmd).await;
        if !self.wait_until(EVENT_TIMEOUT, |r| r.interface_state_set).await {
            panic!("timed out waiting for IFFLAGS_STATUS");
        }
    }
}

#[cfg(test)]
mod tests {
    use core::{assert, assert_eq};

    use super::*;
    use crate::rpu::regions;

    /// The nRF7002-DK's ceilings, from its devicetree.
    const DK_CEILING: TxPowerCeiling = TxPowerCeiling {
        dsss_2g: 21,
        mcs0_2g: 16,
        mcs7_2g: 16,
        mcs0_5g_low: 9,
        mcs7_5g_low: 9,
        mcs0_5g_mid: 11,
        mcs7_5g_mid: 11,
        mcs0_5g_high: 13,
        mcs7_5g_high: 13,
    };

    /// OTP whose crystal calibration byte is 0x2d, with the given flags and package.
    fn otp(flags: u32, package_info: u32) -> Otp {
        let mut info: c::host_rpu_umac_info = unsafe { zeroed() };
        let mut calib = [0u32; 9];
        calib[c::OTP_OFF_CALIB_XO as usize / 4] = 0x2d << (8 * (c::OTP_OFF_CALIB_XO % 4));
        info.calib = calib;
        Otp {
            info,
            flags,
            ft_prog_ver: 0xFFFF_FFFF,
            package_info,
        }
    }

    const NOT_PROGRAMMED: u32 = 0xFFFF_FFFF;

    #[test]
    fn bundled_firmware_matches_the_bindings() {
        let fw = parse_firmware(FIRMWARE);
        let images = [fw.umac_pri, fw.umac_sec, fw.lmac_pri, fw.lmac_sec];
        assert!(images.iter().all(|image| !image.is_empty()));
        let headers = size_of::<c::nrf70_fw_image_info>() + 4 * size_of::<c::nrf70_fw_image>();
        let data: usize = images.iter().map(|image| image.len()).sum();
        assert_eq!(headers + data, FIRMWARE.len());
    }

    #[test]
    fn firmware_images_fit_their_patch_regions() {
        let fw = parse_firmware(FIRMWARE);
        let patches = [
            (Processor::Umac, c::RPU_MEM_UMAC_PATCH_BIMG, fw.umac_pri),
            (Processor::Umac, c::RPU_MEM_UMAC_PATCH_BIN, fw.umac_sec),
            (Processor::Lmac, c::RPU_MEM_LMAC_PATCH_BIMG, fw.lmac_pri),
            (Processor::Lmac, c::RPU_MEM_LMAC_PATCH_BIN, fw.lmac_sec),
        ];
        for (processor, addr, image) in patches {
            let (mem, offs) = regions::remap_global_addr_to_region_and_offset(addr, Some(processor));
            // load_fw pads the last chunk to a whole word.
            assert!(mem.start + offs + image.len().next_multiple_of(4) as u32 <= mem.end);
        }
        // The secondary images end before the primary ones start.
        assert!(c::RPU_MEM_UMAC_PATCH_BIN + fw.umac_sec.len() as u32 <= c::RPU_MEM_UMAC_PATCH_BIMG);
        assert!(c::RPU_MEM_LMAC_PATCH_BIN + fw.lmac_sec.len() as u32 <= c::RPU_MEM_LMAC_PATCH_BIMG);
    }

    #[test]
    fn unprogrammed_package_gets_the_qfn_defaults() {
        let p = rf_params(&otp(NOT_PROGRAMMED, NOT_PROGRAMMED), &DK_CEILING);
        let xo = p.xo_offset.xo_freq_offset;
        let syst = p.syst_tx_pwr_offset;
        assert_eq!(xo, c::QFN_XO_VAL as u8);
        assert_eq!(syst.syst_off_lb_chan, byte(c::QFN_SYSTEM_OFFSET_LB));
    }

    #[test]
    fn csp_package_gets_the_csp_defaults() {
        let p = rf_params(&otp(NOT_PROGRAMMED, c::CSP_PACKAGE_INFO), &DK_CEILING);
        let syst = p.syst_tx_pwr_offset;
        let temp_volt = p.temp_volt_backoff;
        assert_eq!(syst.syst_off_lb_chan, byte(c::CSP_SYSTEM_OFFSET_LB));
        assert_eq!(temp_volt.hb_vbt_lt_vlow, byte(c::CSP_HB_VBT_LT_VLOW));
    }

    #[test]
    fn crystal_calibration_comes_from_otp_when_programmed() {
        let flags = c::CALIB_XO_FLAG_MASK as u32;
        let p = rf_params(&otp(flags, NOT_PROGRAMMED), &DK_CEILING);
        let xo = p.xo_offset.xo_freq_offset;
        assert_eq!(xo, 0x2d);
    }

    #[test]
    fn tx_power_ceiling_is_the_lower_of_board_and_package() {
        let p = rf_params(&otp(NOT_PROGRAMMED, NOT_PROGRAMMED), &DK_CEILING);
        let ceil = p.max_pwr_ceil;
        // Quarter dB. The board is lower here...
        assert_eq!(ceil.max_hb_low_chan_mcs7_pwr, 9 * 4);
        // ...and the package there (QFN_MAX_TX_PWR_HB_HIGH_CHAN_MCS0, 12 dBm, under the DK's 13).
        assert_eq!(
            ceil.max_hb_high_chan_mcs0_pwr,
            byte(c::QFN_MAX_TX_PWR_HB_HIGH_CHAN_MCS0)
        );
        assert_eq!(ceil.max_dsss_pwr, 21 * 4);
    }

    #[test]
    fn phy_params_are_the_sdk_defaults() {
        let p = rf_params(&otp(NOT_PROGRAMMED, NOT_PROGRAMMED), &DK_CEILING);
        let phy = p.phy_params;
        assert_eq!(phy[..5], [0x00, 0x70, 0x77, 0x00, 0x3F]);
        // Band edge backoffs, antenna gains and PCB losses stay 0.
        let edges = c::EDGE_BACKOFF_OFFSETS::BAND_2G_LW_ED_BKF_DSSS_OFST as usize - c::RF_PARAMS_CONF_SIZE as usize;
        assert!(phy[edges..edges + 34].iter().all(|&b| b == 0));
    }

    fn umac_info_with_mac(mac: [u8; 6]) -> c::host_rpu_umac_info {
        let mut bytes = [0u8; 8];
        bytes[..6].copy_from_slice(&mac);
        let mut info: c::host_rpu_umac_info = unsafe { zeroed() };
        info.mac_address0 = [
            u32::from_le_bytes(unwrap!(bytes[..4].try_into())),
            u32::from_le_bytes(unwrap!(bytes[4..].try_into())),
        ];
        info
    }

    #[test]
    fn otp_mac_address_must_be_unicast_and_programmed() {
        let mac = [0xf4, 0xce, 0x36, 0x00, 0x8b, 0x19];
        assert_eq!(otp_mac_address(&umac_info_with_mac(mac)), Some(mac));
        assert_eq!(otp_mac_address(&umac_info_with_mac([0xFF; 6])), None);
        assert_eq!(otp_mac_address(&umac_info_with_mac([0; 6])), None);
        assert_eq!(otp_mac_address(&umac_info_with_mac([0x01, 0, 0, 0, 0, 1])), None);
    }
}
