use core::mem::size_of;

use self::structures::SysHead;
use crate::config::{DataConfigParams, RxBufPoolParams};
use crate::messages::{Message, RpuMessageType};

#[repr(C, packed)]
pub struct SysCommand<V: SysCommandVariant> {
    sys_head: SysHead,
    data: V,
}

impl<V: SysCommandVariant> SysCommand<V> {
    pub fn new(data: V) -> Self {
        Self {
            sys_head: SysHead {
                cmd_event: V::COMMAND_TYPE as _,
                len: size_of::<Self>() as _,
            },
            data,
        }
    }
}

pub trait SysCommandVariant {
    const COMMAND_TYPE: SysCommandType;
}

impl<V: SysCommandVariant> Message for SysCommand<V> {
    const MESSAGE_TYPE: RpuMessageType = RpuMessageType::System;
}

#[repr(u32)]
pub enum SysCommandType {
    Init,
    Tx,
    IfType,
    Mode,
    GetStats,
    ClearStats,
    Rx,
    Power,
    DeInit,
    BTCoex,
    RfTest,
    HeGiLtfConfig,
    UmacIntStats,
    RadioTestInit,
}

#[doc = " struct nrf_wifi_cmd_sys_init - Initialize UMAC\n @sys_head: umac header, see &nrf_wifi_sys_head\n @wdev_id : id of the interface.\n @sys_params: iftype, mac address, see nrf_wifi_sys_params\n @rx_buf_pools: LMAC Rx buffs pool params, see struct rx_buf_pool_params\n @data_config_params: Data configuration params, see struct nrf_wifi_data_config_params\n After host driver bringup host sends the NRF_WIFI_CMD_INIT to the RPU.\n then RPU initializes and responds with NRF_WIFI_EVENT_BUFF_CONFIG."]
#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
pub(crate) struct Init {
    pub wdev_id: ::core::ffi::c_uint,
    pub sys_params: structures::SysParams,
    pub rx_buf_pools: [RxBufPoolParams; 3],
    pub data_config_params: DataConfigParams,
    pub temp_vbat_config_params: structures::TempVbatConfig,
}

impl SysCommandVariant for Init {
    const COMMAND_TYPE: SysCommandType = SysCommandType::Init;
}

pub mod structures {
    #[doc = " struct nrf_wifi_sys_head - Command/Event header.\n @cmd: Command/Event id.\n @len: Payload length.\n\n This header needs to be initialized in every command and has the event\n id info in case of events."]
    #[repr(C, packed)]
    #[derive(Debug, Copy, Clone)]
    pub struct SysHead {
        pub cmd_event: ::core::ffi::c_uint,
        pub len: ::core::ffi::c_uint,
    }

    #[doc = " struct nrf_wifi_sys_params - Init parameters during NRF_WIFI_CMD_INIT\n @mac_addr: MAC address of the interface\n @sleep_enable: enable rpu sleep\n @hw_bringup_time:\n @sw_bringup_time:\n @bcn_time_out:\n @calib_sleep_clk:\n @rf_params: RF parameters\n @rf_params_valid: Indicates whether the @rf_params has a valid value.\n @phy_calib: PHY calibration parameters\n\n System parameters provided for command NRF_WIFI_CMD_INIT"]
    #[repr(C, packed)]
    #[derive(Debug, Copy, Clone)]
    pub struct SysParams {
        pub sleep_enable: ::core::ffi::c_uint,
        pub hw_bringup_time: ::core::ffi::c_uint,
        pub sw_bringup_time: ::core::ffi::c_uint,
        pub bcn_time_out: ::core::ffi::c_uint,
        pub calib_sleep_clk: ::core::ffi::c_uint,
        pub phy_calib: ::core::ffi::c_uint,
        pub mac_addr: [::core::ffi::c_uchar; 6usize],
        pub rf_params: [::core::ffi::c_uchar; 200usize],
        pub rf_params_valid: ::core::ffi::c_uchar,
    }

    #[repr(C, packed)]
    #[derive(Debug, Copy, Clone)]
    pub struct TempVbatConfig {
        pub temp_based_calib_en: ::core::ffi::c_uint,
        pub temp_calib_bitmap: ::core::ffi::c_uint,
        pub vbat_calibp_bitmap: ::core::ffi::c_uint,
        pub temp_vbat_mon_period: ::core::ffi::c_uint,
        pub vth_very_low: ::core::ffi::c_int,
        pub vth_low: ::core::ffi::c_int,
        pub vth_hi: ::core::ffi::c_int,
        pub temp_threshold: ::core::ffi::c_int,
        pub vbat_threshold: ::core::ffi::c_int,
    }
}
