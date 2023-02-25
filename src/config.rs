pub struct Config {
    pub(crate) rx_buf_pools: [RxBufPoolParams; 3],
    pub(crate) data_config: DataConfigParams,
}

impl Config {
    pub fn new_only_scan() -> Self {
        Self {
            rx_buf_pools: [RxBufPoolParams {
                buf_sz: 2,
                num_bufs: 1000,
            }; 3],
            data_config: DataConfigParams::default(),
        }
    }

    pub const fn new_with_tx(
        max_tx_tokens: u16,
        max_tx_aggregation: u8,
        tx_max_data_size: u16,
        rx_num_bufs: u16,
        rx_max_data_size: u16,
    ) -> Self {
        const RPU_PKTRAM_SIZE: u32 = 0x2C000;

        assert!(max_tx_tokens >= 1, "At least one TX token is required");
        assert!(max_tx_aggregation <= 16, "Max TX aggregation is 16");
        assert!(rx_num_bufs >= 1, "At least one RX buffer is required");

        const MAX_RX_QUEUES: u16 = 3;
        const TX_BUF_HEADROOM: u32 = 52;

        let total_tx_frames = max_tx_tokens as u32 * max_tx_aggregation as u32;
        let max_tx_frame_size = tx_max_data_size as u32 * TX_BUF_HEADROOM;
        let total_tx_size = total_tx_frames * max_tx_frame_size;
        let total_rx_size = rx_num_bufs as u32 * rx_max_data_size as u32;

        assert!(
            (total_tx_size + total_rx_size) <= RPU_PKTRAM_SIZE,
            "Packet RAM overflow in Sheliak"
        );

        Self {
            rx_buf_pools: [RxBufPoolParams {
                buf_sz: rx_max_data_size,
                num_bufs: rx_num_bufs / MAX_RX_QUEUES,
            }; 3],
            data_config: DataConfigParams {
                rate_protection_type: 0,
                aggregation: 1,
                wmm: 1,
                max_num_tx_agg_sessions: 4,
                max_num_rx_agg_sessions: 8,
                max_tx_aggregation,
                reorder_buf_size: 64,
                max_rxampdu_size: 3,
            },
        }
    }
}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
pub(crate) struct RxBufPoolParams {
    pub buf_sz: ::core::ffi::c_ushort,
    pub num_bufs: ::core::ffi::c_ushort,
}

#[doc = " struct nrf_wifi_data_config_params - Data config parameters\n @rate_protection_type:0->NONE, 1->RTS/CTS, 2->CTS2SELF\n @aggregation: Agreegation is enabled(NRF_WIFI_FEATURE_ENABLE) or disabled\n\t\t(NRF_WIFI_FEATURE_DISABLE)\n @wmm: WMM is enabled(NRF_WIFI_FEATURE_ENABLE) or disabled\n\t\t(NRF_WIFI_FEATURE_DISABLE)\n @max_num_tx_agg_sessions: Max number of aggregated TX sessions\n @max_num_rx_agg_sessions: Max number of aggregated RX sessions\n @reorder_buf_size: Reorder buffer size (1 to 64)\n @max_rxampdu_size: Max RX AMPDU size (8/16/32/64 KB), see\n\t\t\t\t\tenum max_rx_ampdu_size\n\n Data configuration parameters provided in command NRF_WIFI_CMD_INIT"]
#[repr(C, packed)]
#[derive(Debug, Copy, Clone, Default)]
pub(crate) struct DataConfigParams {
    pub rate_protection_type: ::core::ffi::c_uchar,
    pub aggregation: ::core::ffi::c_uchar,
    pub wmm: ::core::ffi::c_uchar,
    pub max_num_tx_agg_sessions: ::core::ffi::c_uchar,
    pub max_num_rx_agg_sessions: ::core::ffi::c_uchar,
    pub max_tx_aggregation: ::core::ffi::c_uchar,
    pub reorder_buf_size: ::core::ffi::c_uchar,
    pub max_rxampdu_size: ::core::ffi::c_int,
}
