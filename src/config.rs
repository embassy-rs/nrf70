use crate::{MemoryRegion, MAX_NUM_OF_RX_QUEUES};

pub(crate) const RX_BUF_HEADROOM: usize = 4;
pub(crate) const TX_BUF_HEADROOM: usize = 52;

pub struct Config<
    const TX_BUFFERS: usize,
    const TX_BUFFER_SIZE_MAX: usize,
    const RX_BUFFERS: usize,
    const RX_BUFFER_SIZE_MAX: usize,
> {
    pub(crate) rx_buf_pools: [RxBufPoolParams; 3],
    pub(crate) rx_desc: [u32; MAX_NUM_OF_RX_QUEUES],
    pub(crate) data_config: DataConfigParams,
    pub(crate) num_tx_tokens: u16,
    pub(crate) num_tx_tokens_per_ac: u16,
    pub(crate) num_tx_tokens_spare: u16,

    pub(crate) rpu_tx_buffer_base: (&'static MemoryRegion, u32),
    pub(crate) rpu_rx_buffer_base: [(&'static MemoryRegion, u32); MAX_NUM_OF_RX_QUEUES],
}

impl Config<0, 0, 3000, 8> {
    pub fn new_only_scan() -> Self {
        Self {
            rx_buf_pools: [RxBufPoolParams {
                buf_sz: 2,
                num_bufs: 1000,
            }; 3],
            rx_desc: [0, 1000, 2000],
            data_config: DataConfigParams::default(),
            num_tx_tokens: 0,
            num_tx_tokens_per_ac: 0,
            num_tx_tokens_spare: 0,
            rpu_tx_buffer_base: (crate::regions::PKTRAM, 0x5000),
            rpu_rx_buffer_base: [
                (crate::regions::PKTRAM, 0x5000 + 2000 * 0),
                (crate::regions::PKTRAM, 0x5000 + 2000 * 1),
                (crate::regions::PKTRAM, 0x5000 + 2000 * 2),
            ],
        }
    }
}

impl<
        const TX_BUFFERS: usize,
        const TX_BUFFER_SIZE_MAX: usize,
        const RX_BUFFERS: usize,
        const RX_BUFFER_SIZE_MAX: usize,
    > Config<TX_BUFFERS, TX_BUFFER_SIZE_MAX, RX_BUFFERS, RX_BUFFER_SIZE_MAX>
{
    pub const fn new_with_tx(max_tx_aggregation: u8) -> Self {
        const RPU_PKTRAM_SIZE: usize = 0x2C000;

        let max_tx_tokens = TX_BUFFERS / max_tx_aggregation as usize;
        assert!(
            TX_BUFFERS as u16 % max_tx_aggregation as u16 == 0,
            "TX_BUFFERS must be a multiple of max_tx_aggregation"
        );

        assert!(max_tx_aggregation <= 16, "Max TX aggregation is 16");
        assert!(RX_BUFFERS >= 1, "At least one RX buffer is required");
        assert!(
            RX_BUFFERS % MAX_NUM_OF_RX_QUEUES == 0,
            "RX_BUFFERS must be a multiple of 3"
        );

        assert!(TX_BUFFER_SIZE_MAX > TX_BUF_HEADROOM);
        assert!(RX_BUFFER_SIZE_MAX > RX_BUF_HEADROOM);

        const MAX_RX_QUEUES: usize = 3;

        let total_tx_size = TX_BUFFERS * TX_BUFFER_SIZE_MAX;
        let total_rx_size = RX_BUFFERS * RX_BUFFER_SIZE_MAX;

        assert!(
            (total_tx_size + total_rx_size) <= RPU_PKTRAM_SIZE,
            "Packet RAM overflow in Sheliak"
        );

        const WIFI_NRF_FMAC_AC_MAX: u16 = 5;

        Self {
            rx_buf_pools: [RxBufPoolParams {
                buf_sz: (RX_BUFFER_SIZE_MAX - RX_BUF_HEADROOM) as u16,
                num_bufs: (RX_BUFFERS / MAX_RX_QUEUES) as u16,
            }; 3],
            rx_desc: [
                0,
                (RX_BUFFERS / MAX_RX_QUEUES) as u32,
                (RX_BUFFERS / MAX_RX_QUEUES) as u32 * 2,
            ],
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
            num_tx_tokens: max_tx_tokens as u16,
            num_tx_tokens_per_ac: max_tx_tokens as u16 / WIFI_NRF_FMAC_AC_MAX,
            num_tx_tokens_spare: max_tx_tokens as u16 % WIFI_NRF_FMAC_AC_MAX,
            rpu_tx_buffer_base: (crate::regions::PKTRAM, 0x5000),
            rpu_rx_buffer_base: [
                (
                    crate::regions::PKTRAM,
                    0x5000
                        + total_tx_size as u32
                        + (RX_BUFFER_SIZE_MAX - RX_BUF_HEADROOM) as u32 * (RX_BUFFERS / MAX_RX_QUEUES) as u32 * 0,
                ),
                (
                    crate::regions::PKTRAM,
                    0x5000
                        + total_tx_size as u32
                        + (RX_BUFFER_SIZE_MAX - RX_BUF_HEADROOM) as u32 * (RX_BUFFERS / MAX_RX_QUEUES) as u32 * 1,
                ),
                (
                    crate::regions::PKTRAM,
                    0x5000
                        + total_tx_size as u32
                        + (RX_BUFFER_SIZE_MAX - RX_BUF_HEADROOM) as u32 * (RX_BUFFERS / MAX_RX_QUEUES) as u32 * 2,
                ),
            ],
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
