//! The RPU as the host reaches it over the bus (NCS's HAL, `hal_api.c` and `hal_mem.c`): its
//! memory, mapped into regions; the queues it exchanges messages, events and buffers through; its
//! interrupt; its sleep in low power mode; and the boot of its processors.

use core::mem::{size_of, zeroed};

use defmt::{debug, info, panic, trace, unwrap, warn};
use embassy_time::{with_timeout, Duration, Instant, Timer};

use crate::boot::Otp;
use crate::command::Command;
use crate::{c, slice8, slice8_mut, sliceit, unsliceit, Bus};

// ========= Packet RAM

// The packet RAM (0xB0000000 to 0xB0030FFF, of which 0xB0005000 on is for the host and the RPU)
// holds the TX buffers, `MAX_TX_AGGREGATION` per TX token, each a header then the frame, and then
// the RX buffers, `RX_BUFS_PER_QUEUE` per RX queue, each a 4-byte header then the frame. RX
// buffers are numbered by descriptor ID across the queues: queue 0 has 0 to N-1, queue 1 N to 2N-1
// and so on.

/// TX tokens: how many TX commands may be in flight.
pub(crate) const MAX_TX_TOKENS: usize = 10;

/// Frames per TX command.
pub(crate) const MAX_TX_AGGREGATION: usize = 6;

const TX_MAX_DATA_SIZE: usize = 1600;

pub(crate) const RX_MAX_DATA_SIZE: usize = 1600;

pub(crate) const RX_BUFS_PER_QUEUE: usize = 16;

const TX_BUFS: usize = MAX_TX_TOKENS * MAX_TX_AGGREGATION;

const TX_BUF_SIZE: usize = c::TX_BUF_HEADROOM as usize + TX_MAX_DATA_SIZE;

const TX_TOTAL_SIZE: usize = TX_BUFS * TX_BUF_SIZE;

pub(crate) const RX_BUFS: usize = RX_BUFS_PER_QUEUE * c::MAX_NUM_OF_RX_QUEUES as usize;

const RX_BUF_SIZE: usize = c::RX_BUF_HEADROOM as usize + RX_MAX_DATA_SIZE;

const RX_TOTAL_SIZE: usize = RX_BUFS * RX_BUF_SIZE;

const _: () = {
    use core::assert;
    assert!(MAX_TX_TOKENS >= 1, "At least one TX token is required");
    assert!(MAX_TX_AGGREGATION <= 16, "Max TX aggregation is 16");
    assert!(RX_BUFS_PER_QUEUE >= 1, "At least one RX buffer per queue is required");
    assert!(
        (TX_TOTAL_SIZE + RX_TOTAL_SIZE) as u32 <= c::RPU_PKTRAM_SIZE,
        "Packet RAM overflow"
    );
};

/// The packet RAM address of buffer `slot` of TX token `token`.
pub(crate) fn tx_buf_addr(token: usize, slot: usize) -> u32 {
    c::RPU_MEM_PKT_BASE + ((token * MAX_TX_AGGREGATION + slot) * TX_BUF_SIZE) as u32
}

/// The packet RAM address of RX buffer `desc_id`. The RX buffers follow the TX area.
pub(crate) fn rx_buf_addr(desc_id: usize) -> u32 {
    c::RPU_MEM_PKT_BASE + (TX_TOTAL_SIZE + RX_BUF_SIZE * desc_id) as u32
}

// ========= Bus access, events and commands

/// In low power mode, how long after the last bus access the RPU may sleep again (NCS
/// `NRF70_RPU_PS_IDLE_TIMEOUT_MS`).
const RPU_IDLE_TIMEOUT: Duration = Duration::from_millis(10);
/// How long the RPU gets to wake up (NCS `RPU_PS_WAKE_TIMEOUT_S`). The data sheet gives 6.7 ms.
const RPU_WAKE_TIMEOUT: Duration = Duration::from_secs(1);

/// Address bit 23 asks for an incrementing address, so that a burst reads or writes consecutive
/// words (NCS `addrmask`). Writes set it in the SPI command already.
const ADDR_INCREMENT: u32 = 0x80_0000;

pub(crate) const SR1_RPU_AWAKE: u8 = 0x02;
pub(crate) const SR1_RPU_READY: u8 = 0x04;

pub(crate) const SR2_RPU_WAKEUP_REQ: u8 = 0x01;

const MAX_EVENT_POOL_LEN: usize = 1000;

/// What is read of an event before its length is known: the data events (an RX event with one
/// frame takes 49 bytes, a TX done one about 40), which are most of them, in one read. NCS reads
/// `RPU_EVENT_COMMON_SIZE_MAX` (128 bytes).
const EVENT_HEAD: usize = 64;

/// Largest command the RPU takes in one buffer; longer ones are sent in fragments.
const MAX_CMD_SIZE: usize = c::MAX_UMAC_CMD_SIZE as usize;

/// How long a processor gets to write its boot signature (NCS `MCU_FW_BOOT_TIMEOUT_MS`).
const FW_BOOT_TIMEOUT: Duration = Duration::from_secs(1);

#[derive(Copy, Clone, Debug, defmt::Format)]
pub(crate) struct MemoryRegion {
    pub(crate) start: u32,
    pub(crate) end: u32,

    /// Number of dummy 32bit words
    pub(crate) latency: u32,

    rpu_mem_start: u32,
    rpu_mem_end: u32,
    processor_restriction: Option<Processor>,
}

#[rustfmt::skip]
pub(crate) mod regions {
    use super::*;
	pub(crate) const SYSBUS       : &MemoryRegion = &MemoryRegion { start: 0x000000, end: 0x008FFF, latency: 1, rpu_mem_start: 0xA4000000, rpu_mem_end: 0xA4FFFFFF, processor_restriction: None };
	pub(crate) const EXT_SYS_BUS  : &MemoryRegion = &MemoryRegion { start: 0x009000, end: 0x03FFFF, latency: 2, rpu_mem_start: 0,          rpu_mem_end: 0,          processor_restriction: None };
	pub(crate) const PBUS         : &MemoryRegion = &MemoryRegion { start: 0x040000, end: 0x07FFFF, latency: 1, rpu_mem_start: 0xA5000000, rpu_mem_end: 0xA5FFFFFF, processor_restriction: None };
	pub(crate) const PKTRAM       : &MemoryRegion = &MemoryRegion { start: 0x0C0000, end: 0x0F0FFF, latency: 0, rpu_mem_start: 0xB0000000, rpu_mem_end: 0xB0FFFFFF, processor_restriction: None };
	pub(crate) const GRAM         : &MemoryRegion = &MemoryRegion { start: 0x080000, end: 0x092000, latency: 2, rpu_mem_start: 0xB7000000, rpu_mem_end: 0xB7FFFFFF, processor_restriction: None };
	pub(crate) const LMAC_ROM     : &MemoryRegion = &MemoryRegion { start: 0x100000, end: 0x134000, latency: 1, rpu_mem_start: 0x80000000, rpu_mem_end: 0x80033FFF, processor_restriction: Some(Processor::Lmac) }; // ROM
	pub(crate) const LMAC_RET_RAM : &MemoryRegion = &MemoryRegion { start: 0x140000, end: 0x14C000, latency: 1, rpu_mem_start: 0x80040000, rpu_mem_end: 0x8004BFFF, processor_restriction: Some(Processor::Lmac) }; // retained RAM
	pub(crate) const LMAC_SRC_RAM : &MemoryRegion = &MemoryRegion { start: 0x180000, end: 0x190000, latency: 1, rpu_mem_start: 0x80080000, rpu_mem_end: 0x8008FFFF, processor_restriction: Some(Processor::Lmac) }; // scratch RAM
	pub(crate) const UMAC_ROM     : &MemoryRegion = &MemoryRegion { start: 0x200000, end: 0x261800, latency: 1, rpu_mem_start: 0x80000000, rpu_mem_end: 0x800617FF, processor_restriction: Some(Processor::Umac) }; // ROM
	pub(crate) const UMAC_RET_RAM : &MemoryRegion = &MemoryRegion { start: 0x280000, end: 0x2A4000, latency: 1, rpu_mem_start: 0x80080000, rpu_mem_end: 0x800A3FFF, processor_restriction: Some(Processor::Umac) }; // retained RAM
	pub(crate) const UMAC_SRC_RAM : &MemoryRegion = &MemoryRegion { start: 0x300000, end: 0x338000, latency: 1, rpu_mem_start: 0x80100000, rpu_mem_end: 0x80137FFF, processor_restriction: Some(Processor::Umac) }; // scratch RAM

    pub(crate) const REGIONS: [&MemoryRegion; 11] = [
        SYSBUS, EXT_SYS_BUS, PBUS, PKTRAM, GRAM, LMAC_ROM, LMAC_RET_RAM, LMAC_SRC_RAM, UMAC_ROM, UMAC_RET_RAM, UMAC_SRC_RAM
    ];

    #[doc(alias = "pal_rpu_addr_offset_get")]
    /// The region that holds `rpu_addr` as `processor` sees it, and the offset in it.
    pub(crate) fn remap_global_addr_to_region_and_offset(rpu_addr: u32, processor: Option<Processor>) -> (&'static MemoryRegion, u32) {
        defmt::unwrap!(
            REGIONS
                .into_iter()
                .filter(|region| region.processor_restriction.is_none() || region.processor_restriction == processor)
                .find(|region| rpu_addr >= region.rpu_mem_start && rpu_addr <= region.rpu_mem_end)
                .map(|region| (region, rpu_addr - region.rpu_mem_start))
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub(crate) enum Processor {
    Lmac,
    Umac,
}

/// This structure encapsulates the information which represents a HPQ.
#[repr(C)]
#[derive(Debug, defmt::Format, Clone, Copy)]
pub(crate) struct HostRpuHPQ {
    /// HPQ address where the host can post the address of a
    /// message intended for the RPU.
    enqueue_addr: u32,
    /// HPQ address where the host can get the address of a
    /// message intended for the host.
    dequeue_addr: u32,
}

/// Hostport queue information passed by the RPU to the host, which the host can
/// use, to communicate with the RPU.
#[repr(C)]
#[derive(Debug, defmt::Format, Clone, Copy)]
pub(crate) struct HostRpuHPQMInfo {
    /// Queue which the RPU uses to inform the host about events.
    event_busy_queue: HostRpuHPQ,
    /// Queue on which the consumed events are pushed so that RPU can reuse them.
    event_avl_queue: HostRpuHPQ,
    /// Queue used by the host to push commands to the RPU.
    cmd_busy_queue: HostRpuHPQ,
    /// Queue which RPU uses to inform host about command buffers which can be used to push commands to the RPU.
    cmd_avl_queue: HostRpuHPQ,
    rx_buf_busy_queue: [HostRpuHPQ; c::MAX_NUM_OF_RX_QUEUES as usize],
}

#[derive(Debug, defmt::Format)]
pub(crate) struct RpuInfo {
    hpqm_info: HostRpuHPQMInfo,
    /// The base address for posting RX commands.
    rx_cmd_base: u32,
    /// The base address for posting TX commands.
    tx_cmd_base: u32,
}

/// The RPU, through the bus.
pub(crate) struct Rpu<BUS> {
    pub(crate) bus: BUS,
    /// Where the RPU keeps its queues and command buffers: read once it has booted.
    info: Option<RpuInfo>,
    /// Counts the messages posted, for the interrupt that tells the RPU of them.
    num_commands: u32,
    /// Set when events have been read since the last interrupt acknowledgement.
    pub(crate) irq_pending_ack: bool,
    /// Whether packet RAM reads in one SPI transaction were found to work at boot.
    bulk_pktram_reads: bool,
    /// [`Config::low_power`](crate::Config::low_power): the RPU may sleep while the driver does
    /// not need the bus.
    low_power: bool,
    /// In low power mode, whether the RPU is awake. The driver asks it to stay awake (it does from
    /// the wake-up of the boot on).
    pub(crate) awake: bool,
    /// In low power mode, when the RPU may sleep again: some time after the last bus access.
    pub(crate) idle_at: Instant,
}

impl<BUS: Bus> Rpu<BUS> {
    /// The RPU behind `bus`, before it is turned on.
    pub(crate) fn new(bus: BUS, low_power: bool) -> Self {
        Self {
            bus,
            info: None,
            num_commands: c::RPU_CMD_START_MAGIC,
            irq_pending_ack: false,
            bulk_pktram_reads: false,
            low_power,
            awake: true,
            idle_at: Instant::now(),
        }
    }

    /// Where the RPU keeps its queues and command buffers.
    fn info(&self) -> &RpuInfo {
        unwrap!(self.info.as_ref())
    }

    /// The chip is turned on: the RPU starts from scratch, and awake, as the wake-up of the boot
    /// asks it to stay.
    pub(crate) fn reset(&mut self) {
        self.info = None;
        self.num_commands = c::RPU_CMD_START_MAGIC;
        self.irq_pending_ack = false;
        self.bulk_pktram_reads = false;
        self.awake = true;
    }

    /// Reads the next queued event into `buf`. Returns its length, 0 if it was dropped, or `None`
    /// if the queue is empty.
    pub(crate) async fn event_next(&mut self, buf: &mut [u32]) -> Option<usize> {
        let event_address = self.hpq_dequeue(self.info().hpqm_info.event_busy_queue).await;

        match event_address {
            // No more events to read. Sometimes when low power mode is enabled
            // we see a wrong address, but it work after a while, so, add a
            // check for that.
            None | Some(0xAAAAAAAA) => None,
            Some(event_address) => {
                self.irq_pending_ack = true;
                Some(self.event_read(event_address, buf).await.unwrap_or(0))
            }
        }
    }

    /// Ends an interrupt once the event queue is empty (NCS `hal_rpu_irq_process`): clears a
    /// watchdog interrupt, and acknowledges the interrupt if events were read.
    pub(crate) async fn irq_service_end(&mut self) {
        if self.irq_watchdog_check().await {
            debug!("RPU watchdog interrupt");
            self.irq_watchdog_ack().await;
        }
        if self.irq_pending_ack {
            self.irq_ack().await;
            self.irq_pending_ack = false;
        }
    }

    /// Lets the RPU raise HOST_IRQ (NCS `hal_rpu_irq_enable`).
    pub(crate) async fn irq_enable(&mut self) {
        // First enable the blockwise interrupt for the relevant block in the master register
        let mut val = self.read32(c::RPU_REG_INT_FROM_RPU_CTRL, None).await;

        val |= 1 << c::RPU_REG_BIT_INT_FROM_RPU_CTRL;

        self.write32(c::RPU_REG_INT_FROM_RPU_CTRL, None, val).await;

        // Now enable the relevant MCU interrupt line
        self.write32(
            c::RPU_REG_INT_FROM_MCU_CTRL,
            None,
            1 << c::RPU_REG_BIT_INT_FROM_MCU_CTRL,
        )
        .await;
    }

    /// Acknowledges the RPU's interrupt: HOST_IRQ goes down until the RPU raises it again.
    pub(crate) async fn irq_ack(&mut self) {
        self.write32(c::RPU_REG_INT_FROM_MCU_ACK, None, 1 << c::RPU_REG_BIT_INT_FROM_MCU_ACK)
            .await;
    }

    /// Checks if the watchdog was the source of the interrupt
    pub(crate) async fn irq_watchdog_check(&mut self) -> bool {
        let val = self.read32(c::RPU_REG_MIPS_MCU_UCCP_INT_STATUS, None).await;
        (val & (1 << c::RPU_REG_BIT_MIPS_WATCHDOG_INT_STATUS)) > 0
    }

    /// Clears the watchdog's interrupt.
    pub(crate) async fn irq_watchdog_ack(&mut self) {
        self.write32(
            c::RPU_REG_MIPS_MCU_UCCP_INT_CLEAR,
            None,
            1 << c::RPU_REG_BIT_MIPS_WATCHDOG_INT_CLEAR,
        )
        .await;
    }

    /// Reads the event at `event_address` into `buf` (NCS `hal_rpu_event_get`). An event longer
    /// than one event buffer continues, without a header, in the buffers queued after it. Returns
    /// the event length, or `None` if it did not fit in `buf` and was dropped.
    async fn event_read(&mut self, event_address: u32, buf: &mut [u32]) -> Option<usize> {
        self.read(event_address, None, &mut buf[..EVENT_HEAD / 4]).await;

        // Get the header from the front of the event data
        let message_header: &c::host_rpu_msg_hdr = unsliceit(slice8(buf));
        let len = message_header.len as usize;
        let resubmit = message_header.resubmit > 0;
        let fits = len <= buf.len() * 4;

        let first = len.min(MAX_EVENT_POOL_LEN);
        if first > EVENT_HEAD && fits {
            // The rest of a longer event's first buffer.
            let rest = &mut buf[EVENT_HEAD / 4..first.div_ceil(4)];
            self.read(event_address + EVENT_HEAD as u32, None, rest).await;
        }
        // Hand each buffer back to the RPU only once it has been read.
        if resubmit {
            self.event_free(event_address).await;
        }

        let mut offset = first;
        while offset < len {
            let Some(fragment) = self.next_event_fragment().await else {
                warn!("event of {} bytes lost a fragment", len);
                return None;
            };
            let n = (len - offset).min(MAX_EVENT_POOL_LEN);
            if fits {
                self.read(fragment, None, &mut buf[offset / 4..(offset + n).div_ceil(4)])
                    .await;
            }
            if resubmit {
                self.event_free(fragment).await;
            }
            offset += n;
        }

        if !fits {
            warn!("event of {} bytes dropped", len);
            return None;
        }
        Some(len)
    }

    /// The address of the next fragment of a long event, which the RPU queues right after the first.
    async fn next_event_fragment(&mut self) -> Option<u32> {
        let queue = self.info().hpqm_info.event_busy_queue;
        for _ in 0..100 {
            if let Some(address) = self.hpq_dequeue(queue).await {
                return Some(address);
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        None
    }

    /// Hands the event buffer at `event_address` back to the RPU.
    async fn event_free(&mut self, event_address: u32) {
        self.hpq_enqueue(self.info().hpqm_info.event_avl_queue, event_address)
            .await;
    }

    /// Writes one command buffer (at most [`MAX_CMD_SIZE`] bytes) and hands it to the RPU.
    async fn cmd_ctrl_send(&mut self, message: &[u32]) {
        assert!(message.len() * 4 <= MAX_CMD_SIZE);

        // Wait until we get an address to write to
        // This queue might already be full with other messages, so we'll just have to wait a bit
        let message_address = loop {
            if let Some(message_address) = self.hpq_dequeue(self.info().hpqm_info.cmd_avl_queue).await {
                break message_address;
            }
        };

        // Write the message to the suggested address
        self.write(message_address, None, message).await;

        // Post the updated information to the RPU
        self.hpq_enqueue(self.info().hpqm_info.cmd_busy_queue, message_address)
            .await;

        self.msg_trigger().await;
    }

    /// Posts `value`, an address, to the queue `hpq`.
    async fn hpq_enqueue(&mut self, hpq: HostRpuHPQ, value: u32) {
        self.write32(hpq.enqueue_addr, None, value).await;
    }

    /// Takes the next address from the queue `hpq`, if there is one.
    async fn hpq_dequeue(&mut self, hpq: HostRpuHPQ) -> Option<u32> {
        let value = self.read32(hpq.dequeue_addr, None).await;

        // Pop element only if it is valid
        if value != 0 {
            self.write32(hpq.dequeue_addr, None, value).await;
            Some(value)
        } else {
            None
        }
    }

    /// Reads where the RPU keeps its queues and command buffers, once it has booted (NCS
    /// `wifi_nrf_hal_dev_init`), and finds out whether packet RAM reads in one transaction work.
    pub(crate) async fn init_info(&mut self) {
        let mut hpqm_info = [0u32; size_of::<HostRpuHPQMInfo>() / 4];
        self.read(c::RPU_MEM_HPQ_INFO, None, &mut hpqm_info).await;

        let rx_cmd_base = self.read32(c::RPU_MEM_RX_CMD_BASE, None).await;

        self.info = Some(RpuInfo {
            hpqm_info: unsafe { core::mem::transmute_copy(&hpqm_info) },
            rx_cmd_base,
            tx_cmd_base: c::RPU_MEM_TX_CMD_BASE,
        });
        self.bulk_pktram_reads = self.check_bulk_reads().await;
    }

    /// Checks that packet RAM reads in one SPI transaction return what word by word reads do.
    /// Without the incrementing address bit, a burst returns the first word over and over.
    async fn check_bulk_reads(&mut self) -> bool {
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(c::RPU_MEM_UMAC_BOOT_SIG, None);
        let mut by_word = [0u32; 32];
        let mut bulk = [0u32; 32];
        for (i, val) in by_word.iter_mut().enumerate() {
            *val = self.raw_read32(mem, offs + i as u32 * 4).await;
        }
        self.bus.read((mem.start + offs) | ADDR_INCREMENT, &mut bulk).await;
        let ok = by_word == bulk;
        if ok {
            info!("packet RAM: bulk reads work");
        } else {
            warn!("packet RAM: bulk reads differ, reading word by word");
        }
        ok
    }

    /// Reads the UMAC's copy of the OTP (NCS `nrf_wifi_hal_otp_info_get`,
    /// `nrf_wifi_hal_otp_ft_prog_ver_get` and `nrf_wifi_hal_otp_pack_info_get`).
    pub(crate) async fn read_otp(&mut self) -> Otp {
        const WORDS: usize = size_of::<c::host_rpu_umac_info>().div_ceil(4);
        let mut words = [0u32; WORDS];
        self.read(c::RPU_MEM_UMAC_BOOT_SIG, None, &mut words).await;
        Otp {
            info: unsafe { core::ptr::read_unaligned(words.as_ptr() as *const c::host_rpu_umac_info) },
            flags: self.read32(c::RPU_MEM_OTP_INFO_FLAGS, None).await,
            ft_prog_ver: self.read32(c::RPU_MEM_OTP_FT_PROG_VERSION, None).await,
            package_info: self.read32(c::RPU_MEM_OTP_PACKAGE_TYPE, None).await,
        }
    }

    /// Sends a command to the RPU. It is taken by reference: an async function keeps what it is
    /// given across its awaits, and the caller its own copy too, so a command passed by value
    /// (the authentication one takes 1.7 KB) was held three times in the runner's future.
    pub(crate) async fn send_cmd<T: Command>(&mut self, cmd: &mut T) {
        cmd.fill();
        self.send_message(T::MESSAGE_TYPE, sliceit(cmd)).await;
    }

    /// Sends a message to the RPU: its header, then `body`. A message longer than one command
    /// buffer goes out in fragments, which the RPU reassembles using the total length in the
    /// header (NCS `hal_rpu_cmd_queue`).
    async fn send_message(&mut self, message_type: c::host_rpu_msg_type, body: &[u8]) {
        const HEADER: usize = size_of::<c::host_rpu_msg>();
        let mut header: c::host_rpu_msg = unsafe { zeroed() };
        header.hdr.len = (HEADER + body.len()) as _;
        header.type_ = message_type as _;
        let mut rest = body;
        let mut first = true;
        while first || !rest.is_empty() {
            let mut buf = [0u32; MAX_CMD_SIZE / 4];
            let bytes = slice8_mut(&mut buf);
            let mut len = 0;
            if first {
                bytes[..HEADER].copy_from_slice(sliceit(&header));
                len = HEADER;
                first = false;
            }
            let n = rest.len().min(MAX_CMD_SIZE - len);
            bytes[len..len + n].copy_from_slice(&rest[..n]);
            len += n;
            rest = &rest[n..];
            if with_timeout(Duration::from_secs(1), self.cmd_ctrl_send(&buf[..len.div_ceil(4)]))
                .await
                .is_err()
            {
                panic!("timed out waiting for a free command buffer");
            }
        }
    }

    /// Hands every RX buffer to the RPU.
    pub(crate) async fn init_rx(&mut self) {
        for desc_id in 0..RX_BUFS {
            // The first word of a buffer names it. NCS writes it at each post; the RPU leaves it
            // alone (no buffer of 14,000 frames had it changed), so it is written once.
            self.write32(rx_buf_addr(desc_id), None, desc_id as u32).await;
            self.rx_buf_post(desc_id).await;
        }
    }

    /// Hands the RPU the TX command of token `token` (NCS `hal_rpu_data_cmd_send`): written to the
    /// token's command buffer, then queued.
    pub(crate) async fn send_tx_command(&mut self, token: usize, command: &[u32]) {
        let info = self.info();
        let (base, queue) = (info.tx_cmd_base, info.hpqm_info.cmd_busy_queue);
        let addr = base + c::RPU_DATA_CMD_SIZE_MAX_TX * token as u32;
        self.write(addr, None, command).await;
        self.hpq_enqueue(queue, addr).await;
        self.msg_trigger().await;
    }

    /// Hands RX buffer `desc_id` to the RPU (NCS `wifi_nrf_fmac_rx_cmd_send`). Its command is written
    /// each time: read back directly, the slot of a command the RPU took holds 0xAAAAAAAA.
    pub(crate) async fn rx_buf_post(&mut self, desc_id: usize) {
        let queue_id = desc_id / RX_BUFS_PER_QUEUE;
        let rpu_addr = rx_buf_addr(desc_id);

        // Create host_rpu_rx_buf_info (it's just one word of the address). The RPU takes packet
        // RAM addresses as offsets, as NCS posts them.
        let command = [(rpu_addr + c::RX_BUF_HEADROOM) & c::RPU_ADDR_MASK_OFFSET];

        // Call wifi_nrf_hal_data_cmd_send with the command
        self.rx_cmd_send(&command, desc_id as u32, queue_id).await;
    }

    /// Posts `command`, the RX command of buffer `desc_id`, to the RX queue `pool_id`.
    async fn rx_cmd_send(&mut self, command: &[u32], desc_id: u32, pool_id: usize) {
        let addr_base = self.info().rx_cmd_base;
        let max_cmd_size = c::RPU_DATA_CMD_SIZE_MAX_RX;

        let addr = addr_base + max_cmd_size * desc_id;
        let host_addr = addr & c::RPU_ADDR_MASK_OFFSET | c::RPU_MCU_CORE_INDIRECT_BASE;

        // Write the command to the core. NCS writes it through the current processor, which is
        // the LMAC once the firmware has booted.
        self.write_core(host_addr, command, Processor::Lmac).await;

        // Post the updated information to the RPU
        self.hpq_enqueue(self.info().hpqm_info.rx_buf_busy_queue[pool_id], addr)
            .await;
    }

    /// Tells the RPU that a message was posted.
    async fn msg_trigger(&mut self) {
        // Indicate to the RPU that the information has been posted
        self.write32(
            c::RPU_REG_INT_TO_MCU_CTRL,
            Some(Processor::Umac),
            self.num_commands | 0x7fff0000,
        )
        .await;
        self.num_commands = self.num_commands.wrapping_add(1);
    }

    /// Writes a firmware patch to `processor`'s memory at `rpu_addr`.
    pub(crate) async fn load_fw(&mut self, processor: Processor, rpu_addr: u32, image: &[u8]) {
        const FW_CHUNK_SIZE: usize = 1024;
        let mut buf = [0u32; FW_CHUNK_SIZE / 4];
        for (i, chunk) in image.chunks(FW_CHUNK_SIZE).enumerate() {
            // The images are not word aligned in nrf70.bin, and one has an odd length: copy each
            // chunk into a word buffer, zero padded.
            let words = chunk.len().div_ceil(4);
            buf[words - 1] = 0;
            slice8_mut(&mut buf)[..chunk.len()].copy_from_slice(chunk);
            let addr = rpu_addr + (FW_CHUNK_SIZE * i) as u32;
            self.write(addr, Some(processor), &buf[..words]).await;
        }
    }

    /// Pulses a processor's soft reset and waits for it to park at its boot exception vector
    /// (NCS `nrf_wifi_hal_proc_reset`).
    pub(crate) async fn proc_reset(&mut self, processor: Processor) {
        let (control, status) = match processor {
            Processor::Lmac => (c::RPU_REG_MIPS_MCU_CONTROL, 0xA4000018),
            Processor::Umac => (c::RPU_REG_MIPS_MCU2_CONTROL, 0xA4000118),
        };
        self.write32(control, None, 0x1).await;
        while self.read32(control, None).await & 0x1 != 0 {}
        while self.read32(status, None).await & 0x1 != 1 {}
    }

    /// Starts a processor on its loaded patches and waits for its boot signature (NCS
    /// `nrf_wifi_hal_fw_patch_boot` and `nrf_wifi_hal_fw_chk_boot`).
    pub(crate) async fn proc_boot(&mut self, processor: Processor) {
        let (boot_sig_addr, boot_sig, sleepctrl_addr, patch_offset, control, vectors) = match processor {
            Processor::Lmac => (
                c::RPU_MEM_LMAC_BOOT_SIG,
                c::LMAC_BOOT_SIG,
                c::RPU_REG_UCC_SLEEP_CTRL_DATA_0,
                c::LMAC_ROM_PATCH_OFFSET,
                c::RPU_REG_MIPS_MCU_CONTROL,
                [
                    (c::RPU_REG_MIPS_MCU_BOOT_EXCP_INSTR_0, c::LMAC_BOOT_EXCP_VECT_0),
                    (c::RPU_REG_MIPS_MCU_BOOT_EXCP_INSTR_1, c::LMAC_BOOT_EXCP_VECT_1),
                    (c::RPU_REG_MIPS_MCU_BOOT_EXCP_INSTR_2, c::LMAC_BOOT_EXCP_VECT_2),
                    (c::RPU_REG_MIPS_MCU_BOOT_EXCP_INSTR_3, c::LMAC_BOOT_EXCP_VECT_3),
                ],
            ),
            Processor::Umac => (
                c::RPU_MEM_UMAC_BOOT_SIG,
                c::UMAC_BOOT_SIG,
                c::RPU_REG_UCC_SLEEP_CTRL_DATA_1,
                c::UMAC_ROM_PATCH_OFFSET,
                c::RPU_REG_MIPS_MCU2_CONTROL,
                [
                    (c::RPU_REG_MIPS_MCU2_BOOT_EXCP_INSTR_0, c::UMAC_BOOT_EXCP_VECT_0),
                    (c::RPU_REG_MIPS_MCU2_BOOT_EXCP_INSTR_1, c::UMAC_BOOT_EXCP_VECT_1),
                    (c::RPU_REG_MIPS_MCU2_BOOT_EXCP_INSTR_2, c::UMAC_BOOT_EXCP_VECT_2),
                    (c::RPU_REG_MIPS_MCU2_BOOT_EXCP_INSTR_3, c::UMAC_BOOT_EXCP_VECT_3),
                ],
            ),
        };

        self.write32(boot_sig_addr, None, 0).await;
        // Tells the ROM where the patch starts.
        self.write32(sleepctrl_addr, None, patch_offset).await;
        for (reg, val) in vectors {
            self.write32(reg, None, val).await;
        }
        self.write32(control, None, 0x1).await;

        let booted = with_timeout(FW_BOOT_TIMEOUT, async {
            while self.read32(boot_sig_addr, None).await != boot_sig {
                Timer::after(Duration::from_millis(10)).await;
            }
        })
        .await;
        if booted.is_err() {
            panic!("{} did not boot", processor);
        }
    }

    /// Waits up to 10 ms for the RPU to report itself awake.
    async fn wait_until_awake(&mut self) {
        for _ in 0..10 {
            if self.bus.read_sr1().await & SR1_RPU_AWAKE != 0 {
                return;
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        panic!("awakening never came")
    }

    /// Waits up to 10 ms for the wake-up request to show.
    async fn wait_until_wakeup_req(&mut self) {
        for _ in 0..10 {
            if self.bus.read_sr2().await == SR2_RPU_WAKEUP_REQ {
                return;
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        panic!("wakeup_req never came")
    }

    /// Wakes the RPU up, at power on.
    pub(crate) async fn wakeup(&mut self) {
        self.bus.write_sr2(SR2_RPU_WAKEUP_REQ).await;
        self.wait_until_wakeup_req().await;
        self.wait_until_awake().await;
    }

    /// Makes sure the RPU is awake before a bus access, in low power mode (NCS
    /// `hal_rpu_ps_wake`). Without it the wake-up request of the boot stays for good.
    async fn ps_wake(&mut self) {
        if !self.low_power {
            return;
        }
        if !self.awake {
            const AWAKE: u8 = SR1_RPU_AWAKE | SR1_RPU_READY;
            self.bus.write_sr2(SR2_RPU_WAKEUP_REQ).await;
            // NCS waits before it reads the state, "to avoid a race condition in the RPU".
            Timer::after(Duration::from_millis(1)).await;
            let deadline = Instant::now() + RPU_WAKE_TIMEOUT;
            while self.bus.read_sr1().await & AWAKE != AWAKE {
                if Instant::now() >= deadline {
                    panic!("the RPU did not wake up");
                }
                Timer::after(Duration::from_millis(1)).await;
            }
            self.awake = true;
        }
        self.idle_at = Instant::now() + RPU_IDLE_TIMEOUT;
    }

    /// Lets the RPU sleep once the bus has been idle for long enough, in low power mode (NCS
    /// `hal_rpu_ps_sleep`).
    pub(crate) async fn ps_sleep(&mut self) {
        if self.low_power && self.awake && Instant::now() >= self.idle_at {
            self.bus.write_sr2(0).await;
            self.awake = false;
        }
    }

    /// Reads the word at `offs` in `mem`, after the region's dummy words.
    async fn raw_read32(&mut self, mem: &MemoryRegion, offs: u32) -> u32 {
        assert!(mem.start + offs + 4 <= mem.end);
        let lat = mem.latency as usize;

        self.ps_wake().await;
        let mut buf = [0u32; 3];
        self.bus.read(mem.start + offs, &mut buf[..lat + 1]).await;
        trace!("read32 {:08x} {:08x}", mem.start + offs, buf[lat]);
        buf[lat]
    }

    /// Reads `buf.len()` words from `offs` in `mem`: in one transaction from packet RAM, where
    /// the address can increment, else word by word.
    async fn raw_read(&mut self, mem: &MemoryRegion, offs: u32, buf: &mut [u32]) {
        assert!(mem.start + offs + (buf.len() as u32 * 4) <= mem.end);

        if mem.latency == 0 && self.bulk_pktram_reads {
            // One transaction with the address incrementing, as NCS reads packet RAM.
            self.ps_wake().await;
            self.bus.read((mem.start + offs) | ADDR_INCREMENT, buf).await;
        } else {
            for (i, val) in buf.iter_mut().enumerate() {
                *val = self.raw_read32(mem, offs + i as u32 * 4).await;
            }
        }
        trace!(
            "read addr={:08x} len={:08x} buf={:02x}",
            mem.start + offs,
            buf.len() * 4,
            slice8(buf)
        );
    }

    /// Writes `val` at `offs` in `mem`.
    pub(crate) async fn raw_write32(&mut self, mem: &MemoryRegion, offs: u32, val: u32) {
        self.raw_write(mem, offs, &[val]).await
    }

    /// Writes `buf` at `offs` in `mem`, in one transaction.
    async fn raw_write(&mut self, mem: &MemoryRegion, offs: u32, buf: &[u32]) {
        assert!(mem.start + offs + (buf.len() as u32 * 4) <= mem.end);
        trace!(
            "write addr={:08x} len={:08x} buf={:02x}",
            mem.start + offs,
            buf.len() * 4,
            slice8(buf)
        );
        self.ps_wake().await;
        self.bus.write(mem.start + offs, buf).await;
    }

    /// Reads a word at `rpu_addr`, as `processor` sees it (`None`: an address every processor
    /// sees the same).
    pub(crate) async fn read32(&mut self, rpu_addr: u32, processor: Option<Processor>) -> u32 {
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(rpu_addr, processor);
        self.raw_read32(mem, offs).await
    }

    /// Reads `buf.len()` words from `rpu_addr`, as `processor` sees it.
    pub(crate) async fn read(&mut self, rpu_addr: u32, processor: Option<Processor>, buf: &mut [u32]) {
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(rpu_addr, processor);
        self.raw_read(mem, offs, buf).await
    }

    /// Writes `val` at `rpu_addr`, as `processor` sees it.
    pub(crate) async fn write32(&mut self, rpu_addr: u32, processor: Option<Processor>, val: u32) {
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(rpu_addr, processor);
        self.raw_write32(mem, offs, val).await
    }

    /// Writes `buf` at `rpu_addr`, as `processor` sees it.
    pub(crate) async fn write(&mut self, rpu_addr: u32, processor: Option<Processor>, buf: &[u32]) {
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(rpu_addr, processor);
        self.raw_write(mem, offs, buf).await
    }

    /// Writes `buf` into `processor`'s memory at `core_address`, through its indirect access
    /// registers, one word at a time.
    async fn write_core(&mut self, core_address: u32, buf: &[u32], processor: Processor) {
        // We receive the address as a byte address, while we need to write it as a word address
        let addr = (core_address & c::RPU_ADDR_MASK_OFFSET) / 4;

        let (addr_reg, data_reg) = match processor {
            Processor::Lmac => (
                c::RPU_REG_MIPS_MCU_SYS_CORE_MEM_CTRL,
                c::RPU_REG_MIPS_MCU_SYS_CORE_MEM_WDATA,
            ),
            Processor::Umac => (
                c::RPU_REG_MIPS_MCU2_SYS_CORE_MEM_CTRL,
                c::RPU_REG_MIPS_MCU2_SYS_CORE_MEM_WDATA,
            ),
        };

        // Write the processor address register
        self.write32(addr_reg, Some(processor), addr).await;

        // Write to the data register one by one
        for data in buf {
            self.write32(data_reg, Some(processor), *data).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use core::assert_eq;

    use super::*;
    use regions::GRAM;

    #[test]
    fn gram_is_read_with_two_dummy_words() {
        let (mem, offs) = regions::remap_global_addr_to_region_and_offset(c::RPU_MEM_LMAC_BOOT_SIG, None);
        assert_eq!((mem.start, mem.latency, offs), (GRAM.start, 2, 0xD50));
    }
}
