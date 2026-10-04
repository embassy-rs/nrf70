//! The data path: frames from embassy-net to the RPU, batched under TX tokens, and received frames,
//! in the 802.11 or A-MSDU form the RPU hands them over, to embassy-net or to the key handshakes.

use core::mem::zeroed;

use defmt::{debug, warn};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

use crate::rpu::{rx_buf_addr, tx_buf_addr, MAX_TX_AGGREGATION, MAX_TX_TOKENS, RX_BUFS, RX_MAX_DATA_SIZE};
use crate::{c, slice8, slice8_mut, sliceit, unsliceit, unsliceit2, Bus, Runner, MTU};
#[cfg(feature = "wpa2")]
use crate::{eapol::ETHERTYPE_EAPOL, wpa2};

/// Frame control bits of an 802.11 header.
const FC_TO_DS: u16 = 0x0100;
const FC_FROM_DS: u16 = 0x0200;

/// How many bytes to skip after an 802.11 data header to reach the payload: the LLC/SNAP header and
/// the ethertype (NCS `nrf_wifi_util_get_skip_header_bytes`).
fn llc_skip(ethertype: u16) -> usize {
    const AARP: u16 = 0x80F3;
    const IPX: u16 = 0x8137;
    2 + match ethertype {
        AARP | IPX => 8,
        e if e >= 0x0600 => 6,
        _ => 0,
    }
}

/// The ethertype in an LLC/SNAP header.
fn llc_ethertype(llc: &[u8]) -> Option<u16> {
    Some(u16::from_be_bytes(llc.get(6..8)?.try_into().ok()?))
}

/// Writes an Ethernet II frame (`dst`, `src`, ethertype or length, payload) into `out`. Returns its
/// length, or `None` if it does not fit.
pub(crate) fn write_ethernet(out: &mut [u8], dst: &[u8], src: &[u8], ethertype: u16, payload: &[u8]) -> Option<usize> {
    let len = 14 + payload.len();
    let out = out.get_mut(..len)?;
    out[0..6].copy_from_slice(dst);
    out[6..12].copy_from_slice(src);
    let type_or_len = if ethertype >= 0x0600 {
        ethertype
    } else {
        payload.len() as u16
    };
    out[12..14].copy_from_slice(&type_or_len.to_be_bytes());
    out[14..].copy_from_slice(payload);
    Some(len)
}

/// A received data frame, as the RPU hands it over, seen as the Ethernet frame it carries.
pub(crate) struct RxFrame<'f> {
    pub(crate) dst: [u8; 6],
    pub(crate) src: [u8; 6],
    /// The ethertype, or the length of an 802.3 frame.
    pub(crate) ethertype: u16,
    pub(crate) payload: &'f [u8],
}

impl<'f> RxFrame<'f> {
    /// Reads `frame` (NCS `nrf_wifi_fmac_rx_event_process` and the conversions it calls).
    /// `pkt_type` is one of the `PKT_TYPE_*` values and `mac_header_len` the 802.11 header length
    /// the RPU reports.
    fn parse(frame: &'f [u8], pkt_type: u32, mac_header_len: usize) -> Option<Self> {
        let mac = |at: usize| -> Option<[u8; 6]> { frame.get(at..at + 6)?.try_into().ok() };
        let (dst, src, body) = match pkt_type {
            c::PKT_TYPE_MPDU => {
                let fc = u16::from_le_bytes(frame.get(0..2)?.try_into().ok()?);
                let addr = |n: usize| mac(4 + 6 * n);
                let (dst, src) = match fc & (FC_TO_DS | FC_FROM_DS) {
                    FC_FROM_DS => (addr(0)?, addr(2)?),
                    FC_TO_DS => (addr(2)?, addr(1)?),
                    0 => (addr(0)?, addr(1)?),
                    _ => (addr(0)?, mac(24)?),
                };
                (dst, src, frame.get(mac_header_len..)?)
            }
            c::PKT_TYPE_MSDU_WITH_MAC => return Self::parse(frame.get(mac_header_len..)?, c::PKT_TYPE_MSDU, 0),
            c::PKT_TYPE_MSDU => {
                // An A-MSDU subframe: destination, source and length, then the LLC/SNAP header.
                let length = u16::from_be_bytes(frame.get(12..14)?.try_into().ok()?) as usize;
                (mac(0)?, mac(6)?, frame.get(14..(14 + length).min(frame.len()))?)
            }
            _ => return None,
        };
        let ethertype = llc_ethertype(body)?;
        Some(Self {
            dst,
            src,
            ethertype,
            payload: body.get(llc_skip(ethertype)..)?,
        })
    }

    /// Writes the frame as an Ethernet II frame into `out`. Returns its length, or `None` if it
    /// does not fit.
    fn write_ethernet(&self, out: &mut [u8]) -> Option<usize> {
        write_ethernet(out, &self.dst, &self.src, self.ethertype, self.payload)
    }
}

/// A key handshake frame, which `rx_frame` returns copied out of the frame's buffer. Never one
/// without the `wpa2` feature.
#[cfg(feature = "wpa2")]
pub(crate) type RxEapol = wpa2::RxEapol;

#[cfg(not(feature = "wpa2"))]
pub(crate) type RxEapol = core::convert::Infallible;

/// Shortest frame that shares a TX token with others. Large frames (a TCP sender's data) go in
/// batches: one command and one TX done event for several frames took the DK's TCP sends from 13.3
/// to 13.9 Mbit/s, and its runner from 44% of the CPU to 14 to 22%. Small ones (acknowledgements)
/// go one by one: batched, they reached the sender in bursts, and transfers to the DK fell from
/// 12.5 to 11.8 Mbit/s.
const TX_BATCH_FRAME_MIN: usize = 256;

/// Frames that go out under one TX token, written to its buffers, waiting for their command.
struct TxBatch {
    token: usize,
    /// The Ethernet header (destination, source, ethertype) the frames share.
    header: [u8; 14],
    /// Their 802.1D priority.
    priority: u32,
    /// The lengths of the frames, in the order of the token's buffers.
    lens: [u16; MAX_TX_AGGREGATION],
    count: usize,
}

impl TxBatch {
    /// Whether `frame` may join the batch: a large frame like the batch's, with the same Ethernet
    /// header and priority, and a buffer left.
    fn takes(&self, frame: &[u8]) -> bool {
        self.count < MAX_TX_AGGREGATION
            && self.lens[0] as usize >= TX_BATCH_FRAME_MIN
            && frame.len() >= TX_BATCH_FRAME_MIN
            && frame.get(..14) == Some(&self.header[..])
            && tx_priority(frame) == self.priority
    }
}

/// The 802.1D priority of an outgoing Ethernet frame, from the precedence bits of an IPv4 header
/// (NCS `get_tid`, which also looks at IPv6 and VLAN tags; everything else goes as best effort).
pub(crate) fn tx_priority(frame: &[u8]) -> u32 {
    const IPV4: [u8; 2] = [0x08, 0x00];
    match frame.get(12..16) {
        Some([t0, t1, _, tos]) if [*t0, *t1] == IPV4 => ((tos & 0xFC) >> 5) as u32,
        _ => 0,
    }
}

impl<'a, BUS: Bus, IN: InputPin + Wait, OUT: OutputPin> Runner<'a, BUS, IN, OUT> {
    /// Hands a frame to the RPU, alone under a TX token (NCS `nrf_wifi_fmac_start_xmit`). The RPU
    /// builds the 802.11 header from the Ethernet one. Returns the token, or `None` if the frame was
    /// not sent: it is shorter than an Ethernet header, or no token is free.
    #[cfg_attr(not(any(feature = "wpa2", feature = "ap")), allow(dead_code))]
    pub(crate) async fn send_frame(&mut self, frame: &[u32], len: usize) -> Option<usize> {
        self.send_frame_flags(frame, len, false, true).await
    }

    /// [`Self::send_frame`], with the power save bits of a frame for a station of the access
    /// point: `more_data` says more frames are kept for it, `eosp` ends its service period.
    #[cfg_attr(not(any(feature = "wpa2", feature = "ap")), allow(dead_code))]
    pub(crate) async fn send_frame_flags(
        &mut self,
        frame: &[u32],
        len: usize,
        more_data: bool,
        eosp: bool,
    ) -> Option<usize> {
        let batch = self.tx_batch_start(frame, len).await?;
        let token = batch.token;
        self.tx_batch_send(batch, more_data, eosp).await;
        Some(token)
    }

    /// Sends a frame from embassy-net, `len` bytes of `tx_frame`, and with it the frames queued
    /// behind it that share its Ethernet header, if they are large: up to [`MAX_TX_AGGREGATION`] go
    /// under one TX token, with one command and one TX done event for all of them (NCS
    /// `tx_cmd_prepare`).
    pub(crate) async fn send_queued_frames(&mut self, tx_frame: &mut [u32; MTU.div_ceil(4)], len: usize) {
        if self.ap_hold(tx_frame, len).await {
            return;
        }
        let Some(mut batch) = self.tx_batch_start(tx_frame, len).await else {
            return;
        };
        while let Some(frame) = self.ch.try_tx_buf().filter(|frame| batch.takes(frame)) {
            let len = frame.len();
            slice8_mut(tx_frame)[..len].copy_from_slice(frame);
            self.ch.tx_done();
            if !self.ap_hold(tx_frame, len).await {
                self.tx_batch_add(&mut batch, tx_frame, len).await;
            }
        }
        self.tx_batch_send(batch, false, true).await;
    }

    /// Starts a batch with `len` bytes of `frame`: takes a TX token and writes the frame to its
    /// first buffer. `None` if the frame is shorter than an Ethernet header or no token is free.
    async fn tx_batch_start(&mut self, frame: &[u32], len: usize) -> Option<TxBatch> {
        let bytes = &slice8(frame)[..len];
        let token = self.tx_tokens_busy.trailing_ones() as usize;
        let header = bytes.get(..14)?.try_into().ok()?;
        if token >= MAX_TX_TOKENS {
            return None;
        }
        self.tx_tokens_busy |= 1 << token;
        self.ap_frame_queued(token, &bytes[..6]);
        let mut batch = TxBatch {
            token,
            header,
            priority: tx_priority(bytes),
            lens: [0; MAX_TX_AGGREGATION],
            count: 0,
        };
        self.tx_batch_add(&mut batch, frame, len).await;
        Some(batch)
    }

    /// Writes `len` bytes of `frame` to the next buffer of `batch`'s token. The caller made sure
    /// that the batch [`takes`](TxBatch::takes) it.
    async fn tx_batch_add(&mut self, batch: &mut TxBatch, frame: &[u32], len: usize) {
        let addr = tx_buf_addr(batch.token, batch.count);
        self.rpu.write(addr, None, &frame[..len.div_ceil(4)]).await;
        batch.lens[batch.count] = len as u16;
        batch.count += 1;
    }

    /// Hands `batch` to the RPU in one command, with the power save bits of [`Self::send_frame_flags`].
    async fn tx_batch_send(&mut self, batch: TxBatch, more_data: bool, eosp: bool) {
        #[repr(C, packed)]
        struct Head {
            msg: c::host_rpu_msg,
            buff: c::tx_buff,
        }
        const HEAD: usize = size_of::<Head>();
        const INFO: usize = size_of::<c::tx_buff_info>();
        let len = HEAD + batch.count * INFO;
        let mut head: Head = unsafe { zeroed() };
        head.msg.hdr.len = len as u32;
        head.msg.type_ = c::host_rpu_msg_type::HOST_RPU_MSG_TYPE_DATA as _;
        head.buff.umac_head = c::umac_head {
            cmd: c::umac_data_commands::CMD_TX_BUFF as u32,
            len: (size_of::<c::tx_buff>() + batch.count * INFO) as u32,
        };
        head.buff.tx_desc_num = batch.token as u8;
        let header = &mut head.buff.mac_hdr_info;
        header.dest.copy_from_slice(&batch.header[0..6]);
        header.src.copy_from_slice(&batch.header[6..12]);
        header.etype = u16::from_be_bytes([batch.header[12], batch.header[13]]);
        header.tx_flags = batch.priority;
        header.more_data = more_data as u8;
        header.eosp = eosp as u8;
        head.buff.num_tx_pkts = batch.count as u8;

        let mut words = [0u32; (HEAD + MAX_TX_AGGREGATION * INFO).div_ceil(4)];
        let bytes = slice8_mut(&mut words);
        bytes[..HEAD].copy_from_slice(sliceit(&head));
        for (i, &pkt_length) in batch.lens[..batch.count].iter().enumerate() {
            let info = c::tx_buff_info {
                pkt_length,
                // The RPU takes packet RAM addresses as offsets.
                ddr_ptr: tx_buf_addr(batch.token, i) & c::RPU_ADDR_MASK_OFFSET,
            };
            bytes[HEAD + i * INFO..][..INFO].copy_from_slice(sliceit(&info));
        }
        self.rpu.send_tx_command(batch.token, &words[..len.div_ceil(4)]).await;
    }

    /// Passes received data frames to embassy-net and gives their buffers back (NCS
    /// `nrf_wifi_fmac_rx_event_process`).
    pub(crate) async fn handle_rx(&mut self, body: &[u8]) {
        let (rx, infos) = unsliceit2::<c::rx_buff>(body);
        let class = rx.rx_pkt_type as i32;
        let mac_header_len = rx.mac_header_len as usize;
        for i in 0..rx.rx_pkt_cnt as usize {
            let Some(info) = infos.get(i * size_of::<c::rx_buff_info>()..) else {
                break;
            };
            let info: &c::rx_buff_info = unsliceit(info);
            let (desc_id, len, pkt_type) = (
                info.descriptor_id as usize,
                info.rx_pkt_len as usize,
                info.pkt_type as u32,
            );
            if desc_id >= RX_BUFS {
                warn!("RX event for invalid descriptor {}", desc_id);
                continue;
            }
            if class == c::rx_pkt_type::RX_PKT_DATA as i32 {
                self.rx_deliver(desc_id, len, pkt_type, mac_header_len).await;
                self.rpu.rx_buf_post(desc_id).await;
            } else if class != c::rx_pkt_type::RX_PKT_BCN_PRB_RSP as i32 {
                // The UMAC refills beacon and probe response buffers itself.
                warn!("RX packet class {} not handled", class);
            }
        }
    }

    async fn rx_deliver(&mut self, desc_id: usize, len: usize, pkt_type: u32, mac_header_len: usize) {
        // A key handshake frame comes back copied, and is handled once the frame's buffer (1.6 KB
        // of the runner's future) is gone: its handshake can go as deep as the next join.
        if let Some(eapol) = self.rx_frame(desc_id, len, pkt_type, mac_header_len).await {
            self.rx_eapol(eapol).await;
        }
    }

    /// Reads a received frame and hands it to embassy-net, or returns it if it is a key handshake
    /// frame.
    async fn rx_frame(&mut self, desc_id: usize, len: usize, pkt_type: u32, mac_header_len: usize) -> Option<RxEapol> {
        if len > RX_MAX_DATA_SIZE {
            warn!("RX frame of {} bytes dropped", len);
            return None;
        }
        let mut frame = [0u32; RX_MAX_DATA_SIZE / 4];
        let words = len.div_ceil(4);
        self.rpu
            .read(rx_buf_addr(desc_id) + c::RX_BUF_HEADROOM, None, &mut frame[..words])
            .await;
        let Some(rx) = RxFrame::parse(&slice8(&frame)[..len], pkt_type, mac_header_len) else {
            warn!("RX frame of type {} not converted", pkt_type);
            return None;
        };
        self.ap_seen(&rx.src);
        #[cfg(feature = "wpa2")]
        if rx.ethertype == ETHERTYPE_EAPOL {
            return wpa2::RxEapol::copy(&rx);
        }
        let Some(out) = self.ch.try_rx_buf() else {
            debug!("RX frame dropped, embassy-net has no buffer free");
            return None;
        };
        match rx.write_ethernet(out) {
            Some(n) => self.ch.rx_done(n),
            None => warn!("RX frame of {} bytes too long for embassy-net", len),
        }
        None
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::assert_eq;
    use std::vec;
    use std::vec::Vec;

    use defmt::unwrap;

    use super::*;

    /// An 802.11 data frame from the AP (FromDS) with an RFC 1042 LLC/SNAP header and an IPv4
    /// payload.
    fn from_ds_frame(payload: &[u8]) -> Vec<u8> {
        let mut frame = vec![
            0x08, 0x02, // frame control: data, FromDS
            0x00, 0x00, // duration
            1, 1, 1, 1, 1, 1, // addr1: our MAC, the destination
            2, 2, 2, 2, 2, 2, // addr2: BSSID
            3, 3, 3, 3, 3, 3, // addr3: the source
            0x00, 0x00, // sequence control
            0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, // LLC/SNAP (RFC 1042)
            0x08, 0x00, // IPv4
        ];
        frame.extend_from_slice(payload);
        frame
    }

    fn to_ethernet(frame: &[u8], pkt_type: u32, mac_header_len: usize, out: &mut [u8]) -> Option<usize> {
        RxFrame::parse(frame, pkt_type, mac_header_len)?.write_ethernet(out)
    }

    #[test]
    fn the_ethertype_is_read_in_each_kind_of_frame() {
        const EAPOL: u16 = 0x888E;
        let ethertype = |frame: &[u8], pkt_type, mac_header_len| {
            RxFrame::parse(frame, pkt_type, mac_header_len).map(|rx| rx.ethertype)
        };
        let mut mpdu = from_ds_frame(&[1, 3, 0, 95]);
        assert_eq!(ethertype(&mpdu, c::PKT_TYPE_MPDU, 24), Some(0x0800));
        mpdu[30..32].copy_from_slice(&EAPOL.to_be_bytes());
        assert_eq!(ethertype(&mpdu, c::PKT_TYPE_MPDU, 24), Some(EAPOL));

        let mut subframe = vec![1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 0, 12];
        subframe.extend_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E, 1, 3, 0, 95]);
        assert_eq!(ethertype(&subframe, c::PKT_TYPE_MSDU, 0), Some(EAPOL));
        let mut with_mac = vec![0; 24];
        with_mac.extend_from_slice(&subframe);
        assert_eq!(ethertype(&with_mac, c::PKT_TYPE_MSDU_WITH_MAC, 24), Some(EAPOL));
        assert_eq!(
            RxFrame::parse(&with_mac, c::PKT_TYPE_MSDU_WITH_MAC, 24).map(|rx| rx.src),
            Some([3; 6])
        );

        // Cut before the ethertype, and a packet type the RPU does not use.
        assert_eq!(ethertype(&mpdu[..31], c::PKT_TYPE_MPDU, 24), None);
        assert_eq!(ethertype(&mpdu, 7, 24), None);
    }

    #[test]
    fn mpdu_from_the_ap_becomes_ethernet() {
        let frame = from_ds_frame(&[0x45, 0xAB, 0xCD]);
        let mut out = [0u8; 64];
        let len = to_ethernet(&frame, c::PKT_TYPE_MPDU, 24, &mut out);
        assert_eq!(
            &out[..unwrap!(len)],
            [1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 0x08, 0x00, 0x45, 0xAB, 0xCD]
        );
    }

    #[test]
    fn amsdu_subframe_becomes_ethernet_without_its_padding() {
        let mut subframe = vec![1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3];
        subframe.extend_from_slice(&11u16.to_be_bytes()); // LLC/SNAP + ethertype + 3 bytes
        subframe.extend_from_slice(&[0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x08, 0x06, 7, 8, 9]);
        subframe.extend_from_slice(&[0, 0, 0]); // padding to a multiple of 4
        let mut out = [0u8; 64];
        let len = to_ethernet(&subframe, c::PKT_TYPE_MSDU, 0, &mut out);
        assert_eq!(
            &out[..unwrap!(len)],
            [1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 0x08, 0x06, 7, 8, 9]
        );
    }

    #[test]
    fn rx_frame_too_big_for_the_buffer_is_refused() {
        let frame = from_ds_frame(&[0; 100]);
        let mut out = [0u8; 64];
        assert_eq!(to_ethernet(&frame, c::PKT_TYPE_MPDU, 24, &mut out), None);
    }

    #[test]
    fn tx_priority_comes_from_the_ipv4_precedence() {
        let mut frame = [0u8; 20];
        frame[12..14].copy_from_slice(&[0x08, 0x00]);
        frame[15] = 0xB8; // DSCP 46 (expedited forwarding): precedence 5
        assert_eq!(tx_priority(&frame), 5);
        frame[12..14].copy_from_slice(&[0x08, 0x06]); // ARP
        assert_eq!(tx_priority(&frame), 0);
    }
}
