use core::mem::size_of;

use crate::c;

pub mod commands;

pub trait Message {
    const MESSAGE_TYPE: RpuMessageType;
}

#[repr(C, packed)]
pub(crate) struct RpuMessage<M: Message> {
    pub header: c::host_rpu_msg_hdr,
    pub message_type: RpuMessageType,
    pub message: M,
}

impl<M: Message> RpuMessage<M> {
    pub const fn new(message: M) -> Self {
        Self {
            header: c::host_rpu_msg_hdr {
                len: size_of::<Self>() as u32,
                resubmit: 0,
            },
            message_type: M::MESSAGE_TYPE,
            message,
        }
    }
}

#[repr(i32)]
pub enum RpuMessageType {
    System,
    Supplicant,
    Data,
    Umac,
}
