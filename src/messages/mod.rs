use core::mem::size_of;

pub mod commands;

pub trait Message {
    const MESSAGE_TYPE: RpuMessageType;
}

/// This structure encapsulates the common information included at the start of
/// each command/event exchanged with the RPU.
#[repr(C)]
#[derive(Debug, defmt::Format, Clone, Copy)]
pub(crate) struct RpuMessageHeader {
    /// Length of the message.
    pub length: u32,
    /// Flag to indicate whether the recipient is expected to resubmit
    /// the cmd/event address back to the trasmitting entity.
    pub resubmit: u32,
}

#[repr(C, packed)]
pub(crate) struct RpuMessage<M: Message> {
    pub header: RpuMessageHeader,
    pub message_type: RpuMessageType,
    pub message: M,
}

impl<M: Message> RpuMessage<M> {
    pub const fn new(message: M) -> Self {
        Self {
            header: RpuMessageHeader {
                length: size_of::<Self>() as u32,
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
