use crate::c;
use crate::messages::{Message, RpuMessageType};

impl Message for c::cmd_sys_init {
    const MESSAGE_TYPE: RpuMessageType = RpuMessageType::System;
}
