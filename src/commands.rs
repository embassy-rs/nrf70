use crate::c;
use core::mem::{size_of, zeroed};

pub trait Command {
    const MESSAGE_TYPE: c::host_rpu_msg_type;
    fn fill(&mut self);
}

macro_rules! impl_cmd {
    (sys, $cmd:path, $num:expr) => {
        impl Command for $cmd {
            const MESSAGE_TYPE: c::host_rpu_msg_type = c::host_rpu_msg_type::HOST_RPU_MSG_TYPE_SYSTEM;
            fn fill(&mut self) {
                self.sys_head = c::sys_head {
                    cmd_event: $num as _,
                    len: size_of::<Self>() as _,
                };
            }
        }
    };
    (umac, $cmd:path, $num:expr) => {
        impl Command for $cmd {
            const MESSAGE_TYPE: c::host_rpu_msg_type = c::host_rpu_msg_type::HOST_RPU_MSG_TYPE_UMAC;
            fn fill(&mut self) {
                self.umac_hdr = c::umac_hdr {
                    cmd_evnt: $num as _,
                    ..unsafe { zeroed() }
                };
            }
        }
    };
}

impl_cmd!(sys, c::cmd_sys_init, c::sys_commands::CMD_INIT);
impl_cmd!(sys, c::cmd_get_stats, c::sys_commands::CMD_GET_STATS);
impl_cmd!(sys, c::cmd_sys_deinit, c::sys_commands::CMD_DEINIT);

impl_cmd!(
    umac,
    c::umac_cmd_change_macaddr,
    c::umac_commands::UMAC_CMD_CHANGE_MACADDR
);
impl_cmd!(umac, c::umac_cmd_chg_vif_state, c::umac_commands::UMAC_CMD_SET_IFFLAGS);
impl_cmd!(umac, c::umac_cmd_scan, c::umac_commands::UMAC_CMD_TRIGGER_SCAN);
impl_cmd!(umac, c::umac_cmd_abort_scan, c::umac_commands::UMAC_CMD_ABORT_SCAN);
impl_cmd!(
    umac,
    c::umac_cmd_get_scan_results,
    c::umac_commands::UMAC_CMD_GET_SCAN_RESULTS
);
