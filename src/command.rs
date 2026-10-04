//! The commands the driver sends the RPU: each is a bindings struct that starts with the header of
//! its message type, which [`Command::fill`] writes before [`Rpu::send_cmd`](crate::rpu::Rpu::send_cmd)
//! sends it.

use core::mem::{size_of, zeroed};

use crate::c;

/// A command struct of the bindings, and how its header is filled in.
pub(crate) trait Command {
    /// The type of the message the command goes in.
    const MESSAGE_TYPE: c::host_rpu_msg_type;

    /// Writes the command's header: its number and, for a system command, its length.
    fn fill(&mut self);
}

macro_rules! impl_cmd {
    (sys: $($cmd:ident => $num:ident),* $(,)?) => {$(
        impl Command for c::$cmd {
            const MESSAGE_TYPE: c::host_rpu_msg_type = c::host_rpu_msg_type::HOST_RPU_MSG_TYPE_SYSTEM;
            fn fill(&mut self) {
                self.sys_head = c::sys_head {
                    cmd_event: c::sys_commands::$num as _,
                    len: size_of::<Self>() as _,
                };
            }
        }
    )*};
    (umac: $($cmd:ident => $num:ident),* $(,)?) => {$(
        impl Command for c::$cmd {
            const MESSAGE_TYPE: c::host_rpu_msg_type = c::host_rpu_msg_type::HOST_RPU_MSG_TYPE_UMAC;
            fn fill(&mut self) {
                // Every UMAC command addresses the default interface, wdev 0.
                self.umac_hdr = c::umac_hdr {
                    cmd_evnt: c::umac_commands::$num as _,
                    ids: c::index_ids {
                        valid_fields: c::INDEX_IDS_WDEV_ID_VALID,
                        wdev_id: 0,
                        ..unsafe { zeroed() }
                    },
                    ..unsafe { zeroed() }
                };
            }
        }
    )*};
}

impl_cmd!(sys: cmd_sys_init => CMD_INIT);

impl_cmd!(umac:
    umac_cmd_change_macaddr => UMAC_CMD_CHANGE_MACADDR,
    umac_cmd_chg_vif_state => UMAC_CMD_SET_IFFLAGS,
    umac_cmd_scan => UMAC_CMD_TRIGGER_SCAN,
    umac_cmd_get_scan_results => UMAC_CMD_GET_SCAN_RESULTS,
    umac_cmd_auth => UMAC_CMD_AUTHENTICATE,
    umac_cmd_assoc => UMAC_CMD_ASSOCIATE,
    umac_cmd_disconn => UMAC_CMD_DEAUTHENTICATE,
    umac_cmd_chg_sta => UMAC_CMD_SET_STATION,
    umac_cmd_get_sta => UMAC_CMD_GET_STATION,
    umac_cmd_key => UMAC_CMD_NEW_KEY,
    umac_cmd_set_key => UMAC_CMD_SET_KEY,
    umac_cmd_set_power_save => UMAC_CMD_SET_POWER_SAVE,
    umac_cmd_get_power_save_info => UMAC_CMD_GET_POWER_SAVE_INFO,
);

#[cfg(feature = "ap")]
impl_cmd!(umac:
    umac_cmd_chg_vif_attr => UMAC_CMD_SET_INTERFACE,
    umac_cmd_set_wiphy => UMAC_CMD_SET_WIPHY,
    umac_cmd_mgmt_frame_reg => UMAC_CMD_REGISTER_FRAME,
    umac_cmd_mgmt_tx => UMAC_CMD_FRAME,
    umac_cmd_start_ap => UMAC_CMD_START_AP,
    umac_cmd_stop_ap => UMAC_CMD_STOP_AP,
    umac_cmd_set_bss => UMAC_CMD_SET_BSS,
    umac_cmd_add_sta => UMAC_CMD_NEW_STATION,
    umac_cmd_del_sta => UMAC_CMD_DEL_STATION,
);
