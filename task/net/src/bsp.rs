// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

// These modules are exported so that we don't have warnings about unused code,
// but you should import Bsp instead, which is autoselected based on board.

cfg_if::cfg_if! {
    if #[cfg(any(target_board = "nucleo-h743zi2", target_board = "nucleo-h753zi"))] {
        mod nucleo_h7;
        pub use nucleo_h7::*;
    } else if #[cfg(target_board = "sidecar-a")] {
        mod sidecar_a;
        pub use sidecar_a::*;
    } else if #[cfg(any(target_board="gimlet-b"))] {
        mod gimlet_b;
        pub use gimlet_b::*;
    } else if #[cfg(target_board = "psc-a")] {
        mod psc_a;
        pub use psc_a::*;
    } else if #[cfg(target_board = "gimletlet-1")] {
        mod gimletlet_mgmt;
        pub use gimletlet_mgmt::*;
    } else if #[cfg(all(feature = "gimletlet-nic",
                        target_board = "gimletlet-2"))] {
        mod gimletlet_nic;
        pub use gimletlet_nic::*;
    } else {
        compile_error!("Board is not supported by the task/net");
    }
}
