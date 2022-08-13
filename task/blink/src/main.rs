// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use drv_stm32xx_sys_api::*;
use userlib::*;

task_slot!(SYS, sys);

#[export_name = "main"]
pub fn main() -> ! {
    const INTERVAL: u64 = 3000;

    let pins = PinSet { port: Port::A, pin_mask: 1 << 7 };
    let sys = Sys::from(SYS.get_task_id());
    sys.gpio_configure_output(pins, OutputType::PushPull, Speed::Low, Pull::None).unwrap();

    loop {
        sys.gpio_set(pins).unwrap();
        hl::sleep_for(INTERVAL);
        sys.gpio_reset(pins).unwrap();
        hl::sleep_for(INTERVAL);
    }
}
