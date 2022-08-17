// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use drv_i2c_devices::lp5562::*;
use drv_stm32xx_sys_api::*;
use userlib::*;

include!(concat!(env!("OUT_DIR"), "/i2c_config.rs"));
task_slot!(I2C, i2c_driver);
task_slot!(SYS, sys);

#[export_name = "main"]
pub fn main() -> ! {
    const INTERVAL: u64 = 2000;

    let pins = PinSet {
        port: Port::A,
        pin_mask: 1 << 7,
    };
    let sys = Sys::from(SYS.get_task_id());
    sys.gpio_configure_output(
        pins,
        OutputType::PushPull,
        Speed::Low,
        Pull::None,
    )
    .unwrap();

    let i2c_task = I2C.get_task_id();
    let dev = Lp5562::new(&i2c_config::devices::lp5562(i2c_task)[0]);

    dev.initialize().unwrap();

    loop {
        sys.gpio_set(pins).unwrap();
        dev.set_color(255, 0, 0).unwrap();
        hl::sleep_for(INTERVAL);
        sys.gpio_reset(pins).unwrap();
        dev.set_color(0, 0, 255).unwrap();
        hl::sleep_for(INTERVAL);
    }
}
