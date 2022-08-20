// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use drv_i2c_devices::{bq25896::*/*, lp5562::* */};
use drv_stm32xx_sys_api::*;
use userlib::*;

include!(concat!(env!("OUT_DIR"), "/i2c_config.rs"));
task_slot!(I2C, i2c_driver);
task_slot!(SYS, sys);

const OUT_PIN: PinSet = Port::A.pin(7);
const BACK_BUTTON: PinSet = Port::C.pin(13);
const PERIPH_POWER: PinSet = Port::A.pin(3);

#[export_name = "main"]
pub fn main() -> ! {
    const INTERVAL: u64 = 2000;

    let sys = Sys::from(SYS.get_task_id());

    sys.gpio_configure_output(
        OUT_PIN,
        OutputType::PushPull,
        Speed::Low,
        Pull::None,
    )
    .unwrap();

    sys.gpio_configure_input(BACK_BUTTON, Pull::None).unwrap();

    sys.gpio_configure_output(
        PERIPH_POWER,
        OutputType::OpenDrain,
        Speed::Low,
        Pull::None,
    )
    .unwrap();

    let i2c_task = I2C.get_task_id();
    //let lp5562 = Lp5562::new(&i2c_config::devices::lp5562(i2c_task)[0]);
    let bq25896 = Bq25896::new(&i2c_config::devices::bq25896(i2c_task)[0]);

    //lp5562.initialize().unwrap();

    let mut button_held = false;

    let mut check_button = || {
        if sys.gpio_read(BACK_BUTTON).unwrap() == 0 {
            if button_held {
                power_off(&sys, &bq25896);
            } else {
                button_held = true;
            }
        } else {
            button_held = false;
        }
    };

    loop {
        check_button();
        sys.gpio_set(OUT_PIN).unwrap();
        //lp5562.set_color(255, 0, 0).unwrap();
        hl::sleep_for(INTERVAL);

        check_button();
        sys.gpio_reset(OUT_PIN).unwrap();
        //lp5562.set_color(0, 0, 255).unwrap();
        hl::sleep_for(INTERVAL);
    }
}

fn power_off(sys: &Sys, bq25896: &Bq25896) {
    // "Crutch: shutting down with ext 3V3 off is causing LSE to stop"
    let _ = sys.gpio_set(PERIPH_POWER);
    hl::sleep_for(1);
    bq25896.power_off().unwrap();
}
