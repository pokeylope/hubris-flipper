// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use core::convert::From;
use drv_i2c_devices::lp5562::*;
use drv_stm32xx_sys_api::*;
use stm32wb::stm32wb55 as device;
use userlib::*;

const BACKLIGHT_DURATION: u64 = 30 * 1000;

struct Button {
    pinset: PinSet,
    invert: bool,
    color: (u8, u8, u8),
}

impl Button {
    fn is_pushed(&self, sys: &Sys) -> bool {
        (sys.gpio_read(self.pinset).unwrap() != 0) != self.invert
    }
}

const BUTTON_UP: Button = Button { pinset: Port::B.pin(10), invert: true, color: (0, 255, 0) };
const BUTTON_DOWN: Button = Button { pinset: Port::C.pin(6), invert: true, color: (255, 255, 0) };
const BUTTON_LEFT: Button = Button { pinset: Port::B.pin(12), invert: true, color: (0, 255, 255) };
const BUTTON_RIGHT: Button = Button { pinset: Port::B.pin(11), invert: true, color: (255, 0, 255) };
const BUTTON_OK: Button = Button { pinset: Port::H.pin(3), invert: false, color: (0, 0, 255) };
const BUTTON_BACK: Button = Button { pinset: Port::C.pin(13), invert: true, color: (0, 0, 255) };

const BUTTONS: [Button; 6] = [
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_LEFT,
    BUTTON_RIGHT,
    BUTTON_OK,
    BUTTON_BACK,
];

include!(concat!(env!("OUT_DIR"), "/i2c_config.rs"));
task_slot!(I2C, i2c_driver);
task_slot!(SYS, sys);

const EXTI3: u32 = 1 << 0;
const EXTI9_5: u32 = 1 << 1;
const EXTI15_10: u32 = 1 << 2;
const TIMER_EXPIRED: u32 = 1 << 3;

const OUT_PIN: PinSet = Port::A.pin(6);

#[export_name = "main"]
fn main() -> ! {
    let sys = Sys::from(SYS.get_task_id());

    sys.gpio_configure_output(
        OUT_PIN,
        OutputType::PushPull,
        Speed::Low,
        Pull::None,
    )
    .unwrap();

    let p = device::Peripherals::take().unwrap();
    let syscfg = p.SYSCFG;
    let exti = p.EXTI;

    for button in &BUTTONS {
        sys.gpio_configure_input(button.pinset, Pull::Up).unwrap();
        configure_exti(&syscfg, &exti, button.pinset.port, get_pin(button.pinset).unwrap());
    }

    let i2c_task = I2C.get_task_id();
    let lp5562 = Lp5562::new(&i2c_config::devices::lp5562(i2c_task)[0]);
    lp5562.initialize().unwrap();
    lp5562.set_color(255, 0, 0).unwrap();
    lp5562.enable_backlight().unwrap();

    sys_irq_control(EXTI3, true);
    sys_irq_control(EXTI9_5, true);
    sys_irq_control(EXTI15_10, true);
    sys_set_timer(Some(sys_get_timer().now + BACKLIGHT_DURATION), TIMER_EXPIRED);
    loop {
        let result = sys_recv_closed(&mut [], EXTI3 | EXTI9_5 | EXTI15_10 | TIMER_EXPIRED, TaskId::KERNEL).unwrap();
        match result.operation {
            TIMER_EXPIRED => lp5562.disable_backlight().unwrap(),
            _ => {
                if let Some(button) = BUTTONS.iter().find(|b| b.is_pushed(&sys)) {
                    lp5562.enable_backlight().unwrap();
                    sys_set_timer(Some(sys_get_timer().now + BACKLIGHT_DURATION), TIMER_EXPIRED);
                    sys.gpio_set(OUT_PIN).unwrap();
                    let (r, g, b) = button.color;
                    lp5562.set_color(r, g, b).unwrap();
                } else {
                    sys.gpio_reset(OUT_PIN).unwrap();
                    lp5562.set_color(255, 0, 0).unwrap();
                }
                exti.pr1.write(|w| unsafe { w.bits(0xffffffff) });
                sys_irq_control(result.operation, true);
            }
        }
    }
}

fn get_pin(pinset: PinSet) -> Option<u8> {
    let pin_mask = pinset.pin_mask;
    (0..32).find(|i| (pin_mask >> i) & 1 != 0)
}

fn configure_exti(
    syscfg: &device::SYSCFG,
    exti: &device::EXTI,
    port: Port,
    pin: u8,
) {
    let port_num = port as u32;
    let pin_shift = 4 * (pin % 4);

    exti.imr1
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pin)) });
    if pin < 4 {
        syscfg.exticr1.modify(|r, w| unsafe {
            w.bits(r.bits() | (port_num << pin_shift))
        });
    } else if pin < 8 {
        syscfg.exticr2.modify(|r, w| unsafe {
            w.bits(r.bits() | (port_num << pin_shift))
        });
    } else if pin < 12 {
        syscfg.exticr3.modify(|r, w| unsafe {
            w.bits(r.bits() | (port_num << pin_shift))
        });
    } else if pin < 16 {
        syscfg.exticr4.modify(|r, w| unsafe {
            w.bits(r.bits() | (port_num << pin_shift))
        });
    } else {
        panic!();
    }
    exti.rtsr1
        .modify(|r, w| unsafe { w.bits(r.bits() | 1 << pin) });
    exti.ftsr1
        .modify(|r, w| unsafe { w.bits(r.bits() | 1 << pin) });
}
