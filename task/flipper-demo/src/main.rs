// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use drv_display_api::*;
use userlib::*;

use embedded_graphics::{
    mono_font::{ascii::FONT_9X18, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Text},
};

task_slot!(DISPLAY, display);

#[export_name = "main"]
fn main() -> ! {
    let display = Display::from(DISPLAY.get_task_id());

    let mut remote_display = RemoteDisplay::new(display);

    let mut inverse = false;

    loop {
        let (fill_color, text_color) = if inverse {
            (BinaryColor::Off, BinaryColor::On)
        } else {
            (BinaryColor::On, BinaryColor::Off)
        };

        remote_display.clear(text_color).unwrap();

        Rectangle::with_center(
            remote_display.bounding_box().center(),
            Size::new(96, 32),
        )
        .into_styled(PrimitiveStyle::with_fill(fill_color))
        .draw(&mut remote_display)
        .ok();

        let style = MonoTextStyle::new(&FONT_9X18, text_color);
        let text = "Hubris!";
        Text::with_alignment(
            text,
            remote_display.bounding_box().center(),
            style,
            Alignment::Center,
        )
        .draw(&mut remote_display)
        .ok();

        remote_display.flush().ok();
        hl::sleep_for(2000);
        inverse = !inverse;
    }
}
