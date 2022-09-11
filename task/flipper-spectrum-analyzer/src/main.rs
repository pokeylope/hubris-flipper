// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use cc1101::{Cc1101, Cc1101Error};
use drv_display_api::*;
use drv_spi_api::*;
use embedded_graphics::{
    mono_font::{ascii::FONT_9X18, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
    text::{Alignment, Text},
};
use userlib::*;

task_slot!(SPI, spi1_driver);
task_slot!(DISPLAY, display);

fn u8_to_str(buf: &mut [u8], mut val: u8) -> usize {
    if val == 0 {
        buf[0] = '0' as u8;
        return 1;
    }
    let mut len = 0;
    while val > 0 && len < buf.len() {
        buf[len] = (val % 10) as u8 + '0' as u8;
        val /= 10;
        len += 1;
    }
    buf[0..len].reverse();
    len
}

fn i32_to_str(buf: &mut [u8], mut val: i32) -> usize {
    if val == 0 {
        buf[0] = '0' as u8;
        return 1;
    }
    let mut len = 0;
    let negate = val < 0;
    let max_len = if negate {
        val = !val + 1;
        buf.len() - 1
    } else {
        buf.len()
    };
    while val > 0 && len < max_len {
        buf[len] = (val % 10) as u8 + '0' as u8;
        val /= 10;
        len += 1;
    }
    if negate {
        buf[len] = '-' as u8;
        len += 1;
    }
    buf[0..len].reverse();
    len
}

const NUM_CHANNELS: u32 = 128;
const VSCROLL: i32 = 48;
const GRAPH_BASELINE: i32 = 50;

#[export_name = "main"]
fn main() -> ! {
    let spi = Spi::from(SPI.get_task_id()).device(0);
    let cc1101 = Cc1101::new(spi);
    cc1101.initialize().unwrap();
    cc1101.load_preset().unwrap();
    cc1101.set_frequency(433920000).unwrap();
    cc1101.flush_rx().unwrap();

    let display = Display::from(DISPLAY.get_task_id());

    let mut remote_display = RemoteDisplay::new(display);

    let freq_center = 315_000_000;
    let chan_spacing = 196078;
    //let chan_spacing = 39215;
    let mut chan_ss = [0u8; NUM_CHANNELS as usize];

    loop {
        let freq_min = freq_center - chan_spacing * (NUM_CHANNELS / 2 + 1);

        for chan in 0..NUM_CHANNELS {
            let freq = freq_min + chan * chan_spacing;
            let rssi = (get_rssi(&cc1101, freq).unwrap() + 132.0) * 2.0;
            chan_ss[chan as usize] = rssi as u8;
        }

        remote_display.clear(BinaryColor::Off).unwrap();

        for chan in 0..NUM_CHANNELS {
            let height =
                ((chan_ss[chan as usize] as i32) - VSCROLL).max(0) >> 2;
            let start = Point::new(chan as i32, GRAPH_BASELINE);
            let end = Point::new(chan as i32, GRAPH_BASELINE - height);
            Line::new(start, end)
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(&mut remote_display)
                .ok();
        }

        remote_display.flush().ok();
    }
}

fn get_rssi(cc1101: &Cc1101, freq: u32) -> Result<f32, Cc1101Error> {
    cc1101.set_frequency(freq)?;
    cc1101.switch_to_rx(true)?;
    //hl::sleep_for(1);
    let rssi = cc1101.get_rssi()?;
    cc1101.switch_to_idle()?;
    Ok(rssi)
}
