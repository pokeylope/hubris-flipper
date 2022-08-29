// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]

use core::cell::Cell;
use drv_spi_api::{SpiDevice, SpiError};
use embedded_graphics::{
    mono_font::{ascii::FONT_9X18, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};

const WIDTH: u32 = 128;
const HEIGHT: u32 = 64;
const MAX_COL: u32 = (WIDTH - 1) as u32;
const MAX_ROW: u32 = (HEIGHT - 1) as u32;
const PAGES: u32 = HEIGHT / 8;
const FB_SIZE: usize = WIDTH as usize * PAGES as usize;

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
enum Command {
    OnOff(bool),          // Switch Display ON/OFF
    SetLine(u8),          // Set Start Line
    SetPage(u8),          // Set Page address
    SetColumnMsb(u8),     // Set Column MSB
    SetColumnLsb(u8),     // Set Column LSB
    SegDirection(bool),   // Reverse scan direction of SEG
    InverseDisplay(bool), // Invert display
    AllPixelOn(bool),     // Set all pixel on
    BiasSelect(bool),     // Select 1/9(0) or 1/7(1) bias
    RMW(bool),            // Enter Read Modify Write mode
    End,                  // Exit Read Modify Write mode
    Reset,                // Software Reset
    ComDirection(bool),   // Com direction reverse
    PowerControl(u8),     // Power control
    RegulationRatio(u8),  // Regulation resistor ratio
    SetEv(u8),            // Set electronic volume
    SetBooster(u8),       // Set Booster level
    Nop,                  // No operation
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Mode {
    Command,
    Data,
}

pub struct St7565r<F>
where
    F: Fn(Mode),
{
    spi: SpiDevice,
    mode: Cell<Mode>,
    toggle_di: F,
    display: St7565rDisplay,
}

impl<F> St7565r<F>
where
    F: Fn(Mode),
{
    pub fn new(spi: SpiDevice, toggle_di: F) -> Self {
        toggle_di(Mode::Command);
        Self {
            spi,
            mode: Cell::new(Mode::Command),
            toggle_di,
            display: St7565rDisplay::new(),
        }
    }

    fn set_mode(&self, mode: Mode) {
        if self.mode.get() != mode {
            (self.toggle_di)(mode);
            self.mode.set(mode);
        }
    }

    fn write(&self, cmd: Command) -> Result<(), SpiError> {
        let (cmd_byte, data) = match cmd {
            Command::OnOff(val) => (0b10101110 | val as u8, None), // Switch Display ON/OFF: last bit
            Command::SetLine(val) => (0b01000000 | val & 0x3f, None), // Set Start Line: last 6 bits
            Command::SetPage(val) => (0b10110000 | val & 0xf, None), // Set Page address: last 4 bits
            Command::SetColumnMsb(val) => (0b00010000 | val & 0xf, None), // Set Column MSB: last 4 bits
            Command::SetColumnLsb(val) => (0b00000000 | val & 0xf, None), // Set Column LSB: last 4 bits
            Command::SegDirection(val) => (0b10100000 | val as u8, None), // Reverse scan direction of SEG: last bit
            Command::InverseDisplay(val) => (0b10100110 | val as u8, None), // Invert display: last bit
            Command::AllPixelOn(val) => (0b10100100 | val as u8, None), // Set all pixel on: last bit
            Command::BiasSelect(val) => (0b10100010 | val as u8, None), // Select 1/9(0) or 1/7(1) bias: last bit
            Command::RMW(val) => (0b11100000 | val as u8, None), // Enter Read Modify Write mode: read+0, write+1
            Command::End => (0b11101110, None), // Exit Read Modify Write mode
            Command::Reset => (0b11100010, None), // Software Reset
            Command::ComDirection(val) => {
                (0b11000000 | if val { 0b1000 } else { 0 }, None)
            } // Com direction reverse: +0b1000
            Command::PowerControl(val) => (0b00101000 | val & 7, None), // Power control: last 3 bits VB:VR:VF
            Command::RegulationRatio(val) => (0b00100000 | val & 7, None), // Regulation resistor ratio: last 3bits
            Command::SetEv(val) => (0b10000001, Some(val)), // Set electronic volume: 5 bits in next byte
            Command::SetBooster(val) => (0b11111000, Some(val & 3)), // Set Booster level, 2x,3x,4x(0); 5x(1); or 6x(3): 2 bits in next byte
            Command::Nop => (0b11100011, None), // No operation
        };
        self.set_mode(Mode::Command);
        if let Some(data_byte) = data {
            self.spi.write(&[cmd_byte, data_byte])
        } else {
            self.spi.write(&[cmd_byte])
        }
    }

    fn write_data(&self, data: u8) -> Result<(), SpiError> {
        self.set_mode(Mode::Data);
        self.spi.write(&[data])
    }

    fn set_page(&self, page: u8) -> Result<(), SpiError> {
        self.write(Command::SetPage(page))
    }

    fn set_col(&self, col: u8) -> Result<(), SpiError> {
        self.write(Command::SetColumnMsb(col >> 4))?;
        self.write(Command::SetColumnLsb(col & 0xf))
    }

    fn flush(&self) -> Result<(), SpiError> {
        for page in 0..PAGES {
            self.set_page(page as u8)?;
            self.set_col(0)?;
            for col in 0..WIDTH {
                self.write_data(
                    self.display.framebuffer[(page * WIDTH + col) as usize],
                )?;
            }
        }
        Ok(())
    }

    fn clear(&mut self) {
        self.display.clear(BinaryColor::Off).ok();
    }

    pub fn initialize(&mut self) -> Result<(), SpiError> {
        self.write(Command::Reset)?;
        self.write(Command::BiasSelect(false))?;
        self.write(Command::SegDirection(false))?;
        self.write(Command::ComDirection(true))?;
        self.write(Command::SetLine(0))?;
        self.write(Command::RegulationRatio(0b101))?;
        self.write(Command::SetEv(33))?;
        self.write(Command::PowerControl(0b111))?;
        self.clear();
        self.flush()?;
        self.write(Command::OnOff(true))?; // display on
        Ok(())
    }

    pub fn test(&mut self) -> Result<(), SpiError> {
        let style = MonoTextStyle::new(&FONT_9X18, BinaryColor::On);
        let text = "Hubris!";
        Text::with_alignment(
            text,
            self.display.bounding_box().center(),
            style,
            Alignment::Center,
        )
        .draw(&mut self.display)?;

        self.flush()?;
        Ok(())
    }
}

struct St7565rDisplay {
    framebuffer: [u8; FB_SIZE],
}

impl St7565rDisplay {
    fn new() -> Self {
        Self {
            framebuffer: [0; FB_SIZE],
        }
    }
}

impl DrawTarget for St7565rDisplay {
    type Color = BinaryColor;

    type Error = SpiError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            // Check if the pixel coordinates are out of bounds (negative or greater than
            // (63,63)). `DrawTarget` implementation are required to discard any out of bounds
            // pixels without returning an error or causing a panic.
            if let Ok((x @ 0..=MAX_COL, y @ 0..=MAX_ROW)) = coord.try_into() {
                let page = y / 8;
                let bit = y % 8;
                let index: usize = (x + page * WIDTH) as usize;
                if color.is_on() {
                    self.framebuffer[index] |= 1 << bit;
                } else {
                    self.framebuffer[index] &= !(1 << bit);
                }
            }
        }

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let val = match color {
            BinaryColor::Off => 0,
            BinaryColor::On => 1,
        };
        self.framebuffer.fill(val);

        Ok(())
    }
}

impl OriginDimensions for St7565rDisplay {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}
