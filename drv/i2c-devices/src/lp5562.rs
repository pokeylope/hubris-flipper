// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Driver for the LP5562 LED controller

use drv_i2c_api::*;
use userlib::{hl::sleep_for, FromPrimitive};

const LED_CURRENT_RED: u8 = 50;
const LED_CURRENT_GREEN: u8 = 50;
const LED_CURRENT_BLUE: u8 = 50;
const LED_CURRENT_WHITE: u8 = 150;

#[derive(Copy, Clone, Debug, PartialEq, FromPrimitive)]
enum Register {
    Enable = 0x00,
    OpMode = 0x01,
    BPwm = 0x02,
    GPwm = 0x03,
    RPwm = 0x04,
    BCurrent = 0x05,
    GCurrent = 0x06,
    RCurrent = 0x07,
    Config = 0x08,

    Reset = 0x0d,
    WPwm = 0x0e,
    WCurrent = 0x0f,

    LedMap = 0x70,
}

pub struct Lp5562 {
    device: I2cDevice,
}

impl core::fmt::Display for Lp5562 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "lp5562: {}", &self.device)
    }
}

impl Lp5562 {
    pub fn new(device: &I2cDevice) -> Self {
        Self { device: *device }
    }

    //fn read_reg(&self, register: Register) -> Result<u8, ResponseCode> {
    //    self.device.read_reg(register as u8)
    //}

    fn write_reg(
        &self,
        register: Register,
        value: u8,
    ) -> Result<(), ResponseCode> {
        self.device.write(&[register as u8, value])
    }

    fn reset(&self) -> Result<(), ResponseCode> {
        self.write_reg(Register::Reset, 0xff)
    }

    fn enable(&self) -> Result<(), ResponseCode> {
        self.write_reg(Register::Enable, 0b1100_0000)?;
        sleep_for(1); // 500 μs
        Ok(())
    }

    fn configure(&self) -> Result<(), ResponseCode> {
        self.write_reg(Register::Config, 0b0110_0001)?;
        self.write_reg(Register::LedMap, 0)
    }

    pub fn initialize(&self) -> Result<(), ResponseCode> {
        self.reset()?;

        self.write_reg(Register::BCurrent, LED_CURRENT_BLUE)?;
        self.write_reg(Register::GCurrent, LED_CURRENT_GREEN)?;
        self.write_reg(Register::RCurrent, LED_CURRENT_RED)?;
        self.write_reg(Register::WCurrent, LED_CURRENT_WHITE)?;

        self.write_reg(Register::BPwm, 0)?;
        self.write_reg(Register::GPwm, 0)?;
        self.write_reg(Register::RPwm, 0)?;
        self.write_reg(Register::WPwm, 0)?;

        self.enable()?;
        self.configure()?;

        Ok(())
    }

    pub fn set_color(
        &self,
        red: u8,
        green: u8,
        blue: u8,
    ) -> Result<(), ResponseCode> {
        self.write_reg(Register::BPwm, blue)?;
        self.write_reg(Register::GPwm, green)?;
        self.write_reg(Register::RPwm, red)?;
        Ok(())
    }

    pub fn enable_backlight(&self) -> Result<(), ResponseCode> {
        self.write_reg(Register::WPwm, 0xff)
    }

    pub fn disable_backlight(&self) -> Result<(), ResponseCode> {
        self.write_reg(Register::WPwm, 0)
    }
}
