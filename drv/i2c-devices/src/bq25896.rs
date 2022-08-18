// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Driver for the BQ25896 battery controller

use drv_i2c_api::*;
use userlib::FromPrimitive;

#[derive(Copy, Clone, Debug, PartialEq, FromPrimitive)]
enum Register {
    R00 = 0x00,
    R01 = 0x01,
    R02 = 0x02,
    R03 = 0x03,
    R04 = 0x04,
    R05 = 0x05,
    R06 = 0x06,
    R07 = 0x07,
    R08 = 0x08,
    R09 = 0x09,
    R0a = 0x0a,
    R0b = 0x0b,
    R0c = 0x0c,
    R0d = 0x0d,
    R0e = 0x0e,
    R0f = 0x0f,
    R10 = 0x10,
    R11 = 0x11,
    R12 = 0x12,
    R13 = 0x13,
    R14 = 0x14,
}

pub struct Bq25896 {
    device: I2cDevice,
}

impl core::fmt::Display for Bq25896 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "bq25896: {}", &self.device)
    }
}

impl Bq25896 {
    pub fn new(device: &I2cDevice) -> Self {
        Self { device: *device }
    }

    fn read_reg(&self, register: Register) -> Result<u8, ResponseCode> {
        self.device.read_reg(register as u8)
    }

    fn write_reg(
        &self,
        register: Register,
        value: u8,
    ) -> Result<(), ResponseCode> {
        self.device.write(&[register as u8, value])
    }

    fn set_bit(&self, register: Register, bit: u8)  -> Result<(), ResponseCode> {
        let mut val = self.read_reg(register)?;
        val |= 1 << bit;
        self.write_reg(register, val)
    }

    fn clear_bit(&self, register: Register, bit: u8)  -> Result<(), ResponseCode> {
        let mut val = self.read_reg(register)?;
        val &= !(1 << bit);
        self.write_reg(register, val)
    }

    fn reset(&self) -> Result<(), ResponseCode> {
        self.write_reg(Register::R14, 1 << 7)
    }

    pub fn initialize(&self) -> Result<(), ResponseCode> {
        self.reset()
    }

    pub fn power_off(&self) -> Result<(), ResponseCode> {
        self.set_bit(Register::R09, 5)
    }
}
