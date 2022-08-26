// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]

use drv_spi_api::{SpiDevice, SpiError};

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
enum Command {
    OnOff = 0b10101110, // Switch Display ON/OFF: last bit
    SetLine = 0b01000000, // Set Start Line: last 6 bits
    SetPage = 0b10110000, // Set Page address: last 4 bits
    SetColumnMsb = 0b00010000, // Set Column MSB: last 4 bits
    SetColumnLsb = 0b00000000, // Set Column LSB: last 4 bits
    SegDirection = 0b10100000, // Reverse scan direction of SEG: last bit
    InverseDisplay = 0b10100110, // Invert display: last bit
    AllPixelOn = 0b10100100, // Set all pixel on: last bit
    BiasSelect = 0b10100010, // Select 1/9(0) or 1/7(1) bias: last bit
    RMW = 0b11100000, // Enter Read Modify Write mode: read+0, write+1
    End = 0b11101110, // Exit Read Modify Write mode
    Reset = 0b11100010, // Software Reset
    ComDirection = 0b11000000, // Com direction reverse: +0b1000
    PowerControl = 0b00101000, // Power control: last 3 bits VB:VR:VF
    RegulationRatio = 0b00100000, // Regulation resistor ration: last 3bits
    SetEv = 0b10000001, // Set electronic volume: 5 bits in next byte
    SetBooster = 0b11111000, // Set Booster level, 4X(0) or 5X(1): last bit in next byte
    Nop = 0b11100011, // No operation
}

pub struct St7565r {
    spi: SpiDevice,
}

impl St7565r {
    pub fn new(spi: SpiDevice) -> Self {
        Self { spi }
    }

    pub fn initialize(&self) -> Result<(), SpiError> {
        self.spi.write(&[Command::Reset as u8])?;
        self.spi.write(&[Command::BiasSelect as u8])?;
        self.spi.write(&[Command::SegDirection as u8])?;
        self.spi.write(&[Command::ComDirection as u8 | 0b1000])?;
        self.spi.write(&[Command::SetLine as u8])?;
        self.spi.write(&[Command::RegulationRatio as u8 | 0b101])?;
        self.spi.write(&[Command::SetEv as u8, 33])?;
        self.spi.write(&[Command::PowerControl as u8 | 0b111])?;
        Ok(())
    }

    pub fn test(&self) -> Result<(), SpiError> {
        self.spi.write(&[Command::OnOff as u8 | 1])?; // display on
        self.spi.write(&[Command::AllPixelOn as u8 | 1])?; // all points on
        Ok(())
    }
}
