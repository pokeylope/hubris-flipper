// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! STM32WB specifics

use crate::periph;
use userlib::FromPrimitive;

/// Peripherals appear in "groups." All peripherals in a group are controlled
/// from the same subset of registers in the RCC.
///
/// The reference manual lacks a term for this, so we made this one up. It would
/// be tempting to refer to these as "buses," but in practice there are almost
/// always more groups than there are buses, particularly on M0.
///
/// This is `pub` mostly for use inside driver-servers.
#[derive(Copy, Clone, Debug, FromPrimitive)]
#[repr(u8)]
pub enum Group {
    Ahb1,
    Ahb2,
    Ahb3,
    Apb1_1,
    Apb1_2,
    Apb2,
}
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u32)]
pub enum Peripheral {
    I2c1 = periph(Group::Apb1_1, 21),
    Spi1 = periph(Group::Apb2, 12),
    Spi2 = periph(Group::Apb1_1, 14),
}
