// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A driver for the STM32WB SPI, in host mode.
//!
//! This is the core logic, separated from the IPC server. The peripheral also
//! supports I2S, which we haven't bothered implementing because we don't have a
//! need for it.
//!
//! # Clocking
//!
//! The SPI block has no fewer than three clock domains.
//!
//! 1. `pclk` contains most of the control logic and operates at the APB
//!    frequency.
//!
//! 2. `ker_ck` contains the clock generator and is driven as a "kernel clock"
//!    from the RCC -- there is a separate mux there to choose its source.
//!
//! 3. The "serial interface domain" (no catchy abbreviation provided) is
//!    clocked at the external SCK rate. This is derived from `ker_ck` in host
//!    role.
//!
//! In host role, the SPI needs to have at least `ker_ck` running to do useful
//! work.
//!
//! # Automagic CRC generation
//!
//! We do not currently support the hardware's automatic CRC features.
//!
//! # Why is everything `spi1`
//!
//! The `stm32wb` PAC crate we currently use has decided that all SPI types
//! should be called `spi1`. This is despite significant hardware differences
//! between SPI1-6. Our is not to question why.

#![no_std]

#[cfg(feature = "wb55")]
use stm32wb::stm32wb55 as device;

pub struct Spi {
    /// Pointer to our register block.
    ///
    /// This is not a `SPIx` type from the `stm32h7` crate because then we're
    /// generic for no good reason and type parameters multiply. Ew.
    reg: &'static device::spi1::RegisterBlock,
}

impl From<&'static device::spi1::RegisterBlock> for Spi {
    fn from(reg: &'static device::spi1::RegisterBlock) -> Self {
        Self { reg }
    }
}

impl Spi {
    pub fn initialize(
        &mut self,
        mbr: device::spi1::cr1::BR_A,
        bits_per_frame: u8,
        comm: device::spi1::cr1::BIDIMODE_A,
        lsbfirst: device::spi1::cr1::LSBFIRST_A,
        cpha: device::spi1::cr1::CPHA_A,
        cpol: device::spi1::cr1::CPOL_A,
    ) {
        // Expected preconditions:
        // - GPIOs configured to proper AF etc - we cannot do this, because we
        // cannot presume to have either direct GPIO access _or_ IPC access.
        // - Clock on, reset off - again, we can't do this directly.

        assert!((4..=32).contains(&bits_per_frame));

        self.reg.cr1.write(|w| {
            w.bidimode().variant(comm)
                .mstr().set_bit()
                .cpol().variant(cpol)
                .cpha().variant(cpha)
                .ssm().set_bit()
                .ssi().set_bit()
                .br().variant(mbr)
                .lsbfirst().variant(lsbfirst)
        });

        self.reg.cr2.modify(|_, w| {
            if bits_per_frame < 9 {
                w.frxth().quarter();
            }
            unsafe { w.ds().bits(bits_per_frame - 1) };
            w.ssoe().set_bit()
        });

    }

    pub fn enable(&mut self, _tsize: u16, div: device::spi1::cr1::BR_A) {
        self.reg.cr1.modify(|_, w| w.br().variant(div));
        self.reg.cr1.modify(|_, w| w.spe().set_bit());
    }

    pub fn start(&mut self) {
    }

    pub fn can_rx_byte(&self) -> bool {
        let sr = self.reg.sr.read();
        sr.rxne().bit_is_set()
    }

    pub fn can_tx_frame(&self) -> bool {
        let sr = self.reg.sr.read();
        sr.txe().bit_is_set()
    }

    pub fn send16(&mut self, bytes: u16) {
        self.reg.dr.write(|w| w.dr().bits(bytes));
    }

    /// Stuffs one byte of data into the SPI TX FIFO.
    ///
    /// Preconditions:
    ///
    /// - There must be room for a byte in the TX FIFO (call `can_tx_frame` to
    ///   check, or call this in response to a TXP interrupt).
    pub fn send8(&mut self, byte: u8) {
        // The DR register can be accessed as a byte, halfword, or word. This
        // determines how many bytes are pushed in. stm32h7/svd2rust don't
        // understand this, and so we have to get a pointer to the byte portion
        // of the register manually and dereference it.

        // Because svd2rust didn't see this one coming, we cannot get a direct
        // reference to the VolatileCell within the wrapped Reg type of dr,
        // nor will the Reg type agree to give us a pointer to its contents like
        // VolatileCell will, presumably to save us from ourselves. And thus we
        // must exploit the fact that VolatileCell is the only (non-zero-sized)
        // member of Reg, and in fact _must_ be for Reg to work correctly when
        // used to overlay registers in memory.

        // Safety: "Downcast" dr to a pointer to its sole member, whose type
        // we know because of our unholy source-code-reading powers.
        let dr: &vcell::VolatileCell<u32> =
            unsafe { core::mem::transmute(&self.reg.dr) };
        // vcell is more pleasant and will happily give us the pointer we want.
        let dr: *mut u32 = dr.as_ptr();
        // As we are a little-endian machine it is sufficient to change the type
        // of the pointer to byte.
        let dr8 = dr as *mut u8;

        // Safety: we are dereferencing a pointer given to us by VolatileCell
        // (and thus UnsafeCell) using the same volatile access it would use.
        unsafe {
            dr8.write_volatile(byte);
        }
    }

    pub fn recv16(&mut self) -> u16 {
        self.reg.dr.read().dr().bits()
    }

    /// Pulls one byte of data from the SPI RX FIFO.
    ///
    /// Preconditions:
    ///
    /// - There must be at least one byte of data in the FIFO (check using
    ///   `has_rx_byte` or call this in response to an RXP interrupt).
    ///
    /// - Frame size must be set to 8 bits or smaller. (Behavior if you write a
    ///   partial frame to the FIFO is not immediately clear from the
    ///   datasheet.)
    pub fn recv8(&mut self) -> u8 {
        // The DR register can be accessed as a byte, halfword, or word. This
        // determines how many bytes are pushed in. stm32h7/svd2rust don't
        // understand this, and so we have to get a pointer to the byte portion
        // of the register manually and dereference it.

        // See send8 for further rationale / ranting.

        // Safety: "Downcast" dr to a pointer to its sole member, whose type
        // we know because of our unholy source-code-reading powers.
        let dr: &vcell::VolatileCell<u32> =
            unsafe { core::mem::transmute(&self.reg.dr) };
        // vcell is more pleasant and will happily give us the pointer we want.
        let dr: *mut u32 = dr.as_ptr();
        // As we are a little-endian machine it is sufficient to change the type
        // of the pointer to byte.
        let dr8 = dr as *mut u8;

        // Safety: we are dereferencing a pointer given to us by VolatileCell
        // (and thus UnsafeCell) using the same volatile access it would use.
        unsafe { dr8.read_volatile() }
    }

    pub fn end(&mut self) {
        //// Disable the transfer state machine.
        self.reg.cr1.modify(|_, w| w.spe().clear_bit());
        // Turn off interrupt enables.
        self.reg
            .cr2
            .modify(|_, w| w.txeie().clear_bit().rxneie().clear_bit());

        //// This is where we'd report errors (TODO). For now, just clear the
        //// error flags, as they're sticky.
        self.clear_ovr();
    }

    fn clear_ovr(&self) {
        self.reg.dr.read();
        self.reg.sr.read();
    }

    pub fn enable_transfer_interrupts(&mut self) {
        self.reg
            .cr2
            .modify(|_, w| w.txeie().set_bit().rxneie().set_bit());
    }

    pub fn disable_can_tx_interrupt(&mut self) {
        self.reg.cr2.modify(|_, w| w.txeie().clear_bit());
    }

    pub fn enable_can_tx_interrupt(&mut self) {
        self.reg.cr2.modify(|_, w| w.txeie().set_bit());
    }

    pub fn read_status(&self) -> u32 {
        self.reg.sr.read().bits()
    }

    pub fn check_overrun(&self) -> bool {
        self.reg.sr.read().ovr().is_overrun()
    }

    pub fn is_busy(&self) -> bool {
        self.reg.sr.read().bsy().is_busy()
    }
}
