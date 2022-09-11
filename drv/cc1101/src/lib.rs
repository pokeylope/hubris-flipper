// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]

use drv_spi_api::{SpiDevice, SpiError};

const CC1101_QUARTZ: u64 = 26000000;
const CC1101_FMASK: u64 = 0xFFFFFF;
const CC1101_FDIV: u64 = 0x10000;
const CC1101_IFDIV: u64 = 0x400;

const CC1101_READ: u8 = 1 << 7; /* Read Bit */
const CC1101_BURST: u8 = 1 << 6; /* Burst Bit */

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, PartialEq)]
enum Command {
    SRES = 0x30,    /* Reset chip. */
    SFSTXON = 0x31, /* Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA): Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround). */
    SXOFF = 0x32,   /* Turn off crystal oscillator. */
    SCAL = 0x33, /* Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0) */
    SRX = 0x34, /* Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1. */
    STX = 0x35, /* In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled: Only go to TX if channel is clear. */
    SIDLE = 0x36, /* Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable. */
    SWOR = 0x38, /* Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0. */
    SPWD = 0x39, /* Enter power down mode when CSn goes high. */
    SFRX = 0x3A, /* Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states. */
    SFTX = 0x3B, /* Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states. */
    SWORRST = 0x3C, /* Reset real time clock to Event1 value. */
    SNOP = 0x3D, /* No operation. May be used to get access to the chip status byte.*/
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
enum Register {
    IOCFG2 = 0x00,   /* GDO2 output pin configuration */
    IOCFG1 = 0x01,   /* GDO1 output pin configuration */
    IOCFG0 = 0x02,   /* GDO0 output pin configuration */
    FIFOTHR = 0x03,  /* RX FIFO and TX FIFO thresholds */
    SYNC1 = 0x04,    /* Sync word, high byte */
    SYNC0 = 0x05,    /* Sync word, low byte */
    PKTLEN = 0x06,   /* Packet length */
    PKTCTRL1 = 0x07, /* Packet automation control */
    PKTCTRL0 = 0x08, /* Packet automation control */
    ADDR = 0x09,     /* Device address */
    CHANNR = 0x0A,   /* Channel number */
    FSCTRL1 = 0x0B,  /* Frequency synthesizer control */
    FSCTRL0 = 0x0C,  /* Frequency synthesizer control */
    FREQ2 = 0x0D,    /* Frequency control word, high byte */
    FREQ1 = 0x0E,    /* Frequency control word, middle byte */
    FREQ0 = 0x0F,    /* Frequency control word, low byte */
    MDMCFG4 = 0x10,  /* Modem configuration */
    MDMCFG3 = 0x11,  /* Modem configuration */
    MDMCFG2 = 0x12,  /* Modem configuration */
    MDMCFG1 = 0x13,  /* Modem configuration */
    MDMCFG0 = 0x14,  /* Modem configuration */
    DEVIATN = 0x15,  /* Modem deviation setting */
    MCSM2 = 0x16,    /* Main Radio Control State Machine configuration */
    MCSM1 = 0x17,    /* Main Radio Control State Machine configuration */
    MCSM0 = 0x18,    /* Main Radio Control State Machine configuration */
    FOCCFG = 0x19,   /* Frequency Offset Compensation configuration */
    BSCFG = 0x1A,    /* Bit Synchronization configuration */
    AGCCTRL2 = 0x1B, /* AGC control */
    AGCCTRL1 = 0x1C, /* AGC control */
    AGCCTRL0 = 0x1D, /* AGC control */
    WOREVT1 = 0x1E,  /* High byte Event 0 timeout */
    WOREVT0 = 0x1F,  /* Low byte Event 0 timeout */
    WORCTRL = 0x20,  /* Wake On Radio control */
    FREND1 = 0x21,   /* Front end RX configuration */
    FREND0 = 0x22,   /* Front end TX configuration */
    FSCAL3 = 0x23,   /* Frequency synthesizer calibration */
    FSCAL2 = 0x24,   /* Frequency synthesizer calibration */
    FSCAL1 = 0x25,   /* Frequency synthesizer calibration */
    FSCAL0 = 0x26,   /* Frequency synthesizer calibration */
    RCCTRL1 = 0x27,  /* RC oscillator configuration */
    RCCTRL0 = 0x28,  /* RC oscillator configuration */
    FSTEST = 0x29,   /* Frequency synthesizer calibration control */
    PTEST = 0x2A,    /* Production test */
    AGCTEST = 0x2B,  /* AGC test */
    TEST2 = 0x2C,    /* Various test settings */
    TEST1 = 0x2D,    /* Various test settings */
    TEST0 = 0x2E,    /* Various test settings */

    STATUS_PARTNUM = 0x30,        /* Chip ID Part Number */
    STATUS_VERSION = 0x31,        /* Chip ID Version */
    STATUS_FREQEST = 0x32,        /* Frequency Offset Estimate from Demodulator */
    STATUS_LQI = 0x33,            /* Demodulator Estimate for Link Quality, 7bit-CRC, 6..0-LQI*/
    STATUS_RSSI = 0x34,           /* Received Signal Strength Indication */
    STATUS_MARCSTATE = 0x35,      /* Main Radio Control State Machine State */
    STATUS_WORTIME1 = 0x36,       /* High Byte of WOR Time */
    STATUS_WORTIME0 = 0x37,       /* Low Byte of WOR Time */
    STATUS_PKTSTATUS = 0x38,      /* Current GDOx Status and Packet Status */
    STATUS_VCO_VC_DAC = 0x39,     /* Current Setting from PLL Calibration Module */
    STATUS_TXBYTES = 0x3A,        /* Underflow and Number of Bytes, 7bit-Underflow, 6..0-Number of Bytes*/
    STATUS_RXBYTES = 0x3B,        /* Overflow and Number of Bytes, 7bit-Overflow*, 6..0-Number of Bytes*/
    STATUS_RCCTRL1_STATUS = 0x3C, /* Last RC Oscillator Calibration Result */
    STATUS_RCCTRL0_STATUS = 0x3D, /* Last RC Oscillator Calibration Result */

    /* Some special registers, use CC1101_BURST to read/write data */
    PATABLE = 0x3E, /* PATABLE register number, an 8-byte table that defines the PA control settings */
    FIFO = 0x3F,    /* FIFO register nunmber, can be combined with CC1101_WRITE and/or CC1101_BURST */
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, PartialEq)]
enum State {
    IDLE = 0b000,             /* IDLE state */
    RX = 0b001,               /* Receive mode */
    TX = 0b010,               /* Transmit mode */
    FSTXON = 0b011,           /* Fast TX ready */
    CALIBRATE = 0b100,        /* Frequency synthesizer calibration is running */
    SETTLING = 0b101,         /* PLL is settling */
    RXFIFO_OVERFLOW = 0b110, /* RX FIFO has overflowed. Read out any useful data, then flush the FIFO with SFRX */
    TXFIFO_UNDERFLOW = 0b111, /* TX FIFO has underflowed. Acknowledge with SFTX */
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, PartialEq)]
enum GdoCfg {
    RxFifoThreshold = 0x00,
    RxFifoThresholdOrPacket = 0x01,
    TxFifoThreshold = 0x02,
    TxFifoFull = 0x03,
    RxOverflow = 0x04,
    TxUnderflow = 0x05,
    SyncWord = 0x06,
    Packet = 0x07,
    Preamble = 0x08,
    ClearChannel = 0x09,
    LockDetector = 0x0A,
    SerialClock = 0x0B,
    SerialSynchronousDataOutput = 0x0C,
    SerialDataOutput = 0x0D,
    CarrierSense = 0x0E,
    CrcOk = 0x0F,
    /* Reserved range: 0x10 - 0x15 */
    RxHardData1 = 0x16,
    RxHardData0 = 0x17,
    /* Reserved range: 0x18 - 0x1A */
    PaPd = 0x1B,
    LnaPd = 0x1C,
    RxSymbolTick = 0x1D,
    /* Reserved range: 0x1E - 0x23 */
    WorEvnt0 = 0x24,
    WorEvnt1 = 0x25,
    Clk256 = 0x26,
    Clk32k = 0x27,
    /* Reserved: 0x28 */
    ChpRdyN = 0x29,
    /* Reserved: 0x2A */
    XoscStable = 0x2B,
    /* Reserved range: 0x2C - 0x2D */
    HighImpedance = 0x2E,
    HW = 0x2F,
    /* Only one CC1101IocfgClkXoscN can be selected as an output at any time */
    ClkXosc1 = 0x30,
    ClkXosc1_5 = 0x31,
    ClkXosc2 = 0x32,
    ClkXosc3 = 0x33,
    ClkXosc4 = 0x34,
    ClkXosc6 = 0x35,
    ClkXosc8 = 0x36,
    ClkXosc12 = 0x37,
    ClkXosc16 = 0x38,
    ClkXosc24 = 0x39,
    ClkXosc32 = 0x3A,
    ClkXosc48 = 0x3B,
    ClkXosc64 = 0x3C,
    ClkXosc96 = 0x3D,
    ClkXosc128 = 0x3E,
    ClkXosc192 = 0x3F,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Cc1101Error {
    SpiError(SpiError),
    BadReg,
}

impl From<SpiError> for Cc1101Error {
    fn from(e: SpiError) -> Self {
        Self::SpiError(e)
    }
}

pub struct Cc1101 {
    spi: SpiDevice,
}

impl Cc1101 {
    pub fn new(spi: SpiDevice) -> Self {
        Self { spi }
    }

    fn strobe_command(&self, cmd: Command) -> Result<(), Cc1101Error> {
        let mut status: [u8; 1] = [0];
        self.spi.exchange(&[cmd as u8], &mut status)?;
        Ok(())
    }

    fn read_reg(&self, reg: Register) -> Result<u8, Cc1101Error> {
        let mut cmd = reg as u8 | CC1101_READ;
        if reg >= Register::STATUS_PARTNUM
            && reg <= Register::STATUS_RCCTRL0_STATUS
        {
            cmd |= CC1101_BURST;
        }
        let mut status: [u8; 2] = [0; 2];
        self.spi.exchange(&[cmd, 0], &mut status)?;
        Ok(status[1])
    }

    fn write_reg(&self, reg: Register, val: u8) -> Result<(), Cc1101Error> {
        if reg >= Register::STATUS_PARTNUM {
            return Err(Cc1101Error::BadReg);
        }
        let mut status: [u8; 2] = [0; 2];
        self.spi.exchange(&[reg as u8, val], &mut status)?;
        Ok(())
    }

    fn get_status(&self) -> Result<u8, Cc1101Error> {
        let mut status: [u8; 1] = [0];
        self.spi.exchange(&[Command::SNOP as u8], &mut status)?;
        Ok(status[0])
    }

    pub fn initialize(&self) -> Result<(), Cc1101Error> {
        self.write_reg(Register::IOCFG0, GdoCfg::HighImpedance as u8)?;
        self.switch_to_idle()
    }

    pub fn load_preset(&self) -> Result<(), Cc1101Error> {
        let cfg = [
            /* GPIO GD0 */
            (Register::IOCFG0, 0x0D), // GD0 as async serial data output/input
            /* FIFO and internals */
            (Register::FIFOTHR, 0x07), // The only important bit is ADC_RETENTION
            /* Packet engine */
            (Register::PKTCTRL0, 0x32), // Async, continious, no whitening
            /* Frequency Synthesizer Control */
            (Register::FSCTRL1, 0x06), // IF = (26*10^6) / (2^10) * 0x06 = 152343.75Hz
            // Modem Configuration
            (Register::MDMCFG0, 0x00), // Channel spacing is 25kHz
            (Register::MDMCFG1, 0x00), // Channel spacing is 25kHz
            (Register::MDMCFG2, 0x30), // Format ASK/OOK, No preamble/sync
            (Register::MDMCFG3, 0x32), // Data rate is 3.79372 kBaud
            //(Register::MDMCFG4, 0x17), // Rx BW filter is 650.000kHz
            //(Register::MDMCFG4, 0x6C), // Rx BW filter is 270.000kHz
            (Register::MDMCFG4, 0x8C), // Rx BW filter is 203.000kHz
            //(Register::MDMCFG4, 0xFC), // Rx BW filter is 58.000kHz

            /* Main Radio Control State Machine */
            (Register::MCSM0, 0x18), // Autocalibrate on idle-to-rx/tx, PO_TIMEOUT is 64 cycles(149-155us)
            /* Frequency Offset Compensation Configuration */
            (Register::FOCCFG, 0x18), // no frequency offset compensation, POST_K same as PRE_K, PRE_K is 4K, GATE is off
            /* Automatic Gain Control */
            // (Register::AGCCTRL0, 0x40), // 01 - Low hysteresis, small asymmetric dead zone, medium gain; 00 - 8 samples agc; 00 - Normal AGC, 00 - 4dB boundary
            // (Register::AGCCTRL1, 0x00), // 0; 0 - LNA 2 gain is decreased to minimum before decreasing LNA gain; 00 - Relative carrier sense threshold disabled; 0000 - RSSI to MAIN_TARGET
            // (Register::AGCCTRL2, 0x03), // 00 - DVGA all; 000 - MAX LNA+LNA2; 011 - MAIN_TARGET 24 dB
            //MAGN_TARGET for RX filter BW =< 100 kHz is 0x3. For higher RX filter BW's MAGN_TARGET is 0x7.
            (Register::AGCCTRL0, 0x91), // 10 - Medium hysteresis, medium asymmetric dead zone, medium gain ; 01 - 16 samples agc; 00 - Normal AGC, 01 - 8dB boundary
            (Register::AGCCTRL1, 0x00), // 0; 0 - LNA 2 gain is decreased to minimum before decreasing LNA gain; 00 - Relative carrier sense threshold disabled; 0000 - RSSI to MAIN_TARGET
            (Register::AGCCTRL2, 0x07), // 00 - DVGA all; 000 - MAX LNA+LNA2; 111 - MAIN_TARGET 42 dB
            //(Register::AGCCTRL2, 0xC0),

            /* Wake on radio and timeouts control */
            (Register::WORCTRL, 0xFB), // WOR_RES is 2^15 periods (0.91 - 0.94 s) 16.5 - 17.2 hours
            /* Frontend configuration */
            (Register::FREND0, 0x11), // Adjusts current TX LO buffer + high is PATABLE[1]
            (Register::FREND1, 0xB6), //
            (Register::TEST2, 0x88),
            (Register::TEST1, 0x31),
            (Register::TEST0, 0x09),
        ];

        for (reg, val) in cfg {
            self.write_reg(reg, val)?;
        }
        Ok(())
    }

    pub fn set_frequency(&self, value: u32) -> Result<u32, Cc1101Error> {
        let real_value = value as u64 * CC1101_FDIV / CC1101_QUARTZ;

        // Sanity check
        assert!((real_value & CC1101_FMASK) == real_value);

        self.write_reg(Register::FREQ2, ((real_value >> 16) & 0xFF) as u8)?;
        self.write_reg(Register::FREQ1, ((real_value >> 8) & 0xFF) as u8)?;
        self.write_reg(Register::FREQ0, ((real_value >> 0) & 0xFF) as u8)?;

        let real_frequency = real_value * CC1101_QUARTZ / CC1101_FDIV;

        self.strobe_command(Command::SCAL)?;

        while (self.get_status()? >> 4) & 0b111 != State::IDLE as u8 {}

        Ok(real_frequency as u32)
    }

    pub fn flush_rx(&self) -> Result<(), Cc1101Error> {
        self.strobe_command(Command::SFRX)
    }

    pub fn switch_to_rx(&self, wait: bool) -> Result<(), Cc1101Error> {
        self.strobe_command(Command::SRX)?;
        if wait {
            while (self.get_status()? >> 4) & 0b111 != State::RX as u8 {}
        }
        Ok(())
    }

    pub fn switch_to_idle(&self) -> Result<(), Cc1101Error> {
        self.strobe_command(Command::SIDLE)
    }

    pub fn shutdown(&self) -> Result<(), Cc1101Error> {
        self.strobe_command(Command::SPWD)
    }

    pub fn get_rssi(&self) -> Result<f32, Cc1101Error> {
        let rssi_dec = self.read_reg(Register::STATUS_RSSI)? as f32;
        let rssi = if rssi_dec >= 128.0 {
            (rssi_dec - 256.0) / 2.0 - 74.0
        } else {
            rssi_dec / 2.0 - 74.0
        };
        Ok(rssi)
    }
}
