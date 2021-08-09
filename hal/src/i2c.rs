//! # Inter-Integrated Circuit (I2C) bus

use core::convert::TryFrom;

use crate::{
    embedded_hal::blocking::i2c::{Read, Write, WriteRead},
    gpio::{OutputType, Pull},
    pac::{rcc::ccipr::I2C3SEL_A, I2C1, I2C2, I2C3, RCC},
    rcc::{pclk1_hz, sysclk_hz},
};

use embedded_time::{fixed_point::FixedPoint, rate::*};

/// I2C error
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Arbitration loss
    Arbitration,
    /// Bus error
    Bus,
    /// Bus busy
    Busy,
    /// Not Acknowledge received
    Nack,
    /* Overrun, // slave mode only
     * Pec, // SMBUS mode only
     * Timeout, // SMBUS mode only
     * Alert, // SMBUS mode only */
}

/// I2C peripheral operating in master mode
pub struct I2c1<PINS> {
    i2c: I2C1,
    pins: PINS,
}

pub struct I2c2<PINS> {
    i2c: I2C2,
    pins: PINS,
}

pub struct I2c3<PINS> {
    i2c: I2C3,
    pins: PINS,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident, $variant:ident) => {
        loop {
            let isr = $i2c.isr.read();
            let icr = &$i2c.icr;

            if isr.arlo().is_lost() {
                icr.write(|w| w.arlocf().clear());
                return Err(Error::Arbitration);
            } else if isr.berr().is_error() {
                icr.write(|w| w.berrcf().clear());
                return Err(Error::Bus);
            } else if isr.nackf().is_nack() {
                while $i2c.isr.read().stopf().is_no_stop() {}
                icr.write(|w| w.nackcf().clear());
                icr.write(|w| w.stopcf().clear());
                return Err(Error::Nack);
            } else if isr.$flag().$variant() {
                break;
            }
        }
    };
}

macro_rules! i2c {
    ($($I2cX:ident: ($I2CX:ident, $i2cXen:ident, $i2cXrst:ident, $i2cXsel:ident,
                     $I2cXSda:ident, $I2cXScl:ident, $i2cXsclAf:ident, $i2cXsdaAf:ident),)+) => {
        $(
            impl<SCL, SDA> $I2cX<(SCL, SDA)> {
                /// Enables clock and resets the peripheral
                fn enable_clock(rcc: &mut RCC) {
                    rcc.apb1enr1.modify(|_, w| w.$i2cXen().enabled());
                }

                fn pulse_reset(rcc: &mut RCC) {
                    rcc.apb1rstr1.modify(|_, w| w.$i2cXrst().reset());
                    rcc.apb1rstr1.modify(|_, w| w.$i2cXrst().clear_bit());
                }

                fn clock(rcc: &RCC) -> Hertz {
                    // NOTE(unsafe) atomic read with no side effects
                    match rcc.ccipr.read().$i2cXsel().variant().unwrap() {
                        I2C3SEL_A::HSI16 => Hertz(16_000_000),
                        I2C3SEL_A::SYSCLK => Hertz(sysclk_hz(rcc)), // TODO move the HAL to embedded-time?
                        I2C3SEL_A::PCLK => Hertz(pclk1_hz(rcc)),
                    }
                }

                /// Configures the I2C peripheral as master with the indicated frequency. The implementation takes care
                /// of setting the peripheral to standard/fast mode depending on the indicated frequency and generates
                /// values for SCLL and SCLH durations
                ///
                /// # Panics
                ///
                /// * Frequency is greater than 1 MHz
                /// * Resulting TIMINGR fields PRESC, SCLDEL, SCADEL, SCLH, SCLL are out of range
                pub fn new(i2c: $I2CX, mut pins: (SCL, SDA), freq: Hertz, rcc: &mut RCC, pullup: bool) -> Self
                    where
                    SCL: crate::gpio::sealed::$I2cXScl + crate::gpio::sealed::PinOps,
                    SDA: crate::gpio::sealed::$I2cXSda + crate::gpio::sealed::PinOps,
                    {
                        assert!(freq.integer() <= 1_000_000); // TODO Return Error instead of panic

                        Self::enable_clock(rcc);
                        Self::pulse_reset(rcc);
                        cortex_m::interrupt::free(|cs| unsafe {
                            pins.0.set_output_type(cs, OutputType::OpenDrain);
                            pins.1.set_output_type(cs, OutputType::OpenDrain);
                            pins.0.$i2cXsclAf(cs);
                            pins.1.$i2cXsdaAf(cs);
                            if (pullup) {
                                pins.0.set_pull(cs, Pull::Up);
                                pins.1.set_pull(cs, Pull::Up);
                            } else {
                                pins.0.set_pull(cs, Pull::None);
                                pins.1.set_pull(cs, Pull::None);
                            }
                        });

                        // TODO review compliance with the timing requirements of I2C
                        // t_I2CCLK = 1 / PCLK1
                        // t_PRESC  = (PRESC + 1) * t_I2CCLK
                        // t_SCLL   = (SCLL + 1) * t_PRESC
                        // t_SCLH   = (SCLH + 1) * t_PRESC
                        //
                        // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
                        // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
                        let i2cclk = Self::clock(rcc).0;
                        let ratio = i2cclk / freq.integer() - 4;
                        let (presc, scll, sclh, sdadel, scldel) = if freq >= 100.kHz() {
                            // fast-mode or fast-mode plus
                            // here we pick SCLL + 1 = 2 * (SCLH + 1)
                            let presc = ratio / 387;

                            let sclh = ((ratio / (presc + 1)) - 3) / 3;
                            let scll = 2 * (sclh + 1) - 1;

                            let (sdadel, scldel) = if freq > 400.kHz() {
                                // fast-mode plus
                                let sdadel = 0;
                                let scldel = i2cclk / 4_000_000 / (presc + 1) - 1;

                                (sdadel, scldel)
                            } else {
                                // fast-mode
                                let sdadel = i2cclk / 8_000_000 / (presc + 1);
                                let scldel = i2cclk / 2_000_000 / (presc + 1) - 1;

                                (sdadel, scldel)
                            };

                            (presc, scll, sclh, sdadel, scldel)
                        } else {
                            // standard-mode
                            // here we pick SCLL = SCLH
                            let presc = ratio / 514;

                            let sclh = ((ratio / (presc + 1)) - 2) / 2;
                            let scll = sclh;

                            let sdadel = i2cclk / 2_000_000 / (presc + 1);
                            let scldel = i2cclk / 800_000 / (presc + 1) - 1;

                            (presc, scll, sclh, sdadel, scldel)
                        };

                        assert!(presc < 16);
                        assert!(scldel < 16);
                        assert!(sdadel < 16);
                        let sclh = u8::try_from(sclh).unwrap();
                        let scll = u8::try_from(scll).unwrap();

                        // Configure for "fast mode" (400 KHz)
                        // NOTE(write): writes all non-reserved bits.
                        i2c.timingr.write(|w| {
                            w.presc()
                                .bits(presc as u8)
                                .sdadel()
                                .bits(sdadel as u8)
                                .scldel()
                                .bits(scldel as u8)
                                .scll()
                                .bits(scll)
                                .sclh()
                                .bits(sclh)
                        });

                        // Enable the peripheral
                        i2c.cr1.write(|w| w.pe().set_bit());

                        Self { i2c, pins }
                    }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> Read for $I2cX<PINS> {
                type Error = Error;

                /// Read `buffer.len()` bytes from `addr`
                ///
                /// # Panics
                ///
                /// * Empty buffer (`buffer.len() == 0`)
                fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                    assert!(!buffer.is_empty());

                    // Detect Bus busy
                    if self.i2c.isr.read().busy().is_busy() {
                        return Err(Error::Busy);
                    }

                    let end = buffer.len() / 0xFF;

                    // Process 255 bytes at a time
                    for (i, buffer) in buffer.chunks_mut(0xFF).enumerate() {
                        // Prepare to receive `bytes`
                        self.i2c.cr2.modify(|_, w| {
                            if i == 0 {
                                w.add10().bit7();
                                w.sadd().bits((addr << 1) as u16);
                                w.rd_wrn().read();
                                w.start().start();
                            }
                            w.nbytes().bits(buffer.len() as u8);
                            if i != end {
                                w.reload().not_completed()
                            } else {
                                w.reload().completed().autoend().automatic()
                            }
                        });

                        for byte in buffer {
                            // Wait until we have received something
                            busy_wait!(self.i2c, rxne, is_not_empty);

                            *byte = self.i2c.rxdr.read().rxdata().bits();
                        }

                        if i != end {
                            // Wait until the last transmission is finished
                            busy_wait!(self.i2c, tcr, is_complete);
                        }
                    }

                    // automatic STOP
                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, stopf, is_stop);

                    self.i2c.icr.write(|w| w.stopcf().clear());

                    Ok(())
                }
            }

            impl<PINS> Write for $I2cX<PINS> {
                type Error = Error;

                /// Write `bytes.len()` bytes to `addr`. 0-byte writes are allowed, in which case the master
                /// will just write the address
                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // Detect Bus busy
                    if self.i2c.isr.read().busy().is_busy() {
                        return Err(Error::Busy);
                    }

                    if bytes.is_empty() {
                        // 0 byte write
                        self.i2c.cr2.modify(|_, w| {
                            w.add10().bit7();
                            w.sadd().bits((addr << 1) as u16);
                            w.rd_wrn().write();
                            w.nbytes().bits(0);
                            w.reload().completed();
                            w.autoend().automatic();
                            w.start().start()
                        });
                    } else {
                        let end = bytes.len() / 0xFF;

                        // Process 255 bytes at a time
                        for (i, bytes) in bytes.chunks(0xFF).enumerate() {
                            // Prepare to send `bytes`
                            self.i2c.cr2.modify(|_, w| {
                                if i == 0 {
                                    w.add10().bit7();
                                    w.sadd().bits((addr << 1) as u16);
                                    w.rd_wrn().write();
                                    w.start().start();
                                }
                                w.nbytes().bits(bytes.len() as u8);
                                if i != end {
                                    w.reload().not_completed()
                                } else {
                                    w.reload().completed().autoend().automatic()
                                }
                            });

                            for byte in bytes {
                                // Wait until we are allowed to send data
                                // (START has been ACKed or last byte went through)
                                busy_wait!(self.i2c, txis, is_empty);

                                // Put byte on the wire
                                // NOTE(write): Writes all non-reserved bits.
                                self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                            }

                            if i != end {
                                // Wait until the last transmission is finished
                                busy_wait!(self.i2c, tcr, is_complete);
                            }
                        }
                    }

                    // automatic STOP
                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, stopf, is_stop);

                    self.i2c.icr.write(|w| w.stopcf().clear());

                    Ok(())
                }
            }

            impl<PINS> WriteRead for $I2cX<PINS> {
                type Error = Error;

                /// Write `bytes.len()` bytes to `addr` and read back `buffer.len()` bytes.
                ///
                /// # Panics
                ///
                /// * `bytes` or `buffer` are empty (use `write` for 0-byte writes)
                fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
                    assert!(!bytes.is_empty() && !buffer.is_empty());

                    // Detect Bus busy
                    if self.i2c.isr.read().busy().is_busy() {
                        return Err(Error::Busy);
                    }

                    let end = bytes.len() / 0xFF;

                    // Process 255 bytes at a time
                    for (i, bytes) in bytes.chunks(0xFF).enumerate() {
                        // Prepare to send `bytes`
                        self.i2c.cr2.modify(|_, w| {
                            if i == 0 {
                                w.add10().bit7();
                                w.sadd().bits((addr << 1) as u16);
                                w.rd_wrn().write();
                                w.start().start();
                            }
                            w.nbytes().bits(bytes.len() as u8);
                            if i != end {
                                w.reload().not_completed()
                            } else {
                                w.reload().completed().autoend().software()
                            }
                        });

                        for byte in bytes {
                            // Wait until we are allowed to send data
                            // (START has been ACKed or last byte went through)
                            busy_wait!(self.i2c, txis, is_empty);

                            // Put byte on the wire
                            // NOTE(write): Writes all non-reserved bits.
                            self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                        }

                        if i != end {
                            // Wait until the last transmission is finished
                            busy_wait!(self.i2c, tcr, is_complete);
                        }
                    }

                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, tc, is_complete);

                    // restart

                    let end = buffer.len() / 0xFF;

                    // Process 255 bytes at a time
                    for (i, buffer) in buffer.chunks_mut(0xFF).enumerate() {
                        // Prepare to receive `bytes`
                        self.i2c.cr2.modify(|_, w| {
                            if i == 0 {
                                w.add10().bit7();
                                w.sadd().bits((addr << 1) as u16);
                                w.rd_wrn().read();
                                w.start().start();
                            }
                            w.nbytes().bits(buffer.len() as u8);
                            if i != end {
                                w.reload().not_completed()
                            } else {
                                w.reload().completed().autoend().automatic()
                            }
                        });

                        for byte in buffer {
                            // Wait until we have received something
                            busy_wait!(self.i2c, rxne, is_not_empty);

                            *byte = self.i2c.rxdr.read().rxdata().bits();
                        }

                        if i != end {
                            // Wait until the last transmission is finished
                            busy_wait!(self.i2c, tcr, is_complete);
                        }
                    }

                    // automatic STOP
                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, stopf, is_stop);

                    self.i2c.icr.write(|w| w.stopcf().clear());

                    Ok(())
                }
            }
        )+
    };

    ([ $($X:literal),+ ]) => {
        paste::paste! {
            i2c!(
                $([<I2c $X>]: ([<I2C $X>], [<i2c $X en>], [<i2c $X rst>], [<i2c $X sel>], [<I2c $X Sda>],
                    [<I2c $X Scl>], [<set_i2c $X _scl_af>], [<set_i2c $X _sda_af>]),)+
            );
        }
    };
}

i2c!([1, 2, 3]);
