//! Inter-Integrated Circuit (I2C) bus

use crate::{
    embedded_hal::blocking::i2c::{Read, Write, WriteRead},
    gpio::{OutputType, Pull},
    pac::{self, rcc::ccipr::I2C3SEL_A, I2C1, I2C2, I2C3, RCC},
    rcc::{pclk1_hz, sysclk_hz},
};

use cortex_m::interrupt::CriticalSection;

/// I2C error
#[derive(Debug)]
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

/// I2C1 peripheral operating in master mode
#[derive(Debug)]
pub struct I2c1<PINS> {
    base: I2C1,
    pins: PINS,
}

/// I2C2 peripheral operating in master mode
#[derive(Debug)]
pub struct I2c2<PINS> {
    base: I2C2,
    pins: PINS,
}

/// I2C3 peripheral operating in master mode
#[derive(Debug)]
pub struct I2c3<PINS> {
    base: I2C3,
    pins: PINS,
}

macro_rules! busy_wait {
    ($self:ident, $flag:ident, $variant:ident) => {
        loop {
            let isr = $self.isr().read();
            let icr = $self.icr();

            if isr.arlo().is_lost() {
                icr.write(|w| w.arlocf().clear());
                return Err(Error::Arbitration);
            // Bus error should be ignored during Master mode. See STM doc. nr. ES0500 (Erratum).
            // Leaving this code here in case it can be used when implementing slave mode
            // } else if isr.berr().is_error() {
            //     icr.write(|w| w.berrcf().clear());
            //     return Err(Error::Bus);
            } else if isr.nackf().is_nack() {
                while $self.isr().read().stopf().is_no_stop() {}
                icr.write(|w| w.nackcf().clear());
                icr.write(|w| w.stopcf().clear());
                return Err(Error::Nack);
            } else if isr.$flag().$variant() {
                break;
            }
        }
    };
}

trait I2cBase {
    fn cr1(&self) -> &pac::i2c1::CR1;
    fn cr2(&self) -> &pac::i2c1::CR2;
    fn icr(&self) -> &pac::i2c1::ICR;
    fn isr(&self) -> &pac::i2c1::ISR;
    fn oar1(&self) -> &pac::i2c1::OAR1;
    fn oar2(&self) -> &pac::i2c1::OAR2;
    fn pecr(&self) -> &pac::i2c1::PECR;
    fn rxdr(&self) -> &pac::i2c1::RXDR;
    fn timeoutr(&self) -> &pac::i2c1::TIMEOUTR;
    fn timingr(&self) -> &pac::i2c1::TIMINGR;
    fn txdr(&self) -> &pac::i2c1::TXDR;

    /// Read `buffer.len()` bytes from `addr`
    ///
    /// # Panics
    ///
    /// * Empty buffer (`buffer.len() == 0`)
    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        assert!(!buffer.is_empty());

        // Detect Bus busy
        if self.isr().read().busy().is_busy() {
            return Err(Error::Busy);
        }

        let end = buffer.len() / 0xFF;

        // Process 255 bytes at a time
        for (i, buffer) in buffer.chunks_mut(0xFF).enumerate() {
            // Prepare to receive `bytes`
            self.cr2().modify(|_, w| {
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
                busy_wait!(self, rxne, is_not_empty);

                *byte = self.rxdr().read().rxdata().bits();
            }

            if i != end {
                // Wait until the last transmission is finished
                busy_wait!(self, tcr, is_complete);
            }
        }

        // automatic STOP
        // Wait until the last transmission is finished
        busy_wait!(self, stopf, is_stop);

        self.icr().write(|w| w.stopcf().clear());

        Ok(())
    }

    /// Write `bytes.len()` bytes to `addr`. 0-byte writes are allowed, in which case the master
    /// will just write the address
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Detect Bus busy
        if self.isr().read().busy().is_busy() {
            return Err(Error::Busy);
        }

        if bytes.is_empty() {
            // 0 byte write
            self.cr2().modify(|_, w| {
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
                self.cr2().modify(|_, w| {
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
                    busy_wait!(self, txis, is_empty);

                    // Put byte on the wire
                    // NOTE(write): Writes all non-reserved bits.
                    self.txdr().write(|w| w.txdata().bits(*byte));
                }

                if i != end {
                    // Wait until the last transmission is finished
                    busy_wait!(self, tcr, is_complete);
                }
            }
        }

        // automatic STOP
        // Wait until the last transmission is finished
        busy_wait!(self, stopf, is_stop);

        self.icr().write(|w| w.stopcf().clear());

        Ok(())
    }

    /// Write `bytes.len()` bytes to `addr` and read back `buffer.len()` bytes.
    ///
    /// # Panics
    ///
    /// * `bytes` or `buffer` are empty (use `write` for 0-byte writes)
    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        assert!(!bytes.is_empty() && !buffer.is_empty());

        // Detect Bus busy
        if self.isr().read().busy().is_busy() {
            return Err(Error::Busy);
        }

        let end = bytes.len() / 0xFF;

        // Process 255 bytes at a time
        for (i, bytes) in bytes.chunks(0xFF).enumerate() {
            // Prepare to send `bytes`
            self.cr2().modify(|_, w| {
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
                busy_wait!(self, txis, is_empty);

                // Put byte on the wire
                // NOTE(write): Writes all non-reserved bits.
                self.txdr().write(|w| w.txdata().bits(*byte));
            }

            if i != end {
                // Wait until the last transmission is finished
                busy_wait!(self, tcr, is_complete);
            }
        }

        // Wait until the last transmission is finished
        busy_wait!(self, tc, is_complete);

        // restart

        let end = buffer.len() / 0xFF;

        // Process 255 bytes at a time
        for (i, buffer) in buffer.chunks_mut(0xFF).enumerate() {
            // Prepare to receive `bytes`
            self.cr2().modify(|_, w| {
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
                busy_wait!(self, rxne, is_not_empty);

                *byte = self.rxdr().read().rxdata().bits();
            }

            if i != end {
                // Wait until the last transmission is finished
                busy_wait!(self, tcr, is_complete);
            }
        }

        // automatic STOP
        // Wait until the last transmission is finished
        busy_wait!(self, stopf, is_stop);

        self.icr().write(|w| w.stopcf().clear());

        Ok(())
    }
}

#[rustfmt::skip]
macro_rules! impl_i2c_base_for {
    ($($name:ident)+) => {
        $(
        impl I2cBase for $name {
            #[inline(always)] fn cr1(&self) -> &pac::i2c1::CR1 { &self.cr1 }
            #[inline(always)] fn cr2(&self) -> &pac::i2c1::CR2 { &self.cr2 }
            #[inline(always)] fn icr(&self) -> &pac::i2c1::ICR { &self.icr }
            #[inline(always)] fn isr(&self) -> &pac::i2c1::ISR { &self.isr }
            #[inline(always)] fn oar1(&self) -> &pac::i2c1::OAR1 { &self.oar1 }
            #[inline(always)] fn oar2(&self) -> &pac::i2c1::OAR2 { &self.oar2 }
            #[inline(always)] fn pecr(&self) -> &pac::i2c1::PECR { &self.pecr }
            #[inline(always)] fn rxdr(&self) -> &pac::i2c1::RXDR { &self.rxdr }
            #[inline(always)] fn timeoutr(&self) -> &pac::i2c1::TIMEOUTR { &self.timeoutr }
            #[inline(always)] fn timingr(&self) -> &pac::i2c1::TIMINGR { &self.timingr }
            #[inline(always)] fn txdr(&self) -> &pac::i2c1::TXDR { &self.txdr }
        }
        )+
    };
}

macro_rules! impl_clocks_reset {
    ($($I2cX:ident: ($i2cXen:ident, $i2cXrst:ident, $i2cXsel:ident),)+) => {
        $(
            impl<SCL, SDA> $I2cX<(SCL, SDA)> {
                /// Enables peripheral clock
                fn enable_clock(rcc: &mut RCC) {
                    rcc.apb1enr1.modify(|_, w| w.$i2cXen().enabled());
                }

                /// Resets peripheral clock
                fn pulse_reset(rcc: &mut RCC) {
                    rcc.apb1rstr1.modify(|_, w| w.$i2cXrst().reset());
                    rcc.apb1rstr1.modify(|_, w| w.$i2cXrst().clear_bit());
                }

                /// Returns the frequency of the peripheral clock driver
                fn clock(rcc: &RCC) -> u32 {
                    // NOTE(unsafe) atomic read with no side effects
                    match rcc.ccipr.read().$i2cXsel().variant().unwrap() {
                        I2C3SEL_A::Hsi16 => 16_000_000,
                        I2C3SEL_A::Sysclk => sysclk_hz(rcc),
                        I2C3SEL_A::Pclk => pclk1_hz(rcc),
                    }
                }
            }
        )+
    }
}

macro_rules! impl_new_free {
    ($($I2cX:ident: ($I2CX:ident, $i2cXen:ident, $i2cXrst:ident, $i2cXsel:ident,
                     $I2cXSda:ident, $I2cXScl:ident, $i2cXsclAf:ident, $i2cXsdaAf:ident),)+) => {
        $(
            impl<SCL, SDA> $I2cX<(SCL, SDA)> {
                /// Configures the I2C peripheral as master with the indicated frequency. The implementation takes care
                /// of setting the peripheral to standard/fast mode depending on the indicated frequency and generates
                /// values for SCLL and SCLH durations
                ///
                /// # Panics
                ///
                /// * Frequency is greater than 1 MHz
                /// * Resulting TIMINGR fields PRESC, SCLDEL, SCADEL, SCLH, SCLL are out of range
                pub fn new(i2c: $I2CX, mut pins: (SCL, SDA), freq_hz: u32, rcc: &mut RCC, pullup: bool, cs: &CriticalSection) -> Self
                    where
                    SCL: crate::gpio::sealed::$I2cXScl + crate::gpio::sealed::PinOps,
                    SDA: crate::gpio::sealed::$I2cXSda + crate::gpio::sealed::PinOps,
                    {
                        assert!(freq_hz <= 1_000_000); // TODO Return Error instead of panic

                        Self::enable_clock(rcc);
                        Self::pulse_reset(rcc);

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

                        let (presc, scll, sclh, sdadel, scldel) = i2c_clocks(Self::clock(rcc), freq_hz);

                        // Configure for "fast mode" (400 KHz)
                        // NOTE(write): writes all non-reserved bits.
                        i2c.timingr.write(|w| {
                            w.presc()
                                .bits(presc)
                                .sdadel()
                                .bits(sdadel)
                                .scldel()
                                .bits(scldel)
                                .scll()
                                .bits(scll)
                                .sclh()
                                .bits(sclh)
                        });

                        // Enable the peripheral
                        i2c.cr1.write(|w| w.pe().set_bit());

                        Self { base: i2c, pins }
                    }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.base, self.pins)
                }
            }
        )+
    }
}

macro_rules! impl_read {
    ($($I2cX:ident)+) => {
        $(
            impl<PINS> Read for $I2cX<PINS> {
                type Error = Error;

                fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                    self.base.read(addr, buffer)
                }
            }
        )+
    }
}

macro_rules! impl_write {
    ($($I2cX:ident)+) => {
        $(
            impl<PINS> Write for $I2cX<PINS> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                    self.base.write(addr, bytes)
                }
            }
        )+
    }
}

macro_rules! impl_write_read {
    ($($I2cX:ident)+) => {
        $(
            impl<PINS> WriteRead for $I2cX<PINS> {
                type Error = Error;

                fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
                    self.base.write_read(addr, bytes, buffer)
                }
            }
        )+
    }
}

macro_rules! i2c {
    ([ $($X:literal),+ ]) => {
        paste::paste! {
            impl_i2c_base_for!($([<I2C $X>])+);
            impl_clocks_reset!($([<I2c $X>]: ([<i2c $X en>], [<i2c $X rst>], [<i2c $X sel>]),)+);
            impl_new_free!($([<I2c $X>]: ([<I2C $X>], [<i2c $X en>], [<i2c $X rst>], [<i2c $X sel>], [<I2c $X Sda>],
                                    [<I2c $X Scl>], [<set_i2c $X _scl_af>], [<set_i2c $X _sda_af>]),)+);
            impl_read!($([<I2c $X>])+);
            impl_write!($([<I2c $X>])+);
            impl_write_read!($([<I2c $X>])+);
        }
    };
}

i2c!([1, 2, 3]);

// TODO review compliance with the timing requirements of I2C
// t_I2CCLK = 1 / PCLK1
// t_PRESC  = (PRESC + 1) * t_I2CCLK
// t_SCLL   = (SCLL + 1) * t_PRESC
// t_SCLH   = (SCLH + 1) * t_PRESC
//
// t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
// t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
/// Returns the I2C parameters necessary to configure the peripheral.
///
/// # Parameters
/// * clock: the frequency of the clock driving the I2C peripheral
/// * freq: the desired frequency for the I2C peripheral
///
/// # Returns:
/// * PRESC
/// * SCLL
/// * SCLH
/// * SDADEL
/// * SCLDEL
fn i2c_clocks(clock_hz: u32, freq_hz: u32) -> (u8, u8, u8, u8, u8) {
    let i2cclk = clock_hz;
    let ratio = i2cclk / freq_hz - 4;
    let (presc, scll, sclh, sdadel, scldel) = if freq_hz >= 100_000 {
        // fast-mode or fast-mode plus
        // here we pick SCLL + 1 = 2 * (SCLH + 1)
        let presc = ratio / 387;

        let sclh = ((ratio / (presc + 1)) - 3) / 3;
        let scll = 2 * (sclh + 1) - 1;

        let (sdadel, scldel) = if freq_hz > 400_000 {
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

    (presc as u8, scll, sclh, sdadel as u8, scldel as u8)
}
