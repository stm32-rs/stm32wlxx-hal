//! True random number generator

use crate::pac;

use core::{
    mem::transmute,
    num::NonZeroU32,
    sync::atomic::{compiler_fence, Ordering::SeqCst},
};

/// RNG trait abstractions.
pub use rand_core;

/// RNG error types
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub enum Error {
    /// A noise error (seed error) occured and automatic correction failed.
    UncorrectableNoise,
    /// RNG frequency is too low.
    ///
    /// Check that the RNG is configured correctly.
    Clock,
}

impl From<Error> for rand_core::Error {
    fn from(e: Error) -> Self {
        match e {
            Error::UncorrectableNoise => NonZeroU32::new(1).unwrap().into(),
            Error::Clock => NonZeroU32::new(2).unwrap().into(),
        }
    }
}

/// RNG clock source selection.
#[derive(Debug, PartialEq, Eq, Hash)]
pub enum ClkSrc {
    /// PLL "Q" clock (PLLQCLK) selected.
    Pll = 0b00,
    /// LSI clock selected.
    Lsi = 0b01,
    /// LSE clock selected.
    Lse = 0b10,
    /// MSI clock selected.
    Msi = 0b11,
}

/// RNG driver.
pub struct Rng {
    rng: pac::RNG,
    err_cnt: u32,
}

impl Rng {
    /// Create a new `Rng` driver from a RNG peripheral.
    ///
    /// This will enable clocks and reset the RNG peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     pac,
    ///     rng::{ClkSrc, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::Msi);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    /// ```
    pub fn new(rng: pac::RNG, rcc: &mut pac::RCC) -> Rng {
        Self::enable_clock(rcc);
        rcc.ahb3rstr.modify(|_, w| w.rngrst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.rngrst().clear_bit());

        // RNG configuration A
        // see table 131 "RNG configurations" in the reference manual
        #[rustfmt::skip]
        rng.cr.write(|w| unsafe {
            w
                .condrst().set_bit()
                .nistc().set_bit()
                .rng_config1().bits(0x0F)
                .clkdiv().bits(0x0)
                .rng_config2().bits(0x0)
                .rng_config3().bits(0xD)
                .ced().clear_bit()
                .ie().clear_bit() // interrupt enable
                .rngen().set_bit()
        });

        #[rustfmt::skip]
        rng.cr.write(|w| unsafe {
            w
                .condrst().clear_bit()
                .nistc().set_bit()
                .rng_config1().bits(0x0F)
                .clkdiv().bits(0x0)
                .rng_config2().bits(0x0)
                .rng_config3().bits(0xD)
                .ced().clear_bit()
                .ie().clear_bit() // interrupt enable
                .rngen().set_bit()
        });

        // when CONDRST is set to 0 by software its value goes to 0 when the
        // reset process is done.
        // It takes about 2 AHB clock cycles + 2 RNG clock cycles
        while rng.cr.read().condrst().bit_is_set() {
            compiler_fence(SeqCst);
        }

        Rng { rng, err_cnt: 0 }
    }

    /// Free the RNG peripheral from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{pac, rng::Rng};
    ///
    /// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rcc: pac::RCC = dp.RCC;
    /// let rng_dp: pac::RNG = dp.RNG;
    ///
    /// let mut rng = Rng::new(rng_dp, &mut rcc);
    /// // ... use rng
    /// let rng_dp: pac::RNG = rng.free();
    /// ```
    pub fn free(self) -> pac::RNG {
        self.rng
    }

    /// Steal the RNG peripheral from whatever is currently using it.
    ///
    /// This will **not** initialize the RNG peripheral (unlike [`new`]).
    ///
    /// # Safety
    ///
    /// This will create a new RNG peripheral, bypassing the singleton checks
    /// that normally occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// the RNG peripheral.
    /// You are also responsible for ensuring the RNG peripheral has been setup
    /// correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::rng::Rng;
    ///
    /// // ... setup happens here
    ///
    /// let rng = unsafe { Rng::steal() };
    /// ```
    ///
    /// [`new`]: Rng::new
    pub unsafe fn steal() -> Rng {
        let dp: pac::Peripherals = pac::Peripherals::steal();
        Rng {
            rng: dp.RNG,
            err_cnt: 0,
        }
    }

    /// Disable the RNG clock.
    pub fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.rngen().disabled());
    }

    /// Enable the RNG clock.
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.rngen().enabled());
        rcc.ahb3enr.read(); // delay after an RCC peripheral clock enabling
    }

    /// Set the RNG clock source.
    pub fn set_clock_source(rcc: &mut pac::RCC, src: ClkSrc) {
        match src {
            ClkSrc::Pll => rcc.ccipr.modify(|_, w| w.rngsel().pllq()),
            ClkSrc::Lsi => rcc.ccipr.modify(|_, w| w.rngsel().lsi()),
            ClkSrc::Lse => rcc.ccipr.modify(|_, w| w.rngsel().lse()),
            ClkSrc::Msi => rcc.ccipr.modify(|_, w| w.rngsel().msi()),
        }
    }

    /// Returns the number of correctable noise errors that have occured.
    ///
    /// This counter will saturate when it hits the maximum value.
    pub fn noise_error_stat(&self) -> u32 {
        self.err_cnt
    }

    /// Reset the correctable noise error counter to zero.
    pub fn reset_noise_error_stat(&mut self) {
        self.err_cnt = 0
    }

    fn wait_for_new_entropy(&mut self) -> Result<(), Error> {
        loop {
            let sr = self.rng.sr.read();
            if sr.seis().bit_is_set() {
                self.recover_from_noise_error()?;
            } else if sr.ceis().bit_is_set() {
                return Err(Error::Clock);
            } else if sr.drdy().bit_is_set() {
                return Ok(());
            }
        }
    }

    /// Try to fill the destination buffer with random data.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     pac,
    ///     rng::{ClkSrc, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::Msi);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    ///
    /// let mut nonce: [u32; 4] = [0; 4];
    /// rng.try_fill_u32(&mut nonce)?;
    /// # Ok::<(), stm32wl_hal::rng::Error>(())
    /// ```
    pub fn try_fill_u32(&mut self, dest: &mut [u32]) -> Result<(), Error> {
        for chunk in dest.chunks_mut(4) {
            self.wait_for_new_entropy()?;
            for dw in chunk {
                *dw = self.rng.dr.read().bits();
            }
        }
        Ok(())
    }

    /// Try to fill the destination buffer with random data.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     pac,
    ///     rng::{ClkSrc, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::Msi);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    ///
    /// let mut nonce: [u8; 16] = [0; 16];
    /// rng.try_fill_u8(&mut nonce)?;
    /// # Ok::<(), stm32wl_hal::rng::Error>(())
    /// ```
    pub fn try_fill_u8(&mut self, dest: &mut [u8]) -> Result<(), Error> {
        for chunk in dest.chunks_mut(16) {
            let mut entropy: [u32; 4] = [0; 4];
            self.try_fill_u32(&mut entropy)?;

            chunk
                .iter_mut()
                // safety: random data has no requirements for safe transmute
                .zip(unsafe { transmute::<[u32; 4], [u8; 16]>(entropy) }.iter())
                .for_each(|(buf_u8, entropy_u8)| *buf_u8 = *entropy_u8);
        }

        Ok(())
    }

    // Reference manual section 22.3.7 "Error management"
    fn recover_from_noise_error(&mut self) -> Result<(), Error> {
        // software reset by writing CONDRST
        self.rng.cr.modify(|_, w| w.condrst().set_bit());
        self.rng.cr.modify(|_, w| w.condrst().clear_bit());

        // when CONDRST is set to 0 by software its value goes to 0 when the
        // reset process is done.
        // It takes about 2 AHB clock cycles + 2 RNG clock cycles
        while self.rng.cr.read().condrst().bit_is_set() {
            compiler_fence(SeqCst);
        }

        loop {
            let sr = self.rng.sr.read();
            if sr.seis().bit_is_set() {
                return Err(Error::UncorrectableNoise);
            }
            if sr.secs().bit_is_clear() {
                self.err_cnt = self.err_cnt.saturating_add(1);
                return Ok(());
            }
        }
    }
}

impl rand_core::RngCore for Rng {
    /// Not recommended for use, panics upon errors.
    fn next_u32(&mut self) -> u32 {
        let mut dws: [u32; 1] = [0; 1];
        self.try_fill_u32(&mut dws).unwrap();
        dws[0]
    }

    /// Not recommended for use, panics upon errors.
    fn next_u64(&mut self) -> u64 {
        let mut bytes: [u8; 8] = [0; 8];
        self.try_fill_u8(&mut bytes).unwrap();
        u64::from_le_bytes(bytes)
    }

    /// Not recommended for use, panics upon errors.
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.try_fill_u8(dest).unwrap()
    }

    /// Use this method if using the `RngCore` for `CryptoRng` traits.
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.try_fill_u8(dest)?;
        Ok(())
    }
}

impl rand_core::CryptoRng for Rng {}
