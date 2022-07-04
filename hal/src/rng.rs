//! True random number generator
//!
//! # Performance
//!
//! Comparison with the [chacha] algorithm seeded by hardware.
//!
//! | Source        | Cycles per `[u32; 4]` |
//! |---------------|-----------------------|
//! | `ChaCha20Rng` | 2,875                 |
//! | `ChaCha12Rng` | 1,764                 |
//! | `ChaCha8Rng`  | 1,216                 |
//! | HW            | 410                   |
//!
//! [chacha]: https://crates.io/crates/chacha20

use crate::pac;

use core::num::NonZeroU32;

/// RNG trait abstractions
pub use rand_core;

/// RNG error types
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// A seed error (noise error) occurred.
    ///
    /// **Note:** There is automatic correction for seed errors included.
    /// This error means that automatic correction was attempted, but failed.
    Seed,
    /// RNG frequency is too low.
    ///
    /// Check that the RNG is configured correctly.
    Clock,
}

impl From<Error> for rand_core::Error {
    fn from(e: Error) -> Self {
        match e {
            // safety: 1 is non-zero
            Error::Seed => unsafe { NonZeroU32::new_unchecked(1) }.into(),
            // safety: 2 is non-zero
            Error::Clock => unsafe { NonZeroU32::new_unchecked(2) }.into(),
        }
    }
}

pub use pac::rcc::ccipr::RNGSEL_A as Clk;

/// RNG driver.
#[derive(Debug)]
pub struct Rng {
    rng: pac::RNG,
    err_cnt: u32,
}

impl Rng {
    /// Create a new `Rng` driver from a RNG peripheral.
    ///
    /// This will select the clock source, enable clocks, and reset the RNG
    /// peripheral.
    ///
    /// This will **NOT** enable the selected clock source, when in doubt use
    /// MSI because it is enabled with power-on-reset.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rng::{Clk, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let mut rng = Rng::new(dp.RNG, Clk::Msi, &mut dp.RCC);
    /// ```
    pub fn new(rng: pac::RNG, clk: Clk, rcc: &mut pac::RCC) -> Rng {
        rcc.ccipr.modify(|_, w| w.rngsel().variant(clk));
        Self::enable_clock(rcc);
        rcc.ahb3rstr.modify(|_, w| w.rngrst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.rngrst().clear_bit());

        // RNG configuration A
        // see table 131 "RNG configurations" in the reference manual
        rng.cr.write(|w| {
            w.condrst().set_bit();
            w.nistc().set_bit();
            w.rng_config1().config_a();
            w.clkdiv().bits(0x0);
            w.rng_config2().config_a_b();
            w.rng_config3().config_a();
            w.ced().enabled();
            w.ie().disabled(); // interrupt enable
            w.rngen().set_bit()
        });

        rng.cr.write(|w| {
            w.condrst().clear_bit();
            w.nistc().set_bit();
            w.rng_config1().config_a();
            w.clkdiv().bits(0x0);
            w.rng_config2().config_a_b();
            w.rng_config3().config_a();
            w.ced().clear_bit();
            w.ie().disabled(); // interrupt enable
            w.rngen().set_bit()
        });

        // when CONDRST is set to 0 by software its value goes to 0 when the
        // reset process is done.
        // It takes about 2 AHB clock cycles + 2 RNG clock cycles
        while rng.cr.read().condrst().bit_is_set() {}

        Rng { rng, err_cnt: 0 }
    }

    /// Free the RNG peripheral from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rng::{Clk, Rng},
    /// };
    ///
    /// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rcc: pac::RCC = dp.RCC;
    /// let rng_dp: pac::RNG = dp.RNG;
    ///
    /// let mut rng = Rng::new(rng_dp, Clk::Msi, &mut rcc);
    /// // ... use rng
    /// let rng_dp: pac::RNG = rng.free();
    /// ```
    #[inline]
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
    /// This will also reset the correctable noise error counter.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rng::Rng;
    ///
    /// // ... setup happens here
    ///
    /// let rng = unsafe { Rng::steal() };
    /// ```
    ///
    /// [`new`]: Rng::new
    #[inline]
    pub unsafe fn steal() -> Rng {
        let dp: pac::Peripherals = pac::Peripherals::steal();
        Rng {
            rng: dp.RNG,
            err_cnt: 0,
        }
    }

    /// Unmask the RNG IRQ in the NVIC.
    ///
    /// # Safety
    ///
    /// This can break mask-based critical sections.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    /// unsafe { stm32wlxx_hal::rng::Rng::unmask_irq() };
    /// ```
    #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    #[inline]
    pub unsafe fn unmask_irq() {
        pac::NVIC::unmask(pac::Interrupt::TRUE_RNG)
    }

    /// Disable the RNG clock.
    ///
    /// # Safety
    ///
    /// 1. You are responsible for ensuring the RNG is in a state where the
    ///    clock can be disabled without entering an error state.
    /// 2. You cannot use the RNG bus while the clock is disabled.
    /// 3. You are responsible for re-enabling the clock before resuming use
    ///    of the RNG.
    /// 4. You are responsible for setting up anything that may have lost state
    ///    while the clock was disabled.
    #[inline]
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.rngen().disabled());
    }

    /// Enable the RNG clock.
    #[inline]
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.rngen().enabled());
        rcc.ahb3enr.read(); // delay after an RCC peripheral clock enabling
    }

    /// Returns the number of correctable seed errors that have occurred.
    ///
    /// This counter will saturate when it hits the maximum value.
    #[inline]
    pub fn seed_error_stat(&self) -> u32 {
        self.err_cnt
    }

    /// Reset the correctable seed error counter to zero.
    #[inline]
    pub fn reset_seed_error_stat(&mut self) {
        self.err_cnt = 0
    }

    fn poll_data_valid(&mut self) -> Result<(), Error> {
        loop {
            let sr = self.rng.sr.read();
            if sr.drdy().bit_is_set() {
                return Ok(());
            } else if sr.secs().bit_is_set() {
                self.recover_from_noise_error()?;
            } else if sr.cecs().bit_is_set() {
                return Err(Error::Clock);
            }
        }
    }

    /// Try to fill the destination buffer with random data.
    ///
    /// This is the native data size for the RNG.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rng::{Clk, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rng = Rng::new(dp.RNG, Clk::Msi, &mut dp.RCC);
    ///
    /// let mut nonce: [u32; 4] = [0; 4];
    /// rng.try_fill_u32(&mut nonce)?;
    /// # Ok::<(), stm32wlxx_hal::rng::Error>(())
    /// ```
    pub fn try_fill_u32(&mut self, dest: &mut [u32]) -> Result<(), Error> {
        for dw in dest {
            *dw = self.try_u32()?;
        }
        Ok(())
    }

    /// Try to fill the destination buffer with random data.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rng::{Clk, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rng = Rng::new(dp.RNG, Clk::Msi, &mut dp.RCC);
    ///
    /// let mut nonce: [u8; 16] = [0; 16];
    /// rng.try_fill_u8(&mut nonce)?;
    /// # Ok::<(), stm32wlxx_hal::rng::Error>(())
    /// ```
    pub fn try_fill_u8(&mut self, dest: &mut [u8]) -> Result<(), Error> {
        for chunk in dest.chunks_mut(4) {
            let mut entropy: [u32; 1] = [0];
            self.try_fill_u32(&mut entropy)?;

            chunk
                .iter_mut()
                .enumerate()
                .for_each(|(idx, byte)| *byte = entropy[0].to_be_bytes()[idx])
        }

        Ok(())
    }

    /// Try to generate a random `u8`.
    ///
    /// This is not efficient if you need to generate a lot of entropy, this is
    /// provided as a convenience function that wraps
    /// [`try_fill_u8`](crate::rng::Rng::try_fill_u8)
    /// for testing and prototyping.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rng::{Clk, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rng = Rng::new(dp.RNG, Clk::Msi, &mut dp.RCC);
    ///
    /// let rand_value: u8 = rng.try_u8()?;
    /// # Ok::<(), stm32wlxx_hal::rng::Error>(())
    /// ```
    pub fn try_u8(&mut self) -> Result<u8, Error> {
        Ok(self.try_u32()? as u8)
    }

    /// Try to generate a random `u16`.
    ///
    /// This is not efficient if you need to generate a lot of entropy, this is
    /// provided as a convenience function that wraps
    /// [`try_fill_u8`](crate::rng::Rng::try_fill_u8)
    /// for testing and prototyping.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rng::{Clk, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rng = Rng::new(dp.RNG, Clk::Msi, &mut dp.RCC);
    ///
    /// let rand_value: u16 = rng.try_u16()?;
    /// # Ok::<(), stm32wlxx_hal::rng::Error>(())
    /// ```
    pub fn try_u16(&mut self) -> Result<u16, Error> {
        Ok(self.try_u32()? as u16)
    }

    /// Try to generate a random `u32`.
    ///
    /// This is not efficient if you need to generate a lot of entropy, this is
    /// provided as a convenience function that wraps
    /// [`try_fill_u32`](crate::rng::Rng::try_fill_u32)
    /// for testing and prototyping.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rng::{Clk, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rng = Rng::new(dp.RNG, Clk::Msi, &mut dp.RCC);
    ///
    /// let rand_value: u32 = rng.try_u32()?;
    /// # Ok::<(), stm32wlxx_hal::rng::Error>(())
    /// ```
    pub fn try_u32(&mut self) -> Result<u32, Error> {
        // reference manual recommends verifying DR is non-zero for
        // **each** read to DR incase there is a seed error between
        // polling SR and reading DR
        self.poll_data_valid()?;
        Ok(self.rng.dr.read().bits())
    }

    /// Try to generate a random `u64`.
    ///
    /// This is not efficient if you need to generate a lot of entropy, this is
    /// provided as a convenience function that wraps
    /// [`try_fill_u8`](crate::rng::Rng::try_fill_u8)
    /// for testing and prototyping.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rng::{Clk, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rng = Rng::new(dp.RNG, Clk::Msi, &mut dp.RCC);
    ///
    /// let rand_value: u64 = rng.try_u64()?;
    /// # Ok::<(), stm32wlxx_hal::rng::Error>(())
    /// ```
    pub fn try_u64(&mut self) -> Result<u64, Error> {
        let mut buf: [u8; 8] = [0; 8];
        self.try_fill_u8(&mut buf)?;
        Ok(u64::from_le_bytes(buf))
    }

    /// Try to generate a random `u128`.
    ///
    /// This is not efficient if you need to generate a lot of entropy, this is
    /// provided as a convenience function that wraps
    /// [`try_fill_u8`](crate::rng::Rng::try_fill_u8)
    /// for testing and prototyping.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rng::{Clk, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rng = Rng::new(dp.RNG, Clk::Msi, &mut dp.RCC);
    ///
    /// let rand_value: u128 = rng.try_u128()?;
    /// # Ok::<(), stm32wlxx_hal::rng::Error>(())
    /// ```
    pub fn try_u128(&mut self) -> Result<u128, Error> {
        let mut buf: [u8; 16] = [0; 16];
        self.try_fill_u8(&mut buf)?;
        Ok(u128::from_le_bytes(buf))
    }

    // Reference manual section 22.3.7 "Error management"
    fn recover_from_noise_error(&mut self) -> Result<(), Error> {
        // software reset by writing CONDRST
        // this automatically clears the seed error interrupt status
        self.rng.cr.modify(|_, w| w.condrst().set_bit());
        self.rng.cr.modify(|_, w| w.condrst().clear_bit());

        // when CONDRST is set to 0 by software its value goes to 0 when the
        // reset process is done.
        // It takes about 2 AHB clock cycles + 2 RNG clock cycles
        while self.rng.cr.read().condrst().bit_is_set() {}

        let sr = self.rng.sr.read();
        if sr.secs().bit_is_set() {
            Err(Error::Seed)
        } else {
            self.err_cnt = self.err_cnt.saturating_add(1);
            Ok(())
        }
    }
}

impl rand_core::RngCore for Rng {
    /// Not recommended for use, panics upon errors.
    fn next_u32(&mut self) -> u32 {
        let mut dws: [u32; 1] = [0; 1];
        unwrap!(self.try_fill_u32(&mut dws));
        dws[0]
    }

    /// Not recommended for use, panics upon errors.
    fn next_u64(&mut self) -> u64 {
        let mut dws: [u32; 2] = [0; 2];
        unwrap!(self.try_fill_u32(&mut dws));
        u64::from(dws[0]) << 32 | u64::from(dws[1])
    }

    /// Not recommended for use, panics upon errors.
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        unwrap!(self.try_fill_u8(dest))
    }

    /// Use this method if using the `RngCore` for `CryptoRng` traits.
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.try_fill_u8(dest)?;
        Ok(())
    }
}

impl rand_core::CryptoRng for Rng {}
