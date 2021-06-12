//! True random number generator.
#![cfg_attr(not(test), no_std)]

use core::{
    mem::transmute,
    num::NonZeroU32,
    sync::atomic::{compiler_fence, Ordering::SeqCst},
};

pub use rand_core;

cfg_if::cfg_if! {
    if #[cfg(feature = "stm32wl5x_cm0p")] {
        /// Peripheral access crate.
        pub use stm32wl::stm32wl5x_cm0p as pac;
    } else if #[cfg(feature = "stm32wl5x_cm4")] {
        /// Peripheral access crate.
        pub use stm32wl::stm32wl5x_cm4 as pac;
    } else if #[cfg(feature = "stm32wle5")] {
        /// Peripheral access crate.
        pub use stm32wl::stm32wle5 as pac;
    } else {
        core::compile_error!("You must select your hardware with a feature flag");
    }
}

/// RNG error types
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub enum Error {
    /// A noise error (seed error) occured and automatic correction failed.
    UncorrectableNoise,
    /// RNG frequency is too low.
    ///
    /// Check that the RNG is configured correctly.
    Clock,
    /// Timeout waiting for the data ready bit.
    Timeout,
}

impl From<Error> for rand_core::Error {
    fn from(e: Error) -> Self {
        match e {
            Error::UncorrectableNoise => NonZeroU32::new(1).unwrap().into(),
            Error::Clock => NonZeroU32::new(2).unwrap().into(),
            Error::Timeout => NonZeroU32::new(3).unwrap().into(),
        }
    }
}

/// RNG driver.
pub struct Rng {
    rng: pac::RNG,
    err_cnt: u32,
}

impl Rng {
    /// Create a new `Rng` driver from a RNG peripheral.
    ///
    /// This will reset the RNG, but it will not enable clocks for the RNG.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal_rng::{pac, Rng};
    ///
    /// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rcc = dp.RCC;
    ///
    /// // ... setup the system clocks
    /// // ... take a look at RCC_CCIPR to select the RNG clock source
    ///
    /// rcc.ahb3enr.modify(|_, w| w.rngen().set_bit());
    /// rcc.ahb3enr.read(); // Delay after an RCC peripheral clock enabling
    ///
    /// let mut rng = Rng::new(dp.RNG, &mut rcc);
    /// ```
    pub fn new(rng: pac::RNG, rcc: &mut pac::RCC) -> Rng {
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

    /// Returns the number of correctable noise errors that have occured.
    ///
    /// This counter will saturate when it hits the maximum value.
    pub fn noise_error_stat(&self) -> u32 {
        self.err_cnt
    }

    /// Reset the correctable noise error counter to zero.
    pub fn rest_noise_error_stat(&mut self) {
        self.err_cnt = 0
    }

    fn wait_for_new_entropy(&mut self) -> Result<(), Error> {
        let mut timeout: u32 = 0;
        loop {
            let sr = self.rng.sr.read();
            if sr.seis().bit_is_set() {
                self.recover_from_noise_error()?;
            } else if sr.ceis().bit_is_set() {
                return Err(Error::Clock.into());
            } else if sr.drdy().bit_is_set() {
                return Ok(());
            }

            // TODO: User selectable timeout duration (or not? this has a fixed duration)
            // TODO: I dislike everything about this.
            timeout = timeout.saturating_add(1);
            if timeout > 100_000 {
                return Err(Error::Timeout.into());
            }
        }
    }

    /// Try to fill the destination buffer with random data by `u32`s.
    pub fn try_fill_u32(&mut self, dest: &mut [u32]) -> Result<(), Error> {
        for chunk in dest.chunks_mut(4) {
            self.wait_for_new_entropy()?;
            for dw in chunk {
                *dw = self.rng.dr.read().bits();
            }
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
        let mut bytes: [u8; 4] = [0; 4];
        self.try_fill_bytes(&mut bytes).unwrap();
        u32::from_le_bytes(bytes)
    }

    /// Not recommended for use, panics upon errors.
    fn next_u64(&mut self) -> u64 {
        let mut bytes: [u8; 8] = [0; 8];
        self.try_fill_bytes(&mut bytes).unwrap();
        u64::from_le_bytes(bytes)
    }

    /// Not recommended for use, panics upon errors.
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.try_fill_bytes(dest).unwrap()
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        for chunk in dest.chunks_mut(16) {
            self.wait_for_new_entropy()?;

            let mut block: [u32; 4] = [0; 4];
            self.try_fill_u32(&mut block)?;
            // safety: random data has no requirements for safe transmute.
            let data: [u8; 16] = unsafe { transmute::<[u32; 4], [u8; 16]>(block) };

            chunk
                .iter_mut()
                .zip(data.iter())
                .for_each(|(buffer_byte, &data_byte)| *buffer_byte = data_byte);
        }

        Ok(())
    }
}

impl rand_core::CryptoRng for Rng {}
