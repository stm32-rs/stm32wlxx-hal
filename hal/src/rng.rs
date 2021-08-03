//! True random number generator

use crate::pac;

use core::{
    num::NonZeroU32,
    sync::atomic::{compiler_fence, Ordering::SeqCst},
};

/// RNG trait abstractions
pub use rand_core;

/// RNG error types
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub enum Error {
    /// A seed error (noise error) occured.
    ///
    /// **Note:** There is automatic correction for seed errors included.
    /// This error means that automatic correction was attempted, but failed.
    Seed,
    /// RNG frequency is too low.
    ///
    /// Check that the RNG is configured correctly.
    Clock,
}

impl Error {
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    fn from_sr(sr: u32) -> Result<(), Error> {
        if sr & (1 << 5) != 0 {
            Err(Error::Clock)
        } else if sr & (1 << 6) != 0 {
            Err(Error::Seed)
        } else {
            Ok(())
        }
    }
}

impl From<Error> for rand_core::Error {
    fn from(e: Error) -> Self {
        match e {
            Error::Seed => NonZeroU32::new(1).unwrap().into(),
            Error::Clock => NonZeroU32::new(2).unwrap().into(),
        }
    }
}

pub use pac::rcc::ccipr::RNGSEL_A as ClkSrc;

/// RNG driver.
#[derive(Debug)]
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
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::MSI);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    /// ```
    pub fn new(rng: pac::RNG, rcc: &mut pac::RCC) -> Rng {
        Self::enable_clock(rcc);
        rcc.ahb3rstr.modify(|_, w| w.rngrst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.rngrst().clear_bit());

        // RNG configuration A
        // see table 131 "RNG configurations" in the reference manual
        #[rustfmt::skip]
        rng.cr.write(|w| {
            w
                .condrst().set_bit()
                .nistc().set_bit()
                .rng_config1().config_a()
                .clkdiv().bits(0x0)
                .rng_config2().config_a_b()
                .rng_config3().config_a()
                .ced().enabled()
                .ie().disabled() // interrupt enable
                .rngen().set_bit()
        });

        #[rustfmt::skip]
        rng.cr.write(|w| {
            w
                .condrst().clear_bit()
                .nistc().set_bit()
                .rng_config1().config_a()
                .clkdiv().bits(0x0)
                .rng_config2().config_a_b()
                .rng_config3().config_a()
                .ced().clear_bit()
                .ie().disabled() // interrupt enable
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
    /// This will also reset the correctable noise error counter.
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
    /// unsafe { stm32wl_hal::rng::Rng::unmask_irq() };
    /// ```
    #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    #[cfg_attr(docsrs, doc(cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))))]
    pub unsafe fn unmask_irq() {
        pac::NVIC::unmask(pac::Interrupt::TRUE_RNG)
    }

    /// Disable the RNG clock.
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.rngen().disabled());
    }

    /// Enable the RNG clock.
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.rngen().enabled());
        rcc.ahb3enr.read(); // delay after an RCC peripheral clock enabling
    }

    /// Set the RNG clock source.
    pub fn set_clock_source(rcc: &mut pac::RCC, src: ClkSrc) {
        rcc.ccipr.modify(|_, w| w.rngsel().variant(src))
    }

    /// Returns the number of correctable seed errors that have occured.
    ///
    /// This counter will saturate when it hits the maximum value.
    pub fn seed_error_stat(&self) -> u32 {
        self.err_cnt
    }

    /// Reset the correctable seed error counter to zero.
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
    /// use stm32wl_hal::{
    ///     pac,
    ///     rng::{ClkSrc, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::MSI);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    ///
    /// let mut nonce: [u32; 4] = [0; 4];
    /// rng.try_fill_u32(&mut nonce)?;
    /// # Ok::<(), stm32wl_hal::rng::Error>(())
    /// ```
    pub fn try_fill_u32(&mut self, dest: &mut [u32]) -> Result<(), Error> {
        for dw in dest {
            // reference manual recommends verifying DR is non-zero for
            // **each** read to DR incase there is a seed error between
            // polling SR and reading DR
            self.poll_data_valid()?;
            *dw = self.rng.dr.read().bits();
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
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::MSI);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    ///
    /// let mut nonce: [u8; 16] = [0; 16];
    /// rng.try_fill_u8(&mut nonce)?;
    /// # Ok::<(), stm32wl_hal::rng::Error>(())
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
    /// use stm32wl_hal::{
    ///     pac,
    ///     rng::{ClkSrc, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::MSI);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    ///
    /// let rand_value: u8 = rng.try_u8()?;
    /// # Ok::<(), stm32wl_hal::rng::Error>(())
    /// ```
    pub fn try_u8(&mut self) -> Result<u8, Error> {
        let mut buf: [u8; 1] = [0];
        self.try_fill_u8(&mut buf)?;
        Ok(buf[0])
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
    /// use stm32wl_hal::{
    ///     pac,
    ///     rng::{ClkSrc, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::MSI);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    ///
    /// let rand_value: u16 = rng.try_u16()?;
    /// # Ok::<(), stm32wl_hal::rng::Error>(())
    /// ```
    pub fn try_u16(&mut self) -> Result<u16, Error> {
        let mut buf: [u8; 2] = [0; 2];
        self.try_fill_u8(&mut buf)?;
        Ok(u16::from_le_bytes(buf))
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
    /// use stm32wl_hal::{
    ///     pac,
    ///     rng::{ClkSrc, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::MSI);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    ///
    /// let rand_value: u32 = rng.try_u32()?;
    /// # Ok::<(), stm32wl_hal::rng::Error>(())
    /// ```
    pub fn try_u32(&mut self) -> Result<u32, Error> {
        let mut buf: [u32; 1] = [0; 1];
        self.try_fill_u32(&mut buf)?;
        Ok(buf[0])
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
    /// use stm32wl_hal::{
    ///     pac,
    ///     rng::{ClkSrc, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::MSI);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    ///
    /// let rand_value: u64 = rng.try_u64()?;
    /// # Ok::<(), stm32wl_hal::rng::Error>(())
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
    /// use stm32wl_hal::{
    ///     pac,
    ///     rng::{ClkSrc, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::MSI);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    ///
    /// let rand_value: u128 = rng.try_u128()?;
    /// # Ok::<(), stm32wl_hal::rng::Error>(())
    /// ```
    pub fn try_u128(&mut self) -> Result<u128, Error> {
        let mut buf: [u8; 16] = [0; 16];
        self.try_fill_u8(&mut buf)?;
        Ok(u128::from_le_bytes(buf))
    }

    /// Try to fill the destination buffer with random data, asynchronously.
    ///
    /// This is the native data size for the RNG.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     pac,
    ///     rng::{ClkSrc, Rng},
    /// };
    ///
    /// # async fn doctest() -> Result<(), stm32wl_hal::rng::Error> {
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::MSI);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    ///
    /// let mut nonce: [u32; 4] = [0; 4];
    /// rng.aio_try_fill_u32(&mut nonce).await?;
    /// # Ok(()) }
    /// ```
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    #[cfg_attr(
        docsrs,
        doc(cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p"))))
    )]
    pub async fn aio_try_fill_u32(&mut self, dest: &mut [u32]) -> Result<(), Error> {
        for dw in dest {
            let sr = self.rng.sr.read();
            if sr.drdy().bit_is_set() {
                *dw = self.rng.dr.read().bits();
            } else {
                self.rng.cr.modify(|_, w| w.ie().enabled());
                match futures::future::poll_fn(aio::poll).await {
                    Err(Error::Seed) => self.recover_from_noise_error()?,
                    Err(Error::Clock) => return Err(Error::Clock),
                    Ok(_) => (),
                }
            }
        }
        Ok(())
    }

    /// Try to fill the destination buffer with random data, asynchronously.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     pac,
    ///     rng::{ClkSrc, Rng},
    /// };
    ///
    /// # async fn doctest() -> Result<(), stm32wl_hal::rng::Error> {
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Rng::set_clock_source(&mut dp.RCC, ClkSrc::MSI);
    /// let mut rng = Rng::new(dp.RNG, &mut dp.RCC);
    ///
    /// let mut nonce: [u8; 16] = [0; 16];
    /// rng.aio_try_fill_u8(&mut nonce).await?;
    /// # Ok(()) }
    /// ```
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    #[cfg_attr(
        docsrs,
        doc(cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p"))))
    )]
    pub async fn aio_try_fill_u8(&mut self, dest: &mut [u8]) -> Result<(), Error> {
        for chunk in dest.chunks_mut(4) {
            let mut entropy: [u32; 1] = [0];
            self.aio_try_fill_u32(&mut entropy).await?;

            chunk
                .iter_mut()
                .enumerate()
                .for_each(|(idx, byte)| *byte = entropy[0].to_be_bytes()[idx])
        }
        Ok(())
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
        while self.rng.cr.read().condrst().bit_is_set() {
            compiler_fence(SeqCst);
        }

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
        self.try_fill_u32(&mut dws).unwrap();
        dws[0]
    }

    /// Not recommended for use, panics upon errors.
    fn next_u64(&mut self) -> u64 {
        let mut dws: [u32; 2] = [0; 2];
        self.try_fill_u32(&mut dws).unwrap();
        u64::from(dws[0]) << 32 | u64::from(dws[1])
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

#[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
mod aio {
    use core::{
        sync::atomic::{AtomicU32, Ordering::SeqCst},
        task::Poll,
    };
    use futures_util::task::AtomicWaker;

    static RNG_WAKER: AtomicWaker = AtomicWaker::new();
    static RNG_RESULT: AtomicU32 = AtomicU32::new(0);

    pub fn poll(cx: &mut core::task::Context<'_>) -> Poll<Result<(), super::Error>> {
        RNG_WAKER.register(cx.waker());
        match RNG_RESULT.load(SeqCst) {
            0 => core::task::Poll::Pending,
            _ => {
                RNG_WAKER.take();
                let sr: u32 = RNG_RESULT.swap(0, SeqCst);
                Poll::Ready(super::Error::from_sr(sr))
            }
        }
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    mod irq {
        use super::{SeqCst, RNG_RESULT, RNG_WAKER};
        use crate::pac::{self, interrupt};

        #[interrupt]
        #[allow(non_snake_case)]
        fn TRUE_RNG() {
            debug_assert_eq!(RNG_RESULT.load(SeqCst), 0);

            let dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };

            // store result
            RNG_RESULT.store(dp.RNG.sr.read().bits(), SeqCst);

            // disable IRQs
            dp.RNG.cr.modify(|_, w| w.ie().disabled());
            // clear IRQ status
            dp.RNG.sr.write(|w| w.seis().clear_bit().ceis().clear_bit());

            RNG_WAKER.wake();
        }
    }
}
