use crate::pac;

/// Trait for peripheral clock enable / disable.
///
/// Typically you will not need to use these methods outside of low-power
/// applications.
///
/// These methods do not consider the clock source, only the peripheral clock
/// enable.
pub trait PeriphClk {
    /// Enable the peripheral clock for core 1.
    fn c1_clk_en(rcc: &mut pac::RCC);

    /// Disable the peripheral clock for core 1.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the peripheral before disabling the clock.
    /// 2. You are responsible for re-enabling the clock before using the
    ///    peripheral.
    unsafe fn c1_clk_dis(rcc: &mut pac::RCC);
}

/// Wrapper around a peripheral for safely disabling the peripheral clock.
#[derive(Debug)]
pub struct WrapClk<P: PeriphClk> {
    periph: P,
}

impl<P: PeriphClk> WrapClk<P> {
    /// Run a closure that accepts the peripheral driver as an argument,
    /// enabling the peripheral clock with [`c1_clk_en`] before calling the
    /// function, and disabling the peripheral clock with [`c1_clk_dis`]
    /// after calling the function.
    ///
    /// This is useful for low power applications that need to access the
    /// peripheral infrequently.
    ///
    /// The power savings vary for each peripheral, check the electrical
    /// characteristics in the datasheet.
    ///
    /// For example the STM32WLE AES peripheral consumes:
    ///
    /// * Range 1: 2.50 μA/MHz, 120μA @ 48 MHz
    /// * Range 2: 2.13 μA/MHz, 102.24μA @ 48 MHz
    /// * LPRun and LPSleep: 1.80 μA/MHz, 86.4μA @ 48 MHz
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     aes::{Aes},
    ///     WrapClk,
    ///     PeriphClk,
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // safety:
    /// // * no need to reset because the AES memory-mapped IO has not been accessed since power-on
    /// // * the wrapper will handle enabling clocks
    /// let mut aeswrap: WrapClk<Aes> = unsafe { Aes::new_no_init(dp.AES) }.into();
    ///
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let mut text: [u32; 4] = [0xf34481ec, 0x3cc627ba, 0xcd5dc3fb, 0x08f273e6];
    /// aeswrap.with_clk(&mut dp.RCC, |aes| aes.encrypt_ecb_inplace(&KEY, &mut text))?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    ///
    /// [`c1_clk_en`]: PeriphClk::c1_clk_en
    /// [`c1_clk_dis`]: PeriphClk::c1_clk_dis
    #[inline]
    pub fn with_clk<F, R>(&mut self, rcc: &mut pac::RCC, f: F) -> R
    where
        F: FnOnce(&mut P) -> R,
    {
        P::c1_clk_en(rcc);
        let ret: R = f(&mut self.periph);
        unsafe { P::c1_clk_dis(rcc) };
        ret
    }
}

impl<P: PeriphClk> From<P> for WrapClk<P> {
    #[inline]
    fn from(periph: P) -> Self {
        WrapClk { periph }
    }
}
