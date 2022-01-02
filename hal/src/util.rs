//! Miscellaneous utilities
use crate::pac;
use cortex_m::{delay::Delay, peripheral::syst::SystClkSource};

/// Create a new [`cortex_m::delay::Delay`] from the current CPU systick
/// frequency.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{pac, util::new_delay};
///
/// let dp = pac::Peripherals::take().unwrap();
/// let cp = pac::CorePeripherals::take().unwrap();
/// let delay = new_delay(cp.SYST, &dp.RCC);
/// ```
#[inline]
pub fn new_delay(syst: pac::SYST, rcc: &pac::RCC) -> Delay {
    Delay::new(
        syst,
        // Delay constructor will set SystClkSource::Core
        crate::rcc::cpu_systick_hz(rcc, SystClkSource::Core),
    )
}
