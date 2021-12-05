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

/// Reset the cycle counter to zero.
///
/// This function will be removed upon the next release of [`cortex_m`] that
/// includes this functionality. See [#347] for details.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{pac, util::reset_cycle_count};
///
/// let mut cp = pac::CorePeripherals::take().unwrap();
/// cp.DCB.enable_trace();
/// cp.DWT.enable_cycle_counter();
/// reset_cycle_count(&mut cp.DWT);
/// ```
///
/// [#347]: https://github.com/rust-embedded/cortex-m/pull/347
#[allow(unused_variables)]
#[inline]
pub fn reset_cycle_count(dwt: &mut pac::DWT) {
    const DWT_CYCCNT: *mut u32 = 0xE0001004 as *mut u32;
    unsafe { DWT_CYCCNT.write_volatile(0) };
}
