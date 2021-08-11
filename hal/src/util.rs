//! Miscellaneous utilities
use crate::pac;
use core::ptr::write_volatile;
use cortex_m::{delay::Delay, peripheral::syst::SystClkSource};

/// Create a new [`cortex_m::delay::Delay`] from the current CPU systick
/// frequency.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, util::new_delay};
///
/// let dp = pac::Peripherals::take().unwrap();
/// let cp = pac::CorePeripherals::take().unwrap();
/// let delay = new_delay(cp.SYST, &dp.RCC);
/// ```
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
/// use stm32wl_hal::{pac, util::reset_cycle_count};
///
/// let mut cp = pac::CorePeripherals::take().unwrap();
/// cp.DCB.enable_trace();
/// cp.DWT.enable_cycle_counter();
/// reset_cycle_count(&mut cp.DWT);
/// ```
///
/// [#347]: https://github.com/rust-embedded/cortex-m/pull/347
#[allow(unused_variables)]
pub fn reset_cycle_count(dwt: &mut pac::DWT) {
    const DWT_CYCCNT: usize = 0xE0001004;
    unsafe { write_volatile(DWT_CYCCNT as *mut u32, 0) };
}
