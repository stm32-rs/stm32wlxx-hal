//! Power control

use core::sync::atomic::{compiler_fence, Ordering::SeqCst};

use cortex_m::interrupt::CriticalSection;

use crate::{pac, rcc::MsiRange};

const SCB_SCR_SLEEPDEEP: u32 = 0x1 << 2;
const SCB_SCR_SLEEPONEXIT: u32 = 0x1 << 1;

/// Wakeup pin options for [`setup_wakeup_pins`].
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WakeupPin {
    /// Wakeup pin disabled.
    Disabled,
    /// Wakeup pin enabled with a rising edge.
    Rising,
    /// Wakeup pin enabled with a falling edge.
    Falling,
}

impl WakeupPin {
    const fn en(&self) -> bool {
        !matches!(self, WakeupPin::Disabled)
    }

    const fn edge(&self) -> bool {
        matches!(self, WakeupPin::Falling)
    }
}

/// Setup the wakeup pins for shutdown and standby low-power modes.
///
/// The reference manual is not 100% specific which wakeup pin corresponds to
/// which physical pin, a footnote of
/// "Table 45. Functionalities depending on system operating mode"
/// in RM0453 Rev 2 implies the following:
///
/// * WP1 corresponds to [`A0`](crate::gpio::pins::A0)
/// * WP2 corresponds to [`C13`](crate::gpio::pins::C13)
/// * WP3 corresponds to [`B3`](crate::gpio::pins::B3)
///
/// If you know where to find more concrete information on this please open
/// an issue.
///
/// # Example
///
/// Enable A0 to wakeup on a falling edge.
///
/// ```no_run
/// use stm32wlxx_hal::{
///     pac,
///     pwr::{setup_wakeup_pins, WakeupPin},
/// };
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// setup_wakeup_pins(
///     &mut dp.PWR,
///     WakeupPin::Falling,
///     WakeupPin::Disabled,
///     WakeupPin::Disabled,
/// );
/// ```
#[inline]
pub fn setup_wakeup_pins(pwr: &mut pac::PWR, wp1: WakeupPin, wp2: WakeupPin, wp3: WakeupPin) {
    pwr.cr3.modify(|_, w| {
        w.ewup1().bit(wp1.en());
        w.ewup2().bit(wp2.en());
        w.ewup3().bit(wp3.en())
    });
    pwr.cr4.modify(|_, w| {
        w.wp1().bit(wp1.edge());
        w.wp2().bit(wp2.edge());
        w.wp3().bit(wp3.edge())
    });
}

/// Enter shutdown mode immediately.
///
/// Wakeup pins should be configured with [`setup_wakeup_pins`] unless
/// you intend to wakeup only via reset.
///
/// This will:
///
/// 1. Disable interrupts.
/// 2. Set `PWR.CR1.LPMS` to shutdown.
/// 3. Set `SCB.SCR.SLEEPDEEP`.
/// 4. Enter WFI.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{pac, pwr::shutdown};
///
/// let mut cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
/// // SLEEPDEEP is implemented with retention
/// // generally you will want to clear this at power-on when using shutdown
/// cp.SCB.clear_sleepdeep();
///
/// // ... do things
///
/// shutdown();
/// ```
#[inline]
pub fn shutdown() -> ! {
    cortex_m::interrupt::disable();

    // safety: interrupts are disabled and no way to use core 2 currently
    // provided by the HAL
    unsafe { (*pac::PWR::PTR).cr1.modify(|_, w| w.lpms().shutdown()) };

    // safety: interrupts are disabled core 2 cannot access our core registers
    unsafe { (*pac::SCB::PTR).scr.modify(|scr| scr | SCB_SCR_SLEEPDEEP) };

    cortex_m::asm::wfi();

    // technically unreachable
    // the unreachable!() macro takes up needless code space
    loop {
        compiler_fence(SeqCst)
    }
}

/// Enable shutdown on return from ISR or the next WFI or WFE.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{pac, pwr::enable_shutdown_sleeponexit};
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// let mut cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
/// // SLEEPDEEP is implemented with retention
/// // generally you will want to clear this at power-on when using shutdown
/// cp.SCB.clear_sleepdeep();
///
/// // ... do things
///
/// enable_shutdown_sleeponexit(&mut dp.PWR, &mut cp.SCB);
/// ```
#[inline]
pub fn enable_shutdown_sleeponexit(pwr: &mut pac::PWR, scb: &mut pac::SCB) {
    unsafe {
        scb.scr
            .modify(|scr| scr | SCB_SCR_SLEEPDEEP | SCB_SCR_SLEEPONEXIT)
    };
    pwr.cr1.modify(|_, w| w.lpms().shutdown());
}

/// Enable shutdown on the next WFI or WFE.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{pac, pwr::enable_shutdown};
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// let mut cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
/// // SLEEPDEEP is implemented with retention
/// // generally you will want to clear this at power-on when using shutdown
/// cp.SCB.clear_sleepdeep();
///
/// // ... do things
///
/// enable_shutdown(&mut dp.PWR, &mut cp.SCB);
/// ```
#[inline]
pub fn enable_shutdown(pwr: &mut pac::PWR, scb: &mut pac::SCB) {
    unsafe { scb.scr.modify(|scr| scr | SCB_SCR_SLEEPDEEP) };
    pwr.cr1.modify(|_, w| w.lpms().shutdown());
}

/// MSI clock ranges for [`enter_lprun_msi`].
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LprunRange {
    /// Range 0 around 100 kHz.
    Range100k = 0b0000,
    /// Range 1 around 200 kHz.
    Range200k = 0b0001,
    /// Range 2 around 400 kHz.
    Range400k = 0b0010,
    /// Range 3 around 800 kHz.
    Range800k = 0b0011,
    /// Range 4 around 1 MHz.
    Range1M = 0b0100,
}

impl From<LprunRange> for MsiRange {
    fn from(lprr: LprunRange) -> Self {
        match lprr {
            LprunRange::Range100k => MsiRange::Range100k,
            LprunRange::Range200k => MsiRange::Range200k,
            LprunRange::Range400k => MsiRange::Range400k,
            LprunRange::Range800k => MsiRange::Range800k,
            LprunRange::Range1M => MsiRange::Range1M,
        }
    }
}

/// Enter low-power run mode with MSI as a clock source.
///
/// # Safety
///
/// 1. This will disable the HSE32 clock if not already disabled.
///    Before calling this method peripherals (e.g. SUBGHZ, RTC) should
///    be switched to a different clock source.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{
///     pac,
///     pwr::{enter_lprun_msi, LprunRange},
/// };
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// cortex_m::interrupt::free(|cs| unsafe {
///     enter_lprun_msi(
///         &mut dp.FLASH,
///         &mut dp.PWR,
///         &mut dp.RCC,
///         LprunRange::Range1M,
///         cs,
///     )
/// });
/// ```
#[inline]
pub unsafe fn enter_lprun_msi(
    flash: &mut pac::FLASH,
    pwr: &mut pac::PWR,
    rcc: &mut pac::RCC,
    range: LprunRange,
    cs: &CriticalSection,
) {
    crate::rcc::set_sysclk_msi(flash, pwr, rcc, range.into(), cs);
    rcc.cr.modify(|_, w| w.hseon().disabled());
    pwr.cr1.modify(|_, w| w.lpr().low_power_mode());
}

/// Exit low-power run mode.
///
/// This will not increase the clock frequencies after exit.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{
///     pac,
///     pwr::{enter_lprun_msi, exit_lprun, LprunRange},
/// };
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// cortex_m::interrupt::free(|cs| unsafe {
///     enter_lprun_msi(
///         &mut dp.FLASH,
///         &mut dp.PWR,
///         &mut dp.RCC,
///         LprunRange::Range1M,
///         cs,
///     )
/// });
///
/// exit_lprun(&mut dp.PWR);
/// ```
#[inline]
pub fn exit_lprun(pwr: &mut pac::PWR) {
    pwr.cr1.modify(|_, w| w.lpr().main_mode());
    while !pwr.sr2.read().reglpf().is_main() {}
}
