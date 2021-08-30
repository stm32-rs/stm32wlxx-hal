//! Power control

use core::sync::atomic::{compiler_fence, Ordering::SeqCst};

use crate::pac;

const SCB_SCR_SLEEPDEEP: u32 = 0x1 << 2;

/// Wakeup pin options.
///
/// Argument of [`setup_wakeup_pins`].
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
/// use stm32wl_hal::{
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
    #[rustfmt::skip]
    pwr.cr3.modify(|_, w| {
        w
            .ewup1().bit(wp1.en())
            .ewup2().bit(wp2.en())
            .ewup3().bit(wp3.en())
    });
    #[rustfmt::skip]
    pwr.cr4.modify(|_, w| {
        w
            .wp1().bit(wp1.edge())
            .wp2().bit(wp2.edge())
            .wp3().bit(wp3.edge())
    });
}

/// Enter shutdown mode immediately.
///
/// This will:
///
/// 1. Disable interrupts.
/// 2. Set PWR.CR1.LPMS to shutdown.
/// 3. Set SCB.SCR.SLEEPDEEP.
/// 4. Enter WFI.
///
/// # Safety
///
/// 1. Existing shutdown mode will result in a power-on-reset.
/// 2. Wakeup pins should be configured with [`setup_wakeup_pins`] unless
///    you intend to wakeup only via reset.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, pwr::shutdown};
///
/// let mut cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
/// // SLEEPDEEP is implemented with retention
/// // generally you will want to clear this at power-on when using shutdown
/// cp.SCB.clear_sleepdeep();
///
/// // ... do things
///
/// unsafe { shutdown() };
/// ```
#[inline]
pub unsafe fn shutdown() -> ! {
    cortex_m::interrupt::disable();

    // safety: interrupts are disabled and no way to use core 2 currently
    // provided by the HAL
    (*pac::PWR::PTR).cr1.modify(|_, w| w.lpms().shutdown());

    // safety: interrupts are disabled core 2 cannot access our core registers
    (*pac::SCB::PTR).scr.modify(|scr| scr | SCB_SCR_SLEEPDEEP);

    cortex_m::asm::wfi();

    // technically unreachable
    // the unreachable!() macro takes up needless code space
    loop {
        compiler_fence(SeqCst)
    }
}
