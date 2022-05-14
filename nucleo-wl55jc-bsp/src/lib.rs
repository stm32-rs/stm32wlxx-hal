//! NUCLEO-WL55JC board support package.

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

pub mod led;
pub mod pb;

pub use stm32wlxx_hal as hal;

use hal::{
    cortex_m::interrupt::CriticalSection,
    gpio::{self, pins, Output, OutputArgs, PinState},
};

/// RF switch.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RfSwitch {
    fe_ctrl1: Output<pins::C4>,
    fe_ctrl2: Output<pins::C5>,
    fe_ctrl3: Output<pins::C3>,
}

impl RfSwitch {
    /// Create a new `RfSwitch` struct from GPIOs.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{cortex_m, gpio::PortC, pac},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let rfs: RfSwitch =
    ///     cortex_m::interrupt::free(|cs| RfSwitch::new(gpioc.c3, gpioc.c4, gpioc.c5, cs));
    /// ```
    pub fn new(c3: pins::C3, c4: pins::C4, c5: pins::C5, cs: &CriticalSection) -> RfSwitch {
        const ARGS: OutputArgs = OutputArgs {
            speed: gpio::Speed::Fast,
            level: gpio::PinState::High,
            ot: gpio::OutputType::PushPull,
            pull: gpio::Pull::None,
        };
        RfSwitch {
            fe_ctrl1: Output::new(c4, &ARGS, cs),
            fe_ctrl2: Output::new(c5, &ARGS, cs),
            fe_ctrl3: Output::new(c3, &ARGS, cs),
        }
    }

    /// Steal the RF switch from whatever is currently using it.
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the RF switch has exclusive access.
    ///    Singleton checks are bypassed with this method.
    /// 2. You must set up the RF switch pins.
    ///    No setup will occur when using this method.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{
    ///         cortex_m,
    ///         gpio::{pins, Output, PortC},
    ///         pac,
    ///     },
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// cortex_m::interrupt::free(|cs| {
    ///     let _: Output<pins::C3> = Output::default(pc.c3, cs);
    ///     let _: Output<pins::C4> = Output::default(pc.c4, cs);
    ///     let _: Output<pins::C5> = Output::default(pc.c5, cs);
    /// });
    ///
    /// // safety:
    /// // 1. we have exclusive access to the underlying pins
    /// // 2. the pins have been setup
    /// let rfs: RfSwitch = unsafe { RfSwitch::steal() };
    /// ```
    #[inline]
    pub unsafe fn steal() -> Self {
        RfSwitch {
            fe_ctrl1: Output::steal(),
            fe_ctrl2: Output::steal(),
            fe_ctrl3: Output::steal(),
        }
    }

    /// Set the RF switch to receive.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{cortex_m, gpio::PortC, pac},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut rfs: RfSwitch =
    ///     cortex_m::interrupt::free(|cs| RfSwitch::new(gpioc.c3, gpioc.c4, gpioc.c5, cs));
    /// rfs.set_rx()
    /// ```
    #[inline]
    pub fn set_rx(&mut self) {
        self.fe_ctrl1.set_level(PinState::High);
        self.fe_ctrl2.set_level(PinState::Low);
        self.fe_ctrl3.set_level(PinState::High);
    }

    /// Set the RF switch to low power transmit.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{cortex_m, gpio::PortC, pac},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut rfs: RfSwitch =
    ///     cortex_m::interrupt::free(|cs| RfSwitch::new(gpioc.c3, gpioc.c4, gpioc.c5, cs));
    /// rfs.set_tx_lp()
    /// ```
    #[inline]
    pub fn set_tx_lp(&mut self) {
        self.fe_ctrl1.set_level(PinState::High);
        self.fe_ctrl2.set_level(PinState::High);
        self.fe_ctrl3.set_level(PinState::High);
    }

    /// Set the RF switch to high power transmit.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{cortex_m, gpio::PortC, pac},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut rfs: RfSwitch =
    ///     cortex_m::interrupt::free(|cs| RfSwitch::new(gpioc.c3, gpioc.c4, gpioc.c5, cs));
    /// rfs.set_tx_hp()
    /// ```
    #[inline]
    pub fn set_tx_hp(&mut self) {
        self.fe_ctrl2.set_level(PinState::High);
        self.fe_ctrl1.set_level(PinState::Low);
        self.fe_ctrl3.set_level(PinState::High);
    }
}
