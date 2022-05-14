//! seeed LoRa-E5 development kit board support package.

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

pub mod led;
pub mod pb;

pub use stm32wlxx_hal as hal;

use hal::{
    cortex_m::interrupt::CriticalSection,
    gpio::{self, pins, Output, OutputArgs, PinState},
};

/// RF switch
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RfSwitch {
    a4: Output<pins::A4>,
    a5: Output<pins::A5>,
}

impl RfSwitch {
    /// Create a new `RfSwitch` struct from GPIOs.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{cortex_m, gpio::PortA, pac},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let rfs: RfSwitch = cortex_m::interrupt::free(|cs| RfSwitch::new(gpioa.a4, gpioa.a5, cs));
    /// ```
    pub fn new(a4: pins::A4, a5: pins::A5, cs: &CriticalSection) -> RfSwitch {
        const ARGS: OutputArgs = OutputArgs {
            speed: gpio::Speed::Fast,
            level: gpio::PinState::High,
            ot: gpio::OutputType::PushPull,
            pull: gpio::Pull::None,
        };
        RfSwitch {
            a4: Output::new(a4, &ARGS, cs),
            a5: Output::new(a5, &ARGS, cs),
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
    /// use lora_e5_bsp::{
    ///     hal::{
    ///         cortex_m,
    ///         gpio::{pins, Output, PortA},
    ///         pac,
    ///     },
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// cortex_m::interrupt::free(|cs| {
    ///     let _: Output<pins::A4> = Output::default(pa.a4, cs);
    ///     let _: Output<pins::A5> = Output::default(pa.a5, cs);
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
            a4: Output::steal(),
            a5: Output::steal(),
        }
    }

    /// Set the RF switch to receive.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{cortex_m, gpio::PortA, pac},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let mut rfs: RfSwitch = cortex_m::interrupt::free(|cs| RfSwitch::new(gpioa.a4, gpioa.a5, cs));
    /// rfs.set_rx();
    /// ```
    #[inline]
    pub fn set_rx(&mut self) {
        self.a5.set_level(PinState::Low);
        self.a4.set_level(PinState::High);
    }

    /// Set the RF switch to high power transmit.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{cortex_m, gpio::PortA, pac},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let mut rfs: RfSwitch = cortex_m::interrupt::free(|cs| RfSwitch::new(gpioa.a4, gpioa.a5, cs));
    /// rfs.set_tx_hp();
    /// ```
    #[inline]
    pub fn set_tx_hp(&mut self) {
        self.a4.set_level(PinState::Low);
        self.a5.set_level(PinState::High);
    }
}
