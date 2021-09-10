//! seeed LoRa-E5 development kit board support package.
#![cfg_attr(not(test), no_std)]
#![forbid(missing_docs)]

pub mod led;
pub mod pb;

pub use stm32wl_hal as hal;

use hal::gpio::{self, pins, Level, Output, OutputArgs};

/// RF switch
#[derive(Debug)]
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
    ///     hal::{gpio::PortA, pac},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let rfs = RfSwitch::new(gpioa.a4, gpioa.a5);
    /// ```
    pub fn new(a4: pins::A4, a5: pins::A5) -> RfSwitch {
        const ARGS: OutputArgs = OutputArgs {
            speed: gpio::Speed::Fast,
            level: gpio::Level::High,
            ot: gpio::OutputType::PushPull,
            pull: gpio::Pull::None,
        };
        RfSwitch {
            a4: Output::new(a4, &ARGS),
            a5: Output::new(a5, &ARGS),
        }
    }

    /// Set the RF switch to receive.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{gpio::PortA, pac},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let mut rfs = RfSwitch::new(gpioa.a4, gpioa.a5);
    /// rfs.set_rx();
    /// ```
    pub fn set_rx(&mut self) {
        self.a5.set_level(Level::Low);
        self.a4.set_level(Level::High);
    }

    /// Set the RF switch to high power transmit.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{gpio::PortA, pac},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let mut rfs = RfSwitch::new(gpioa.a4, gpioa.a5);
    /// rfs.set_tx_hp();
    /// ```
    pub fn set_tx_hp(&mut self) {
        self.a4.set_level(Level::Low);
        self.a5.set_level(Level::High);
    }
}
