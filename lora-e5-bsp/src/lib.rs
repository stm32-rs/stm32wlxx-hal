//! seeed LoRa-E5 development kit board support package.

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

use hal::{
    cortex_m::interrupt::CriticalSection,
    gpio::{self, pins, Output, OutputArgs, PinState},
};
pub use stm32wlxx_hal as hal;
use stm32wlxx_hal::subghz::{RfSwRx, RfSwTx};

use crate::hal::subghz::{PaConfig, TxParams};

pub mod led;
pub mod pb;

#[cfg(feature = "defmt")]
use dfmt as defmt;

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
}

impl RfSwRx for RfSwitch {
    /// Set the RF switch to receive.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{cortex_m, gpio::PortA, pac, subghz::RfSwRx},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let mut rfs: RfSwitch = cortex_m::interrupt::free(|cs| RfSwitch::new(gpioa.a4, gpioa.a5, cs));
    /// rfs.set_rx();
    /// ```
    fn set_rx(&mut self) {
        self.a5.set_level(PinState::Low);
        self.a4.set_level(PinState::High);
    }
}

impl RfSwTx for RfSwitch {
    const PA_CONFIG: PaConfig = PaConfig::HP_22;
    const TX_PARAMS: TxParams = TxParams::HP;

    /// Set the RF switch to high power transmit.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{cortex_m, gpio::PortA, pac, subghz::RfSwTx},
    ///     RfSwitch,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let mut rfs: RfSwitch = cortex_m::interrupt::free(|cs| RfSwitch::new(gpioa.a4, gpioa.a5, cs));
    /// rfs.set_tx();
    /// ```
    fn set_tx(&mut self) {
        self.a4.set_level(PinState::Low);
        self.a5.set_level(PinState::High);
    }
}
