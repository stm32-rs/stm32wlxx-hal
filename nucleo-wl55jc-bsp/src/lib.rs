//! NUCLEO-WL55JC board support package.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code, missing_docs)]

pub use stm32wl_hal as hal;

use hal::gpio::{pins, Level, Output, OutputArgs, OutputType, Pull, Speed};

/// RF switch.
#[derive(Debug)]
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
    /// use bsp::hal::{gpio::PortC, pac};
    /// use bsp::RfSwitch;
    /// use nucleo_wl55jc_bsp as bsp;
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let rfs = RfSwitch::new(gpioc.pc3, gpioc.pc4, gpioc.pc5);
    /// ```
    pub fn new(c3: pins::C3, c4: pins::C4, c5: pins::C5) -> RfSwitch {
        const ARGS: OutputArgs = OutputArgs {
            speed: Speed::Fast,
            level: Level::High,
            ot: OutputType::PushPull,
            pull: Pull::None,
        };
        RfSwitch {
            fe_ctrl1: Output::new(c4, &ARGS),
            fe_ctrl2: Output::new(c5, &ARGS),
            fe_ctrl3: Output::new(c3, &ARGS),
        }
    }

    /// Set the RF switch to receive.
    pub fn set_rx(&mut self) {
        self.fe_ctrl1.set_output_level(Level::High);
        self.fe_ctrl2.set_output_level(Level::Low);
        self.fe_ctrl3.set_output_level(Level::High);
    }

    /// Set the RF switch to low power transmit.
    pub fn set_tx_lp(&mut self) {
        self.fe_ctrl1.set_output_level(Level::High);
        self.fe_ctrl2.set_output_level(Level::High);
        self.fe_ctrl3.set_output_level(Level::High);
    }

    /// Set the RF switch to high power transmit.
    pub fn set_tx_hp(&mut self) {
        self.fe_ctrl2.set_output_level(Level::High);
        self.fe_ctrl1.set_output_level(Level::Low);
        self.fe_ctrl3.set_output_level(Level::High);
    }
}
