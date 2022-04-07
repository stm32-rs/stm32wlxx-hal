//! LEDs

use stm32wlxx_hal as hal;

use core::ops::Not;
use hal::{
    cortex_m::interrupt::CriticalSection,
    embedded_hal::digital::v2::OutputPin,
    gpio::{self, pins, Output, OutputArgs},
};

const LED_ARGS: OutputArgs = OutputArgs {
    speed: gpio::Speed::Fast,
    level: gpio::PinState::High,
    ot: gpio::OutputType::PushPull,
    pull: gpio::Pull::None,
};

/// D5 LED.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct D5 {
    gpio: Output<pins::B5>,
}

impl D5 {
    /// Create a new D5 LED.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{cortex_m, gpio::PortB, pac},
    ///     led,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let mut d5 = cortex_m::interrupt::free(|cs| led::D5::new(gpiob.b5, cs));
    /// d5.set_on();
    /// ```
    pub fn new(b5: pins::B5, cs: &CriticalSection) -> Self {
        Self {
            gpio: Output::new(b5, &LED_ARGS, cs),
        }
    }

    /// Free the GPIO pin from the LED struct.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{cortex_m, gpio::PortB, pac},
    ///     led,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let mut d5 = cortex_m::interrupt::free(|cs| led::D5::new(gpiob.b5, cs));
    /// // ... use LED
    /// let b5 = d5.free();
    /// ```
    pub fn free(self) -> pins::B5 {
        self.gpio.free()
    }

    /// Steal the LED from whatever is currently using it.
    ///
    /// This will **not** initialize the GPIO peripheral.
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the LED has exclusive access
    ///    to the underlying GPIO.
    ///    Singleton checks are bypassed with this method.
    /// 2. You are responsible for setting up the underlying GPIO correctly.
    ///    No setup will occur when using this method.
    ///
    /// # Example
    ///
    /// ```
    /// use lora_e5_bsp::led::D5;
    ///
    /// // ... setup happens here
    ///
    /// let d5: D5 = unsafe { D5::steal() };
    /// ```
    pub unsafe fn steal() -> Self {
        Self {
            gpio: Output::steal(),
        }
    }

    /// Set the LED on.
    pub fn set_on(&mut self) {
        self.gpio.set_low().unwrap()
    }

    /// Set the LED off.
    pub fn set_off(&mut self) {
        self.gpio.set_high().unwrap()
    }

    /// Toggle the LED state.
    pub fn toggle(&mut self) {
        self.gpio.set_level(self.gpio.level().not())
    }
}
