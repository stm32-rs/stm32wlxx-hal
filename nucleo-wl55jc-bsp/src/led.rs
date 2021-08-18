//! LEDs

use stm32wl_hal as hal;

use hal::{
    embedded_hal::digital::v2::OutputPin,
    gpio::{self, pins, Output, OutputArgs},
};

const LED_ARGS: OutputArgs = OutputArgs {
    speed: gpio::Speed::Fast,
    level: gpio::Level::Low,
    ot: gpio::OutputType::PushPull,
    pull: gpio::Pull::None,
};

/// Simple trait for an LED
pub trait Led<OutPin>
where
    OutPin: OutputPin<Error = core::convert::Infallible>,
{
    /// Output pin driving the LED.
    fn output(&mut self) -> &mut OutPin;

    /// Set the LED on.
    fn set_on(&mut self) {
        self.output().set_high().unwrap()
    }

    /// Set the LED off.
    fn set_off(&mut self) {
        self.output().set_low().unwrap()
    }

    /// Toggle the LED state.
    fn toggle(&mut self) {
        self.gpio.set_level(self.gpio.level().toggle())
    }
}

/// Red LED
///
/// Marked as LED3 on the PCB
#[derive(Debug)]
pub struct Red {
    gpio: Output<pins::B11>,
}

impl Red {
    /// Create a new red LED.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortB, pac},
    ///     led::{self, Led},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let mut red = led::Red::new(gpiob.pb11);
    /// red.set_on();
    /// ```
    pub fn new(b11: pins::B11) -> Self {
        Self {
            gpio: Output::new(b11, &LED_ARGS),
        }
    }

    /// Free the GPIO pin from the LED struct.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortB, pac},
    ///     led::{self, Led},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let mut red = led::Red::new(gpiob.pb11);
    /// // ... use LED
    /// let pb11 = red.free();
    /// ```
    pub fn free(self) -> pins::B11 {
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
    /// use nucleo_wl55jc_bsp::led::Red;
    ///
    /// // ... setup happens here
    ///
    /// let red: Red = unsafe { Red::steal() };
    /// ```
    pub unsafe fn steal() -> Self {
        Self {
            gpio: Output::steal(),
        }
    }
}

impl Led<Output<pins::B11>> for Red {
    fn output(&mut self) -> &mut Output<pins::B11> {
        &mut self.gpio
    }
}

/// Green LED
///
/// Marked as LED2 on the PCB
#[derive(Debug)]
pub struct Green {
    gpio: Output<pins::B9>,
}

impl Green {
    /// Create a new green LED.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortB, pac},
    ///     led::{self, Led},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let mut green = led::Green::new(gpiob.pb9);
    /// green.set_on();
    /// ```
    pub fn new(b9: pins::B9) -> Self {
        Self {
            gpio: Output::new(b9, &LED_ARGS),
        }
    }

    /// Free the GPIO pin from the LED struct.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortB, pac},
    ///     led::{self, Led},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let mut green = led::Green::new(gpiob.pb9);
    /// // ... use LED
    /// let pb9 = green.free();
    /// ```
    pub fn free(self) -> pins::B9 {
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
    /// use nucleo_wl55jc_bsp::led::Green;
    ///
    /// // ... setup happens here
    ///
    /// let green: Green = unsafe { Green::steal() };
    /// ```
    pub unsafe fn steal() -> Self {
        Self {
            gpio: Output::steal(),
        }
    }
}

impl Led<Output<pins::B9>> for Green {
    fn output(&mut self) -> &mut Output<pins::B9> {
        &mut self.gpio
    }
}

/// Blue LED
///
/// Marked as LED1 on the PCB
#[derive(Debug)]
pub struct Blue {
    gpio: Output<pins::B15>,
}

impl Blue {
    /// Create a new blue LED.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortB, pac},
    ///     led::{self, Led},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let mut blue = led::Blue::new(gpiob.pb15);
    /// blue.set_on();
    /// ```
    pub fn new(b15: pins::B15) -> Self {
        Self {
            gpio: Output::new(b15, &LED_ARGS),
        }
    }

    /// Free the GPIO pin from the LED struct.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortB, pac},
    ///     led::{self, Led},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let mut blue = led::Blue::new(gpiob.pb15);
    /// // ... use LED
    /// let pb15 = blue.free();
    /// ```
    pub fn free(self) -> pins::B15 {
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
    /// use nucleo_wl55jc_bsp::led::Blue;
    ///
    /// // ... setup happens here
    ///
    /// let blue: Blue = unsafe { Blue::steal() };
    /// ```
    pub unsafe fn steal() -> Self {
        Self {
            gpio: Output::steal(),
        }
    }
}

impl Led<Output<pins::B15>> for Blue {
    fn output(&mut self) -> &mut Output<pins::B15> {
        &mut self.gpio
    }
}
