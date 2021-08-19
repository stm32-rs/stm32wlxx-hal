//! Push-buttons
use stm32wl_hal::gpio::{pins, Exti, Input, Pull};

const PULL: Pull = Pull::Up;

/// Push-button D0.
#[derive(Debug)]
pub struct D0 {
    gpio: Input<pins::A0>,
}

/// Push-button labeled "Boot".
#[derive(Debug)]
pub struct Boot {
    gpio: Input<pins::B13>,
}

/// Simple trait for a push-button
pub trait PushButton {
    /// Input pin for the push-button
    ///
    /// This can be used to access the EXTI trait for the pin.
    type Pin: Exti;

    /// Returns `True` if the button is currently being pushed.
    fn is_pushed(&self) -> bool;
}

impl PushButton for D0 {
    type Pin = pins::A0;

    #[inline]
    fn is_pushed(&self) -> bool {
        self.gpio.level().is_low()
    }
}

impl PushButton for Boot {
    type Pin = pins::B13;

    #[inline]
    fn is_pushed(&self) -> bool {
        self.gpio.level().is_low()
    }
}

impl D0 {
    /// Create a new push-button D0.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{gpio::PortA, pac},
    ///     pb::{PushButton, D0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let d0 = D0::new(gpioa.a0);
    /// ```
    pub fn new(a0: pins::A0) -> Self {
        Self {
            gpio: Input::new(a0, PULL),
        }
    }

    /// Free the GPIO pin from the push-button struct.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{gpio::PortA, pac},
    ///     pb::{PushButton, D0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let d0 = D0::new(gpioa.a0);
    /// // ... use push button
    /// let c0 = d0.free();
    /// ```
    pub fn free(self) -> pins::A0 {
        self.gpio.free()
    }

    /// Steal the push-button from whatever is currently using it.
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the push-button has exclusive access
    ///    to the underlying GPIO.
    ///    Singleton checks are bypassed with this method.
    /// 2. You are responsible for setting up the underlying GPIO correctly.
    ///    No setup will occur when using this method.
    ///
    /// # Example
    ///
    /// ```
    /// use lora_e5_bsp::pb::D0;
    ///
    /// // ... setup happens here
    ///
    /// let d0: D0 = unsafe { D0::steal() };
    /// ```
    pub unsafe fn steal() -> Self {
        Self {
            gpio: Input::steal(),
        }
    }
}

impl Boot {
    /// Create a new boot push-button.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{gpio::PortB, pac},
    ///     pb::{Boot, PushButton},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let boot = Boot::new(gpiob.b13);
    /// ```
    pub fn new(b13: pins::B13) -> Self {
        Self {
            gpio: Input::new(b13, PULL),
        }
    }

    /// Free the GPIO pin from the push-button struct.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use lora_e5_bsp::{
    ///     hal::{gpio::PortB, pac},
    ///     pb::{Boot, PushButton},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let boot = Boot::new(gpiob.b13);
    /// // ... use push button
    /// let b13 = boot.free();
    /// ```
    pub fn free(self) -> pins::B13 {
        self.gpio.free()
    }

    /// Steal the push-button from whatever is currently using it.
    ///
    /// This will **not** initialize the GPIO peripheral.
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the push-button has exclusive access
    ///    to the underlying GPIO.
    ///    Singleton checks are bypassed with this method.
    /// 2. You are responsible for setting up the underlying GPIO correctly.
    ///    No setup will occur when using this method.
    ///
    /// # Example
    ///
    /// ```
    /// use lora_e5_bsp::pb::Boot;
    ///
    /// // ... setup happens here
    ///
    /// let boot: Boot = unsafe { Boot::steal() };
    /// ```
    pub unsafe fn steal() -> Self {
        Self {
            gpio: Input::steal(),
        }
    }
}
