//! Push-buttons
use stm32wl_hal::gpio::{pins, Exti, Input, Pull};

const PULL: Pull = Pull::Up;

/// Push-button 3.
#[derive(Debug)]
pub struct Pb3 {
    gpio: Input<pins::C6>,
}

/// Push-button 2.
#[derive(Debug)]
pub struct Pb2 {
    gpio: Input<pins::A1>,
}

/// Push-button 1.
#[derive(Debug)]
pub struct Pb1 {
    gpio: Input<pins::A0>,
}

/// Simple trait for a push-button
pub trait PushButton {
    /// Input pin for the push-button
    ///
    /// This can be used to access the EXTI trait for the pin.
    ///
    /// # Example
    ///
    /// Setup EXTI to fire an interrupt when PB3 is pushed.
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{
    ///         gpio::{Exti, ExtiTrg, PortC},
    ///         pac,
    ///     },
    ///     pb::{Pb3, PushButton},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let pb3 = Pb3::new(gpioc.c6);
    ///
    /// <Pb3 as PushButton>::Pin::setup_exti_c1(&mut dp.EXTI, &mut dp.SYSCFG, ExtiTrg::Falling);
    /// ```
    type Pin: Exti;

    /// Returns `True` if the button is currently being pushed.
    fn is_pushed(&self) -> bool;
}

impl PushButton for Pb3 {
    type Pin = pins::C6;

    #[inline]
    fn is_pushed(&self) -> bool {
        self.gpio.level().is_low()
    }
}

impl PushButton for Pb2 {
    type Pin = pins::A1;

    #[inline]
    fn is_pushed(&self) -> bool {
        self.gpio.level().is_low()
    }
}

impl PushButton for Pb1 {
    type Pin = pins::A0;

    #[inline]
    fn is_pushed(&self) -> bool {
        self.gpio.level().is_low()
    }
}

impl Pb3 {
    /// Create a new push-button 3.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortC, pac},
    ///     pb::{Pb3, PushButton},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let pb3 = Pb3::new(gpioc.c6);
    /// ```
    pub fn new(c6: pins::C6) -> Self {
        Self {
            gpio: Input::new(c6, PULL),
        }
    }

    /// Free the GPIO pin from the push-button struct.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortC, pac},
    ///     pb::{Pb3, PushButton},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let pb3 = Pb3::new(gpioc.c6);
    /// // ... use push button
    /// let c6 = pb3.free();
    /// ```
    pub fn free(self) -> pins::C6 {
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
    /// use nucleo_wl55jc_bsp::pb::Pb3;
    ///
    /// // ... setup happens here
    ///
    /// let pb3: Pb3 = unsafe { Pb3::steal() };
    /// ```
    pub unsafe fn steal() -> Self {
        Self {
            gpio: Input::steal(),
        }
    }
}

impl Pb2 {
    /// Create a new push-button 2.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortA, pac},
    ///     pb::{Pb2, PushButton},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let pb2 = Pb2::new(gpioa.a1);
    /// ```
    pub fn new(a1: pins::A1) -> Self {
        Self {
            gpio: Input::new(a1, PULL),
        }
    }

    /// Free the GPIO pin from the push-button struct.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortA, pac},
    ///     pb::{Pb2, PushButton},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let pb2 = Pb2::new(gpioa.a1);
    /// // ... use push button
    /// let a1 = pb2.free();
    /// ```
    pub fn free(self) -> pins::A1 {
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
    /// use nucleo_wl55jc_bsp::pb::Pb2;
    ///
    /// // ... setup happens here
    ///
    /// let pb2: Pb2 = unsafe { Pb2::steal() };
    /// ```
    pub unsafe fn steal() -> Self {
        Self {
            gpio: Input::steal(),
        }
    }
}

impl Pb1 {
    /// Create a new push-button 2.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortA, pac},
    ///     pb::{Pb1, PushButton},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let pb1 = Pb1::new(gpioa.a0);
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
    /// use nucleo_wl55jc_bsp::{
    ///     hal::{gpio::PortA, pac},
    ///     pb::{Pb1, PushButton},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let pb1 = Pb1::new(gpioa.a0);
    /// // ... use push button
    /// let a0 = pb1.free();
    /// ```
    pub fn free(self) -> pins::A0 {
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
    /// use nucleo_wl55jc_bsp::pb::Pb1;
    ///
    /// // ... setup happens here
    ///
    /// let pb1: Pb1 = unsafe { Pb1::steal() };
    /// ```
    pub unsafe fn steal() -> Self {
        Self {
            gpio: Input::steal(),
        }
    }
}
