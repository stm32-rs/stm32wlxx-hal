//! Push-buttons
use stm32wl_hal::{
    gpio::{pins, Input, Pull},
    pac,
};

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

/// Push-button IRQ triggers.
#[derive(Debug)]
pub enum IrqTrig {
    /// Fire the interrupt when the button is pushed.
    OnPush,
    /// Fire the interrupt when the button is released.
    OnRelease,
    /// Fire the interrupt on both push and release.
    Both,
}

/// Simple trait for a push-button
pub trait PushButton {
    /// Returns `True` if the button is currently being pushed.
    fn is_pushed(&self) -> bool;

    /// Setup the push-button to fire an interrupt.
    ///
    /// This will:
    /// 1. Set the SYSCFG EXTICR to the push-button pin
    /// 2. Enable falling/rising triggers (or both)
    /// 3. Unmask the IRQ in the EXTI IMR
    ///
    /// This will **not** unmask the EXTI IRQ in the NVIC.
    fn setup_exti(syscfg: &mut pac::SYSCFG, exti: &mut pac::EXTI, tri: IrqTrig);

    /// Clear a pending IRQ in the EXTI for the push-button.
    fn clear_pending(exti: &mut pac::EXTI);
}

impl PushButton for D0 {
    fn is_pushed(&self) -> bool {
        self.gpio.level().is_low()
    }

    fn setup_exti(syscfg: &mut pac::SYSCFG, exti: &mut pac::EXTI, tri: IrqTrig) {
        syscfg.exticr1.modify(|_, w| w.exti0().pa0());
        if matches!(tri, IrqTrig::OnRelease | IrqTrig::Both) {
            exti.rtsr1.modify(|_, w| w.rt0().enabled());
        }
        if matches!(tri, IrqTrig::OnPush | IrqTrig::Both) {
            exti.ftsr1.modify(|_, w| w.ft0().enabled());
        }
        exti.c1imr1.modify(|_, w| w.im0().unmasked());
    }

    fn clear_pending(exti: &mut pac::EXTI) {
        exti.pr1.write(|w| w.pif0().set_bit());
    }
}

impl PushButton for Boot {
    fn is_pushed(&self) -> bool {
        self.gpio.level().is_low()
    }

    fn setup_exti(syscfg: &mut pac::SYSCFG, exti: &mut pac::EXTI, tri: IrqTrig) {
        syscfg.exticr4.modify(|_, w| w.exti13().pb13());
        if matches!(tri, IrqTrig::OnRelease | IrqTrig::Both) {
            exti.rtsr1.modify(|_, w| w.rt13().enabled());
        }
        if matches!(tri, IrqTrig::OnPush | IrqTrig::Both) {
            exti.ftsr1.modify(|_, w| w.ft13().enabled());
        }
        exti.c1imr1.modify(|_, w| w.im13().unmasked());
    }

    fn clear_pending(exti: &mut pac::EXTI) {
        exti.pr1.write(|w| w.pif13().set_bit());
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
    /// let d0 = D0::new(gpioc.pa0);
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
    /// let d0 = D0::new(gpioc.pa0);
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
    /// let boot = Boot::new(gpioc.b13);
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
    /// let boot = Boot::new(gpioc.b13);
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
