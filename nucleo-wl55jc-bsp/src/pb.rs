//! Push-buttons
use stm32wl_hal::{
    gpio::{pins, Input, Pull},
    pac,
};

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
    /// 3. Unmask the IRQ in the EXTI IMR (hard-coded for core 1)
    ///
    /// This will **not** unmask the EXTI IRQ in the NVIC.
    fn setup_exti(syscfg: &mut pac::SYSCFG, exti: &mut pac::EXTI, tri: IrqTrig);

    /// Clear a pending IRQ in the EXTI for the push-button.
    fn clear_pending(exti: &mut pac::EXTI);
}

impl PushButton for Pb3 {
    fn is_pushed(&self) -> bool {
        self.gpio.level().is_low()
    }

    fn setup_exti(syscfg: &mut pac::SYSCFG, exti: &mut pac::EXTI, tri: IrqTrig) {
        syscfg.exticr2.modify(|_, w| w.exti6().pc6());
        if matches!(tri, IrqTrig::OnRelease | IrqTrig::Both) {
            exti.rtsr1.modify(|_, w| w.rt6().enabled());
        }
        if matches!(tri, IrqTrig::OnPush | IrqTrig::Both) {
            exti.ftsr1.modify(|_, w| w.ft6().enabled());
        }
        exti.c1imr1.modify(|_, w| w.im6().unmasked());
    }

    fn clear_pending(exti: &mut pac::EXTI) {
        exti.pr1.write(|w| w.pif6().set_bit());
    }
}

impl PushButton for Pb2 {
    fn is_pushed(&self) -> bool {
        self.gpio.level().is_low()
    }

    fn setup_exti(syscfg: &mut pac::SYSCFG, exti: &mut pac::EXTI, tri: IrqTrig) {
        syscfg.exticr1.modify(|_, w| w.exti1().pa1());
        if matches!(tri, IrqTrig::OnRelease | IrqTrig::Both) {
            exti.rtsr1.modify(|_, w| w.rt1().enabled());
        }
        if matches!(tri, IrqTrig::OnPush | IrqTrig::Both) {
            exti.ftsr1.modify(|_, w| w.ft1().enabled());
        }
        exti.c1imr1.modify(|_, w| w.im1().unmasked());
    }

    fn clear_pending(exti: &mut pac::EXTI) {
        exti.pr1.write(|w| w.pif1().set_bit());
    }
}

impl PushButton for Pb1 {
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
    /// let pb3 = Pb3::new(gpioc.pc6);
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
    /// let pb3 = Pb3::new(gpioc.pc6);
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
    /// let pb2 = Pb2::new(gpioa.pa1);
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
    /// let pb2 = Pb2::new(gpioa.pa1);
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
    /// let pb1 = Pb1::new(gpioa.pa0);
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
    /// let pb1 = Pb1::new(gpioa.pa0);
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
