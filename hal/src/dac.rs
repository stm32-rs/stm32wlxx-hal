//! Digital to analog converter

use super::pac;

use crate::gpio::{pins::A10, Analog};
use pac::dac::mcr::MODE1_A;

/// ADC modes that use the A10 output pin
///
/// The DAC is connected to the A10 output pin in all these modes.
/// Some modes may also connect to the chip peripherals in addition to the
/// A10 output pin.
///
/// The output buffer can be enabled to allow a high drive capability.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[repr(u8)]
pub enum ModePin {
    /// Normal mode with the buffer enabled
    NormBuf = 0b000,
    /// Normal mode with the buffer enabled and output to chip peripherals
    NormChipBuf = 0b001,
    /// Normal mode with the buffer disabled
    NormNoBuf = 0b010,
    /// Sample and hold with the buffer enabled
    SampleHoldBuf = 0b100,
    /// Sample and hold with the buffer enabled and output to chip peripherals
    SampleHoldChipBuf = 0b101,
    /// Sample and hold with the buffer disabled
    SampleHoldNoBuf = 0b110,
}

impl From<ModePin> for MODE1_A {
    fn from(mp: ModePin) -> Self {
        match mp {
            ModePin::NormBuf => MODE1_A::NormalPinBuffer,
            ModePin::NormChipBuf => MODE1_A::NormalPinChipBuffer,
            ModePin::NormNoBuf => MODE1_A::NormalPinNoBuffer,
            ModePin::SampleHoldBuf => MODE1_A::ShpinBuffer,
            ModePin::SampleHoldChipBuf => MODE1_A::ShpinChipBuffer,
            ModePin::SampleHoldNoBuf => MODE1_A::ShpinNoBuffer,
        }
    }
}

/// DAC modes with output to chip peripherals
///
/// The DAC does not output to the A10 pin in these modes.
///
/// The DAC is connected to the chip peripherals in these modes, and has
/// no output buffer.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[repr(u8)]
pub enum ModeChip {
    /// Sample and hold
    SampleHold = 0b111,
    /// Normal mode
    Norm = 0b011,
}

impl From<ModeChip> for MODE1_A {
    fn from(mc: ModeChip) -> Self {
        match mc {
            ModeChip::SampleHold => MODE1_A::ShchipNoBuffer,
            ModeChip::Norm => MODE1_A::NormalChipNoBuffer,
        }
    }
}

/// Digital to analog converter driver
#[derive(Debug)]
pub struct Dac {
    dac: pac::DAC,
    out: Option<Analog<A10>>,
}

impl Dac {
    /// Create a new DAC driver from a DAC peripheral.
    ///
    /// This will enable clocks and reset the DAC peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dac::Dac, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let mut dac = Dac::new(dp.DAC, &mut dp.RCC);
    /// ```
    pub fn new(dac: pac::DAC, rcc: &mut pac::RCC) -> Dac {
        Self::enable_clock(rcc);
        unsafe { Self::pulse_reset(rcc) };

        let mut dac: Dac = Dac { dac, out: None };
        // we do not yet have ownership of A10
        dac.set_mode_chip(ModeChip::Norm);
        dac
    }

    /// Steal the DAC peripheral from whatever is currently using it.
    ///
    /// This will **not** initialize the DAC peripheral (unlike [`new`]).
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the DAC has exclusive access to the
    ///    peripheral. Singleton checks are bypassed with this method.
    /// 2. You are responsible for setting up the DAC correctly.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     dac::{Dac, ModeChip},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// Dac::enable_clock(&mut dp.RCC);
    /// // safety:
    /// // 1. Nothing else is using the DAC in this code.
    /// // 2. This code performs setup for the DAC.
    /// unsafe { Dac::pulse_reset(&mut dp.RCC) };
    ///
    /// // safety:
    /// // 1. Nothing else is using the DAC in this code.
    /// // 2. The DAC has been setup, clocks are enabled and the DAC has been reset.
    /// // 3. We do not have ownership of A10 so we switch to a non-pin mode.
    /// let mut dac = unsafe { Dac::steal() };
    /// dac.set_mode_chip(ModeChip::Norm);
    /// ```
    ///
    /// [`new`]: Dac::new
    pub unsafe fn steal() -> Dac {
        let dp: pac::Peripherals = pac::Peripherals::steal();
        Dac {
            dac: dp.DAC,
            out: None,
        }
    }

    /// Free the DAC peripheral from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dac::Dac, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let dac = dp.DAC;
    ///
    /// let mut dac_driver = Dac::new(dac, &mut dp.RCC);
    /// // ... use DAC
    /// let dac = dac_driver.free();
    /// ```
    pub fn free(self) -> pac::DAC {
        self.dac
    }

    /// Unmask the DAC interrupt.
    ///
    /// # Safety
    ///
    /// This can break mask-based critical sections.
    ///
    /// # Example
    ///
    /// ```no_run
    /// unsafe { stm32wlxx_hal::dac::Dac::unmask_irq() };
    /// ```
    #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    pub unsafe fn unmask_irq() {
        pac::NVIC::unmask(pac::Interrupt::DAC)
    }

    /// Mask the DAC interrupt.
    ///
    /// # Example
    ///
    /// ```no_run
    /// stm32wlxx_hal::dac::Dac::mask_irq();
    /// ```
    #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    pub fn mask_irq() {
        pac::NVIC::mask(pac::Interrupt::DAC)
    }

    /// Enable the DAC clock.
    ///
    /// This is done for you in [`new`](Dac::new)
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dac::Dac, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// Dac::enable_clock(&mut dp.RCC);
    /// ```
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr1.modify(|_, w| w.dac1en().enabled());
    }

    /// Disable the DAC clock.
    ///
    /// # Safety
    ///
    /// 1. You are responsible for ensuring the DAC is in a state where the
    ///    clock can be disabled without entering an error state.
    /// 2. You cannot use the DAC while the clock is disabled.
    /// 3. You are responsible for re-enabling the clock before resuming use
    ///    of the DAC.
    /// 4. You are responsible for setting up anything that may have lost state
    ///    while the clock was disabled.
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr1.modify(|_, w| w.dac1en().disabled());
    }

    /// Reset the DAC.
    ///
    /// # Safety
    ///
    /// 1. The DAC must not be in-use.
    /// 2. You are responsible for setting up the DAC after a reset.
    /// 3. After reset the DAC will be using pin A10, if you do not have
    ///    ownership of this pin switch the DAC to a non-pin mode with
    ///    [`set_mode_chip`](Dac::set_mode_chip).
    ///
    /// # Example
    ///
    /// See [`steal`](Dac::steal)
    pub unsafe fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb1rstr1.modify(|_, w| w.dacrst().set_bit());
        rcc.apb1rstr1.modify(|_, w| w.dacrst().clear_bit());
    }

    /// Set the DAC mode with the A10 output pin.
    ///
    /// # Panics
    ///
    /// * (debug) DAC channel is enabled
    /// * (debug) DAC calibration is enabled
    ///
    /// # Example
    ///
    /// Normal mode
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     dac::{Dac, ModePin},
    ///     gpio::{Analog, PortA},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut dac: Dac = Dac::new(dp.DAC, &mut dp.RCC);
    /// let mut gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    ///
    /// cortex_m::interrupt::free(|cs| dac.set_mode_pin(Analog::new(gpioa.a10, cs), ModePin::NormBuf));
    /// ```
    ///
    /// Sample and hold.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     dac::{Dac, ModePin},
    ///     gpio::{Analog, PortA},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the LSI clock for DAC sample and hold mode
    /// dp.RCC.csr.modify(|_, w| w.lsion().on());
    /// while dp.RCC.csr.read().lsirdy().is_not_ready() {}
    ///
    /// let mut dac: Dac = Dac::new(dp.DAC, &mut dp.RCC);
    /// let mut gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    ///
    /// cortex_m::interrupt::free(|cs| {
    ///     dac.set_mode_pin(Analog::new(gpioa.a10, cs), ModePin::SampleHoldBuf)
    /// });
    /// ```
    pub fn set_mode_pin(&mut self, a10: Analog<A10>, mode: ModePin) {
        let cr = self.dac.cr.read();
        debug_assert!(cr.cen1().bit_is_clear());
        debug_assert!(cr.en1().bit_is_clear());
        self.dac.mcr.write(|w| w.mode1().variant(mode.into()));
        // RM0453 rev1 page 397
        // "For ADC, DAC and COMP, configure the desired I/O in analog mode in
        // the GPIOx_MODER register and configure the required function in the
        // ADC, DAC and COMP registers."
        self.out = Some(a10)
    }

    /// Set the DAC mode to output to on-chip peripherals.
    ///
    /// If A10 is currently in-use by the DAC this method returns the A10 pin.
    /// This method is the only way to retrieve A10 if the DAC has ownership
    /// of the pin.
    ///
    /// # Panics
    ///
    /// * (debug) DAC channel is enabled
    /// * (debug) DAC calibration is enabled
    ///
    /// # Example
    ///
    /// Normal mode.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     dac::{Dac, ModeChip},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut dac: Dac = Dac::new(dp.DAC, &mut dp.RCC);
    /// dac.set_mode_chip(ModeChip::Norm);
    /// ```
    ///
    /// Sample and hold mode.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     dac::{Dac, ModeChip},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the LSI clock for DAC sample and hold mode
    /// dp.RCC.csr.modify(|_, w| w.lsion().on());
    /// while dp.RCC.csr.read().lsirdy().is_not_ready() {}
    ///
    /// let mut dac: Dac = Dac::new(dp.DAC, &mut dp.RCC);
    /// dac.set_mode_chip(ModeChip::SampleHold);
    /// ```
    ///
    /// Retrieving the A10 pin.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     dac::{Dac, ModeChip, ModePin},
    ///     gpio::{pins, Analog, PortA},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut dac: Dac = Dac::new(dp.DAC, &mut dp.RCC);
    /// let mut gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    ///
    /// cortex_m::interrupt::free(|cs| dac.set_mode_pin(Analog::new(gpioa.a10, cs), ModePin::NormBuf));
    /// let a10: Analog<pins::A10> = dac.set_mode_chip(ModeChip::Norm).unwrap();
    /// ```
    pub fn set_mode_chip(&mut self, mode: ModeChip) -> Option<Analog<A10>> {
        let cr = self.dac.cr.read();
        debug_assert!(cr.cen1().bit_is_clear());
        debug_assert!(cr.en1().bit_is_clear());
        self.dac.mcr.write(|w| w.mode1().variant(mode.into()));
        self.out.take()
    }

    /// Setup the DAC for use with a software trigger.
    ///
    /// This will enable the DAC.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dac::Dac, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut dac: Dac = Dac::new(dp.DAC, &mut dp.RCC);
    /// dac.setup_soft_trigger();
    /// ```
    pub fn setup_soft_trigger(&mut self) {
        self.dac.cr.write(|w| {
            w.cen1().normal();
            w.tsel1().swtrig();
            w.ten1().enabled();
            w.en1().enabled()
        });
    }

    /// Disable the DAC.
    pub fn disable(&mut self) {
        self.dac.cr.write(|w| w.en1().disabled());
    }

    /// Set the value of the DAC output.
    ///
    /// The DAC should be setup for use with a software trigger with
    /// [`setup_soft_trigger`](Dac::setup_soft_trigger) before calling this
    /// method.
    ///
    /// # Panics
    ///
    /// * (debug) DAC is not trigger is not set to software
    /// * (debug) DAC is not enabled
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dac::Dac, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut dac: Dac = Dac::new(dp.DAC, &mut dp.RCC);
    ///
    /// dac.setup_soft_trigger();
    /// dac.soft_trigger(12);
    /// ```
    pub fn soft_trigger(&mut self, val: u16) {
        let cr = self.dac.cr.read();
        debug_assert!(cr.tsel1().is_swtrig());
        debug_assert!(cr.en1().is_enabled());
        self.dac.dhr12r1.write(|w| w.dacc1dhr().bits(val));
        self.dac.swtrgr.write(|w| w.swtrig1().trigger());
    }

    /// Get the current DAC output.
    ///
    /// **Note:** Only the lower 12 bits of the return value are used.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dac::Dac, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut dac: Dac = Dac::new(dp.DAC, &mut dp.RCC);
    ///
    /// dac.setup_soft_trigger();
    /// dac.soft_trigger(1234);
    /// assert_eq!(dac.out(), 1234);
    /// ```
    pub fn out(&self) -> u16 {
        self.dac.dor1.read().dacc1dor().bits()
    }

    /// Set the offset trim value.
    ///
    /// **Note:** Only the lower 5 bits of the `trim` value are used.
    #[allow(dead_code)]
    fn set_trim(&mut self, trim: u8) {
        self.dac.ccr.write(|w| unsafe { w.otrim1().bits(trim) })
    }

    /// Set the sample time in number of LSI cycles.
    ///
    /// This is valid only in sample and hold mode.
    ///
    /// **Note:** Only the lower 9 bits of the `cycles` value are used.
    ///
    /// # Panics
    ///
    /// * (debug) DAC not in sample and hold mode
    pub fn set_sample_cycles(&mut self, cycles: u16) {
        debug_assert_eq!(self.dac.mcr.read().mode1().bits() & 0b100, 0);
        // 3 LSI periods maximum
        while self.dac.sr.read().bwst1().bit_is_set() {}
        self.dac.shsr1.write(|w| w.tsample1().bits(cycles))
    }

    /// Set the hold time in number of LSI cycles.
    ///
    /// This is valid only in sample and hold mode.
    ///
    /// **Note:** Only the lower 9 bits of the `cycles` value are used.
    ///
    /// # Panics
    ///
    /// * (debug) DAC channel is enabled
    /// * (debug) DAC calibration is enabled
    /// * (debug) DAC not in sample and hold mode
    pub fn set_hold_cycles(&mut self, cycles: u16) {
        debug_assert_eq!(self.dac.mcr.read().mode1().bits() & 0b100, 0);
        let cr = self.dac.cr.read();
        debug_assert!(cr.cen1().bit_is_clear());
        debug_assert!(cr.en1().bit_is_clear());
        self.dac.shhr.write(|w| w.thold1().bits(cycles))
    }

    /// Set the refresh time in number of LSI cycles.
    ///
    /// This is valid only in sample and hold mode.
    ///
    /// # Panics
    ///
    /// * (debug) DAC channel is enabled
    /// * (debug) DAC calibration is enabled
    /// * (debug) DAC not in sample and hold mode
    pub fn set_refresh_cycles(&mut self, cycles: u8) {
        debug_assert_eq!(self.dac.mcr.read().mode1().bits() & 0b100, 0);
        let cr = self.dac.cr.read();
        debug_assert!(cr.cen1().bit_is_clear());
        debug_assert!(cr.en1().bit_is_clear());
        self.dac.shrr.write(|w| w.trefresh1().bits(cycles))
    }
}
