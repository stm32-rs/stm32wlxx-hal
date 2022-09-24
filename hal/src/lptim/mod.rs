//! Low-power timers
//!
//! The low power timers implements the `embedded-hal`
//! [`PwmPin`](embedded_hal::PwmPin) and
//! [`CountDown`](embedded_hal::timer::CountDown) traits.
//!
//! # Example
//!
//! Setup a one-shot timer.
//!
//! ```no_run
//! use stm32wlxx_hal::{
//!     embedded_hal::timer::CountDown,
//!     lptim::{self, LpTim, LpTim1, Prescaler::Div1},
//!     pac,
//! };
//!
//! let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
//!
//! // enable the HSI16 source clock
//! dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
//! while dp.RCC.cr.read().hsirdy().is_not_ready() {}
//!
//! let mut lptim1: LpTim1 = LpTim1::new(dp.LPTIM1, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
//! // wait 16,000 cycles at 16MHz
//! lptim1.start(16_000_u16);
//! nb::block!(lptim1.wait());
//! ```

mod cfgr;
mod cr;

pub use cfgr::{Cfgr, Filter, Prescaler, TrgPol, TrgSel, TrgSel3};
use cortex_m::interrupt::CriticalSection;
pub use cr::Cr;

use crate::{
    gpio::{
        pins,
        sealed::{LpTim1Etr, LpTim1Out, LpTim2Etr, LpTim2Out, LpTim3Etr, LpTim3Out},
    },
    pac, Ratio,
};
use core::cmp::min;
use paste::paste;
use void::Void;

/// Timer IRQs.
pub mod irq {
    /// Repetition register update OK.
    ///
    /// Set by hardware when the APB bus write to the RCR register has been
    /// successfully completed.
    pub const REPOK: u32 = 1 << 8;
    /// Update event occurred.
    pub const UE: u32 = 1 << 7;
    /// Counter direction change up to down.
    ///
    /// Set by hardware when the counter direction changes from up to down.
    pub const DOWN: u32 = 1 << 6;
    /// Counter direction change down to up.
    ///
    /// Set by hardware when the counter direction changes from down to up.
    pub const UP: u32 = 1 << 5;
    /// Autoreload register update OK.
    ///
    /// Set by hardware when the APB bus write to the ARR register has been
    /// successfully completed.
    pub const ARROK: u32 = 1 << 4;
    /// Compare register update OK.
    ///
    /// Set by hardware when the APB bus write to the CMP register has been
    /// successfully completed.
    pub const CMPOK: u32 = 1 << 3;
    /// External trigger edge event.
    ///
    /// Set by hardware when a valid edge on the selected external trigger
    /// input has occurred.
    pub const EXTTRIG: u32 = 1 << 2;
    /// Autoreload match.
    ///
    /// Set by hardware when the count value has reached the auto-reload value.
    pub const ARRM: u32 = 1 << 1;
    /// Compare match.
    ///
    /// Set by hardware when the count value has reached the compare value.
    pub const CMPM: u32 = 1;

    /// Mask of all IRQs.
    pub const ALL: u32 = REPOK | UE | DOWN | UP | ARROK | CMPOK | EXTTRIG | ARRM | CMPM;
}

use sealed::{LpTim as SealedLpTim, LpTimBase};
pub(crate) mod sealed {
    use super::{Cfgr, Cr, Ratio};

    pub trait LpTimBase {
        fn isr() -> u32;
        fn set_icr(&mut self, icr: u32);
        fn set_ier(&mut self, ier: u32);
        fn cfgr(&self) -> Cfgr;
        fn set_cfgr(&mut self, cfgr: Cfgr);
        fn modify_cfgr<F: FnOnce(Cfgr) -> Cfgr>(&mut self, f: F);
        fn cr(&self) -> Cr;
        fn set_cr(&mut self, cr: Cr);
        fn cmp(&self) -> u16;
        fn set_cmp(&mut self, cmp: u16);
        fn autoreload(&self) -> u16;
        fn set_autoreload(&mut self, ar: u16);
        unsafe fn cnt() -> u16;
        fn set_or(&mut self, or: u32);
        fn set_rep(&mut self, rep: u8);
    }

    pub trait LpTim {
        type Pac: LpTimBase;
        fn as_tim(&self) -> &Self::Pac;
        fn as_mut_tim(&mut self) -> &mut Self::Pac;
        fn _hz(&self) -> &Ratio<u32>;
    }
}

macro_rules! impl_lptim_base_for {
    ($lptim:ident) => {
        impl LpTimBase for pac::$lptim {
            #[inline(always)]
            fn isr() -> u32 {
                // safety: atomic read with no side effects
                unsafe { (*pac::$lptim::PTR).isr.read().bits() }
            }

            #[inline(always)]
            fn set_icr(&mut self, icr: u32) {
                // safety: reserved bits are masked
                self.icr.write(|w| unsafe { w.bits(icr & irq::ALL) })
            }

            #[inline(always)]
            fn set_ier(&mut self, ier: u32) {
                // safety: reserved bits are masked
                self.ier.write(|w| unsafe { w.bits(ier & irq::ALL) })
            }

            #[inline(always)]
            fn cfgr(&self) -> Cfgr {
                self.cfgr.read().bits().into()
            }

            #[inline(always)]
            fn set_cfgr(&mut self, cfgr: Cfgr) {
                // safety: reserved bits are masked
                self.cfgr
                    .write(|w| unsafe { w.bits(u32::from(cfgr) & 0x01FE_EEDF) })
            }

            #[inline(always)]
            fn modify_cfgr<F: FnOnce(Cfgr) -> Cfgr>(&mut self, f: F) {
                self.set_cfgr(f(self.cfgr()))
            }

            #[inline(always)]
            fn cr(&self) -> Cr {
                self.cr.read().bits().into()
            }

            #[inline(always)]
            fn set_cr(&mut self, cr: Cr) {
                // safety: reserved bits are masked
                self.cr.write(|w| unsafe { w.bits(u32::from(cr) & 0x1F) })
            }

            #[inline(always)]
            fn cmp(&self) -> u16 {
                self.cmp.read().cmp().bits()
            }

            #[inline(always)]
            fn set_cmp(&mut self, cmp: u16) {
                self.cmp.write(|w| w.cmp().bits(cmp));
            }

            #[inline(always)]
            fn autoreload(&self) -> u16 {
                self.arr.read().arr().bits()
            }

            #[inline(always)]
            fn set_autoreload(&mut self, ar: u16) {
                self.arr.write(|w| w.arr().bits(ar));
            }

            /// # Safety
            ///
            /// This read will reset the counter if RSTARE in the CR register
            /// is set.
            #[inline(always)]
            unsafe fn cnt() -> u16 {
                (*pac::$lptim::PTR).cnt.read().cnt().bits()
            }

            #[inline(always)]
            fn set_or(&mut self, or: u32) {
                // safety: reserved bits are masked
                self.or.write(|w| unsafe { w.bits(or & 0b11) })
            }

            #[inline(always)]
            fn set_rep(&mut self, rep: u8) {
                self.rcr.write(|w| w.rep().bits(rep))
            }
        }
    };
}

impl_lptim_base_for!(LPTIM1);
impl_lptim_base_for!(LPTIM2);
impl_lptim_base_for!(LPTIM3);

/// Timer clock selection.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[repr(u8)]
pub enum Clk {
    /// PCLK selected.
    Pclk = 0b00,
    /// LSI clock selected.
    Lsi = 0b01,
    /// HSI16 clock selected.
    Hsi16 = 0b10,
    /// LSE clock selected.
    Lse = 0b11,
}

impl Clk {
    const fn from_bits(bits: u8) -> Clk {
        match bits & 0b11 {
            0b00 => Clk::Pclk,
            0b01 => Clk::Lsi,
            0b10 => Clk::Hsi16,
            _ => Clk::Lse,
        }
    }
}

impl From<u8> for Clk {
    fn from(bits: u8) -> Self {
        Self::from_bits(bits)
    }
}

macro_rules! paste_lptim {
    ($n:expr) => {
        paste! {
            #[doc = "Low-power timer " $n " driver."]
            #[derive(Debug)]
            pub struct [<LpTim $n>] {
                tim: pac::[<LPTIM $n>],
                hz: Ratio<u32>,
            }

            impl sealed::LpTim for [<LpTim $n>] {
                type Pac = pac::[<LPTIM $n>];

                #[inline(always)]
                fn as_tim(&self) -> &Self::Pac {
                    &self.tim
                }

                #[inline(always)]
                fn as_mut_tim(&mut self) -> &mut Self::Pac {
                    &mut self.tim
                }

                #[inline(always)]
                fn _hz(&self) -> &Ratio<u32> {
                    &self.hz
                }
            }
        }
    };
}

paste_lptim!(1);
paste_lptim!(2);
paste_lptim!(3);

/// Low-power timer trait.
pub trait LpTim: sealed::LpTim {
    /// Trigger selection options.
    type TrgSel: Into<u32>;

    /// Create a new LPTIM driver.
    ///
    /// This will enable the ADC clock and reset the ADC peripheral.
    ///
    /// **Note:** This will select the clock source, but you are responsible
    /// for enabling that clock source.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     lptim::{self, LpTim, LpTim1, Prescaler::Div1},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let lptim1: LpTim1 = LpTim1::new(dp.LPTIM1, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
    /// ```
    fn new(tim: Self::Pac, clk: Clk, pre: Prescaler, rcc: &mut pac::RCC) -> Self;

    /// Free the LPTIM registers from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     lptim::{self, LpTim, LpTim1, Prescaler::Div1},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let lptim1: LpTim1 = LpTim1::new(dp.LPTIM1, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
    /// // ... use lptim
    /// let lptim1: pac::LPTIM1 = lptim1.free();
    /// ```
    fn free(self) -> Self::Pac;

    /// Reset the LPTIM peripheral.
    ///
    /// [`new`](Self::new) will pulse reset for you.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the timer before calling this function.
    /// 2. You are responsible for setting up the timer after a reset.
    unsafe fn pulse_reset(rcc: &mut pac::RCC);

    /// Enable clocks for the LPTIM peripheral.
    ///
    /// [`new`](Self::new) will enable clocks for you.
    fn enable_clock(rcc: &mut pac::RCC);

    /// Disable the LPTIM peripheral clock.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the timer before disabling the clock.
    /// 2. You are responsible for re-enabling the clock before using the timer.
    unsafe fn disable_clock(rcc: &mut pac::RCC);

    /// Get the clock source.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     lptim::{self, LpTim, LpTim1},
    ///     pac,
    /// };
    ///
    /// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// // PCLK is the power-on-reset value
    /// assert_eq!(LpTim1::clk(&dp.RCC), lptim::Clk::Pclk);
    /// ```
    fn clk(rcc: &pac::RCC) -> Clk;

    /// Get the clock speed in hertz.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     lptim::{self, LpTim, LpTim1, Prescaler::Div1},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let lptim1: LpTim1 = LpTim1::new(dp.LPTIM1, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
    /// assert_eq!(lptim1.hz().to_integer(), 16_000_000);
    /// ```
    fn hz(&self) -> &Ratio<u32> {
        self._hz()
    }

    /// Get the timer count.
    #[inline]
    fn cnt() -> u16 {
        // safety: there is no interface for the user to set the RSTARE bit
        unsafe { Self::Pac::cnt() }
    }

    /// Get the interrupt status.
    #[inline]
    fn isr() -> u32 {
        Self::Pac::isr()
    }

    /// Enable and disable interrupts.
    ///
    /// # Panics
    ///
    /// * (debug) timer is enabled.
    ///
    /// # Example
    ///
    /// Enable all IRQs.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     lptim::{self, LpTim, LpTim1, Prescaler::Div1},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut lptim1: LpTim1 = LpTim1::new(dp.LPTIM1, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
    /// lptim1.set_ier(lptim::irq::ALL);
    /// ```
    #[inline]
    fn set_ier(&mut self, ier: u32) {
        debug_assert!(!self.is_enabled());
        self.as_mut_tim().set_ier(ier)
    }

    /// Clear interrupts.
    ///
    /// # Safety
    ///
    /// There is a big erratum entry for interrupts getting stuck, and the
    /// interrupts should only be cleared under certain circumstances
    /// (see workaround).
    ///
    /// ## Device may remain stuck in LPTIM interrupt when clearing event flag
    ///
    /// ### Description
    ///
    /// This limitation occurs when the LPTIM is configured in interrupt mode
    /// (at least one interrupt is enabled) and the software clears any flag in
    /// LPTIM_ISR register by writing its corresponding bit in LPTIM_ICR register.
    /// If the interrupt status flag corresponding to a disabled interrupt is
    /// cleared simultaneously with a new event detection,
    /// the set and clear commands might reach the APB domain at the same time,
    /// leading to an asynchronous interrupt signal permanently stuck high.
    /// This issue can occur either during an interrupt subroutine execution
    /// (where the flag clearing is usually done),
    /// or outside an interrupt subroutine.
    /// Consequently, the firmware remains stuck in the LPTIM interrupt routine,
    /// and the device cannot enter Stop mode.
    ///
    /// ### Workaround
    ///
    /// To avoid this issue, it is strongly advised to follow the recommendations
    /// listed below:
    ///
    /// * Clear the flag only when its corresponding interrupt is enabled in the
    ///   interrupt enable register.
    /// * If for specific reasons, it is required to clear some flags that have
    ///   corresponding interrupt lines disabled in the interrupt enable register,
    ///   it is recommended to clear them during the current subroutine prior to
    ///   those which have corresponding interrupt line enabled in the interrupt
    ///   enable register.
    /// * Flags must not be cleared outside the interrupt subroutine.
    #[inline]
    unsafe fn set_icr(&mut self, icr: u32) {
        self.as_mut_tim().set_icr(icr)
    }

    /// Returns `true` if the timer is enabled.
    #[inline]
    fn is_enabled(&self) -> bool {
        self.as_tim().cr().enabled()
    }

    /// Setup a non-pin trigger.
    ///
    /// # Panics
    ///
    /// * (debug) timer is enabled.
    fn setup_trigger(&mut self, filter: Filter, pol: TrgPol, sel: Self::TrgSel) {
        debug_assert!(!self.is_enabled());
        self.as_mut_tim().modify_cfgr(|w| {
            w.set_trg_sel(sel.into())
                .set_trg_pol(pol)
                .set_trg_filter(filter)
        })
    }

    /// Set the maximum duty cycle (autoreload value).
    ///
    /// This is used to control the frequency of the PWM output.
    ///
    /// This function does not poll for completion, use [`isr`](Self::isr)
    /// and [`ARROK`](irq::ARROK) to determine when the value has been updated.
    ///
    /// # Example
    ///
    /// Set the frequency to 50Hz (20ms period) for servo motor control.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     gpio,
    ///     lptim::{self, LpTim, LpTim1},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let pc: gpio::PortC = gpio::PortC::split(dp.GPIOC, &mut dp.RCC);
    ///
    /// let mut lptim1: LpTim1 = LpTim1::new(
    ///     dp.LPTIM1,
    ///     lptim::Clk::Hsi16,
    ///     lptim::Prescaler::Div8,
    ///     &mut dp.RCC,
    /// );
    /// cortex_m::interrupt::free(|cs| lptim1.new_output_pin(pc.c1, cs));
    ///
    /// // 20ms period to control PWM servo motors
    /// const SERVO_FREQ_HZ: u32 = 50;
    ///
    /// // source_freq / prescaler = 16_000_000 MHz / 8 = 2 Mhz
    /// let lptim1_freq_hz = lptim1.hz().to_integer();
    ///
    /// // 2MHz / 50Hz = 40_000
    /// let max_duty: u32 = lptim1_freq_hz / SERVO_FREQ_HZ;
    ///
    /// lptim1.set_max_duty(max_duty.try_into().unwrap());
    /// ```
    #[inline]
    fn set_max_duty(&mut self, duty: u16) {
        if !self.is_enabled() {
            const CR: Cr = Cr::RESET.enable();
            self.as_mut_tim().set_cr(CR);

            // RM0461 Rev 4 "Timer enable":
            // After setting the ENABLE bit, a delay of two counter
            // clock is needed before the LPTIM is actually enabled.
            const MAX_SYS_FREQ: u32 = 48_000_000;
            let delay: u32 = (MAX_SYS_FREQ * 2) / self.hz().to_integer();
            cortex_m::asm::delay(delay);
        }

        self.as_mut_tim().set_autoreload(duty);
    }
}

impl LpTim for LpTim1 {
    type TrgSel = TrgSel;

    fn new(mut tim: Self::Pac, clk: Clk, pre: Prescaler, rcc: &mut pac::RCC) -> Self {
        rcc.ccipr.modify(|_, w| w.lptim1sel().bits(clk as u8));
        unsafe { Self::pulse_reset(rcc) }
        Self::enable_clock(rcc);
        tim.set_cfgr(Cfgr::RESET.set_prescaler(pre).set_wavepol(true));

        let src: Ratio<u32> = match clk {
            Clk::Pclk => crate::rcc::apb1timx(rcc),
            Clk::Lsi => Ratio::new_raw(crate::rcc::lsi_hz(rcc).into(), 1),
            Clk::Hsi16 => Ratio::new_raw(16_000_000, 1),
            Clk::Lse => Ratio::new_raw(32_768, 1),
        };
        let hz: Ratio<u32> = src / pre.div().into();

        Self { tim, hz }
    }

    #[inline]
    fn free(self) -> Self::Pac {
        self.tim
    }

    #[inline]
    unsafe fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb1rstr1.modify(|_, w| w.lptim1rst().reset());
        rcc.apb1rstr1.modify(|_, w| w.lptim1rst().no_reset());
    }

    #[inline]
    fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr1.modify(|_, w| w.lptim1en().enabled());
        rcc.apb1enr1.read(); // delay after an RCC peripheral clock enabling
    }

    #[inline]
    unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr1.modify(|_, w| w.lptim1en().disabled());
    }

    #[inline]
    fn clk(rcc: &pac::RCC) -> Clk {
        rcc.ccipr.read().lptim1sel().bits().into()
    }
}

impl LpTim for LpTim2 {
    type TrgSel = TrgSel;

    fn new(mut tim: Self::Pac, clk: Clk, pre: Prescaler, rcc: &mut pac::RCC) -> Self {
        rcc.ccipr.modify(|_, w| w.lptim2sel().bits(clk as u8));
        unsafe { Self::pulse_reset(rcc) }
        Self::enable_clock(rcc);
        tim.set_cfgr(Cfgr::RESET.set_prescaler(pre).set_wavepol(true));

        let src: Ratio<u32> = match clk {
            Clk::Pclk => crate::rcc::apb1timx(rcc),
            Clk::Lsi => Ratio::new_raw(crate::rcc::lsi_hz(rcc).into(), 1),
            Clk::Hsi16 => Ratio::new_raw(16_000_000, 1),
            Clk::Lse => Ratio::new_raw(32_768, 1),
        };
        let hz: Ratio<u32> = src / pre.div().into();

        Self { tim, hz }
    }

    #[inline]
    fn free(self) -> Self::Pac {
        self.tim
    }

    #[inline]
    unsafe fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb1rstr2.modify(|_, w| w.lptim2rst().reset());
        rcc.apb1rstr2.modify(|_, w| w.lptim2rst().no_reset());
    }

    #[inline]
    fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr2.modify(|_, w| w.lptim2en().enabled());
        rcc.apb1enr2.read(); // delay after an RCC peripheral clock enabling
    }

    #[inline]
    unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr2.modify(|_, w| w.lptim2en().disabled());
    }

    #[inline]
    fn clk(rcc: &pac::RCC) -> Clk {
        rcc.ccipr.read().lptim2sel().bits().into()
    }
}

impl LpTim for LpTim3 {
    type TrgSel = TrgSel3;

    fn new(mut tim: Self::Pac, clk: Clk, pre: Prescaler, rcc: &mut pac::RCC) -> Self {
        rcc.ccipr.modify(|_, w| w.lptim3sel().bits(clk as u8));
        unsafe { Self::pulse_reset(rcc) }
        Self::enable_clock(rcc);
        tim.set_cfgr(Cfgr::RESET.set_prescaler(pre).set_wavepol(true));

        let src: Ratio<u32> = match clk {
            Clk::Pclk => crate::rcc::apb1timx(rcc),
            Clk::Lsi => Ratio::new_raw(crate::rcc::lsi_hz(rcc).into(), 1),
            Clk::Hsi16 => Ratio::new_raw(16_000_000, 1),
            Clk::Lse => Ratio::new_raw(32_768, 1),
        };
        let hz: Ratio<u32> = src / pre.div().into();

        Self { tim, hz }
    }

    #[inline]
    fn free(self) -> Self::Pac {
        self.tim
    }

    #[inline]
    unsafe fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb1rstr2.modify(|_, w| w.lptim3rst().reset());
        rcc.apb1rstr2.modify(|_, w| w.lptim3rst().no_reset());
    }

    #[inline]
    fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr2.modify(|_, w| w.lptim3en().enabled());
        rcc.apb1enr2.read(); // delay after an RCC peripheral clock enabling
    }

    #[inline]
    unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr2.modify(|_, w| w.lptim3en().disabled());
    }

    #[inline]
    fn clk(rcc: &pac::RCC) -> Clk {
        rcc.ccipr.read().lptim3sel().bits().into()
    }
}

macro_rules! impl_eh_countdown_for {
    ($tim:ident) => {
        impl embedded_hal::timer::CountDown for $tim {
            type Time = u16;

            fn start<T>(&mut self, count: T)
            where
                T: Into<Self::Time>,
            {
                // enable timer
                {
                    const CR: Cr = Cr::RESET.enable();
                    self.as_mut_tim().set_cr(CR);

                    // RM0461 Rev 4 "Timer enable":
                    // After setting the ENABLE bit, a delay of two counter
                    // clock is needed before the LPTIM is actually enabled.
                    const MAX_SYS_FREQ: u32 = 48_000_000;
                    let delay: u32 = (MAX_SYS_FREQ * 2) / self.hz().to_integer();
                    cortex_m::asm::delay(delay);
                }

                // can only be modified when enabled
                let count: u16 = count.into();
                self.as_mut_tim().set_autoreload(count);
                while Self::isr() & irq::ARROK == 0 {}
                unsafe { self.set_icr(irq::ARROK) };

                {
                    const CR: Cr = Cr::RESET.enable().set_single();
                    self.as_mut_tim().set_cr(CR);
                }
            }

            fn wait(&mut self) -> nb::Result<(), Void> {
                if Self::isr() & irq::UE == 0 {
                    Err(nb::Error::WouldBlock)
                } else {
                    unsafe { self.set_icr(irq::UE) };
                    Ok(())
                }
            }
        }
    };
}

impl_eh_countdown_for!(LpTim1);
impl_eh_countdown_for!(LpTim2);
impl_eh_countdown_for!(LpTim3);

macro_rules! impl_eh_pwmpin_for {
    ($tim:ident) => {
        impl embedded_hal::PwmPin for $tim {
            type Duty = u16;

            fn disable(&mut self) {
                self.as_mut_tim().set_cr(Cr::DISABLE);
            }

            fn enable(&mut self) {
                const CR: Cr = Cr::RESET.enable().set_continuous();
                self.as_mut_tim().set_cr(CR);
            }

            fn get_duty(&self) -> Self::Duty {
                self.as_tim().cmp()
            }

            fn get_max_duty(&self) -> Self::Duty {
                self.as_tim().autoreload()
            }

            fn set_duty(&mut self, duty: Self::Duty) {
                if !self.is_enabled() {
                    const CR: Cr = Cr::RESET.enable();
                    self.as_mut_tim().set_cr(CR);

                    // RM0461 Rev 4 "Timer enable":
                    // After setting the ENABLE bit, a delay of two counter
                    // clock is needed before the LPTIM is actually enabled.
                    const MAX_SYS_FREQ: u32 = 48_000_000;
                    let delay: u32 = (MAX_SYS_FREQ * 2) / self.hz().to_integer();
                    cortex_m::asm::delay(delay);
                }

                // can only be modified when enabled
                let max_duty: u16 = self.get_max_duty();
                self.as_mut_tim().set_cmp(min(max_duty, duty));
            }
        }
    };
}

impl_eh_pwmpin_for!(LpTim1);
impl_eh_pwmpin_for!(LpTim2);
impl_eh_pwmpin_for!(LpTim3);

/// Low-power timer 1 trigger pin.
///
/// Constructed with [`new_trigger_pin`](LpTim1::new_trigger_pin).
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LpTim1Trg<P> {
    pin: P,
}

impl<P> LpTim1Trg<P> {
    fn free(self) -> P {
        self.pin
    }
}

impl LpTim1 {
    /// Setup a new pin trigger.
    ///
    /// # Panics
    ///
    /// * (debug) timer is enabled.
    ///
    /// # Example
    ///
    /// See [`LpTim3::new_trigger_pin`].
    pub fn new_trigger_pin<P: LpTim1Etr>(
        &mut self,
        filter: Filter,
        pol: TrgPol,
        mut pin: P,
        cs: &CriticalSection,
    ) -> LpTim1Trg<P> {
        debug_assert!(!self.is_enabled());
        pin.set_lptim1_etr_af(cs);
        self.as_mut_tim()
            .modify_cfgr(|w| w.set_trg_sel(0).set_trg_pol(pol).set_trg_filter(filter));
        LpTim1Trg { pin }
    }

    /// Free the trigger pin previously created with
    /// [`new_trigger_pin`](Self::new_trigger_pin).
    ///
    /// This is will the trigger source to a software trigger.
    ///
    /// # Panics
    ///
    /// * (debug) timer is enabled.
    ///
    /// # Example
    ///
    /// See [`LpTim3::free_trigger_pin`].
    pub fn free_trigger_pin<P: LpTim1Etr>(&mut self, pin: LpTim1Trg<P>) -> P {
        debug_assert!(!self.is_enabled());
        self.as_mut_tim()
            .modify_cfgr(|w| w.set_trg_pol(TrgPol::Soft));
        pin.free()
    }
}

/// Low-power timer 2 trigger pin.
///
/// Constructed with [`new_trigger_pin`](LpTim2::new_trigger_pin).
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LpTim2Trg<P> {
    pin: P,
}

impl<P> LpTim2Trg<P> {
    fn free(self) -> P {
        self.pin
    }
}

impl LpTim2 {
    /// Setup a new pin trigger.
    ///
    /// # Panics
    ///
    /// * (debug) timer is enabled.
    ///
    /// # Example
    ///
    /// See [`LpTim3::new_trigger_pin`].
    pub fn new_trigger_pin<P: LpTim2Etr>(
        &mut self,
        filter: Filter,
        pol: TrgPol,
        mut pin: P,
        cs: &CriticalSection,
    ) -> LpTim2Trg<P> {
        debug_assert!(!self.is_enabled());
        pin.set_lptim2_etr_af(cs);
        self.as_mut_tim()
            .modify_cfgr(|w| w.set_trg_sel(0).set_trg_pol(pol).set_trg_filter(filter));
        LpTim2Trg { pin }
    }

    /// Free the trigger pin previously created with
    /// [`new_trigger_pin`](Self::new_trigger_pin).
    ///
    /// This is will the trigger source to a software trigger.
    ///
    /// # Panics
    ///
    /// * (debug) timer is enabled.
    ///
    /// # Example
    ///
    /// See [`LpTim3::free_trigger_pin`].
    pub fn free_trigger_pin<P: LpTim2Etr>(&mut self, pin: LpTim2Trg<P>) -> P {
        debug_assert!(!self.is_enabled());
        self.as_mut_tim()
            .modify_cfgr(|w| w.set_trg_pol(TrgPol::Soft));
        pin.free()
    }
}

/// Low-power timer 3 trigger pin.
///
/// Constructed with [`new_trigger_pin`](LpTim3::new_trigger_pin).
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LpTim3Trg {
    pin: pins::A11,
}

impl LpTim3Trg {
    fn free(self) -> pins::A11 {
        self.pin
    }
}

impl LpTim3 {
    /// Setup a new pin trigger.
    ///
    /// # Panics
    ///
    /// * (debug) timer is enabled.
    ///
    /// # Example
    ///
    /// Setup a one-shot timer that starts after a transition on pin
    /// [`A11`](crate::gpio::pins::A11).
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     cortex_m,
    ///     embedded_hal::timer::CountDown,
    ///     gpio::PortA,
    ///     lptim::{self, Filter, LpTim, LpTim3, LpTim3Trg, Prescaler::Div1, TrgPol},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let mut lptim3: LpTim3 = LpTim3::new(dp.LPTIM3, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
    /// let lptim3trg: LpTim3Trg = cortex_m::interrupt::free(|cs| {
    ///     lptim3.new_trigger_pin(pa.a11, Filter::Any, TrgPol::Both, cs)
    /// });
    /// lptim3.start(12_345_u16);
    /// // timer will only start after any transition on pin A11
    /// nb::block!(lptim3.wait());
    /// ```
    pub fn new_trigger_pin(
        &mut self,
        mut pin: pins::A11,
        filter: Filter,
        pol: TrgPol,
        cs: &CriticalSection,
    ) -> LpTim3Trg {
        debug_assert!(!self.is_enabled());
        pin.set_lptim3_etr_af(cs);
        self.as_mut_tim()
            .modify_cfgr(|w| w.set_trg_sel(0).set_trg_pol(pol).set_trg_filter(filter));
        LpTim3Trg { pin }
    }

    /// Free the trigger pin previously created with
    /// [`new_trigger_pin`](Self::new_trigger_pin).
    ///
    /// This is will the trigger source to a software trigger.
    ///
    /// # Panics
    ///
    /// * (debug) timer is enabled.
    ///
    /// # Example
    ///
    /// Setup a one-shot timer that starts after a transition on pin A11.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     embedded_hal::timer::CountDown,
    ///     gpio::PortA,
    ///     lptim::{self, Filter, LpTim, LpTim3, LpTim3Trg, Prescaler::Div1, TrgPol},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let mut lptim3: LpTim3 = LpTim3::new(dp.LPTIM3, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
    /// let lptim3trg: LpTim3Trg = cortex_m::interrupt::free(|cs| {
    ///     lptim3.new_trigger_pin(pa.a11, Filter::Any, TrgPol::Both, cs)
    /// });
    /// lptim3.start(12_345_u16);
    /// // timer will only start after any transition on pin A11
    /// nb::block!(lptim3.wait());
    ///
    /// // free the pin
    /// let a11 = lptim3.free_trigger_pin(lptim3trg);
    /// ```
    pub fn free_trigger_pin(&mut self, pin: LpTim3Trg) -> pins::A11 {
        debug_assert!(!self.is_enabled());
        self.as_mut_tim()
            .modify_cfgr(|w| w.set_trg_pol(TrgPol::Soft));
        pin.free()
    }
}

/// Low-power timer 1 output pin.
///
/// Constructed with [`new_output_pin`](LpTim1::new_output_pin).
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LpTim1OutPin<P> {
    pin: P,
}

impl<P> LpTim1OutPin<P> {
    fn free(self) -> P {
        self.pin
    }
}

impl LpTim1 {
    /// Setup a new output pin.
    #[inline]
    pub fn new_output_pin<P: LpTim1Out>(
        &mut self,
        mut pin: P,
        cs: &CriticalSection,
    ) -> LpTim1OutPin<P> {
        pin.set_lptim1_out_af(cs);
        LpTim1OutPin { pin }
    }

    /// Free the output pin previously created with
    /// [`new_output_pin`](Self::new_output_pin).
    #[inline]
    pub fn free_output_pin<P: LpTim1Out>(&mut self, pin: LpTim1OutPin<P>) -> P {
        pin.free()
    }
}

/// Low-power timer 2 output pin.
///
/// Constructed with [`new_output_pin`](LpTim2::new_output_pin).
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LpTim2OutPin<P> {
    pin: P,
}

impl<P> LpTim2OutPin<P> {
    fn free(self) -> P {
        self.pin
    }
}

impl LpTim2 {
    /// Setup a new output pin.
    #[inline]
    pub fn new_output_pin<P: LpTim2Out>(
        &mut self,
        mut pin: P,
        cs: &CriticalSection,
    ) -> LpTim2OutPin<P> {
        pin.set_lptim2_out_af(cs);
        LpTim2OutPin { pin }
    }

    /// Free the output pin previously created with
    /// [`new_output_pin`](Self::new_output_pin).
    #[inline]
    pub fn free_output_pin<P: LpTim2Out>(&mut self, pin: LpTim2OutPin<P>) -> P {
        pin.free()
    }
}

/// Low-power timer 3 output pin.
///
/// Constructed with [`new_output_pin`](LpTim3::new_output_pin).
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LpTim3OutPin {
    pin: pins::A1,
}

impl LpTim3OutPin {
    fn free(self) -> pins::A1 {
        self.pin
    }
}

impl LpTim3 {
    /// Setup a new output pin.
    #[inline]
    pub fn new_output_pin(&mut self, mut pin: pins::A1, cs: &CriticalSection) -> LpTim3OutPin {
        pin.set_lptim3_out_af(cs);
        LpTim3OutPin { pin }
    }

    /// Free the output pin previously created with
    /// [`new_output_pin`](Self::new_output_pin).
    #[inline]
    pub fn free_output_pin(&mut self, pin: LpTim3OutPin) -> pins::A1 {
        pin.free()
    }
}
