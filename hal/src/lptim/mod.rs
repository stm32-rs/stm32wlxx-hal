//! Low-power timers
//!
//! Unlike other modules all functionality is exposed via a single trait shared
//! for all timers, [`LpTim`].
//!
//! # Example
//!
//! Setup a one-shot timer.
//!
//! ```no_run
//! use stm32wl_hal::{
//!     embedded_hal::timer::CountDown,
//!     lptim::{self, LpTim, LpTim1, Prescaler::Div1},
//!     pac,
//! };
//!
//! let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
//!
//! // enable the HSI16 source clock
//! dp.RCC.cr.write(|w| w.hsion().set_bit());
//! while dp.RCC.cr.read().hsirdy().is_not_ready() {}
//!
//! let mut lptim1: LpTim1 = LpTim1::new(dp.LPTIM1, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
//! // wait 16,000 cycles at 16MHz
//! lptim1.start(16_000_u16);
//! nb::block!(lptim1.wait());
//! ```
#![warn(missing_docs)]

mod cfgr;
mod cr;

pub use cfgr::{Cfgr, Filter, Prescaler, TrgPol, TrgSel, TrgSel3};
pub use cr::Cr;

use crate::{pac, Ratio};
use paste::paste;
use void::Void;

/// Timer IRQs.
pub mod irq {
    /// Repetition register update OK.
    pub const REPOK: u32 = 1 << 8;
    /// LPTIM update event occured.
    pub const UE: u32 = 1 << 7;
    /// Counter direction change up to down.
    pub const DOWN: u32 = 1 << 6;
    /// Counter direction change down to up.
    pub const UP: u32 = 1 << 5;
    /// Autoreload register update OK.
    pub const ARROK: u32 = 1 << 4;
    /// Compare register update OK.
    pub const CMPOK: u32 = 1 << 3;
    /// Extern trigger edge event.
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
    use super::{Cfgr, Cr};

    pub trait LpTimBase {
        fn isr() -> u32;
        fn set_icr(&mut self, icr: u32);
        fn set_ier(&mut self, ier: u32);
        fn cfgr(&self) -> Cfgr;
        fn set_cfgr(&mut self, cfgr: Cfgr);
        fn modify_cfgr<F: FnOnce(Cfgr) -> Cfgr>(&mut self, f: F);
        fn cr(&self) -> Cr;
        fn set_cr(&mut self, cr: Cr);
        fn set_cmp(&mut self, cmp: u16);
        fn set_autoreload(&mut self, ar: u16);
        unsafe fn cnt() -> u16;
        fn set_or(&mut self, or: u32);
        fn set_rep(&mut self, rep: u8);
    }

    pub trait LpTim {
        type Pac: LpTimBase;
        fn as_tim(&self) -> &Self::Pac;
        fn as_mut_tim(&mut self) -> &mut Self::Pac;
    }
}

macro_rules! impl_lptim_base_for {
    ($lptim:ident) => {
        impl LpTimBase for pac::$lptim {
            #[inline(always)]
            fn isr() -> u32 {
                // safety: atomic read with no side effects
                unsafe { (*pac::$lptim::ptr()).isr.read().bits() }
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
            fn set_cmp(&mut self, cmp: u16) {
                self.cmp.write(|w| w.cmp().bits(cmp));
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
                (*pac::$lptim::ptr()).cnt.read().cnt().bits()
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
                tim: pac::[<LPTIM $n>]
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
            }
        }
    };
}

paste_lptim!(1);
paste_lptim!(2);
paste_lptim!(3);

/// Low-power timer trait.
pub trait LpTim: sealed::LpTim {
    /// Tigger selection options.
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
    /// use stm32wl_hal::{
    ///     lptim::{self, LpTim, LpTim1, Prescaler::Div1},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.write(|w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let lptim1: LpTim1 = LpTim1::new(dp.LPTIM1, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
    /// ```
    fn new(tim: Self::Pac, clk: Clk, div: Prescaler, rcc: &mut pac::RCC) -> Self;

    /// Free the LPTIM registers from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     lptim::{self, LpTim, LpTim1, Prescaler::Div1},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.write(|w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let lptim1: LpTim1 = LpTim1::new(dp.LPTIM1, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
    /// // ... use lptim
    /// let lptim1: pac::LPTIM1 = lptim1.free();
    /// ```
    fn free(self) -> Self::Pac;

    /// Steal the timer peripheral from whatever is currently using it.
    ///
    /// This will **not** initialize the timer peripheral (unlike [`new`]).
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the timer has exclusive access to the
    ///    peripheral. Singleton checks are bypassed with this method.
    /// 2. You are responsible for setting up the timer correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::lptim::{LpTim, LpTim1};
    ///
    /// // ... setup happens here
    ///
    /// let lptim1 = unsafe { LpTim1::steal() };
    /// ```
    ///
    /// [`new`]: Self::new
    unsafe fn steal() -> Self;

    /// Reset the LPTIM peripheral.
    ///
    /// [`new`](Self::new) will pulse reset for you.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the timer before pulsing reset.
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
    /// use stm32wl_hal::{
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
    /// use stm32wl_hal::{
    ///     lptim::{self, LpTim, LpTim1, Prescaler::Div1},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.write(|w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let lptim1: LpTim1 = LpTim1::new(dp.LPTIM1, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
    /// assert_eq!(lptim1.hz(&dp.RCC).to_integer(), 16_000_000);
    /// ```
    fn hz(&self, rcc: &pac::RCC) -> Ratio<u32> {
        let src: Ratio<u32> = match Self::clk(rcc) {
            Clk::Pclk => crate::rcc::apb1timx(rcc),
            Clk::Lsi => Ratio::new_raw(crate::rcc::lsi_hz(rcc).into(), 1),
            Clk::Hsi16 => Ratio::new_raw(16_000_000, 1),
            Clk::Lse => Ratio::new_raw(32_768, 1),
        };
        src / self.as_tim().cfgr().prescaler().div().into()
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
    #[inline]
    fn set_ier(&mut self, ier: u32) {
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
    fn is_enabled(&self) -> bool {
        self.as_tim().cr().enabled()
    }

    /// Setup the trigger.
    ///
    /// This relies upon global state for GPIO triggers.
    /// The design choice was a complex API that was impossible to misuse or a
    /// simple API that could be misused without undefined behaviour; I picked
    /// the latter.
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
    /// use stm32wl_hal::{
    ///     embedded_hal::timer::CountDown,
    ///     gpio::{LpTim3Trg, PortA},
    ///     lptim::{self, Filter, LpTim, LpTim3, Prescaler::Div1, TrgPol, TrgSel3},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.write(|w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// // A11 must be configured as a trigger pin
    /// // otherwise the timer will never start
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let _: LpTim3Trg = LpTim3Trg::new(gpioa.pa11);
    ///
    /// let mut lptim3: LpTim3 = LpTim3::new(dp.LPTIM3, lptim::Clk::Hsi16, Div1, &mut dp.RCC);
    /// lptim3.setup_trigger(Filter::Any, TrgPol::Both, TrgSel3::Pin);
    /// lptim3.start(12_345_u16);
    /// // timer will only start after any transition on pin A11
    /// nb::block!(lptim3.wait());
    /// ```
    fn setup_trigger(&mut self, filter: Filter, pol: TrgPol, sel: Self::TrgSel) {
        debug_assert!(!self.is_enabled());
        self.as_mut_tim().modify_cfgr(|w| {
            w.set_trg_sel(sel.into())
                .set_trg_pol(pol)
                .set_trg_filter(filter)
        })
    }
}

impl LpTim for LpTim1 {
    type TrgSel = TrgSel;

    #[inline]
    fn new(mut tim: Self::Pac, clk: Clk, div: Prescaler, rcc: &mut pac::RCC) -> Self {
        rcc.ccipr.modify(|_, w| w.lptim1sel().bits(clk as u8));
        unsafe { Self::pulse_reset(rcc) }
        Self::enable_clock(rcc);
        tim.set_cfgr(Cfgr::RESET.set_prescaler(div));
        Self { tim }
    }

    #[inline]
    unsafe fn steal() -> Self {
        Self {
            tim: pac::Peripherals::steal().LPTIM1,
        }
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

    #[inline]
    fn new(mut tim: Self::Pac, clk: Clk, div: Prescaler, rcc: &mut pac::RCC) -> Self {
        rcc.ccipr.modify(|_, w| w.lptim2sel().bits(clk as u8));
        unsafe { Self::pulse_reset(rcc) }
        Self::enable_clock(rcc);
        tim.set_cfgr(Cfgr::RESET.set_prescaler(div));
        Self { tim }
    }

    #[inline]
    unsafe fn steal() -> Self {
        Self {
            tim: pac::Peripherals::steal().LPTIM2,
        }
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

    #[inline]
    fn new(mut tim: Self::Pac, clk: Clk, div: Prescaler, rcc: &mut pac::RCC) -> Self {
        rcc.ccipr.modify(|_, w| w.lptim3sel().bits(clk as u8));
        unsafe { Self::pulse_reset(rcc) }
        Self::enable_clock(rcc);
        tim.set_cfgr(Cfgr::RESET.set_prescaler(div));
        Self { tim }
    }

    #[inline]
    unsafe fn steal() -> Self {
        Self {
            tim: pac::Peripherals::steal().LPTIM3,
        }
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

macro_rules! impl_eh_for {
    ($tim:ident) => {
        impl embedded_hal::timer::CountDown for $tim {
            type Time = u16;

            fn start<T>(&mut self, count: T)
            where
                T: Into<Self::Time>,
            {
                // disable TIM
                self.as_mut_tim().set_cr(Cr::DISABLE);
                // COUNTRST must never be set to `1` by software before it is already
                // cleared to `0` by hardware.
                // Software should consequently check that COUNTRST bit is already
                // cleared to `0` before attempting to set it to `1`.
                while self.as_tim().cr().cnt_rst() {}

                // reset counter
                {
                    const CR: Cr = Cr::RESET.enable().set_cnt_rst();
                    self.as_mut_tim().set_cr(CR);
                }

                // can only be modified when enabled
                self.as_mut_tim().set_cmp(count.into());
                self.as_mut_tim().set_autoreload(0);

                {
                    const CR: Cr = Cr::RESET.enable().set_single();
                    self.as_mut_tim().set_cr(CR);
                }
            }

            fn wait(&mut self) -> nb::Result<(), Void> {
                if Self::isr() & irq::CMPM == 0 {
                    Err(nb::Error::WouldBlock)
                } else {
                    Ok(())
                }
            }
        }
    };
}

impl_eh_for!(LpTim1);
impl_eh_for!(LpTim2);
impl_eh_for!(LpTim3);
