//! Analog to digital converter
//!
//! Quickstart:
//!
//! * [`Adc::pin`] Sample an analog pin
//! * [`Adc::temperature`] Sample the junction temperature
//! * [`Adc::vbat`] Sample the battery voltage
#![cfg_attr(feature = "stm32wl5x_cm0p", allow(rustdoc::broken_intra_doc_links))]
#![cfg_attr(feature = "stm32wl5x_cm0p", allow(dead_code))]
#![cfg_attr(feature = "stm32wl5x_cm0p", allow(unused_imports))]

#[cfg(not(feature = "stm32wl5x_cm0p"))]
pub use pac::adc::cfgr2::{OVSR_A as OversampleRatio, OVSS_A as OversampleShift};

use crate::gpio;
use crate::Ratio;

use super::pac;
use core::{ptr::read_volatile, time::Duration};

use embedded_hal::blocking::delay::DelayUs;

// DS13293 rev 1 table 12
// TS ADC raw data acquired at 30 °C (± 5 °C),
// VDDA = VREF+ = 3.3 V (± 10 mV)
fn ts_cal1() -> u16 {
    unsafe { read_volatile(0x1FFF_75A8 as *const u16) }
}

// DS13293 rev 1 table 12
// TS ADC raw data acquired at 130 °C (± 5 °C),
// VDDA = VREF+ = 3.3 V (± 10 mV)
fn ts_cal2() -> u16 {
    unsafe { read_volatile(0x1FFF_75C8 as *const u16) }
}

fn ts_cal() -> (u16, u16) {
    (ts_cal1(), ts_cal2())
}

const TS_CAL1_TEMP: i16 = 30;
const TS_CAL2_TEMP: i16 = 130;
const TS_CAL_TEMP_DELTA: i16 = TS_CAL2_TEMP - TS_CAL1_TEMP;

/// t<sub>S_temp</sub> temperature sensor minimum sampling time
///
/// Value from DS13293 Rev 1 page 121 table 83 "TS characteristics"
pub const TS_MIN_SAMPLE: Duration = Duration::from_micros(5);
/// t<sub>START</sub> temperature sensor typical startup time when entering
/// continuous mode
///
/// Value from DS13293 Rev 1 page 121 table 83 "TS characteristics"
pub const TS_START_TYP: Duration = Duration::from_micros(70);
/// t<sub>START</sub> temperature sensor maximum startup time when entering
/// continuous mode
///
/// Value from DS13293 Rev 1 page 121 table 83 "TS characteristics"
pub const TS_START_MAX: Duration = Duration::from_micros(120);

/// t<sub>ADCVREG_SETUP</sub> ADC voltage regulator maximum startup time
///
/// Value from DS13293 Rev 1 page 117 table 80 "ADC characteristics"
pub const T_ADCVREG_SETUP: Duration = Duration::from_micros(20);

/// [`T_ADCVREG_SETUP`] expressed in microseconds
///
/// # Example
///
/// ```
/// use stm32wlxx_hal::adc::{T_ADCVREG_SETUP, T_ADCVREG_SETUP_MICROS};
///
/// assert_eq!(
///     u128::from(T_ADCVREG_SETUP_MICROS),
///     T_ADCVREG_SETUP.as_micros()
/// );
/// ```
pub const T_ADCVREG_SETUP_MICROS: u8 = T_ADCVREG_SETUP.as_micros() as u8;

/// Mask of all valid channels
///
/// Channels 0-17, but without 15 and 16 because they are reserved.
const CH_MASK: u32 = 0x27FFF;

/// Interrupt masks
///
/// Used for [`Adc::set_isr`] and [`Adc::set_ier`].
pub mod irq {
    /// Channel configuration ready
    pub const CCRDY: u32 = 1 << 13;
    /// End of calibration
    pub const EOCAL: u32 = 1 << 11;
    /// Analog watchdog 3
    pub const AWD3: u32 = 1 << 9;
    /// Analog watchdog 2
    pub const AWD2: u32 = 1 << 8;
    /// Analog watchdog 1
    pub const AWD1: u32 = 1 << 7;
    /// Overrun
    pub const OVR: u32 = 1 << 4;
    /// End of conversion sequence
    pub const EOS: u32 = 1 << 3;
    /// End of conversion
    pub const EOC: u32 = 1 << 2;
    /// End of sampling
    pub const EOSMP: u32 = 1 << 1;
    /// ADC ready
    pub const ADRDY: u32 = 1;

    /// All IRQs
    pub const ALL: u32 = CCRDY | EOCAL | AWD3 | AWD2 | AWD1 | OVR | EOS | EOC | EOSMP | ADRDY;
}

/// Internal voltage reference ADC calibration
///
/// This is raw ADC data acquired at 30 °C (± 5 °C).
///
/// V<sub>DDA</sub> = V<sub>REF+</sub> = 3.3 V (± 10mV)
pub fn vref_cal() -> u16 {
    // DS13293 rev 1 table 13
    unsafe { read_volatile(0x1FFF_75AA as *const u16) }
}

/// ADC clock mode
///
/// In all synchronous clock modes, there is no jitter in the delay from a
/// timer trigger to the start of a conversion.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Clk {
    /// Asynchronous clock mode HSI16
    RccHsi,
    /// Asynchronous clock mode PLLP
    RccPllP,
    /// Asynchronous clock mode SYSCLK
    RccSysClk,
    /// Synchronous clock mode, pclk/2
    PClkDiv2,
    /// Synchronous clock mode, pclk/4
    PClkDiv4,
    /// Synchronous clock mode, pclk
    ///
    /// This configuration must be enabled only if PCLK has a 50% duty clock
    /// cycle (APB prescaler configured inside the RCC must be bypassed and
    /// the system clock must by 50% duty cycle)
    PClk,
}

#[cfg(not(feature = "stm32wl5x_cm0p"))]
impl Clk {
    const fn ckmode(&self) -> pac::adc::cfgr2::CKMODE_A {
        match self {
            Clk::RccHsi | Clk::RccPllP | Clk::RccSysClk => pac::adc::cfgr2::CKMODE_A::Adclk,
            Clk::PClkDiv2 => pac::adc::cfgr2::CKMODE_A::PclkDiv2,
            Clk::PClkDiv4 => pac::adc::cfgr2::CKMODE_A::PclkDiv4,
            Clk::PClk => pac::adc::cfgr2::CKMODE_A::Pclk,
        }
    }

    const fn adcsel(&self) -> pac::rcc::ccipr::ADCSEL_A {
        match self {
            Clk::RccHsi => pac::rcc::ccipr::ADCSEL_A::Hsi16,
            Clk::RccPllP => pac::rcc::ccipr::ADCSEL_A::Pllp,
            Clk::RccSysClk => pac::rcc::ccipr::ADCSEL_A::Sysclk,
            _ => pac::rcc::ccipr::ADCSEL_A::NoClock,
        }
    }
}

/// ADC sample times
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Ts {
    /// 1.5 ADC clock cycles
    Cyc1 = 0,
    /// 3.5 ADC clock cycles
    Cyc3 = 1,
    /// 7.5 ADC clock cycles
    Cyc7 = 2,
    /// 12.5 ADC clock cycles
    Cyc12 = 3,
    /// 19.5 ADC clock cycles
    Cyc19 = 4,
    /// 39.5 ADC clock cycles
    Cyc39 = 5,
    /// 79.5 ADC clock cycles
    Cyc79 = 6,
    /// 160.5 ADC clock cycles
    Cyc160 = 7,
}

impl Default for Ts {
    /// Reset value of the sample time.
    fn default() -> Self {
        Ts::Cyc1
    }
}

impl Ts {
    /// Maximum sample time, 160.5 ADC clock cycles.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::adc::Ts;
    ///
    /// assert_eq!(Ts::MAX, Ts::Cyc160);
    /// ```
    pub const MAX: Self = Self::Cyc160;

    /// Minimum sample time, 1.5 ADC clock cycles.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::adc::Ts;
    ///
    /// assert_eq!(Ts::MIN, Ts::Cyc1);
    /// ```
    pub const MIN: Self = Self::Cyc1;

    /// Number of cycles.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::adc::Ts;
    ///
    /// assert!(f32::from(Ts::Cyc1.cycles()) - 1.5 < 0.001);
    /// assert!(f32::from(Ts::Cyc3.cycles()) - 3.5 < 0.001);
    /// assert!(f32::from(Ts::Cyc7.cycles()) - 7.5 < 0.001);
    /// assert!(f32::from(Ts::Cyc12.cycles()) - 12.5 < 0.001);
    /// assert!(f32::from(Ts::Cyc19.cycles()) - 19.5 < 0.001);
    /// assert!(f32::from(Ts::Cyc39.cycles()) - 39.5 < 0.001);
    /// assert!(f32::from(Ts::Cyc79.cycles()) - 79.5 < 0.001);
    /// assert!(f32::from(Ts::Cyc160.cycles()) - 160.5 < 0.001);
    /// ```
    pub const fn cycles(&self) -> Ratio<u16> {
        match self {
            Ts::Cyc1 => Ratio::new_raw(3, 2),
            Ts::Cyc3 => Ratio::new_raw(7, 2),
            Ts::Cyc7 => Ratio::new_raw(15, 2),
            Ts::Cyc12 => Ratio::new_raw(25, 2),
            Ts::Cyc19 => Ratio::new_raw(39, 2),
            Ts::Cyc39 => Ratio::new_raw(79, 2),
            Ts::Cyc79 => Ratio::new_raw(159, 2),
            Ts::Cyc160 => Ratio::new_raw(321, 2),
        }
    }

    /// Get the cycles as a duration.
    ///
    /// Fractional nano-seconds are rounded towards zero.
    ///
    /// You can get the ADC frequency with [`Adc::clock_hz`].
    ///
    /// # Example
    ///
    /// Assuming the ADC clock frequency is 16 MHz.
    ///
    /// ```
    /// use core::time::Duration;
    /// use stm32wlxx_hal::adc::Ts;
    ///
    /// const FREQ: u32 = 16_000_000;
    ///
    /// assert_eq!(Ts::Cyc1.as_duration(FREQ), Duration::from_nanos(93));
    /// assert_eq!(Ts::Cyc3.as_duration(FREQ), Duration::from_nanos(218));
    /// assert_eq!(Ts::Cyc7.as_duration(FREQ), Duration::from_nanos(468));
    /// assert_eq!(Ts::Cyc12.as_duration(FREQ), Duration::from_nanos(781));
    /// assert_eq!(Ts::Cyc19.as_duration(FREQ), Duration::from_nanos(1_218));
    /// assert_eq!(Ts::Cyc39.as_duration(FREQ), Duration::from_nanos(2_468));
    /// assert_eq!(Ts::Cyc79.as_duration(FREQ), Duration::from_nanos(4_968));
    /// assert_eq!(Ts::Cyc160.as_duration(FREQ), Duration::from_nanos(10_031));
    /// ```
    ///
    /// [`Adc::clock_hz`]: crate::adc::Adc::clock_hz
    pub const fn as_duration(&self, hz: u32) -> Duration {
        let numer: u64 = (*self.cycles().numer() as u64).saturating_mul(1_000_000_000);
        let denom: u64 = (*self.cycles().denom() as u64).saturating_mul(hz as u64);
        Duration::from_nanos(numer / denom)
    }
}

impl From<Ts> for u8 {
    fn from(ts: Ts) -> Self {
        ts as u8
    }
}

impl From<Ts> for u32 {
    fn from(ts: Ts) -> Self {
        ts as u32
    }
}

/// ADC channels
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[repr(u8)]
pub enum Ch {
    /// ADC input 0.
    ///
    /// Connected to [`B13`](crate::gpio::pins::B13).
    In0 = 0,
    /// ADC input 1.
    ///
    /// Connected to [`B14`](crate::gpio::pins::B14).
    In1 = 1,
    /// ADC input 2.
    ///
    /// Connected to [`B3`](crate::gpio::pins::B3).
    In2 = 2,
    /// ADC input 3.
    ///
    /// Connected to [`B4`](crate::gpio::pins::B4).
    In3 = 3,
    /// ADC input 4.
    ///
    /// Connected to [`B2`](crate::gpio::pins::B2).
    In4 = 4,
    /// ADC input 5.
    ///
    /// Connected to [`B1`](crate::gpio::pins::B1).
    In5 = 5,
    /// ADC input 6.
    ///
    /// Connected to [`A10`](crate::gpio::pins::A10).
    In6 = 6,
    /// ADC input 7.
    ///
    /// Connected to [`A11`](crate::gpio::pins::A11).
    In7 = 7,
    /// ADC input 8.
    ///
    /// Connected to [`A12`](crate::gpio::pins::A12).
    In8 = 8,
    /// ADC input 9.
    ///
    /// Connected to [`A13`](crate::gpio::pins::A13).
    In9 = 9,
    /// ADC input 10.
    ///
    /// Connected to [`A14`](crate::gpio::pins::A14).
    In10 = 10,
    /// ADC input 11.
    ///
    /// Connected to [`A15`](crate::gpio::pins::A15).
    In11 = 11,
    /// Junction temperature sensor.
    Vts = 12,
    /// Internal voltage reference.
    Vref = 13,
    /// Battery voltage divided by 3.
    Vbat = 14,
    // 15, 16 are reserved
    /// Digital to analog converter output.
    ///
    /// The DAC outputs to this internal pin only when configured to output to
    /// chip peripherals.
    Dac = 17,
}

impl Ch {
    /// Bitmask of the channel.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::adc::Ch;
    ///
    /// assert_eq!(Ch::In0.mask(), 0x001);
    /// assert_eq!(Ch::In8.mask(), 0x100);
    /// ```
    pub const fn mask(self) -> u32 {
        1 << (self as u8)
    }
}

/// Analog to digital converter driver.
#[derive(Debug)]
#[cfg(not(feature = "stm32wl5x_cm0p"))]
pub struct Adc {
    adc: pac::ADC,
}

#[cfg(not(feature = "stm32wl5x_cm0p"))]
impl Adc {
    /// Create a new ADC driver from a ADC peripheral.
    ///
    /// This will enable the ADC clock and reset the ADC peripheral.
    ///
    /// **Note:** This will select the clock source, but you are responsible
    /// for enabling that clock source.
    ///
    /// # Example
    ///
    /// Initialize the ADC with HSI16.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// ```
    ///
    /// Initialize the ADC with PCLK/4.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// ```
    #[inline]
    pub fn new(adc: pac::ADC, clk: Clk, rcc: &mut pac::RCC) -> Self {
        unsafe { Self::pulse_reset(rcc) };
        Self::enable_clock(rcc);
        let mut adc: Self = Self { adc };
        adc.set_clock_source(clk, rcc);
        adc
    }

    /// Create a new ADC driver from an ADC peripheral without initialization.
    ///
    /// This is a slightly safer version of [`steal`](Self::steal).
    ///
    /// # Safety
    ///
    /// 1. Reset the ADC peripheral if determinism is required.
    /// 2. Enable the ADC peripheral clock before using the ADC.
    /// 3. Select the clock source if a non-default clock is required.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // safety: nothing is using the peripheral
    /// unsafe { Adc::pulse_reset(&mut dp.RCC) };
    ///
    /// Adc::enable_clock(&mut dp.RCC);
    ///
    /// // safety: ADC peripheral has been reset and clocks are enabled
    /// let mut adc: Adc = unsafe { Adc::new_no_init(dp.ADC) };
    ///
    /// // select the ADC clock, optional
    /// adc.set_clock_source(adc::Clk::PClkDiv4, &mut dp.RCC);
    /// ```
    #[inline]
    pub const unsafe fn new_no_init(adc: pac::ADC) -> Self {
        Self { adc }
    }

    /// Free the ADC peripheral from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// // ... use ADC
    /// let adc: pac::ADC = adc.free();
    /// ```
    #[inline]
    pub fn free(self) -> pac::ADC {
        self.adc
    }

    /// Steal the ADC peripheral from whatever is currently using it.
    ///
    /// This will **not** initialize the ADC (unlike [`new`]).
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the ADC has exclusive access to the
    ///    peripheral. Singleton checks are bypassed with this method.
    /// 2. Reset the ADC peripheral if determinism is required.
    /// 3. Enable the ADC peripheral clock before using the ADC.
    /// 4. Select the clock source if a non-default clock is required.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let _: pac::ADC = dp.ADC;
    ///
    /// unsafe { Adc::pulse_reset(&mut dp.RCC) };
    ///
    /// Adc::enable_clock(&mut dp.RCC);
    ///
    /// let mut adc: Adc = unsafe { Adc::steal() };
    ///
    /// // select the ADC clock, optional
    /// adc.set_clock_source(adc::Clk::PClkDiv4, &mut dp.RCC);
    /// ```
    ///
    /// [`new`]: Adc::new
    #[inline]
    pub unsafe fn steal() -> Adc {
        Adc {
            adc: pac::Peripherals::steal().ADC,
        }
    }

    /// Set the ADC clock source.
    ///
    /// # Panics
    ///
    /// * (debug) ADC is enabled
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc: Adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    ///
    /// // change the clock source
    /// adc.disable();
    /// adc.set_clock_source(adc::Clk::PClkDiv4, &mut dp.RCC);
    /// ```
    #[inline]
    pub fn set_clock_source(&mut self, clk: Clk, rcc: &mut pac::RCC) {
        debug_assert!(!self.is_enabled());
        self.adc
            .cfgr2
            .modify(|_, w| w.ckmode().variant(clk.ckmode()));
        rcc.ccipr.modify(|_, w| w.adcsel().variant(clk.adcsel()));
    }

    /// Get the ADC clock source.
    ///
    /// Returns `None` if the ADC is configured for an asynchronous clock,
    /// but no asynchronous clock is selected.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc: Adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    ///
    /// assert_eq!(adc.clock_source(&dp.RCC), Some(adc::Clk::PClkDiv4));
    /// ```
    pub fn clock_source(&self, rcc: &pac::RCC) -> Option<Clk> {
        use pac::{adc::cfgr2::CKMODE_A, rcc::ccipr::ADCSEL_A};

        match self.adc.cfgr2.read().ckmode().variant() {
            CKMODE_A::Adclk => match rcc.ccipr.read().adcsel().variant() {
                ADCSEL_A::NoClock => None,
                ADCSEL_A::Hsi16 => Some(Clk::RccHsi),
                ADCSEL_A::Pllp => Some(Clk::RccPllP),
                ADCSEL_A::Sysclk => Some(Clk::RccSysClk),
            },
            CKMODE_A::PclkDiv2 => Some(Clk::PClkDiv2),
            CKMODE_A::PclkDiv4 => Some(Clk::PClkDiv4),
            CKMODE_A::Pclk => Some(Clk::PClk),
        }
    }

    /// Disable the ADC clock.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the ADC before disabling the clock.
    /// 2. You are responsible for re-enabling the clock before using the ADC.
    #[inline]
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb2enr.modify(|_, w| w.adcen().disabled());
    }

    /// Enable the ADC clock.
    ///
    /// [`new`](crate::adc::Adc::new) will enable clocks for you.
    #[inline]
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb2enr.modify(|_, w| w.adcen().enabled());
        rcc.apb2enr.read(); // delay after an RCC peripheral clock enabling
    }

    /// Pulse the ADC reset.
    ///
    /// [`new`](crate::adc::Adc::new) will pulse reset for you.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the ADC before calling this function.
    /// 2. You are responsible for setting up the ADC after a reset.
    #[inline]
    pub unsafe fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb2rstr.modify(|_, w| w.adcrst().set_bit());
        rcc.apb2rstr.modify(|_, w| w.adcrst().clear_bit());
    }

    /// Calculate the ADC clock frequency in hertz.
    ///
    /// **Note:** If the ADC prescaler register erroneously returns a reserved
    /// value the code will default to an ADC prescaler of 1.
    ///
    /// Fractional frequencies will be rounded towards zero.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let adc: Adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// assert_eq!(adc.clock_hz(&dp.RCC), 16_000_000);
    /// ```
    pub fn clock_hz(&self, rcc: &pac::RCC) -> u32 {
        use pac::{
            adc::{ccr::PRESC_A, cfgr2::CKMODE_A},
            rcc::ccipr::ADCSEL_A,
        };

        let source_freq: Ratio<u32> = match self.adc.cfgr2.read().ckmode().variant() {
            CKMODE_A::Adclk => {
                let src: Ratio<u32> = match rcc.ccipr.read().adcsel().variant() {
                    ADCSEL_A::NoClock => Ratio::new_raw(0, 1),
                    ADCSEL_A::Hsi16 => Ratio::new_raw(16_000_000, 1),
                    ADCSEL_A::Pllp => crate::rcc::pllpclk(rcc, &rcc.pllcfgr.read()),
                    ADCSEL_A::Sysclk => crate::rcc::sysclk(rcc, &rcc.cfgr.read()),
                };

                // only the asynchronous clocks have the prescaler applied
                let ccr = self.adc.ccr.read();
                let prescaler: u32 = match ccr.presc().variant() {
                    Some(p) => match p {
                        PRESC_A::Div1 => 1,
                        PRESC_A::Div2 => 2,
                        PRESC_A::Div4 => 4,
                        PRESC_A::Div6 => 6,
                        PRESC_A::Div8 => 8,
                        PRESC_A::Div10 => 10,
                        PRESC_A::Div12 => 12,
                        PRESC_A::Div16 => 16,
                        PRESC_A::Div32 => 32,
                        PRESC_A::Div64 => 64,
                        PRESC_A::Div128 => 128,
                        PRESC_A::Div256 => 256,
                    },
                    None => {
                        error!("Reserved ADC prescaler value {:#X}", ccr.presc().bits());
                        1
                    }
                };

                src / prescaler
            }
            CKMODE_A::PclkDiv2 => crate::rcc::pclk2(rcc, &rcc.cfgr.read()) / 2,
            CKMODE_A::PclkDiv4 => crate::rcc::pclk2(rcc, &rcc.cfgr.read()) / 4,
            CKMODE_A::Pclk => crate::rcc::pclk2(rcc, &rcc.cfgr.read()),
        };

        source_freq.to_integer()
    }

    /// Unmask the ADC IRQ in the NVIC.
    ///
    /// # Safety
    ///
    /// This can break mask-based critical sections.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    /// unsafe { stm32wlxx_hal::adc::Adc::unmask_irq() };
    /// ```
    #[cfg(feature = "rt")]
    #[inline]
    pub unsafe fn unmask_irq() {
        pac::NVIC::unmask(pac::Interrupt::ADC)
    }

    /// Mask the ADC IRQ in the NVIC.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    /// unsafe { stm32wlxx_hal::adc::Adc::mask_irq() }
    /// ```
    #[cfg(feature = "rt")]
    #[inline]
    pub fn mask_irq() {
        pac::NVIC::mask(pac::Interrupt::ADC)
    }

    /// Set sample times for **all** channels.
    ///
    /// For each bit in the mask:
    ///
    /// * `0`: Sample time is set by the `sel0` argument.
    /// * `1`: Sample time is set by the `sel1` argument.
    ///
    /// # Panics
    ///
    /// * (debug) An ADC conversion is in-progress
    ///
    /// # Example
    ///
    /// Set ADC channels [`In0`] and [`In1`] (pins [`B13`] and [`B14`]
    /// respectively) and the internal V<sub>BAT</sub> to a sample time of
    /// 39.5 ADC clock cycles, and set all other channels to a sample time of
    /// 160.5 clock cycles.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     self as hal,
    ///     adc::{self, Adc, Ts},
    ///     gpio::pins::{B13, B14},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// adc.set_sample_times(
    ///     B14::ADC_CH.mask() | B13::ADC_CH.mask() | adc::Ch::Vbat.mask(),
    ///     Ts::Cyc160,
    ///     Ts::Cyc39,
    /// );
    /// ```
    ///
    /// [`In0`]: crate::adc::Ch::In0
    /// [`In1`]: crate::adc::Ch::In1
    /// [`B13`]: crate::gpio::pins::B13
    /// [`B14`]: crate::gpio::pins::B14
    #[inline]
    pub fn set_sample_times(&mut self, mask: u32, sel0: Ts, sel1: Ts) {
        debug_assert!(self.adc.cr.read().adstart().is_not_active());
        self.adc.smpr.write(|w| unsafe {
            w.bits((mask & CH_MASK) << 8 | u32::from(sel1) << 4 | u32::from(sel0))
        })
    }

    /// Sets all channels to the maximum sample time.
    ///
    /// This is a helper for testing and rapid prototyping purpose because
    /// [`set_sample_times`](Adc::set_sample_times) is verbose.
    ///
    /// This method is equivalent to this:
    ///
    /// ```no_run
    /// # let mut adc = unsafe { stm32wlxx_hal::adc::Adc::steal() };
    /// use stm32wlxx_hal::adc::Ts;
    ///
    /// adc.set_sample_times(0, Ts::Cyc160, Ts::Cyc160);
    /// ```
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// adc.set_max_sample_time();
    /// ```
    #[inline]
    pub fn set_max_sample_time(&mut self) {
        self.set_sample_times(0, Ts::Cyc160, Ts::Cyc160);
    }

    /// Clear interrupts.
    ///
    /// # Example
    ///
    /// Clear all interrupts.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// adc.set_isr(adc::irq::ALL);
    /// ```
    #[inline]
    pub fn set_isr(&mut self, isr: u32) {
        // saftey: reserved bits are masked and will be held at reset value
        self.adc.isr.write(|w| unsafe { w.bits(isr & irq::ALL) })
    }

    /// Read the interrupt status.
    ///
    /// # Example
    ///
    /// Check if the ADC is ready.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// // this will be false because the ADC is not enabled
    /// let ready: bool = Adc::isr().adrdy().is_ready();
    /// ```
    #[inline]
    pub fn isr() -> pac::adc::isr::R {
        // saftey: atomic read with no side-effects
        unsafe { (*pac::ADC::PTR).isr.read() }
    }

    /// Enable and disable interrupts.
    ///
    /// # Example
    ///
    /// Enable all IRQs
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// adc.set_ier(adc::irq::ALL);
    /// ```
    #[inline]
    pub fn set_ier(&mut self, ier: u32) {
        // saftey: reserved bits are masked and will be held at reset value
        self.adc.ier.write(|w| unsafe { w.bits(ier & irq::ALL) })
    }

    /// Configure the channel sequencer.
    ///
    /// This is advanced ADC usage, most of the time you will want to use a
    /// one of the available sample methods that will configure this.
    ///
    /// * [`pin`](Self::pin)
    /// * [`temperature`](Self::temperature)
    /// * [`vbat`](Self::vbat)
    ///
    /// This will not poll for completion, when this method returns the channel
    /// configuration may not be ready.
    ///
    /// # Panics
    ///
    /// * (debug) ADC conversion is in-progress.
    ///
    /// # Example
    ///
    /// Select the ADC V<sub>BAT</sub> channel.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// adc.start_chsel(adc::Ch::Vbat.mask());
    /// while Adc::isr().ccrdy().is_not_complete() {}
    /// ```
    #[inline]
    pub fn start_chsel(&mut self, ch: u32) {
        debug_assert!(self.adc.cr.read().adstart().is_not_active());
        // See section 18.3.8 page 542 "Channel selection"
        // saftey: reserved bits are masked and will be held at reset value
        self.adc
            .chselr0()
            .write(|w| unsafe { w.chsel().bits(ch & CH_MASK) });
    }

    #[inline]
    fn cfg_ch_seq(&mut self, ch: u32) {
        self.start_chsel(ch);
        while self.adc.isr.read().ccrdy().is_not_complete() {}
    }

    /// Start an ADC conversion.
    ///
    /// This is advanced ADC usage, most of the time you will want to use a
    /// one of the available sample methods that will configure this.
    ///
    /// * [`pin`](Self::pin)
    /// * [`temperature`](Self::temperature)
    /// * [`vbat`](Self::vbat)
    ///
    /// This will not poll for completion, when this method returns the AD
    /// conversion may not be complete.
    ///
    /// # Panics
    ///
    /// * (debug) ADC is not enabled
    /// * (debug) ADC has a pending disable request
    ///
    /// # Example
    ///
    /// See [`data`](Self::data).
    #[inline]
    pub fn start_conversion(&mut self) {
        debug_assert!(self.is_enabled());
        debug_assert!(self.adc.cr.read().addis().is_not_disabling());
        self.adc.cr.write(|w| w.adstart().start_conversion());
    }

    /// Stop an ADC conversion if there is one in-progress.
    pub fn stop_conversion(&mut self) {
        if self.adc.cr.read().adstart().is_active() {
            self.adc.cr.write(|w| w.adstp().stop_conversion());
            while self.adc.cr.read().adstp().bit_is_set() {}
        }
    }

    /// Read the ADC conversion data.
    ///
    /// This is advanced ADC usage, most of the time you will want to use a
    /// one of the available sample methods.
    ///
    /// * [`pin`](Self::pin)
    /// * [`temperature`](Self::temperature)
    /// * [`vbat`](Self::vbat)
    ///
    /// # Example
    ///
    /// Read the ADC V<sub>BAT</sub> channel.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    ///     util::new_delay,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay = new_delay(cp.SYST, &dp.RCC);
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    ///
    /// // calibrate the ADC before it is enabled
    /// adc.calibrate(&mut delay);
    ///
    /// // enable the ADC
    /// adc.enable();
    ///
    /// // set the sample times to the maximum (160.5 ADC cycles)
    /// adc.set_max_sample_time();
    ///
    /// // select the Vbat channel and poll for completion
    /// adc.start_chsel(adc::Ch::Vbat.mask());
    /// while Adc::isr().ccrdy().is_not_complete() {}
    ///
    /// // start the conversion and poll for completion
    /// adc.start_conversion();
    /// while Adc::isr().eoc().is_not_complete() {}
    ///
    /// // read the ADC data
    /// let vbat: u16 = adc.data();
    /// ```
    #[inline]
    pub fn data(&self) -> u16 {
        self.adc.dr.read().data().bits()
    }

    fn poll_data(&self) -> u16 {
        while self.adc.isr.read().eoc().is_not_complete() {}
        let data: u16 = self.data();
        self.adc.isr.write(|w| w.eoc().set_bit());
        data
    }

    /// Enable the temperature sensor.
    ///
    /// You **MUST** wait for the temperature sensor to start up
    /// ([`TS_START_TYP`] or [`TS_START_MAX`])
    /// before the samples will be accurate.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac, rcc,
    ///     util::new_delay,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay = new_delay(cp.SYST, &dp.RCC);
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.enable_tsen();
    /// // wait for the temperature sensor to startup
    /// delay.delay_us(adc::TS_START_MAX.as_micros() as u32);
    /// ```
    #[inline]
    pub fn enable_tsen(&mut self) {
        self.adc.ccr.modify(|_, w| w.tsen().enabled())
    }

    /// Disable the temperature sensor.
    #[inline]
    pub fn disable_tsen(&mut self) {
        self.adc.ccr.modify(|_, w| w.tsen().disabled())
    }

    /// Returns `true` if the temperature sensor is enabled.
    #[inline]
    #[must_use]
    pub fn is_tsen_enabled(&mut self) -> bool {
        self.adc.ccr.read().tsen().is_enabled()
    }

    /// Get the junction temperature.
    ///
    /// # Panics
    ///
    /// * (debug) ADC is not enabled
    /// * (debug) ADC has a pending disable request
    /// * (debug) Temperature sensor is not enabled
    ///
    /// # Sample Time
    ///
    /// You must set a sampling time with
    /// [`set_sample_times`](Adc::set_sample_times) greater than or equal to
    /// [`TS_MIN_SAMPLE`] before calling this method.
    /// When in doubt use the maximum sampling time, [`Ts::Cyc160`].
    ///
    /// # Calibration
    ///
    /// The temperature calibration provided on-chip appears to be for an
    /// uncalibrated ADC, though I can find no mention of this in the
    /// datasheet.
    ///
    /// If the ADC has been calibrated with [`calibrate`] the calibration offset
    /// will be removed from the sample.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac, rcc,
    ///     util::new_delay,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay = new_delay(cp.SYST, &dp.RCC);
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.enable();
    /// adc.enable_tsen();
    /// delay.delay_us(adc::TS_START_MAX.as_micros() as u32);
    /// adc.set_max_sample_time();
    ///
    /// let tj: i16 = adc.temperature().to_integer();
    /// ```
    ///
    /// [`calibrate`]: crate::adc::Adc::calibrate
    pub fn temperature(&mut self) -> Ratio<i16> {
        debug_assert!(self.is_enabled());
        debug_assert!(self.is_tsen_enabled());

        self.cfg_ch_seq(Ch::Vts.mask());
        self.start_conversion();

        let (ts_cal1, ts_cal2): (u16, u16) = ts_cal();
        let ret: Ratio<i16> =
            Ratio::new_raw(TS_CAL_TEMP_DELTA, ts_cal2.wrapping_sub(ts_cal1) as i16);

        let calfact: u8 = self.adc.calfact.read().calfact().bits();
        let ts_data: u16 = self.poll_data().saturating_add(u16::from(calfact));

        ret * (ts_data.wrapping_sub(ts_cal1) as i16) + TS_CAL1_TEMP
    }

    /// Enable the internal voltage reference.
    #[inline]
    pub fn enable_vref(&mut self) {
        self.adc.ccr.modify(|_, w| w.vrefen().enabled())
    }

    /// Disable the internal voltage reference.
    #[inline]
    pub fn disable_vref(&mut self) {
        self.adc.ccr.modify(|_, w| w.vrefen().disabled())
    }

    /// Returns `true` if the internal voltage reference is enabled.
    #[inline]
    #[must_use]
    pub fn is_vref_enabled(&mut self) -> bool {
        self.adc.ccr.read().vrefen().is_enabled()
    }

    /// Enable oversampling.
    ///
    /// # Panics
    ///
    /// * (debug) ADC is enabled
    #[inline]
    #[cfg(not(feature = "stm32wl5x_cm0p"))]
    pub fn enable_oversampling(&mut self, ratio: OversampleRatio, shift: OversampleShift) {
        debug_assert!(!self.is_enabled());
        self.adc.cfgr2.modify(|_, w| {
            w.ovse()
                .enabled()
                .ovsr()
                .variant(ratio)
                .ovss()
                .variant(shift)
        })
    }

    /// Disables oversampling.
    ///
    /// # Panics
    ///
    /// * (debug) ADC is enabled
    #[inline]
    pub fn disable_oversampling(&mut self) {
        debug_assert!(!self.is_enabled());
        self.adc.cfgr2.modify(|_, w| w.ovse().disabled())
    }

    /// Returns `true` if oversampling is enabled.
    #[inline]
    #[must_use]
    pub fn is_oversampling_enabled(&mut self) -> bool {
        self.adc.cfgr2.read().ovse().is_enabled()
    }

    /// Read the internal voltage reference.
    ///
    /// # Panics
    ///
    /// * (debug) ADC is not enabled
    /// * (debug) ADC has a pending disable request
    /// * (debug) Voltage reference is not enabled
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac, rcc,
    ///     util::new_delay,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay = new_delay(cp.SYST, &dp.RCC);
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.calibrate(&mut delay);
    /// adc.set_max_sample_time();
    /// adc.enable();
    /// adc.enable_vref();
    ///
    /// let vref: u16 = adc.vref();
    /// let vref_cal: u16 = adc::vref_cal();
    /// let error: i16 = ((vref as i16) - (vref_cal as i16)).abs();
    /// assert!(error < 10);
    /// ```
    pub fn vref(&mut self) -> u16 {
        debug_assert!(self.is_enabled());
        debug_assert!(self.is_vref_enabled());
        self.cfg_ch_seq(Ch::Vref.mask());
        self.start_conversion();
        self.poll_data()
    }

    /// Sample the DAC output.
    ///
    /// The DAC must be configured to output to chip peripherals for this to
    /// work as expected.
    ///
    /// # Panics
    ///
    /// * (debug) ADC is not enabled
    /// * (debug) ADC has a pending disable request
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     dac::{Dac, ModeChip},
    ///     pac, rcc,
    ///     util::new_delay,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay = new_delay(cp.SYST, &dp.RCC);
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.calibrate(&mut delay);
    /// adc.set_max_sample_time();
    ///
    /// let mut dac: Dac = Dac::new(dp.DAC, &mut dp.RCC);
    /// dac.set_mode_chip(ModeChip::Norm);
    ///
    /// dac.setup_soft_trigger();
    /// dac.soft_trigger(1234);
    /// // should be in the same ballpark as the DAC output
    /// let sample: u16 = adc.dac();
    /// ```
    pub fn dac(&mut self) -> u16 {
        debug_assert!(self.is_enabled());
        self.cfg_ch_seq(Ch::Dac.mask());
        self.start_conversion();
        self.poll_data()
    }

    /// Sample a GPIO pin.
    ///
    /// # Panics
    ///
    /// * (debug) ADC is not enabled
    /// * (debug) ADC has a pending disable request
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     gpio::{pins::B4, Analog, PortB},
    ///     pac, rcc,
    ///     util::new_delay,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay = new_delay(cp.SYST, &dp.RCC);
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.calibrate(&mut delay);
    /// adc.set_max_sample_time();
    /// adc.enable();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let b4: Analog<B4> = cortex_m::interrupt::free(|cs| Analog::new(gpiob.b4, cs));
    ///
    /// let sample: u16 = adc.pin(&b4);
    /// ```
    #[allow(unused_variables)]
    pub fn pin<P: gpio::sealed::AdcCh>(&mut self, pin: &gpio::Analog<P>) -> u16 {
        debug_assert!(self.is_enabled());
        self.cfg_ch_seq(P::ADC_CH.mask());
        self.start_conversion();
        self.poll_data()
    }

    /// Enable V<sub>BAT</sub>.
    ///
    /// To prevent any unwanted consumption on the battery, it is recommended to
    /// enable the bridge divider only when needed for ADC conversion.
    #[inline]
    pub fn enable_vbat(&mut self) {
        self.adc.ccr.modify(|_, w| w.vbaten().enabled())
    }

    /// Disable V<sub>BAT</sub>.
    #[inline]
    pub fn disable_vbat(&mut self) {
        self.adc.ccr.modify(|_, w| w.vbaten().disabled());
    }

    /// Returns `true` if V<sub>BAT</sub> is enabled.
    #[inline]
    #[must_use]
    pub fn is_vbat_enabled(&self) -> bool {
        self.adc.ccr.read().vbaten().is_enabled()
    }

    /// Sample the V<sub>BAT</sub> pin.
    ///
    /// This is internally connected to a bridge divider, the converted digital
    /// value is a third the V<sub>BAT</sub> voltage.
    ///
    /// # Panics
    ///
    /// * (debug) ADC is not enabled
    /// * (debug) ADC has a pending disable request
    /// * (debug) V<sub>BAT</sub> is not enabled
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     self as hal,
    ///     adc::{self, Adc},
    ///     dac::{Dac, ModeChip},
    ///     pac, rcc,
    ///     util::new_delay,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay = new_delay(cp.SYST, &dp.RCC);
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.calibrate(&mut delay);
    ///
    /// adc.enable();
    /// adc.enable_vbat();
    /// adc.set_max_sample_time();
    /// let sample: u16 = adc.vbat();
    /// ```
    pub fn vbat(&mut self) -> u16 {
        debug_assert!(self.is_enabled());
        debug_assert!(self.is_vbat_enabled());
        self.cfg_ch_seq(Ch::Vbat.mask());
        self.start_conversion();
        self.poll_data()
    }
}

// on-off control
// see RM0453 rev 1 section 18.3.4 page 537
#[cfg(not(feature = "stm32wl5x_cm0p"))]
impl Adc {
    /// Returns `true` if the ADC is enabled.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// assert_eq!(adc.is_enabled(), false);
    /// ```
    #[inline]
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.adc.cr.read().aden().bit_is_set()
    }

    /// Returns `true` if an ADC disable command is in-progress.
    #[inline]
    #[must_use]
    pub fn disable_in_progress(&self) -> bool {
        self.adc.cr.read().addis().bit_is_set()
    }

    /// Returns `true` if the ADC is disabled, and there is no disable command
    /// in-progress.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// assert_eq!(adc.is_disabled(), true);
    /// ```
    #[inline]
    #[must_use]
    pub fn is_disabled(&self) -> bool {
        let cr = self.adc.cr.read();
        cr.aden().bit_is_clear() && cr.addis().bit_is_clear()
    }

    /// Start the ADC enable procedure.
    ///
    /// This is advanced ADC usage, most of the time you will want to use
    /// [`enable`](Self::enable).
    ///
    /// This will not poll for completion, when this method returns the ADC
    /// may not be enabled.
    ///
    /// The method returns `true` if the caller function should poll for
    /// completion (the ADC was not already enabled),
    /// if the ADC was already enabled and the ADRDY interrupt was cleared then
    /// the ADRDY bit will **not** be set again after calling this method
    /// which can lead to polling loops that will never terminate.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// if adc.start_enable() {
    ///     while Adc::isr().adrdy().is_not_ready() {}
    /// }
    /// ```
    #[inline]
    #[must_use = "the return value indicates if you should wait for completion"]
    pub fn start_enable(&mut self) -> bool {
        if self.adc.cr.read().aden().is_disabled() {
            self.adc.isr.write(|w| w.adrdy().set_bit());
            self.adc.cr.write(|w| w.aden().set_bit());
            true
        } else {
            false
        }
    }

    /// Enable the ADC and poll for completion.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// adc.enable();
    /// ```
    #[inline]
    pub fn enable(&mut self) {
        if self.start_enable() {
            while self.adc.isr.read().adrdy().is_not_ready() {}
        }
    }

    /// Start the ADC disable procedure.
    ///
    /// This is advanced ADC usage, most of the time you will want to use
    /// [`disable`](Self::disable).
    ///
    /// This will not poll for completion, when this function returns the ADC
    /// may not be disabled.
    ///
    /// This will stop any conversions in-progress.
    ///
    /// The ADC takes about 20 CPU cycles to disable with a 48MHz sysclk and
    /// the ADC on the 16MHz HSI clock.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// adc.enable();
    /// // ... use ADC
    /// adc.start_disable();
    /// while !adc.is_disabled() {}
    /// ```
    pub fn start_disable(&mut self) {
        // RM0453 rev 1 section 18.3.4 page 537 ADC on-off control
        // 1. Check that ADSTART = 0 in the ADC_CR register to ensure that no
        //    conversion is ongoing.
        //    If required, stop any ongoing conversion by writing 1 to the
        //    ADSTP bit in the ADC_CR register and waiting until this bit is
        //    read at 0.
        // 2. Set ADDIS = 1 in the ADC_CR register.
        // 3. If required by the application, wait until ADEN = 0 in the ADC_CR
        //    register, indicating that the ADC is fully disabled
        //    (ADDIS is automatically reset once ADEN = 0).
        // 4. Clear the ADRDY bit in ADC_ISR register by programming this bit to 1
        //    (optional).
        self.stop_conversion();
        // Setting ADDIS to `1` is only effective when ADEN = 1 and ADSTART = 0
        // (which ensures that no conversion is ongoing)
        if self.adc.cr.read().aden().bit_is_set() {
            self.adc.cr.write(|w| w.addis().set_bit());
        }
    }

    /// Disable the ADC and poll for completion.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::PClkDiv4, &mut dp.RCC);
    /// adc.enable();
    /// // ... use ADC
    /// adc.disable();
    /// ```
    pub fn disable(&mut self) {
        self.start_disable();
        while !self.is_disabled() {}
    }
}

// calibration related methods
// see RM0453 rev 1 section 18.3.3 page 536
#[cfg(not(feature = "stm32wl5x_cm0p"))]
impl Adc {
    /// Calibrate the ADC for additional accuracy.
    ///
    /// Calibration should be performed before starting A/D conversion.
    /// It removes the offset error which may vary from chip to chip due to
    /// process variation.
    ///
    /// The calibration factor is lost in the following cases:
    /// * The power supply is removed from the ADC
    ///   (for example when entering STANDBY or VBAT mode)
    /// * The ADC peripheral is reset.
    ///
    /// This will disable the ADC if it is not already disabled.
    ///
    /// This function is the simple way to calibrate the ADC, you can use
    /// these methods to achieve the same results if you desire finer controls:
    ///
    /// * [`enable_vreg`](Self::enable_vreg)
    /// * A delay function
    /// * [`start_calibrate`](Self::start_calibrate)
    /// * [`set_ier`](Self::set_ier) (optional)
    /// * [`Adc::isr`]
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac, rcc,
    ///     util::new_delay,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay = new_delay(cp.SYST, &dp.RCC);
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.calibrate(&mut delay);
    /// ```
    pub fn calibrate<D: DelayUs<u8>>(&mut self, delay: &mut D) {
        self.enable_vreg();

        // voltage regulator output is available after T_ADCVREG_SETUP
        delay.delay_us(T_ADCVREG_SETUP_MICROS);

        self.start_calibrate();

        // takes 401 cycles at 48MHz sysclk, ADC at 16MHz with HSI
        while self.adc.cr.read().adcal().is_calibrating() {}
        self.adc.isr.write(|w| w.eocal().set_bit());
    }

    /// Enable the ADC voltage regulator for calibration.
    ///
    /// This is advanced ADC usage, most of the time you will want to use
    /// [`calibrate`](Self::calibrate).
    ///
    /// This will disable the ADC and DMA request generation if not already
    /// disabled.
    ///
    /// You **MUST** wait [`T_ADCVREG_SETUP`] before the voltage regulator
    /// output is available.  This delay is not performed for you.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac, rcc,
    ///     util::new_delay,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay = new_delay(cp.SYST, &dp.RCC);
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.enable_vreg();
    /// delay.delay_us(u32::from(adc::T_ADCVREG_SETUP_MICROS));
    /// ```
    pub fn enable_vreg(&mut self) {
        // RM0453 rev 1 section 18.3.3 page 536 Software calibration procedure
        // 1. Ensure that ADEN = 0, ADVREGEN = 1 and DMAEN = 0.
        // 2. Set ADCAL = 1.
        // 3. Wait until ADCAL = 0 (or until EOCAL = 1).
        //    This can be handled by interrupt if the interrupt is enabled by
        //    setting the EOCALIE bit in the ADC_IER register
        // 4. The calibration factor can be read from bits 6:0 of ADC_DR or
        //    ADC_CALFACT registers.

        // takes appx 55 cycles when already disabled
        self.disable();

        // enable the voltage regulator as soon as possible to start the
        // countdown on the regulator setup time
        // this is a write because all other fields must be zero
        self.adc.cr.write(|w| w.advregen().enabled());
        // disable DMA per the calibration procedure
        self.adc.cfgr1.modify(|_, w| w.dmaen().clear_bit());
    }

    /// Disable the ADC voltage regulator.
    ///
    /// # Panics
    ///
    /// * (debug) ADC is enabled
    #[inline]
    pub fn disable_vreg(&mut self) {
        debug_assert!(self.is_disabled());
        self.adc.cr.write(|w| w.advregen().disabled());
    }

    /// Start the ADC calibration.
    ///
    /// This is advanced ADC usage, most of the time you will want to use
    /// [`calibrate`](Self::calibrate).
    ///
    /// When this function returns the ADC calibration has started, but
    /// may not have finished.
    /// Check if the ADC calibration has finished with [`Adc::isr`].
    ///
    /// # Panics
    ///
    /// * (debug) ADC is enabled.
    /// * (debug) ADC voltage regulator is not enabled.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac, rcc,
    ///     util::new_delay,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay = new_delay(cp.SYST, &dp.RCC);
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    ///
    /// adc.enable_vreg();
    /// delay.delay_us(u32::from(adc::T_ADCVREG_SETUP_MICROS));
    /// adc.start_calibrate();
    /// // datasheet says this takes 82 ADC clock cycles
    /// // my measurements are closer to 120 ADC clock cycles
    /// while Adc::isr().eocal().is_not_complete() {}
    /// ```
    #[inline]
    pub fn start_calibrate(&mut self) {
        debug_assert!(self.adc.cr.read().advregen().is_enabled());
        debug_assert!(self.is_disabled());
        self.adc
            .cr
            .write(|w| w.adcal().start_calibration().advregen().enabled());
    }

    /// Get the ADC calibration factor.
    ///
    /// # Example
    ///
    /// See [`force_cal`](Self::force_cal).
    #[inline]
    pub fn calfact(&self) -> u8 {
        self.adc.calfact.read().calfact().bits()
    }

    /// Force the ADC calibration.
    ///
    /// The calibration factor is lost each time power is removed from the ADC
    /// (for example when entering standby or V<sub>BAT</sub> mode)
    /// It is possible to save and restore the calibration factor with firmware
    /// to save time when re-starting the ADC (as long as temperature and
    /// voltage are stable during the ADC power-down).
    ///
    /// # Panics
    ///
    /// * (debug) ADC is not enabled
    /// * (debug) ADC conversion is in-progress
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     adc::{self, Adc},
    ///     pac, rcc,
    ///     util::new_delay,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay = new_delay(cp.SYST, &dp.RCC);
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.calibrate(&mut delay);
    ///
    /// // save the calibration factor
    /// let calfact: u8 = adc.calfact();
    ///
    /// // restore the calibration
    /// adc.enable(); // ADC must be enabled to restore the calibration
    /// adc.force_cal(calfact);
    /// ```
    #[inline]
    pub fn force_cal(&mut self, calfact: u8) {
        debug_assert!(self.is_enabled());
        debug_assert!(self.adc.cr.read().adstart().bit_is_clear());
        self.adc.calfact.write(|w| w.calfact().bits(calfact))
    }
}
