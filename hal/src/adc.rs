//! Analog to digital converter

pub use num_rational::Ratio;

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

// temperature sensor min sampling time
#[allow(dead_code)]
const TS_TEMP: Duration = Duration::from_micros(5);

// typical startup time when entering continuous mode
#[allow(dead_code)]
const T_START_TYP: Duration = Duration::from_micros(70);
const T_START_MAX: Duration = Duration::from_micros(120);
const T_START_MAX_MICROS: u8 = T_START_MAX.as_micros() as u8;

const T_ADCVREG_SETUP: Duration = Duration::from_micros(20);
const T_ADCVREG_SETUP_MICROS: u8 = T_ADCVREG_SETUP.as_micros() as u8;

/// Mask of all valid channels
const CH_MASK: u32 = 0x13FFF;

/// Internal voltage reference ADC calibration
///
/// This is raw ADC data aquired at 30 °C (± 5 °C).
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

impl Clk {
    const fn ckmode(&self) -> pac::adc::cfgr2::CKMODE_A {
        match self {
            Clk::RccHsi | Clk::RccPllP | Clk::RccSysClk => pac::adc::cfgr2::CKMODE_A::ADCLK,
            Clk::PClkDiv2 => pac::adc::cfgr2::CKMODE_A::PCLK_DIV2,
            Clk::PClkDiv4 => pac::adc::cfgr2::CKMODE_A::PCLK_DIV4,
            Clk::PClk => pac::adc::cfgr2::CKMODE_A::PCLK,
        }
    }

    const fn adcsel(&self) -> pac::rcc::ccipr::ADCSEL_A {
        match self {
            Clk::RccHsi => pac::rcc::ccipr::ADCSEL_A::HSI16,
            Clk::RccPllP => pac::rcc::ccipr::ADCSEL_A::PLLP,
            Clk::RccSysClk => pac::rcc::ccipr::ADCSEL_A::SYSCLK,
            _ => pac::rcc::ccipr::ADCSEL_A::NOCLOCK,
        }
    }
}

/// ADC sample times
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum SampleTime {
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

impl Default for SampleTime {
    fn default() -> Self {
        SampleTime::Cyc1
    }
}

impl From<SampleTime> for u8 {
    fn from(st: SampleTime) -> Self {
        st as u8
    }
}

impl From<SampleTime> for u32 {
    fn from(st: SampleTime) -> Self {
        st as u32
    }
}

/// ADC channels
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[repr(u8)]
pub enum Ch {
    /// ADC input 0
    In0 = 0,
    /// ADC input 1
    In1 = 1,
    /// ADC input 2
    In2 = 2,
    /// ADC input 3
    In3 = 3,
    /// ADC input 4
    In4 = 4,
    /// ADC input 5
    In5 = 5,
    /// ADC input 6
    In6 = 6,
    /// ADC input 7
    In7 = 7,
    /// ADC input 8
    In8 = 8,
    /// ADC input 9
    In9 = 9,
    /// ADC input 10
    In10 = 10,
    /// ADC input 11
    In11 = 11,
    /// Junction temperature sensor
    Vts = 12,
    /// Internal voltage reference
    Vref = 13,
    /// Battery voltage divided by 3
    Vbat = 14,
    // 15, 16 are reserved
    /// Digital to analog coverter output
    Dac = 17,
}

impl Ch {
    /// Bitmask of the channel.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::adc::Ch;
    ///
    /// assert_eq!(Ch::In0.mask(), 0x001);
    /// assert_eq!(Ch::In8.mask(), 0x100);
    /// ```
    pub const fn mask(self) -> u32 {
        1 << (self as u8)
    }
}

/// Analog to digital converter driver
#[derive(Debug)]
pub struct Adc {
    adc: pac::ADC,
}

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
    /// use stm32wl_hal::{
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
    pub fn new(adc: pac::ADC, clk: Clk, rcc: &mut pac::RCC) -> Adc {
        unsafe { Self::pulse_reset(rcc) };
        Self::enable_clock(rcc);
        adc.cfgr2.write(|w| w.ckmode().variant(clk.ckmode()));
        rcc.ccipr.modify(|_, w| w.adcsel().variant(clk.adcsel()));

        Adc { adc }
    }

    /// Free the ADC peripheral from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     adc::{self, Adc},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let adc: pac::ADC = dp.ADC;
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut adc_driver: Adc = Adc::new(adc, adc::Clk::RccHsi, &mut dp.RCC);
    /// // ... use ADC
    /// let adc: pac::ADC = adc_driver.free();
    /// ```
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
    ///    peripheral.  Singleton checks are bypassed with this method.
    /// 2. You are responsible for setting up the ADC correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::adc::Adc;
    ///
    /// // ... setup happens here
    ///
    /// let adc = unsafe { Adc::steal() };
    /// ```
    ///
    /// [`new`]: Adc::new
    pub unsafe fn steal() -> Adc {
        Adc {
            adc: pac::Peripherals::steal().ADC,
        }
    }

    /// Disable the ADC clock.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the ADC before disabling the clock.
    /// 2. You are responsible for en-enabling the clock before using the ADC.
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb2enr.modify(|_, w| w.adcen().disabled());
    }

    /// Enable the ADC clock.
    ///
    /// [`new`](crate::adc::Adc::new) will enable clocks for you.
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
    /// 1. Ensure nothing is using the ADC before pulsing reset.
    /// 2. You are responsible for setting up the ADC after a reset.
    pub unsafe fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb2rstr.modify(|_, w| w.adcrst().set_bit());
        rcc.apb2rstr.modify(|_, w| w.adcrst().clear_bit());
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
    /// unsafe { stm32wl_hal::adc::Adc::unmask_irq() };
    /// ```
    #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    #[cfg_attr(docsrs, doc(cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))))]
    pub unsafe fn unmask_irq() {
        pac::NVIC::unmask(pac::Interrupt::ADC)
    }

    /// Mask the ADC IRQ in the NVIC.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    /// stm32wl_hal::adc::Adc::mask_irq()
    /// ```
    #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    #[cfg_attr(docsrs, doc(cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))))]
    pub fn mask_irq() {
        pac::NVIC::mask(pac::Interrupt::ADC)
    }

    fn set_sample_times(&mut self, mask: u32, sel1: SampleTime, sel2: SampleTime) {
        self.adc.smpr.write(|w| unsafe {
            w.bits((mask & CH_MASK) << 8 | u32::from(sel2) << 4 | u32::from(sel1))
        })
    }

    /// Returns `true` if the ADC is enabled.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
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
    pub fn is_enabled(&self) -> bool {
        self.adc.cr.read().aden().bit_is_set()
    }

    /// Returns `true` if the ADC is disabled.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
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
    pub fn is_disabled(&self) -> bool {
        let cr = self.adc.cr.read();
        cr.aden().bit_is_clear() && cr.addis().bit_is_clear()
    }

    /// Start the ADC enable procedure.
    ///
    /// This will not poll for completion, when this function returns the ADC
    /// may not be enabled.
    ///
    /// Returns `true` if the caller function should poll for completion
    fn start_enable(&mut self) -> bool {
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
    /// use stm32wl_hal::{
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
    /// adc.enable();
    /// ```
    pub fn enable(&mut self) {
        if self.start_enable() {
            while self.adc.isr.read().adrdy().is_not_ready() {}
        }
    }

    /// Enable the ADC and wait for a completion interrupt.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # async fn doctest() {
    /// use stm32wl_hal::{
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
    /// adc.aio_enable().await;
    /// # }
    /// ```
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    #[cfg_attr(
        docsrs,
        doc(cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p"))))
    )]
    pub async fn aio_enable(&mut self) {
        if self.start_enable() {
            self.adc.ier.write(|w| w.adrdyie().enabled());
            futures::future::poll_fn(aio::poll).await;
        }
    }

    /// Start the ADC disable procedure.
    ///
    /// This will stop any conversions in-progress and set the addis bit if
    /// the ADC is enabled.
    ///
    /// This will not poll for completion, when this function returns the ADC
    /// may not be disabled.
    ///
    /// The ADC takes about 20 CPU cycles to disable with a 48MHz sysclk and
    /// the ADC on the 16MHz HSI clock.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
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
        if self.adc.cr.read().adstart().bit_is_set() {
            self.adc.cr.modify(|_, w| w.adstp().stop_conversion());
            while self.adc.cr.read().adstp().bit_is_set() {}
        }
        // Setting ADDIS to `1` is only effective when ADEN = 1 and ADSTART = 0
        // (which ensures that no conversion is ongoing)
        if self.adc.cr.read().aden().bit_is_set() {
            self.adc.cr.modify(|_, w| w.addis().set_bit());
        }
    }

    fn setup_calibrate(&mut self) {
        // RM0453 rev 1 section 18.3.3 page 536 Software calibration procedure
        // 1. Ensure that ADEN = 0, ADVREGEN = 1 and DMAEN = 0.
        // 2. Set ADCAL = 1.
        // 3. Wait until ADCAL = 0 (or until EOCAL = 1).
        //    This can be handled by interrupt if the interrupt is enabled by
        //    setting the EOCALIE bit in the ADC_IER register
        // 4. The calibration factor can be read from bits 6:0 of ADC_DR or
        //    ADC_CALFACT registers.

        self.start_disable();
        while !self.is_disabled() {}

        // enable the voltage regulator as soon as possible to start the
        // countdown on the regulator setup time
        self.adc.cr.write(|w| w.advregen().enabled());
        // disable DMA per the calibration procedure
        self.adc.cfgr1.write(|w| w.dmaen().clear_bit());
    }

    // requires setup_calibrate and delay for T_ADCVREG_SETUP
    fn start_calibrate(&mut self) {
        self.adc
            .cr
            .write(|w| w.adcal().start_calibration().advregen().enabled());
    }

    /// Configure the channel sequencer
    ///
    /// See section 18.3.8 page 542 "Channel selection"
    fn set_chsel(&mut self, ch: u32) {
        self.adc
            .chselr0()
            .write(|w| unsafe { w.chsel().bits(ch & 0x13FFF) });
    }

    fn cfg_ch_seq(&mut self, ch: u32) {
        self.set_chsel(ch);
        while self.adc.isr.read().ccrdy().is_not_complete() {}
    }

    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    async fn aio_cfg_ch_seq(&mut self, ch: u32) {
        self.set_chsel(ch);
        self.adc.ier.write(|w| w.ccrdyie().enabled());
        futures::future::poll_fn(aio::poll).await;
    }

    fn data(&self) -> u16 {
        while self.adc.isr.read().eoc().is_not_complete() {}
        self.adc.dr.read().data().bits()
    }

    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    async fn aio_data(&self) -> u16 {
        self.adc.ier.write(|w| w.eocie().enabled());
        futures::future::poll_fn(aio::poll).await;
        self.adc.dr.read().data().bits()
    }

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
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     adc::{self, Adc},
    ///     cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
    ///     pac, rcc,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.calibrate(&mut delay);
    /// ```
    pub fn calibrate<D: DelayUs<u8>>(&mut self, delay: &mut D) {
        self.setup_calibrate();

        // voltage regulator output is available after T_ADCVREG_SETUP
        delay.delay_us(T_ADCVREG_SETUP_MICROS);

        self.start_calibrate();

        // takes 401 cycles at 48MHz sysclk, ADC at 16MHz with HSI
        while self.adc.cr.read().adcal().is_calibrating() {}
    }

    /// Asynchronously Calibrate the ADC for additional accuracy.
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
    /// # Example
    ///
    /// ```no_run
    /// # async fn doctest() {
    /// use stm32wl_hal::{
    ///     adc::{self, Adc},
    ///     cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
    ///     pac, rcc,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.aio_calibrate(&mut delay).await;
    /// # }
    /// ```
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    #[cfg_attr(
        docsrs,
        doc(cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p"))))
    )]
    pub async fn aio_calibrate<D: DelayUs<u8>>(&mut self, delay: &mut D) {
        self.setup_calibrate();

        // voltage regulator output is available after T_ADCVREG_SETUP
        // TODO: replace with async delay
        delay.delay_us(T_ADCVREG_SETUP_MICROS);

        self.adc.ier.write(|w| w.eocalie().enabled());

        self.start_calibrate();

        futures::future::poll_fn(aio::poll).await;
    }

    /// Get the junction jemperature
    ///
    /// This will enable the ADC if not already enabled.
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
    /// use stm32wl_hal::{
    ///     adc::{self, Adc},
    ///     cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
    ///     pac, rcc,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// let tj: i16 = adc.temperature(&mut delay).to_integer();
    /// ```
    ///
    /// [`calibrate`]: crate::adc::Adc::calibrate
    pub fn temperature<D: DelayUs<u8>>(&mut self, delay: &mut D) -> Ratio<i16> {
        self.enable();

        // 1. Select the ADC VIN[12] input channel
        self.cfg_ch_seq(Ch::Vts.mask());

        // 2. Select an appropriate sampling time specified in the device
        //    datasheet (TS_temp).
        // TODO: replace with calculation of sample times from ADC clock
        self.set_sample_times(0, SampleTime::Cyc160, SampleTime::Cyc160);

        // 3. Set the TSEN bit in the ADC_CCR register to wake up the
        //    temperature sensor from power down mode
        //    and wait for its stabilization time (tSTART).
        self.adc.ccr.modify(|_, w| w.tsen().enabled());
        delay.delay_us(T_START_MAX_MICROS);

        // 4. Start the ADC conversion by setting the ADSTART bit in the ADC_CR
        //    register (or by external trigger)
        self.adc.cr.write(|w| w.adstart().set_bit());

        let (ts_cal1, ts_cal2): (u16, u16) = ts_cal();
        let ret: Ratio<i16> = Ratio::new(TS_CAL_TEMP_DELTA, ts_cal2.wrapping_sub(ts_cal1) as i16);

        // 5. Read the resulting VTS data in the ADC_DR register
        let calfact: u8 = self.adc.calfact.read().calfact().bits();
        let ts_data: u16 = self.data().saturating_add(u16::from(calfact));
        self.adc.ccr.modify(|_, w| w.tsen().disabled());
        // 6. Calculate the temperature
        ret * (ts_data.wrapping_sub(ts_cal1) as i16) + TS_CAL1_TEMP
    }

    /// Get the junction jemperature.
    ///
    /// This will enable the ADC if not already enabled.
    ///
    /// # Calibration
    ///
    /// The temperature calibration provided on-chip appears to be for an
    /// uncalibrated ADC, though I can find no mention of this in the
    /// datasheet.
    ///
    /// If the ADC has been calibrated with [`calibrate`] or [`aio_calibrate`]
    /// the calibration offset will be removed from the sample.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # async fn doctest() {
    /// use stm32wl_hal::{
    ///     adc::{self, Adc},
    ///     cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
    ///     pac, rcc,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// let tj: i16 = adc.aio_temperature(&mut delay).await.to_integer();
    /// # }
    /// ```
    ///
    /// [`calibrate`]: crate::adc::Adc::calibrate
    /// [`aio_calibrate`]: crate::adc::Adc::aio_calibrate
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    #[cfg_attr(
        docsrs,
        doc(cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p"))))
    )]
    pub async fn aio_temperature<D: DelayUs<u8>>(&mut self, delay: &mut D) -> Ratio<i16> {
        self.aio_enable().await;

        // 1. Select the ADC VIN[12] input channel
        self.aio_cfg_ch_seq(Ch::Vts.mask()).await;

        // 2. Select an appropriate sampling time specified in the device
        //    datasheet (TS_temp).
        // TODO: replace with calculation of sample times from ADC clock
        self.set_sample_times(0, SampleTime::Cyc160, SampleTime::Cyc160);

        // 3. Set the TSEN bit in the ADC_CCR register to wake up the
        //    temperature sensor from power down mode
        //    and wait for its stabilization time (tSTART).
        self.adc.ccr.modify(|_, w| w.tsen().enabled());
        // TODO: replace with async delay
        delay.delay_us(T_START_MAX_MICROS);

        // 4. Start the ADC conversion by setting the ADSTART bit in the ADC_CR
        //    register (or by external trigger)
        self.adc.cr.write(|w| w.adstart().set_bit());

        let (ts_cal1, ts_cal2): (u16, u16) = ts_cal();
        let ret: Ratio<i16> = Ratio::new(TS_CAL_TEMP_DELTA, ts_cal2.wrapping_sub(ts_cal1) as i16);

        // 5. Read the resulting VTS data in the ADC_DR register
        let calfact: u8 = self.adc.calfact.read().calfact().bits();
        let ts_data: u16 = self.aio_data().await.saturating_add(u16::from(calfact));
        self.adc.ccr.modify(|_, w| w.tsen().disabled());
        // 6. Calculate the temperature
        ret * (ts_data.wrapping_sub(ts_cal1) as i16) + TS_CAL1_TEMP
    }

    /// Read the internal voltage reference.
    ///
    /// This will enable the ADC if not already enabled.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     adc::{self, Adc},
    ///     cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
    ///     pac, rcc,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.calibrate(&mut delay);
    /// let vref: u16 = adc.vref();
    /// let vref_cal: u16 = adc::vref_cal();
    /// let error: i16 = ((vref as i16) - (vref_cal as i16)).abs();
    /// assert!(error < 10);
    /// ```
    pub fn vref(&mut self) -> u16 {
        self.enable();
        self.cfg_ch_seq(Ch::Vref.mask());
        self.set_sample_times(0, SampleTime::Cyc160, SampleTime::Cyc160);
        self.adc.ccr.modify(|_, w| w.vrefen().enabled());
        self.adc.cr.write(|w| w.adstart().set_bit());
        let data: u16 = self.data();
        self.adc.ccr.modify(|_, w| w.vrefen().disabled());
        data
    }

    /// Read the internal voltage reference.
    ///
    /// This will enable the ADC if not already enabled.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # async fn doctest() {
    /// use stm32wl_hal::{
    ///     adc::{self, Adc},
    ///     cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
    ///     pac, rcc,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let mut delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));
    ///
    /// let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    /// adc.aio_calibrate(&mut delay).await;
    /// let vref: u16 = adc.aio_vref().await;
    /// let vref_cal: u16 = adc::vref_cal();
    /// let error: i16 = ((vref as i16) - (vref_cal as i16)).abs();
    /// assert!(error < 10);
    /// # }
    /// ```
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    #[cfg_attr(
        docsrs,
        doc(cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p"))))
    )]
    pub async fn aio_vref(&mut self) -> u16 {
        self.aio_enable().await;
        self.aio_cfg_ch_seq(Ch::Vref.mask()).await;
        self.set_sample_times(0, SampleTime::Cyc160, SampleTime::Cyc160);
        self.adc.ccr.modify(|_, w| w.vrefen().enabled());
        self.adc.cr.write(|w| w.adstart().set_bit());
        let data: u16 = self.aio_data().await;
        self.adc.ccr.modify(|_, w| w.vrefen().disabled());
        data
    }
}

#[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
mod aio {
    use core::{
        sync::atomic::{AtomicU32, Ordering::SeqCst},
        task::Poll,
    };
    use futures_util::task::AtomicWaker;

    static ADC_WAKER: AtomicWaker = AtomicWaker::new();
    static ADC_RESULT: AtomicU32 = AtomicU32::new(0);

    pub fn poll(cx: &mut core::task::Context<'_>) -> Poll<u32> {
        ADC_WAKER.register(cx.waker());
        match ADC_RESULT.load(SeqCst) {
            0 => core::task::Poll::Pending,
            _ => {
                ADC_WAKER.take();
                let isr: u32 = ADC_RESULT.swap(0, SeqCst);
                Poll::Ready(isr)
            }
        }
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    mod irq {
        use super::{SeqCst, ADC_RESULT, ADC_WAKER};
        use crate::pac::{self, interrupt};

        #[interrupt]
        #[allow(non_snake_case)]
        fn ADC() {
            debug_assert_eq!(ADC_RESULT.load(SeqCst), 0);

            let adc: pac::ADC = unsafe { pac::Peripherals::steal() }.ADC;

            // store result
            ADC_RESULT.store(adc.isr.read().bits(), SeqCst);

            // clear and disable IRQs
            #[rustfmt::skip]
            adc.isr.write(|w| {
                w
                    .ccrdy().set_bit()
                    .eocal().set_bit()
                    .awd3().set_bit()
                    .awd2().set_bit()
                    .awd1().set_bit()
                    .ovr().set_bit()
                    .eos().set_bit()
                    .eoc().set_bit()
                    .eosmp().set_bit()
                    .adrdy().set_bit()
            });
            #[rustfmt::skip]
            adc.ier.write(|w|  {
                w
                    .ccrdyie().disabled()
                    .eocalie().disabled()
                    .awd3ie().disabled()
                    .awd2ie().disabled()
                    .awd1ie().disabled()
                    .ovrie().disabled()
                    .eosie().disabled()
                    .eocie().disabled()
                    .eosmpie().disabled()
                    .adrdyie().disabled()
            });

            ADC_WAKER.wake();
        }
    }
}
