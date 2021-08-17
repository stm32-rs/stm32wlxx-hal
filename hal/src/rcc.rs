//! Reset and clocking control

use crate::Ratio;
use core::{
    convert::{TryFrom, TryInto},
    sync::atomic::{compiler_fence, Ordering::SeqCst},
};
use cortex_m::peripheral::syst::SystClkSource;

use crate::pac;

fn rcc_set_flash_latency_from_msi_range(
    flash: &pac::FLASH,
    rcc: &pac::RCC,
    msi_range: MsiRange,
    vos: Vos,
) {
    let msi_freq: u32 = msi_range.as_hertz();
    let div: u32 = u32::from(hclk3_prescaler_div(rcc));
    let flash_clk_src_freq: u32 = msi_freq / div;

    let latency: FlashLatency = FlashLatency::from_hertz(vos, flash_clk_src_freq);

    flash.acr.modify(|_, w| w.latency().variant(latency.into()));

    while flash.acr.read().latency().bits() != (latency as u8) {
        compiler_fence(SeqCst);
    }
}

#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum FlashLatency {
    /// Zero wait state.
    Zero = 0b000,
    /// One wait state.
    One = 0b001,
    /// Two wait states.
    Two = 0b010,
}

impl FlashLatency {
    pub const fn from_hertz(vos: Vos, hz: u32) -> FlashLatency {
        match vos {
            Vos::Range1 => match hz {
                0..=18_000_000 => FlashLatency::Zero,
                19_000_000..=36_000_000 => FlashLatency::One,
                _ => FlashLatency::Two,
            },
            Vos::Range2 => match hz {
                0..=6_000_000 => FlashLatency::Zero,
                7_000_000..=12_000_000 => FlashLatency::One,
                _ => FlashLatency::Two,
            },
        }
    }
}

impl From<FlashLatency> for u8 {
    fn from(x: FlashLatency) -> Self {
        x as u8
    }
}

impl From<FlashLatency> for pac::flash::acr::LATENCY_A {
    fn from(fl: FlashLatency) -> Self {
        match fl {
            FlashLatency::Zero => pac::flash::acr::LATENCY_A::WS0,
            FlashLatency::One => pac::flash::acr::LATENCY_A::WS1,
            FlashLatency::Two => pac::flash::acr::LATENCY_A::WS2,
        }
    }
}

/// MSI clock ranges
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Clone, Copy)]
enum MsiRange {
    /// Around 100 kHz.
    Range0 = 0b0000,
    /// Around 200 kHz.
    Range1 = 0b0001,
    /// Around 400 kHz.
    Range2 = 0b0010,
    /// Around 800 kHz.
    Range3 = 0b0011,
    /// Around 1 MHz.
    Range4 = 0b0100,
    /// Around 2 MHz.
    Range5 = 0b0101,
    /// Around 4 MHz.
    Range6 = 0b0110,
    /// Around 8 MHz.
    Range7 = 0b0111,
    /// Around 16 MHz.
    Range8 = 0b1000,
    /// Around 24 MHz.
    Range9 = 0b1001,
    /// Around 32 MHz.
    Range10 = 0b1010,
    /// Around 48 MHz.
    Range11 = 0b1011,
}

impl MsiRange {
    pub const fn as_hertz(&self) -> u32 {
        match self {
            MsiRange::Range0 => 100_000,
            MsiRange::Range1 => 200_000,
            MsiRange::Range2 => 400_000,
            MsiRange::Range3 => 800_000,
            MsiRange::Range4 => 1_000_000,
            MsiRange::Range5 => 2_000_000,
            MsiRange::Range6 => 4_000_000,
            MsiRange::Range7 => 8_000_000,
            MsiRange::Range8 => 16_000_000,
            MsiRange::Range9 => 24_000_000,
            MsiRange::Range10 => 32_000_000,
            MsiRange::Range11 => 48_000_000,
        }
    }

    /// Get the current MSI clock range from hardware registers.
    ///
    /// The unwraps in this function are desired because the other values are
    /// reserved.
    pub fn from_rcc(rcc: &pac::RCC) -> MsiRange {
        use pac::rcc::cr::MSIRGSEL_A::{CR, CSR};

        let cr = rcc.cr.read();
        match cr.msirgsel().variant() {
            CSR => unwrap!(rcc.csr.read().msisrange().bits().try_into()),
            CR => unwrap!(cr.msirange().bits().try_into()),
        }
    }
}

impl Default for MsiRange {
    fn default() -> Self {
        MsiRange::Range6
    }
}

impl From<MsiRange> for u8 {
    fn from(x: MsiRange) -> Self {
        x as u8
    }
}

impl TryFrom<u8> for MsiRange {
    type Error = u8;
    fn try_from(x: u8) -> Result<Self, Self::Error> {
        match x {
            0b0000 => Ok(MsiRange::Range0),
            0b0001 => Ok(MsiRange::Range1),
            0b0010 => Ok(MsiRange::Range2),
            0b0011 => Ok(MsiRange::Range3),
            0b0100 => Ok(MsiRange::Range4),
            0b0101 => Ok(MsiRange::Range5),
            0b0110 => Ok(MsiRange::Range6),
            0b0111 => Ok(MsiRange::Range7),
            0b1000 => Ok(MsiRange::Range8),
            0b1001 => Ok(MsiRange::Range9),
            0b1010 => Ok(MsiRange::Range10),
            0b1011 => Ok(MsiRange::Range11),
            _ => Err(x),
        }
    }
}

impl From<MsiRange> for pac::rcc::cr::MSIRANGE_A {
    fn from(mr: MsiRange) -> Self {
        match mr {
            MsiRange::Range0 => pac::rcc::cr::MSIRANGE_A::RANGE100K,
            MsiRange::Range1 => pac::rcc::cr::MSIRANGE_A::RANGE200K,
            MsiRange::Range2 => pac::rcc::cr::MSIRANGE_A::RANGE400K,
            MsiRange::Range3 => pac::rcc::cr::MSIRANGE_A::RANGE800K,
            MsiRange::Range4 => pac::rcc::cr::MSIRANGE_A::RANGE1M,
            MsiRange::Range5 => pac::rcc::cr::MSIRANGE_A::RANGE2M,
            MsiRange::Range6 => pac::rcc::cr::MSIRANGE_A::RANGE4M,
            MsiRange::Range7 => pac::rcc::cr::MSIRANGE_A::RANGE8M,
            MsiRange::Range8 => pac::rcc::cr::MSIRANGE_A::RANGE16M,
            MsiRange::Range9 => pac::rcc::cr::MSIRANGE_A::RANGE24M,
            MsiRange::Range10 => pac::rcc::cr::MSIRANGE_A::RANGE32M,
            MsiRange::Range11 => pac::rcc::cr::MSIRANGE_A::RANGE48M,
        }
    }
}

fn hclk3_prescaler_div(rcc: &pac::RCC) -> u16 {
    pre_div(rcc.extcfgr.read().shdhpre().bits())
}

/// Prescaler divisor.
/// Works for SHDHPRE, C2HPRE, HPRE.
const fn pre_div(pre: u8) -> u16 {
    match pre {
        0b0001 => 3,
        0b0010 => 5,
        0b0101 => 6,
        0b0110 => 10,
        0b0111 => 32,
        0b1000 => 2,
        0b1001 => 4,
        0b1010 => 8,
        0b1011 => 16,
        0b1100 => 64,
        0b1101 => 128,
        0b1110 => 256,
        0b1111 => 512,
        _ => 1,
    }
}

/// APB Prescaler divisor.
/// Works for PPRE1, PPRE2.
const fn ppre_div(pre: u8) -> u8 {
    match pre {
        0b100 => 2,
        0b101 => 4,
        0b110 => 8,
        0b111 => 16,
        _ => 1,
    }
}

/// Voltage scaling
///
/// See RM0453 rev 1 section 6.1.4 dynamic voltage scaling management
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[repr(u8)]
enum Vos {
    /// High-performance range
    ///
    /// The main regulator provides a typical output voltage at 1.2 V.
    /// The system clock  frequency can be up to 64 MHz.
    /// The Flash memory access time for read access is minimum.
    /// Write and erase operations are possible.
    Range1 = 0b01,
    /// Low-power range
    ///
    /// The main regulator provides a typical output voltage at 1.0 V.
    /// The system clock frequency can be up to 16 MHz.
    /// The Flash memory access time for a read access is increased as compared
    /// to range 1.
    /// Write and erase operations are possible.
    #[allow(dead_code)]
    Range2 = 0b10,
}

impl From<Vos> for pac::pwr::cr1::VOS_A {
    fn from(vos: Vos) -> Self {
        match vos {
            Vos::Range1 => pac::pwr::cr1::VOS_A::V1_2,
            Vos::Range2 => pac::pwr::cr1::VOS_A::V1_0,
        }
    }
}

fn set_sysclk_from_msi_range(
    flash: &mut pac::FLASH,
    pwr: &mut pac::PWR,
    rcc: &mut pac::RCC,
    range: MsiRange,
    vos: Vos,
    _cs: &cortex_m::interrupt::CriticalSection,
) {
    const MSI_CALIBRATION: u8 = 0;

    pwr.cr1.modify(|_, w| w.vos().variant(vos.into()));

    let cfgr = rcc.cfgr.read();

    if !(cfgr.sws().is_msi() || cfgr.sws().is_pllr() && rcc.pllcfgr.read().pllsrc().is_msi()) {
        rcc.cfgr.modify(|_, w| w.sw().msi());
    }

    if range > MsiRange::from_rcc(rcc) {
        rcc_set_flash_latency_from_msi_range(flash, rcc, range, vos);
        rcc.cr
            .modify(|_, w| w.msirgsel().cr().msirange().variant(range.into()));
        rcc.icscr.modify(|_, w| w.msitrim().bits(MSI_CALIBRATION));
    } else {
        rcc.cr
            .modify(|_, w| w.msirgsel().cr().msirange().variant(range.into()));
        rcc.icscr.modify(|_, w| w.msitrim().bits(MSI_CALIBRATION));
        rcc_set_flash_latency_from_msi_range(flash, rcc, range, vos);
    }

    // HCLK1 configuration
    rcc.cfgr.modify(|_, w| w.hpre().div1());
    while rcc.cfgr.read().hpref().is_not_applied() {
        compiler_fence(SeqCst);
    }

    // HCLK3 configuration
    rcc.extcfgr.modify(|_, w| w.shdhpre().div1());
    while rcc.extcfgr.read().shdhpref().is_not_applied() {
        compiler_fence(SeqCst);
    }

    // PCLK1 configuration
    rcc.cfgr.modify(|_, w| w.ppre1().div1());
    while rcc.cfgr.read().ppre1f().is_not_applied() {
        compiler_fence(SeqCst);
    }

    // PCLK2 configuration
    rcc.cfgr.modify(|_, w| w.ppre2().div1());
    while rcc.cfgr.read().ppre2f().is_not_applied() {
        compiler_fence(SeqCst);
    }

    assert!(rcc.cr.read().msirdy().bit_is_set());
    rcc.cfgr.modify(|_, w| w.sw().msi());
    while !rcc.cfgr.read().sws().is_msi() {
        compiler_fence(SeqCst);
    }
}

/// Set the sysclk to the MSI source at 48MHz
///
/// This function is currently a hack.
/// In the future this should look more like other HALs:
/// * Return a clock structure
/// * Constration RCC
/// * Allow other frequencies to be set
pub fn set_sysclk_to_msi_48megahertz(
    flash: &mut pac::FLASH,
    pwr: &mut pac::PWR,
    rcc: &mut pac::RCC,
) {
    cortex_m::interrupt::free(|cs| {
        set_sysclk_from_msi_range(flash, pwr, rcc, MsiRange::Range11, Vos::Range1, cs)
    })
}

#[cfg_attr(feature = "stm32wl5x_cm0p", allow(dead_code))]
fn pllclk(rcc: &pac::RCC, pllcfgr: &pac::rcc::pllcfgr::R) -> Ratio<u32> {
    use pac::rcc::{
        cr::HSEPRE_A::{DIV1, DIV2},
        pllcfgr::PLLSRC_A as PLLSRC,
    };

    let src_freq: u32 = match pllcfgr.pllsrc().variant() {
        PLLSRC::NOCLOCK => 0,
        PLLSRC::MSI => MsiRange::from_rcc(rcc).as_hertz(),
        PLLSRC::HSI16 => 16_000_000,
        PLLSRC::HSE32 => match rcc.cr.read().hsepre().variant() {
            DIV1 => 32_000_000,
            DIV2 => 16_000_000,
        },
    };

    let pll_m: u32 = pllcfgr.pllm().bits().wrapping_add(1).into();
    let pll_n: u32 = pllcfgr.plln().bits().into();

    Ratio::new_raw(pll_n * src_freq, pll_m)
}

#[cfg_attr(feature = "stm32wl5x_cm0p", allow(dead_code))]
pub(crate) fn pllpclk(rcc: &pac::RCC, pllcfgr: &pac::rcc::pllcfgr::R) -> Ratio<u32> {
    let src: Ratio<u32> = pllclk(rcc, pllcfgr);
    let pll_p: u32 = pllcfgr.pllp().bits().wrapping_add(1).into();

    src / pll_p
}

pub(crate) fn sysclk(rcc: &pac::RCC, cfgr: &pac::rcc::cfgr::R) -> Ratio<u32> {
    use pac::rcc::{
        cfgr::SWS_A::{HSE32, HSI16, MSI, PLLR},
        cr::HSEPRE_A::{DIV1, DIV2},
        pllcfgr::PLLSRC_A as PLLSRC,
    };

    match cfgr.sws().variant() {
        MSI => Ratio::new_raw(MsiRange::from_rcc(rcc).as_hertz(), 1),
        HSI16 => Ratio::new_raw(16_000_000, 1),
        HSE32 => match rcc.cr.read().hsepre().variant() {
            DIV1 => Ratio::new_raw(32_000_000, 1),
            DIV2 => Ratio::new_raw(16_000_000, 1),
        },
        PLLR => {
            let pllcfgr = rcc.pllcfgr.read();
            let src_freq: u32 = match pllcfgr.pllsrc().variant() {
                // cannot be executing this code if there is no clock
                PLLSRC::NOCLOCK => unreachable!(),
                PLLSRC::MSI => MsiRange::from_rcc(rcc).as_hertz(),
                PLLSRC::HSI16 => 16_000_000,
                PLLSRC::HSE32 => match rcc.cr.read().hsepre().variant() {
                    DIV1 => 32_000_000,
                    DIV2 => 16_000_000,
                },
            };

            let pll_m: u32 = pllcfgr.pllm().bits().wrapping_add(1).into();
            let pll_n: u32 = pllcfgr.plln().bits().into();
            let pll_r: u32 = pllcfgr.pllr().bits().wrapping_add(1).into();

            // proof that this will not panic:
            //
            // pll_n is max 127, src_freq is max 32_000_000
            // max numer is 4_064_000_000 (less than u32::MAX)
            //
            // pll_m is max 8, pll_r is max 8
            // max denom is 64 (less than u32::MAX)
            //
            // pll_m and pll_r are both min 1 (denom cannot be zero)
            Ratio::new_raw(pll_n * src_freq, pll_m * pll_r)
        }
    }
}

/// Calculate the current system clock frequency in hertz
///
/// Fractional frequencies will be rounded down.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, rcc::sysclk_hz};
///
/// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
///
/// // without any initialization sysclk will be 4MHz
/// assert_eq!(sysclk_hz(&dp.RCC), 4_000_000);
/// ```
pub fn sysclk_hz(rcc: &pac::RCC) -> u32 {
    let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
    sysclk(rcc, &cfgr).to_integer()
}

fn hclk1(rcc: &pac::RCC, cfgr: &pac::rcc::cfgr::R) -> Ratio<u32> {
    let div: u32 = pre_div(cfgr.hpre().bits()).into();
    sysclk(rcc, cfgr) / div
}

/// Calculate the current hclk1 frequency in hertz
///
/// Fractional frequencies will be rounded down.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, rcc::hclk1_hz};
///
/// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
///
/// // without any initialization hclk1 will be 4MHz
/// assert_eq!(hclk1_hz(&dp.RCC), 4_000_000);
/// ```
pub fn hclk1_hz(rcc: &pac::RCC) -> u32 {
    let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
    hclk1(rcc, &cfgr).to_integer()
}

#[cfg(any(feature = "stm32wl5x_cm4", feature = "stm32wl5x_cm0p"))]
fn hclk2(rcc: &pac::RCC, cfgr: &pac::rcc::cfgr::R) -> Ratio<u32> {
    let div: u32 = pre_div(rcc.extcfgr.read().c2hpre().bits()).into();
    sysclk(rcc, cfgr) / div
}

/// Calculate the current hclk2 frequency in hertz
///
/// Fractional frequencies will be rounded down.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, rcc::hclk2_hz};
///
/// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
///
/// // without any initialization hclk2 will be 4MHz
/// assert_eq!(hclk2_hz(&dp.RCC), 4_000_000);
/// ```
#[cfg(any(feature = "stm32wl5x_cm4", feature = "stm32wl5x_cm0p"))]
#[cfg_attr(
    docsrs,
    doc(cfg(any(feature = "stm32wl5x_cm4", feature = "stm32wl5x_cm0p")))
)]
pub fn hclk2_hz(rcc: &pac::RCC) -> u32 {
    let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
    hclk2(rcc, &cfgr).to_integer()
}

fn hclk3(rcc: &pac::RCC, cfgr: &pac::rcc::cfgr::R) -> Ratio<u32> {
    let div: u32 = pre_div(rcc.extcfgr.read().shdhpre().bits()).into();
    sysclk(rcc, cfgr) / div
}

/// Calculate the current hclk3 frequency in hertz
///
/// Fractional frequencies will be rounded down.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, rcc::hclk3_hz};
///
/// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
///
/// // without any initialization hclk3 will be 4MHz
/// assert_eq!(hclk3_hz(&dp.RCC), 4_000_000);
/// ```
pub fn hclk3_hz(rcc: &pac::RCC) -> u32 {
    let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
    hclk3(rcc, &cfgr).to_integer()
}

fn cpu1_systick(rcc: &pac::RCC, cfgr: &pac::rcc::cfgr::R, src: SystClkSource) -> Ratio<u32> {
    let hclk1: Ratio<u32> = hclk1(rcc, cfgr);
    match src {
        SystClkSource::Core => hclk1,
        SystClkSource::External => hclk1 / 8,
    }
}

fn pclk1(rcc: &pac::RCC, cfgr: &pac::rcc::cfgr::R) -> Ratio<u32> {
    let div: u32 = ppre_div(cfgr.ppre1().bits()).into();
    hclk1(rcc, cfgr) / div
}

/// Calculate the current PCLK1 frequency in hertz
///
/// Fractional frequencies will be rounded down.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, rcc::pclk1_hz};
///
/// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
///
/// // without any initialization pclk1 will be 4MHz
/// assert_eq!(pclk1_hz(&dp.RCC), 4_000_000);
/// ```
pub fn pclk1_hz(rcc: &pac::RCC) -> u32 {
    let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
    pclk1(rcc, &cfgr).to_integer()
}

fn pclk2(rcc: &pac::RCC, cfgr: &pac::rcc::cfgr::R) -> Ratio<u32> {
    let div: u32 = ppre_div(cfgr.ppre2().bits()).into();
    hclk1(rcc, cfgr) / div
}

/// Calculate the current PCLK2 frequency in hertz
///
/// Fractional frequencies will be rounded down.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, rcc::pclk2_hz};
///
/// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
///
/// // without any initialization pclk2 will be 4MHz
/// assert_eq!(pclk2_hz(&dp.RCC), 4_000_000);
/// ```
pub fn pclk2_hz(rcc: &pac::RCC) -> u32 {
    let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
    pclk2(rcc, &cfgr).to_integer()
}

/// Calculate the current CPU1 systick frequency in hertz
///
/// Fractional frequencies will be rounded down.
///
/// # Example
///
/// Created a systick based delay structure.
///
/// ```no_run
/// use stm32wl_hal::{
///     cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
///     pac,
///     rcc::cpu1_systick_hz,
/// };
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
///
/// // Delay constructor will set the clock source to core
/// // note: this code is only valid if running on CPU1
/// //       cpu_systick_hz is better for this use-case
/// let mut delay: Delay = Delay::new(cp.SYST, cpu1_systick_hz(&dp.RCC, SystClkSource::Core));
/// delay.delay_ms(100);
/// ```
pub fn cpu1_systick_hz(rcc: &pac::RCC, src: SystClkSource) -> u32 {
    let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
    cpu1_systick(rcc, &cfgr, src).to_integer()
}

#[cfg(any(feature = "stm32wl5x_cm4", feature = "stm32wl5x_cm0p"))]
fn cpu2_systick(rcc: &pac::RCC, cfgr: pac::rcc::cfgr::R, src: SystClkSource) -> Ratio<u32> {
    let hclk2: Ratio<u32> = hclk2(rcc, &cfgr);
    match src {
        SystClkSource::Core => hclk2,
        SystClkSource::External => hclk2 / 8,
    }
}

/// Calculate the current CPU2 systick frequency in hertz
///
/// Fractional frequencies will be rounded down.
///
/// # Example
///
/// Created a systick based delay structure.
///
/// ```no_run
/// use stm32wl_hal::{
///     cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
///     pac,
///     rcc::cpu2_systick_hz,
/// };
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
///
/// // Delay constructor will set the clock source to core
/// // note: this code is only valid if running on CPU2
/// //       cpu_systick_hz is better for this use-case
/// let mut delay: Delay = Delay::new(cp.SYST, cpu2_systick_hz(&dp.RCC, SystClkSource::Core));
/// delay.delay_ms(100);
/// ```
#[cfg(any(feature = "stm32wl5x_cm4", feature = "stm32wl5x_cm0p"))]
#[cfg_attr(
    docsrs,
    doc(cfg(any(feature = "stm32wl5x_cm4", feature = "stm32wl5x_cm0p")))
)]
pub fn cpu2_systick_hz(rcc: &pac::RCC, src: SystClkSource) -> u32 {
    let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
    cpu2_systick(rcc, cfgr, src).to_integer()
}

/// Calculate the current CPU systick frequency in hertz.
///
/// This will automatically select the correct CPU based on the feature
/// flag passed to the HAL.
///
/// # Example
///
/// Created a systick based delay structure.
///
/// ```no_run
/// use stm32wl_hal::{
///     cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
///     pac,
///     rcc::cpu_systick_hz,
/// };
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();
///
/// // Delay constructor will set the clock source to core
/// let mut delay: Delay = Delay::new(cp.SYST, cpu_systick_hz(&dp.RCC, SystClkSource::Core));
/// delay.delay_ms(100);
/// ```
pub fn cpu_systick_hz(rcc: &pac::RCC, src: SystClkSource) -> u32 {
    #[cfg(feature = "stm32wl5x_cm0p")]
    {
        cpu2_systick_hz(rcc, src)
    }

    #[cfg(not(feature = "stm32wl5x_cm0p"))]
    {
        cpu1_systick_hz(rcc, src)
    }
}

/// Calculate the LSI clock frequency in hertz.
///
/// The LSI is either 32 kHz without the LSI prescaler or 128 Hz with the LSI
/// prescaler.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::rcc::lsi_hz;
///
/// // LSI is not divided at power on
/// assert_eq!(lsi_hz(), 32_000);
/// ```
pub fn lsi_hz() -> u16 {
    use pac::rcc::csr::LSIPRE_A::{DIV1, DIV128};
    const LSI_BASE_HZ: u16 = 32_000;
    const LSI_DIV_HZ: u16 = 32_000 / 128;

    // safety: volatile read with no side effects to an always-on domain
    match unsafe { pac::Peripherals::steal() }
        .RCC
        .csr
        .read()
        .lsipre()
        .variant()
    {
        DIV1 => LSI_BASE_HZ,
        DIV128 => LSI_DIV_HZ,
    }
}
