//! Reset and clocking control
//!
//! Unlike other HALs clocks do not get frozen.
//! A lot of applications for this chip will require low-power considerations,
//! and there are many scenarios where you will want to adjust the clocks.

use crate::{pac, Ratio};
use core::convert::{TryFrom, TryInto};
use cortex_m::{interrupt::CriticalSection, peripheral::syst::SystClkSource};

pub use pac::rcc::csr::LSIPRE_A as LsiPre;

fn hclk3_prescaler_div(rcc: &pac::RCC) -> u16 {
    pre_div(rcc.extcfgr.read().shdhpre().bits())
}

fn set_flash_latency(flash: &pac::FLASH, rcc: &pac::RCC, target_sysclk_hz: u32, vos: Vos) {
    let div: u32 = u32::from(hclk3_prescaler_div(rcc));
    let flash_clk_src_freq: u32 = target_sysclk_hz / div;

    let latency: FlashLatency = FlashLatency::from_hertz(vos, flash_clk_src_freq);

    flash.acr.modify(|_, w| w.latency().variant(latency.into()));

    while flash.acr.read().latency().bits() != (latency as u8) {}
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
            Vos::V1_2 => match hz {
                0..=18_000_000 => FlashLatency::Zero,
                18_000_001..=36_000_000 => FlashLatency::One,
                _ => FlashLatency::Two,
            },
            Vos::V1_0 => match hz {
                0..=6_000_000 => FlashLatency::Zero,
                6_000_001..=12_000_000 => FlashLatency::One,
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

/// MSI clock ranges.
// developers: this exists because the MSI range appears in two different registers.
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MsiRange {
    /// Range 0 around 100 kHz.
    Range100k = 0b0000,
    /// Range 1 around 200 kHz.
    Range200k = 0b0001,
    /// Range 2 around 400 kHz.
    Range400k = 0b0010,
    /// Range 3 around 800 kHz.
    Range800k = 0b0011,
    /// Range 4 around 1 MHz.
    Range1M = 0b0100,
    /// Range 5 around 2 MHz.
    Range2M = 0b0101,
    /// Range 6 around 4 MHz.
    Range4M = 0b0110,
    /// Range 7 around 8 MHz.
    Range8M = 0b0111,
    /// Range 8 around 16 MHz.
    Range16M = 0b1000,
    /// Range 9 around 24 MHz.
    Range24M = 0b1001,
    /// Range 10 around 32 MHz.
    Range32M = 0b1010,
    /// Range 11 around 48 MHz.
    Range48M = 0b1011,
}

impl MsiRange {
    /// Get the frequency for an MSI range in hertz.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::rcc::MsiRange;
    ///
    /// assert_eq!(MsiRange::Range100k, 100_000);
    /// assert_eq!(MsiRange::Range200k, 200_000);
    /// assert_eq!(MsiRange::Range400k, 400_000);
    /// assert_eq!(MsiRange::Range800k, 800_000);
    /// assert_eq!(MsiRange::Range1M, 1_000_000);
    /// assert_eq!(MsiRange::Range2M, 2_000_000);
    /// assert_eq!(MsiRange::Range4M, 4_000_000);
    /// assert_eq!(MsiRange::Range8M, 8_000_000);
    /// assert_eq!(MsiRange::Range16M, 16_000_000);
    /// assert_eq!(MsiRange::Range24M, 24_000_000);
    /// assert_eq!(MsiRange::Range32M, 32_000_000);
    /// assert_eq!(MsiRange::Range48M, 48_000_000);
    /// ```
    pub const fn to_hz(&self) -> u32 {
        match self {
            MsiRange::Range100k => 100_000,
            MsiRange::Range200k => 200_000,
            MsiRange::Range400k => 400_000,
            MsiRange::Range800k => 800_000,
            MsiRange::Range1M => 1_000_000,
            MsiRange::Range2M => 2_000_000,
            MsiRange::Range4M => 4_000_000,
            MsiRange::Range8M => 8_000_000,
            MsiRange::Range16M => 16_000_000,
            MsiRange::Range24M => 24_000_000,
            MsiRange::Range32M => 32_000_000,
            MsiRange::Range48M => 48_000_000,
        }
    }

    /// Get the current MSI clock range from hardware registers.
    ///
    /// The unwraps in this function are desired because the other values are
    /// reserved.
    fn from_rcc(rcc: &pac::RCC) -> MsiRange {
        use pac::rcc::cr::MSIRGSEL_A::{CR, CSR};

        let cr = rcc.cr.read();
        match cr.msirgsel().variant() {
            CSR => unwrap!(rcc.csr.read().msisrange().bits().try_into()),
            CR => unwrap!(cr.msirange().bits().try_into()),
        }
    }

    fn vos(&self) -> Vos {
        if self > &MsiRange::Range16M {
            Vos::V1_2
        } else {
            Vos::V1_0
        }
    }
}

impl Default for MsiRange {
    fn default() -> Self {
        MsiRange::Range4M
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
            0b0000 => Ok(MsiRange::Range100k),
            0b0001 => Ok(MsiRange::Range200k),
            0b0010 => Ok(MsiRange::Range400k),
            0b0011 => Ok(MsiRange::Range800k),
            0b0100 => Ok(MsiRange::Range1M),
            0b0101 => Ok(MsiRange::Range2M),
            0b0110 => Ok(MsiRange::Range4M),
            0b0111 => Ok(MsiRange::Range8M),
            0b1000 => Ok(MsiRange::Range16M),
            0b1001 => Ok(MsiRange::Range24M),
            0b1010 => Ok(MsiRange::Range32M),
            0b1011 => Ok(MsiRange::Range48M),
            _ => Err(x),
        }
    }
}

impl From<MsiRange> for pac::rcc::cr::MSIRANGE_A {
    fn from(mr: MsiRange) -> Self {
        match mr {
            MsiRange::Range100k => pac::rcc::cr::MSIRANGE_A::RANGE100K,
            MsiRange::Range200k => pac::rcc::cr::MSIRANGE_A::RANGE200K,
            MsiRange::Range400k => pac::rcc::cr::MSIRANGE_A::RANGE400K,
            MsiRange::Range800k => pac::rcc::cr::MSIRANGE_A::RANGE800K,
            MsiRange::Range1M => pac::rcc::cr::MSIRANGE_A::RANGE1M,
            MsiRange::Range2M => pac::rcc::cr::MSIRANGE_A::RANGE2M,
            MsiRange::Range4M => pac::rcc::cr::MSIRANGE_A::RANGE4M,
            MsiRange::Range8M => pac::rcc::cr::MSIRANGE_A::RANGE8M,
            MsiRange::Range16M => pac::rcc::cr::MSIRANGE_A::RANGE16M,
            MsiRange::Range24M => pac::rcc::cr::MSIRANGE_A::RANGE24M,
            MsiRange::Range32M => pac::rcc::cr::MSIRANGE_A::RANGE32M,
            MsiRange::Range48M => pac::rcc::cr::MSIRANGE_A::RANGE48M,
        }
    }
}

/// Voltage scaling
///
/// See RM0453 rev 1 section 6.1.4 dynamic voltage scaling management
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Vos {
    /// High-performance range (range 1)
    ///
    /// * The main regulator provides a typical output voltage at 1.2 V.
    /// * The system clock frequency can be up to 64 MHz.
    /// * The Flash memory access time for read access is minimum.
    /// * Write and erase operations are possible.
    V1_2 = 0b01,
    /// Low-power range (range 2)
    ///
    /// * The main regulator provides a typical output voltage at 1.0 V.
    /// * The system clock frequency can be up to 16 MHz.
    /// * The Flash memory access time for a read access is increased as
    ///   compared to range 1.
    /// * Write and erase operations are possible.
    V1_0 = 0b10,
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

/// Set the sysclk to use the HSE 32MHz clock.
///
/// The VOS argument selects the voltage range which determines the clock
/// prescaler:
/// * 1.2V: 32MHz (div 1)
/// * 1.0V: 16MHz (div 2)
///
/// # Safety
///
/// 1. Ensure peripherals are not in-use before calling this function.
/// 2. Ensure peripherals have their clocks adjusted correctly for the new
///    sysclk frequency after calling this function.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{
///     pac,
///     rcc::{set_sysclk_hse, Vos},
/// };
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// cortex_m::interrupt::free(|cs| unsafe {
///     set_sysclk_hse(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, Vos::V1_2, cs)
/// });
/// ```
#[allow(unused_variables)]
pub unsafe fn set_sysclk_hse(
    flash: &mut pac::FLASH,
    pwr: &mut pac::PWR,
    rcc: &mut pac::RCC,
    vos: Vos,
    cs: &CriticalSection,
) {
    use pac::rcc::cr::HSEPRE_A;
    let (pre, target_sysclk_hz): (HSEPRE_A, u32) = match vos {
        Vos::V1_2 => (HSEPRE_A::DIV1, 32_000_000),
        Vos::V1_0 => (HSEPRE_A::DIV2, 16_000_000),
    };

    // increase VOS range
    if vos == Vos::V1_2 {
        pwr.cr1.modify(|_, w| w.vos().v1_2());
        while pwr.sr2.read().vosf().is_change() {}
    }

    // setting HSEBYPWR is only valid when HSE is off
    // assume the user knows what they are doing in this scenario
    if rcc.cr.read().hseon().is_disabled() {
        rcc.cr.modify(|_, w| {
            w.hseon()
                .enabled()
                .hsepre()
                .variant(pre)
                .hsebyppwr()
                .vddtcxo()
        });
    } else {
        rcc.cr
            .modify(|_, w| w.hseon().enabled().hsepre().variant(pre));
    }

    while rcc.cr.read().hserdy().is_not_ready() {}

    let current_sysclk_hz: u32 = sysclk_hz(rcc);
    if target_sysclk_hz > current_sysclk_hz {
        // freq increase, set new flash latency first
        set_flash_latency(flash, rcc, target_sysclk_hz, vos);
        rcc.cfgr.modify(|_, w| w.sw().hse32());
        while !rcc.cfgr.read().sws().is_hse32() {}
    } else {
        // freq decrease, set new flash latency last
        rcc.cfgr.modify(|_, w| w.sw().hse32());
        while !rcc.cfgr.read().sws().is_hse32() {}
        set_flash_latency(flash, rcc, target_sysclk_hz, vos);
    }

    // decrease VOS range
    if vos == Vos::V1_0 {
        pwr.cr1.modify(|_, w| w.vos().v1_0());
    }
}

/// Set the sysclk to use the HSI 16MHz clock.
///
/// # Safety
///
/// 1. Ensure peripherals are not in-use before calling this function.
/// 2. Ensure peripherals have their clocks adjusted correctly for the new
///    sysclk frequency after calling this function.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, rcc::set_sysclk_hsi};
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// cortex_m::interrupt::free(|cs| unsafe {
///     set_sysclk_hsi(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs)
/// });
/// ```
#[allow(unused_variables)]
pub unsafe fn set_sysclk_hsi(
    flash: &mut pac::FLASH,
    pwr: &mut pac::PWR,
    rcc: &mut pac::RCC,
    cs: &CriticalSection,
) {
    rcc.cr.modify(|_, w| w.hsion().enabled());
    while rcc.cr.read().hsirdy().is_not_ready() {}

    let current_sysclk_hz: u32 = sysclk_hz(rcc);
    const TARGET_SYSCLK_HZ: u32 = 16_000_000;

    if TARGET_SYSCLK_HZ > current_sysclk_hz {
        // freq increase, set new flash latency first
        set_flash_latency(flash, rcc, TARGET_SYSCLK_HZ, Vos::V1_0);
        rcc.cfgr.modify(|_, w| w.sw().hsi16());
        while !rcc.cfgr.read().sws().is_hsi16() {}
    } else {
        // freq decrease, set new flash latency last
        rcc.cfgr.modify(|_, w| w.sw().hsi16());
        while !rcc.cfgr.read().sws().is_hsi16() {}
        set_flash_latency(flash, rcc, TARGET_SYSCLK_HZ, Vos::V1_0);
    }

    // decrease VOS range
    pwr.cr1.modify(|_, w| w.vos().v1_0());
}

/// Set the sysclk from an MSI range.
///
/// # Safety
///
/// 1. Ensure peripherals are not in-use before calling this function.
/// 2. Ensure peripherals have their clocks adjusted correctly for the new
///    sysclk frequency after calling this function.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{
///     pac,
///     rcc::{set_sysclk_msi, MsiRange},
/// };
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// cortex_m::interrupt::free(|cs| unsafe {
///     set_sysclk_msi(
///         &mut dp.FLASH,
///         &mut dp.PWR,
///         &mut dp.RCC,
///         MsiRange::Range48M,
///         cs,
///     )
/// });
/// ```
pub unsafe fn set_sysclk_msi(
    flash: &mut pac::FLASH,
    pwr: &mut pac::PWR,
    rcc: &mut pac::RCC,
    range: MsiRange,
    cs: &CriticalSection,
) {
    // startup the MSI clock
    rcc.cr.modify(|_, w| w.msion().enabled());

    let vos: Vos = range.vos();

    // increase VOS range
    if vos == Vos::V1_2 {
        pwr.cr1.modify(|_, w| w.vos().v1_2());
        while pwr.sr2.read().vosf().is_change() {}
    }

    let cfgr = rcc.cfgr.read();

    // ES0500 Rev 3 erratum handling:
    //
    // A voltage drop to 1.08 V may occur on the 1.2 V regulated supply when the
    // MSI frequency is changed as follows:
    // * from MSI at 400 kHz to MSI at 24 MHz and above
    // * from MSI at 1 MHZ to MSI at 48 MHz
    // As a result, the voltage drop may cause CPU HardFault.
    // To ensure there is no impact on the 1.2 V supply, introduce an
    // intermediate MSI frequency
    //
    // Open question:
    // Does this apply when the CPU is clocked by the PLL via MSI?
    if cfgr.sws().is_msi() {
        let current_range: MsiRange = MsiRange::from_rcc(rcc);

        if ((current_range == MsiRange::Range400k) && (range >= MsiRange::Range24M))
            || ((current_range == MsiRange::Range1M) && (range == MsiRange::Range48M))
        {
            set_sysclk_msi_inner(flash, rcc, MsiRange::Range16M, vos, cs)
        }
    }

    set_sysclk_msi_inner(flash, rcc, range, vos, cs);

    // decrease VOS range
    if vos == Vos::V1_0 {
        pwr.cr1.modify(|_, w| w.vos().v1_0());
    }
}

unsafe fn set_sysclk_msi_inner(
    flash: &mut pac::FLASH,
    rcc: &mut pac::RCC,
    range: MsiRange,
    vos: Vos,
    _cs: &CriticalSection,
) {
    // MSI was enabled by the caller, wait for it to be ready
    // MSIRGSEL can only be set when MSI is ready (or off)
    while rcc.cr.read().msirdy().is_not_ready() {}

    let current_sysclk_hz: u32 = sysclk_hz(rcc);
    let target_sysclk_hz: u32 = range.to_hz();

    if target_sysclk_hz > current_sysclk_hz {
        // freq increase, set new flash latency first
        set_flash_latency(flash, rcc, target_sysclk_hz, vos);
        rcc.cr
            .modify(|_, w| w.msirgsel().cr().msirange().variant(range.into()));
        rcc.icscr.modify(|_, w| w.msitrim().bits(0));

        // switch to MSI clock source
        rcc.cfgr.modify(|_, w| w.sw().msi());
        while !rcc.cfgr.read().sws().is_msi() {}
    } else {
        // freq decrease, set new flash latency last
        // defmt::info!("rcc.cr write: {:#X}", rcc.cr.read().bits());
        // cortex_m::asm::delay(range.to_hz());
        rcc.cr.modify(|_, w| w.msirange().variant(range.into()));
        // defmt::info!("rcc.cr write: {:#X}", rcc.cr.read().bits());
        // cortex_m::asm::delay(range.to_hz());
        rcc.cr.modify(|_, w| w.msirgsel().cr());
        // defmt::info!("msitrim");
        // cortex_m::asm::delay(range.to_hz());
        rcc.icscr.modify(|_, w| w.msitrim().bits(0));

        // switch to MSI clock source
        rcc.cfgr.modify(|_, w| w.sw().msi());
        while !rcc.cfgr.read().sws().is_msi() {}

        set_flash_latency(flash, rcc, target_sysclk_hz, vos);
    }
}

/// Set the sysclk to the MSI source at 48MHz.
///
/// This is a convenience function that wraps [`set_sysclk_msi`]
/// to set the system clock to the highest frequency.
///
/// # Safety
///
/// 1. Ensure peripherals are not in-use before calling this function.
/// 2. Ensure peripherals have their clocks adjusted correctly for the new
///    sysclk frequency after calling this function.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, rcc::set_sysclk_msi_max};
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// unsafe { set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC) };
/// ```
pub unsafe fn set_sysclk_msi_max(flash: &mut pac::FLASH, pwr: &mut pac::PWR, rcc: &mut pac::RCC) {
    cortex_m::interrupt::free(|cs| set_sysclk_msi(flash, pwr, rcc, MsiRange::Range48M, cs))
}

#[cfg_attr(feature = "stm32wl5x_cm0p", allow(dead_code))]
fn pllclk(rcc: &pac::RCC, pllcfgr: &pac::rcc::pllcfgr::R) -> Ratio<u32> {
    use pac::rcc::{
        cr::HSEPRE_A::{DIV1, DIV2},
        pllcfgr::PLLSRC_A as PLLSRC,
    };

    let src_freq: u32 = match pllcfgr.pllsrc().variant() {
        PLLSRC::NOCLOCK => 0,
        PLLSRC::MSI => MsiRange::from_rcc(rcc).to_hz(),
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
        MSI => Ratio::new_raw(MsiRange::from_rcc(rcc).to_hz(), 1),
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
                PLLSRC::MSI => MsiRange::from_rcc(rcc).to_hz(),
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

pub(crate) fn hclk3(rcc: &pac::RCC, cfgr: &pac::rcc::cfgr::R) -> Ratio<u32> {
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

pub(crate) fn apb1timx(rcc: &pac::RCC) -> Ratio<u32> {
    let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
    // * If the APB prescaler (PPREx) selects the PCLKx clock to be HCLK1 not divided,
    //   the timer clock frequencies are set to the HCLK1 frequency (timer clock = HCLK1).
    // * If the APB prescaler (PPREx) selects the PCLKx clock to be HCLK1 divided by n,
    //   the timer clock frequencies are set to HCLK1 divided by (n / 2) (timer clock = 2 x PCLKx).
    let div: u32 = match cfgr.ppre1().bits() {
        0b101 => 2, // 4 / 2
        0b110 => 4, // 8 / 2
        0b111 => 8, // 16 / 2
        _ => 1,     // 2 / 2 and all others
    };
    hclk1(rcc, &cfgr) / div
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

pub(crate) fn pclk2(rcc: &pac::RCC, cfgr: &pac::rcc::cfgr::R) -> Ratio<u32> {
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
/// The LSI is either 32 kHz without the LSI prescaler or 250 Hz with the LSI
/// prescaler.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, rcc::lsi_hz};
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
///
/// // LSI is not divided at power on
/// assert_eq!(lsi_hz(&dp.RCC), 32_000);
/// ```
#[inline]
pub fn lsi_hz(rcc: &pac::RCC) -> u16 {
    use pac::rcc::csr::LSIPRE_A::{DIV1, DIV128};
    const LSI_BASE_HZ: u16 = 32_000;
    const LSI_DIV_HZ: u16 = 32_000 / 128;

    // safety: volatile read with no side effects to an always-on domain
    match rcc.csr.read().lsipre().variant() {
        DIV1 => LSI_BASE_HZ,
        DIV128 => LSI_DIV_HZ,
    }
}

/// Setup the LSI clock and wait for completion.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{
///     pac,
///     rcc::{setup_lsi, LsiPre},
/// };
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// setup_lsi(&mut dp.RCC, LsiPre::DIV1);
/// ```
#[inline]
pub fn setup_lsi(rcc: &mut pac::RCC, pre: LsiPre) {
    let csr = rcc.csr.read();
    // LSI pre-scaler is applied after an on-off cycle (if a change is required)
    if csr.lsion().is_on() && csr.lsipre().variant() != pre {
        rcc.csr.modify(|_, w| w.lsipre().variant(pre).lsion().off());
    }
    enable_lsi(rcc)
}

/// Enable the LSI clock with the currently configured pre-scaler and wait
/// for completion.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, rcc::enable_lsi};
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// enable_lsi(&mut dp.RCC);
/// ```
#[inline]
pub fn enable_lsi(rcc: &mut pac::RCC) {
    rcc.csr.modify(|_, w| w.lsion().on());
    while rcc.csr.read().lsirdy().is_not_ready() {}
}

/// Reset the backup domain.
///
/// # Safety
///
/// 1. This will disable the LSE clock.
///    Ensure no peripherals are using the LSE clock before calling this function.
/// 2. This will reset the real-time clock.
///    Setup the RTC after calling this function.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::{pac, rcc::pulse_reset_backup_domain};
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// unsafe { pulse_reset_backup_domain(&mut dp.RCC, &mut dp.PWR) };
/// ```
#[inline]
pub unsafe fn pulse_reset_backup_domain(rcc: &mut pac::RCC, pwr: &mut pac::PWR) {
    pwr.cr1.modify(|_, w| w.dbp().enabled());
    rcc.bdcr.modify(|_, w| w.bdrst().set_bit());
    rcc.bdcr.modify(|_, w| w.bdrst().clear_bit());
}
