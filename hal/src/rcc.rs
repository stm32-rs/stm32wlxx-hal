//! Reset and clocking control
//!
//! Unlike other HALs clocks do not get frozen.
//! A lot of applications for this chip will require low-power,
//! and there are many scenarios where you will want to adjust the clocks.
//!
//! Quickstart: [`set_sysclk_msi_max`]

use crate::{pac, Ratio};
use cortex_m::{interrupt::CriticalSection, peripheral::syst::SystClkSource};

use pac::flash::acr::LATENCY_A;
pub use pac::rcc::bdcr::LSCOSEL_A as LscoSel;
pub use pac::rcc::csr::LSIPRE_A as LsiPre;

fn hclk3_prescaler_div(rcc: &pac::RCC) -> u16 {
    pre_div(rcc.extcfgr.read().shdhpre().bits())
}

fn set_flash_latency(flash: &pac::FLASH, rcc: &pac::RCC, target_sysclk_hz: u32, vos: Vos) {
    let div: u32 = u32::from(hclk3_prescaler_div(rcc));
    let flash_clk_src_freq: u32 = target_sysclk_hz / div;

    let ws: LATENCY_A = match vos {
        Vos::V1_2 => match flash_clk_src_freq {
            0..=18_000_000 => LATENCY_A::Ws0,
            18_000_001..=36_000_000 => LATENCY_A::Ws1,
            _ => LATENCY_A::Ws2,
        },
        Vos::V1_0 => match flash_clk_src_freq {
            0..=6_000_000 => LATENCY_A::Ws0,
            6_000_001..=12_000_000 => LATENCY_A::Ws1,
            _ => LATENCY_A::Ws2,
        },
    };

    flash.acr.modify(|_, w| w.latency().variant(ws));

    while flash.acr.read().latency().variant() != Some(ws) {}
}

/// MSI clock ranges.
// developers: this exists because the MSI range appears in two different registers.
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
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
    /// use stm32wlxx_hal::rcc::MsiRange;
    ///
    /// assert_eq!(MsiRange::Range100k.to_hz(), 100_000);
    /// assert_eq!(MsiRange::Range200k.to_hz(), 200_000);
    /// assert_eq!(MsiRange::Range400k.to_hz(), 400_000);
    /// assert_eq!(MsiRange::Range800k.to_hz(), 800_000);
    /// assert_eq!(MsiRange::Range1M.to_hz(), 1_000_000);
    /// assert_eq!(MsiRange::Range2M.to_hz(), 2_000_000);
    /// assert_eq!(MsiRange::Range4M.to_hz(), 4_000_000);
    /// assert_eq!(MsiRange::Range8M.to_hz(), 8_000_000);
    /// assert_eq!(MsiRange::Range16M.to_hz(), 16_000_000);
    /// assert_eq!(MsiRange::Range24M.to_hz(), 24_000_000);
    /// assert_eq!(MsiRange::Range32M.to_hz(), 32_000_000);
    /// assert_eq!(MsiRange::Range48M.to_hz(), 48_000_000);
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
    /// impossible to set in hardware.
    fn from_rcc(rcc: &pac::RCC) -> MsiRange {
        use pac::rcc::cr::MSIRGSEL_A::{Cr, Csr};

        let cr = rcc.cr.read();
        match cr.msirgsel().variant() {
            Csr => unwrap!(rcc.csr.read().msisrange().bits().try_into()),
            Cr => unwrap!(cr.msirange().bits().try_into()),
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
            MsiRange::Range100k => pac::rcc::cr::MSIRANGE_A::Range100k,
            MsiRange::Range200k => pac::rcc::cr::MSIRANGE_A::Range200k,
            MsiRange::Range400k => pac::rcc::cr::MSIRANGE_A::Range400k,
            MsiRange::Range800k => pac::rcc::cr::MSIRANGE_A::Range800k,
            MsiRange::Range1M => pac::rcc::cr::MSIRANGE_A::Range1m,
            MsiRange::Range2M => pac::rcc::cr::MSIRANGE_A::Range2m,
            MsiRange::Range4M => pac::rcc::cr::MSIRANGE_A::Range4m,
            MsiRange::Range8M => pac::rcc::cr::MSIRANGE_A::Range8m,
            MsiRange::Range16M => pac::rcc::cr::MSIRANGE_A::Range16m,
            MsiRange::Range24M => pac::rcc::cr::MSIRANGE_A::Range24m,
            MsiRange::Range32M => pac::rcc::cr::MSIRANGE_A::Range32m,
            MsiRange::Range48M => pac::rcc::cr::MSIRANGE_A::Range48m,
        }
    }
}

/// Voltage scaling
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
/// 1. Peripherals must not be in-use before calling this function.
/// 2. Peripherals may need their prescalers adjusted for the new sysclk frequency.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{
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
        Vos::V1_2 => (HSEPRE_A::Div1, 32_000_000),
        Vos::V1_0 => (HSEPRE_A::Div2, 16_000_000),
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
/// 1. Peripherals must not be in-use before calling this function.
/// 2. Peripherals may need their prescalers adjusted for the new sysclk frequency.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{pac, rcc::set_sysclk_hsi};
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
/// 1. Peripherals must not be in-use before calling this function.
/// 2. Peripherals may need their prescalers adjusted for the new sysclk frequency.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{
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
/// 1. Peripherals must not be in-use before calling this function.
/// 2. Peripherals may need their prescalers adjusted for the new sysclk frequency.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{pac, rcc::set_sysclk_msi_max};
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// cortex_m::interrupt::free(|cs| unsafe {
///     set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs)
/// });
/// ```
pub unsafe fn set_sysclk_msi_max(
    flash: &mut pac::FLASH,
    pwr: &mut pac::PWR,
    rcc: &mut pac::RCC,
    cs: &CriticalSection,
) {
    set_sysclk_msi(flash, pwr, rcc, MsiRange::Range48M, cs)
}

#[cfg_attr(feature = "stm32wl5x_cm0p", allow(dead_code))]
fn pllclk(rcc: &pac::RCC, pllcfgr: &pac::rcc::pllcfgr::R) -> Ratio<u32> {
    use pac::rcc::{
        cr::HSEPRE_A::{Div1, Div2},
        pllcfgr::PLLSRC_A as PLLSRC,
    };

    let src_freq: u32 = match pllcfgr.pllsrc().variant() {
        PLLSRC::NoClock => 0,
        PLLSRC::Msi => MsiRange::from_rcc(rcc).to_hz(),
        PLLSRC::Hsi16 => 16_000_000,
        PLLSRC::Hse32 => match rcc.cr.read().hsepre().variant() {
            Div1 => 32_000_000,
            Div2 => 16_000_000,
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
        cfgr::SWS_A::{Hse32, Hsi16, Msi, Pllr},
        cr::HSEPRE_A::{Div1, Div2},
        pllcfgr::PLLSRC_A as PLLSRC,
    };

    match cfgr.sws().variant() {
        Msi => Ratio::new_raw(MsiRange::from_rcc(rcc).to_hz(), 1),
        Hsi16 => Ratio::new_raw(16_000_000, 1),
        Hse32 => match rcc.cr.read().hsepre().variant() {
            Div1 => Ratio::new_raw(32_000_000, 1),
            Div2 => Ratio::new_raw(16_000_000, 1),
        },
        Pllr => {
            let pllcfgr = rcc.pllcfgr.read();
            let src_freq: u32 = match pllcfgr.pllsrc().variant() {
                // cannot be executing this code if there is no clock
                PLLSRC::NoClock => unreachable!(),
                PLLSRC::Msi => MsiRange::from_rcc(rcc).to_hz(),
                PLLSRC::Hsi16 => 16_000_000,
                PLLSRC::Hse32 => match rcc.cr.read().hsepre().variant() {
                    Div1 => 32_000_000,
                    Div2 => 16_000_000,
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
/// use stm32wlxx_hal::{pac, rcc::sysclk_hz};
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
/// use stm32wlxx_hal::{pac, rcc::hclk1_hz};
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
/// use stm32wlxx_hal::{pac, rcc::hclk2_hz};
///
/// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
///
/// // without any initialization hclk2 will be 4MHz
/// assert_eq!(hclk2_hz(&dp.RCC), 4_000_000);
/// ```
#[cfg(any(feature = "stm32wl5x_cm4", feature = "stm32wl5x_cm0p"))]
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
/// use stm32wlxx_hal::{pac, rcc::hclk3_hz};
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

pub(crate) fn pclk1(rcc: &pac::RCC, cfgr: &pac::rcc::cfgr::R) -> Ratio<u32> {
    let div: u32 = ppre_div(cfgr.ppre1().bits()).into();
    hclk1(rcc, cfgr) / div
}

pub(crate) fn apb1timx(rcc: &pac::RCC) -> Ratio<u32> {
    let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
    // * If the APB prescaler (PPREx) selects the PCLKx clock to be HCLK1 not divided,
    //   the timer clock frequencies are set to the HCLK1 frequency (timer clock = HCLK1).
    // * If the APB prescaler (PPREx) selects the PCLKx clock to be HCLK1 divided by n,
    //   the timer clock frequencies are set to HCLK1 divided by (n / 2) (timer clock = 2 × PCLKx).
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
/// use stm32wlxx_hal::{pac, rcc::pclk1_hz};
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
/// use stm32wlxx_hal::{pac, rcc::pclk2_hz};
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
/// use stm32wlxx_hal::{
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
/// use stm32wlxx_hal::{
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
/// use stm32wlxx_hal::{
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
/// use stm32wlxx_hal::{pac, rcc::lsi_hz};
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
///
/// // LSI is not divided at power on
/// assert_eq!(lsi_hz(&dp.RCC), 32_000);
/// ```
#[inline]
pub fn lsi_hz(rcc: &pac::RCC) -> u16 {
    use pac::rcc::csr::LSIPRE_A::{Div1, Div128};
    const LSI_BASE_HZ: u16 = 32_000;
    const LSI_DIV_HZ: u16 = 32_000 / 128;

    // safety: volatile read with no side effects to an always-on domain
    match rcc.csr.read().lsipre().variant() {
        Div1 => LSI_BASE_HZ,
        Div128 => LSI_DIV_HZ,
    }
}

/// Setup the LSI clock and wait for completion.
///
/// This will temporarily disable the LSI clock if the prescaler needs to be
/// changed.
///
/// # Safety
///
/// 1. Ensure there are no peripherals using the LSI clock source before calling
///    this function.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{
///     pac,
///     rcc::{setup_lsi, LsiPre},
/// };
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
/// unsafe { setup_lsi(&mut dp.RCC, LsiPre::Div1) };
/// ```
#[inline]
pub unsafe fn setup_lsi(rcc: &mut pac::RCC, pre: LsiPre) {
    rcc.csr.modify(|r, w| {
        // LSI pre-scaler is applied after an on-off cycle
        // leave LSI on if it is already on with the correct prescaler
        let lsion: bool = r.lsion().is_on() && r.lsipre().variant() == pre;

        w.lsipre().variant(pre).lsion().bit(lsion)
    });
    enable_lsi(rcc)
}

/// Enable the LSI clock with the currently configured pre-scaler and wait
/// for completion.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::{pac, rcc::enable_lsi};
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
/// use stm32wlxx_hal::{pac, rcc::pulse_reset_backup_domain};
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

/// Low-speed oscillator output pin.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Lsco {
    pin: crate::gpio::pins::A2,
}

impl Lsco {
    /// Enable the low-speed oscillator output.
    ///
    /// # Safety
    ///
    /// 1. Backup domain write protect must be disabled.
    /// 2. The selected clock must be enabled for system use.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     gpio::PortA,
    ///     pac,
    ///     rcc::{Lsco, LscoSel},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // disable backup domain write protect
    /// dp.PWR.cr1.modify(|_, w| w.dbp().enabled());
    ///
    /// // enable the LSE clock
    /// dp.RCC
    ///     .bdcr
    ///     .modify(|_, w| w.lseon().on().lsesysen().enabled());
    /// while dp.RCC.bdcr.read().lserdy().is_not_ready() {}
    /// while dp.RCC.bdcr.read().lsesysrdy().is_not_ready() {}
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let a2: Lsco = cortex_m::interrupt::free(|cs| unsafe {
    ///     Lsco::enable(gpioa.a2, LscoSel::Lse, &mut dp.RCC, cs)
    /// });
    /// ```
    #[inline]
    pub unsafe fn enable(
        mut a2: crate::gpio::pins::A2,
        sel: LscoSel,
        rcc: &mut pac::RCC,
        cs: &CriticalSection,
    ) -> Self {
        use crate::gpio::sealed::Lsco;
        a2.set_lsco_af(cs);
        rcc.bdcr
            .modify(|_, w| w.lscoen().enabled().lscosel().variant(sel));
        Self { pin: a2 }
    }

    /// Disable the low-speed oscillator output.
    ///
    /// # Safety
    ///
    /// 1. Backup domain write protect must be disabled.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     gpio::{pins, PortA},
    ///     pac,
    ///     rcc::{Lsco, LscoSel},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // disable backup domain write protect
    /// dp.PWR.cr1.modify(|_, w| w.dbp().enabled());
    ///
    /// // enable the LSE clock
    /// dp.RCC
    ///     .bdcr
    ///     .modify(|_, w| w.lseon().on().lsesysen().enabled());
    /// while dp.RCC.bdcr.read().lserdy().is_not_ready() {}
    /// while dp.RCC.bdcr.read().lsesysrdy().is_not_ready() {}
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let lsco: Lsco = cortex_m::interrupt::free(|cs| unsafe {
    ///     Lsco::enable(gpioa.a2, LscoSel::Lse, &mut dp.RCC, cs)
    /// });
    ///
    /// // ... use LSCO
    ///
    /// let a2: pins::A2 = unsafe { lsco.disable(&mut dp.RCC) };
    /// ```
    #[inline]
    pub unsafe fn disable(self, rcc: &mut pac::RCC) -> crate::gpio::pins::A2 {
        rcc.bdcr.modify(|_, w| w.lscoen().disabled());
        self.pin
    }
}
