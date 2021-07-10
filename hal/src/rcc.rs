//! Reset and clocking control

use core::{
    convert::{TryFrom, TryInto},
    sync::atomic::{compiler_fence, Ordering::SeqCst},
};

use crate::pac;

fn rcc_set_flash_latency_from_msi_range(
    flash: &pac::FLASH,
    rcc: &pac::RCC,
    msi_range: &MsiRange,
    vos: Vos,
) {
    let msi_freq: u32 = msi_range.as_hertz();
    let div: u32 = u32::from(hclk3_prescaler_div(rcc));
    let flash_clk_src_freq: u32 = msi_freq / div;

    let latency: u8 = FlashLatency::from_hertz(vos, flash_clk_src_freq).into();

    flash
        .acr
        .modify(|_, w| unsafe { w.latency().bits(latency) });

    while flash.acr.read().latency().bits() != latency {
        compiler_fence(SeqCst);
    }
}

#[repr(u8)]
#[derive(Debug, PartialEq, Eq)]
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

#[repr(u8)]
#[derive(Debug, PartialEq, Eq)]
enum SysclkSource {
    Msi = 0b00,
    Hsi16 = 0b01,
    Hse32 = 0b10,
    Pllrclk = 0b11,
}

impl SysclkSource {
    pub fn from_bits(bits: u8) -> SysclkSource {
        match bits & 0b11 {
            0b00 => SysclkSource::Msi,
            0b01 => SysclkSource::Hsi16,
            0b10 => SysclkSource::Hse32,
            0b11 => SysclkSource::Pllrclk,
            _ => unreachable!(),
        }
    }

    /// Returns `true` if the sysclk_source is [`Msi`].
    pub fn is_msi(&self) -> bool {
        matches!(self, Self::Msi)
    }

    /// Returns `true` if the sysclk_source is [`Hsi16`].
    #[allow(dead_code)]
    pub fn is_hsi16(&self) -> bool {
        matches!(self, Self::Hsi16)
    }

    /// Returns `true` if the sysclk_source is [`Hse32`].
    #[allow(dead_code)]
    pub fn is_hse32(&self) -> bool {
        matches!(self, Self::Hse32)
    }

    /// Returns `true` if the sysclk_source is [`Pllrclk`].
    pub fn is_pllrclk(&self) -> bool {
        matches!(self, Self::Pllrclk)
    }
}

impl TryFrom<u8> for SysclkSource {
    type Error = u8;

    fn try_from(bits: u8) -> Result<Self, Self::Error> {
        match bits {
            0b00 => Ok(SysclkSource::Msi),
            0b01 => Ok(SysclkSource::Hsi16),
            0b10 => Ok(SysclkSource::Hse32),
            0b11 => Ok(SysclkSource::Pllrclk),
            _ => Err(bits),
        }
    }
}

impl From<SysclkSource> for u8 {
    fn from(src: SysclkSource) -> Self {
        src as u8
    }
}

#[repr(u8)]
enum PllSrc {
    None = 0b00,
    Msi = 0b01,
    Hsi16 = 0b10,
    Hse32 = 0b11,
}

impl PllSrc {
    pub fn from_bits(bits: u8) -> PllSrc {
        match bits & 0b11 {
            0b00 => PllSrc::None,
            0b01 => PllSrc::Msi,
            0b10 => PllSrc::Hsi16,
            0b11 => PllSrc::Hse32,
            _ => unreachable!(),
        }
    }

    /// Returns `true` if the pll_src is [`Msi`].
    fn is_msi(&self) -> bool {
        matches!(self, Self::Msi)
    }
}

/// MSI clock ranges.
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
        let cr = rcc.cr.read();
        if cr.msirgsel().bit_is_set() {
            cr.msirange().bits().try_into().unwrap()
        } else {
            rcc.csr.read().msisrange().bits().try_into().unwrap()
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

fn hclk3_prescaler_div(rcc: &pac::RCC) -> u16 {
    match rcc.extcfgr.read().shdhpre().bits() {
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

/// Voltage scaling.
///
/// See table 58 "Clock source frequency" in the reference manual.
#[derive(Debug, PartialEq, Eq)]
#[repr(u8)]
#[allow(dead_code)]
enum Vos {
    Range1 = 0b01,
    Range2 = 0b10,
}

/// Set the sysclk to the MSI source at 48MHz.
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
    const VOS: Vos = Vos::Range1;
    const MSI_RANGE: MsiRange = MsiRange::Range11;
    const MSI_CALIBRATION: u8 = 0;

    pwr.cr1.modify(|_, w| unsafe { w.vos().bits(VOS as u8) });

    let sysclk_source: SysclkSource = SysclkSource::from_bits(rcc.cfgr.read().sws().bits());
    let pll_config: PllSrc = PllSrc::from_bits(rcc.pllcfgr.read().pllsrc().bits());

    if !(sysclk_source.is_msi() || sysclk_source.is_pllrclk() && pll_config.is_msi()) {
        rcc.cfgr
            .modify(|_, w| w.sw().bits(SysclkSource::Msi.into()));
    }

    if MSI_RANGE > MsiRange::from_rcc(rcc) {
        rcc_set_flash_latency_from_msi_range(flash, rcc, &MSI_RANGE, VOS);
        rcc.cr
            .modify(|_, w| unsafe { w.msirgsel().set_bit().msirange().bits(MSI_RANGE.into()) });
        rcc.icscr.modify(|_, w| w.msitrim().bits(MSI_CALIBRATION));
    } else {
        rcc.cr
            .modify(|_, w| unsafe { w.msirgsel().set_bit().msirange().bits(MSI_RANGE.into()) });
        rcc.icscr.modify(|_, w| w.msitrim().bits(MSI_CALIBRATION));
        rcc_set_flash_latency_from_msi_range(flash, rcc, &MSI_RANGE, VOS);
    }

    // HCLK1 configuration
    rcc.cfgr.modify(|_, w| unsafe { w.hpre().bits(0) });
    while rcc.cfgr.read().hpref().bit_is_clear() {
        compiler_fence(SeqCst);
    }

    // HCLK3 configuration
    rcc.extcfgr.modify(|_, w| unsafe { w.shdhpre().bits(0) });
    while rcc.extcfgr.read().shdhpref().bit_is_clear() {
        compiler_fence(SeqCst);
    }

    // PCLK1 configuration
    rcc.cfgr.modify(|_, w| unsafe { w.ppre1().bits(0) });
    while rcc.cfgr.read().ppre1f().bit_is_clear() {
        compiler_fence(SeqCst);
    }

    // PCLK2 configuration
    rcc.cfgr.modify(|_, w| unsafe { w.ppre2().bits(0) });
    while rcc.cfgr.read().ppre2f().bit_is_clear() {
        compiler_fence(SeqCst);
    }

    assert!(rcc.cr.read().msirdy().bit_is_set());
    rcc.cfgr
        .modify(|_, w| w.sw().bits(SysclkSource::Msi.into()));
    while SysclkSource::try_from(rcc.cfgr.read().sws().bits()) != Ok(SysclkSource::Msi) {
        compiler_fence(SeqCst);
    }
}
