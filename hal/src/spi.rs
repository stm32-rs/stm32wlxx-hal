//! Serial peripheral interface
#![forbid(missing_docs)]

use crate::{gpio, pac};

use core::ptr::{read_volatile, write_volatile};

pub use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3};

pub use pac::spi1::cr1::BR_A as BaudDiv;

/// SPI errors
#[derive(Debug)]
pub enum Error {
    /// Frame format error
    Framing,
    /// CRC check error
    Crc,
    /// NSS mode fault
    ModeFault,
    /// RX FIFO overrun
    Overrun,
}

const DR_OFFSET: usize = 0xC;
const SPI1_BASE: usize = 0x4001_3000;
const SPI2_BASE: usize = 0x4000_3800;
const SPI3_BASE: usize = 0x5801_0000;

trait SpiRegs {
    const DR: usize;

    fn cr1(&self) -> &pac::spi1::CR1;
    fn cr2(&self) -> &pac::spi1::CR2;
    fn sr(&self) -> &pac::spi1::SR;

    #[inline(always)]
    fn status(&self) -> Result<pac::spi1::sr::R, Error> {
        let sr = self.sr().read();
        if sr.ovr().bit_is_set() {
            Err(Error::Overrun)
        } else if sr.fre().bit_is_set() {
            Err(Error::Framing)
        } else if sr.modf().bit_is_set() {
            Err(Error::ModeFault)
        } else if sr.crcerr().bit_is_set() {
            Err(Error::Crc)
        } else {
            Ok(sr)
        }
    }

    fn write_word(&mut self, word: u8) -> Result<(), Error> {
        loop {
            if !self.status()?.ftlvl().is_full() {
                unsafe { write_volatile(Self::DR as *mut u8, word) };
                return Ok(());
            }
        }
    }

    fn read_word(&mut self) -> Result<u8, Error> {
        loop {
            if !self.status()?.frlvl().is_empty() {
                return Ok(unsafe { read_volatile(Self::DR as *const u8) });
            }
        }
    }

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Error> {
        for word in words.iter_mut() {
            self.write_word(*word)?;
            *word = self.read_word()?;
        }
        Ok(words)
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        for word in words.iter() {
            self.write_word(*word)?;
            let _: u8 = self.read_word()?;
        }
        Ok(())
    }
}

#[derive(Debug)]
struct Spi1Regs {
    regs: pac::SPI1,
}

#[derive(Debug)]
struct Spi2Regs {
    regs: pac::SPI2,
}

#[derive(Debug)]
struct Spi3Regs {
    regs: pac::SPI3,
}

macro_rules! impl_spi_regs_for {
    ($name:ident, $dr:expr) => {
        impl SpiRegs for $name {
            const DR: usize = $dr;
            #[inline(always)]
            fn cr1(&self) -> &pac::spi1::CR1 {
                &self.regs.cr1
            }
            #[inline(always)]
            fn cr2(&self) -> &pac::spi1::CR2 {
                &self.regs.cr2
            }
            #[inline(always)]
            fn sr(&self) -> &pac::spi1::SR {
                &self.regs.sr
            }
        }
    };
}

impl_spi_regs_for!(Spi1Regs, SPI1_BASE + DR_OFFSET);
impl_spi_regs_for!(Spi2Regs, SPI2_BASE + DR_OFFSET);
impl_spi_regs_for!(Spi3Regs, SPI3_BASE + DR_OFFSET);

const fn cpha_from_phase(phase: Phase) -> bool {
    match phase {
        Phase::CaptureOnFirstTransition => false,
        Phase::CaptureOnSecondTransition => true,
    }
}

const fn cpol_from_polarity(polarity: Polarity) -> bool {
    match polarity {
        Polarity::IdleLow => false,
        Polarity::IdleHigh => true,
    }
}

/// SPI1 driver
#[derive(Debug)]
pub struct Spi1<MOSI, MISO, SCK> {
    spi: Spi1Regs,
    mosi: MOSI,
    miso: MISO,
    sck: SCK,
}

/// SPI2 driver
#[derive(Debug)]
pub struct Spi2<MOSI, MISO, SCK> {
    spi: Spi2Regs,
    mosi: MOSI,
    miso: MISO,
    sck: SCK,
}

/// SPI3 driver
///
/// This is private because SPI3 is an internal bus for the SubGHz radio,
/// which the HAL provides an interface for.
#[derive(Debug)]
pub(crate) struct Spi3 {
    spi: Spi3Regs,
}

impl<MOSI, MISO, SCK> Spi1<MOSI, MISO, SCK>
where
    MOSI: gpio::sealed::Spi1Mosi,
    MISO: gpio::sealed::Spi1Miso,
    SCK: gpio::sealed::Spi1Sck,
{
    /// Create a new `Spi1` driver.
    ///
    /// This will enable clocks and reset the SPI1 peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortA},
    ///     pac,
    ///     spi::{BaudDiv, Spi1, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let spi: Spi1<pins::A7, pins::A6, pins::A5> = Spi1::new(
    ///     dp.SPI1,
    ///     pa.pa7,
    ///     pa.pa6,
    ///     pa.pa5,
    ///     MODE_0,
    ///     BaudDiv::DIV4,
    ///     &mut dp.RCC,
    /// );
    /// ```
    pub fn new(
        spi1: pac::SPI1,
        mut mosi: MOSI,
        mut miso: MISO,
        mut sck: SCK,
        mode: Mode,
        div: BaudDiv,
        rcc: &mut pac::RCC,
    ) -> Spi1<MOSI, MISO, SCK> {
        Self::enable_clock(rcc);
        Self::toggle_reset(rcc);

        mosi.set_spi1_mosi_af();
        miso.set_spi1_miso_af();
        sck.set_spi1_sck_af();

        #[rustfmt::skip]
        spi1.cr1.write(|w|
            w
                .ssi().set_bit()
                .ssm().set_bit()
                .spe().set_bit()
                .br().variant(div)
                .mstr().set_bit()
                .cpol().bit(cpol_from_polarity(mode.polarity))
                .cpha().bit(cpha_from_phase(mode.phase))
        );

        // cr2 register reset values are sane for now

        Spi1 {
            spi: Spi1Regs { regs: spi1 },
            mosi,
            miso,
            sck,
        }
    }

    /// Free the GPIOs from the SPI driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortA},
    ///     pac,
    ///     spi::{BaudDiv, Spi1, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let spi: Spi1<pins::A7, pins::A6, pins::A5> = Spi1::new(
    ///     dp.SPI1,
    ///     pa.pa7,
    ///     pa.pa6,
    ///     pa.pa5,
    ///     MODE_0,
    ///     BaudDiv::DIV4,
    ///     &mut dp.RCC,
    /// );
    ///
    /// // use SPI bus
    ///
    /// let (spi1, pa7, pa6, pa5) = spi.free();
    /// ```
    pub fn free(self) -> (pac::SPI1, MOSI, MISO, SCK) {
        (self.spi.regs, self.mosi, self.miso, self.sck)
    }

    /// Disable the SPI1 clock
    pub fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb2enr.modify(|_, w| w.spi1en().disabled());
    }

    /// Enable the SPI1 clock
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb2enr.modify(|_, w| w.spi1en().enabled());
        rcc.apb2enr.read(); // delay after an RCC peripheral clock enabling
    }

    fn toggle_reset(rcc: &mut pac::RCC) {
        rcc.apb2rstr.modify(|_, w| w.spi1rst().set_bit());
        rcc.apb2rstr.modify(|_, w| w.spi1rst().clear_bit());
    }
}

impl<MOSI, MISO, SCK> Spi2<MOSI, MISO, SCK>
where
    MOSI: gpio::sealed::Spi2Mosi,
    MISO: gpio::sealed::Spi2Miso,
    SCK: gpio::sealed::Spi2Sck,
{
    /// Create a new `Spi2` driver.
    ///
    /// This will enable clocks and reset the SPI2 peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortB},
    ///     pac,
    ///     spi::{BaudDiv, Spi2, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pb: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let spi: Spi2<pins::B15, pins::B14, pins::B13> = Spi2::new(
    ///     dp.SPI2,
    ///     pb.pb15,
    ///     pb.pb14,
    ///     pb.pb13,
    ///     MODE_0,
    ///     BaudDiv::DIV4,
    ///     &mut dp.RCC,
    /// );
    /// ```
    pub fn new(
        spi2: pac::SPI2,
        mut mosi: MOSI,
        mut miso: MISO,
        mut sck: SCK,
        mode: Mode,
        div: BaudDiv,
        rcc: &mut pac::RCC,
    ) -> Spi2<MOSI, MISO, SCK> {
        Self::enable_clock(rcc);
        Self::toggle_reset(rcc);

        mosi.set_spi2_mosi_af();
        miso.set_spi2_miso_af();
        sck.set_spi2_sck_af();

        #[rustfmt::skip]
        spi2.cr1.write(|w|
            w
                .ssi().set_bit()
                .ssm().set_bit()
                .spe().set_bit()
                .br().variant(div)
                .mstr().set_bit()
                .cpol().bit(cpol_from_polarity(mode.polarity))
                .cpha().bit(cpha_from_phase(mode.phase))
        );

        // cr2 register reset values are sane for now

        Spi2 {
            spi: Spi2Regs { regs: spi2 },
            mosi,
            miso,
            sck,
        }
    }

    /// Free the GPIOs from the SPI driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortB},
    ///     pac,
    ///     spi::{BaudDiv, Spi2, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pb: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let spi: Spi2<pins::B15, pins::B14, pins::B13> = Spi2::new(
    ///     dp.SPI2,
    ///     pb.pb15,
    ///     pb.pb14,
    ///     pb.pb13,
    ///     MODE_0,
    ///     BaudDiv::DIV4,
    ///     &mut dp.RCC,
    /// );
    ///
    /// // use SPI bus
    ///
    /// let (spi2, pb15, pb14, pb13) = spi.free();
    /// ```
    pub fn free(self) -> (pac::SPI2, MOSI, MISO, SCK) {
        (self.spi.regs, self.mosi, self.miso, self.sck)
    }

    /// Disable the SPI2 clock
    pub fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr1.modify(|_, w| w.spi2s2en().disabled());
    }

    /// Enable the SPI2 clock
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr1.modify(|_, w| w.spi2s2en().enabled());
        rcc.apb1enr1.read(); // delay after an RCC peripheral clock enabling
    }

    fn toggle_reset(rcc: &mut pac::RCC) {
        rcc.apb1rstr1.modify(|_, w| w.spi2s2rst().set_bit());
        rcc.apb1rstr1.modify(|_, w| w.spi2s2rst().clear_bit());
    }
}

impl Spi3 {
    pub fn new(spi3: pac::SPI3, div: BaudDiv, rcc: &mut pac::RCC) -> Spi3 {
        Self::enable_clock(rcc);
        Self::toggle_reset(rcc);

        #[rustfmt::skip]
        spi3.cr1.write(|w|
            w
                .ssi().set_bit()
                .ssm().set_bit()
                .spe().set_bit()
                .br().variant(div)
                .mstr().set_bit()
                // hard coded because we know the SPI mode of the radio
                .cpol().idle_low()
                .cpha().first_edge()
        );

        Spi3 {
            spi: Spi3Regs { regs: spi3 },
        }
    }

    pub unsafe fn steal() -> Spi3 {
        Spi3 {
            spi: Spi3Regs {
                regs: pac::Peripherals::steal().SPI3,
            },
        }
    }

    /// Disable the SPI3 (SubGHz SPI) clock
    pub fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb3enr.modify(|_, w| w.subghzspien().disabled());
    }

    /// Enable the SPI3 (SubGHz SPI) clock
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb3enr.modify(|_, w| w.subghzspien().enabled());
        rcc.apb3enr.read(); // delay after an RCC peripheral clock enabling
    }

    /// Toggle the SPI3 (SubGHz SPI) reset
    pub fn toggle_reset(rcc: &mut pac::RCC) {
        rcc.apb1rstr1.modify(|_, w| w.spi2s2rst().set_bit());
        rcc.apb1rstr1.modify(|_, w| w.spi2s2rst().clear_bit());
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Transfer<u8> for Spi1<MOSI, MISO, SCK>
where
    MOSI: gpio::sealed::Spi1Mosi,
    MISO: gpio::sealed::Spi1Miso,
    SCK: gpio::sealed::Spi1Sck,
{
    type Error = Error;

    #[inline(always)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi.transfer(words)
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Write<u8> for Spi1<MOSI, MISO, SCK>
where
    MOSI: gpio::sealed::Spi1Mosi,
    MISO: gpio::sealed::Spi1Miso,
    SCK: gpio::sealed::Spi1Sck,
{
    type Error = Error;

    #[inline(always)]
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write(words)
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Transfer<u8> for Spi2<MOSI, MISO, SCK>
where
    MOSI: gpio::sealed::Spi2Mosi,
    MISO: gpio::sealed::Spi2Miso,
    SCK: gpio::sealed::Spi2Sck,
{
    type Error = Error;

    #[inline(always)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi.transfer(words)
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Write<u8> for Spi2<MOSI, MISO, SCK>
where
    MOSI: gpio::sealed::Spi2Mosi,
    MISO: gpio::sealed::Spi2Miso,
    SCK: gpio::sealed::Spi2Sck,
{
    type Error = Error;

    #[inline(always)]
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write(words)
    }
}

impl embedded_hal::blocking::spi::Transfer<u8> for Spi3 {
    type Error = Error;

    #[inline(always)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi.transfer(words)
    }
}

impl embedded_hal::blocking::spi::Write<u8> for Spi3 {
    type Error = Error;

    #[inline(always)]
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write(words)
    }
}
