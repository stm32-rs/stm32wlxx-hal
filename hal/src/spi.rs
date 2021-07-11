//! Serial peripheral interface

use crate::{gpio, pac};

use core::ptr::{read_volatile, write_volatile};

pub use embedded_hal::spi::{Mode, MODE_0, MODE_1, MODE_2, MODE_3};

/// SPI errors.
#[derive(Debug)]
pub enum Error {
    /// Frame format error.
    Framing,
    /// CRC check error.
    Crc,
    /// NSS mode fault.
    ModeFault,
    /// RX FIFO overrun.
    Overrun,
}

const SPI1_BASE: usize = 0x4001_3000;
const SPI1_DR: usize = SPI1_BASE + 0xC;
#[allow(dead_code)]
const SPI2_BASE: usize = 0x4000_3800;

/// SPI1 driver.
pub struct Spi1<MOSI, MISO, SCK> {
    spi1: pac::SPI1,
    mosi: MOSI,
    miso: MISO,
    sck: SCK,
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
    ///     spi::{Spi1, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rcc = dp.RCC;
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut rcc);
    /// let spi: Spi1<pins::A7, pins::A6, pins::A5> =
    ///     Spi1::new(dp.SPI1, pa.pa7, pa.pa6, pa.pa5, MODE_0, &mut rcc);
    /// ```
    pub fn new(
        spi1: pac::SPI1,
        mut mosi: MOSI,
        mut miso: MISO,
        mut sck: SCK,
        mode: Mode,
        rcc: &mut pac::RCC,
    ) -> Spi1<MOSI, MISO, SCK> {
        Self::enable_clock(rcc);

        rcc.apb2rstr.modify(|_, w| w.spi1rst().set_bit());
        rcc.apb2rstr.modify(|_, w| w.spi1rst().clear_bit());
        mosi.set_spi1_mosi_af();
        miso.set_spi1_miso_af();
        sck.set_spi1_sck_af();

        let cpha: bool = match mode.phase {
            embedded_hal::spi::Phase::CaptureOnFirstTransition => false,
            embedded_hal::spi::Phase::CaptureOnSecondTransition => true,
        };

        let cpol: bool = match mode.polarity {
            embedded_hal::spi::Polarity::IdleLow => false,
            embedded_hal::spi::Polarity::IdleHigh => true,
        };

        #[rustfmt::skip]
        spi1.cr1.write(|w|
            w
                .ssi().set_bit()
                .ssm().set_bit()
                .spe().set_bit()
                .br().div4()
                .mstr().set_bit()
                .cpol().bit(cpol)
                .cpha().bit(cpha)
        );

        // cr2 register reset values are sane for now

        Spi1 {
            spi1,
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
    ///     spi::{Spi1, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rcc = dp.RCC;
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut rcc);
    /// let spi: Spi1<pins::A7, pins::A6, pins::A5> =
    ///     Spi1::new(dp.SPI1, pa.pa7, pa.pa6, pa.pa5, MODE_0, &mut rcc);
    ///
    /// // use SPI bus
    ///
    /// let (spi1, pa7, pa6, pa5) = spi.free();
    /// ```
    pub fn free(self) -> (pac::SPI1, MOSI, MISO, SCK) {
        (self.spi1, self.mosi, self.miso, self.sck)
    }

    /// Disable the SPI1 clock.
    pub fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb2enr.modify(|_, w| w.spi1en().disabled());
    }

    /// Enable the SPI1 clock.
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb2enr.modify(|_, w| w.spi1en().enabled());
        rcc.apb2enr.read(); // delay after an RCC peripheral clock enabling
    }

    #[inline(always)]
    fn status(&self) -> Result<pac::spi1::sr::R, Error> {
        let sr = self.spi1.sr.read();
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
}

impl<MOSI, MISO, SCK> Spi1<MOSI, MISO, SCK>
where
    MOSI: gpio::sealed::Spi1Mosi,
    MISO: gpio::sealed::Spi1Miso,
    SCK: gpio::sealed::Spi1Sck,
{
    fn write_word(&mut self, word: u8) -> Result<(), Error> {
        loop {
            if self.status()?.txe().bit_is_set() {
                // access size must be 1 byte
                unsafe { write_volatile(SPI1_DR as *mut u8, word) };
                return Ok(());
            }
        }
    }

    fn read_word(&mut self) -> Result<u8, Error> {
        loop {
            if !self.status()?.frlvl().is_empty() {
                // access size must be 1 byte
                return Ok(unsafe { read_volatile(SPI1_DR as *const u8) });
            }
        }
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Transfer<u8> for Spi1<MOSI, MISO, SCK>
where
    MOSI: gpio::sealed::Spi1Mosi,
    MISO: gpio::sealed::Spi1Miso,
    SCK: gpio::sealed::Spi1Sck,
{
    type Error = Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        for word in words.iter_mut() {
            self.write_word(*word)?;
            *word = self.read_word()?;
        }
        Ok(words)
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Write<u8> for Spi1<MOSI, MISO, SCK>
where
    MOSI: gpio::sealed::Spi1Mosi,
    MISO: gpio::sealed::Spi1Miso,
    SCK: gpio::sealed::Spi1Sck,
{
    type Error = Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for word in words.iter() {
            self.write_word(*word)?;
            let _: u8 = self.read_word()?;
        }
        Ok(())
    }
}
