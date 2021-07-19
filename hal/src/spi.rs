//! Serial peripheral interface
#![deny(missing_docs)]

use crate::{
    dma::{self, DmaCh},
    gpio, pac,
};

use core::{
    ptr::{read_volatile, write_volatile},
    sync::atomic::{compiler_fence, Ordering::SeqCst},
};

pub use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3};

pub use pac::spi1::cr1::BR_A as BaudDiv;

/// SPI errors
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Error {
    /// Frame format error
    Framing,
    /// CRC check error
    Crc,
    /// NSS mode fault
    ModeFault,
    /// RX FIFO overrun
    Overrun,
    /// DMA error
    ///
    /// This can only occur on SPI transfers that use the DMA.
    Dma,
}

impl From<dma::Error> for Error {
    fn from(e: dma::Error) -> Self {
        match e {
            dma::Error::Xfer => Error::Dma,
        }
    }
}

const DR_OFFSET: usize = 0xC;
const SPI1_BASE: usize = 0x4001_3000;
const SPI2_BASE: usize = 0x4000_3800;
const SPI3_BASE: usize = 0x5801_0000;

trait SpiBase {
    const DR: usize;
    const DMA_TX_ID: u8;
    const DMA_RX_ID: u8;

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

    #[inline(always)]
    fn poll_busy(&self) -> Result<(), Error> {
        while self.status()?.bsy().is_busy() {
            compiler_fence(SeqCst)
        }
        Ok(())
    }

    fn enable_dma(&mut self) {
        self.cr2()
            .modify(|_, w| w.txdmaen().enabled().rxdmaen().enabled().frxth().quarter());
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

    fn write_with_dma(
        &mut self,
        tx_dma: &mut DmaCh,
        rx_dma: &mut DmaCh,
        words: &[u8],
    ) -> Result<(), Error> {
        const RX_CR: dma::Cr = dma::Cr::RESET.set_dir_from_periph().set_mem_inc(false);
        const TX_CR: dma::Cr = dma::Cr::RESET.set_dir_from_mem().set_mem_inc(true);

        // setup DMAs without enabling
        rx_dma.set_cr(RX_CR);
        tx_dma.set_cr(TX_CR);

        rx_dma.clear_all_flags();
        tx_dma.clear_all_flags();

        let garbage: [u8; 1] = [0];

        rx_dma.set_mem_addr(garbage.as_ptr() as u32);
        tx_dma.set_mem_addr(words.as_ptr() as u32);

        rx_dma.set_periph_addr(Self::DR as u32);
        tx_dma.set_periph_addr(Self::DR as u32);

        let ndt: u32 = words.len() as u32;
        rx_dma.set_num_data_xfer(ndt);
        tx_dma.set_num_data_xfer(ndt);

        rx_dma.set_mux_cr_reqid(Self::DMA_RX_ID);
        tx_dma.set_mux_cr_reqid(Self::DMA_TX_ID);

        // RX MUST come before TX
        rx_dma.set_cr(RX_CR.set_enable(true));
        tx_dma.set_cr(TX_CR.set_enable(true));

        let ret: Result<(), Error> = loop {
            let status = self.status()?;
            let tx_dma_flags: u8 = tx_dma.flags();
            let rx_dma_flags: u8 = rx_dma.flags();
            if tx_dma_flags & dma::flags::XFER_ERR != 0 {
                break Err(Error::Dma);
            }
            if rx_dma_flags & dma::flags::XFER_ERR != 0 {
                break Err(Error::Dma);
            }
            if status.bsy().is_not_busy()
                && tx_dma_flags & dma::flags::XFER_CPL != 0
                && rx_dma_flags & dma::flags::XFER_CPL != 0
            {
                break Ok(());
            }
        };

        // DMA disable must come BEFORE memory barrier
        // TX must come before RX
        tx_dma.set_cr(dma::Cr::DISABLE);
        rx_dma.set_cr(dma::Cr::DISABLE);

        ret
    }

    fn transfer_with_dma<'w>(
        &mut self,
        tx_dma: &mut DmaCh,
        rx_dma: &mut DmaCh,
        words: &'w mut [u8],
    ) -> Result<&'w [u8], Error> {
        const RX_CR: dma::Cr = dma::Cr::RESET.set_dir_from_periph().set_mem_inc(true);
        const TX_CR: dma::Cr = dma::Cr::RESET.set_dir_from_mem().set_mem_inc(true);

        // setup DMAs without enabling
        rx_dma.set_cr(RX_CR);
        tx_dma.set_cr(TX_CR);

        rx_dma.clear_all_flags();
        tx_dma.clear_all_flags();

        rx_dma.set_mem_addr(words.as_ptr() as u32);
        tx_dma.set_mem_addr(words.as_ptr() as u32);

        rx_dma.set_periph_addr(Self::DR as u32);
        tx_dma.set_periph_addr(Self::DR as u32);

        let ndt: u32 = words.len() as u32;
        rx_dma.set_num_data_xfer(ndt);
        tx_dma.set_num_data_xfer(ndt);

        rx_dma.set_mux_cr_reqid(Self::DMA_RX_ID);
        tx_dma.set_mux_cr_reqid(Self::DMA_TX_ID);

        // RX MUST come before TX
        rx_dma.set_cr(RX_CR.set_enable(true));
        tx_dma.set_cr(TX_CR.set_enable(true));

        let ret: Result<&'w [u8], Error> = loop {
            let status = self.status()?;
            let tx_dma_flags: u8 = tx_dma.flags();
            let rx_dma_flags: u8 = rx_dma.flags();
            if tx_dma_flags & dma::flags::XFER_ERR != 0 {
                break Err(Error::Dma);
            }
            if rx_dma_flags & dma::flags::XFER_ERR != 0 {
                break Err(Error::Dma);
            }
            if status.bsy().is_not_busy()
                && tx_dma_flags & dma::flags::XFER_CPL != 0
                && rx_dma_flags & dma::flags::XFER_CPL != 0
            {
                break Ok(words);
            }
        };

        // DMA disable must come BEFORE memory barrier
        // TX must come before RX
        tx_dma.set_cr(dma::Cr::DISABLE);
        rx_dma.set_cr(dma::Cr::DISABLE);

        // memory in dst may have changed, but the compiler does not know it
        compiler_fence(SeqCst);
        cortex_m::asm::dmb();
        compiler_fence(SeqCst);

        ret
    }
}

#[derive(Debug)]
struct Spi1Base {
    pac: pac::SPI1,
}

#[derive(Debug)]
struct Spi2Base {
    pac: pac::SPI2,
}

#[derive(Debug)]
struct Spi3Base {
    pac: pac::SPI3,
}

macro_rules! impl_spi_base_for {
    ($name:ident, $dr:expr, $tx_id:expr, $rx_id:expr) => {
        impl SpiBase for $name {
            const DR: usize = $dr;
            const DMA_TX_ID: u8 = $tx_id as u8;
            const DMA_RX_ID: u8 = $rx_id as u8;
            #[inline(always)]
            fn cr1(&self) -> &pac::spi1::CR1 {
                &self.pac.cr1
            }
            #[inline(always)]
            fn cr2(&self) -> &pac::spi1::CR2 {
                &self.pac.cr2
            }
            #[inline(always)]
            fn sr(&self) -> &pac::spi1::SR {
                &self.pac.sr
            }
        }
    };
}

use stm32wl::stm32wl5x_cm4::dmamux::c0cr::DMAREQ_ID_A::{
    SPI1_RX_DMA, SPI1_TX_DMA, SPI2_RX_DMA, SPI2_TX_DMA, SUBGHZSPI_RX, SUBGHZSPI_TX,
};

impl_spi_base_for!(Spi1Base, SPI1_BASE + DR_OFFSET, SPI1_TX_DMA, SPI1_RX_DMA);
impl_spi_base_for!(Spi2Base, SPI2_BASE + DR_OFFSET, SPI2_TX_DMA, SPI2_RX_DMA);
impl_spi_base_for!(Spi3Base, SPI3_BASE + DR_OFFSET, SUBGHZSPI_TX, SUBGHZSPI_RX);

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
    base: Spi1Base,
    mosi: MOSI,
    miso: MISO,
    sck: SCK,
}

/// SPI1 driver with DMA operations
#[derive(Debug)]
pub struct Spi1Dma<MOSI, MISO, SCK> {
    spi1: Spi1<MOSI, MISO, SCK>,
    tx_dma: DmaCh,
    rx_dma: DmaCh,
}

/// SPI2 driver
#[derive(Debug)]
pub struct Spi2<MOSI, MISO, SCK> {
    base: Spi2Base,
    mosi: MOSI,
    miso: MISO,
    sck: SCK,
}

/// SPI2 driver with DMA operations
#[derive(Debug)]
pub struct Spi2Dma<MOSI, MISO, SCK> {
    spi2: Spi2<MOSI, MISO, SCK>,
    tx_dma: DmaCh,
    rx_dma: DmaCh,
}

/// SPI3 driver
///
/// This *should* be private because SPI3 is an internal bus for the SubGHz
/// radio, which the HAL provides an interface for.
///
/// It is public for testing purposes, the radio allows us to write tests
/// for the SPI driver with hardware that everyone using this HAL will have.
#[derive(Debug)]
#[doc(hidden)]
pub struct Spi3 {
    base: Spi3Base,
}

/// SPI3 driver with DMA operations
#[derive(Debug)]
#[doc(hidden)]
pub struct Spi3Dma {
    spi3: Spi3,
    tx_dma: DmaCh,
    rx_dma: DmaCh,
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
            base: Spi1Base { pac: spi1 },
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
        (self.base.pac, self.mosi, self.miso, self.sck)
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

impl<MOSI, MISO, SCK> Spi1Dma<MOSI, MISO, SCK> {
    /// Crate a new `Spi1Dma` driver from a `Spi1` driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     dma::AllDma,
    ///     gpio::{pins, PortA},
    ///     pac,
    ///     spi::{BaudDiv, Spi1, Spi1Dma, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    ///
    /// let spi1 = Spi1Dma::new(
    ///     Spi1::new(
    ///         dp.SPI1,
    ///         pa.pa7,
    ///         pa.pa6,
    ///         pa.pa5,
    ///         MODE_0,
    ///         BaudDiv::DIV4,
    ///         &mut dp.RCC,
    ///     ),
    ///     dma.d1c1,
    ///     dma.d2c1,
    /// );
    /// ```
    pub fn new(
        mut spi1: Spi1<MOSI, MISO, SCK>,
        tx_dma: DmaCh,
        rx_dma: DmaCh,
    ) -> Spi1Dma<MOSI, MISO, SCK> {
        spi1.base.enable_dma();
        Spi1Dma {
            spi1,
            tx_dma,
            rx_dma,
        }
    }

    /// Free the DMA channels and the `Spi1` driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     dma::AllDma,
    ///     gpio::{pins, PortA},
    ///     pac,
    ///     spi::{BaudDiv, Spi1, Spi1Dma, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    ///
    /// let spi1_dma = Spi1Dma::new(
    ///     Spi1::new(
    ///         dp.SPI1,
    ///         pa.pa7,
    ///         pa.pa6,
    ///         pa.pa5,
    ///         MODE_0,
    ///         BaudDiv::DIV4,
    ///         &mut dp.RCC,
    ///     ),
    ///     dma.d1c1,
    ///     dma.d2c1,
    /// );
    ///
    /// // use SPI bus
    ///
    /// let (spi1, d1c1, d2c2) = spi1_dma.free();
    /// ```
    pub fn free(self) -> (Spi1<MOSI, MISO, SCK>, DmaCh, DmaCh) {
        (self.spi1, self.tx_dma, self.rx_dma)
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
            base: Spi2Base { pac: spi2 },
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
        (self.base.pac, self.mosi, self.miso, self.sck)
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

impl<MOSI, MISO, SCK> Spi2Dma<MOSI, MISO, SCK> {
    /// Crate a new `Spi2Dma` driver from a `Spi2` driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     dma::AllDma,
    ///     gpio::{pins, PortB},
    ///     pac,
    ///     spi::{BaudDiv, Spi2, Spi2Dma, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pb: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    ///
    /// let spi1 = Spi2Dma::new(
    ///     Spi2::new(
    ///         dp.SPI2,
    ///         pb.pb15,
    ///         pb.pb14,
    ///         pb.pb13,
    ///         MODE_0,
    ///         BaudDiv::DIV4,
    ///         &mut dp.RCC,
    ///     ),
    ///     dma.d1c1,
    ///     dma.d2c1,
    /// );
    /// ```
    pub fn new(
        mut spi2: Spi2<MOSI, MISO, SCK>,
        tx_dma: DmaCh,
        rx_dma: DmaCh,
    ) -> Spi2Dma<MOSI, MISO, SCK> {
        spi2.base.enable_dma();
        Spi2Dma {
            spi2,
            tx_dma,
            rx_dma,
        }
    }

    /// Free the DMA channels and the `Spi2` driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     dma::AllDma,
    ///     gpio::{pins, PortB},
    ///     pac,
    ///     spi::{BaudDiv, Spi2, Spi2Dma, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pb: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    ///
    /// let spi2_dma = Spi2Dma::new(
    ///     Spi2::new(
    ///         dp.SPI2,
    ///         pb.pb15,
    ///         pb.pb14,
    ///         pb.pb13,
    ///         MODE_0,
    ///         BaudDiv::DIV4,
    ///         &mut dp.RCC,
    ///     ),
    ///     dma.d1c1,
    ///     dma.d2c1,
    /// );
    ///
    /// // use SPI bus
    ///
    /// let (spi2, d1c1, d2c2) = spi2_dma.free();
    /// ```
    pub fn free(self) -> (Spi2<MOSI, MISO, SCK>, DmaCh, DmaCh) {
        (self.spi2, self.tx_dma, self.rx_dma)
    }
}

impl Spi3 {
    #[allow(missing_docs)]
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
            base: Spi3Base { pac: spi3 },
        }
    }

    #[allow(missing_docs)]
    pub unsafe fn steal() -> Spi3 {
        Spi3 {
            base: Spi3Base {
                pac: pac::Peripherals::steal().SPI3,
            },
        }
    }

    #[allow(missing_docs)]
    pub fn free(self) -> pac::SPI3 {
        self.base.pac
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

impl Spi3Dma {
    #[allow(missing_docs)]
    pub fn new(mut spi3: Spi3, tx_dma: DmaCh, rx_dma: DmaCh) -> Spi3Dma {
        spi3.base.enable_dma();
        Spi3Dma {
            spi3,
            tx_dma,
            rx_dma,
        }
    }

    #[allow(missing_docs)]
    pub fn free(self) -> (Spi3, DmaCh, DmaCh) {
        (self.spi3, self.tx_dma, self.rx_dma)
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
        self.base.transfer(words)
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
        self.base.write(words)
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Transfer<u8> for Spi1Dma<MOSI, MISO, SCK> {
    type Error = Error;

    #[inline(always)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi1
            .base
            .transfer_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Write<u8> for Spi1Dma<MOSI, MISO, SCK> {
    type Error = Error;

    #[inline(always)]
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi1
            .base
            .write_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
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
        self.base.transfer(words)
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
        self.base.write(words)
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Transfer<u8> for Spi2Dma<MOSI, MISO, SCK> {
    type Error = Error;

    #[inline(always)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi2
            .base
            .transfer_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Write<u8> for Spi2Dma<MOSI, MISO, SCK> {
    type Error = Error;

    #[inline(always)]
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi2
            .base
            .write_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
    }
}

impl embedded_hal::blocking::spi::Transfer<u8> for Spi3 {
    type Error = Error;

    #[inline(always)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.base.transfer(words)
    }
}

impl embedded_hal::blocking::spi::Write<u8> for Spi3 {
    type Error = Error;

    #[inline(always)]
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.base.write(words)
    }
}

impl embedded_hal::blocking::spi::Transfer<u8> for Spi3Dma {
    type Error = Error;

    #[inline(always)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi3
            .base
            .transfer_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
    }
}

impl embedded_hal::blocking::spi::Write<u8> for Spi3Dma {
    type Error = Error;

    #[inline(always)]
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi3
            .base
            .write_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
    }
}
