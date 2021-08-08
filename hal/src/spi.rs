//! Serial peripheral interface
#![deny(missing_docs)]

use crate::{
    dma::{self, DmaCh, NoDmaCh},
    gpio,
    pac::{self, SPI1, SPI2, SPI3},
};

use core::{
    ptr::{read_volatile, write_volatile},
    sync::atomic::{compiler_fence, Ordering::SeqCst},
};

pub use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3};

pub use pac::spi1::cr1::BR_A as BaudDiv;

/// SPI errors
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Frame format error
    Framing,
    /// CRC check error
    Crc,
    /// NSS mode fault
    ModeFault,
    /// FIFO overrun
    Overrun,
    /// RX DMA error
    ///
    /// This can only occur on SPI transfers that use the RX DMA.
    RxDma,
    /// TX DMA error
    ///
    /// This can only occur on SPI transfers that use the TX DMA.
    TxDma,
}

const DR_OFFSET: usize = 0xC;
const SPI1_BASE: usize = 0x4001_3000;
const SPI2_BASE: usize = 0x4000_3800;
const SPI3_BASE: usize = 0x5801_0000;

#[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
async fn aio_write_with_dma<BASE: SpiBase>(
    spi: &mut BASE,
    tx_dma: &mut DmaCh,
    rx_dma: &mut DmaCh,
    words: &[u8],
) -> Result<(), Error> {
    const RX_CR: dma::Cr = dma::Cr::RESET
        .set_dir_from_periph()
        .set_xfer_cpl_irq_en(true)
        .set_xfer_err_irq_en(true)
        .set_mem_inc(false)
        .set_enable(true);
    const TX_CR: dma::Cr = dma::Cr::RESET
        .set_dir_from_mem()
        .set_mem_inc(true)
        .set_enable(true);

    let garbage: [u8; 1] = [0];

    rx_dma.set_mem_addr(garbage.as_ptr() as u32);
    tx_dma.set_mem_addr(words.as_ptr() as u32);

    let ndt: u32 = words.len() as u32;
    rx_dma.set_num_data_xfer(ndt);
    tx_dma.set_num_data_xfer(ndt);

    // RX MUST come before TX
    rx_dma.set_cr(RX_CR);
    tx_dma.set_cr(TX_CR);

    // wait for RX DMA to complete
    let rx_dma_result: Result<(), crate::dma::Error> =
        futures::future::poll_fn(|cx| dma::aio::poll(rx_dma.mux_ch(), cx)).await;

    // RX is disabled in ISR
    tx_dma.set_cr(dma::Cr::DISABLE);

    // RX flags are cleared in ISR
    tx_dma.clear_all_flags();

    if rx_dma_result.is_err() {
        Err(Error::RxDma)
    } else {
        spi.status()?;
        if tx_dma.flags() & dma::flags::XFER_ERR != 0 {
            Err(Error::TxDma)
        } else {
            Ok(())
        }
    }
}

#[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
async fn aio_transfer_with_dma<BASE: SpiBase>(
    spi: &mut BASE,
    tx_dma: &mut DmaCh,
    rx_dma: &mut DmaCh,
    words: &mut [u8],
) -> Result<(), Error> {
    const RX_CR: dma::Cr = dma::Cr::RESET
        .set_dir_from_periph()
        .set_xfer_cpl_irq_en(true)
        .set_xfer_err_irq_en(true)
        .set_mem_inc(true)
        .set_enable(true);
    const TX_CR: dma::Cr = dma::Cr::RESET
        .set_dir_from_mem()
        .set_mem_inc(true)
        .set_enable(true);

    rx_dma.set_mem_addr(words.as_ptr() as u32);
    tx_dma.set_mem_addr(words.as_ptr() as u32);

    let ndt: u32 = words.len() as u32;
    rx_dma.set_num_data_xfer(ndt);
    tx_dma.set_num_data_xfer(ndt);

    // RX MUST come before TX
    rx_dma.set_cr(RX_CR);
    tx_dma.set_cr(TX_CR);

    // wait for RX DMA to complete
    let rx_dma_result: Result<(), crate::dma::Error> =
        futures::future::poll_fn(|cx| dma::aio::poll(rx_dma.mux_ch(), cx)).await;

    // RX is disabled in ISR
    tx_dma.set_cr(dma::Cr::DISABLE);

    // RX flags are cleared in ISR
    tx_dma.clear_all_flags();

    if rx_dma_result.is_err() {
        Err(Error::RxDma)
    } else {
        spi.status()?;
        if tx_dma.flags() & dma::flags::XFER_ERR != 0 {
            Err(Error::TxDma)
        } else {
            Ok(())
        }
    }
}

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

    fn one_time_dma_setup(&mut self, tx_dma: &mut DmaCh, rx_dma: &mut DmaCh) {
        self.cr2()
            .write(|w| w.txdmaen().enabled().rxdmaen().enabled().frxth().quarter());

        tx_dma.set_cr(dma::Cr::DISABLE);
        rx_dma.set_cr(dma::Cr::DISABLE);

        rx_dma.clear_all_flags();
        tx_dma.clear_all_flags();

        rx_dma.set_periph_addr(Self::DR as u32);
        tx_dma.set_periph_addr(Self::DR as u32);

        rx_dma.set_mux_cr_reqid(Self::DMA_RX_ID);
        tx_dma.set_mux_cr_reqid(Self::DMA_TX_ID);

        unsafe { rx_dma.unmask_irq() };
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
        const RX_CR: dma::Cr = dma::Cr::RESET
            .set_dir_from_periph()
            .set_mem_inc(false)
            .set_enable(true);
        const TX_CR: dma::Cr = dma::Cr::RESET
            .set_dir_from_mem()
            .set_mem_inc(true)
            .set_enable(true);

        let garbage: [u8; 1] = [0];

        rx_dma.set_mem_addr(garbage.as_ptr() as u32);
        tx_dma.set_mem_addr(words.as_ptr() as u32);

        let ndt: u32 = words.len() as u32;
        rx_dma.set_num_data_xfer(ndt);
        tx_dma.set_num_data_xfer(ndt);

        // RX MUST come before TX
        rx_dma.set_cr(RX_CR);
        tx_dma.set_cr(TX_CR);

        let ret: Result<(), Error> = loop {
            let status = self.status()?;
            let tx_dma_flags: u8 = tx_dma.flags();
            let rx_dma_flags: u8 = rx_dma.flags();
            if tx_dma_flags & dma::flags::XFER_ERR != 0 {
                break Err(Error::TxDma);
            }
            if rx_dma_flags & dma::flags::XFER_ERR != 0 {
                break Err(Error::RxDma);
            }
            if status.bsy().is_not_busy()
                && tx_dma_flags & dma::flags::XFER_CPL != 0
                && rx_dma_flags & dma::flags::XFER_CPL != 0
            {
                break Ok(());
            }
        };

        // TX must come before RX
        tx_dma.set_cr(dma::Cr::DISABLE);
        rx_dma.set_cr(dma::Cr::DISABLE);

        rx_dma.clear_all_flags();
        tx_dma.clear_all_flags();

        ret
    }

    fn transfer_with_dma<'w>(
        &mut self,
        tx_dma: &mut DmaCh,
        rx_dma: &mut DmaCh,
        words: &'w mut [u8],
    ) -> Result<&'w [u8], Error> {
        const RX_CR: dma::Cr = dma::Cr::RESET
            .set_dir_from_periph()
            .set_mem_inc(true)
            .set_enable(true);
        const TX_CR: dma::Cr = dma::Cr::RESET
            .set_dir_from_mem()
            .set_mem_inc(true)
            .set_enable(true);

        rx_dma.set_mem_addr(words.as_ptr() as u32);
        tx_dma.set_mem_addr(words.as_ptr() as u32);

        rx_dma.set_periph_addr(Self::DR as u32);
        tx_dma.set_periph_addr(Self::DR as u32);

        let ndt: u32 = words.len() as u32;
        rx_dma.set_num_data_xfer(ndt);
        tx_dma.set_num_data_xfer(ndt);

        // RX MUST come before TX
        rx_dma.set_cr(RX_CR);
        tx_dma.set_cr(TX_CR);

        let ret: Result<&'w [u8], Error> = loop {
            let status = self.status()?;
            let tx_dma_flags: u8 = tx_dma.flags();
            let rx_dma_flags: u8 = rx_dma.flags();
            if tx_dma_flags & dma::flags::XFER_ERR != 0 {
                break Err(Error::TxDma);
            }
            if rx_dma_flags & dma::flags::XFER_ERR != 0 {
                break Err(Error::RxDma);
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

        rx_dma.clear_all_flags();
        tx_dma.clear_all_flags();

        // tell the compiler the memory in dst may have changed
        compiler_fence(SeqCst);
        // tell the cpu the memory in dst may have changed
        cortex_m::asm::dmb();

        ret
    }
}

macro_rules! impl_spi_base_for {
    ($name:ident, $dr:expr, $tx_id:expr, $rx_id:expr) => {
        impl SpiBase for $name {
            const DR: usize = $dr;
            const DMA_TX_ID: u8 = $tx_id as u8;
            const DMA_RX_ID: u8 = $rx_id as u8;
            #[inline(always)]
            fn cr1(&self) -> &pac::spi1::CR1 {
                &self.cr1
            }
            #[inline(always)]
            fn cr2(&self) -> &pac::spi1::CR2 {
                &self.cr2
            }
            #[inline(always)]
            fn sr(&self) -> &pac::spi1::SR {
                &self.sr
            }
        }
    };
}

use pac::dmamux::c0cr::DMAREQ_ID_A::{
    SPI1_RX_DMA, SPI1_TX_DMA, SPI2_RX_DMA, SPI2_TX_DMA, SUBGHZSPI_RX, SUBGHZSPI_TX,
};

impl_spi_base_for!(SPI1, SPI1_BASE + DR_OFFSET, SPI1_TX_DMA, SPI1_RX_DMA);
impl_spi_base_for!(SPI2, SPI2_BASE + DR_OFFSET, SPI2_TX_DMA, SPI2_RX_DMA);
impl_spi_base_for!(SPI3, SPI3_BASE + DR_OFFSET, SUBGHZSPI_TX, SUBGHZSPI_RX);

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
pub struct Spi1<MOSI, MISO, SCK, DMA> {
    base: pac::SPI1,
    mosi: MOSI,
    miso: MISO,
    sck: SCK,
    tx_dma: DMA,
    rx_dma: DMA,
}

/// SPI2 driver
#[derive(Debug)]
pub struct Spi2<MOSI, MISO, SCK, DMA> {
    base: pac::SPI2,
    mosi: MOSI,
    miso: MISO,
    sck: SCK,
    tx_dma: DMA,
    rx_dma: DMA,
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
pub struct Spi3<DMA> {
    base: pac::SPI3,
    tx_dma: DMA,
    rx_dma: DMA,
}

impl<MOSI, MISO, SCK, DMA> Spi1<MOSI, MISO, SCK, DMA>
where
    MOSI: gpio::sealed::Spi1Mosi,
    MISO: gpio::sealed::Spi1Miso,
    SCK: gpio::sealed::Spi1Sck,
{
    /// Disable the SPI1 clock
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb2enr.modify(|_, w| w.spi1en().disabled());
    }

    /// Enable the SPI1 clock
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb2enr.modify(|_, w| w.spi1en().enabled());
        rcc.apb2enr.read(); // delay after an RCC peripheral clock enabling
    }

    fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb2rstr.modify(|_, w| w.spi1rst().set_bit());
        rcc.apb2rstr.modify(|_, w| w.spi1rst().clear_bit());
    }

    fn init_no_dma(
        spi1: &mut pac::SPI1,
        mosi: &mut MOSI,
        miso: &mut MISO,
        sck: &mut SCK,
        mode: Mode,
        div: BaudDiv,
        rcc: &mut pac::RCC,
    ) {
        Self::enable_clock(rcc);
        Self::pulse_reset(rcc);
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
    }
}

impl<MOSI, MISO, SCK> Spi1<MOSI, MISO, SCK, NoDmaCh>
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
    ///     dma::NoDmaCh,
    ///     gpio::{pins, PortA},
    ///     pac,
    ///     spi::{BaudDiv, Spi1, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let spi: Spi1<pins::A7, pins::A6, pins::A5, NoDmaCh> = Spi1::new(
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
        mut spi1: pac::SPI1,
        mut mosi: MOSI,
        mut miso: MISO,
        mut sck: SCK,
        mode: Mode,
        div: BaudDiv,
        rcc: &mut pac::RCC,
    ) -> Spi1<MOSI, MISO, SCK, NoDmaCh> {
        Self::init_no_dma(&mut spi1, &mut mosi, &mut miso, &mut sck, mode, div, rcc);

        Spi1 {
            base: spi1,
            mosi,
            miso,
            sck,
            tx_dma: NoDmaCh::new(),
            rx_dma: NoDmaCh::new(),
        }
    }

    /// Free the GPIOs from the SPI driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     dma::NoDmaCh,
    ///     gpio::{pins, PortA},
    ///     pac,
    ///     spi::{BaudDiv, Spi1, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let spi: Spi1<pins::A7, pins::A6, pins::A5, NoDmaCh> = Spi1::new(
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
        (self.base, self.mosi, self.miso, self.sck)
    }
}

impl<MOSI, MISO, SCK> Spi1<MOSI, MISO, SCK, DmaCh>
where
    MOSI: gpio::sealed::Spi1Mosi,
    MISO: gpio::sealed::Spi1Miso,
    SCK: gpio::sealed::Spi1Sck,
{
    /// Create a new `Spi1` driver with DMA operations.
    ///
    /// This will enable clocks and reset the SPI1 peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     dma::{AllDma, DmaCh},
    ///     gpio::{pins, PortA},
    ///     pac,
    ///     spi::{BaudDiv, Spi1, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    ///
    /// let spi: Spi1<pins::A7, pins::A6, pins::A5, DmaCh> = Spi1::new_with_dma(
    ///     dp.SPI1,
    ///     pa.pa7,
    ///     pa.pa6,
    ///     pa.pa5,
    ///     dma.d1c1,
    ///     dma.d2c1,
    ///     MODE_0,
    ///     BaudDiv::DIV4,
    ///     &mut dp.RCC,
    /// );
    /// ```
    #[allow(clippy::too_many_arguments)]
    pub fn new_with_dma(
        mut spi1: pac::SPI1,
        mut mosi: MOSI,
        mut miso: MISO,
        mut sck: SCK,
        mut tx_dma: DmaCh,
        mut rx_dma: DmaCh,
        mode: Mode,
        div: BaudDiv,
        rcc: &mut pac::RCC,
    ) -> Spi1<MOSI, MISO, SCK, DmaCh> {
        Self::init_no_dma(&mut spi1, &mut mosi, &mut miso, &mut sck, mode, div, rcc);
        spi1.one_time_dma_setup(&mut tx_dma, &mut rx_dma);
        Spi1 {
            base: spi1,
            mosi,
            miso,
            sck,
            tx_dma,
            rx_dma,
        }
    }

    /// Free the GPIOs and DMA channels from the SPI driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     dma::{AllDma, DmaCh},
    ///     gpio::{pins, PortA},
    ///     pac,
    ///     spi::{BaudDiv, Spi1, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    ///
    /// let spi: Spi1<pins::A7, pins::A6, pins::A5, DmaCh> = Spi1::new_with_dma(
    ///     dp.SPI1,
    ///     pa.pa7,
    ///     pa.pa6,
    ///     pa.pa5,
    ///     dma.d1c1,
    ///     dma.d2c1,
    ///     MODE_0,
    ///     BaudDiv::DIV4,
    ///     &mut dp.RCC,
    /// );
    ///
    /// // use SPI bus
    ///
    /// let (spi1, pa7, pa6, pa5, tx_dma, rx_dma) = spi.free();
    /// ```
    pub fn free(self) -> (pac::SPI1, MOSI, MISO, SCK, DmaCh, DmaCh) {
        (
            self.base,
            self.mosi,
            self.miso,
            self.sck,
            self.tx_dma,
            self.rx_dma,
        )
    }

    /// Asynchronously write with the DMA.
    ///
    /// In a perfect world this would be in an async trait, unfortunately
    /// async traits require more workarounds than I am comforatable with right
    /// now.
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    pub async fn aio_write_with_dma(&mut self, words: &[u8]) -> Result<(), Error> {
        aio_write_with_dma(&mut self.base, &mut self.tx_dma, &mut self.rx_dma, words).await
    }

    /// Asynchronously transfer with the DMA.
    ///
    /// In a perfect world this would be in an async trait, unfortunately
    /// async traits require more workarounds than I am comforatable with right
    /// now.
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    pub async fn aio_transfer_with_dma(&mut self, words: &mut [u8]) -> Result<(), Error> {
        aio_transfer_with_dma(&mut self.base, &mut self.tx_dma, &mut self.rx_dma, words).await
    }
}

impl<MOSI, MISO, SCK, DMA> Spi2<MOSI, MISO, SCK, DMA>
where
    MOSI: gpio::sealed::Spi2Mosi,
    MISO: gpio::sealed::Spi2Miso,
    SCK: gpio::sealed::Spi2Sck,
{
    /// Disable the SPI2 clock
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr1.modify(|_, w| w.spi2s2en().disabled());
    }

    /// Enable the SPI2 clock
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr1.modify(|_, w| w.spi2s2en().enabled());
        rcc.apb1enr1.read(); // delay after an RCC peripheral clock enabling
    }

    fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb1rstr1.modify(|_, w| w.spi2s2rst().set_bit());
        rcc.apb1rstr1.modify(|_, w| w.spi2s2rst().clear_bit());
    }

    fn init_no_dma(
        spi2: &mut pac::SPI2,
        mosi: &mut MOSI,
        miso: &mut MISO,
        sck: &mut SCK,
        mode: Mode,
        div: BaudDiv,
        rcc: &mut pac::RCC,
    ) {
        Self::enable_clock(rcc);
        Self::pulse_reset(rcc);
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
    }
}

impl<MOSI, MISO, SCK> Spi2<MOSI, MISO, SCK, NoDmaCh>
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
    ///     dma::NoDmaCh,
    ///     gpio::{pins, PortB},
    ///     pac,
    ///     spi::{BaudDiv, Spi2, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pb: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let spi: Spi2<pins::B15, pins::B14, pins::B13, NoDmaCh> = Spi2::new(
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
        mut spi2: pac::SPI2,
        mut mosi: MOSI,
        mut miso: MISO,
        mut sck: SCK,
        mode: Mode,
        div: BaudDiv,
        rcc: &mut pac::RCC,
    ) -> Spi2<MOSI, MISO, SCK, NoDmaCh> {
        Self::init_no_dma(&mut spi2, &mut mosi, &mut miso, &mut sck, mode, div, rcc);
        Spi2 {
            base: spi2,
            mosi,
            miso,
            sck,
            tx_dma: NoDmaCh::new(),
            rx_dma: NoDmaCh::new(),
        }
    }

    /// Free the GPIOs from the SPI driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     dma::NoDmaCh,
    ///     gpio::{pins, PortB},
    ///     pac,
    ///     spi::{BaudDiv, Spi2, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pb: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let spi: Spi2<pins::B15, pins::B14, pins::B13, NoDmaCh> = Spi2::new(
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
        (self.base, self.mosi, self.miso, self.sck)
    }
}

impl<MOSI, MISO, SCK> Spi2<MOSI, MISO, SCK, DmaCh>
where
    MOSI: gpio::sealed::Spi2Mosi,
    MISO: gpio::sealed::Spi2Miso,
    SCK: gpio::sealed::Spi2Sck,
{
    /// Create a new `Spi2` driver with DMA operations.
    ///
    /// This will enable clocks and reset the SPI2 peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     dma::{AllDma, DmaCh},
    ///     gpio::{pins, PortB},
    ///     pac,
    ///     spi::{BaudDiv, Spi2, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pb: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    ///
    /// let spi: Spi2<pins::B15, pins::B14, pins::B13, DmaCh> = Spi2::new_with_dma(
    ///     dp.SPI2,
    ///     pb.pb15,
    ///     pb.pb14,
    ///     pb.pb13,
    ///     dma.d1c1,
    ///     dma.d2c1,
    ///     MODE_0,
    ///     BaudDiv::DIV4,
    ///     &mut dp.RCC,
    /// );
    /// ```
    #[allow(clippy::too_many_arguments)]
    pub fn new_with_dma(
        mut spi2: pac::SPI2,
        mut mosi: MOSI,
        mut miso: MISO,
        mut sck: SCK,
        mut tx_dma: DmaCh,
        mut rx_dma: DmaCh,
        mode: Mode,
        div: BaudDiv,
        rcc: &mut pac::RCC,
    ) -> Spi2<MOSI, MISO, SCK, DmaCh> {
        Self::init_no_dma(&mut spi2, &mut mosi, &mut miso, &mut sck, mode, div, rcc);
        spi2.one_time_dma_setup(&mut tx_dma, &mut rx_dma);
        Spi2 {
            base: spi2,
            mosi,
            miso,
            sck,
            tx_dma,
            rx_dma,
        }
    }

    /// Free the GPIOs and DMA channels from the SPI driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     dma::{AllDma, DmaCh},
    ///     gpio::{pins, PortB},
    ///     pac,
    ///     spi::{BaudDiv, Spi2, MODE_0},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let pb: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    ///
    /// let spi: Spi2<pins::B15, pins::B14, pins::B13, DmaCh> = Spi2::new_with_dma(
    ///     dp.SPI2,
    ///     pb.pb15,
    ///     pb.pb14,
    ///     pb.pb13,
    ///     dma.d1c1,
    ///     dma.d2c1,
    ///     MODE_0,
    ///     BaudDiv::DIV4,
    ///     &mut dp.RCC,
    /// );
    ///
    /// // use SPI bus
    ///
    /// let (spi2, pb15, pb14, pb13, tx_dma, rx_dma) = spi.free();
    /// ```
    pub fn free(self) -> (pac::SPI2, MOSI, MISO, SCK, DmaCh, DmaCh) {
        (
            self.base,
            self.mosi,
            self.miso,
            self.sck,
            self.tx_dma,
            self.rx_dma,
        )
    }

    /// Asynchronously write with the DMA.
    ///
    /// In a perfect world this would be in an async trait, unfortunately
    /// async traits require more workarounds than I am comforatable with right
    /// now.
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    pub async fn aio_write_with_dma(&mut self, words: &[u8]) -> Result<(), Error> {
        aio_write_with_dma(&mut self.base, &mut self.tx_dma, &mut self.rx_dma, words).await
    }

    /// Asynchronously transfer with the DMA.
    ///
    /// In a perfect world this would be in an async trait, unfortunately
    /// async traits require more workarounds than I am comforatable with right
    /// now.
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    pub async fn aio_transfer_with_dma(&mut self, words: &mut [u8]) -> Result<(), Error> {
        aio_transfer_with_dma(&mut self.base, &mut self.tx_dma, &mut self.rx_dma, words).await
    }
}

#[allow(missing_docs)] // struct is hidden
impl<DMA> Spi3<DMA> {
    /// Disable the SPI3 (SubGHz SPI) clock
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb3enr.modify(|_, w| w.subghzspien().disabled());
    }

    /// Enable the SPI3 (SubGHz SPI) clock
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb3enr.modify(|_, w| w.subghzspien().enabled());
        rcc.apb3enr.read(); // delay after an RCC peripheral clock enabling
    }

    /// Pulse the SPI3 (SubGHz SPI) reset
    pub fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb3rstr.write(|w| w.subghzspirst().set_bit());
        rcc.apb3rstr.write(|w| w.subghzspirst().clear_bit());
    }

    fn init_no_dma(spi3: &mut pac::SPI3, div: BaudDiv, rcc: &mut pac::RCC) {
        Self::enable_clock(rcc);
        Self::pulse_reset(rcc);

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
    }
}

#[allow(missing_docs)] // struct is hidden
impl Spi3<NoDmaCh> {
    pub fn new(mut spi3: pac::SPI3, div: BaudDiv, rcc: &mut pac::RCC) -> Spi3<NoDmaCh> {
        Self::init_no_dma(&mut spi3, div, rcc);

        Spi3 {
            base: spi3,
            tx_dma: NoDmaCh::new(),
            rx_dma: NoDmaCh::new(),
        }
    }

    #[allow(clippy::missing_safety_doc)]
    pub unsafe fn steal() -> Spi3<NoDmaCh> {
        Spi3 {
            base: pac::Peripherals::steal().SPI3,
            tx_dma: NoDmaCh::new(),
            rx_dma: NoDmaCh::new(),
        }
    }

    pub fn free(self) -> pac::SPI3 {
        self.base
    }
}

#[allow(missing_docs)] // struct is hidden
impl Spi3<DmaCh> {
    pub fn new(
        mut spi3: pac::SPI3,
        mut tx_dma: DmaCh,
        mut rx_dma: DmaCh,
        div: BaudDiv,
        rcc: &mut pac::RCC,
    ) -> Spi3<DmaCh> {
        Self::init_no_dma(&mut spi3, div, rcc);
        spi3.one_time_dma_setup(&mut tx_dma, &mut rx_dma);
        Spi3 {
            base: spi3,
            tx_dma,
            rx_dma,
        }
    }

    #[allow(clippy::missing_safety_doc)]
    pub unsafe fn steal_with_dma(tx_dma: DmaCh, rx_dma: DmaCh) -> Spi3<DmaCh> {
        Spi3 {
            base: pac::Peripherals::steal().SPI3,
            tx_dma,
            rx_dma,
        }
    }

    pub fn free(self) -> (pac::SPI3, DmaCh, DmaCh) {
        (self.base, self.tx_dma, self.rx_dma)
    }

    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    pub async fn aio_write_with_dma(&mut self, words: &[u8]) -> Result<(), Error> {
        aio_write_with_dma(&mut self.base, &mut self.tx_dma, &mut self.rx_dma, words).await
    }

    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    pub async fn aio_transfer_with_dma(&mut self, words: &mut [u8]) -> Result<(), Error> {
        aio_transfer_with_dma(&mut self.base, &mut self.tx_dma, &mut self.rx_dma, words).await
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Transfer<u8> for Spi1<MOSI, MISO, SCK, NoDmaCh>
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

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Write<u8> for Spi1<MOSI, MISO, SCK, NoDmaCh>
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

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Transfer<u8> for Spi1<MOSI, MISO, SCK, DmaCh>
where
    MOSI: gpio::sealed::Spi1Mosi,
    MISO: gpio::sealed::Spi1Miso,
    SCK: gpio::sealed::Spi1Sck,
{
    type Error = Error;

    #[inline(always)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.base
            .transfer_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Write<u8> for Spi1<MOSI, MISO, SCK, DmaCh>
where
    MOSI: gpio::sealed::Spi1Mosi,
    MISO: gpio::sealed::Spi1Miso,
    SCK: gpio::sealed::Spi1Sck,
{
    type Error = Error;

    #[inline(always)]
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.base
            .write_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Transfer<u8> for Spi2<MOSI, MISO, SCK, NoDmaCh>
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

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Write<u8> for Spi2<MOSI, MISO, SCK, NoDmaCh>
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

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Transfer<u8> for Spi2<MOSI, MISO, SCK, DmaCh>
where
    MOSI: gpio::sealed::Spi2Mosi,
    MISO: gpio::sealed::Spi2Miso,
    SCK: gpio::sealed::Spi2Sck,
{
    type Error = Error;

    #[inline(always)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.base
            .transfer_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
    }
}

impl<MOSI, MISO, SCK> embedded_hal::blocking::spi::Write<u8> for Spi2<MOSI, MISO, SCK, DmaCh>
where
    MOSI: gpio::sealed::Spi2Mosi,
    MISO: gpio::sealed::Spi2Miso,
    SCK: gpio::sealed::Spi2Sck,
{
    type Error = Error;

    #[inline(always)]
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.base
            .write_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
    }
}

impl embedded_hal::blocking::spi::Transfer<u8> for Spi3<NoDmaCh> {
    type Error = Error;

    #[inline(always)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.base.transfer(words)
    }
}

impl embedded_hal::blocking::spi::Write<u8> for Spi3<NoDmaCh> {
    type Error = Error;

    #[inline(always)]
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.base.write(words)
    }
}

impl embedded_hal::blocking::spi::Transfer<u8> for Spi3<DmaCh> {
    type Error = Error;

    #[inline(always)]
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.base
            .transfer_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
    }
}

impl embedded_hal::blocking::spi::Write<u8> for Spi3<DmaCh> {
    type Error = Error;

    #[inline(always)]
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.base
            .write_with_dma(&mut self.tx_dma, &mut self.rx_dma, words)
    }
}
