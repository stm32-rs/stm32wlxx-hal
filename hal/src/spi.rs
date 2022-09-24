//! Serial peripheral interface
//!
//! The [`Spi`] struct implement the [`embedded-hal`] traits for transfers.
//!
//! # Constructors
//!
//! There was a minor cartesian explosion when writing this module.
//!
//! | Function                            | Bus | plex         | DMA |
//! |-------------------------------------|-----|--------------|-----|
//! | [`new_spi1_full_duplex`]            | 1   | Full-duplex  | No  |
//! | [`new_spi1_full_duplex_dma`]        | 1   | Full-duplex  | Yes |
//! | [`new_spi1_mosi_simplex`]           | 1   | MOSI-simplex | No  |
//! | [`new_spi1_mosi_simplex_dma`]       | 1   | MOSI-simplex | Yes |
//! | [`new_spi2_full_duplex`]            | 2   | Full-duplex  | No  |
//! | [`new_spi2_full_duplex_dma`]        | 2   | Full-duplex  | Yes |
//! | [`new_spi2_mosi_simplex`]           | 2   | MOSI-simplex | No  |
//! | [`new_spi2_mosi_simplex_dma`]       | 2   | MOSI-simplex | Yes |
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [`new_spi1_full_duplex_dma`]: Spi::new_spi1_full_duplex_dma
//! [`new_spi1_full_duplex`]: Spi::new_spi1_full_duplex
//! [`new_spi1_mosi_simplex_dma`]: Spi::new_spi1_mosi_simplex_dma
//! [`new_spi1_mosi_simplex`]: Spi::new_spi1_mosi_simplex
//! [`new_spi2_full_duplex_dma`]: Spi::new_spi2_full_duplex_dma
//! [`new_spi2_full_duplex`]: Spi::new_spi2_full_duplex
//! [`new_spi2_mosi_simplex_dma`]: Spi::new_spi2_mosi_simplex_dma
//! [`new_spi2_mosi_simplex`]: Spi::new_spi2_mosi_simplex

use crate::{
    dma::{self, DmaCh},
    gpio::sealed::{
        Spi1Miso, Spi1Mosi, Spi1Sck, Spi2Miso, Spi2Mosi, Spi2Sck, SpiMiso, SpiMosi, SpiSck,
    },
    pac::{self, SPI1, SPI2},
};

pub use embedded_hal::{
    blocking::spi::{Transfer, Write},
    spi::{FullDuplex, Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3},
};

use cortex_m::interrupt::CriticalSection;

typestate!(NoSck, "no SCK on a generic SPI structure");
typestate!(NoMosi, "no MOSI on a generic SPI structure");
typestate!(NoMiso, "no MISO on a generic SPI structure");

typestate!(SgSck, "the internal Sub-GHz SCK");
typestate!(SgMosi, "the internal Sub-GHz MOSI");
typestate!(SgMiso, "the internal Sub-GHz MISO");

impl SpiSck for SgSck {}
impl SpiMosi for SgMosi {}
impl SpiMiso for SgMiso {}

use pac::spi1::cr1::BR_A;

/// Baud rate divisors.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum BaudRate {
    /// Source clock / 256
    Div256 = BR_A::Div256 as u8,
    /// Source clock / 128
    Div128 = BR_A::Div128 as u8,
    /// Source clock / 64
    Div64 = BR_A::Div64 as u8,
    /// Source clock / 32
    Div32 = BR_A::Div32 as u8,
    /// Source clock / 16
    Div16 = BR_A::Div16 as u8,
    /// Source clock / 8
    Div8 = BR_A::Div8 as u8,
    /// Source clock / 4
    Div4 = BR_A::Div4 as u8,
    /// Source clock / 2
    Div2 = BR_A::Div2 as u8,
}

impl BaudRate {
    /// Get baud rate divisor.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::spi::BaudRate;
    ///
    /// assert_eq!(BaudRate::Div256.div(), 256);
    /// assert_eq!(BaudRate::Div128.div(), 128);
    /// assert_eq!(BaudRate::Div64.div(), 64);
    /// assert_eq!(BaudRate::Div32.div(), 32);
    /// assert_eq!(BaudRate::Div16.div(), 16);
    /// assert_eq!(BaudRate::Div8.div(), 8);
    /// assert_eq!(BaudRate::Div4.div(), 4);
    /// assert_eq!(BaudRate::Div2.div(), 2);
    /// ```
    pub const fn div(&self) -> u16 {
        match self {
            BaudRate::Div256 => 256,
            BaudRate::Div128 => 128,
            BaudRate::Div64 => 64,
            BaudRate::Div32 => 32,
            BaudRate::Div16 => 16,
            BaudRate::Div8 => 8,
            BaudRate::Div4 => 4,
            BaudRate::Div2 => 2,
        }
    }
}

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

// a space for the RX DMA to transfer into when TX'ing in full-duplex
static mut GARBAGE: [u8; 1] = [0];

use sealed::SpiRegs;
pub(crate) mod sealed {
    use super::{Error, GARBAGE};
    use crate::{
        dma::{self, DmaCh},
        pac,
    };
    use core::{
        ptr::{read_volatile, write_volatile},
        sync::atomic::{compiler_fence, Ordering::SeqCst},
    };

    use pac::dmamux::c0cr::DMAREQ_ID_A::{
        Spi1RxDma, Spi1TxDma, Spi2RxDma, Spi2TxDma, SubghzspiRx, SubghzspiTx,
    };

    const SPI1_BASE: usize = 0x4001_3000;
    const SPI2_BASE: usize = 0x4000_3800;
    const SPI3_BASE: usize = 0x5801_0000;
    const DR_OFFSET: usize = 0xC;

    pub trait SpiRegs: core::ops::Deref<Target = pac::spi1::RegisterBlock> {
        const DR: usize;
        const DMA_TX_ID: u8;
        const DMA_RX_ID: u8;

        fn status(&self) -> Result<pac::spi1::sr::R, Error> {
            let sr = self.deref().sr.read();
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

        fn write_simplex_u8(&mut self, words: &[u8]) -> Result<(), Error> {
            for word in words.iter() {
                self.write_word(*word)?;
            }
            Ok(())
        }

        fn write_simplex_u8_dma<TxDma: DmaCh>(
            &mut self,
            tx_dma: &mut TxDma,
            words: &[u8],
        ) -> Result<(), Error> {
            if words.is_empty() {
                return Ok(());
            }

            const CR: dma::Cr = dma::Cr::RESET
                .set_dir_from_mem()
                .set_mem_inc(true)
                .set_enable(true);

            tx_dma.set_mem_addr(words.as_ptr() as u32);
            tx_dma.set_num_data_xfer(words.len() as u32);
            tx_dma.set_cr(CR);

            let ret: Result<(), Error> = loop {
                let status = self.status()?;
                let tx_dma_flags: u8 = tx_dma.flags();
                if tx_dma_flags & dma::flags::XFER_ERR != 0 {
                    break Err(Error::TxDma);
                }
                if status.bsy().is_not_busy()
                    && status.ftlvl().is_empty()
                    && tx_dma_flags & dma::flags::XFER_CPL != 0
                {
                    break Ok(());
                }
            };

            tx_dma.set_cr(dma::Cr::DISABLE);
            tx_dma.clear_all_flags();

            ret
        }

        fn write_full_duplex_u8(&mut self, words: &[u8]) -> Result<(), Error> {
            for word in words.iter() {
                self.write_word(*word)?;
                let _: u8 = self.read_word()?;
            }
            Ok(())
        }

        fn write_full_duplex_u8_dma<RxDma: DmaCh, TxDma: DmaCh>(
            &mut self,
            rx_dma: &mut RxDma,
            tx_dma: &mut TxDma,
            words: &[u8],
        ) -> Result<(), Error> {
            if words.is_empty() {
                return Ok(());
            }

            const RX_CR: dma::Cr = dma::Cr::RESET
                .set_dir_from_periph()
                .set_mem_inc(false)
                .set_enable(true);
            const TX_CR: dma::Cr = dma::Cr::RESET
                .set_dir_from_mem()
                .set_mem_inc(true)
                .set_enable(true);

            rx_dma.set_mem_addr(unsafe { GARBAGE.as_mut_ptr() } as u32);
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
                    && status.ftlvl().is_empty()
                    && status.frlvl().is_empty()
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

        fn transfer_u8_dma<'w, RxDma: DmaCh, TxDma: DmaCh>(
            &mut self,
            rx_dma: &mut RxDma,
            tx_dma: &mut TxDma,
            words: &'w mut [u8],
        ) -> Result<&'w [u8], Error> {
            if words.is_empty() {
                return Ok(words);
            }

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
                    && status.ftlvl().is_empty()
                    && status.frlvl().is_empty()
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

        fn transfer_u8<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Error> {
            for word in words.iter_mut() {
                self.write_word(*word)?;
                *word = self.read_word()?;
            }
            Ok(words)
        }

        fn nb_read_u8(&mut self) -> nb::Result<u8, Error> {
            if self.status()?.frlvl().is_empty() {
                Err(nb::Error::WouldBlock)
            } else {
                Ok(unsafe { read_volatile(Self::DR as *const u8) })
            }
        }

        fn nb_send_u8(&mut self, word: u8) -> nb::Result<(), Error> {
            if self.status()?.ftlvl().is_full() {
                Err(nb::Error::WouldBlock)
            } else {
                unsafe { write_volatile(Self::DR as *mut u8, word) };
                Ok(())
            }
        }
    }

    impl SpiRegs for pac::SPI1 {
        const DR: usize = SPI1_BASE + DR_OFFSET;
        const DMA_TX_ID: u8 = Spi1TxDma as u8;
        const DMA_RX_ID: u8 = Spi1RxDma as u8;
    }

    impl SpiRegs for pac::SPI2 {
        const DR: usize = SPI2_BASE + DR_OFFSET;
        const DMA_TX_ID: u8 = Spi2TxDma as u8;
        const DMA_RX_ID: u8 = Spi2RxDma as u8;
    }

    impl SpiRegs for pac::SPI3 {
        const DR: usize = SPI3_BASE + DR_OFFSET;
        const DMA_TX_ID: u8 = SubghzspiTx as u8;
        const DMA_RX_ID: u8 = SubghzspiRx as u8;
    }
}

/// SPI 1 and 2 driver.
#[derive(Debug)]
pub struct Spi<SPI, SCK, MISO, MOSI> {
    spi: SPI,
    sck: SCK,
    miso: MISO,
    mosi: MOSI,
}

/// SPI 3 (Sub-GHz) driver.
#[derive(Debug)]
#[doc(hidden)]
pub struct Spi3<MISO, MOSI> {
    spi: pac::SPI3,
    miso: MISO,
    mosi: MOSI,
}

impl<SCK, MISO, MOSI> Spi<pac::SPI1, SCK, MISO, MOSI> {
    /// Reset the SPI.
    ///
    /// # Safety
    ///
    /// 1. The SPI must not be in-use.
    /// 2. You are responsible for setting up the SPI after a reset.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     spi::{NoMiso, NoMosi, NoSck, Spi},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// unsafe { Spi::<pac::SPI1, NoSck, NoMiso, NoMosi>::pulse_reset(&mut dp.RCC) };
    /// ```
    #[inline]
    pub unsafe fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb2rstr.modify(|_, w| w.spi1rst().set_bit());
        rcc.apb2rstr.modify(|_, w| w.spi1rst().clear_bit());
    }

    /// Disable the SPI clock.
    ///
    /// # Safety
    ///
    /// 1. You are responsible for ensuring the SPI is in a state where
    ///    the clock can be disabled without entering an error state.
    /// 2. You cannot use the SPI while the clock is disabled.
    /// 3. You are responsible for re-enabling the clock before resuming
    ///    use of the SPI.
    /// 4. You are responsible for setting up anything that may have lost
    ///    state while the clock was disabled.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     spi::{NoMiso, NoMosi, NoSck, Spi},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// unsafe { Spi::<pac::SPI1, NoSck, NoMiso, NoMosi>::disable_clock(&mut dp.RCC) };
    /// ```
    #[inline]
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb2enr.modify(|_, w| w.spi1en().disabled());
    }

    /// Enable the SPI clock.
    ///
    /// This is done for you in `new_` constructors.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     spi::{NoMiso, NoMosi, NoSck, Spi},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// Spi::<pac::SPI1, NoSck, NoMiso, NoMosi>::enable_clock(&mut dp.RCC);
    /// ```
    #[inline]
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb2enr.modify(|_, w| w.spi1en().enabled());
        rcc.apb2enr.read(); // delay after an RCC peripheral clock enabling
    }
}

impl<SCK, MISO, MOSI> Spi<pac::SPI2, SCK, MISO, MOSI> {
    /// Reset the SPI.
    ///
    /// # Safety
    ///
    /// 1. The SPI must not be in-use.
    /// 2. You are responsible for setting up the SPI after a reset.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     spi::{NoMiso, NoMosi, NoSck, Spi},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// unsafe { Spi::<pac::SPI2, NoSck, NoMiso, NoMosi>::pulse_reset(&mut dp.RCC) };
    /// ```
    #[inline]
    pub unsafe fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb1rstr1.modify(|_, w| w.spi2s2rst().set_bit());
        rcc.apb1rstr1.modify(|_, w| w.spi2s2rst().clear_bit());
    }

    /// Disable the SPI clock.
    ///
    /// # Safety
    ///
    /// 1. You are responsible for ensuring the SPI is in a state where
    ///    the clock can be disabled without entering an error state.
    /// 2. You cannot use the SPI while the clock is disabled.
    /// 3. You are responsible for re-enabling the clock before resuming
    ///    use of the SPI.
    /// 4. You are responsible for setting up anything that may have lost
    ///    state while the clock was disabled.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     spi::{NoMiso, NoMosi, NoSck, Spi},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// unsafe { Spi::<pac::SPI2, NoSck, NoMiso, NoMosi>::disable_clock(&mut dp.RCC) };
    /// ```
    #[inline]
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr1.modify(|_, w| w.spi2s2en().disabled());
    }

    /// Enable the SPI clock.
    ///
    /// This is done for you in `new_` constructors.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     spi::{NoMiso, NoMosi, NoSck, Spi},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// Spi::<pac::SPI2, NoSck, NoMiso, NoMosi>::enable_clock(&mut dp.RCC);
    /// ```
    #[inline]
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb1enr1.modify(|_, w| w.spi2s2en().enabled());
        rcc.apb1enr1.read(); // delay after an RCC peripheral clock enabling
    }
}

#[allow(missing_docs)] // struct is hidden
#[allow(clippy::missing_safety_doc)]
impl<MISO, MOSI> Spi3<MISO, MOSI> {
    #[inline]
    pub unsafe fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.apb3rstr.modify(|_, w| w.subghzspirst().set_bit());
        rcc.apb3rstr.modify(|_, w| w.subghzspirst().clear_bit());
    }

    #[inline]
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.apb3enr.modify(|_, w| w.subghzspien().disabled());
    }

    #[inline]
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.apb3enr.modify(|_, w| w.subghzspien().enabled());
        rcc.apb3enr.read(); // delay after an RCC peripheral clock enabling
    }
}

macro_rules! impl_new_full_duplex {
    ($n:expr) => {
        paste::paste! {
            impl<SCK, MISO, MOSI> Spi<[<SPI $n>], SCK, MISO, MOSI>
            where
                SCK: [<Spi $n Sck>],
                MISO: [<Spi $n Miso>],
                MOSI: [<Spi $n Mosi>],
            {
                /// Create a new full-duplex master.
                ///
                /// # Example
                ///
                /// ```no_run
                /// use stm32wlxx_hal::{
                ///     cortex_m,
                ///     gpio::PortA,
                ///     pac,
                ///     spi::{BaudRate::Div2, Spi, MODE_0},
                /// };
                ///
                /// let mut dp = pac::Peripherals::take().unwrap();
                ///
                /// let pa = PortA::split(dp.GPIOA, &mut dp.RCC);
                /// let spi = cortex_m::interrupt::free(|cs| {
                ///     Spi::new_spi1_full_duplex(
                ///         dp.SPI1,
                ///         (pa.a5, pa.a6, pa.a7),
                ///         MODE_0,
                ///         Div2,
                ///         &mut dp.RCC,
                ///         cs,
                ///     )
                /// });
                /// ```
                pub fn [<new_spi $n _full_duplex>](
                    spi: [<SPI $n>],
                    mut pins: (SCK, MISO, MOSI),
                    mode: Mode,
                    div: BaudRate,
                    rcc: &mut pac::RCC,
                    cs: &CriticalSection,
                ) -> Self {
                    Self::enable_clock(rcc);
                    unsafe { Self::pulse_reset(rcc) };

                    pins.0.[<set_spi $n _sck_af>](cs);
                    pins.1.[<set_spi $n _miso_af>](cs);
                    pins.2.[<set_spi $n _mosi_af>](cs);

                    spi.cr1.write(|w| {
                        w
                            .ssi().set_bit()
                            .ssm().set_bit()
                            .spe().set_bit()
                            .br().bits(div as u8)
                            .mstr().set_bit()
                            .cpol().bit(cpol_from_polarity(mode.polarity))
                            .cpha().bit(cpha_from_phase(mode.phase))
                    });

                    spi.cr2.write(|w| w.frxth().quarter());

                    Self {
                        spi,
                        sck: pins.0,
                        miso: pins.1,
                        mosi: pins.2,
                    }
                }
            }
        }
    };
}

impl_new_full_duplex!(1);
impl_new_full_duplex!(2);

macro_rules! impl_new_full_duplex_dma {
    ($n:expr) => {
        paste::paste! {
            impl<SCK, MISO, MOSI, MISODMA, MOSIDMA> Spi<[<SPI $n>], SCK, (MISO, MISODMA), (MOSI, MOSIDMA)>
            where
                SCK: [<Spi $n Sck>],
                MISO: [<Spi $n Miso>],
                MOSI: [<Spi $n Mosi>],
                MISODMA: DmaCh,
                MOSIDMA: DmaCh,
            {
                /// Create a new full-duplex master with DMA transfers.
                ///
                /// # Example
                ///
                /// ```no_run
                /// use stm32wlxx_hal::{
                ///     cortex_m,
                ///     dma::AllDma,
                ///     gpio::PortA,
                ///     pac,
                ///     spi::{BaudRate::Div2, Spi, MODE_0},
                /// };
                ///
                /// let mut dp = pac::Peripherals::take().unwrap();
                ///
                /// let dma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
                /// let pa = PortA::split(dp.GPIOA, &mut dp.RCC);
                /// let spi = cortex_m::interrupt::free(|cs| {
                ///     Spi::new_spi1_full_duplex_dma(
                ///         dp.SPI1,
                ///         (pa.a5, pa.a6, pa.a7),
                ///         (dma.d1.c1, dma.d1.c2),
                ///         MODE_0,
                ///         Div2,
                ///         &mut dp.RCC,
                ///         cs,
                ///     )
                /// });
                /// ```
                pub fn [<new_spi $n _full_duplex_dma>](
                    spi: [<SPI $n>],
                    mut pins: (SCK, MISO, MOSI),
                    mut dmas: (MISODMA, MOSIDMA),
                    mode: Mode,
                    div: BaudRate,
                    rcc: &mut pac::RCC,
                    cs: &CriticalSection,
                ) -> Self {
                    Self::enable_clock(rcc);
                    unsafe { Self::pulse_reset(rcc) };

                    pins.0.[<set_spi $n _sck_af>](cs);
                    pins.1.[<set_spi $n _miso_af>](cs);
                    pins.2.[<set_spi $n _mosi_af>](cs);

                    spi.cr1.write(|w| {
                        w
                            .ssi().set_bit()
                            .ssm().set_bit()
                            .spe().set_bit()
                            .br().bits(div as u8)
                            .mstr().set_bit()
                            .cpol().bit(cpol_from_polarity(mode.polarity))
                            .cpha().bit(cpha_from_phase(mode.phase))
                    });

                    dmas.0.set_cr(dma::Cr::DISABLE);
                    dmas.0.clear_all_flags();
                    dmas.0.set_periph_addr([<SPI $n>]::DR as u32);
                    dmas.0.set_mux_cr_reqid([<SPI $n>]::DMA_RX_ID);

                    dmas.1.set_cr(dma::Cr::DISABLE);
                    dmas.1.clear_all_flags();
                    dmas.1.set_periph_addr([<SPI $n>]::DR as u32);
                    dmas.1.set_mux_cr_reqid([<SPI $n>]::DMA_TX_ID);

                    spi.cr2
                        .write(|w| w.txdmaen().enabled().rxdmaen().enabled().frxth().quarter());

                    Self {
                        spi,
                        sck: pins.0,
                        miso: (pins.1, dmas.0),
                        mosi: (pins.2, dmas.1),
                    }
                }
            }
        }
    };
}

impl_new_full_duplex_dma!(1);
impl_new_full_duplex_dma!(2);

macro_rules! impl_new_mosi_simplex {
    ($n:expr) => {
        paste::paste! {
            impl<SCK, MOSI> Spi<[<SPI $n>], SCK, NoMiso, MOSI>
            where
                SCK: [<Spi $n Sck>],
                MOSI: [<Spi $n Mosi>],
            {
                /// Create a new MOSI-simplex master.
                ///
                /// # Example
                ///
                /// ```no_run
                /// use stm32wlxx_hal::{
                ///     cortex_m,
                ///     gpio::PortA,
                ///     pac,
                ///     spi::{BaudRate::Div2, Spi, MODE_0},
                /// };
                ///
                /// let mut dp = pac::Peripherals::take().unwrap();
                ///
                /// let pa = PortA::split(dp.GPIOA, &mut dp.RCC);
                /// let spi = cortex_m::interrupt::free(|cs| {
                ///     Spi::new_spi1_mosi_simplex(
                ///         dp.SPI1,
                ///         (pa.a5, pa.a7),
                ///         MODE_0,
                ///         Div2,
                ///         &mut dp.RCC,
                ///         cs,
                ///     )
                /// });
                /// ```
                pub fn [<new_spi $n _mosi_simplex>](
                    spi: [<SPI $n>],
                    mut pins: (SCK, MOSI),
                    mode: Mode,
                    div: BaudRate,
                    rcc: &mut pac::RCC,
                    cs: &CriticalSection,
                ) -> Self {
                    Self::enable_clock(rcc);
                    unsafe { Self::pulse_reset(rcc) };

                    pins.0.[<set_spi $n _sck_af>](cs);
                    pins.1.[<set_spi $n _mosi_af>](cs);

                    spi.cr1.write(|w|
                        w
                            .bidimode().set_bit()
                            .bidioe().set_bit()
                            .ssi().set_bit()
                            .ssm().set_bit()
                            .spe().set_bit()
                            .br().bits(div as u8)
                            .mstr().set_bit()
                            .cpol().bit(cpol_from_polarity(mode.polarity))
                            .cpha().bit(cpha_from_phase(mode.phase))
                    );

                    spi.cr2.write(|w| w.frxth().quarter());

                    Self {
                        spi,
                        sck: pins.0,
                        miso: NoMiso::new(),
                        mosi: pins.1,
                    }
                }
            }
        }
    };
}

impl_new_mosi_simplex!(1);
impl_new_mosi_simplex!(2);

macro_rules! impl_new_mosi_simplex_dma {
    ($n:expr) => {
        paste::paste! {
            impl<SCK, MOSI, MOSIDMA> Spi<[<SPI $n>], SCK, NoMiso, (MOSI, MOSIDMA)>
            where
                SCK: [<Spi $n Sck>],
                MOSI: [<Spi $n Mosi>],
                MOSIDMA: DmaCh,
            {
                /// Create a new MOSI-simplex master with DMA transfers.
                ///
                /// # Example
                ///
                /// ```no_run
                /// use stm32wlxx_hal::{
                ///     cortex_m,
                ///     dma::AllDma,
                ///     gpio::PortA,
                ///     pac,
                ///     spi::{BaudRate::Div2, Spi, MODE_0},
                /// };
                ///
                /// let mut dp = pac::Peripherals::take().unwrap();
                ///
                /// let dma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
                /// let pa = PortA::split(dp.GPIOA, &mut dp.RCC);
                /// let spi = cortex_m::interrupt::free(|cs| {
                ///     Spi::new_spi1_mosi_simplex_dma(
                ///         dp.SPI1,
                ///         (pa.a5, pa.a7),
                ///         dma.d1.c1,
                ///         MODE_0,
                ///         Div2,
                ///         &mut dp.RCC,
                ///         cs,
                ///     )
                /// });
                /// ```
                pub fn [<new_spi $n _mosi_simplex_dma>](
                    spi: [<SPI $n>],
                    mut pins: (SCK, MOSI),
                    mut dma: MOSIDMA,
                    mode: Mode,
                    div: BaudRate,
                    rcc: &mut pac::RCC,
                    cs: &CriticalSection,
                ) -> Self {
                    Self::enable_clock(rcc);
                    unsafe { Self::pulse_reset(rcc) };

                    pins.0.[<set_spi $n _sck_af>](cs);
                    pins.1.[<set_spi $n _mosi_af>](cs);

                    spi.cr1.write(|w|
                        w
                            .bidimode().set_bit()
                            .bidioe().set_bit()
                            .ssi().set_bit()
                            .ssm().set_bit()
                            .spe().set_bit()
                            .br().bits(div as u8)
                            .mstr().set_bit()
                            .cpol().bit(cpol_from_polarity(mode.polarity))
                            .cpha().bit(cpha_from_phase(mode.phase))
                    );

                    dma.set_cr(dma::Cr::DISABLE);
                    dma.clear_all_flags();
                    dma.set_periph_addr([<SPI $n>]::DR as u32);
                    dma.set_mux_cr_reqid([<SPI $n>]::DMA_TX_ID);

                    spi.cr2.write(|w| w.txdmaen().enabled().frxth().quarter());

                    Self {
                        spi,
                        sck: pins.0,
                        miso: NoMiso::new(),
                        mosi: (pins.1, dma),
                    }
                }
            }
        }
    };
}

impl_new_mosi_simplex_dma!(1);
impl_new_mosi_simplex_dma!(2);

macro_rules! impl_new_miso_simplex {
    ($n:expr) => {
        paste::paste! {
            impl<SCK, MISO> Spi<[<SPI $n>], SCK, MISO, NoMosi>
            where
                SCK: [<Spi $n Sck>],
                MISO: [<Spi $n Miso>],
            {
                /// Create a new MOSI-simplex slave.
                ///
                /// # Example
                ///
                /// ```no_run
                /// use stm32wlxx_hal::{
                ///     cortex_m,
                ///     gpio::PortA,
                ///     pac,
                ///     spi::{Spi, MODE_0},
                /// };
                ///
                /// let mut dp = pac::Peripherals::take().unwrap();
                ///
                /// let pa = PortA::split(dp.GPIOA, &mut dp.RCC);
                /// let spi = cortex_m::interrupt::free(|cs| {
                ///     Spi::new_spi1_miso_simplex_slave(
                ///         dp.SPI1,
                ///         (pa.a5, pa.a6),
                ///         MODE_0,
                ///         &mut dp.RCC,
                ///         cs,
                ///     )
                /// });
                /// ```
                pub fn [<new_spi $n _miso_simplex_slave>](
                    spi: [<SPI $n>],
                    mut pins: (SCK, MISO),
                    mode: Mode,
                    rcc: &mut pac::RCC,
                    cs: &CriticalSection,
                ) -> Self {
                    Self::enable_clock(rcc);
                    unsafe { Self::pulse_reset(rcc) };

                    pins.0.[<set_spi $n _sck_af>](cs);
                    pins.1.[<set_spi $n _miso_af>](cs);

                    spi.cr1.write(|w| {
                        w
                            .bidimode().set_bit()
                            .bidioe().set_bit()
                            .ssi().set_bit()
                            .ssm().set_bit()
                            .spe().set_bit()
                            .cpol().bit(cpol_from_polarity(mode.polarity))
                            .cpha().bit(cpha_from_phase(mode.phase))
                    });

                    spi.cr2.write(|w| w.frxth().quarter());

                    Self {
                        spi,
                        sck: pins.0,
                        miso: pins.1,
                        mosi: NoMosi::new(),
                    }
                }
            }
        }
    };
}

impl_new_miso_simplex!(1);
impl_new_miso_simplex!(2);

macro_rules! impl_new_miso_simplex_dma {
    ($n:expr) => {
        paste::paste! {
            impl<SCK, MISO, MISODMA> Spi<[<SPI $n>], SCK, (MISO, MISODMA), NoMosi>
            where
                SCK: [<Spi $n Sck>],
                MISO: [<Spi $n Miso>],
                MISODMA: DmaCh,
            {
                /// Create a new MISO-simplex slave with DMA transfers.
                ///
                /// # Example
                ///
                /// ```no_run
                /// use stm32wlxx_hal::{
                ///     cortex_m,
                ///     dma::AllDma,
                ///     gpio::PortA,
                ///     pac,
                ///     spi::{Spi, MODE_0},
                /// };
                ///
                /// let mut dp = pac::Peripherals::take().unwrap();
                ///
                /// let dma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
                /// let pa = PortA::split(dp.GPIOA, &mut dp.RCC);
                /// let spi = cortex_m::interrupt::free(|cs| {
                ///     Spi::new_spi1_miso_simplex_slave_dma(
                ///         dp.SPI1,
                ///         (pa.a5, pa.a6),
                ///         dma.d1.c1,
                ///         MODE_0,
                ///         &mut dp.RCC,
                ///         cs,
                ///     )
                /// });
                /// ```
                pub fn [<new_spi $n _miso_simplex_slave_dma>](
                    spi: [<SPI $n>],
                    mut pins: (SCK, MISO),
                    mut dma: MISODMA,
                    mode: Mode,
                    rcc: &mut pac::RCC,
                    cs: &CriticalSection,
                ) -> Self {
                    Self::enable_clock(rcc);
                    unsafe { Self::pulse_reset(rcc) };

                    pins.0.[<set_spi $n _sck_af>](cs);
                    pins.1.[<set_spi $n _miso_af>](cs);

                    spi.cr1.write(|w| {
                        w
                            .bidimode().set_bit()
                            .bidioe().set_bit()
                            .ssi().set_bit()
                            .ssm().set_bit()
                            .spe().set_bit()
                            .cpol().bit(cpol_from_polarity(mode.polarity))
                            .cpha().bit(cpha_from_phase(mode.phase))
                    });

                    dma.set_cr(dma::Cr::DISABLE);
                    dma.clear_all_flags();
                    dma.set_periph_addr([<SPI $n>]::DR as u32);
                    dma.set_mux_cr_reqid([<SPI $n>]::DMA_TX_ID);

                    spi.cr2.write(|w| w.txdmaen().enabled().frxth().quarter());

                    Self {
                        spi,
                        sck: pins.0,
                        miso: (pins.1, dma),
                        mosi: NoMosi::new(),
                    }
                }
            }
        }
    };
}

impl_new_miso_simplex_dma!(1);
impl_new_miso_simplex_dma!(2);

impl Spi3<SgMiso, SgMosi> {
    pub(crate) fn new(spi: pac::SPI3, div: BaudRate, rcc: &mut pac::RCC) -> Self {
        Self::enable_clock(rcc);
        unsafe { Self::pulse_reset(rcc) };

        spi.cr1.write(|w| {
            w.ssi().set_bit();
            w.ssm().set_bit();
            w.spe().set_bit();
            w.br().bits(div as u8);
            w.mstr().set_bit();
            // hard coded because we know the SPI mode of the radio
            w.cpol().idle_low();
            w.cpha().first_edge()
        });
        spi.cr2.write(|w| w.frxth().quarter());

        Self {
            spi,
            mosi: SgMosi::new(),
            miso: SgMiso::new(),
        }
    }

    pub(crate) unsafe fn steal() -> Self {
        Self {
            spi: pac::Peripherals::steal().SPI3,
            mosi: SgMosi::new(),
            miso: SgMiso::new(),
        }
    }
}

impl<MISO, MOSI> Spi3<MISO, MOSI> {
    pub(crate) fn free(self) -> (pac::SPI3, MISO, MOSI) {
        (self.spi, self.miso, self.mosi)
    }
}

impl<MISODMA: DmaCh, MOSIDMA: DmaCh> Spi3<MISODMA, MOSIDMA> {
    pub(crate) fn new_with_dma(
        spi: pac::SPI3,
        mut miso_dma: MISODMA,
        mut mosi_dma: MOSIDMA,
        div: BaudRate,
        rcc: &mut pac::RCC,
    ) -> Self {
        Self::enable_clock(rcc);
        unsafe { Self::pulse_reset(rcc) };

        spi.cr1.write(|w| {
            w.ssi().set_bit();
            w.ssm().set_bit();
            w.spe().set_bit();
            w.br().bits(div as u8);
            w.mstr().set_bit();
            // hard coded because we know the SPI mode of the radio
            w.cpol().idle_low();
            w.cpha().first_edge()
        });

        mosi_dma.set_cr(dma::Cr::DISABLE);
        mosi_dma.clear_all_flags();
        mosi_dma.set_periph_addr(pac::SPI3::DR as u32);
        mosi_dma.set_mux_cr_reqid(pac::SPI3::DMA_TX_ID);

        miso_dma.set_cr(dma::Cr::DISABLE);
        miso_dma.clear_all_flags();
        miso_dma.set_periph_addr(pac::SPI3::DR as u32);
        miso_dma.set_mux_cr_reqid(pac::SPI3::DMA_RX_ID);

        spi.cr2
            .write(|w| w.txdmaen().enabled().rxdmaen().enabled().frxth().quarter());

        Self {
            spi,
            miso: miso_dma,
            mosi: mosi_dma,
        }
    }

    pub(crate) unsafe fn steal_with_dma(miso_dma: MISODMA, mosi_dma: MOSIDMA) -> Self {
        Self {
            spi: pac::Peripherals::steal().SPI3,
            miso: miso_dma,
            mosi: mosi_dma,
        }
    }
}

impl<SPI, SCK, MISO, MOSI> Spi<SPI, SCK, MISO, MOSI> {
    /// Free the SPI peripheral, pins, and DMA channel(s) from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     dma::AllDma,
    ///     gpio::PortA,
    ///     pac,
    ///     spi::{BaudRate::Div2, Spi, MODE_0},
    /// };
    ///
    /// let mut dp = pac::Peripherals::take().unwrap();
    ///
    /// let dma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    /// let pa = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let spi = cortex_m::interrupt::free(|cs| {
    ///     Spi::new_spi1_full_duplex_dma(
    ///         dp.SPI1,
    ///         (pa.a5, pa.a6, pa.a7),
    ///         (dma.d1.c1, dma.d1.c2),
    ///         MODE_0,
    ///         Div2,
    ///         &mut dp.RCC,
    ///         cs,
    ///     )
    /// });
    /// // .. use spi
    /// let (spi1, a5, (a6, d1c1), (a7, d1c2)) = spi.free();
    /// ```
    pub fn free(self) -> (SPI, SCK, MISO, MOSI) {
        (self.spi, self.sck, self.miso, self.mosi)
    }
}

impl<SPI: SpiRegs, SCK, MISO, MOSI> Spi<SPI, SCK, MISO, MOSI> {
    /// Read the SPI status register.
    #[inline]
    pub fn status(&self) -> pac::spi1::sr::R {
        self.spi.sr.read()
    }
}

impl<SPI: SpiRegs, SCK: SpiSck, MISO: SpiMiso, MISODMA: DmaCh> Write<u8>
    for Spi<SPI, SCK, (MISO, MISODMA), NoMosi>
{
    type Error = Error;
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write_simplex_u8_dma(&mut self.miso.1, words)
    }
}

impl<SPI: SpiRegs, SCK: SpiSck, MOSI: SpiMosi, MOSIDMA: DmaCh> Write<u8>
    for Spi<SPI, SCK, NoMiso, (MOSI, MOSIDMA)>
{
    type Error = Error;
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write_simplex_u8_dma(&mut self.mosi.1, words)
    }
}

impl<SPI: SpiRegs, SCK: SpiSck, MOSI: SpiMosi> Write<u8> for Spi<SPI, SCK, NoMiso, MOSI> {
    type Error = Error;
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write_simplex_u8(words)
    }
}

impl<SPI: SpiRegs, SCK: SpiSck, MISO: SpiMiso, MOSI: SpiMosi> Write<u8>
    for Spi<SPI, SCK, MISO, MOSI>
{
    type Error = Error;
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write_full_duplex_u8(words)
    }
}

impl<SPI: SpiRegs, SCK: SpiSck, MISO: SpiMiso, MOSI: SpiMosi, MISODMA: DmaCh, MOSIDMA: DmaCh>
    Write<u8> for Spi<SPI, SCK, (MISO, MISODMA), (MOSI, MOSIDMA)>
{
    type Error = Error;
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi
            .write_full_duplex_u8_dma(&mut self.miso.1, &mut self.mosi.1, words)
    }
}

impl<SPI: SpiRegs, SCK: SpiSck, MISO: SpiMiso, MOSI: SpiMosi> FullDuplex<u8>
    for Spi<SPI, SCK, MISO, MOSI>
{
    type Error = Error;
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.spi.nb_read_u8()
    }
    fn send(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.spi.nb_send_u8(word)
    }
}

impl<SPI: SpiRegs, SCK: SpiSck, MISO: SpiMiso, MOSI: SpiMosi> Transfer<u8>
    for Spi<SPI, SCK, MISO, MOSI>
{
    type Error = Error;
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi.transfer_u8(words)
    }
}

impl<SPI: SpiRegs, SCK: SpiSck, MISO: SpiMiso, MOSI: SpiMosi, MISODMA: DmaCh, MOSIDMA: DmaCh>
    Transfer<u8> for Spi<SPI, SCK, (MISO, MISODMA), (MOSI, MOSIDMA)>
{
    type Error = Error;
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi
            .transfer_u8_dma(&mut self.miso.1, &mut self.mosi.1, words)
    }
}

// sub-GHz SPI traits

impl Transfer<u8> for Spi3<SgMiso, SgMosi> {
    type Error = Error;
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi.transfer_u8(words)
    }
}

impl<MISODMA: DmaCh, MOSIDMA: DmaCh> Transfer<u8> for Spi3<MISODMA, MOSIDMA> {
    type Error = Error;
    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi
            .transfer_u8_dma(&mut self.miso, &mut self.mosi, words)
    }
}

impl Write<u8> for Spi3<SgMiso, SgMosi> {
    type Error = Error;
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write_full_duplex_u8(words)
    }
}

impl<MISODMA: DmaCh, MOSIDMA: DmaCh> Write<u8> for Spi3<MISODMA, MOSIDMA> {
    type Error = Error;
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi
            .write_full_duplex_u8_dma(&mut self.miso, &mut self.mosi, words)
    }
}
