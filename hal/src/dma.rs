//! Direct memory access controller

use core::ptr::{read_volatile, write_volatile};

use super::pac;

/// IRQ flags
pub mod flags {
    /// Global interrupt, combination of all other interrupts.
    pub const GLOBAL: u8 = 1 << 0;
    /// Transfer complete
    pub const XFER_CPL: u8 = 1 << 1;
    /// Transfer hald complete
    pub const XFER_HLF: u8 = 1 << 2;
    /// Transfer error
    pub const XFER_ERR: u8 = 1 << 3;
}

struct DmaCr {
    reg: u32,
}

impl DmaCr {
    pub const DISABLE: u32 = Self::reset().set_enable(false).raw();

    pub const fn new(val: u32) -> DmaCr {
        DmaCr { reg: val }
    }

    pub const fn reset() -> DmaCr {
        Self::new(0)
    }

    pub const fn set_enable(mut self, en: bool) -> DmaCr {
        const EN_MASK: u32 = 0b1;
        if en {
            self.reg |= EN_MASK;
        } else {
            self.reg &= !EN_MASK;
        }
        self
    }

    pub const fn set_xfer_cpl_irq_en(mut self, en: bool) -> DmaCr {
        const TCIE_MASK: u32 = 0b1 << 1;
        if en {
            self.reg |= TCIE_MASK;
        } else {
            self.reg &= !TCIE_MASK;
        }
        self
    }

    #[allow(dead_code)]
    pub const fn set_xfer_half_irq_en(mut self, en: bool) -> DmaCr {
        const HTIE_MASK: u32 = 0b1 << 2;
        if en {
            self.reg |= HTIE_MASK;
        } else {
            self.reg &= !HTIE_MASK;
        }
        self
    }

    pub const fn set_xfer_err_irq_en(mut self, en: bool) -> DmaCr {
        const TEIE_MASK: u32 = 0b1 << 3;
        if en {
            self.reg |= TEIE_MASK;
        } else {
            self.reg &= !TEIE_MASK;
        }
        self
    }

    pub const fn set_dir_from_mem(mut self, from_mem: bool) -> DmaCr {
        const DIR_MASK: u32 = 0b1 << 4;
        if from_mem {
            self.reg |= DIR_MASK;
        } else {
            self.reg &= !DIR_MASK;
        }
        self
    }

    #[allow(dead_code)]
    pub const fn set_circ_mode(mut self, circ: bool) -> DmaCr {
        const CIRC_MASK: u32 = 0b1 << 5;
        if circ {
            self.reg |= CIRC_MASK;
        } else {
            self.reg &= !CIRC_MASK;
        }
        self
    }

    #[allow(dead_code)]
    pub const fn set_periph_inc(mut self, inc: bool) -> DmaCr {
        const PINC_MASK: u32 = 0b1 << 6;
        if inc {
            self.reg |= PINC_MASK;
        } else {
            self.reg &= !PINC_MASK;
        }
        self
    }

    pub const fn set_mem_inc(mut self, inc: bool) -> DmaCr {
        const MINC_MASK: u32 = 0b1 << 7;
        if inc {
            self.reg |= MINC_MASK;
        } else {
            self.reg &= !MINC_MASK;
        }
        self
    }

    pub const fn raw(self) -> u32 {
        self.reg
    }
}

/// DMA MUX request mappings.
#[non_exhaustive]
pub enum MuxMap {
    Spi1Rx,
    Spi1Tx,
    Spi2Rx,
    Spi2Tx,
    SubGhzSpiRx,
    SubGhzSpiTx,
}

impl MuxMap {
    const fn id(&self) -> u8 {
        match self {
            Self::Spi1Rx => 7,
            Self::Spi1Tx => 8,
            Self::Spi2Rx => 9,
            Self::Spi2Tx => 10,
            Self::SubGhzSpiRx => 41,
            Self::SubGhzSpiTx => 42,
        }
    }

    const fn pa(&self) -> u32 {
        const SPI_OFFS: u32 = 0xC;
        const SPI1_BASE: u32 = 0x4001_3000 + SPI_OFFS;
        const SPI2_BASE: u32 = 0x4000_3800 + SPI_OFFS;
        const SPI3_BASE: u32 = 0x5801_0000 + SPI_OFFS;
        match self {
            Self::Spi1Rx | Self::Spi1Tx => SPI1_BASE,
            Self::Spi2Rx | Self::Spi2Tx => SPI2_BASE,
            Self::SubGhzSpiRx | Self::SubGhzSpiTx => SPI3_BASE,
        }
    }
}

const DMA1_BASE: usize = 0x4002_0000;
const DMA2_BASE: usize = 0x4002_0400;

const MUX_BASE: usize = 0x4002_0800;
const MUX_CSR_ADDR: usize = MUX_BASE + 0x80;
const MUX_CCFR_ADDR: usize = MUX_BASE + 0x84;
#[allow(dead_code)]
const MUX_RGSR_ADDR: usize = MUX_BASE + 0x140;
#[allow(dead_code)]
const MUX_RGCFR_ADDR: usize = MUX_BASE + 0x144;

/// DMA errors
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Error {
    /// Transfer error
    Xfer,
}

/// DMA channel
#[derive(Debug)]
pub struct DmaCh<const IS_DMA2: bool, const CH: u8> {
    _priv: (),
}

impl<const IS_DMA2: bool, const CH: u8> DmaCh<IS_DMA2, CH> {
    const MUX_CH: u8 = if IS_DMA2 { CH + 6 } else { CH - 1 };
    const MUX_CR_ADDR: usize = MUX_BASE + 0x04 * Self::MUX_CH as usize;
    #[allow(dead_code)]
    const MUX_RGCR_ADDR: usize = MUX_BASE + 0x100 + 0x04 * Self::MUX_CH as usize;

    const BASE: usize = if IS_DMA2 { DMA2_BASE } else { DMA1_BASE };
    const ISR_ADDR: usize = Self::BASE;
    const IFCR_ADDR: usize = Self::BASE + 0x04;
    const CR_ADDR: usize = Self::BASE + 0x08 + 0x14 * ((CH as usize) - 1);
    const NDT_ADDR: usize = Self::BASE + 0x0C + 0x14 * ((CH as usize) - 1);
    const PA_ADDR: usize = Self::BASE + 0x10 + 0x14 * ((CH as usize) - 1);
    const MA_ADDR: usize = Self::BASE + 0x14 + 0x14 * ((CH as usize) - 1);

    const fn new() -> DmaCh<IS_DMA2, CH> {
        DmaCh { _priv: () }
    }

    /// Steal the DMA channel from whatever is currently using it.
    ///
    /// This will **not** initialize the DMA peripheral or the DMAMUX.
    ///
    /// # Safety
    ///
    /// This will create a new DMA channel, bypassing the singleton checks
    /// that normally occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// the DMA channel.
    /// You are also responsible for ensuring the DMA channel has been setup
    /// correctly.
    /// You are also responsible for ensuring the constant type parameters are
    /// valid (channel number is 1-7).
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::dma::DmaCh;
    ///
    /// // ... setup happens here
    ///
    /// let dma_ch = unsafe { DmaCh::<false, 1>::steal() };
    /// ```
    pub const unsafe fn steal() -> DmaCh<IS_DMA2, CH> {
        DmaCh { _priv: () }
    }

    /// Get the interrupt flags for the DMA channel.
    ///
    /// # Example
    ///
    /// Check if the transfer is complete.
    ///
    /// ```no_run
    /// use stm32wl_hal::dma::{flags, DmaCh};
    ///
    /// # let dma_ch = unsafe { DmaCh::<true, 1>::steal() };
    /// let xfer_cpl: bool = dma_ch.flags() & flags::XFER_CPL != 0;
    /// ```
    pub fn flags(&self) -> u8 {
        let raw: u32 = unsafe { read_volatile(Self::ISR_ADDR as *const u32) };
        (raw >> (4 * (CH - 1))) as u8
    }

    fn clear_flags(&mut self, flags: u8) {
        unsafe { write_volatile(Self::IFCR_ADDR as *mut u32, (flags & 0xF) as u32) }
    }

    fn clear_all_flags(&mut self) {
        self.clear_flags(flags::GLOBAL | flags::XFER_CPL | flags::XFER_HLF | flags::XFER_ERR)
    }

    fn set_pa(&mut self, pa: u32) {
        unsafe { write_volatile(Self::PA_ADDR as *mut u32, pa) }
    }

    fn set_ma(&mut self, ma: u32) {
        unsafe { write_volatile(Self::MA_ADDR as *mut u32, ma) }
    }

    fn set_ndt(&mut self, ndt: u32) {
        unsafe { write_volatile(Self::NDT_ADDR as *mut u32, ndt) }
    }

    fn set_cr(&mut self, cr: u32) {
        unsafe { write_volatile(Self::CR_ADDR as *mut u32, cr) }
    }

    fn set_mux_cr(&mut self, cr: u32) {
        unsafe { write_volatile(Self::MUX_CR_ADDR as *mut u32, cr) }
    }

    /// Returns `true` if the DMA MUX synchronization overrun bit is set for
    /// this channel.
    pub fn sync_ovr(&self) -> bool {
        let csr: u32 = unsafe { read_volatile(MUX_CSR_ADDR as *const u32) };
        csr >> Self::MUX_CH & 0b1 == 0b1
    }

    #[allow(dead_code)]
    fn clr_sync_ovr(&mut self) {
        unsafe { write_volatile(MUX_CCFR_ADDR as *mut u32, 1 << Self::MUX_CH) };
    }

    /// Transfer from memory to a peripheral.
    pub unsafe fn mem_to_periph(&mut self, src: &[u8], dst: MuxMap) -> Result<(), Error> {
        const CR: DmaCr = DmaCr::reset()
            .set_xfer_cpl_irq_en(true)
            .set_xfer_err_irq_en(true)
            .set_dir_from_mem(true)
            .set_mem_inc(true);

        self.clear_all_flags();

        self.set_cr(CR.raw());
        self.set_ma(src.as_ptr() as u32);
        self.set_pa(dst.pa());
        self.set_ndt(src.len() as u32);
        self.set_mux_cr(dst.id() as u32);
        self.set_cr(CR.set_enable(true).raw());

        let ret: Result<(), Error> = loop {
            let flags = self.flags();
            if flags & flags::XFER_ERR != 0 {
                break Err(Error::Xfer);
            }
            if flags & flags::XFER_CPL != 0 {
                break Ok(());
            }
        };

        self.set_cr(DmaCr::DISABLE);
        ret
    }

    /// Transfer from a peripheral to memory.
    pub unsafe fn periph_to_mem(&mut self, src: MuxMap, dst: &[u8]) -> Result<(), Error> {
        const CR: DmaCr = DmaCr::reset()
            .set_xfer_cpl_irq_en(true)
            .set_xfer_err_irq_en(true)
            .set_dir_from_mem(false)
            .set_mem_inc(true);

        self.clear_all_flags();

        self.set_cr(CR.raw());
        self.set_ma(dst.as_ptr() as u32);
        self.set_pa(src.pa());
        self.set_ndt(dst.len() as u32);
        self.set_mux_cr(src.id() as u32);
        self.set_cr(CR.set_enable(true).raw());

        let ret: Result<(), Error> = loop {
            let flags = self.flags();
            if flags & flags::XFER_ERR != 0 {
                break Err(Error::Xfer);
            }
            if flags & flags::XFER_CPL != 0 {
                break Ok(());
            }
        };

        self.set_cr(DmaCr::DISABLE);
        ret
    }
}

/// All DMA channels
#[derive(Debug)]
pub struct AllDma {
    /// DMA controller 1 channel 1
    pub d1c1: DmaCh<false, 1>,
    /// DMA controller 1 channel 2
    pub d1c2: DmaCh<false, 2>,
    /// DMA controller 1 channel 3
    pub d1c3: DmaCh<false, 3>,
    /// DMA controller 1 channel 4
    pub d1c4: DmaCh<false, 4>,
    /// DMA controller 1 channel 5
    pub d1c5: DmaCh<false, 5>,
    /// DMA controller 1 channel 6
    pub d1c6: DmaCh<false, 6>,
    /// DMA controller 1 channel 7
    pub d1c7: DmaCh<false, 7>,
    /// DMA controller 2 channel 1
    pub d2c1: DmaCh<true, 1>,
    /// DMA controller 2 channel 2
    pub d2c2: DmaCh<true, 2>,
    /// DMA controller 2 channel 3
    pub d2c3: DmaCh<true, 3>,
    /// DMA controller 2 channel 4
    pub d2c4: DmaCh<true, 4>,
    /// DMA controller 2 channel 5
    pub d2c5: DmaCh<true, 5>,
    /// DMA controller 2 channel 6
    pub d2c6: DmaCh<true, 6>,
    /// DMA controller 2 channel 7
    pub d2c7: DmaCh<true, 7>,
}

impl AllDma {
    /// Split the DMA registers into individual channels.
    ///
    /// This will enable clocks and reset the DMAMUX and both DMA controllers.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{dma::AllDma, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    /// ```
    #[allow(unused_variables)]
    pub fn split(
        dmamux: pac::DMAMUX,
        dma1: pac::DMA1,
        dma2: pac::DMA2,
        rcc: &mut pac::RCC,
    ) -> AllDma {
        #[rustfmt::skip]
        rcc.ahb1enr.modify(|_, w| {
            w
                .dmamux1en().enabled()
                .dma2en().enabled()
                .dma1en().enabled()
        });
        rcc.ahb1enr.read(); // delay after an RCC peripheral clock enabling

        #[rustfmt::skip]
        rcc.ahb1rstr.modify(|_, w| {
            w
                .dmamux1rst().set_bit()
                .dma2rst().set_bit()
                .dma1rst().set_bit()
        });
        #[rustfmt::skip]
        rcc.ahb1rstr.modify(|_, w| {
            w
                .dmamux1rst().clear_bit()
                .dma2rst().clear_bit()
                .dma1rst().clear_bit()
        });

        const RET: AllDma = AllDma {
            d1c1: DmaCh::new(),
            d1c2: DmaCh::new(),
            d1c3: DmaCh::new(),
            d1c4: DmaCh::new(),
            d1c5: DmaCh::new(),
            d1c6: DmaCh::new(),
            d1c7: DmaCh::new(),
            d2c1: DmaCh::new(),
            d2c2: DmaCh::new(),
            d2c3: DmaCh::new(),
            d2c4: DmaCh::new(),
            d2c5: DmaCh::new(),
            d2c6: DmaCh::new(),
            d2c7: DmaCh::new(),
        };

        RET
    }
}
