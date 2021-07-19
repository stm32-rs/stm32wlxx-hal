//! Direct memory access controller
#![deny(missing_docs)]

// developers notes:
//
// This is the way it is (not using the PAC, unsafe code) for several reasons:
//
// * svd2rust is not monomorphic for multiple instantiations of the same
//   peripheral
// * svd2rust does not have a way to index arrays of registers
// * even if svd2rust had the two above the DMAs and DMAMUX have links between
//   them that the PAC cannot see
//
// This is the best solution I could come up with (at the time), feedback is
// very much welcome.

mod cr;

use core::ptr::{read_volatile, write_volatile};

use super::pac;

pub use cr::{Cr, Dir, Priority, Size};

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

/// DMA controller instance
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
pub enum DmaCtrl {
    /// DMA1
    Dma1,
    /// DMA2
    Dma2,
}

impl DmaCtrl {
    const fn base(&self) -> usize {
        match self {
            DmaCtrl::Dma1 => 0x4002_0000,
            DmaCtrl::Dma2 => 0x4002_0400,
        }
    }
}

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
pub struct DmaCh {
    /// zero-index channel number (0-6)
    ch: u8,
    /// zero-index mux channel number (0-13)
    mux_ch: u8,
    // here be registers
    mux_cr: *mut u32,
    mux_rgcr: *mut u32,
    isr: *const u32,
    ifcr: *mut u32,
    cr: *mut u32,
    ndt: *mut u32,
    pa: *mut u32,
    ma: *mut u32,
}

impl DmaCh {
    /// Steal the DMA channel from whatever is currently using it.
    ///
    /// This will **not** initialize the DMA peripheral or the DMAMUX.
    ///
    /// # Safety
    ///
    /// This will create a steal DMA channel, bypassing the singleton checks
    /// that normally occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// the DMA channel.
    /// You are also responsible for ensuring the DMA channel has been setup
    /// correctly.
    /// You are also responsible for ensuring the arguments are
    /// valid (channel number is 1-7).
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::dma::{DmaCh, DmaCtrl};
    ///
    /// // ... setup happens here
    ///
    /// let dma1ch1 = unsafe { DmaCh::steal(DmaCtrl::Dma1, 1) };
    /// ```
    pub const unsafe fn steal(ctrl: DmaCtrl, ch: u8) -> DmaCh {
        let mux_ch: u8 = match ctrl {
            DmaCtrl::Dma1 => ch - 1,
            DmaCtrl::Dma2 => ch + 6,
        };
        let mux_ch_u: usize = mux_ch as usize;

        let ch: u8 = ch - 1;
        let ch_u: usize = ch as usize;

        DmaCh {
            ch,
            mux_ch,
            mux_cr: (MUX_BASE + 0x4 * mux_ch_u) as *mut u32,
            mux_rgcr: (MUX_BASE + 0x100 + 0x4 * mux_ch_u) as *mut u32,
            isr: ctrl.base() as *const u32,
            ifcr: (ctrl.base() + 0x4) as *mut u32,
            cr: (ctrl.base() + 0x08 + 0x14 * ch_u) as *mut u32,
            ndt: (ctrl.base() + 0x0C + 0x14 * ch_u) as *mut u32,
            pa: (ctrl.base() + 0x10 + 0x14 * ch_u) as *mut u32,
            ma: (ctrl.base() + 0x14 + 0x14 * ch_u) as *mut u32,
        }
    }

    /// Get the interrupt flags for the DMA channel.
    ///
    /// **Note:** The upper 4 bits of the return value are unused.
    ///
    /// # Example
    ///
    /// Check if the transfer is complete.
    ///
    /// ```no_run
    /// use stm32wl_hal::dma::flags;
    ///
    /// # let dma = unsafe { stm32wl_hal::dma::DmaCh::steal(stm32wl_hal::dma::DmaCtrl::Dma1, 1) };
    /// let xfer_cpl: bool = dma.flags() & flags::XFER_CPL != 0;
    /// ```
    pub fn flags(&self) -> u8 {
        let raw: u32 = unsafe { read_volatile(self.isr) };
        ((raw >> (4 * self.ch)) & 0xF) as u8
    }

    fn clear_flags(&mut self, flags: u8) {
        unsafe { write_volatile(self.ifcr, (flags & 0xF) as u32) }
    }

    pub(crate) fn clear_all_flags(&mut self) {
        self.clear_flags(flags::GLOBAL | flags::XFER_CPL | flags::XFER_HLF | flags::XFER_ERR)
    }

    pub(crate) fn set_periph_addr(&mut self, pa: u32) {
        unsafe { write_volatile(self.pa, pa) }
    }

    pub(crate) fn set_mem_addr(&mut self, ma: u32) {
        unsafe { write_volatile(self.ma, ma) }
    }

    pub(crate) fn set_num_data_xfer(&mut self, ndt: u32) {
        unsafe { write_volatile(self.ndt, ndt) }
    }

    pub(crate) fn set_cr(&mut self, cr: Cr) {
        unsafe { write_volatile(self.cr, cr.raw()) }
    }

    pub(crate) fn set_mux_cr_reqid(&mut self, req_id: u8) {
        unsafe { write_volatile(self.mux_cr, req_id as u32) }
    }

    /// Returns `true` if the DMA MUX synchronization overrun bit is set for
    /// this channel.
    pub fn sync_ovr(&self) -> bool {
        let csr: u32 = unsafe { read_volatile(MUX_CSR_ADDR as *const u32) };
        csr >> self.mux_ch & 0b1 == 0b1
    }

    #[allow(dead_code)]
    fn clr_sync_ovr(&mut self) {
        unsafe { write_volatile(MUX_CCFR_ADDR as *mut u32, 1 << self.mux_ch) };
    }
}

/// All DMA channels
#[derive(Debug)]
pub struct AllDma {
    /// DMA controller 1 channel 1
    pub d1c1: DmaCh,
    /// DMA controller 1 channel 2
    pub d1c2: DmaCh,
    /// DMA controller 1 channel 3
    pub d1c3: DmaCh,
    /// DMA controller 1 channel 4
    pub d1c4: DmaCh,
    /// DMA controller 1 channel 5
    pub d1c5: DmaCh,
    /// DMA controller 1 channel 6
    pub d1c6: DmaCh,
    /// DMA controller 1 channel 7
    pub d1c7: DmaCh,
    /// DMA controller 2 channel 1
    pub d2c1: DmaCh,
    /// DMA controller 2 channel 2
    pub d2c2: DmaCh,
    /// DMA controller 2 channel 3
    pub d2c3: DmaCh,
    /// DMA controller 2 channel 4
    pub d2c4: DmaCh,
    /// DMA controller 2 channel 5
    pub d2c5: DmaCh,
    /// DMA controller 2 channel 6
    pub d2c6: DmaCh,
    /// DMA controller 2 channel 7
    pub d2c7: DmaCh,
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

        const RET: AllDma = unsafe {
            AllDma {
                d1c1: DmaCh::steal(DmaCtrl::Dma1, 1),
                d1c2: DmaCh::steal(DmaCtrl::Dma1, 2),
                d1c3: DmaCh::steal(DmaCtrl::Dma1, 3),
                d1c4: DmaCh::steal(DmaCtrl::Dma1, 4),
                d1c5: DmaCh::steal(DmaCtrl::Dma1, 5),
                d1c6: DmaCh::steal(DmaCtrl::Dma1, 6),
                d1c7: DmaCh::steal(DmaCtrl::Dma1, 7),
                d2c1: DmaCh::steal(DmaCtrl::Dma2, 1),
                d2c2: DmaCh::steal(DmaCtrl::Dma2, 2),
                d2c3: DmaCh::steal(DmaCtrl::Dma2, 3),
                d2c4: DmaCh::steal(DmaCtrl::Dma2, 4),
                d2c5: DmaCh::steal(DmaCtrl::Dma2, 5),
                d2c6: DmaCh::steal(DmaCtrl::Dma2, 6),
                d2c7: DmaCh::steal(DmaCtrl::Dma2, 7),
            }
        };

        RET
    }
}
