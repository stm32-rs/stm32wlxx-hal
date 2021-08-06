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

use core::{
    ops::Mul,
    ptr::{read_volatile, write_volatile},
};

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
    /// interrupt number
    irq: pac::Interrupt,
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
    /// valid: channel number is 1-7, interrupt matches the channel.
    const unsafe fn new(ctrl: DmaCtrl, ch: u8, irq: pac::Interrupt) -> DmaCh {
        let mux_ch: u8 = match ctrl {
            DmaCtrl::Dma1 => ch - 1,
            DmaCtrl::Dma2 => ch + 6,
        };
        let mux_ch_u: usize = mux_ch as usize;

        let ch: u8 = ch - 1;
        let ch_u: usize = ch as usize;

        // TODO: enable when you can assert in const fn
        // assert_ne!(ch, 0);
        // assert!(ch <= 7);

        DmaCh {
            ch,
            mux_ch,
            irq,
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

    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    pub(crate) const fn mux_ch(&self) -> usize {
        self.mux_ch as usize
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
    /// # let dma = unsafe { stm32wl_hal::dma::AllDma::steal().d1c1 };
    /// let xfer_cpl: bool = dma.flags() & flags::XFER_CPL != 0;
    /// ```
    pub fn flags(&self) -> u8 {
        let raw: u32 = unsafe { read_volatile(self.isr) };
        ((raw >> self.ch.mul(4)) & 0xF) as u8
    }

    fn clear_flags(&mut self, flags: u8) {
        let val: u32 = u32::from(flags & 0xF) << self.ch.mul(4);
        unsafe { write_volatile(self.ifcr, val) }
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

    /// Unmask the DMA interrupt in the NVIC.
    ///
    /// # Safety
    ///
    /// This can break mask-based critical sections.
    ///
    /// # Cortex-M0+
    ///
    /// On the STM32WL5X Cortex-M0+ core the DMA interrupts are not independent
    /// (one per channel), and enabling the interrupt for a DMA channel will
    /// enable other IRQs in the same group:
    ///
    /// * DMA1 channel 3:1 secure and non-secure interrupt (C2IMR2\[2:0\])
    /// * DMA1 channel 7:4 secure and non-secure interrupt (C2IMR2\[6:3\])
    /// * DMA2 channel 7:1 secure and non-secure interrupt (C2IMR2\[14:8\])
    ///   DMAMUX1 overrun interrupt (C2IMR2\[15\])
    pub unsafe fn unmask_irq(&self) {
        pac::NVIC::unmask(self.irq)
    }

    /// Mask the DMA interrupt in the NVIC.
    pub fn mask_irq(&self) {
        pac::NVIC::mask(self.irq)
    }
}

/// Type marker to indicate a peripheral is not using DMA
#[derive(Debug)]
pub struct NoDmaCh {
    _priv: (),
}

impl NoDmaCh {
    pub(crate) const fn new() -> NoDmaCh {
        NoDmaCh { _priv: () }
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

#[cfg(any(feature = "stm32wl5x_cm4", feature = "stm32wle5"))]
const ALL_DMA: AllDma = unsafe {
    AllDma {
        d1c1: DmaCh::new(DmaCtrl::Dma1, 1, pac::Interrupt::DMA1_CH1),
        d1c2: DmaCh::new(DmaCtrl::Dma1, 2, pac::Interrupt::DMA1_CH2),
        d1c3: DmaCh::new(DmaCtrl::Dma1, 3, pac::Interrupt::DMA1_CH3),
        d1c4: DmaCh::new(DmaCtrl::Dma1, 4, pac::Interrupt::DMA1_CH4),
        d1c5: DmaCh::new(DmaCtrl::Dma1, 5, pac::Interrupt::DMA1_CH5),
        d1c6: DmaCh::new(DmaCtrl::Dma1, 6, pac::Interrupt::DMA1_CH6),
        d1c7: DmaCh::new(DmaCtrl::Dma1, 7, pac::Interrupt::DMA1_CH7),
        d2c1: DmaCh::new(DmaCtrl::Dma2, 1, pac::Interrupt::DMA2_CH1),
        d2c2: DmaCh::new(DmaCtrl::Dma2, 2, pac::Interrupt::DMA2_CH2),
        d2c3: DmaCh::new(DmaCtrl::Dma2, 3, pac::Interrupt::DMA2_CH3),
        d2c4: DmaCh::new(DmaCtrl::Dma2, 4, pac::Interrupt::DMA2_CH4),
        d2c5: DmaCh::new(DmaCtrl::Dma2, 5, pac::Interrupt::DMA2_CH5),
        d2c6: DmaCh::new(DmaCtrl::Dma2, 6, pac::Interrupt::DMA2_CH6),
        d2c7: DmaCh::new(DmaCtrl::Dma2, 7, pac::Interrupt::DMA2_CH7),
    }
};

#[cfg(feature = "stm32wl5x_cm0p")]
const ALL_DMA: AllDma = unsafe {
    AllDma {
        d1c1: DmaCh::new(DmaCtrl::Dma1, 1, pac::Interrupt::DMA1_CH3_1),
        d1c2: DmaCh::new(DmaCtrl::Dma1, 2, pac::Interrupt::DMA1_CH3_1),
        d1c3: DmaCh::new(DmaCtrl::Dma1, 3, pac::Interrupt::DMA1_CH3_1),
        d1c4: DmaCh::new(DmaCtrl::Dma1, 4, pac::Interrupt::DMA1_CH7_4),
        d1c5: DmaCh::new(DmaCtrl::Dma1, 5, pac::Interrupt::DMA1_CH7_4),
        d1c6: DmaCh::new(DmaCtrl::Dma1, 6, pac::Interrupt::DMA1_CH7_4),
        d1c7: DmaCh::new(DmaCtrl::Dma1, 7, pac::Interrupt::DMA1_CH7_4),
        d2c1: DmaCh::new(DmaCtrl::Dma2, 1, pac::Interrupt::DMA2_CH7_1_DMAMUX1_OVR),
        d2c2: DmaCh::new(DmaCtrl::Dma2, 2, pac::Interrupt::DMA2_CH7_1_DMAMUX1_OVR),
        d2c3: DmaCh::new(DmaCtrl::Dma2, 3, pac::Interrupt::DMA2_CH7_1_DMAMUX1_OVR),
        d2c4: DmaCh::new(DmaCtrl::Dma2, 4, pac::Interrupt::DMA2_CH7_1_DMAMUX1_OVR),
        d2c5: DmaCh::new(DmaCtrl::Dma2, 5, pac::Interrupt::DMA2_CH7_1_DMAMUX1_OVR),
        d2c6: DmaCh::new(DmaCtrl::Dma2, 6, pac::Interrupt::DMA2_CH7_1_DMAMUX1_OVR),
        d2c7: DmaCh::new(DmaCtrl::Dma2, 7, pac::Interrupt::DMA2_CH7_1_DMAMUX1_OVR),
    }
};

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

        ALL_DMA
    }

    /// Steal all DMA channels.
    ///
    /// This will **not** initialize the DMA peripheral or the DMAMUX.
    ///
    /// # Safety
    ///
    /// This will create a steal all DMA channels, bypassing the singleton
    /// checks that normally occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// each DMA channel.
    /// You are also responsible for ensuring the DMA channel has been setup
    /// correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::dma::AllDma;
    ///
    /// // ... setup occurs here
    ///
    /// let dma: AllDma = unsafe { AllDma::steal() };
    /// ```
    pub const unsafe fn steal() -> AllDma {
        ALL_DMA
    }
}

#[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
pub(crate) mod aio {
    use core::{
        sync::atomic::{AtomicU8, Ordering::SeqCst},
        task::Poll,
    };
    use futures_util::task::AtomicWaker;

    #[allow(clippy::declare_interior_mutable_const)]
    const WAKER: AtomicWaker = AtomicWaker::new();
    #[allow(clippy::declare_interior_mutable_const)]
    const FLAGS: AtomicU8 = AtomicU8::new(0);

    static DMA_WAKER: [AtomicWaker; 14] = [WAKER; 14];
    static DMA_FLAGS: [AtomicU8; 14] = [FLAGS; 14];

    pub fn poll(mux_ch: usize, cx: &mut core::task::Context<'_>) -> Poll<Result<(), super::Error>> {
        DMA_WAKER[mux_ch].register(cx.waker());
        match DMA_FLAGS[mux_ch].load(SeqCst) {
            0 => core::task::Poll::Pending,
            _ => {
                DMA_WAKER[mux_ch].take();
                let flags: u8 = DMA_FLAGS[mux_ch].swap(0, SeqCst);
                if flags & super::flags::XFER_ERR != 0 {
                    Poll::Ready(Err(super::Error::Xfer))
                } else {
                    Poll::Ready(Ok(()))
                }
            }
        }
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    mod irq {
        use super::{
            super::{Cr, DmaCh, ALL_DMA},
            DMA_FLAGS, DMA_WAKER,
        };
        use crate::pac::interrupt;
        use core::sync::atomic::Ordering::SeqCst;

        macro_rules! dma_irq_handler {
            ($name:ident, $dma:ident) => {
                #[interrupt]
                #[allow(non_snake_case)]
                fn $name() {
                    let mut dma: DmaCh = ALL_DMA.$dma;
                    const DMA_IDX: usize = ALL_DMA.$dma.mux_ch as usize;

                    debug_assert_eq!(DMA_FLAGS[DMA_IDX].load(SeqCst), 0);

                    // store result
                    DMA_FLAGS[DMA_IDX].store(dma.flags(), SeqCst);

                    // disable DMA
                    dma.set_cr(Cr::DISABLE);

                    // clear flags
                    dma.clear_all_flags();

                    // wake
                    DMA_WAKER[DMA_IDX].wake();
                }
            };
        }

        dma_irq_handler!(DMA1_CH1, d1c1);
        dma_irq_handler!(DMA1_CH2, d1c2);
        dma_irq_handler!(DMA1_CH3, d1c3);
        dma_irq_handler!(DMA1_CH4, d1c4);
        dma_irq_handler!(DMA1_CH5, d1c5);
        dma_irq_handler!(DMA1_CH6, d1c6);
        dma_irq_handler!(DMA1_CH7, d1c7);
        dma_irq_handler!(DMA2_CH1, d2c1);
        dma_irq_handler!(DMA2_CH2, d2c2);
        dma_irq_handler!(DMA2_CH3, d2c3);
        dma_irq_handler!(DMA2_CH4, d2c4);
        dma_irq_handler!(DMA2_CH5, d2c5);
        dma_irq_handler!(DMA2_CH6, d2c6);
        dma_irq_handler!(DMA2_CH7, d2c7);
    }
}
