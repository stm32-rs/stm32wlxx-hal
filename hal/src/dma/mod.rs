//! Direct memory access

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

use core::ops::Mul;

use super::pac;

pub use cr::{Cr, Dir, Priority, Size};

/// IRQ flags.
pub mod flags {
    /// Global interrupt, combination of all other interrupts.
    pub const GLOBAL: u8 = 1 << 0;
    /// Transfer complete.
    pub const XFER_CPL: u8 = 1 << 1;
    /// Transfer half complete.
    pub const XFER_HLF: u8 = 1 << 2;
    /// Transfer error.
    pub const XFER_ERR: u8 = 1 << 3;
}

const DMA1_BASE: usize = 0x4002_0000;
const DMA2_BASE: usize = 0x4002_0400;

const MUX_BASE: usize = 0x4002_0800;
const MUX_CSR_ADDR: usize = MUX_BASE + 0x80;
const MUX_CCFR_ADDR: usize = MUX_BASE + 0x84;
// const MUX_RGSR_ADDR: usize = MUX_BASE + 0x140;
// const MUX_RGCFR_ADDR: usize = MUX_BASE + 0x144;

/// DMA errors.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Transfer error.
    Xfer,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct Dma<const BASE: usize, const CH: u8> {}

impl<const BASE: usize, const CH: u8> Dma<BASE, CH> {
    pub const MUX_CH: u8 = match BASE {
        DMA1_BASE => CH,
        DMA2_BASE => CH + 7,
        _ => core::panic!("DMA base address is invalid"),
    };
    const MUX_CR: *mut u32 = (MUX_BASE + 0x4 * (Self::MUX_CH as usize)) as *mut u32;
    // const MUX_RGCR: *mut u32 = (MUX_BASE + 0x100 + 0x4 * (Self::MUX_CH as usize)) as *mut u32;
    const ISR: *mut u32 = BASE as *mut u32;
    const IFCR: *mut u32 = (BASE + 0x4) as *mut u32;
    const CR: *mut u32 = (BASE + 0x08 + 0x14 * (CH as usize)) as *mut u32;
    const NDT: *mut u32 = (BASE + 0x0C + 0x14 * (CH as usize)) as *mut u32;
    const PA: *mut u32 = (BASE + 0x10 + 0x14 * (CH as usize)) as *mut u32;
    const MA: *mut u32 = (BASE + 0x14 + 0x14 * (CH as usize)) as *mut u32;

    #[inline]
    pub const fn new() -> Dma<BASE, CH> {
        Dma {}
    }

    #[inline]
    pub fn flags(&self) -> u8 {
        let raw: u32 = unsafe { Self::ISR.read_volatile() };
        ((raw >> CH.mul(4)) & 0xF) as u8
    }

    #[inline]
    pub fn clear_flags(&mut self, flags: u8) {
        let val: u32 = u32::from(flags & 0xF) << CH.mul(4);
        unsafe { Self::IFCR.write_volatile(val) }
    }

    #[inline]
    pub fn set_periph_addr(&mut self, pa: u32) {
        unsafe { Self::PA.write_volatile(pa) }
    }

    #[inline]
    pub fn set_mem_addr(&mut self, ma: u32) {
        unsafe { Self::MA.write_volatile(ma) }
    }

    #[inline]
    pub fn set_num_data_xfer(&mut self, ndt: u32) {
        unsafe { Self::NDT.write_volatile(ndt) }
    }

    #[inline]
    pub fn num_data_xfer(&self) -> u32 {
        unsafe { Self::NDT.read_volatile() }
    }

    #[inline]
    pub fn set_cr(&mut self, cr: Cr) {
        unsafe { Self::CR.write_volatile(cr.raw()) }
    }

    #[inline]
    pub fn cr(&self) -> Cr {
        Cr::new(unsafe { Self::CR.read_volatile() })
    }

    #[inline]
    pub fn set_mux_cr_reqid(&mut self, req_id: u8) {
        unsafe { Self::MUX_CR.write_volatile(req_id as u32) }
    }

    /// Returns `true` if the DMA MUX synchronization overrun bit is set for
    /// this channel.
    #[inline]
    pub fn sync_ovr(&self) -> bool {
        let csr: u32 = unsafe { (MUX_CSR_ADDR as *const u32).read_volatile() };
        csr >> Self::MUX_CH & 0b1 == 0b1
    }

    #[inline]
    #[allow(dead_code)]
    pub fn clr_sync_ovr(&mut self) {
        unsafe { (MUX_CCFR_ADDR as *mut u32).write_volatile(1 << Self::MUX_CH) };
    }
}

pub(crate) mod sealed {
    use super::Cr;

    pub trait DmaOps {
        fn set_periph_addr(&mut self, pa: u32);
        fn set_mem_addr(&mut self, ma: u32);
        fn set_num_data_xfer(&mut self, ndt: u32);
        fn num_data_xfer(&self) -> u32;
        fn set_cr(&mut self, cr: Cr);
        fn cr(&self) -> Cr;
        fn set_mux_cr_reqid(&mut self, req_id: u8);
        fn sync_ovr(&self) -> bool;
        fn clr_sync_ovr(&mut self);
    }
}

/// DMA channel trait.
pub trait DmaCh: sealed::DmaOps {
    /// DMA IRQ number.
    const IRQ: pac::Interrupt;

    /// Get the interrupt flags for the DMA channel.
    ///
    /// **Note:** The lower 4 bits of the return value are unused.
    ///
    /// # Example
    ///
    /// Check if the transfer is complete.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::dma::{flags, DmaCh};
    ///
    /// # let dma = unsafe { stm32wlxx_hal::dma::AllDma::steal().d1.c1 };
    /// let xfer_cpl: bool = dma.flags() & flags::XFER_CPL != 0;
    /// ```
    fn flags(&self) -> u8;

    /// Clear interrupt flags on the DMA channel.
    ///
    /// **Note:** The lower 4 bits of the `flags` argument are used.
    ///
    /// # Example
    ///
    /// Check and clear all set flags.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::dma::{flags, DmaCh};
    ///
    /// # let mut dma = unsafe { stm32wlxx_hal::dma::AllDma::steal().d1.c1 };
    /// let flags: u8 = dma.flags();
    /// dma.clear_flags(flags);
    /// ```
    fn clear_flags(&mut self, flags: u8);

    /// Clear all interrupt flags on the DMA channel.
    #[inline]
    fn clear_all_flags(&mut self) {
        self.clear_flags(flags::GLOBAL | flags::XFER_CPL | flags::XFER_HLF | flags::XFER_ERR)
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
    #[inline]
    unsafe fn unmask_irq(&self) {
        pac::NVIC::unmask(Self::IRQ)
    }

    /// Mask the DMA interrupt in the NVIC.
    #[inline]
    fn mask_irq(&self) {
        pac::NVIC::mask(Self::IRQ)
    }
}

macro_rules! dma_ch {
    ($ctrl:expr, $ch:expr, $irq:ident) => {
        paste::paste! {
            #[doc = "Controller " $ctrl " channel " $ch "."]
            #[derive(Debug)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub struct [<Dma $ctrl Ch $ch>] {
                pub(crate) dma: Dma<[<DMA $ctrl _BASE>], { $ch - 1 }>,
            }

            impl [<Dma $ctrl Ch $ch>] {
                const fn new() -> Self {
                    Self { dma: Dma::new() }
                }
            }

            impl DmaCh for [<Dma $ctrl Ch $ch>] {
                const IRQ: pac::Interrupt = irq_num::$irq;

                #[inline]
                fn flags(&self) -> u8 {
                    self.dma.flags()
                }

                #[inline]
                fn clear_flags(&mut self, flags: u8) {
                    self.dma.clear_flags(flags)
                }
            }

            impl sealed::DmaOps for [<Dma $ctrl Ch $ch>] {
                #[inline]
                fn set_periph_addr(&mut self, pa: u32) {
                    self.dma.set_periph_addr(pa)
                }
                #[inline]
                fn set_mem_addr(&mut self, ma: u32) {
                    self.dma.set_mem_addr(ma)
                }
                #[inline]
                fn set_num_data_xfer(&mut self, ndt: u32) {
                    self.dma.set_num_data_xfer(ndt)
                }
                #[inline]
                fn num_data_xfer(&self) -> u32 {
                    self.dma.num_data_xfer()
                }
                #[inline]
                fn set_cr(&mut self, cr: Cr) {
                    self.dma.set_cr(cr)
                }
                #[inline]
                fn cr(&self) -> Cr {
                    self.dma.cr()
                }
                #[inline]
                fn set_mux_cr_reqid(&mut self, req_id: u8) {
                    self.dma.set_mux_cr_reqid(req_id)
                }
                #[inline]
                fn sync_ovr(&self) -> bool {
                    self.dma.sync_ovr()
                }
                #[inline]
                fn clr_sync_ovr(&mut self) {
                    self.dma.clr_sync_ovr()
                }
            }
        }
    };
}

#[cfg(any(feature = "stm32wl5x_cm4", feature = "stm32wle5"))]
mod irq_num {
    use crate::pac::Interrupt;
    pub const DMA1_CH1: Interrupt = Interrupt::DMA1_CH1;
    pub const DMA1_CH2: Interrupt = Interrupt::DMA1_CH2;
    pub const DMA1_CH3: Interrupt = Interrupt::DMA1_CH3;
    pub const DMA1_CH4: Interrupt = Interrupt::DMA1_CH4;
    pub const DMA1_CH5: Interrupt = Interrupt::DMA1_CH5;
    pub const DMA1_CH6: Interrupt = Interrupt::DMA1_CH6;
    pub const DMA1_CH7: Interrupt = Interrupt::DMA1_CH7;
    pub const DMA2_CH1: Interrupt = Interrupt::DMA2_CH1;
    pub const DMA2_CH2: Interrupt = Interrupt::DMA2_CH2;
    pub const DMA2_CH3: Interrupt = Interrupt::DMA2_CH3;
    pub const DMA2_CH4: Interrupt = Interrupt::DMA2_CH4;
    pub const DMA2_CH5: Interrupt = Interrupt::DMA2_CH5;
    pub const DMA2_CH6: Interrupt = Interrupt::DMA2_CH6;
    pub const DMA2_CH7: Interrupt = Interrupt::DMA2_CH7;
}

#[cfg(feature = "stm32wl5x_cm0p")]
mod irq_num {
    use crate::pac::Interrupt;
    pub const DMA1_CH1: Interrupt = Interrupt::DMA1_CH3_1;
    pub const DMA1_CH2: Interrupt = Interrupt::DMA1_CH3_1;
    pub const DMA1_CH3: Interrupt = Interrupt::DMA1_CH3_1;
    pub const DMA1_CH4: Interrupt = Interrupt::DMA1_CH7_4;
    pub const DMA1_CH5: Interrupt = Interrupt::DMA1_CH7_4;
    pub const DMA1_CH6: Interrupt = Interrupt::DMA1_CH7_4;
    pub const DMA1_CH7: Interrupt = Interrupt::DMA1_CH7_4;
    pub const DMA2_CH1: Interrupt = Interrupt::DMA2_CH7_1_DMAMUX1_OVR;
    pub const DMA2_CH2: Interrupt = Interrupt::DMA2_CH7_1_DMAMUX1_OVR;
    pub const DMA2_CH3: Interrupt = Interrupt::DMA2_CH7_1_DMAMUX1_OVR;
    pub const DMA2_CH4: Interrupt = Interrupt::DMA2_CH7_1_DMAMUX1_OVR;
    pub const DMA2_CH5: Interrupt = Interrupt::DMA2_CH7_1_DMAMUX1_OVR;
    pub const DMA2_CH6: Interrupt = Interrupt::DMA2_CH7_1_DMAMUX1_OVR;
    pub const DMA2_CH7: Interrupt = Interrupt::DMA2_CH7_1_DMAMUX1_OVR;
}

dma_ch!(1, 1, DMA1_CH1);
dma_ch!(1, 2, DMA1_CH2);
dma_ch!(1, 3, DMA1_CH3);
dma_ch!(1, 4, DMA1_CH4);
dma_ch!(1, 5, DMA1_CH5);
dma_ch!(1, 6, DMA1_CH6);
dma_ch!(1, 7, DMA1_CH7);
dma_ch!(2, 1, DMA2_CH1);
dma_ch!(2, 2, DMA2_CH2);
dma_ch!(2, 3, DMA2_CH3);
dma_ch!(2, 4, DMA2_CH4);
dma_ch!(2, 5, DMA2_CH5);
dma_ch!(2, 6, DMA2_CH6);
dma_ch!(2, 7, DMA2_CH7);

/// All DMA channels.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AllDma {
    /// DMA controller 1.
    pub d1: Dma1,
    /// DMA controller 2.
    pub d2: Dma2,
}

/// All DMA controller 1 channels.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Dma1 {
    /// DMA controller 1 channel 1
    pub c1: Dma1Ch1,
    /// DMA controller 1 channel 2
    pub c2: Dma1Ch2,
    /// DMA controller 1 channel 3
    pub c3: Dma1Ch3,
    /// DMA controller 1 channel 4
    pub c4: Dma1Ch4,
    /// DMA controller 1 channel 5
    pub c5: Dma1Ch5,
    /// DMA controller 1 channel 6
    pub c6: Dma1Ch6,
    /// DMA controller 1 channel 7
    pub c7: Dma1Ch7,
}

/// All DMA controller 2 channels.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Dma2 {
    /// DMA controller 2 channel 1
    pub c1: Dma2Ch1,
    /// DMA controller 2 channel 2
    pub c2: Dma2Ch2,
    /// DMA controller 2 channel 3
    pub c3: Dma2Ch3,
    /// DMA controller 2 channel 4
    pub c4: Dma2Ch4,
    /// DMA controller 2 channel 5
    pub c5: Dma2Ch5,
    /// DMA controller 2 channel 6
    pub c6: Dma2Ch6,
    /// DMA controller 2 channel 7
    pub c7: Dma2Ch7,
}

const DMA1: Dma1 = Dma1 {
    c1: Dma1Ch1::new(),
    c2: Dma1Ch2::new(),
    c3: Dma1Ch3::new(),
    c4: Dma1Ch4::new(),
    c5: Dma1Ch5::new(),
    c6: Dma1Ch6::new(),
    c7: Dma1Ch7::new(),
};

const DMA2: Dma2 = Dma2 {
    c1: Dma2Ch1::new(),
    c2: Dma2Ch2::new(),
    c3: Dma2Ch3::new(),
    c4: Dma2Ch4::new(),
    c5: Dma2Ch5::new(),
    c6: Dma2Ch6::new(),
    c7: Dma2Ch7::new(),
};

const ALL_DMA: AllDma = AllDma { d1: DMA1, d2: DMA2 };

impl AllDma {
    /// Split the DMA registers into individual channels.
    ///
    /// This will enable clocks and reset the DMA1, DMA2, and DMAMUX peripherals.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dma::AllDma, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    /// ```
    #[allow(unused_variables)]
    #[inline]
    pub fn split(
        dmamux: pac::DMAMUX,
        dma1: pac::DMA1,
        dma2: pac::DMA2,
        rcc: &mut pac::RCC,
    ) -> Self {
        Self::enable_clocks(rcc);
        unsafe { Self::pulse_resets(rcc) };
        ALL_DMA
    }

    /// Reset the DMA1, DMA2, and DMAMUX peripherals.
    ///
    /// [`split`](Self::split) will pulse reset for you.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the DMA1, DMA2, and DMAMUX peripherals
    ///    before calling this function.
    ///
    /// # Example
    ///
    /// See [`steal`](Self::steal).
    #[inline]
    pub unsafe fn pulse_resets(rcc: &mut pac::RCC) {
        rcc.ahb1rstr.modify(|_, w| {
            w.dmamux1rst().set_bit();
            w.dma2rst().set_bit();
            w.dma1rst().set_bit()
        });
        rcc.ahb1rstr.modify(|_, w| {
            w.dmamux1rst().clear_bit();
            w.dma2rst().clear_bit();
            w.dma1rst().clear_bit()
        });
    }

    /// Enable clocks for the DMA1, DMA2, and DMAMUX peripherals.
    ///
    /// [`split`](Self::split) will enable clocks for you.
    ///
    /// # Example
    ///
    /// See [`steal`](Self::steal).
    #[inline]
    pub fn enable_clocks(rcc: &mut pac::RCC) {
        rcc.ahb1enr.modify(|_, w| {
            w.dmamux1en().enabled();
            w.dma2en().enabled();
            w.dma1en().enabled()
        });
        rcc.ahb1enr.read(); // delay after an RCC peripheral clock enabling
    }

    /// Disable clocks for the DMA1, DMA2, and DMAMUX peripherals.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the DMA1, DMA2, and DMAMUX peripherals before
    ///    disabling the clock.
    /// 2. You are responsible for re-enabling the clock before using the DMA1,
    ///    DMA2, and DMAMUX peripherals.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dma::AllDma, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    /// // ... use DMA channels
    ///
    /// // safety: DMA is not in use
    /// unsafe { AllDma::disable_clocks(&mut dp.RCC) };
    ///
    /// // have a low power nap or something
    ///
    /// AllDma::enable_clocks(&mut dp.RCC);
    /// // ... use DMA channels
    /// ```
    #[inline]
    pub unsafe fn disable_clocks(rcc: &mut pac::RCC) {
        rcc.ahb1enr.modify(|_, w| {
            w.dmamux1en().disabled();
            w.dma2en().disabled();
            w.dma1en().disabled()
        });
    }

    /// Steal all DMA channels.
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the DMA channels has exclusive access.
    ///    Singleton checks are bypassed with this method.
    /// 2. You are responsible for resetting and enabling clocks on the
    ///    DMA1, DMA2, and DMAMUX peripherals.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dma::AllDma, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // DMAs cannot be used via registers now
    /// let _: pac::DMA1 = dp.DMA1;
    /// let _: pac::DMA2 = dp.DMA2;
    /// let _: pac::DMAMUX = dp.DMAMUX;
    ///
    /// // safety: nothing is using the peripherals
    /// unsafe { AllDma::pulse_resets(&mut dp.RCC) };
    ///
    /// AllDma::enable_clocks(&mut dp.RCC);
    ///
    /// // safety
    /// // 1. We have exclusive access
    /// // 2. peripherals have been setup
    /// let dmas: AllDma = unsafe { AllDma::steal() };
    /// ```
    #[inline]
    pub const unsafe fn steal() -> Self {
        ALL_DMA
    }
}

impl Dma1 {
    /// Split the DMA registers into individual channels.
    ///
    /// This will enable clocks and reset the DMA1 and DMAMUX peripherals.
    ///
    /// Most of the time you will want to use [`AllDma::split`].
    /// This is provided for low-power use cases where you do not need
    /// both DMA controllers.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dma::Dma1, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let dma1: Dma1 = Dma1::split(dp.DMAMUX, dp.DMA1, &mut dp.RCC);
    /// ```
    #[allow(unused_variables)]
    #[inline]
    pub fn split(dmamux: pac::DMAMUX, dma1: pac::DMA1, rcc: &mut pac::RCC) -> Self {
        Self::enable_clocks(rcc);
        unsafe { Self::pulse_resets(rcc) };
        DMA1
    }

    /// Reset the DMA1 and DMAMUX peripherals.
    ///
    /// [`split`](Self::split) will pulse reset for you.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the DMA1, and DMAMUX peripherals
    ///    before calling this function.
    ///
    /// # Example
    ///
    /// See [`steal`](Self::steal).
    #[inline]
    pub unsafe fn pulse_resets(rcc: &mut pac::RCC) {
        rcc.ahb1rstr
            .modify(|_, w| w.dmamux1rst().set_bit().dma1rst().set_bit());
        rcc.ahb1rstr
            .modify(|_, w| w.dmamux1rst().clear_bit().dma1rst().clear_bit());
    }

    /// Enable clocks for the DMA1 and DMAMUX peripherals.
    ///
    /// [`split`](Self::split) will enable clocks for you.
    ///
    /// # Example
    ///
    /// See [`steal`](Self::steal).
    #[inline]
    pub fn enable_clocks(rcc: &mut pac::RCC) {
        rcc.ahb1enr
            .modify(|_, w| w.dmamux1en().enabled().dma1en().enabled());
        rcc.ahb1enr.read(); // delay after an RCC peripheral clock enabling
    }

    /// Disable clocks for the DMA1 and DMAMUX peripherals.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the DMA1, and DMAMUX peripherals before
    ///    disabling the clock.
    /// 2. You are responsible for re-enabling the clock before using the DMA1,
    ///    and DMAMUX peripherals.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dma::Dma1, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let dma1: Dma1 = Dma1::split(dp.DMAMUX, dp.DMA1, &mut dp.RCC);
    /// // ... use DMA channels
    ///
    /// // safety: DMA is not in use
    /// unsafe { Dma1::disable_clocks(&mut dp.RCC) };
    ///
    /// // have a low power nap or something
    ///
    /// Dma1::enable_clocks(&mut dp.RCC);
    /// // ... use DMA channels
    /// ```
    #[inline]
    pub unsafe fn disable_clocks(rcc: &mut pac::RCC) {
        rcc.ahb1enr
            .modify(|_, w| w.dmamux1en().disabled().dma1en().disabled());
    }

    /// Steal the DMA1 channels.
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the DMA channels has exclusive access.
    ///    Singleton checks are bypassed with this method.
    /// 2. You are responsible for resetting and enabling clocks on the
    ///    DMA1, and DMAMUX peripherals.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dma::Dma1, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // DMAs cannot be used via registers now
    /// let _: pac::DMA1 = dp.DMA1;
    /// let _: pac::DMAMUX = dp.DMAMUX;
    ///
    /// // safety: nothing is using the peripherals
    /// unsafe { Dma1::pulse_resets(&mut dp.RCC) };
    ///
    /// Dma1::enable_clocks(&mut dp.RCC);
    ///
    /// // safety
    /// // 1. We have exclusive access
    /// // 2. peripherals have been setup
    /// let dmas: Dma1 = unsafe { Dma1::steal() };
    /// ```
    #[inline]
    pub const unsafe fn steal() -> Self {
        DMA1
    }
}
