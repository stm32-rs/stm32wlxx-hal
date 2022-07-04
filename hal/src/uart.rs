//! Universal synchronous/asynchronous receiver transmitter
use crate::{
    dma::{self, DmaCh},
    gpio::{self},
    pac, rcc, Ratio,
};
use cortex_m::interrupt::CriticalSection;
use embedded_hal::prelude::*;

typestate!(NoRx, "no RX on a generic UART structure");
typestate!(NoTx, "no TX on a generic UART structure");

/// UART clock selection.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Clk {
    /// PCLK
    PClk = 0b00,
    /// System clock
    Sysclk = 0b01,
    /// HSI16 clock
    Hsi16 = 0b10,
    /// LSE clock
    Lse = 0b11,
}

/// UART errors.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Overrun.
    Overrun,
    /// Start bit noise detected.
    Noise,
    /// Framing error.
    Framing,
    /// Parity error.
    Parity,
    /// RX DMA error
    ///
    /// This can only occur on UART transfers that use the RX DMA.
    RxDma,
    /// TX DMA error
    ///
    /// This can only occur on UART transfers that use the TX DMA.
    TxDma,
}

/// UART1 driver.
#[derive(Debug)]
pub struct Uart1<RX, TX> {
    uart: pac::USART1,
    rx: RX,
    tx: TX,
}

/// UART2 driver.
#[derive(Debug)]
pub struct Uart2<RX, TX> {
    uart: pac::USART2,
    rx: RX,
    tx: TX,
}

/// Low-power UART driver.
#[derive(Debug)]
pub struct LpUart<RX, TX> {
    uart: pac::LPUART,
    rx: RX,
    tx: TX,
}

impl LpUart<NoRx, NoTx> {
    /// Create a new LPUART driver from a LPUART peripheral.
    ///
    /// This will enable clocks and reset the LPUART peripheral.
    ///
    /// # Panics
    ///
    /// * Source frequency is not between 3× and 4096× the baud rate
    /// * The derived baud rate register value is less than `0x300`
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     uart::{self, LpUart, NoRx, NoTx},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let uart: LpUart<NoRx, NoTx> = LpUart::new(dp.LPUART, 115_200, uart::Clk::Hsi16, &mut dp.RCC);
    /// ```
    pub fn new(uart: pac::LPUART, baud: u32, clk: Clk, rcc: &mut pac::RCC) -> LpUart<NoRx, NoTx> {
        unsafe { Self::pulse_reset(rcc) };
        Self::enable_clock(rcc);

        rcc.ccipr.modify(|_, w| w.lpuart1sel().bits(clk as u8));

        let ret: LpUart<NoRx, NoTx> = LpUart {
            uart,
            rx: NoRx::new(),
            tx: NoTx::new(),
        };

        let baud: u64 = baud.into();
        let freq: u64 = ret.clock_hz(rcc).into();
        assert!(freq >= baud.saturating_mul(3) && freq <= baud.saturating_mul(4096));

        let br: u32 = ((freq * 256) / baud) as u32;
        assert!(br >= 0x300);
        ret.uart.brr.write(|w| unsafe { w.brr().bits(br) });
        ret.uart.cr1.write(|w| w.ue().set_bit().fifoen().set_bit());

        ret
    }
}

impl Uart1<NoRx, NoTx> {
    /// Create a new UART driver from a UART peripheral.
    ///
    /// This will enable clocks and reset the UART peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     uart::{self, NoRx, NoTx, Uart1},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let uart: Uart1<NoRx, NoTx> = Uart1::new(dp.USART1, 115_200, uart::Clk::Hsi16, &mut dp.RCC);
    /// ```
    pub fn new(uart: pac::USART1, baud: u32, clk: Clk, rcc: &mut pac::RCC) -> Uart1<NoRx, NoTx> {
        unsafe { Self::pulse_reset(rcc) };
        Self::enable_clock(rcc);

        rcc.ccipr.modify(|_, w| w.usart1sel().bits(clk as u8));

        let ret: Uart1<NoRx, NoTx> = Uart1 {
            uart,
            rx: NoRx::new(),
            tx: NoTx::new(),
        };

        let freq: u32 = ret.clock_hz(rcc);

        // only for oversampling of 16 (default), change for oversampling of 8
        let br: u16 = (freq / baud) as u16;
        ret.uart.brr.write(|w| w.brr().bits(br));
        ret.uart.cr1.write(|w| w.ue().set_bit().fifoen().set_bit());

        ret
    }
}

impl Uart2<NoRx, NoTx> {
    /// Create a new UART driver from a UART peripheral.
    ///
    /// This will enable clocks and reset the UART peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     uart::{self, NoRx, NoTx, Uart2},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// // enable the HSI16 source clock
    /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    ///
    /// let uart: Uart2<NoRx, NoTx> = Uart2::new(dp.USART2, 115_200, uart::Clk::Hsi16, &mut dp.RCC);
    /// ```
    pub fn new(uart: pac::USART2, baud: u32, clk: Clk, rcc: &mut pac::RCC) -> Uart2<NoRx, NoTx> {
        unsafe { Self::pulse_reset(rcc) };
        Self::enable_clock(rcc);

        rcc.ccipr.modify(|_, w| w.usart2sel().bits(clk as u8));

        let ret: Uart2<NoRx, NoTx> = Uart2 {
            uart,
            rx: NoRx::new(),
            tx: NoTx::new(),
        };

        let freq: u32 = ret.clock_hz(rcc);

        // only for oversampling of 16 (default), change for oversampling of 8
        let br: u16 = (freq / baud) as u16;
        ret.uart.brr.write(|w| w.brr().bits(br));
        ret.uart.cr1.write(|w| w.ue().set_bit().fifoen().set_bit());

        ret
    }
}

const LPUART_BASE: usize = 0x4000_8000;
const UART1_BASE: usize = 0x4001_3800;
const UART2_BASE: usize = 0x4000_4400;
const RDR_OFFSET: usize = 0x24;
const TDR_OFFSET: usize = 0x28;

macro_rules! impl_consts {
    ($uart:ident, $rx_req_id:expr, $tx_req_id:expr, $base:expr) => {
        impl<RX, TX> $uart<RX, TX> {
            const DMA_RX_ID: u8 = $rx_req_id;
            const DMA_TX_ID: u8 = $tx_req_id;
            const RDR: usize = $base + RDR_OFFSET;
            const TDR: usize = $base + TDR_OFFSET;
        }
    };
}

impl_consts!(LpUart, 21, 22, LPUART_BASE);
impl_consts!(Uart1, 17, 18, UART1_BASE);
impl_consts!(Uart2, 19, 20, UART2_BASE);

macro_rules! impl_clock_hz {
    ($uart:ident, $sel:ident, $presc:ident, $method:ident, $pclk_method:ident) => {
        impl<RX, TX> $uart<RX, TX> {
            /// Calculate the clock frequency.
            ///
            /// Fractional frequencies will be rounded towards zero.
            pub fn clock_hz(&self, rcc: &pac::RCC) -> u32 {
                use pac::{rcc::ccipr::$sel, $presc::presc::PRESCALER_A};
                let src: Ratio<u32> = match rcc.ccipr.read().$method().variant() {
                    $sel::Pclk => {
                        let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
                        rcc::$pclk_method(rcc, &cfgr)
                    }
                    $sel::Sysclk => {
                        let cfgr: pac::rcc::cfgr::R = rcc.cfgr.read();
                        rcc::sysclk(rcc, &cfgr)
                    }
                    $sel::Hsi16 => Ratio::new_raw(16_000_000, 1),
                    $sel::Lse => Ratio::new_raw(32_768, 1),
                };
                let pre: u32 = match self.uart.presc.read().prescaler().variant() {
                    Some(p) => match p {
                        PRESCALER_A::Div1 => 1,
                        PRESCALER_A::Div2 => 2,
                        PRESCALER_A::Div4 => 4,
                        PRESCALER_A::Div6 => 6,
                        PRESCALER_A::Div8 => 8,
                        PRESCALER_A::Div10 => 10,
                        PRESCALER_A::Div12 => 12,
                        PRESCALER_A::Div16 => 16,
                        PRESCALER_A::Div32 => 32,
                        PRESCALER_A::Div64 => 64,
                        PRESCALER_A::Div128 => 128,
                        PRESCALER_A::Div256 => 256,
                    },
                    None => 256,
                };
                (src / pre).to_integer()
            }
        }
    };
}

impl_clock_hz!(LpUart, LPUART1SEL_A, lpuart, lpuart1sel, pclk1);
impl_clock_hz!(Uart1, USART1SEL_A, usart1, usart1sel, pclk2);
impl_clock_hz!(Uart2, USART1SEL_A, usart1, usart2sel, pclk1);

macro_rules! impl_pulse_reset {
    ($uart:ident, $reg:ident, $method:ident) => {
        impl $uart<NoRx, NoTx> {
            /// Reset the UART.
            ///
            /// # Safety
            ///
            /// 1. The UART must not be in-use.
            /// 2. You are responsible for setting up the UART after a reset.
            ///
            /// # Example
            ///
            /// See [`steal`](Self::steal)
            pub unsafe fn pulse_reset(rcc: &mut pac::RCC) {
                rcc.$reg.modify(|_, w| w.$method().set_bit());
                rcc.$reg.modify(|_, w| w.$method().clear_bit());
            }
        }
    };
}

impl_pulse_reset!(LpUart, apb1rstr2, lpuart1rst);
impl_pulse_reset!(Uart1, apb2rstr, usart1rst);
impl_pulse_reset!(Uart2, apb1rstr1, usart2rst);

macro_rules! impl_clock_en_dis {
    ($uart:ident, $reg:ident, $method:ident) => {
        impl $uart<NoRx, NoTx> {
            /// Enable the UART clock.
            ///
            /// This is done for you in [`new`](Self::new)
            ///
            /// # Example
            ///
            /// ```no_run
            /// use stm32wlxx_hal::{pac, uart::LpUart};
            ///
            /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
            /// LpUart::enable_clock(&mut dp.RCC);
            /// ```
            pub fn enable_clock(rcc: &mut pac::RCC) {
                rcc.$reg.modify(|_, w| w.$method().enabled())
            }

            /// Disable the UART clock.
            ///
            /// # Safety
            ///
            /// 1. You are responsible for ensuring the UART is in a state where
            ///    the clock can be disabled without entering an error state.
            /// 2. You cannot use the UART while the clock is disabled.
            /// 3. You are responsible for re-enabling the clock before resuming
            ///    use of the UART.
            /// 4. You are responsible for setting up anything that may have lost
            ///    state while the clock was disabled.
            pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
                rcc.$reg.modify(|_, w| w.$method().disabled())
            }
        }
    };
}

impl_clock_en_dis!(LpUart, apb1enr2, lpuart1en);
impl_clock_en_dis!(Uart1, apb2enr, usart1en);
impl_clock_en_dis!(Uart2, apb1enr1, usart2en);

macro_rules! impl_free_steal {
    ($uart:ident, $periph:ident) => {
        impl $uart<NoRx, NoTx> {
            /// Steal the UART peripheral from whatever is currently using it.
            ///
            /// This will **not** initialize the peripheral (unlike [`new`]).
            ///
            /// # Safety
            ///
            /// 1. Ensure that the code stealing the UART has exclusive access to the
            ///    peripheral. Singleton checks are bypassed with this method.
            /// 2. You are responsible for setting up the UART correctly.
            ///
            /// # Example
            ///
            /// ```no_run
            /// use stm32wlxx_hal::{
            ///     pac,
            ///     uart::{LpUart, NoRx, NoTx},
            /// };
            ///
            /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
            ///
            /// LpUart::enable_clock(&mut dp.RCC);
            /// // safety:
            /// // 1. Nothing else is using the LPUART in this code.
            /// // 2. This code performs setup for the LPUART.
            /// unsafe { LpUart::pulse_reset(&mut dp.RCC) };
            ///
            /// // safety:
            /// // 1. Nothing else is using the LPUART in this code.
            /// // 2. The LPUART has been setup, clocks are enabled and the LPUART has been reset.
            /// let mut lpuart: LpUart<NoRx, NoTx> = unsafe { LpUart::steal() };
            /// ```
            ///
            /// [`new`]: Self::new
            pub unsafe fn steal() -> $uart<NoRx, NoTx> {
                $uart {
                    uart: pac::Peripherals::steal().$periph,
                    rx: NoRx::new(),
                    tx: NoTx::new(),
                }
            }
        }

        impl<RX, TX> $uart<RX, TX> {
            /// Free the UART peripheral from the driver.
            ///
            /// # Example
            ///
            /// ```no_run
            /// use stm32wlxx_hal::{
            ///     pac,
            ///     uart::{self, LpUart, NoRx, NoTx},
            /// };
            ///
            /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
            /// let lpuart: pac::LPUART = dp.LPUART;
            ///
            /// let lpuart: LpUart<NoRx, NoTx> =
            ///     LpUart::new(lpuart, 115_200, uart::Clk::Hsi16, &mut dp.RCC);
            /// // ... use LPUART
            /// let lpuart: pac::LPUART = lpuart.free();
            /// ```
            pub fn free(self) -> pac::$periph {
                self.uart
            }
        }
    };
}

impl_free_steal!(LpUart, LPUART);
impl_free_steal!(Uart1, USART1);
impl_free_steal!(Uart2, USART2);

macro_rules! impl_tx_en_dis {
    ($uart:ident, $trt:ident, $method:ident) => {
        impl<RX> $uart<RX, NoTx> {
            /// Enable the UART transmitter.
            ///
            /// # Example
            ///
            /// ```no_run
            /// use stm32wlxx_hal::{
            ///     cortex_m,
            ///     gpio::{pins, PortB},
            ///     pac,
            ///     uart::{self, LpUart, NoRx},
            /// };
            ///
            /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
            ///
            /// // enable the HSI16 source clock
            /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
            /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
            ///
            /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
            /// let uart: LpUart<NoRx, pins::B11> = cortex_m::interrupt::free(|cs| {
            ///     LpUart::new(dp.LPUART, 115_200, uart::Clk::Hsi16, &mut dp.RCC)
            ///         .enable_tx(gpiob.b11, cs)
            /// });
            /// ```
            pub fn enable_tx<TX: gpio::sealed::$trt>(
                self,
                mut tx: TX,
                cs: &CriticalSection,
            ) -> $uart<RX, TX> {
                tx.$method(cs);
                self.uart.cr1.modify(|_, w| w.te().enabled());
                $uart {
                    uart: self.uart,
                    rx: self.rx,
                    tx,
                }
            }
        }

        impl<RX> $uart<RX, NoTx> {
            /// Enable the UART transmitter with a DMA channel.
            ///
            /// # Example
            ///
            /// ```no_run
            /// use stm32wlxx_hal::{
            ///     cortex_m,
            ///     dma::{AllDma, Dma2Ch7},
            ///     gpio::{pins, PortB},
            ///     pac,
            ///     uart::{self, LpUart, NoRx},
            /// };
            ///
            /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
            ///
            /// // enable the HSI16 source clock
            /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
            /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
            ///
            /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
            /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
            /// let uart: LpUart<NoRx, (pins::B11, Dma2Ch7)> = cortex_m::interrupt::free(|cs| {
            ///     LpUart::new(dp.LPUART, 115_200, uart::Clk::Hsi16, &mut dp.RCC)
            ///         .enable_tx_dma(gpiob.b11, dma.d2.c7, cs)
            /// });
            /// ```
            pub fn enable_tx_dma<TxPin: gpio::sealed::$trt, TxDma: DmaCh>(
                self,
                mut tx: TxPin,
                mut tx_dma: TxDma,
                cs: &CriticalSection,
            ) -> $uart<RX, (TxPin, TxDma)> {
                tx.$method(cs);
                self.uart.cr1.modify(|_, w| w.te().enabled());
                self.uart.cr3.modify(|_, w| w.dmat().enabled());

                tx_dma.set_cr(dma::Cr::DISABLE);
                tx_dma.clear_all_flags();
                tx_dma.set_periph_addr(Self::TDR as u32);
                tx_dma.set_mux_cr_reqid(Self::DMA_TX_ID);

                $uart {
                    uart: self.uart,
                    rx: self.rx,
                    tx: (tx, tx_dma),
                }
            }
        }

        impl<RX, TX> $uart<RX, TX> {
            /// Disable the UART transmitter.
            ///
            /// # Example
            ///
            /// ```no_run
            /// use stm32wlxx_hal::{
            ///     cortex_m,
            ///     gpio::{pins, PortB},
            ///     pac,
            ///     uart::{self, LpUart, NoRx, NoTx},
            /// };
            ///
            /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
            ///
            /// // enable the HSI16 source clock
            /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
            /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
            ///
            /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
            /// let uart: LpUart<NoRx, pins::B11> = cortex_m::interrupt::free(|cs| {
            ///     LpUart::new(dp.LPUART, 115_200, uart::Clk::Hsi16, &mut dp.RCC)
            ///         .enable_tx(gpiob.b11, cs)
            /// });
            ///
            /// let (uart, b11): (LpUart<NoRx, NoTx>, pins::B11) = uart.disable_tx();
            /// ```
            pub fn disable_tx(self) -> ($uart<RX, NoTx>, TX) {
                self.uart.cr1.modify(|_, w| w.te().disabled());
                self.uart.cr3.modify(|_, w| w.dmat().disabled());
                (
                    $uart {
                        uart: self.uart,
                        rx: self.rx,
                        tx: NoTx::new(),
                    },
                    self.tx,
                )
            }
        }
    };
}

impl_tx_en_dis!(LpUart, LpUart1Tx, set_lpuart1_tx_af);
impl_tx_en_dis!(Uart1, Uart1Tx, set_uart1_tx_af);
impl_tx_en_dis!(Uart2, Uart2Tx, set_uart2_tx_af);

macro_rules! impl_rx_en_dis {
    ($uart:ident, $trt:ident, $method:ident) => {
        impl<TX> $uart<NoRx, TX> {
            /// Enable the UART receiver.
            ///
            /// # Example
            ///
            /// ```no_run
            /// use stm32wlxx_hal::{
            ///     cortex_m,
            ///     gpio::{pins, PortB},
            ///     pac,
            ///     uart::{self, LpUart, NoTx},
            /// };
            ///
            /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
            ///
            /// // enable the HSI16 source clock
            /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
            /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
            ///
            /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
            /// let uart: LpUart<pins::B10, NoTx> = cortex_m::interrupt::free(|cs| {
            ///     LpUart::new(dp.LPUART, 115_200, uart::Clk::Hsi16, &mut dp.RCC)
            ///         .enable_rx(gpiob.b10, cs)
            /// });
            /// ```
            pub fn enable_rx<RX: gpio::sealed::$trt>(
                self,
                mut rx: RX,
                cs: &CriticalSection,
            ) -> $uart<RX, TX> {
                rx.$method(cs);
                self.uart.cr1.modify(|_, w| w.re().enabled());
                $uart {
                    uart: self.uart,
                    rx,
                    tx: self.tx,
                }
            }
        }

        impl<TX> $uart<NoRx, TX> {
            /// Enable the UART receiver with a DMA channel.
            ///
            /// # Example
            ///
            /// ```no_run
            /// use stm32wlxx_hal::{
            ///     dma::{AllDma, Dma2Ch2},
            ///     gpio::{pins, PortB},
            ///     pac,
            ///     uart::{self, LpUart, NoTx},
            /// };
            ///
            /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
            ///
            /// // enable the HSI16 source clock
            /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
            /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
            ///
            /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
            /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
            /// let uart: LpUart<(pins::B10, Dma2Ch2), NoTx> = cortex_m::interrupt::free(|cs| {
            ///     LpUart::new(dp.LPUART, 115_200, uart::Clk::Hsi16, &mut dp.RCC)
            ///         .enable_rx_dma(gpiob.b10, dma.d2.c2, cs)
            /// });
            /// ```
            pub fn enable_rx_dma<RxPin: gpio::sealed::$trt, RxDma: DmaCh>(
                self,
                mut rx: RxPin,
                mut rx_dma: RxDma,
                cs: &CriticalSection,
            ) -> $uart<(RxPin, RxDma), TX> {
                rx.$method(cs);
                self.uart.cr1.modify(|_, w| w.re().enabled());
                self.uart.cr3.modify(|_, w| w.dmar().enabled());

                rx_dma.set_cr(dma::Cr::DISABLE);
                rx_dma.clear_all_flags();
                rx_dma.set_periph_addr(Self::RDR as u32);
                rx_dma.set_mux_cr_reqid(Self::DMA_RX_ID);

                $uart {
                    uart: self.uart,
                    rx: (rx, rx_dma),
                    tx: self.tx,
                }
            }
        }

        impl<RX, TX> $uart<RX, TX> {
            /// Disable the UART receiver.
            ///
            /// # Example
            ///
            /// ```no_run
            /// use stm32wlxx_hal::{
            ///     gpio::{pins, PortB},
            ///     pac,
            ///     uart::{self, LpUart, NoRx, NoTx},
            /// };
            ///
            /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
            ///
            /// // enable the HSI16 source clock
            /// dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
            /// while dp.RCC.cr.read().hsirdy().is_not_ready() {}
            ///
            /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
            /// let uart: LpUart<pins::B10, NoTx> = cortex_m::interrupt::free(|cs| {
            ///     LpUart::new(dp.LPUART, 115_200, uart::Clk::Hsi16, &mut dp.RCC)
            ///         .enable_rx(gpiob.b10, cs)
            /// });
            ///
            /// let (uart, b10): (LpUart<NoRx, NoTx>, pins::B10) = uart.disable_rx();
            /// ```
            pub fn disable_rx(self) -> ($uart<NoRx, TX>, RX) {
                self.uart.cr1.modify(|_, w| w.re().disabled());
                self.uart.cr3.modify(|_, w| w.dmar().disabled());
                (
                    $uart {
                        uart: self.uart,
                        rx: NoRx::new(),
                        tx: self.tx,
                    },
                    self.rx,
                )
            }
        }
    };
}

impl_rx_en_dis!(LpUart, LpUart1Rx, set_lpuart1_rx_af);
impl_rx_en_dis!(Uart1, Uart1Rx, set_uart1_rx_af);
impl_rx_en_dis!(Uart2, Uart2Rx, set_uart2_rx_af);

macro_rules! impl_status {
    ($uart:ident, $pacmod:ident) => {
        impl<RX, TX> $uart<RX, TX> {
            #[inline]
            fn status(&self) -> Result<pac::$pacmod::isr::R, Error> {
                let isr = self.uart.isr.read();
                if isr.pe().bit_is_set() {
                    Err(Error::Parity)
                } else if isr.fe().bit_is_set() {
                    Err(Error::Framing)
                } else if isr.ne().bit_is_set() {
                    Err(Error::Noise)
                } else if isr.ore().bit_is_set() {
                    Err(Error::Overrun)
                } else {
                    Ok(isr)
                }
            }
        }
    };
}

impl_status!(LpUart, lpuart);
impl_status!(Uart1, usart1);
impl_status!(Uart2, usart1);

macro_rules! impl_eh_traits {
    ($uart:ident, $rx_trait:ident, $tx_trait:ident, $rxne:ident, $txnf:ident) => {
        impl<RX, TX> embedded_hal::serial::Write<u8> for $uart<RX, TX>
        where
            TX: gpio::sealed::$tx_trait,
        {
            type Error = Error;

            #[inline]
            fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
                if self.status()?.$txnf().bit_is_set() {
                    self.uart.tdr.write(|w| w.tdr().bits(word as u16));
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }

            #[inline]
            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                if self.status()?.busy().bit_is_set() {
                    Err(nb::Error::WouldBlock)
                } else {
                    Ok(())
                }
            }
        }

        impl<RX, TxPin, TxDma> embedded_hal::blocking::serial::Write<u8>
            for $uart<RX, (TxPin, TxDma)>
        where
            TxPin: gpio::sealed::$tx_trait,
            TxDma: DmaCh,
        {
            type Error = Error;

            fn bwrite_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
                if buffer.is_empty() {
                    return Ok(());
                }

                const CR: dma::Cr = dma::Cr::RESET
                    .set_dir_from_mem()
                    .set_mem_inc(true)
                    .set_enable(true);

                self.tx.1.set_mem_addr(buffer.as_ptr() as u32);

                let ndt: u32 = buffer.len() as u32;
                self.tx.1.set_num_data_xfer(ndt);

                self.tx.1.set_cr(CR);

                let ret: Result<(), Error> = loop {
                    self.status()?;
                    let dma_flags: u8 = self.tx.1.flags();
                    if dma_flags & dma::flags::XFER_ERR != 0 {
                        break Err(Error::TxDma);
                    } else if dma_flags & dma::flags::XFER_CPL != 0 {
                        break Ok(());
                    }
                };

                self.tx.1.set_cr(dma::Cr::DISABLE);
                self.tx.1.clear_all_flags();

                ret
            }

            #[inline]
            fn bflush(&mut self) -> Result<(), Self::Error> {
                while self.status()?.busy().bit_is_set() {}
                Ok(())
            }
        }

        impl<RX, TX> embedded_hal::serial::Read<u8> for $uart<RX, TX>
        where
            RX: gpio::sealed::$rx_trait,
        {
            type Error = Error;

            #[inline]
            fn read(&mut self) -> nb::Result<u8, Self::Error> {
                if self.status()?.$rxne().bit_is_set() {
                    Ok(self.uart.rdr.read().rdr().bits() as u8)
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }

        impl<RxPin, RxDma, TX> $uart<(RxPin, RxDma), TX>
        where
            RxPin: gpio::sealed::$rx_trait,
            RxDma: DmaCh,
        {
            /// This is not an embedded-hal trait, it is added simply for
            /// parity with what exists on the TX side.
            pub fn bread_all(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
                if buffer.is_empty() {
                    return Ok(());
                }

                const CR: dma::Cr = dma::Cr::RESET
                    .set_dir_from_periph()
                    .set_mem_inc(true)
                    .set_enable(true);

                self.rx.1.set_mem_addr(buffer.as_ptr() as u32);

                let ndt: u32 = buffer.len() as u32;
                self.rx.1.set_num_data_xfer(ndt);

                self.rx.1.set_cr(CR);

                let ret: Result<(), Error> = loop {
                    self.status()?;
                    let dma_flags: u8 = self.rx.1.flags();
                    if dma_flags & dma::flags::XFER_ERR != 0 {
                        break Err(Error::TxDma);
                    } else if dma_flags & dma::flags::XFER_CPL != 0 {
                        break Ok(());
                    }
                };

                self.rx.1.set_cr(dma::Cr::DISABLE);
                self.rx.1.clear_all_flags();

                ret
            }
        }

        impl<RX, TX> core::fmt::Write for $uart<RX, TX>
        where
            $uart<RX, TX>: embedded_hal::serial::Write<u8>,
        {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                let _ = s
                    .as_bytes()
                    .into_iter()
                    .map(|c| nb::block!(self.write(*c)))
                    .last();
                Ok(())
            }
        }
    };
}

impl_eh_traits!(LpUart, LpUart1Rx, LpUart1Tx, rxfne, txfnf);
impl_eh_traits!(Uart1, Uart1Rx, Uart1Tx, rxne, txe);
impl_eh_traits!(Uart2, Uart2Rx, Uart2Tx, rxne, txe);
