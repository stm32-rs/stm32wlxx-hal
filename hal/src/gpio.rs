//! General purpose input-output pins

use crate::{adc, pac};
use core::ptr::{read_volatile, write_volatile};
use cortex_m::interrupt::CriticalSection;

/// GPIO output types.
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputType {
    /// Push-pull output.
    PushPull = 0b0,
    /// Open-drain output.
    ///
    /// This is typically used with [`Pull::Up`].
    OpenDrain = 0b1,
}

/// GPIO speeds.
///
/// Refer to the device datasheet for the frequency specifications and the power
/// supply and load conditions for each speed.
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// Low speed.
    Low = 0b00,
    /// Medium speed.
    Medium = 0b01,
    /// Fast speed.
    Fast = 0b10,
    /// High speed.
    High = 0b11,
}

/// GPIO pull-up and pull-down.
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]

pub enum Pull {
    /// No pull-up, no pull-down.
    None = 0b00,
    /// Pull-up.
    Up = 0b01,
    /// Pull-down.
    Down = 0b10,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Pin<const BASE: usize, const N: u8> {}

impl<const BASE: usize, const N: u8> Pin<BASE, N> {
    const MODER_R: *const u32 = BASE as *const u32;
    const MODER_W: *mut u32 = BASE as *mut u32;
    const OTYPER_R: *const u32 = (BASE + 0x4) as *const u32;
    const OTYPER_W: *mut u32 = (BASE + 0x4) as *mut u32;
    const OSPEEDR_R: *const u32 = (BASE + 0x8) as *const u32;
    const OSPEEDR_W: *mut u32 = (BASE + 0x8) as *mut u32;
    const PUPDR_R: *const u32 = (BASE + 0xC) as *const u32;
    const PUPDR_W: *mut u32 = (BASE + 0xC) as *mut u32;
    const IDR: *const u32 = (BASE + 0x10) as *const u32;
    const ODR: *const u32 = (BASE + 0x14) as *const u32;
    const BSRR: *mut u32 = (BASE + 0x18) as *mut u32;

    const AF: usize = if N > 7 { BASE + 0x24 } else { BASE + 0x20 };
    const AF_R: *const u32 = Self::AF as *const u32;
    const AF_W: *mut u32 = Self::AF as *mut u32;
    const AF_SHIFT: u8 = if N > 7 { (N - 8) * 4 } else { N * 4 };

    pub(crate) const fn new() -> Pin<BASE, N> {
        Pin {}
    }

    #[inline(always)]
    pub(crate) unsafe fn set_mode(&mut self, _cs: &CriticalSection, mode: sealed::Mode) {
        let mut val: u32 = read_volatile(Self::MODER_R);
        val &= !(0b11 << (N * 2));
        val |= (mode as u8 as u32) << (N * 2);
        write_volatile(Self::MODER_W, val);
    }

    #[inline(always)]
    pub(crate) unsafe fn set_output_type(&mut self, _cs: &CriticalSection, ot: OutputType) {
        let mut val: u32 = read_volatile(Self::OTYPER_R);
        match ot {
            OutputType::PushPull => val &= !(1 << N),
            OutputType::OpenDrain => val |= 1 << N,
        }
        write_volatile(Self::OTYPER_W, val);
    }

    #[inline(always)]
    pub(crate) unsafe fn set_speed(&mut self, _cs: &CriticalSection, speed: Speed) {
        let mut val: u32 = read_volatile(Self::OSPEEDR_R);
        val &= !(0b11 << (N * 2));
        val |= (speed as u8 as u32) << (N * 2);
        write_volatile(Self::OSPEEDR_W, val);
    }

    #[inline(always)]
    pub(crate) unsafe fn set_pull(&mut self, _cs: &CriticalSection, pull: Pull) {
        let mut val: u32 = read_volatile(Self::PUPDR_R);
        val &= !(0b11 << (N * 2));
        val |= (pull as u8 as u32) << (N * 2);
        write_volatile(Self::PUPDR_W, val);
    }

    #[inline(always)]
    pub(crate) fn input_level(&self) -> Level {
        if unsafe { read_volatile(Self::IDR) } & (1 << N) == 0 {
            Level::Low
        } else {
            Level::High
        }
    }

    #[inline(always)]
    pub(crate) fn output_level(&self) -> Level {
        if unsafe { read_volatile(Self::ODR) } & (1 << N) == 0 {
            Level::Low
        } else {
            Level::High
        }
    }

    #[inline(always)]
    pub(crate) fn set_output_level(&mut self, level: Level) {
        let val: u32 = match level {
            Level::Low => 1 << (N + 16),
            Level::High => 1 << N,
        };
        unsafe { write_volatile(Self::BSRR, val) }
    }

    #[inline(always)]
    pub(crate) unsafe fn set_alternate_function(&mut self, cs: &CriticalSection, af: u8) {
        self.set_mode(cs, sealed::Mode::Alternate);
        let mut val: u32 = read_volatile(Self::AF_R);
        val &= !(0b1111 << Self::AF_SHIFT);
        val |= (af as u8 as u32) << Self::AF_SHIFT;
        write_volatile(Self::AF_W, val);
    }
}

pub(crate) mod sealed {
    use super::{adc, CriticalSection, Level, OutputType, Pull, Speed};

    /// GPIO modes.
    #[repr(u8)]
    pub enum Mode {
        Input = 0b00,
        Output = 0b01,
        Alternate = 0b10,
        Analog = 0b11,
    }

    /// This is the same methods as Pin, but in a trait so that the individual
    /// Pin structures can implement it in a light wrapper without putting a ton
    /// of code into the macro which will result in longer compile times.
    pub trait PinOps {
        unsafe fn steal() -> Self;
        unsafe fn set_mode(&mut self, cs: &CriticalSection, mode: Mode);
        unsafe fn set_output_type(&mut self, cs: &CriticalSection, ot: OutputType);
        unsafe fn set_speed(&mut self, cs: &CriticalSection, speed: Speed);
        unsafe fn set_pull(&mut self, cs: &CriticalSection, pull: Pull);
        fn input_level(&self) -> Level;
        fn output_level(&self) -> Level;
        fn set_output_level(&mut self, level: Level);
        unsafe fn set_alternate_function(&mut self, cs: &CriticalSection, af: u8);
    }

    macro_rules! af_trait {
        ($trt:ident, $method:ident) => {
            pub trait $trt {
                fn $method(&mut self, cs: &CriticalSection);
            }
        };
    }

    af_trait!(Spi1Mosi, set_spi1_mosi_af);
    af_trait!(Spi1Miso, set_spi1_miso_af);
    af_trait!(Spi1Sck, set_spi1_sck_af);
    af_trait!(Spi1Nss, set_spi1_nss_af);
    af_trait!(Spi2Mosi, set_spi2_mosi_af);
    af_trait!(Spi2Miso, set_spi2_miso_af);
    af_trait!(Spi2Sck, set_spi2_sck_af);
    af_trait!(Spi2Nss, set_spi2_nss_af);
    af_trait!(SubGhzSpiMosi, set_subghz_spi_mosi_af);
    af_trait!(SubGhzSpiMiso, set_subghz_spi_miso_af);
    af_trait!(SubGhzSpiSck, set_subghz_spi_sck_af);
    af_trait!(SubGhzSpiNss, set_subghz_spi_nss_af);
    af_trait!(I2c1Sda, set_i2c1_sda_af);
    af_trait!(I2c1Scl, set_i2c1_scl_af);
    af_trait!(I2c1Smba, set_i2c1_smba_af);
    af_trait!(I2c2Sda, set_i2c2_sda_af);
    af_trait!(I2c2Scl, set_i2c2_scl_af);
    af_trait!(I2c2Smba, set_i2c2_smba_af);
    af_trait!(I2c3Sda, set_i2c3_sda_af);
    af_trait!(I2c3Scl, set_i2c3_scl_af);
    af_trait!(I2c3Smba, set_i2c3_smba_af);
    af_trait!(RfBusy, set_rfbusy_af);
    af_trait!(RfIrq0, set_rf_irq0_af);
    af_trait!(RfIrq1, set_rf_irq1_af);
    af_trait!(RfIrq2, set_rf_irq2_af);
    af_trait!(Uart1Ck, set_uart1_ck_af);
    af_trait!(Uart1Tx, set_uart1_tx_af);
    af_trait!(Uart1Rx, set_uart1_rx_af);
    af_trait!(Uart1Cts, set_uart1_cts_af);
    af_trait!(Uart1Rts, set_uart1_rts_af);
    af_trait!(Uart2Ck, set_uart2_ck_af);
    af_trait!(Uart2Tx, set_uart2_tx_af);
    af_trait!(Uart2Rx, set_uart2_rx_af);
    af_trait!(Uart2Cts, set_uart2_cts_af);
    af_trait!(Uart2Rts, set_uart2_rts_af);
    af_trait!(LpUart1Ck, set_lpuart1_ck_af);
    af_trait!(LpUart1Tx, set_lpuart1_tx_af);
    af_trait!(LpUart1Rx, set_lpuart1_rx_af);
    af_trait!(LpUart1Cts, set_lpuart1_cts_af);
    af_trait!(LpUart1Rts, set_lpuart1_rts_af);
    af_trait!(LpUart1RtsDe, set_lpuart1_rts_de_af);
    af_trait!(IrOut, set_irout_af);
    af_trait!(LpTim1Out, set_lptim1_out_af);
    af_trait!(LpTim1In1, set_lptim1_in1_af);
    af_trait!(LpTim1In2, set_lptim1_in2_af);
    af_trait!(LpTim1Etr, set_lptim1_etr_af);
    af_trait!(LpTim2Out, set_lptim2_out_af);
    af_trait!(LpTim2In1, set_lptim2_in1_af);
    af_trait!(LpTim2Etr, set_lptim2_etr_af);
    af_trait!(LpTim3Out, set_lptim3_out_af);
    af_trait!(LpTim3Etr, set_lptim3_etr_af);
    af_trait!(LpTim3In1, set_lptim3_in1_af);

    /// Indicate a GPIO pin can be sampled by the ADC
    pub trait AdcCh {
        const ADC_CH: adc::Ch;
    }
}

/// GPIO pins
pub mod pins {
    // Switch to this when avaliable on stable
    // https://github.com/rust-lang/rust/issues/51910
    // const GPIOA_BASE: usize = pac::GPIOA::ptr() as *const _ as usize;
    // const GPIOB_BASE: usize = pac::GPIOB::ptr() as *const _ as usize;
    // const GPIOC_BASE: usize = pac::GPIOC::ptr() as *const _ as usize;

    const GPIOA_BASE: usize = 0x4800_0000;
    const GPIOB_BASE: usize = 0x4800_0400;
    const GPIOC_BASE: usize = 0x4800_0800;

    use super::{adc, CriticalSection, Level, OutputType, Pin, Pull, Speed};

    macro_rules! gpio_struct {
        ($name:ident, $base:expr, $n:expr, $doc:expr) => {
            #[doc=$doc]
            #[derive(Debug)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub struct $name {
                pin: Pin<$base, $n>,
            }

            impl $name {
                pub(crate) const fn new() -> Self {
                    $name { pin: Pin::new() }
                }
            }

            impl super::sealed::PinOps for $name {
                #[inline(always)]
                unsafe fn steal() -> Self {
                    Self::new()
                }

                #[inline(always)]
                unsafe fn set_mode(&mut self, cs: &CriticalSection, mode: super::sealed::Mode) {
                    self.pin.set_mode(cs, mode)
                }

                #[inline(always)]
                unsafe fn set_output_type(&mut self, cs: &CriticalSection, ot: OutputType) {
                    self.pin.set_output_type(cs, ot)
                }

                #[inline(always)]
                unsafe fn set_speed(&mut self, cs: &CriticalSection, speed: Speed) {
                    self.pin.set_speed(cs, speed)
                }

                #[inline(always)]
                unsafe fn set_pull(&mut self, cs: &CriticalSection, pull: Pull) {
                    self.pin.set_pull(cs, pull)
                }

                #[inline(always)]
                fn input_level(&self) -> Level {
                    self.pin.input_level()
                }

                #[inline(always)]
                fn output_level(&self) -> Level {
                    self.pin.output_level()
                }

                #[inline(always)]
                fn set_output_level(&mut self, level: Level) {
                    self.pin.set_output_level(level)
                }

                #[inline(always)]
                unsafe fn set_alternate_function(&mut self, cs: &CriticalSection, af: u8) {
                    self.pin.set_alternate_function(cs, af)
                }
            }
        };
    }

    gpio_struct!(A0, GPIOA_BASE, 0, "Port A pin 0");
    gpio_struct!(A1, GPIOA_BASE, 1, "Port A pin 1");
    gpio_struct!(A2, GPIOA_BASE, 2, "Port A pin 2");
    gpio_struct!(A3, GPIOA_BASE, 3, "Port A pin 3");
    gpio_struct!(A4, GPIOA_BASE, 4, "Port A pin 4");
    gpio_struct!(A5, GPIOA_BASE, 5, "Port A pin 5");
    gpio_struct!(A6, GPIOA_BASE, 6, "Port A pin 6");
    gpio_struct!(A7, GPIOA_BASE, 7, "Port A pin 7");
    gpio_struct!(A8, GPIOA_BASE, 8, "Port A pin 8");
    gpio_struct!(A9, GPIOA_BASE, 9, "Port A pin 9");
    gpio_struct!(A10, GPIOA_BASE, 10, "Port A pin 10");
    gpio_struct!(A11, GPIOA_BASE, 11, "Port A pin 11");
    gpio_struct!(A12, GPIOA_BASE, 12, "Port A pin 12");
    gpio_struct!(A13, GPIOA_BASE, 13, "Port A pin 13");
    gpio_struct!(A14, GPIOA_BASE, 14, "Port A pin 14");
    gpio_struct!(A15, GPIOA_BASE, 15, "Port A pin 15");

    gpio_struct!(B0, GPIOB_BASE, 0, "Port B pin 0");
    gpio_struct!(B1, GPIOB_BASE, 1, "Port B pin 1");
    gpio_struct!(B2, GPIOB_BASE, 2, "Port B pin 2");
    gpio_struct!(B3, GPIOB_BASE, 3, "Port B pin 3");
    gpio_struct!(B4, GPIOB_BASE, 4, "Port B pin 4");
    gpio_struct!(B5, GPIOB_BASE, 5, "Port B pin 5");
    gpio_struct!(B6, GPIOB_BASE, 6, "Port B pin 6");
    gpio_struct!(B7, GPIOB_BASE, 7, "Port B pin 7");
    gpio_struct!(B8, GPIOB_BASE, 8, "Port B pin 8");
    gpio_struct!(B9, GPIOB_BASE, 9, "Port B pin 9");
    gpio_struct!(B10, GPIOB_BASE, 10, "Port B pin 10");
    gpio_struct!(B11, GPIOB_BASE, 11, "Port B pin 11");
    gpio_struct!(B12, GPIOB_BASE, 12, "Port B pin 12");
    gpio_struct!(B13, GPIOB_BASE, 13, "Port B pin 13");
    gpio_struct!(B14, GPIOB_BASE, 14, "Port B pin 14");
    gpio_struct!(B15, GPIOB_BASE, 15, "Port B pin 15");

    gpio_struct!(C0, GPIOC_BASE, 0, "Port C pin 0");
    gpio_struct!(C1, GPIOC_BASE, 1, "Port C pin 1");
    gpio_struct!(C2, GPIOC_BASE, 2, "Port C pin 2");
    gpio_struct!(C3, GPIOC_BASE, 3, "Port C pin 3");
    gpio_struct!(C4, GPIOC_BASE, 4, "Port C pin 4");
    gpio_struct!(C5, GPIOC_BASE, 5, "Port C pin 5");
    gpio_struct!(C6, GPIOC_BASE, 6, "Port C pin 6");
    gpio_struct!(C13, GPIOC_BASE, 13, "Port C pin 13");
    gpio_struct!(C14, GPIOC_BASE, 14, "Port C pin 14");
    gpio_struct!(C15, GPIOC_BASE, 15, "Port C pin 15");

    macro_rules! impl_af {
        ($trt:ident, $gpio:ident, $method:ident, $num:expr) => {
            impl super::sealed::$trt for $gpio {
                #[inline(always)]
                fn $method(&mut self, cs: &CriticalSection) {
                    unsafe { self.pin.set_alternate_function(cs, $num) }
                }
            }
        };
    }

    impl_af!(LpTim1Out, A4, set_lptim1_out_af, 1);
    impl_af!(LpTim1Out, A14, set_lptim1_out_af, 1);
    impl_af!(LpTim1Out, B2, set_lptim1_out_af, 1);
    impl_af!(LpTim1In1, B5, set_lptim1_in1_af, 1);
    impl_af!(LpTim1Etr, B6, set_lptim1_etr_af, 1);
    impl_af!(LpTim1In2, B7, set_lptim1_in2_af, 1);
    impl_af!(LpTim1In1, C0, set_lptim1_in1_af, 1);
    impl_af!(LpTim1Out, C1, set_lptim1_out_af, 1);
    impl_af!(LpTim1In2, C2, set_lptim1_in2_af, 1);
    impl_af!(LpTim1Etr, C3, set_lptim1_etr_af, 1);

    impl_af!(LpTim3Out, A1, set_lptim3_out_af, 3);
    impl_af!(Spi2Miso, A5, set_spi2_miso_af, 3);
    impl_af!(Spi2Nss, A9, set_spi2_nss_af, 3);
    impl_af!(LpTim3Etr, A11, set_lptim3_etr_af, 3);
    impl_af!(LpTim3In1, A12, set_lptim3_in1_af, 3);
    impl_af!(Spi2Mosi, C1, set_spi2_mosi_af, 3);

    impl_af!(I2c3Smba, A0, set_i2c3_smba_af, 4);
    impl_af!(I2c1Smba, A1, set_i2c1_smba_af, 4);
    impl_af!(I2c2Smba, A6, set_i2c2_smba_af, 4);
    impl_af!(I2c3Scl, A7, set_i2c3_scl_af, 4);
    impl_af!(I2c1Scl, A9, set_i2c1_scl_af, 4);
    impl_af!(I2c1Sda, A10, set_i2c1_sda_af, 4);
    impl_af!(I2c2Sda, A11, set_i2c2_sda_af, 4);
    impl_af!(I2c2Scl, A12, set_i2c2_scl_af, 4);
    impl_af!(I2c2Smba, A13, set_i2c2_smba_af, 4);
    impl_af!(I2c1Smba, A13, set_i2c1_smba_af, 4);
    impl_af!(I2c2Sda, A15, set_i2c2_sda_af, 4);
    impl_af!(I2c3Smba, B2, set_i2c3_smba_af, 4);
    impl_af!(I2c3Sda, B4, set_i2c3_sda_af, 4);
    impl_af!(I2c1Smba, B5, set_i2c1_smba_af, 4);
    impl_af!(I2c1Scl, B6, set_i2c1_scl_af, 4);
    impl_af!(I2c1Sda, B7, set_i2c1_sda_af, 4);
    impl_af!(I2c1Scl, B8, set_i2c1_scl_af, 4);
    impl_af!(I2c1Sda, B9, set_i2c1_sda_af, 4);
    impl_af!(I2c3Scl, B10, set_i2c3_scl_af, 4);
    impl_af!(I2c3Sda, B11, set_i2c3_sda_af, 4);
    impl_af!(I2c3Smba, B12, set_i2c3_smba_af, 4);
    impl_af!(I2c3Scl, B13, set_i2c3_scl_af, 4);
    impl_af!(I2c3Sda, B14, set_i2c3_sda_af, 4);
    impl_af!(I2c2Scl, B15, set_i2c2_scl_af, 4);
    impl_af!(I2c3Scl, C0, set_i2c3_scl_af, 4);
    impl_af!(I2c3Sda, C1, set_i2c3_sda_af, 4);

    impl_af!(Spi1Sck, A0, set_spi1_sck_af, 5);
    impl_af!(Spi1Nss, A4, set_spi1_nss_af, 5);
    impl_af!(Spi1Sck, A5, set_spi1_sck_af, 5);
    impl_af!(Spi1Miso, A6, set_spi1_miso_af, 5);
    impl_af!(Spi1Mosi, A7, set_spi1_mosi_af, 5);
    impl_af!(Spi2Sck, A8, set_spi2_sck_af, 5);
    impl_af!(Spi2Sck, A9, set_spi2_sck_af, 5);
    impl_af!(Spi2Mosi, A10, set_spi2_mosi_af, 5);
    impl_af!(Spi1Miso, A11, set_spi1_miso_af, 5);
    impl_af!(Spi1Mosi, A12, set_spi1_mosi_af, 5);
    impl_af!(Spi1Nss, A15, set_spi1_nss_af, 5);
    impl_af!(Spi1Nss, B2, set_spi1_nss_af, 5);
    impl_af!(Spi1Sck, B3, set_spi1_sck_af, 5);
    impl_af!(Spi1Miso, B4, set_spi1_miso_af, 5);
    impl_af!(Spi1Mosi, B5, set_spi1_mosi_af, 5);
    impl_af!(Spi2Nss, B9, set_spi2_nss_af, 5);
    impl_af!(Spi2Sck, B10, set_spi2_sck_af, 5);
    impl_af!(Spi2Nss, B12, set_spi2_nss_af, 5);
    impl_af!(Spi2Sck, B13, set_spi2_sck_af, 5);
    impl_af!(Spi2Miso, B14, set_spi2_miso_af, 5);
    impl_af!(Spi2Mosi, B15, set_spi2_mosi_af, 5);
    impl_af!(Spi2Mosi, C3, set_spi2_mosi_af, 5);
    impl_af!(Spi2Miso, C2, set_spi2_miso_af, 5);

    impl_af!(RfBusy, A12, set_rfbusy_af, 6);
    impl_af!(RfIrq0, B3, set_rf_irq0_af, 6);
    impl_af!(RfIrq1, B5, set_rf_irq1_af, 6);
    impl_af!(RfIrq2, B8, set_rf_irq2_af, 6);

    impl_af!(Uart2Cts, A0, set_uart2_cts_af, 7);
    impl_af!(Uart2Rts, A1, set_uart2_rts_af, 7);
    impl_af!(Uart2Tx, A2, set_uart2_tx_af, 7);
    impl_af!(Uart2Rx, A3, set_uart2_rx_af, 7);
    impl_af!(Uart2Ck, A4, set_uart2_ck_af, 7);
    impl_af!(Uart1Ck, A8, set_uart1_ck_af, 7);
    impl_af!(Uart1Tx, A9, set_uart1_tx_af, 7);
    impl_af!(Uart1Rx, A10, set_uart1_rx_af, 7);
    impl_af!(Uart1Cts, A11, set_uart1_cts_af, 7);
    impl_af!(Uart1Rts, A12, set_uart1_rts_af, 7);
    impl_af!(Uart1Rts, B3, set_uart1_rts_af, 7);
    impl_af!(Uart1Cts, B4, set_uart1_cts_af, 7);
    impl_af!(Uart1Ck, B5, set_uart1_ck_af, 7);
    impl_af!(Uart1Tx, B6, set_uart1_tx_af, 7);
    impl_af!(Uart1Rx, B7, set_uart1_rx_af, 7);

    impl_af!(LpUart1Rts, A1, set_lpuart1_rts_af, 8);
    impl_af!(LpUart1Tx, A2, set_lpuart1_tx_af, 8);
    impl_af!(LpUart1Rx, A3, set_lpuart1_rx_af, 8);
    impl_af!(LpUart1Cts, A6, set_lpuart1_cts_af, 8);
    impl_af!(IrOut, A13, set_irout_af, 8);
    impl_af!(LpUart1RtsDe, B1, set_lpuart1_rts_de_af, 8);
    impl_af!(IrOut, B9, set_irout_af, 8);
    impl_af!(LpUart1Rx, B10, set_lpuart1_rx_af, 8);
    impl_af!(LpUart1Tx, B11, set_lpuart1_tx_af, 8);
    impl_af!(LpUart1Rts, B12, set_lpuart1_rts_af, 8);
    impl_af!(LpUart1Cts, B13, set_lpuart1_cts_af, 8);
    impl_af!(LpUart1Rx, C0, set_lpuart1_rx_af, 8);
    impl_af!(LpUart1Tx, C1, set_lpuart1_tx_af, 8);

    impl_af!(SubGhzSpiNss, A4, set_subghz_spi_nss_af, 13);
    impl_af!(SubGhzSpiSck, A5, set_subghz_spi_sck_af, 13);
    impl_af!(SubGhzSpiMiso, A6, set_subghz_spi_miso_af, 13);
    impl_af!(SubGhzSpiMosi, A7, set_subghz_spi_mosi_af, 13);

    impl_af!(LpTim2Out, A4, set_lptim2_out_af, 14);
    impl_af!(LpTim2Etr, A5, set_lptim2_etr_af, 14);
    impl_af!(LpTim2Out, A8, set_lptim2_out_af, 14);
    impl_af!(LpTim2In1, B1, set_lptim2_in1_af, 14);
    impl_af!(LpTim2In1, C0, set_lptim2_in1_af, 14);
    impl_af!(LpTim2Etr, C3, set_lptim2_etr_af, 14);

    // keep the trait separate from the pin so that users cant use the ADC_CH
    // but are unable to implement the sealed trait themselves
    macro_rules! impl_adc_ch {
        ($pin:ident, $ch:expr) => {
            impl $pin {
                /// Analog to digital converter channel when this pin is
                /// configured as [`Analog`](crate::gpio::Analog).
                pub const ADC_CH: adc::Ch = $ch;
            }

            impl super::sealed::AdcCh for $pin {
                const ADC_CH: adc::Ch = Self::ADC_CH;
            }
        };
    }

    impl_adc_ch!(A10, adc::Ch::In6);
    impl_adc_ch!(A11, adc::Ch::In7);
    impl_adc_ch!(A12, adc::Ch::In8);
    impl_adc_ch!(A13, adc::Ch::In9);
    impl_adc_ch!(A14, adc::Ch::In10);
    impl_adc_ch!(A15, adc::Ch::In11);
    impl_adc_ch!(B1, adc::Ch::In5);
    impl_adc_ch!(B2, adc::Ch::In4);
    impl_adc_ch!(B3, adc::Ch::In2);
    impl_adc_ch!(B4, adc::Ch::In3);
    impl_adc_ch!(B13, adc::Ch::In0);
    impl_adc_ch!(B14, adc::Ch::In1);
}

/// Port A GPIOs
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub struct PortA {
    pub a0: pins::A0,
    pub a1: pins::A1,
    pub a2: pins::A2,
    pub a3: pins::A3,
    pub a4: pins::A4,
    pub a5: pins::A5,
    pub a6: pins::A6,
    pub a7: pins::A7,
    pub a8: pins::A8,
    pub a9: pins::A9,
    pub a10: pins::A10,
    pub a11: pins::A11,
    pub a12: pins::A12,
    pub a13: pins::A13,
    pub a14: pins::A14,
    pub a15: pins::A15,
}

impl PortA {
    const GPIOS: PortA = PortA {
        a0: pins::A0::new(),
        a1: pins::A1::new(),
        a2: pins::A2::new(),
        a3: pins::A3::new(),
        a4: pins::A4::new(),
        a5: pins::A5::new(),
        a6: pins::A6::new(),
        a7: pins::A7::new(),
        a8: pins::A8::new(),
        a9: pins::A9::new(),
        a10: pins::A10::new(),
        a11: pins::A11::new(),
        a12: pins::A12::new(),
        a13: pins::A13::new(),
        a14: pins::A14::new(),
        a15: pins::A15::new(),
    };

    /// Reset GPIO port A and split the port into individual pins.
    ///
    /// This will enable clocks and reset the GPIO port.
    ///
    /// # Example
    ///
    /// Get GPIO A0.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortA},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let a0: pins::A0 = gpioa.a0;
    /// ```
    #[allow(unused_variables)]
    pub fn split(gpioa: pac::GPIOA, rcc: &mut pac::RCC) -> Self {
        Self::enable_clock(rcc);
        rcc.ahb2rstr.modify(|_, w| w.gpioarst().set_bit());
        rcc.ahb2rstr.modify(|_, w| w.gpioarst().clear_bit());

        Self::GPIOS
    }

    /// Steal the port A GPIOs from whatever is currently using them.
    ///
    /// This will **not** initialize the GPIOs (unlike [`split`]).
    ///
    /// # Safety
    ///
    /// This will create new GPIOs, bypassing the singleton checks that normally
    /// occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// the GPIOs.
    /// You are also responsible for ensuring the GPIO peripheral has been
    /// setup correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::PortA;
    ///
    /// // ... setup happens here
    ///
    /// let gpioa: PortA = unsafe { PortA::steal() };
    /// ```
    ///
    /// [`split`]: crate::gpio::PortA::split
    pub unsafe fn steal() -> Self {
        Self::GPIOS
    }

    /// Disable the GPIOA clock.
    ///
    /// # Safety
    ///
    /// 1. You cannot use any port-A GPIO pin while the clock is disabled.
    /// 2. You are responsible for re-enabling the clock before resuming use
    ///    of any port A GPIO.
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpioaen().disabled());
    }

    /// Enable the GPIOA clock.
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpioaen().enabled());
        rcc.ahb2enr.read(); // delay after an RCC peripheral clock enabling
    }
}

/// Port B GPIOs
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub struct PortB {
    pub b0: pins::B0,
    pub b1: pins::B1,
    pub b2: pins::B2,
    pub b3: pins::B3,
    pub b4: pins::B4,
    pub b5: pins::B5,
    pub b6: pins::B6,
    pub b7: pins::B7,
    pub b8: pins::B8,
    pub b9: pins::B9,
    pub b10: pins::B10,
    pub b11: pins::B11,
    pub b12: pins::B12,
    pub b13: pins::B13,
    pub b14: pins::B14,
    pub b15: pins::B15,
}

impl PortB {
    const GPIOS: PortB = PortB {
        b0: pins::B0::new(),
        b1: pins::B1::new(),
        b2: pins::B2::new(),
        b3: pins::B3::new(),
        b4: pins::B4::new(),
        b5: pins::B5::new(),
        b6: pins::B6::new(),
        b7: pins::B7::new(),
        b8: pins::B8::new(),
        b9: pins::B9::new(),
        b10: pins::B10::new(),
        b11: pins::B11::new(),
        b12: pins::B12::new(),
        b13: pins::B13::new(),
        b14: pins::B14::new(),
        b15: pins::B15::new(),
    };

    /// Reset GPIO port B and split the port into individual pins.
    ///
    /// This will enable clocks and reset the GPIO port.
    ///
    /// # Example
    ///
    /// Get GPIO B0.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortB},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let b0: pins::B0 = gpiob.b0;
    /// ```
    #[allow(unused_variables)]
    pub fn split(gpiob: pac::GPIOB, rcc: &mut pac::RCC) -> Self {
        Self::enable_clock(rcc);
        rcc.ahb2rstr.modify(|_, w| w.gpiobrst().set_bit());
        rcc.ahb2rstr.modify(|_, w| w.gpiobrst().clear_bit());

        Self::GPIOS
    }

    /// Steal the port B GPIOs from whatever is currently using them.
    ///
    /// This will **not** initialize the GPIOs (unlike [`split`]).
    ///
    /// # Safety
    ///
    /// This will create new GPIOs, bypassing the singleton checks that normally
    /// occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// the GPIOs.
    /// You are also responsible for ensuring the GPIO peripheral has been
    /// setup correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::PortB;
    ///
    /// // ... setup happens here
    ///
    /// let gpioa: PortB = unsafe { PortB::steal() };
    /// ```
    ///
    /// [`split`]: crate::gpio::PortB::split
    pub unsafe fn steal() -> Self {
        Self::GPIOS
    }

    /// Disable the GPIOB clock.
    ///
    /// # Safety
    ///
    /// 1. You cannot use any port-B GPIO pin while the clock is disabled.
    /// 2. You are responsible for re-enabling the clock before resuming use
    ///    of any port B GPIO.
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpioben().disabled());
    }

    /// Enable the GPIOB clock.
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpioben().enabled());
        rcc.ahb2enr.read(); // delay after an RCC peripheral clock enabling
    }
}

/// Port C GPIOs
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub struct PortC {
    pub c0: pins::C0,
    pub c1: pins::C1,
    pub c2: pins::C2,
    pub c3: pins::C3,
    pub c4: pins::C4,
    pub c5: pins::C5,
    pub c6: pins::C6,
    pub c13: pins::C13,
    pub c14: pins::C14,
    pub c15: pins::C15,
}

impl PortC {
    const GPIOS: PortC = PortC {
        c0: pins::C0::new(),
        c1: pins::C1::new(),
        c2: pins::C2::new(),
        c3: pins::C3::new(),
        c4: pins::C4::new(),
        c5: pins::C5::new(),
        c6: pins::C6::new(),
        c13: pins::C13::new(),
        c14: pins::C14::new(),
        c15: pins::C15::new(),
    };

    /// Reset GPIO port C and split the port into individual pins.
    ///
    /// This will enable clocks and reset the GPIO port.
    ///
    /// # Example
    ///
    /// Get GPIO C0.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let c0: pins::C0 = gpioc.c0;
    /// ```
    #[allow(unused_variables)]
    pub fn split(gpioc: pac::GPIOC, rcc: &mut pac::RCC) -> Self {
        Self::enable_clock(rcc);
        rcc.ahb2rstr.modify(|_, w| w.gpiocrst().set_bit());
        rcc.ahb2rstr.modify(|_, w| w.gpiocrst().clear_bit());

        Self::GPIOS
    }

    /// Steal the port C GPIOs from whatever is currently using them.
    ///
    /// This will **not** initialize the GPIOs (unlike [`split`]).
    ///
    /// # Safety
    ///
    /// This will create new GPIOs, bypassing the singleton checks that normally
    /// occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// the GPIOs.
    /// You are also responsible for ensuring the GPIO peripheral has been
    /// setup correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::PortC;
    ///
    /// // ... setup happens here
    ///
    /// let gpioa: PortC = unsafe { PortC::steal() };
    /// ```
    ///
    /// [`split`]: crate::gpio::PortC::split
    pub unsafe fn steal() -> Self {
        Self::GPIOS
    }

    /// Disable the GPIOC clock.
    ///
    /// # Safety
    ///
    /// 1. You cannot use any port-C GPIO pin while the clock is disabled.
    /// 2. You are responsible for re-enabling the clock before resuming use
    ///    of any port C GPIO.
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpiocen().disabled());
    }

    /// Enable the GPIOC clock.
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb2enr.modify(|_, w| w.gpiocen().enabled());
        rcc.ahb2enr.read(); // delay after an RCC peripheral clock enabling
    }
}

/// Digital input or output level.
#[derive(Debug, Eq, PartialEq, PartialOrd, Ord, Clone, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    /// GPIO logic low.
    Low,
    /// GPIO logic high.
    High,
}

impl Level {
    /// Toggle the level.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::Level;
    ///
    /// assert_eq!(Level::High.toggle(), Level::Low);
    /// assert_eq!(Level::Low.toggle(), Level::High);
    /// ```
    pub const fn toggle(self) -> Level {
        match self {
            Level::Low => Level::High,
            Level::High => Level::Low,
        }
    }

    /// Returns `true` if the level is low.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::Level;
    ///
    /// assert_eq!(Level::Low.is_low(), true);
    /// assert_eq!(Level::High.is_low(), false);
    /// ```
    pub fn is_low(&self) -> bool {
        matches!(self, Self::Low)
    }

    /// Returns `true` if the level is high.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::Level;
    ///
    /// assert_eq!(Level::High.is_high(), true);
    /// assert_eq!(Level::Low.is_high(), false);
    /// ```
    pub fn is_high(&self) -> bool {
        matches!(self, Self::High)
    }
}

/// Output pin arguments.
///
/// Argument of [`Output::new`].
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OutputArgs {
    /// Output speed.
    pub speed: Speed,
    /// Initial output level.
    pub level: Level,
    /// Output type.
    pub ot: OutputType,
    /// IO pull configuration.
    ///
    /// This is only used if the output type is [`OutputType::OpenDrain`].
    pub pull: Pull,
}

impl OutputArgs {
    /// Create a new `OutputArgs` struct.
    ///
    /// This is the same as `default`, but in a `const` fn.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::OutputArgs;
    ///
    /// assert_eq!(OutputArgs::new(), OutputArgs::default());
    /// ```
    pub const fn new() -> Self {
        OutputArgs {
            speed: Speed::High,
            level: Level::Low,
            ot: OutputType::PushPull,
            pull: Pull::None,
        }
    }
}

impl Default for OutputArgs {
    fn default() -> Self {
        Self::new()
    }
}

/// Output pin.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Output<P> {
    pin: P,
}

impl<P> Output<P>
where
    P: sealed::PinOps,
{
    /// Create a new output pin from a GPIO.
    ///
    /// # Example
    ///
    /// Configure GPIO port C3, C4, C5 as outputs.
    /// These are the GPIOs for the RF switch on the NUCLEO-WL55JC2.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{self, pins, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// const OUTPUT_ARGS: gpio::OutputArgs = gpio::OutputArgs {
    ///     level: gpio::Level::Low,
    ///     speed: gpio::Speed::High,
    ///     ot: gpio::OutputType::PushPull,
    ///     pull: gpio::Pull::None,
    /// };
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut c3: Output<pins::C3> = Output::new(gpioc.c3, &OUTPUT_ARGS);
    /// let mut c4: Output<pins::C4> = Output::new(gpioc.c4, &OUTPUT_ARGS);
    /// let mut c5: Output<pins::C5> = Output::new(gpioc.c5, &OUTPUT_ARGS);
    /// ```
    pub fn new(mut pin: P, args: &OutputArgs) -> Self {
        cortex_m::interrupt::free(|cs| unsafe {
            pin.set_output_type(cs, args.ot);
            if args.ot == OutputType::OpenDrain {
                pin.set_pull(cs, args.pull)
            } else {
                pin.set_pull(cs, Pull::None)
            }
            pin.set_speed(cs, args.speed);
            pin.set_output_level(args.level);
            pin.set_mode(cs, sealed::Mode::Output);
        });
        Output { pin }
    }

    /// Create a new output pin from a GPIO using the default settings.
    ///
    /// # Example
    ///
    /// Configure GPIO C0 as an output.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Output, OutputArgs, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut c0: Output<pins::C0> = Output::default(gpioc.c0);
    /// ```
    pub fn default(pin: P) -> Self {
        Self::new(pin, &OutputArgs::new())
    }

    /// Steal the output GPIO from whatever is currently using it.
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the GPIO has exclusive access to the
    ///    peripheral. Singleton checks are bypassed with this method.
    /// 2. You are responsible for setting up the GPIO correctly.
    ///    No setup will occur when using this method.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::{pins, Output};
    ///
    /// // ... setup occurs here
    ///
    /// let b11: Output<pins::B11> = unsafe { Output::steal() };
    /// ```
    pub unsafe fn steal() -> Self {
        Output { pin: P::steal() }
    }

    /// Free the GPIO pin.
    ///
    /// # Example
    ///
    /// Configure a GPIO as an output, then free it.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let c0: Output<pins::C0> = Output::default(gpioc.c0);
    /// let c0: pins::C0 = c0.free();
    /// ```
    pub fn free(self) -> P {
        self.pin
    }

    /// Set the GPIO output level.
    ///
    /// This is the same as the `OutputPin` trait from the embedded hal, but
    /// without the `Infallible` result types.
    ///
    /// # Example
    ///
    /// Pulse a GPIO pin.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Level, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut c0: Output<pins::C0> = Output::default(gpioc.c0);
    /// c0.set_level(Level::High);
    /// c0.set_level(Level::Low);
    /// ```
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_output_level(level)
    }

    /// Set the GPIO output level high.
    ///
    /// This is the same as the `OutputPin` trait from the embedded hal, but
    /// without the `Infallible` result types.
    ///
    /// # Example
    ///
    /// Set GPIO C0 high.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut c0: Output<pins::C0> = Output::default(gpioc.c0);
    /// c0.set_level_high();
    /// ```
    pub fn set_level_high(&mut self) {
        self.set_level(Level::High)
    }

    /// Set the GPIO output level high.
    ///
    /// This is the same as the `OutputPin` trait from the embedded hal, but
    /// without the `Infallible` result types.
    ///
    /// # Example
    ///
    /// Set GPIO C0 low.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut c0: Output<pins::C0> = Output::default(gpioc.c0);
    /// c0.set_level_low();
    /// ```
    pub fn set_level_low(&mut self) {
        self.set_level(Level::Low)
    }

    /// Get the current GPIO output level.
    ///
    /// # Example
    ///
    /// Toggle a GPIO pin.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Level, Output, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut c0: Output<pins::C0> = Output::default(gpioc.c0);
    /// c0.set_level(c0.level().toggle());
    /// ```
    pub fn level(&self) -> Level {
        self.pin.output_level()
    }
}

impl<P> embedded_hal::digital::v2::OutputPin for Output<P>
where
    P: sealed::PinOps,
{
    type Error = core::convert::Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_output_level(Level::Low);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_output_level(Level::High);
        Ok(())
    }
}

impl<P> embedded_hal::digital::v2::StatefulOutputPin for Output<P>
where
    P: sealed::PinOps,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.pin.output_level().is_high())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.pin.output_level().is_low())
    }
}

impl<P: sealed::PinOps> embedded_hal::digital::v2::toggleable::Default for Output<P> {}

/// Input pin
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Input<P> {
    pin: P,
}

impl<P> Input<P>
where
    P: sealed::PinOps,
{
    /// Create a new input pin from a GPIO.
    ///
    /// # Example
    ///
    /// Configure GPIO C6 as an input.
    /// This is the GPIO for button 3 on the NUCLEO-WL55JC2.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Input, PortC, Pull},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut c6: Input<pins::C6> = Input::new(gpioc.c6, Pull::Up);
    /// ```
    pub fn new(mut pin: P, pull: Pull) -> Self {
        cortex_m::interrupt::free(|cs| unsafe {
            pin.set_pull(cs, pull);
            pin.set_output_type(cs, OutputType::PushPull);
            pin.set_mode(cs, sealed::Mode::Input);
        });
        Input { pin }
    }

    /// Create a new input pin from a GPIO with default settings.
    ///
    /// # Example
    ///
    /// Configure GPIO C0 as an input.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Input, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut c0: Input<pins::C0> = Input::default(gpioc.c0);
    /// ```
    pub fn default(pin: P) -> Self {
        Self::new(pin, Pull::None)
    }

    /// Steal the input GPIO from whatever is currently using it.
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the GPIO has exclusive access to the
    ///    peripheral. Singleton checks are bypassed with this method.
    /// 2. You are responsible for setting up the GPIO correctly.
    ///    No setup will occur when using this method.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::gpio::{pins, Input};
    ///
    /// // ... setup occurs here
    ///
    /// let c6: Input<pins::C6> = unsafe { Input::steal() };
    /// ```
    pub unsafe fn steal() -> Self {
        Input { pin: P::steal() }
    }

    /// Free the GPIO pin.
    ///
    /// # Example
    ///
    /// Configure a GPIO as an input, then free it.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Input, PortC},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let c0_input: Input<pins::C0> = Input::default(gpioc.c0);
    /// let c0: pins::C0 = c0_input.free();
    /// ```
    pub fn free(self) -> P {
        self.pin
    }

    /// Get the input level.
    ///
    /// # Example
    ///
    /// Get the input level of C6.
    /// This is the GPIO for button 3 on the NUCLEO-WL55JC2.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Input, Level, PortC, Pull},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    /// let mut c6: Input<pins::C6> = Input::new(gpioc.c6, Pull::Up);
    ///
    /// let button_3_is_pressed: bool = c6.level() == Level::High;
    /// ```
    pub fn level(&self) -> Level {
        self.pin.input_level()
    }
}

/// Analog pin
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Analog<P> {
    pin: P,
}

impl<P> Analog<P>
where
    P: sealed::PinOps + sealed::AdcCh,
{
    /// Analog to digital converter channel.
    pub const ADC_CH: adc::Ch = P::ADC_CH;

    /// Create a new analog pin from a GPIO.
    ///
    /// # Example
    ///
    /// Configure GPIO B14 as an analog pin (ADC_IN1).
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Analog, PortB},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let mut b14: Analog<pins::B14> = Analog::new(gpiob.b14);
    /// ```
    pub fn new(mut pin: P) -> Self {
        cortex_m::interrupt::free(|cs| unsafe {
            pin.set_mode(cs, sealed::Mode::Analog);
        });
        Analog { pin }
    }

    /// Free the GPIO pin.
    ///
    /// # Example
    ///
    /// Configure a GPIO as an analog pin, then free it.
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, Analog, PortB},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let b14: Analog<pins::B14> = Analog::new(gpiob.b14);
    /// let b14: pins::B14 = b14.free();
    /// ```
    pub fn free(self) -> P {
        self.pin
    }
}

impl<P> From<P> for Analog<P>
where
    P: sealed::PinOps + sealed::AdcCh,
{
    fn from(p: P) -> Self {
        Analog::new(p)
    }
}

/// RF Busy pin
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RfBusy {
    pin: pins::A12,
}

impl RfBusy {
    /// Create a new RF Busy pin from a pin A12.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{PortA, RfBusy},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let a12: RfBusy = RfBusy::new(gpioa.a12);
    /// ```
    pub fn new(mut pin: pins::A12) -> Self {
        use sealed::RfBusy;
        cortex_m::interrupt::free(|cs| pin.set_rfbusy_af(cs));
        Self { pin }
    }

    /// Free the GPIO pin.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortA, RfBusy},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let a12: RfBusy = RfBusy::new(gpioa.a12);
    /// let a12: pins::A12 = a12.free();
    /// ```
    pub fn free(self) -> pins::A12 {
        self.pin
    }
}

/// RF IRQ 0 pin
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RfIrq0 {
    pin: pins::B3,
}

impl RfIrq0 {
    /// Create a new RF IRQ 0 pin from a pin B3.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{PortB, RfIrq0},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let b3: RfIrq0 = RfIrq0::new(gpiob.b3);
    /// ```
    pub fn new(mut pin: pins::B3) -> Self {
        use sealed::RfIrq0;
        cortex_m::interrupt::free(|cs| pin.set_rf_irq0_af(cs));
        Self { pin }
    }

    /// Free the GPIO pin.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortB, RfIrq0},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let b3: RfIrq0 = RfIrq0::new(gpiob.b3);
    /// let b3: pins::B3 = b3.free();
    /// ```
    pub fn free(self) -> pins::B3 {
        self.pin
    }
}

/// RF IRQ 1 pin
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RfIrq1 {
    pin: pins::B5,
}

impl RfIrq1 {
    /// Create a new RF IRQ 1 pin from a pin B5.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{PortB, RfIrq1},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let b5: RfIrq1 = RfIrq1::new(gpiob.b5);
    /// ```
    pub fn new(mut pin: pins::B5) -> Self {
        use sealed::RfIrq1;
        cortex_m::interrupt::free(|cs| pin.set_rf_irq1_af(cs));
        Self { pin }
    }

    /// Free the GPIO pin.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortB, RfIrq1},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let b5: RfIrq1 = RfIrq1::new(gpiob.b5);
    /// let b5: pins::B5 = b5.free();
    /// ```
    pub fn free(self) -> pins::B5 {
        self.pin
    }
}

/// RF IRQ 2 pin
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RfIrq2 {
    pin: pins::B8,
}

impl RfIrq2 {
    /// Create a new RF IRQ 2 pin from a pin B8.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{PortB, RfIrq2},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let b8: RfIrq2 = RfIrq2::new(gpiob.b8);
    /// ```
    pub fn new(mut pin: pins::B8) -> Self {
        use sealed::RfIrq2;
        cortex_m::interrupt::free(|cs| pin.set_rf_irq2_af(cs));
        Self { pin }
    }

    /// Free the GPIO pin.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, PortB, RfIrq2},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let b8: RfIrq2 = RfIrq2::new(gpiob.b8);
    /// let b8: pins::B8 = b8.free();
    /// ```
    pub fn free(self) -> pins::B8 {
        self.pin
    }
}

/// Low-power timer 1 trigger pin.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LpTim1Trg<P> {
    pin: P,
}

impl<P: sealed::LpTim1Etr> LpTim1Trg<P> {
    /// Create a new low-power timer 3 trigger from a pin.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, LpTim1Trg, PortB},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let b6: LpTim1Trg<pins::B6> = LpTim1Trg::new(gpiob.b6);
    /// ```
    pub fn new(mut pin: P) -> Self {
        cortex_m::interrupt::free(|cs| pin.set_lptim1_etr_af(cs));
        Self { pin }
    }

    /// Free the GPIO pin.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, LpTim1Trg, PortB},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    /// let b6: LpTim1Trg<pins::B6> = LpTim1Trg::new(gpiob.b6);
    /// let b6: pins::B6 = b6.free();
    /// ```
    pub fn free(self) -> P {
        self.pin
    }
}

/// Low-power timer 2 trigger pin.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LpTim2Trg<P> {
    pin: P,
}

impl<P: sealed::LpTim2Etr> LpTim2Trg<P> {
    /// Create a new low-power timer 3 trigger from a pin.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, LpTim2Trg, PortA},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let a5: LpTim2Trg<pins::A5> = LpTim2Trg::new(gpioa.a5);
    /// ```
    pub fn new(mut pin: P) -> Self {
        cortex_m::interrupt::free(|cs| pin.set_lptim2_etr_af(cs));
        Self { pin }
    }

    /// Free the GPIO pin.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, LpTim2Trg, PortA},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let a5: LpTim2Trg<pins::A5> = LpTim2Trg::new(gpioa.a5);
    /// let a5: pins::A5 = a5.free();
    /// ```
    pub fn free(self) -> P {
        self.pin
    }
}

/// Low-power timer 3 trigger pin.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LpTim3Trg {
    pin: pins::A11,
}

impl LpTim3Trg {
    /// Create a new low-power timer 3 trigger from pin A11.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, LpTim3Trg, PortA},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let a11: LpTim3Trg = LpTim3Trg::new(gpioa.a11);
    /// ```
    pub fn new(mut pin: pins::A11) -> Self {
        use sealed::LpTim3Etr;
        cortex_m::interrupt::free(|cs| pin.set_lptim3_etr_af(cs));
        Self { pin }
    }

    /// Free the GPIO pin.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     gpio::{pins, LpTim3Trg, PortA},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let a11: LpTim3Trg = LpTim3Trg::new(gpioa.a11);
    /// let a11: pins::A11 = a11.free();
    /// ```
    pub fn free(self) -> pins::A11 {
        self.pin
    }
}
