// Most of the coverage for SPI comes from the Sub-GHz testsuite.

#![no_std]
#![no_main]

use core::ptr::{read_volatile, write_volatile};
use defmt::unwrap;
use defmt_rtt as _; // global logger
use itertools::iproduct;
use nucleo_wl55jc_bsp::hal::{
    cortex_m::{self, interrupt::CriticalSection},
    dma::AllDma,
    embedded_hal::blocking::spi::{Transfer, Write},
    gpio::{PortA, PortC},
    pac::{self, DWT},
    rcc,
    spi::{
        BaudRate, Mode, NoMiso, NoMosi, NoSck, Phase, Polarity, Spi, MODE_0, MODE_1, MODE_2, MODE_3,
    },
};
use panic_probe as _;

const FREQ: u32 = 48_000_000;
const CYC_PER_MICRO: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:us}", DWT::cycle_count() / CYC_PER_MICRO);

pub struct TestArgs {
    dma: AllDma,
    pa: PortA,
    spi1: pac::SPI1,
    spi2: pac::SPI2,
    rcc: pac::RCC,
}

struct SpiSlave {
    pub spi: pac::SPI2,
}

impl SpiSlave {
    fn new(
        spi: pac::SPI2,
        mode: Mode,
        rxonly: bool,
        rcc: &mut pac::RCC,
        _cs: &CriticalSection,
    ) -> Self {
        Spi::<pac::SPI2, NoSck, NoMiso, NoMosi>::enable_clock(rcc);
        unsafe { Spi::<pac::SPI2, NoSck, NoMiso, NoMosi>::pulse_reset(rcc) };

        // setup GPIOs
        let dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
        dp.GPIOA.moder.modify(|_, w| w.moder9().alternate());
        dp.GPIOA.afrh.modify(|_, w| w.afrh9().bits(5));
        dp.GPIOC
            .moder
            .modify(|_, w| w.moder2().alternate().moder3().alternate());
        dp.GPIOC
            .afrl
            .modify(|_, w| w.afrl2().bits(5).afrl3().bits(5));

        spi.cr1.write(|w| {
            w.rxonly().bit(rxonly);
            w.ssi().set_bit();
            w.ssm().set_bit();
            w.spe().set_bit();
            w.cpol().bit(mode.polarity == Polarity::IdleHigh);
            w.cpha().bit(mode.phase == Phase::CaptureOnSecondTransition)
        });

        spi.cr2.write(|w| w.frxth().quarter());

        SpiSlave { spi }
    }

    fn set_ssi(&mut self, ssi: bool) {
        self.spi.cr1.modify(|_, w| w.ssi().bit(ssi))
    }

    fn read_word(&mut self) -> u8 {
        loop {
            if !self.spi.sr.read().frlvl().is_empty() {
                return unsafe { read_volatile(self.spi.dr.as_ptr() as *const u8) };
            }
        }
    }

    fn write_word(&mut self, word: u8) {
        loop {
            if !self.spi.sr.read().ftlvl().is_full() {
                unsafe { write_volatile(self.spi.dr.as_ptr() as *mut u8, word) };
                return;
            }
        }
    }

    fn read(&mut self, words: &mut [u8]) {
        words.iter_mut().for_each(|word| *word = self.read_word())
    }

    fn write(&mut self, words: &[u8]) {
        words.iter().for_each(|word| self.write_word(*word))
    }
}

unsafe fn setup() -> TestArgs {
    let mut dp: pac::Peripherals = pac::Peripherals::steal();
    let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    let _: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);

    TestArgs {
        dma,
        pa,
        spi1: dp.SPI1,
        spi2: dp.SPI2,
        rcc: dp.RCC,
    }
}

const BAUD_RATES: [BaudRate; 7] = [
    BaudRate::Div256,
    BaudRate::Div128,
    BaudRate::Div64,
    BaudRate::Div32,
    BaudRate::Div16,
    BaudRate::Div8,
    BaudRate::Div4,
    // BaudRate::Div2, // has signal integrity issues
];

const SPI_MODES: [Mode; 4] = [MODE_0, MODE_1, MODE_2, MODE_3];

fn mode_num(m: Mode) -> u8 {
    match m {
        MODE_0 => 0,
        MODE_1 => 1,
        MODE_2 => 2,
        MODE_3 => 3,
    }
}

const DATA: &[u8] = b"hey";

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() {
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());

        cortex_m::interrupt::free(|cs| unsafe {
            rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs)
        });
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        cp.DWT.set_cycle_count(0);
        defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);

        defmt::warn!(
            "SPI tests require (SCK, MISO, MOSI) pins SPI1 (A5, A6, A7) \
            connected to SPI2 (A9, C2, C3)"
        );
    }

    #[test]
    fn full_duplex_loopback() {
        for (&br, &mode) in iproduct!(BAUD_RATES.iter(), SPI_MODES.iter()) {
            defmt::debug!("÷{} MODE_{}", br.div(), mode_num(mode));
            let mut ta: TestArgs = unsafe { setup() };

            let mut s = cortex_m::interrupt::free(|cs| {
                SpiSlave::new(ta.spi2, mode, false, &mut ta.rcc, cs)
            });

            let mut m = cortex_m::interrupt::free(|cs| {
                Spi::new_spi1_full_duplex(
                    ta.spi1,
                    (ta.pa.a5, ta.pa.a6, ta.pa.a7),
                    mode,
                    br,
                    &mut ta.rcc,
                    cs,
                )
            });

            for _ in 0..8 {
                cortex_m::asm::delay(21);
                s.set_ssi(false);
                unwrap!(m.write(DATA));
                let mut buf: [u8; 3] = [0; 3];
                s.read(&mut buf);
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);
            }

            for _ in 0..8 {
                cortex_m::asm::delay(21);
                s.set_ssi(false);
                s.write(DATA);
                let mut buf: [u8; 3] = [0x12, 0x34, 0x56];
                unwrap!(m.transfer(&mut buf));
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);

                let mut slave_buf: [u8; 3] = [0; 3];
                s.read(&mut slave_buf);
                defmt::assert_eq!(slave_buf, [0x12, 0x34, 0x56]);
            }
        }
    }

    #[test]
    fn full_duplex_dma_loopback() {
        for (&br, &mode) in iproduct!(BAUD_RATES.iter(), SPI_MODES.iter()) {
            defmt::debug!("÷{} MODE_{}", br.div(), mode_num(mode));
            let mut ta: TestArgs = unsafe { setup() };

            let mut s = cortex_m::interrupt::free(|cs| {
                SpiSlave::new(ta.spi2, mode, false, &mut ta.rcc, cs)
            });

            let mut m = cortex_m::interrupt::free(|cs| {
                Spi::new_spi1_full_duplex_dma(
                    ta.spi1,
                    (ta.pa.a5, ta.pa.a6, ta.pa.a7),
                    (ta.dma.d2.c1, ta.dma.d2.c2),
                    mode,
                    br,
                    &mut ta.rcc,
                    cs,
                )
            });

            for _ in 0..8 {
                s.set_ssi(false);
                unwrap!(m.write(DATA));
                let mut buf: [u8; 3] = [0; 3];
                s.read(&mut buf);
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);
            }

            for _ in 0..8 {
                s.set_ssi(false);
                s.write(DATA);
                let mut buf: [u8; 3] = [0x12, 0x34, 0x56];
                unwrap!(m.transfer(&mut buf));
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);

                let mut slave_buf: [u8; 3] = [0; 3];
                s.read(&mut slave_buf);
                defmt::assert_eq!(slave_buf, [0x12, 0x34, 0x56]);
            }
        }
    }

    #[test]
    fn mosi_simplex_loopback() {
        for (&br, &mode) in iproduct!(BAUD_RATES.iter(), SPI_MODES.iter()) {
            defmt::debug!("÷{} MODE_{}", br.div(), mode_num(mode));
            let mut ta: TestArgs = unsafe { setup() };

            let mut s =
                cortex_m::interrupt::free(|cs| SpiSlave::new(ta.spi2, mode, true, &mut ta.rcc, cs));

            let mut m = cortex_m::interrupt::free(|cs| {
                Spi::new_spi1_mosi_simplex(ta.spi1, (ta.pa.a5, ta.pa.a7), mode, br, &mut ta.rcc, cs)
            });

            for _ in 0..8 {
                s.set_ssi(false);
                unwrap!(m.write(DATA));
                cortex_m::asm::delay(u32::from(br.div()) * 16);
                let mut buf: [u8; 3] = [0; 3];
                s.read(&mut buf);
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);
            }
        }
    }

    #[test]
    fn mosi_simplex_dma_loopback() {
        for (&br, &mode) in iproduct!(BAUD_RATES.iter(), SPI_MODES.iter()) {
            defmt::debug!("÷{} MODE_{}", br.div(), mode_num(mode));
            let mut ta: TestArgs = unsafe { setup() };

            let mut s =
                cortex_m::interrupt::free(|cs| SpiSlave::new(ta.spi2, mode, true, &mut ta.rcc, cs));

            let mut m = cortex_m::interrupt::free(|cs| {
                Spi::new_spi1_mosi_simplex_dma(
                    ta.spi1,
                    (ta.pa.a5, ta.pa.a7),
                    ta.dma.d1.c1,
                    mode,
                    br,
                    &mut ta.rcc,
                    cs,
                )
            });

            for _ in 0..8 {
                s.set_ssi(false);
                unwrap!(m.write(DATA));
                cortex_m::asm::delay(u32::from(br.div()) * 16);
                let mut buf: [u8; 3] = [0; 3];
                s.read(&mut buf);
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);
            }
        }
    }
}
