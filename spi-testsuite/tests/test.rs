// Most of the coverage for SPI comes from the Sub-GHz testsuite.

#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use itertools::iproduct;
use panic_probe as _;
use stm32wl_hal::{
    dma::AllDma,
    embedded_hal::blocking::spi::{Transfer, Write},
    gpio::{PortA, PortC},
    pac::{self, DWT},
    rcc,
    spi::{BaudRate, Mode, Read, Spi, MODE_0, MODE_1, MODE_2, MODE_3},
    util::reset_cycle_count,
};

const FREQ: u32 = 48_000_000;
const CYC_PER_MICRO: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:µs}", DWT::get_cycle_count() / CYC_PER_MICRO);

pub struct TestArgs {
    dma: AllDma,
    pa: PortA,
    pc: PortC,
    spi1: pac::SPI1,
    spi2: pac::SPI2,
    rcc: pac::RCC,
}

unsafe fn setup() -> TestArgs {
    let mut dp: pac::Peripherals = pac::Peripherals::steal();
    let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    let pa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    let pc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);

    TestArgs {
        dma,
        pa,
        pc,
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

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC);
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        reset_cycle_count(&mut cp.DWT);
        defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);

        defmt::warn!(
            "SPI tests require (SCK, MISO, MOSI) pins SPI1 (A5, A6, A7) connected to SPI2 (A9, C2, C3)"
        );
    }

    #[test]
    fn full_duplex_loopback() {
        for (&br, &mode) in iproduct!(BAUD_RATES.iter(), SPI_MODES.iter()) {
            defmt::debug!("÷{} MODE_{}", br.div(), mode_num(mode));
            let mut ta: TestArgs = unsafe { setup() };

            let mut s = Spi::new_spi2_full_duplex_slave(
                ta.spi2,
                (ta.pa.a9, ta.pc.c2, ta.pc.c3),
                mode,
                &mut ta.rcc,
            );

            let mut m = Spi::new_spi1_full_duplex(
                ta.spi1,
                (ta.pa.a5, ta.pa.a6, ta.pa.a7),
                mode,
                br,
                &mut ta.rcc,
            );

            for _ in 0..8 {
                s.set_ssi(false);
                unwrap!(m.write(DATA));
                let mut buf: [u8; 3] = [0; 3];
                unwrap!(s.read(&mut buf));
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);
            }

            for _ in 0..8 {
                s.set_ssi(false);
                unwrap!(s.write(DATA));
                let mut buf: [u8; 3] = [0x12, 0x34, 0x56];
                unwrap!(m.transfer(&mut buf));
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);

                let mut slave_buf: [u8; 3] = [0; 3];
                unwrap!(s.read(&mut slave_buf));
                defmt::assert_eq!(slave_buf, [0x12, 0x34, 0x56]);
            }
        }
    }

    #[test]
    fn full_duplex_loopback_dma() {
        for (&br, &mode) in iproduct!(BAUD_RATES.iter(), SPI_MODES.iter()) {
            defmt::debug!("÷{} MODE_{}", br.div(), mode_num(mode));
            let mut ta: TestArgs = unsafe { setup() };

            let mut s = Spi::new_spi2_full_duplex_slave_dma(
                ta.spi2,
                (ta.pa.a9, ta.pc.c2, ta.pc.c3),
                (ta.dma.d1.c1, ta.dma.d1.c2),
                mode,
                &mut ta.rcc,
            );

            let mut m = Spi::new_spi1_full_duplex_dma(
                ta.spi1,
                (ta.pa.a5, ta.pa.a6, ta.pa.a7),
                (ta.dma.d2.c1, ta.dma.d2.c2),
                mode,
                br,
                &mut ta.rcc,
            );

            for _ in 0..8 {
                s.set_ssi(false);
                unwrap!(m.write(DATA));
                let mut buf: [u8; 3] = [0; 3];
                unwrap!(s.read(&mut buf));
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);
            }

            for _ in 0..8 {
                s.set_ssi(false);
                unwrap!(s.write(DATA));
                let mut buf: [u8; 3] = [0x12, 0x34, 0x56];
                unwrap!(m.transfer(&mut buf));
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);

                let mut slave_buf: [u8; 3] = [0; 3];
                unwrap!(s.read(&mut slave_buf));
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
                Spi::new_spi2_mosi_simplex_slave(ta.spi2, (ta.pa.a9, ta.pc.c3), mode, &mut ta.rcc);

            let mut m =
                Spi::new_spi1_mosi_simplex(ta.spi1, (ta.pa.a5, ta.pa.a7), mode, br, &mut ta.rcc);

            for _ in 0..8 {
                s.set_ssi(false);
                unwrap!(m.write(DATA));
                let mut buf: [u8; 3] = [0; 3];
                unwrap!(s.read(&mut buf));
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);
            }
        }
    }

    #[test]
    fn mosi_simplex_loopback_dma() {
        for (&br, &mode) in iproduct!(BAUD_RATES.iter(), SPI_MODES.iter()) {
            defmt::debug!("÷{} MODE_{}", br.div(), mode_num(mode));
            let mut ta: TestArgs = unsafe { setup() };

            let mut s = Spi::new_spi2_mosi_simplex_slave_dma(
                ta.spi2,
                (ta.pa.a9, ta.pc.c3),
                ta.dma.d1.c2,
                mode,
                &mut ta.rcc,
            );

            let mut m = Spi::new_spi1_mosi_simplex_dma(
                ta.spi1,
                (ta.pa.a5, ta.pa.a7),
                ta.dma.d1.c1,
                mode,
                br,
                &mut ta.rcc,
            );

            for _ in 0..8 {
                s.set_ssi(false);
                unwrap!(m.write(DATA));
                let mut buf: [u8; 3] = [0; 3];
                unwrap!(s.read(&mut buf));
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);
            }
        }
    }

    #[test]
    fn miso_simplex_loopback() {
        for (&br, &mode) in iproduct!(BAUD_RATES.iter(), SPI_MODES.iter()) {
            defmt::debug!("÷{} MODE_{}", br.div(), mode_num(mode));
            let mut ta: TestArgs = unsafe { setup() };

            let mut s =
                Spi::new_spi2_miso_simplex_slave(ta.spi2, (ta.pa.a9, ta.pc.c2), mode, &mut ta.rcc);

            let mut m = Spi::new_spi1_full_duplex(
                ta.spi1,
                (ta.pa.a5, ta.pa.a6, ta.pa.a7),
                mode,
                br,
                &mut ta.rcc,
            );

            for _ in 0..8 {
                s.set_ssi(false);
                unwrap!(s.write(DATA));
                let mut buf: [u8; 3] = [0; 3];
                unwrap!(m.transfer(&mut buf));
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);
            }
        }
    }

    #[test]
    fn miso_simplex_loopback_dma() {
        for (&br, &mode) in iproduct!(BAUD_RATES.iter(), SPI_MODES.iter()) {
            defmt::debug!("÷{} MODE_{}", br.div(), mode_num(mode));
            let mut ta: TestArgs = unsafe { setup() };

            let mut s = Spi::new_spi2_miso_simplex_slave_dma(
                ta.spi2,
                (ta.pa.a9, ta.pc.c2),
                ta.dma.d1.c2,
                mode,
                &mut ta.rcc,
            );

            let mut m = Spi::new_spi1_full_duplex(
                ta.spi1,
                (ta.pa.a5, ta.pa.a6, ta.pa.a7),
                mode,
                br,
                &mut ta.rcc,
            );

            for _ in 0..8 {
                s.set_ssi(false);
                unwrap!(s.write(DATA));
                let mut buf: [u8; 3] = [0; 3];
                unwrap!(m.transfer(&mut buf));
                s.set_ssi(true);
                defmt::assert_eq!(buf, DATA);
            }
        }
    }
}
