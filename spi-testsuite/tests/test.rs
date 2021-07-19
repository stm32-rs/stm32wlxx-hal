#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    eh::blocking::spi::{Transfer, Write},
    pac, rcc,
    spi::{BaudDiv::DIV128, Spi3, Spi3Dma},
};

fn rfbusys() -> bool {
    unsafe { pac::Peripherals::steal() }
        .PWR
        .sr2
        .read()
        .rfbusys()
        .bit_is_set()
}

fn poll_not_busy() {
    // TODO: this is a terrible timeout
    let mut count: u32 = 1_000_000;
    while rfbusys() {
        count -= 1;
        if count == 0 {
            let dp = unsafe { pac::Peripherals::steal() };
            panic!(
                "pwr.sr2=0x{:X} pwr.subghzspicr=0x{:X} pwr.cr1=0x{:X}",
                dp.PWR.sr2.read().bits(),
                dp.PWR.subghzspicr.read().bits(),
                dp.PWR.cr1.read().bits(),
            );
        }
    }
}

fn clear_nss() {
    unsafe { pac::Peripherals::steal() }
        .PWR
        .subghzspicr
        .write(|w| w.nss().clear_bit());
}

fn set_nss() {
    unsafe { pac::Peripherals::steal() }
        .PWR
        .subghzspicr
        .write(|w| w.nss().set_bit());
}

#[defmt_test::tests]
mod tests {
    use stm32wl_hal::{dma::AllDma, gpio::PortA, subghz::SubGhz};

    use super::*;

    #[init]
    fn init() -> Spi3Dma {
        let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
        let mut rcc = dp.RCC;

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut rcc);

        let dma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut rcc);
        let p = PortA::split(dp.GPIOA, &mut rcc);

        let mut sg: SubGhz = SubGhz::new(dp.SPI3, &mut rcc);
        sg.enable_spi_debug(p.pa4, p.pa5, p.pa6, p.pa7);
        let spi3 = sg.free();

        // this will put the radio into a bad state
        let spi3 = Spi3Dma::new(Spi3::new(spi3, DIV128, &mut rcc), dma.d1c1, dma.d2c2);

        // reset radio
        rcc.csr.modify(|_, w| w.rfrst().set_bit());
        rcc.csr.modify(|_, w| w.rfrst().clear_bit());

        // pulse NSS
        clear_nss();
        set_nss();

        // clear RF busy
        unsafe { pac::Peripherals::steal() }
            .PWR
            .scr
            .write(|w| w.cwrfbusyf().set_bit());

        spi3
    }

    #[test]
    fn spi_dma_poll(spi: &mut Spi3Dma) {
        const DATA: [u8; 255] = [0xA5; 255];
        let mut buf: [u8; 255] = [0; 255];
        let mut status_buf: [u8; 1] = [0];

        defmt::info!("polling for not busy");

        poll_not_busy();

        defmt::info!("starting buffer write");

        clear_nss();

        // write buffer at offset 0
        spi.write(&[0x0E, 0]).unwrap();
        spi.write(&DATA).unwrap();

        set_nss();

        defmt::info!("done buffer write");

        poll_not_busy();

        defmt::info!("starting buffer read");

        clear_nss();

        // read buffer at offset 0
        spi.write(&[0x1E, 0]).unwrap();
        spi.transfer(&mut status_buf).unwrap();
        spi.transfer(&mut buf).unwrap();

        set_nss();

        assert_eq!(DATA, buf);
    }
}
