#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use nucleo_wl55jc_bsp::hal::{
    cortex_m,
    info::{self, Core, Uid64},
    pac::{self, DWT},
    rcc,
};
use panic_probe as _;

const FREQ: u32 = 48_000_000;
const CYC_PER_MICRO: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:us}", DWT::cycle_count() / CYC_PER_MICRO);

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() {
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());

        cortex_m::interrupt::free(|cs| unsafe {
            rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs)
        });

        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        cp.DWT.set_cycle_count(0);
    }

    #[test]
    fn core() {
        defmt::assert_eq!(Core::CT, Core::Cm4);
        defmt::assert_eq!(Core::from_cpuid(), Core::Cm4);
    }

    #[test]
    fn flash_size() {
        defmt::assert_eq!(info::flash_size_kibibyte(), 256);
        defmt::assert_eq!(info::flash_size(), 256 * 1024);
    }

    #[test]
    fn uid64() {
        defmt::assert_eq!(Uid64::from_device().dev_id(), 0x15);
        defmt::assert_eq!(Uid64::from_device().company_id(), 0x0080E1);
        defmt::assert_eq!(Uid64::from_device().devnum(), Uid64::read_devnum());
    }
}
