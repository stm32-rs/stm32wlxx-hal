#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    embedded_hal::timer::CountDown,
    lptim::{self, LpTim, LpTim1, LpTim2, LpTim3, Prescaler},
    pac::{self, DWT},
    rcc,
    util::reset_cycle_count,
};

const LPTIM3_FREQ: u32 = 16_000_000;
const FREQ: u32 = 48_000_000;
const CYC_PER_US: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:µs}", DWT::get_cycle_count() / CYC_PER_US);

#[defmt_test::tests]
mod tests {
    use super::*;

    #[allow(dead_code)]
    struct TestArgs {
        lptim1: LpTim1,
        lptim2: LpTim2,
        lptim3: LpTim3,
        rcc: pac::RCC,
    }

    #[init]
    fn init() -> TestArgs {
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC);
        defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);

        dp.RCC.cr.write(|w| w.hsion().set_bit());
        while dp.RCC.cr.read().hsirdy().is_not_ready() {}

        defmt::assert_eq!(LpTim1::clk(&dp.RCC), lptim::Clk::Pclk);

        let lptim1: LpTim1 =
            LpTim1::new(dp.LPTIM1, lptim::Clk::Hsi16, Prescaler::Div1, &mut dp.RCC);
        let lptim2: LpTim2 =
            LpTim2::new(dp.LPTIM2, lptim::Clk::Hsi16, Prescaler::Div1, &mut dp.RCC);
        let lptim3: LpTim3 =
            LpTim3::new(dp.LPTIM3, lptim::Clk::Hsi16, Prescaler::Div1, &mut dp.RCC);

        defmt::assert_eq!(LpTim1::clk(&dp.RCC), lptim::Clk::Hsi16);

        defmt::assert_eq!(lptim1.hz(&dp.RCC).to_integer(), LPTIM3_FREQ);
        defmt::assert_eq!(FREQ % LPTIM3_FREQ, 0);

        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        reset_cycle_count(&mut cp.DWT);

        TestArgs {
            lptim1,
            lptim2,
            lptim3,
            rcc: dp.RCC,
        }
    }

    #[test]
    fn oneshot(ta: &mut TestArgs) {
        const CYCLES: u16 = 10_000;
        let start: u32 = DWT::get_cycle_count();
        ta.lptim3.start(CYCLES);
        unwrap!(nb::block!(ta.lptim3.wait()).ok());
        let end: u32 = DWT::get_cycle_count();

        // compare elapsed lptim cycles to elapsed CPU cycles
        let elapsed: u32 = (end - start) * (FREQ / LPTIM3_FREQ);

        const TOLERANCE: u32 = 100;
        let elapsed_upper: u32 = elapsed + TOLERANCE;
        let elapsed_lower: u32 = elapsed - TOLERANCE;

        defmt::debug!("{} < {} < {}", elapsed_lower, elapsed, elapsed_upper);
        defmt::assert!(elapsed_lower <= elapsed && elapsed <= elapsed_upper);
    }
}
