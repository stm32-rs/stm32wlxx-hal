#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    embedded_hal::digital::v2::ToggleableOutputPin,
    embedded_hal::timer::CountDown,
    gpio::{pins, LpTim3Trg, Output, PortA, PortB},
    lptim::{self, Filter, LpTim, LpTim1, LpTim2, LpTim3, Prescaler, TrgPol, TrgSel3},
    pac::{self, DWT},
    rcc,
    util::reset_cycle_count,
};

const LPTIM1_FREQ: u32 = 125_000;
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
        b7: Output<pins::B7>,
    }

    #[init]
    fn init() -> TestArgs {
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());

        unsafe { rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC) };
        defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);

        dp.RCC.cr.write(|w| w.hsion().set_bit());
        while dp.RCC.cr.read().hsirdy().is_not_ready() {}

        defmt::assert_eq!(LpTim1::clk(&dp.RCC), lptim::Clk::Pclk);

        let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
        let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
        let _: LpTim3Trg = LpTim3Trg::new(gpioa.a11);

        let lptim1: LpTim1 =
            LpTim1::new(dp.LPTIM1, lptim::Clk::Hsi16, Prescaler::Div128, &mut dp.RCC);
        let lptim2: LpTim2 =
            LpTim2::new(dp.LPTIM2, lptim::Clk::Hsi16, Prescaler::Div1, &mut dp.RCC);
        let lptim3: LpTim3 =
            LpTim3::new(dp.LPTIM3, lptim::Clk::Hsi16, Prescaler::Div1, &mut dp.RCC);

        defmt::assert_eq!(LpTim1::clk(&dp.RCC), lptim::Clk::Hsi16);

        defmt::assert_eq!(lptim1.hz(&dp.RCC).to_integer(), LPTIM1_FREQ);
        defmt::assert_eq!(lptim3.hz(&dp.RCC).to_integer(), LPTIM3_FREQ);
        defmt::assert_eq!(FREQ % LPTIM1_FREQ, 0);
        defmt::assert_eq!(FREQ % LPTIM3_FREQ, 0);

        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        reset_cycle_count(&mut cp.DWT);

        TestArgs {
            lptim1,
            lptim2,
            lptim3,
            rcc: dp.RCC,
            b7: Output::default(gpiob.b7),
        }
    }

    #[test]
    fn oneshot(ta: &mut TestArgs) {
        const CYCLES: u16 = 100;
        let start: u32 = DWT::get_cycle_count();
        ta.lptim1.start(CYCLES);
        unwrap!(nb::block!(ta.lptim1.wait()).ok());
        let end: u32 = DWT::get_cycle_count();

        // compare elapsed lptim cycles to elapsed CPU cycles
        let elapsed: u32 = (end - start) * (FREQ / LPTIM1_FREQ);

        const TOLERANCE: u32 = 100;
        let elapsed_upper: u32 = elapsed + TOLERANCE;
        let elapsed_lower: u32 = elapsed - TOLERANCE;

        defmt::debug!("{} < {} < {}", elapsed_lower, elapsed, elapsed_upper);
        defmt::assert!(elapsed_lower <= elapsed && elapsed <= elapsed_upper);
    }

    #[test]
    fn oneshot_external_trigger(ta: &mut TestArgs) {
        defmt::warn!("Pin B7 must be connected to A11 for this test to pass");

        const CYCLES: u16 = 10_000;
        unsafe { LpTim3::pulse_reset(&mut ta.rcc) };

        ta.lptim3
            .setup_trigger(Filter::Any, TrgPol::Both, TrgSel3::Pin);
        ta.lptim3.start(CYCLES);

        // wait 10 LPTIM3 cycles
        let start: u32 = DWT::get_cycle_count();
        loop {
            let elapsed: u32 = DWT::get_cycle_count() - start;
            if elapsed > (FREQ / LPTIM3_FREQ) * 10 {
                break;
            }
        }

        // timer should still read 0 because it has not triggered
        defmt::assert_eq!(LpTim3::cnt(), 0);

        let start: u32 = DWT::get_cycle_count();
        // timer should start when this pin toggles
        unwrap!(ta.b7.toggle());
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
