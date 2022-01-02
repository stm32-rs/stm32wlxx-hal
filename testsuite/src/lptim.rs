#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use nucleo_wl55jc_bsp::hal::{
    cortex_m,
    embedded_hal::digital::v2::ToggleableOutputPin,
    embedded_hal::timer::CountDown,
    gpio::{pins, Output, PortA, PortB},
    lptim::{self, Filter, LpTim, LpTim1, LpTim2, LpTim3, Prescaler, TrgPol},
    pac::{self, DWT},
    rcc,
};
use panic_probe as _;

const FREQ: u32 = 48_000_000;
const CYC_PER_US: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:us}", DWT::cycle_count() / CYC_PER_US);

fn test_clk_src<LPTIM>(lptim: &mut LPTIM, src: lptim::Clk, rcc: &pac::RCC)
where
    LPTIM: LpTim + CountDown,
    <LPTIM as CountDown>::Time: From<u16>,
{
    defmt::assert_eq!(LPTIM::clk(rcc), src);
    let lptim_freq: u32 = lptim.hz().to_integer();

    let cycles: u16 = u16::try_from(lptim_freq / 32).unwrap_or(u16::MAX);
    let start: u32 = DWT::cycle_count();
    lptim.start(cycles);
    unwrap!(nb::block!(lptim.wait()).ok());
    let end: u32 = DWT::cycle_count();

    // compare elapsed lptim cycles to elapsed CPU cycles
    let elapsed: u32 = end.wrapping_sub(start);
    let expected_elapsed: u32 = u32::from(cycles) * (FREQ / lptim_freq);

    let elapsed_upper: u32 = if lptim_freq > 10_000 {
        // 6.25% tolerance
        expected_elapsed + expected_elapsed / 16
    } else {
        // upper limit gets massively skewed at low frequencies due to
        // delays when enabling the timer
        expected_elapsed.saturating_mul(3)
    };

    // the embedded-hal trait guarantees **at least** n cycles
    // this _should_ just be `expected_elapsed`, but there is some
    // measurement error with independent clock sources
    let elapsed_lower: u32 = expected_elapsed - expected_elapsed / 128;

    defmt::assert!(
        elapsed_lower <= elapsed && elapsed <= elapsed_upper,
        "Timer is incorrect: {} <= {} <= {}",
        elapsed_lower,
        elapsed,
        elapsed_upper
    );
}

#[defmt_test::tests]
mod tests {
    use super::*;

    struct TestArgs {
        lptim3: LpTim3,
        rcc: pac::RCC,
        b7: Output<pins::B7>,
    }

    #[init]
    fn init() -> TestArgs {
        cortex_m::interrupt::free(|cs| {
            let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
            let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());

            unsafe { rcc::pulse_reset_backup_domain(&mut dp.RCC, &mut dp.PWR) };
            unsafe { rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs) };
            defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);

            cp.DCB.enable_trace();
            cp.DWT.enable_cycle_counter();
            cp.DWT.set_cycle_count(0);

            // enable HSI
            dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
            while dp.RCC.cr.read().hsirdy().is_not_ready() {}

            // enable LSE
            dp.PWR.cr1.modify(|_, w| w.dbp().enabled());
            dp.RCC
                .bdcr
                .modify(|_, w| w.lseon().on().lsesysen().enabled());
            while dp.RCC.bdcr.read().lserdy().is_not_ready() {}

            // enable LSI
            rcc::enable_lsi(&mut dp.RCC);

            let _: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
            let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);

            let lptim3: LpTim3 =
                LpTim3::new(dp.LPTIM3, lptim::Clk::Hsi16, Prescaler::Div1, &mut dp.RCC);

            TestArgs {
                lptim3,
                rcc: dp.RCC,
                b7: Output::default(gpiob.b7, cs),
            }
        })
    }

    #[test]
    fn oneshot_external_trigger(ta: &mut TestArgs) {
        defmt::warn!("Pin B7 must be connected to A11 for this test to pass");
        let lptim3_freq: u32 = ta.lptim3.hz().to_integer();
        defmt::assert_eq!(FREQ % lptim3_freq, 0);

        let a11 = unsafe { PortA::steal() }.a11;

        cortex_m::interrupt::free(|cs| {
            ta.lptim3
                .new_trigger_pin(a11, Filter::Any, TrgPol::Both, cs)
        });
        ta.lptim3.start(u16::MAX);

        // wait 10 LPTIM3 cycles
        let start: u32 = DWT::cycle_count();
        loop {
            let elapsed: u32 = DWT::cycle_count() - start;
            if elapsed > (FREQ / lptim3_freq) * 10 {
                break;
            }
        }

        // timer should still read 0 because it has not triggered
        defmt::assert_eq!(LpTim3::cnt(), 0);

        // timer should start when this pin toggles
        unwrap!(ta.b7.toggle());

        // wait 10 LPTIM3 cycles
        let start: u32 = DWT::cycle_count();
        loop {
            let elapsed: u32 = DWT::cycle_count() - start;
            if elapsed > (FREQ / lptim3_freq) * 10 {
                break;
            }
        }

        defmt::assert_ne!(LpTim3::cnt(), 0);
    }

    #[test]
    fn oneshot(ta: &mut TestArgs) {
        const SRCS: [lptim::Clk; 4] = [
            lptim::Clk::Hsi16,
            lptim::Clk::Lse,
            lptim::Clk::Lsi,
            lptim::Clk::Pclk,
        ];
        const PRESCALERS: [lptim::Prescaler; 8] = [
            lptim::Prescaler::Div1,
            lptim::Prescaler::Div2,
            lptim::Prescaler::Div4,
            lptim::Prescaler::Div8,
            lptim::Prescaler::Div16,
            lptim::Prescaler::Div32,
            lptim::Prescaler::Div64,
            lptim::Prescaler::Div128,
        ];

        for (src, pre) in itertools::iproduct!(SRCS, PRESCALERS) {
            let dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
            let mut lptim1: LpTim1 = LpTim1::new(dp.LPTIM1, src, pre, &mut ta.rcc);
            let freq: u32 = lptim1.hz().to_integer();
            defmt::info!("LPTIM1 {} / {} = {} Hz", src, pre.div(), freq);
            test_clk_src(&mut lptim1, src, &dp.RCC);
        }

        for (src, pre) in itertools::iproduct!(SRCS, PRESCALERS) {
            let dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
            let mut lptim2: LpTim2 = LpTim2::new(dp.LPTIM2, src, pre, &mut ta.rcc);
            let freq: u32 = lptim2.hz().to_integer();
            defmt::info!("LPTIM2 {} / {} = {} Hz", src, pre.div(), freq);
            test_clk_src(&mut lptim2, src, &dp.RCC);
        }

        for (src, pre) in itertools::iproduct!(SRCS, PRESCALERS) {
            let dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
            let mut lptim3: LpTim3 = LpTim3::new(dp.LPTIM3, src, pre, &mut ta.rcc);
            let freq: u32 = lptim3.hz().to_integer();
            defmt::info!("LPTIM3 {} / {} = {} Hz", src, pre.div(), freq);
            test_clk_src(&mut lptim3, src, &dp.RCC);
        }
    }
}
