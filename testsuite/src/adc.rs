#![no_std]
#![no_main]

use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use defmt::unwrap;
use defmt_rtt as _; // global logger
use nucleo_wl55jc_bsp::hal::{
    adc::{self, Adc, Clk, OversampleRatio, OversampleShift},
    cortex_m::{self, delay::Delay},
    pac::{self, DWT},
    rcc,
    util::new_delay,
};
use panic_probe as _;

const ADC_FREQ: u32 = 12_000_000;
const FREQ: u32 = 48_000_000;
const FREQ_RATIO: u32 = FREQ / ADC_FREQ;
const CYC_PER_US: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:us}", DWT::cycle_count() / CYC_PER_US);

fn validate_vbat(sample: u16, oversample: i16) {
    const EXPECTED: i16 = 4096 / 3;
    let expected: i16 = EXPECTED * oversample;
    let delta: i16 = unwrap!(i16::try_from(sample).ok()) - expected;

    defmt::info!("VBAT={} Δ {}", sample, delta);
    let tolerance: i16 = 20 * oversample;
    defmt::assert!(delta < tolerance);
}

#[defmt_test::tests]
mod tests {
    use super::*;

    struct TestArgs {
        adc: Adc,
        delay: Delay,
        rcc: pac::RCC,
    }

    #[init]
    fn init() -> TestArgs {
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());

        cortex_m::interrupt::free(|cs| unsafe {
            rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs)
        });
        defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);

        let delay = new_delay(cp.SYST, &dp.RCC);

        let adc: Adc = Adc::new(dp.ADC, Clk::PClkDiv4, &mut dp.RCC);
        defmt::assert_eq!(adc.clock_hz(&dp.RCC), ADC_FREQ);

        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        cp.DWT.set_cycle_count(0);

        TestArgs {
            adc,
            delay,
            rcc: dp.RCC,
        }
    }

    #[test]
    fn enable(ta: &mut TestArgs) {
        defmt::assert!(ta.adc.is_disabled());
        defmt::assert!(!ta.adc.is_enabled());

        // disable -> enable
        let start: u32 = DWT::cycle_count();
        ta.adc.enable();
        let end: u32 = DWT::cycle_count();
        compiler_fence(SeqCst);
        let elapsed: u32 = end - start;
        defmt::info!(
            "Enable cycles: {} CPU {} ADC",
            elapsed,
            elapsed / FREQ_RATIO
        );
        defmt::assert!(!ta.adc.is_disabled());
        defmt::assert!(ta.adc.is_enabled());

        // enable -> enable
        ta.adc.enable();
        defmt::assert!(!ta.adc.is_disabled());
        defmt::assert!(ta.adc.is_enabled());

        // enable -> disable
        let start: u32 = DWT::cycle_count();
        ta.adc.disable();
        let end: u32 = DWT::cycle_count();
        compiler_fence(SeqCst);
        let elapsed: u32 = end - start;
        defmt::info!(
            "Disable cycles: {} CPU {} ADC",
            elapsed,
            elapsed / FREQ_RATIO
        );
        defmt::assert!(ta.adc.is_disabled());
        defmt::assert!(!ta.adc.is_enabled());

        // disable -> disable
        ta.adc.disable();
        defmt::assert!(ta.adc.is_disabled());
        defmt::assert!(!ta.adc.is_enabled());
    }

    #[test]
    fn tsen_enable(ta: &mut TestArgs) {
        defmt::assert!(!ta.adc.is_tsen_enabled());

        // test disable -> enable and enable -> enable
        for _ in 0..2 {
            ta.adc.enable_tsen();
            ta.delay.delay_us(adc::TS_START_MAX.as_micros() as u32);
            defmt::assert!(ta.adc.is_tsen_enabled());
        }

        // test enable -> disable and disable -> disable
        for _ in 0..2 {
            ta.adc.disable_tsen();
            defmt::assert!(!ta.adc.is_tsen_enabled());
        }
    }

    #[test]
    fn vbat_enable(ta: &mut TestArgs) {
        defmt::assert!(!ta.adc.is_vbat_enabled());

        // test disable -> enable and enable -> enable
        for _ in 0..2 {
            ta.adc.enable_vbat();
            defmt::assert!(ta.adc.is_vbat_enabled());
        }

        // test enable -> disable and disable -> disable
        for _ in 0..2 {
            ta.adc.disable_vbat();
            defmt::assert!(!ta.adc.is_vbat_enabled());
        }
    }

    #[test]
    fn vref_enable(ta: &mut TestArgs) {
        defmt::assert!(!ta.adc.is_vref_enabled());

        // test disable -> enable and enable -> enable
        for _ in 0..2 {
            ta.adc.enable_vref();
            defmt::assert!(ta.adc.is_vref_enabled());
        }

        // test enable -> disable and disable -> disable
        for _ in 0..2 {
            ta.adc.disable_vref();
            defmt::assert!(!ta.adc.is_vref_enabled());
        }
    }

    #[test]
    fn vref(ta: &mut TestArgs) {
        ta.adc.start_disable();
        while !ta.adc.is_disabled() {}

        ta.adc.enable();
        ta.adc.enable_vref();
        ta.adc.set_max_sample_time();
        let pre: u16 = ta.adc.vref();

        // long-form calibration
        defmt::assert_eq!(ta.adc.calfact(), 0);
        ta.adc.enable_vreg();
        ta.delay.delay_us(u32::from(adc::T_ADCVREG_SETUP_MICROS));
        ta.adc.start_calibrate();
        while Adc::isr().eocal().is_not_complete() {}
        defmt::assert_ne!(ta.adc.calfact(), 0);

        ta.adc.enable();
        let post: u16 = ta.adc.vref();

        let vref_cal: u16 = adc::vref_cal();
        defmt::info!("vref_cal: {}", vref_cal);
        let pre_delta: i16 = (pre as i16) - (vref_cal as i16);
        let post_delta: i16 = (post as i16) - (vref_cal as i16);
        defmt::info!("vref pre-calibration: {} Δ {}", pre, pre_delta.abs());
        defmt::info!("vref post-calibration: {} Δ {}", post, post_delta.abs());
        defmt::assert!(post_delta.abs() < pre_delta.abs());
    }

    #[test]
    fn test_hsi16(ta: &mut TestArgs) {
        let original: Clk = unwrap!(ta.adc.clock_source(&ta.rcc));

        // enable HSI16
        ta.rcc.cr.modify(|_, w| w.hsion().set_bit());
        while ta.rcc.cr.read().hsirdy().is_not_ready() {}

        ta.adc.disable();
        ta.adc.set_clock_source(Clk::RccHsi, &mut ta.rcc);
        defmt::assert_eq!(ta.adc.clock_source(&ta.rcc), Some(Clk::RccHsi));
        defmt::assert_eq!(ta.adc.clock_hz(&ta.rcc), 16_000_000);
        ta.adc.calibrate(&mut ta.delay);
        ta.adc.enable();
        ta.adc.enable_vref();
        ta.adc.set_max_sample_time();

        let vref_cal: u16 = adc::vref_cal();
        let vref: u16 = ta.adc.vref();

        let delta: i16 = ((vref_cal as i16) - (vref as i16)).abs();
        defmt::info!("vref: {} Δ {}", vref, delta);
        defmt::assert!(delta < 25);

        ta.adc.disable();
        ta.adc.set_clock_source(original, &mut ta.rcc);
        ta.rcc.cr.modify(|_, w| w.hsion().clear_bit());
    }

    #[test]
    fn temperature(ta: &mut TestArgs) {
        ta.adc.disable();
        ta.adc.calibrate(&mut ta.delay);
        ta.adc.enable();

        ta.adc.enable_tsen();
        ta.delay.delay_us(adc::TS_START_MAX.as_micros() as u32);
        ta.adc.set_max_sample_time();
        let temp: i16 = ta.adc.temperature().to_integer();

        defmt::info!("Temperature: {} °C", temp);
        defmt::assert!(temp > 25);
        defmt::assert!(temp < 70);
    }

    #[test]
    fn vbat(ta: &mut TestArgs) {
        // short form calibration
        defmt::assert_ne!(ta.adc.calfact(), 0);
        ta.adc.force_cal(0);
        defmt::assert_eq!(ta.adc.calfact(), 0);
        ta.adc.calibrate(&mut ta.delay);
        defmt::assert_ne!(ta.adc.calfact(), 0);

        ta.adc.enable();
        ta.adc.enable_vbat();
        ta.adc.set_max_sample_time();
        let sample: u16 = ta.adc.vbat();

        validate_vbat(sample, 1);
    }

    #[test]
    fn vbat_oversample(ta: &mut TestArgs) {
        ta.adc.disable();
        ta.adc
            .enable_oversampling(OversampleRatio::Mul2, OversampleShift::NoShift);
        ta.adc.enable();
        let sample: u16 = ta.adc.vbat();
        validate_vbat(sample / 2, 2);

        ta.adc.disable();
        defmt::assert!(ta.adc.is_oversampling_enabled());
        ta.adc.disable_oversampling();
        defmt::assert!(!ta.adc.is_oversampling_enabled());
    }

    #[test]
    fn vbat_advanced(ta: &mut TestArgs) {
        ta.adc.disable();

        ta.adc.enable_vreg();
        ta.delay.delay_us(u32::from(adc::T_ADCVREG_SETUP_MICROS));
        ta.adc.start_calibrate();
        let start: u32 = DWT::cycle_count();
        while Adc::isr().eocal().is_not_complete() {}
        let end: u32 = DWT::cycle_count();
        compiler_fence(SeqCst);
        let elapsed: u32 = end - start;
        defmt::info!(
            "Calibration cycles: CPU {} ADC {}",
            elapsed,
            elapsed / FREQ_RATIO
        );

        let _: bool = ta.adc.start_enable();
        let start: u32 = DWT::cycle_count();
        while Adc::isr().adrdy().is_not_ready() {}
        let end: u32 = DWT::cycle_count();
        let elapsed: u32 = end - start;
        defmt::info!(
            "Enable cycles: CPU {} ADC {}",
            elapsed,
            elapsed / FREQ_RATIO
        );

        ta.adc.start_chsel(adc::Ch::Vbat.mask());
        let start: u32 = DWT::cycle_count();
        while Adc::isr().ccrdy().is_not_complete() {}
        let end: u32 = DWT::cycle_count();
        compiler_fence(SeqCst);
        let elapsed: u32 = end - start;
        defmt::info!(
            "Channel configuration cycles: CPU {} ADC {}",
            elapsed,
            elapsed / FREQ_RATIO
        );

        ta.adc.set_max_sample_time();
        ta.adc.start_conversion();
        let start: u32 = DWT::cycle_count();
        while Adc::isr().eoc().is_not_complete() {}
        let end: u32 = DWT::cycle_count();
        compiler_fence(SeqCst);
        let elapsed: u32 = end - start;
        defmt::info!(
            "Sample cycles: CPU {} ADC {}",
            elapsed,
            elapsed / FREQ_RATIO
        );

        // this is a sanity check to ensure reading the ADC data register
        // does not have side effects because it is behind a &self reference
        // instead of &mut self
        let vbat1: u16 = ta.adc.data();
        let vbat2: u16 = ta.adc.data();
        validate_vbat(vbat1, 1);
        validate_vbat(vbat2, 1);
        defmt::assert_eq!(vbat1, vbat2);
    }

    #[test]
    fn stop_conversion(ta: &mut TestArgs) {
        ta.adc.set_isr(adc::irq::ALL);
        ta.adc.start_conversion();
        ta.adc.stop_conversion();

        // wait 161 ADC cycles (maximum sample time) before checking
        let start: u32 = DWT::cycle_count();
        loop {
            let elapsed: u32 = (DWT::cycle_count() - start) * FREQ_RATIO;
            if elapsed > u32::from(adc::Ts::MAX.cycles().to_integer()) + 1 {
                break;
            }
        }

        defmt::assert_eq!(Adc::isr().bits(), 0);

        // check that stop conversion works without a conversion in-progress
        ta.adc.stop_conversion();
    }
}
