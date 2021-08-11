#![no_std]
#![no_main]

use core::convert::TryFrom;
use defmt::unwrap;
use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    adc::{self, Adc},
    cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
    pac, rcc,
};

fn validate_temperature(temp: i16) {
    defmt::info!("Temperature: {} °C", temp);
    defmt::assert!(temp > 25);
    defmt::assert!(temp < 70);
}

fn validate_vref(pre: u16, post: u16) {
    let vref_cal: u16 = adc::vref_cal();
    defmt::info!("vref_cal: {}", vref_cal);
    let pre_delta: i16 = (pre as i16) - (vref_cal as i16);
    let post_delta: i16 = (post as i16) - (vref_cal as i16);
    defmt::info!("vref pre-calibration: {} Δ {}", pre, pre_delta.abs());
    defmt::info!("vref post-calibration: {} Δ {}", post, post_delta.abs());
    defmt::assert!(post_delta.abs() < pre_delta.abs());
}

#[defmt_test::tests]
mod tests {
    use super::*;

    struct TestArgs {
        adc: Adc,
        delay: Delay,
    }

    #[init]
    fn init() -> TestArgs {
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
        let cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC);

        let delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));

        dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
        while dp.RCC.cr.read().hsirdy().is_not_ready() {}
        let adc: Adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);

        assert_eq!(adc.clock_hz(&dp.RCC), 16_000_000);

        TestArgs { adc, delay }
    }

    #[test]
    fn enable(ta: &mut TestArgs) {
        defmt::assert!(ta.adc.is_disabled());
        defmt::assert!(!ta.adc.is_enabled());

        // test disable -> enable and enable -> enable
        for _ in 0..2 {
            ta.adc.enable();
            defmt::assert!(!ta.adc.is_disabled());
            defmt::assert!(ta.adc.is_enabled());
        }

        // test enable -> disable and disable -> disable
        for _ in 0..2 {
            ta.adc.start_disable();
            while !ta.adc.is_disabled() {}
            defmt::assert!(ta.adc.is_disabled());
            defmt::assert!(!ta.adc.is_enabled());
        }
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

        validate_vref(pre, post);
    }

    #[test]
    fn temperature(ta: &mut TestArgs) {
        ta.adc.enable_tsen();
        ta.delay.delay_us(adc::TS_START_MAX.as_micros() as u32);
        ta.adc.set_max_sample_time();
        let temp: i16 = ta.adc.temperature().to_integer();
        validate_temperature(temp);
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
        const EXPECTED: i16 = 4096 / 3;

        let delta: i16 = unwrap!(i16::try_from(sample).ok()) - EXPECTED;

        defmt::info!("VBAT={} Δ {}", sample, delta);

        defmt::assert!(delta < 20);
    }
}
