#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    adc::{self, Adc},
    cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
    pac, rcc,
};

#[cfg(feature = "aio")]
async fn aio_temperature_inner() {
    let mut dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
    let cp: pac::CorePeripherals = unsafe { pac::CorePeripherals::steal() };
    let mut delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));
    let mut adc: Adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);

    let temp: i16 = adc.aio_temperature(&mut delay).await.to_integer();
    defmt::info!("Temperature: {} °C", temp);
    assert!(temp > 25);
    assert!(temp < 70);
}

#[cfg(feature = "aio")]
async fn aio_vref_inner() {
    let mut dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
    let cp: pac::CorePeripherals = unsafe { pac::CorePeripherals::steal() };
    let mut delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));
    let mut adc: Adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);

    let vref_cal: u16 = adc::vref_cal();
    defmt::info!("vref_cal: {}", vref_cal);

    let uncal_vref: u16 = adc.vref();
    let uncal_delta: i16 = (uncal_vref as i16) - (vref_cal as i16);
    defmt::info!(
        "vref pre-calibration: {} Δ {}",
        uncal_vref,
        uncal_delta.abs()
    );

    adc.aio_calibrate(&mut delay).await;

    let vref: u16 = adc.aio_vref().await;
    let delta: i16 = (vref as i16) - (vref_cal as i16);
    defmt::info!("vref post-calibration: {} Δ {}", vref, delta.abs());

    assert!(delta.abs() < uncal_delta.abs());
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
        let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
        let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC);

        let delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));

        #[cfg(feature = "aio")]
        {
            let start: usize = cortex_m_rt::heap_start() as usize;
            let size: usize = 2048; // in bytes
            unsafe { ate::ALLOCATOR.init(start, size) };
            unsafe { Adc::unmask_irq() };
        }

        dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
        while dp.RCC.cr.read().hsirdy().is_not_ready() {}
        let adc: Adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);

        TestArgs { adc, delay }
    }

    #[test]
    fn enable(ta: &mut TestArgs) {
        assert!(ta.adc.is_disabled());
        assert!(!ta.adc.is_enabled());

        // test disable -> enable and enable -> enable
        for _ in 0..2 {
            ta.adc.enable();
            assert!(!ta.adc.is_disabled());
            assert!(ta.adc.is_enabled());
        }

        // test enable -> disable and disable -> disable
        for _ in 0..2 {
            ta.adc.start_disable();
            while !ta.adc.is_disabled() {}
            assert!(ta.adc.is_disabled());
            assert!(!ta.adc.is_enabled());
        }
    }

    #[test]
    fn vref(ta: &mut TestArgs) {
        ta.adc.start_disable();
        while !ta.adc.is_disabled() {}

        let vref_cal: u16 = adc::vref_cal();
        defmt::info!("vref_cal: {}", vref_cal);

        let uncal_vref: u16 = ta.adc.vref();
        let uncal_delta: i16 = (uncal_vref as i16) - (vref_cal as i16);
        defmt::info!(
            "vref pre-calibration: {} Δ {}",
            uncal_vref,
            uncal_delta.abs()
        );

        ta.adc.calibrate(&mut ta.delay);

        let vref: u16 = ta.adc.vref();
        let delta: i16 = (vref as i16) - (vref_cal as i16);
        defmt::info!("vref post-calibration: {} Δ {}", vref, delta.abs());

        assert!(delta.abs() < uncal_delta.abs());
    }

    #[test]
    #[cfg(feature = "aio")]
    fn aio_vref(_ta: &mut TestArgs) {
        let mut executor = ate::Executor::new();
        executor.spawn(ate::Task::new(aio_vref_inner()));
        executor.run();
    }

    #[test]
    fn temperature(ta: &mut TestArgs) {
        let temp: i16 = ta.adc.temperature(&mut ta.delay).to_integer();
        defmt::info!("Temperature: {} °C", temp);
        assert!(temp > 25);
        assert!(temp < 70);
    }

    #[test]
    #[cfg(feature = "aio")]
    fn aio_temperature(_ta: &mut TestArgs) {
        let mut executor = ate::Executor::new();
        executor.spawn(ate::Task::new(aio_temperature_inner()));
        executor.run();
    }
}
