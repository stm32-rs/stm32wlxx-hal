#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use nucleo_wl55jc_bsp::hal::{
    adc::{self, Adc},
    cortex_m::{self, delay::Delay, peripheral::syst::SystClkSource},
    dac::{Dac, ModeChip, ModePin},
    gpio::{Analog, PortA},
    pac, rcc,
};
use panic_probe as _;

#[defmt_test::tests]
mod tests {
    use super::*;

    struct TestArgs {
        adc: Adc,
        dac: Dac,
        delay: Delay,
    }

    #[init]
    fn init() -> TestArgs {
        cortex_m::interrupt::free(|cs| {
            let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
            let cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());

            unsafe { rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs) };

            let delay: Delay =
                Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));

            dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
            while dp.RCC.cr.read().hsirdy().is_not_ready() {}

            dp.RCC.csr.modify(|_, w| w.lsion().on());
            while dp.RCC.csr.read().lsirdy().is_not_ready() {}

            let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
            let mut dac: Dac = Dac::new(dp.DAC, &mut dp.RCC);
            let adc: Adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);

            dac.set_mode_pin(Analog::new(gpioa.a10, cs), ModePin::NormChipBuf);

            TestArgs { adc, dac, delay }
        })
    }

    #[test]
    fn pin_output(ta: &mut TestArgs) {
        ta.dac.setup_soft_trigger();
        ta.dac.soft_trigger(2048);
        defmt::assert_eq!(ta.dac.out(), 2048);

        // insert a delay here if you want to poke around with a voltmeter
        // should be appx 1.65V
        // ta.delay.delay_ms(10_000);
    }

    #[test]
    fn loopback(ta: &mut TestArgs) {
        ta.adc.start_disable();
        while !ta.adc.is_disabled() {}
        ta.adc.calibrate(&mut ta.delay);
        ta.adc.enable();
        ta.adc.set_max_sample_time();

        ta.dac.disable();
        unwrap!(ta.dac.set_mode_chip(ModeChip::Norm));
        ta.dac.setup_soft_trigger();
        ta.dac.soft_trigger(1024);

        let out: u16 = ta.dac.out();
        let sample: u16 = ta.adc.dac();
        defmt::info!("DAC out: {}", out);
        defmt::info!("ADC in: {}", sample);
        let delta: i32 = (i32::from(sample) - i32::from(ta.dac.out())).abs();

        defmt::assert!(delta < 20);
    }
}
