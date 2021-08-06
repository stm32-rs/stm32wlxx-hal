#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _; // panic handler
use stm32wl_hal::{
    self as hal,
    adc::{self, Adc, SampleTime},
    cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
    dac::{Dac, ModeChip},
    pac, rcc,
};

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();

    // enable the HSI16 source clock
    dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    while dp.RCC.cr.read().hsirdy().is_not_ready() {}

    let mut delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));

    let mut adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);
    adc.calibrate(&mut delay);

    adc.enable();
    adc.enable_vbat();
    adc.set_sample_times(0, SampleTime::Cyc160, SampleTime::Cyc160);
    let sample: u16 = adc.vbat();
    defmt::info!("VBAT {}", sample);

    loop {
        hal::cortex_m::asm::bkpt();
    }
}
