// Blinks the 3 LEDs on the NUCLEO-WL55JC2 in a sequence.

#![no_std]
#![no_main]

use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use cortex_m::peripheral::syst::SystClkSource;
use panic_rtt_target as _;
use rtt_target::rprintln;
use stm32wl_hal::{
    adc::{self, Adc},
    cortex_m::delay::Delay,
    pac, rcc,
};

#[cortex_m_rt::entry]
fn main() -> ! {
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 4096
                mode: BlockIfFull
                name: "Terminal"
            }
        }
    };
    rtt_target::set_print_channel(channels.up.0);
    rprintln!("Hello from rprintln!");

    let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    let mut cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();

    rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC);

    let mut delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));

    dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    let mut adc: Adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);

    rprintln!("Starting sample");
    rprintln!("Temperature: {} °C", adc.temperature(&mut delay).round());
    rprintln!("Temperature: {} °C", adc.temperature(&mut delay).round());
    rprintln!("VREFINT: {}", adc::vrefint_cal());
    rprintln!("VREF: {}", adc.vref());
    adc.calibrate(&mut delay);
    rprintln!("Temperature: {} °C", adc.temperature(&mut delay).round());
    rprintln!("Temperature: {} °C", adc.temperature(&mut delay).round());
    rprintln!("VREF: {}", adc.vref());

    loop {
        compiler_fence(SeqCst);
        cortex_m::asm::bkpt();
    }
}
