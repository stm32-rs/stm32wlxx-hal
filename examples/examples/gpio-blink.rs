// Blinks the 3 LEDs on the NUCLEO-WL55JC2 in a sequence.

#![no_std]
#![no_main]

use cortex_m::{delay::Delay, peripheral::syst::SystClkSource};
use panic_rtt_target as _;
use rtt_target::rprintln;
use stm32wl_hal::{
    gpio::{Level, Output, PortB},
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
    let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();

    let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    let mut led1 = Output::default(gpiob.pb9);
    let mut led2 = Output::default(gpiob.pb15);
    let mut led3 = Output::default(gpiob.pb11);

    let mut delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));

    rprintln!("Starting blinky");

    loop {
        for &level in &[Level::High, Level::Low] {
            led1.set_level(level);
            delay.delay_ms(600);
            led2.set_level(level);
            delay.delay_ms(600);
            led3.set_level(level);
            delay.delay_ms(600);
        }
    }
}
