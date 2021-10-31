// Blinks the 3 LEDs on the NUCLEO-WL55JC2 in a sequence.

#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _; // panic handler
use stm32wlxx_hal::{
    self as hal,
    cortex_m::{self, delay::Delay},
    gpio::{pins, Output, PinState, PortB},
    pac,
    util::new_delay,
};

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    let mut dp: pac::Peripherals = defmt::unwrap!(pac::Peripherals::take());
    let cp: pac::CorePeripherals = defmt::unwrap!(pac::CorePeripherals::take());

    let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    let (mut led1, mut led2, mut led3): (Output<pins::B9>, Output<pins::B15>, Output<pins::B11>) =
        cortex_m::interrupt::free(|cs| {
            (
                Output::default(gpiob.b9, cs),
                Output::default(gpiob.b15, cs),
                Output::default(gpiob.b11, cs),
            )
        });

    let mut delay: Delay = new_delay(cp.SYST, &dp.RCC);

    defmt::info!("Starting blinky");

    loop {
        for &level in &[PinState::High, PinState::Low] {
            led1.set_level(level);
            delay.delay_ms(600);
            led2.set_level(level);
            delay.delay_ms(600);
            led3.set_level(level);
            delay.delay_ms(600);
        }
    }
}
