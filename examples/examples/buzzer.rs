// Sounds a buzzer, will work only for TTN Generic node with STM32WL5x
// see https://github.com/TheThingsIndustries/generic-node-se
// The datasheet of the buzzer `MLT-7525` shows that the oscillation frequency is `2700Hz` so the period
// is around 185us

#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _; // panic handler

use stm32wlxx_hal::{
    self as hal,
    cortex_m::{self, delay::Delay},
    gpio::{pins, Output, PinState, PortA},
    pac,
    util::new_delay,
};


#[hal::cortex_m_rt::entry]
fn main() -> ! {
    let mut dp: pac::Peripherals = defmt::unwrap!(pac::Peripherals::take());
    let cp: pac::CorePeripherals = defmt::unwrap!(pac::CorePeripherals::take());

    let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    let mut buzzer : Output<pins::A15> = cortex_m::interrupt::free(|cs| {Output::default(gpioa.a15, cs)});

    let mut delay: Delay = new_delay(cp.SYST, &dp.RCC);

    defmt::info!("Starting buzzer");

    loop {
        for &level in &[PinState::High, PinState::Low] {
            buzzer.set_level(level);
            delay.delay_us(185);
        }
    }
}
