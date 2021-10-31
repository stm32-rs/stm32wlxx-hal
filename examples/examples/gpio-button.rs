// Prints over RTT when the state of B3 on the NUCLEO-WL55JC2 changes.

#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _; // panic handler
use stm32wlxx_hal::{
    self as hal, cortex_m,
    gpio::{pins, Input, PinState, PortC, Pull},
    pac,
};

const fn pinstate_str(state: PinState) -> &'static str {
    match state {
        PinState::Low => "Low",
        PinState::High => "High",
    }
}

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    let mut dp: pac::Peripherals = defmt::unwrap!(pac::Peripherals::take());

    let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    let c6: Input<pins::C6> = cortex_m::interrupt::free(|cs| Input::new(gpioc.c6, Pull::Up, cs));

    let mut prev_level: PinState = c6.level();
    defmt::info!("B3 initial level: {}", pinstate_str(prev_level));

    loop {
        let level: PinState = c6.level();
        if level != prev_level {
            defmt::info!(
                "B3 state changed from {} to {}",
                pinstate_str(prev_level),
                pinstate_str(level)
            );
            prev_level = level;
        }
    }
}
