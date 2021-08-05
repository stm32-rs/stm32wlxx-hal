// Prints over RTT when the state of B3 on the NUCLEO-WL55JC2 changes.

#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _; // panic handler
use stm32wl_hal::{
    self as hal,
    gpio::{pins, Input, Level, PortC, Pull},
    pac,
};

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();

    let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    let pc6: Input<pins::C6> = Input::new(gpioc.pc6, Pull::Up);

    let mut prev_level: Level = pc6.level();
    defmt::info!("B3 initial level: {:?}", prev_level);

    loop {
        let level: Level = pc6.level();
        if level != prev_level {
            defmt::info!("B3 state changed from {:?} to {:?}", prev_level, level);
            prev_level = level;
        }
    }
}
