// Prints default clock speeds, should work for all STM32WL5x boards.

#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _; // panic handler
use stm32wlxx_hal::{self as hal, pac, rcc};

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    let dp: pac::Peripherals = defmt::unwrap!(pac::Peripherals::take());

    defmt::println!("sysclk_hz: {}", rcc::sysclk_hz(&dp.RCC));
    defmt::println!("hclk1_hz: {}", rcc::hclk1_hz(&dp.RCC));
    defmt::println!("hclk2_hz: {}", rcc::hclk2_hz(&dp.RCC));
    defmt::println!("hclk3_hz: {}", rcc::hclk3_hz(&dp.RCC));
    defmt::println!("lsi_hz: {}", rcc::lsi_hz(&dp.RCC));

    loop {
        hal::cortex_m::asm::bkpt();
    }
}
