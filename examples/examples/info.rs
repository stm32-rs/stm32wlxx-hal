// Prints device information, should work for all STM32WL5x boards.

#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _; // panic handler
use stm32wlxx_hal::{
    self as hal,
    info::{self, Package, Uid, Uid64},
};

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Flash size: {} KiB", info::flash_size_kibibyte());
    defmt::println!("Package: {:?}", Package::from_device());
    defmt::println!("UID64: {}", Uid64::from_device());
    defmt::println!("UID: {}", Uid::from_device());

    loop {
        hal::cortex_m::asm::bkpt();
    }
}
