// Prints device information, should work for all boards.

#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _; // panic handler
use stm32wl_hal::{self as hal, info};

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Flash size: {} KiB", info::flash_size_kibibyte());
    defmt::info!("Package: {:?}", info::package());
    defmt::info!("UID64: {}", info::uid64());
    defmt::info!("UID: {}", info::uid());

    loop {
        hal::cortex_m::asm::bkpt();
    }
}
