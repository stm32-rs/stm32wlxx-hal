// Prints device information, should work for all boards.

#![no_std]
#![no_main]

use panic_rtt_target as _;
use rtt_target::rprintln;
use stm32wl_hal::info;

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

    rprintln!("Flash size: {} KiB", info::flash_size_kibibyte());
    rprintln!("Package: {:?}", info::package());
    rprintln!("UID64: {}", info::uid64());

    rprintln!("Exiting");

    loop {
        cortex_m::asm::bkpt();
    }
}
