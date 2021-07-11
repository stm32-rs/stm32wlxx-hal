// Blinks the 3 LEDs on the NUCLEO-WL55JC2 in a sequence.

#![no_std]
#![no_main]

use panic_rtt_target as _;
use rtt_target::rprintln;
use stm32wl_hal::{
    gpio::{Level, Output, PortB},
    pac,
};

// Note: This is a bad delay function because it will not be consistent.
fn wait_a_little() {
    for _ in 0..6000 {
        cortex_m::asm::nop()
    }
}

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

    let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
    let mut led1 = Output::default(gpiob.pb9);
    let mut led2 = Output::default(gpiob.pb15);
    let mut led3 = Output::default(gpiob.pb11);

    rprintln!("Starting blinky");

    loop {
        for &level in &[Level::High, Level::Low] {
            led1.set_output_level(level);
            wait_a_little();
            led2.set_output_level(level);
            wait_a_little();
            led3.set_output_level(level);
            wait_a_little();
        }
    }
}
