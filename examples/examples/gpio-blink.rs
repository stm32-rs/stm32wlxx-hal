// Blinks the 3 LEDs on the NUCLEO-WL55JC2 in a sequence.

#![no_std]
#![no_main]

use core::fmt::Write;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use rtt_target::rprintln;
use stm32wl_hal::{
    gpio::{Level, Output, PortB},
    pac,
};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    if let Some(mut channel) = unsafe { rtt_target::UpChannel::conjure(0) } {
        channel.set_mode(rtt_target::ChannelMode::BlockIfFull);

        writeln!(channel, "{}", info).ok();
    }

    loop {
        compiler_fence(SeqCst);
    }
}

// Note: This is a bad delay function because it will not be consistent.
fn wait_a_little() {
    for _ in 0..2000 {
        cortex_m::asm::nop()
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 4096
                mode: BlockIfFull
                name: "Terminal"
            }
        }
    };

    writeln!(&mut channels.up.0, "Hello from writeln!").ok();

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
