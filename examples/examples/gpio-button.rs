// Prints over RTT when the state of B3 on the NUCLEO-WL55JC2 changes.

#![no_std]
#![no_main]

use core::fmt::Write;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use rtt_target::rprintln;
use stm32wl_hal::gpio::{pac, pins, Input, Level, PortC, Pull};

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
    let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    let pc6: Input<pins::C6> = Input::new(gpioc.pc6, Pull::Up);

    let mut prev_level: Level = pc6.level();
    rprintln!("B3 initial level: {:?}", prev_level);

    loop {
        let level: Level = pc6.level();
        if level != prev_level {
            rprintln!("B3 state changed from {:?} to {:?}", prev_level, level);
            prev_level = level;
        }
    }
}
