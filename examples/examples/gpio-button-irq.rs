// Prints over RTT when the state of B3 on the NUCLEO-WL55JC2 changes using interrupts.

#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _; // panic handler
use stm32wl_hal::{
    self as hal,
    gpio::{pins, Exti, ExtiTrg, Input, PortC, Pull},
    pac::{self, interrupt},
};

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    let mut dp: pac::Peripherals = defmt::unwrap!(pac::Peripherals::take());

    let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
    let _c6: Input<pins::C6> = Input::new(gpioc.c6, Pull::Up);

    pins::C6::setup_exti_c1(&mut dp.EXTI, &mut dp.SYSCFG, ExtiTrg::Both);
    unsafe { pins::C6::unmask() };

    loop {
        hal::cortex_m::asm::wfe();
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn EXTI9_5() {
    defmt::info!("B3 pressed or released!");
    pins::C6::clear_exti();
}
