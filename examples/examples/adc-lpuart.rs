// Adc sample result send to lpuart, should work for all STM32WL5x boards.

#![no_std]
#![no_main]

use defmt_rtt as _;     // global logger
use panic_probe as _;   // panic handler
use defmt::unwrap;
use core::fmt::Write;
use stm32wlxx_hal::{
    self as hal,
    info::{self, Package, Uid, Uid64},
    adc::{self, Adc, Clk},
    cortex_m::{self},
    pac,
    pwr::enable_shutdown_sleeponexit,
    rcc,
    gpio::{pins, PortA},
    uart::{self, LpUart},
    util::new_delay,
};

const ADC_FREQ: u32 = 12_000_000;
const FREQ: u32 = 48_000_000;

#[hal::cortex_m_rt::entry]
fn main() -> ! {
    // info
    defmt::println!("Flash size: {} KiB", info::flash_size_kibibyte());
    defmt::println!("Package: {:?}", Package::from_device());
    defmt::println!("UID64: {}", Uid64::from_device());
    defmt::println!("UID: {}", Uid::from_device());

    // periph
    let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
    let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());
    cortex_m::interrupt::free(|cs| unsafe {
        rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs);
    });
    defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);
    let mut delay = new_delay(cp.SYST, &dp.RCC);
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
    cp.DWT.set_cycle_count(0);

    // adc
    let mut adc: Adc = Adc::new(dp.ADC, Clk::PClkDiv4, &mut dp.RCC);
    defmt::assert_eq!(adc.clock_hz(&dp.RCC), ADC_FREQ);
    adc.disable();
    adc.calibrate(&mut delay);
    adc.enable();
    adc.enable_vbat();
    adc.set_max_sample_time();
    let sample: u16 = adc.vbat();
    let vbat = (3.3 * f32::from(sample) * 3.0) / 4096.0;
    defmt::info!("sample {} vbat {}", sample, vbat);

    adc.enable_tsen();
    delay.delay_us(adc::TS_START_MAX.as_micros() as u32);
    adc.set_max_sample_time();
    let temp: i16 = adc.temperature().to_integer();
    defmt::info!("Temperature: {} °C", temp);
    
    // lpuart
    dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
    while dp.RCC.cr.read().hsirdy().is_not_ready() {}
    let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
    let mut lpuart: LpUart<pins::A3, pins::A2> = cortex_m::interrupt::free(|cs| {
        LpUart::new(dp.LPUART, 115200, uart::Clk::Hsi16, &mut dp.RCC).enable_rx(gpioa.a3, cs).enable_tx(gpioa.a2, cs)
    });
    unwrap!(write!(&mut lpuart, "vbat {} temperature {}\r\n", vbat, temp).ok());

    loop {
        enable_shutdown_sleeponexit(&mut dp.PWR, &mut cp.SCB);
    }
}
