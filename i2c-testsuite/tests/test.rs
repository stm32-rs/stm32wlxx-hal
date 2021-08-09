#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use embedded_time::rate::Hertz;
use panic_probe as _;
use stm32wl_hal::{
    embedded_hal::blocking::i2c::WriteRead,
    gpio::{pins, PortB},
    i2c::I2c1,
    pac, rcc,
};

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> I2c1<(pins::B8, pins::B7)> {
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC);
        let gpiob = PortB::split(dp.GPIOB, &mut dp.RCC);
        I2c1::new(
            dp.I2C1,
            (gpiob.pb8, gpiob.pb7),
            Hertz(100_000),
            &mut dp.RCC,
            false,
        )
    }

    #[test]
    fn sht31_measurement(i2c: &mut I2c1<(pins::B8, pins::B7)>) {
        defmt::warn!("A SHT31 sensor must be connected to the board on pins PB8 (SCL) & PB7 (SDA) for this test to work");
        let cmd: [u8; 2] = [0x2C, 0x06];
        let mut response: [u8; 6] = [0; 6];

        let result = i2c.write_read(0x44, &cmd, &mut response);
        match result {
            Ok(()) => defmt::info!("Bytes received: {:x}", response),
            Err(e) => defmt::error!("I2C error: {}", e),
        }
    }
}
