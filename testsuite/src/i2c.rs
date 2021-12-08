#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use nucleo_wl55jc_bsp::hal::{
    cortex_m,
    embedded_hal::blocking::i2c::WriteRead,
    gpio::{pins, PortA, PortB},
    i2c::{I2c1, I2c2},
    pac::{self, interrupt},
    rcc,
};
use panic_probe as _;

const LOOPBACK_ADDR: u8 = 0x77;
const LOOPBACK_DATA_IN: u8 = 0xD0;
const LOOPBACK_DATA_OUT: u8 = 0xA5;
const I2C_FREQUENCY: u32 = 100_000;

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> I2c1<(pins::B8, pins::B7)> {
        cortex_m::interrupt::free(|cs| {
            let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());

            unsafe { rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs) };
            let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
            let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);

            // initialize I2C2 clocks and pins
            let i2c2 = I2c2::new(
                dp.I2C2,
                (gpioa.a12, gpioa.a11),
                I2C_FREQUENCY,
                &mut dp.RCC,
                true,
                cs,
            );
            let (i2c2, _) = i2c2.free();

            // disable i2c2 to reconfigure for secondary mode
            i2c2.cr1.modify(|_, w| w.pe().disabled());

            // set the i2c2 secondary address to 0x77
            i2c2.oar1.write(|w| {
                w.oa1en()
                    .enabled()
                    .oa1mode()
                    .bit7()
                    .oa1()
                    .bits((LOOPBACK_ADDR << 1) as u16)
            });
            i2c2.cr1.modify(|_, w| {
                w.gcen().set_bit(); // general call enable
                w.errie().enabled(); // enable error IRQs
                w.addrie().enabled(); // secondary address match IRQ
                w.pe().enabled() // re-enable peripheral
            });

            unsafe {
                pac::NVIC::unmask(pac::Interrupt::I2C2_EV);
                pac::NVIC::unmask(pac::Interrupt::I2C2_ER);
            }

            I2c1::new(
                dp.I2C1,
                (gpiob.b8, gpiob.b7),
                I2C_FREQUENCY,
                &mut dp.RCC,
                false,
                cs,
            )
        })
    }

    #[test]
    fn sht31_measurement(i2c: &mut I2c1<(pins::B8, pins::B7)>) {
        defmt::warn!("A SHT31 sensor must be connected to the board on pins B8 (SCL) & B7 (SDA) for this test to work");
        let cmd: [u8; 2] = [0x2C, 0x06];
        let mut response: [u8; 6] = [0; 6];

        let result = i2c.write_read(0x44, &cmd, &mut response);
        match result {
            Ok(()) => defmt::info!("Bytes received: {:x}", response),
            Err(e) => defmt::error!("I2C error: {}", e),
        }
    }

    #[test]
    fn loopback(i2c: &mut I2c1<(pins::B8, pins::B7)>) {
        defmt::warn!("I2C1 pins B8 (SCL) and B7 (SDA) must be connected to I2C pins A12 (SCL) and A11 (SDA) for this test to pass");

        let cmd: [u8; 1] = [LOOPBACK_DATA_IN];
        let mut response: [u8; 1] = [0; 1];

        let result = i2c.write_read(LOOPBACK_ADDR, &cmd, &mut response);
        match result {
            Ok(()) => defmt::assert_eq!(LOOPBACK_DATA_OUT, response[0]),
            Err(e) => {
                defmt::panic!("I2C error: {}", e);
            }
        }
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn I2C2_EV() {
    let dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
    let isr = dp.I2C2.isr.read();
    defmt::debug!("I2C2 ISR={:#08X}", isr.bits());

    if isr.rxne().is_not_empty() {
        let rxdr: u8 = dp.I2C2.rxdr.read().rxdata().bits();
        defmt::assert_eq!(rxdr, LOOPBACK_DATA_IN);
    }

    if isr.addr().is_match() {
        dp.I2C2.txdr.write(|w| w.txdata().bits(LOOPBACK_DATA_OUT));
        dp.I2C2.icr.write(|w| w.addrcf().set_bit());
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn I2C2_ER() {
    cortex_m::interrupt::disable();
    let dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
    defmt::panic!("I2C2 error ISR={:#08X}", dp.I2C2.isr.read().bits());
}
