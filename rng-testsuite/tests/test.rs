#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    pac, rcc,
    rng::{rand_core::RngCore, Rng},
};

#[defmt_test::tests]
mod tests {
    use stm32wl_hal::rng::ClkSrc;

    use super::*;

    #[init]
    fn init() -> Rng {
        let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
        let mut rcc = dp.RCC;

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut rcc);

        Rng::set_clock_source(&mut rcc, ClkSrc::Msi);
        Rng::new(dp.RNG, &mut rcc)
    }

    /// This is not a cryptographically secure test, this only ensures that the
    /// driver is operating nominally, is does not check for hardware
    /// correctness.
    #[test]
    fn random_enough_for_me(rng: &mut Rng) {
        let mut bytes: [u8; 32] = [0; 32];
        rng.try_fill_bytes(&mut bytes).unwrap();

        let mut sum: f64 = 0.0;
        for byte in bytes.iter() {
            sum += *byte as f64;
        }

        sum /= bytes.len() as f64;

        const MID: f64 = 127.5;
        const TOLERANCE: f64 = 15.0;
        const LO: f64 = MID - TOLERANCE;
        const HI: f64 = MID + TOLERANCE;

        defmt::assert!(sum > LO && sum < HI, "{:?}", sum);
    }
}
