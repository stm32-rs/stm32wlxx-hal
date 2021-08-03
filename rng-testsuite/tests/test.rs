#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    pac, rcc,
    rng::{rand_core::RngCore, Rng},
};

#[cfg(feature = "aio")]
async fn aio_random_enough_for_me_inner() {
    let mut rng: Rng = unsafe { Rng::steal() };
    let mut bytes: [u8; 33] = [0; 33];
    rng.try_fill_bytes(&mut bytes).unwrap();
    validate_randomness(&bytes)
}

/// This is not a cryptographically secure validation, this only ensures that
/// the driver is operating nominally, is does not check for hardware
/// correctness.
fn validate_randomness(entropy: &[u8]) {
    let mut sum: f64 = 0.0;
    for byte in entropy.iter() {
        sum += *byte as f64;
    }

    sum /= entropy.len() as f64;

    const MID: f64 = 127.5;
    const TOLERANCE: f64 = MID / 2.0;
    const LO: f64 = MID - TOLERANCE;
    const HI: f64 = MID + TOLERANCE;

    defmt::assert!(sum > LO && sum < HI, "{:?}", sum);
}

#[defmt_test::tests]
mod tests {
    use stm32wl_hal::rng::ClkSrc;

    use super::*;

    #[init]
    fn init() -> Rng {
        let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
        let mut rcc = dp.RCC;

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut rcc);

        #[cfg(feature = "aio")]
        {
            let start: usize = stm32wl_hal::cortex_m_rt::heap_start() as usize;
            let size: usize = 2048; // in bytes
            unsafe { ate::ALLOCATOR.init(start, size) };
            unsafe { Rng::unmask_irq() };
        }

        Rng::set_clock_source(&mut rcc, ClkSrc::MSI);
        Rng::new(dp.RNG, &mut rcc)
    }

    #[test]
    fn random_enough_for_me(rng: &mut Rng) {
        let mut bytes: [u8; 35] = [0; 35];
        rng.try_fill_bytes(&mut bytes).unwrap();
        validate_randomness(&bytes)
    }

    #[test]
    #[cfg(feature = "aio")]
    fn aio_random_enough_for_me(_rng: &mut Rng) {
        let mut executor = ate::Executor::new();
        executor.spawn(ate::Task::new(aio_random_enough_for_me_inner()));
        executor.run();
    }
}
