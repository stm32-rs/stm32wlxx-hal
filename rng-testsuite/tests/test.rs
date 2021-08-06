#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    pac, rcc,
    rng::{rand_core::RngCore, ClkSrc, Rng},
};

#[cfg(feature = "aio")]
async fn aio_random_enough_for_me_inner() {
    let mut rng: Rng = unsafe { Rng::steal() };
    let mut bytes: [u8; 33] = [0; 33];
    unwrap!(rng.aio_try_fill_u8(&mut bytes).await);
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
    use super::*;

    #[init]
    fn init() -> Rng {
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
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

    /// Not really a test, just a benchmark
    #[test]
    fn cha_cha(rng: &mut Rng) {
        defmt::warn!("Test results are only valid for release mode");
        use rand_chacha::{rand_core::SeedableRng, ChaCha12Rng, ChaCha20Rng, ChaCha8Rng};

        let mut seed: [u8; 32] = [0; 32];

        unwrap!(rng.try_fill_u8(&mut seed));
        let mut cha20: ChaCha20Rng = ChaCha20Rng::from_seed([0u8; 32]);
        unwrap!(rng.try_fill_u8(&mut seed));
        let mut cha12: ChaCha12Rng = ChaCha12Rng::from_seed([0u8; 32]);
        unwrap!(rng.try_fill_u8(&mut seed));
        let mut cha8: ChaCha8Rng = ChaCha8Rng::from_seed([0u8; 32]);

        let mut cp = unwrap!(pac::CorePeripherals::take());
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();

        const NUM_BLK: usize = 200;
        const BUF_SIZE: usize = NUM_BLK * 4 * 4;
        static mut BUF: [u8; BUF_SIZE] = [0; BUF_SIZE];

        let start: u32 = pac::DWT::get_cycle_count();
        let ret = cha20.try_fill_bytes(unsafe { &mut BUF });
        let end: u32 = pac::DWT::get_cycle_count();
        unwrap!(ret);
        let cha20cyc: u32 = end.wrapping_sub(start);

        let start: u32 = pac::DWT::get_cycle_count();
        let ret = cha12.try_fill_bytes(unsafe { &mut BUF });
        let end: u32 = pac::DWT::get_cycle_count();
        unwrap!(ret);
        let cha12cyc: u32 = end.wrapping_sub(start);

        let start: u32 = pac::DWT::get_cycle_count();
        let ret = cha8.try_fill_bytes(unsafe { &mut BUF });
        let end: u32 = pac::DWT::get_cycle_count();
        unwrap!(ret);
        let cha8cyc: u32 = end.wrapping_sub(start);

        let start: u32 = pac::DWT::get_cycle_count();
        let ret = rng.try_fill_bytes(unsafe { &mut BUF });
        let end: u32 = pac::DWT::get_cycle_count();
        unwrap!(ret);
        let rngcyc: u32 = end.wrapping_sub(start);

        defmt::info!("ChaCha20: {}", (cha20cyc as f32) / (NUM_BLK as f32));
        defmt::info!("ChaCha12: {}", (cha12cyc as f32) / (NUM_BLK as f32));
        defmt::info!("ChaCha8: {}", (cha8cyc as f32) / (NUM_BLK as f32));
        defmt::info!("HW: {}", (rngcyc as f32) / (NUM_BLK as f32));
    }

    #[test]
    fn random_enough_for_me(rng: &mut Rng) {
        let mut bytes: [u8; 35] = [0; 35];
        unwrap!(rng.try_fill_bytes(&mut bytes));
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
