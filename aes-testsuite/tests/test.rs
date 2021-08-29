#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{aes::Aes, cortex_m::peripheral::DWT, pac, rcc, util::reset_cycle_count};

const FREQ: u32 = 48_000_000;
const CYC_PER_US: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:µs}", DWT::get_cycle_count() / CYC_PER_US);

pub const fn u128_to_u32(u: u128) -> [u32; 4] {
    [
        (u >> 96) as u32,
        (u >> 64) as u32,
        (u >> 32) as u32,
        u as u32,
    ]
}

// NIST AESAVS Nov 15 2002 B.1
const PLAINTEXT_CHIPHERTEXT: [(u128, u128); 7] = [
    (
        0xf34481ec3cc627bacd5dc3fb08f273e6,
        0x0336763e966d92595a567cc9ce537f5e,
    ),
    (
        0x9798c4640bad75c7c3227db910174e72,
        0xa9a1631bf4996954ebc093957b234589,
    ),
    (
        0x96ab5c2ff612d9dfaae8c31f30c42168,
        0xff4f8391a6a40ca5b25d23bedd44a597,
    ),
    (
        0x6a118a874519e64e9963798a503f1d35,
        0xdc43be40be0e53712f7e2bf5ca707209,
    ),
    (
        0xcb9fceec81286ca3e989bd979b0cb284,
        0x92beedab1895a94faa69b632e5cc47ce,
    ),
    (
        0xb26aeb1874e47ca8358ff22378f09144,
        0x459264f4798f6a78bacb89c15ed3d601,
    ),
    (
        0x58c8e00b2631686d54eab84b91f0aca1,
        0x08a4e2efec8a8e3312ca7460b9040bbf,
    ),
];

const KEY: [u32; 4] = [0; 4];

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Aes {
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());

        unsafe { rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC) };
        defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);

        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        reset_cycle_count(&mut cp.DWT);

        Aes::new(dp.AES, &mut dp.RCC)
    }

    #[test]
    fn encrypt_ecb(aes: &mut Aes) {
        let mut total_elapsed: usize = 0;
        for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
            let mut output_ciphertext: [u32; 4] = [0; 4];
            let start: u32 = DWT::get_cycle_count();
            unwrap!(aes.encrypt_ecb(&KEY, &u128_to_u32(*plaintext), &mut output_ciphertext));
            let end: u32 = DWT::get_cycle_count();
            total_elapsed += (end - start) as usize;
            let expected: [u32; 4] = u128_to_u32(*ciphertext);
            defmt::assert_eq!(output_ciphertext, expected);
        }
        defmt::info!(
            "Average cycles per encrypt: {}",
            total_elapsed / PLAINTEXT_CHIPHERTEXT.len()
        );
    }

    #[test]
    fn encrypt_ecb_inplace(aes: &mut Aes) {
        let mut total_elapsed: usize = 0;
        for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
            let mut output_ciphertext: [u32; 4] = u128_to_u32(*plaintext);
            let start: u32 = DWT::get_cycle_count();
            unwrap!(aes.encrypt_ecb_inplace(&KEY, &mut output_ciphertext));
            let end: u32 = DWT::get_cycle_count();
            total_elapsed += (end - start) as usize;
            let expected: [u32; 4] = u128_to_u32(*ciphertext);
            defmt::assert_eq!(output_ciphertext, expected);
        }
        defmt::info!(
            "Average cycles per encrypt: {}",
            total_elapsed / PLAINTEXT_CHIPHERTEXT.len()
        );
    }

    #[test]
    fn decrypt_ecb(aes: &mut Aes) {
        let mut total_elapsed: usize = 0;
        for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
            let mut output_plaintext: [u32; 4] = [0; 4];
            let start: u32 = DWT::get_cycle_count();
            unwrap!(aes.decrypt_ecb(&KEY, &u128_to_u32(*ciphertext), &mut output_plaintext));
            let end: u32 = DWT::get_cycle_count();
            total_elapsed += (end - start) as usize;
            let expected: [u32; 4] = u128_to_u32(*plaintext);
            defmt::assert_eq!(output_plaintext, expected);
        }
        defmt::info!(
            "Average cycles per decrypt: {}",
            total_elapsed / PLAINTEXT_CHIPHERTEXT.len()
        );
    }

    #[test]
    fn decrypt_ecb_inplace(aes: &mut Aes) {
        let mut total_elapsed: usize = 0;
        for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
            let mut output_plaintext: [u32; 4] = u128_to_u32(*ciphertext);
            let start: u32 = DWT::get_cycle_count();
            unwrap!(aes.decrypt_ecb_inplace(&KEY, &mut output_plaintext));
            let end: u32 = DWT::get_cycle_count();
            total_elapsed += (end - start) as usize;
            let expected: [u32; 4] = u128_to_u32(*plaintext);
            defmt::assert_eq!(output_plaintext, expected);
        }
        defmt::info!(
            "Average cycles per decrypt: {}",
            total_elapsed / PLAINTEXT_CHIPHERTEXT.len()
        );
    }
}
