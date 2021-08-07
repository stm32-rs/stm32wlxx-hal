#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{aes::Aes, pac, rcc};

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

#[cfg(feature = "aio")]
async fn aio_decrypt_ecb_inner() {
    let mut aes: Aes = unsafe { Aes::steal() };
    for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
        let result: [u32; 4] = unwrap!(aes.aio_decrypt_ecb(&KEY, &u128_to_u32(*ciphertext)).await);
        let plaintext_u32: [u32; 4] = u128_to_u32(*plaintext);
        defmt::assert_eq!(result, plaintext_u32);
    }
}

#[cfg(feature = "aio")]
async fn aio_encrypt_ecb_inner() {
    let mut aes: Aes = unsafe { Aes::steal() };
    for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
        let result: [u32; 4] = unwrap!(aes.aio_encrypt_ecb(&KEY, &u128_to_u32(*plaintext)).await);
        let ciphertext_u32: [u32; 4] = u128_to_u32(*ciphertext);
        defmt::assert_eq!(result, ciphertext_u32);
    }
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Aes {
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
        let mut rcc = dp.RCC;

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut rcc);

        #[cfg(feature = "aio")]
        {
            let start: usize = stm32wl_hal::cortex_m_rt::heap_start() as usize;
            let size: usize = 2048; // in bytes
            unsafe { ate::ALLOCATOR.init(start, size) };
            unsafe { Aes::unmask_irq() };
        }

        Aes::new(dp.AES, &mut rcc)
    }

    #[test]
    fn encrypt_ecb(aes: &mut Aes) {
        for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
            let result: [u32; 4] = unwrap!(aes.encrypt_ecb(&KEY, &u128_to_u32(*plaintext)));
            let ciphertext_u32: [u32; 4] = u128_to_u32(*ciphertext);
            defmt::assert_eq!(result, ciphertext_u32);
        }
    }

    #[test]
    fn decrypt_ecb(aes: &mut Aes) {
        for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
            let result: [u32; 4] = unwrap!(aes.decrypt_ecb(&KEY, &u128_to_u32(*ciphertext)));
            let plaintext_u32: [u32; 4] = u128_to_u32(*plaintext);
            defmt::assert_eq!(result, plaintext_u32);
        }
    }

    #[test]
    #[cfg(feature = "aio")]
    fn aio_encrypt_ecb(_aes: &mut Aes) {
        let mut executor = ate::Executor::new();
        executor.spawn(ate::Task::new(aio_encrypt_ecb_inner()));
        executor.run();
    }

    #[test]
    #[cfg(feature = "aio")]
    fn aio_decrypt_ecb(_aes: &mut Aes) {
        let mut executor = ate::Executor::new();
        executor.spawn(ate::Task::new(aio_decrypt_ecb_inner()));
        executor.run();
    }
}
