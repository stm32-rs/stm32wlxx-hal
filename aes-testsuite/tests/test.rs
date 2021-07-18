#![no_std]
#![no_main]
#![cfg_attr(feature = "aio", feature(alloc_error_handler))]

use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    aes::{Aes, Key, Key128},
    pac, rcc,
};

#[cfg(feature = "aio")]
use ate::alloc_cortex_m::CortexMHeap;

#[global_allocator]
#[cfg(feature = "aio")]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

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

const KEY: Key = Key::K128(Key128::from_u128(0));

#[cfg_attr(feature = "aio", alloc_error_handler)]
#[cfg(feature = "aio")]
fn oom(_layout: core::alloc::Layout) -> ! {
    cortex_m::interrupt::disable();

    let dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
    let mut rcc: pac::RCC = dp.RCC;

    use stm32wl_hal::gpio;
    let gpiob: gpio::PortB = gpio::PortB::split(dp.GPIOB, &mut rcc);
    let mut led1 = gpio::Output::default(gpiob.pb9);
    let mut led2 = gpio::Output::default(gpiob.pb15);
    let mut led3 = gpio::Output::default(gpiob.pb11);

    led1.set_level_high();
    led2.set_level_high();
    led3.set_level_high();

    use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
    loop {
        compiler_fence(SeqCst);
    }
}

#[cfg(feature = "aio")]
async fn aio_decrypt_ecb_inner() {
    let mut aes: Aes = unsafe { Aes::steal() };
    for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
        let result: [u32; 4] = aes
            .aio_decrypt_ecb(&KEY, &u128_to_u32(*ciphertext))
            .await
            .unwrap();
        let plaintext_u32: [u32; 4] = u128_to_u32(*plaintext);
        assert!(
            result == plaintext_u32,
            "\nResult:   {:08X?}\nExpected: {:08X?}\n",
            result,
            plaintext_u32
        );
    }
}

#[cfg(feature = "aio")]
async fn aio_encrypt_ecb_inner() {
    let mut aes: Aes = unsafe { Aes::steal() };
    for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
        let result: [u32; 4] = aes
            .aio_encrypt_ecb(&KEY, &u128_to_u32(*plaintext))
            .await
            .unwrap();
        let ciphertext_u32: [u32; 4] = u128_to_u32(*ciphertext);
        assert!(
            result == ciphertext_u32,
            "\nResult:   {:08X?}\nExpected: {:08X?}\n",
            result,
            ciphertext_u32
        );
    }
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Aes {
        let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
        let mut rcc = dp.RCC;

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut rcc);

        #[cfg(feature = "aio")]
        {
            let start: usize = cortex_m_rt::heap_start() as usize;
            let size: usize = 2048; // in bytes
            unsafe { ALLOCATOR.init(start, size) };
            unsafe { Aes::unmask_irq() };
        }

        Aes::new(dp.AES, &mut rcc)
    }

    #[test]
    fn encrypt_ecb(aes: &mut Aes) {
        for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
            let result: [u32; 4] = aes.encrypt_ecb(&KEY, &u128_to_u32(*plaintext)).unwrap();
            let ciphertext_u32: [u32; 4] = u128_to_u32(*ciphertext);
            assert!(
                result == ciphertext_u32,
                "\nResult:   {:08X?}\nExpected: {:08X?}\n",
                result,
                ciphertext_u32
            );
        }
    }

    #[test]
    fn decrypt_ecb(aes: &mut Aes) {
        for (plaintext, ciphertext) in PLAINTEXT_CHIPHERTEXT.iter() {
            let result: [u32; 4] = aes.decrypt_ecb(&KEY, &u128_to_u32(*ciphertext)).unwrap();
            let plaintext_u32: [u32; 4] = u128_to_u32(*plaintext);
            assert!(
                result == plaintext_u32,
                "\nResult:   {:08X?}\nExpected: {:08X?}\n",
                result,
                plaintext_u32
            );
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
