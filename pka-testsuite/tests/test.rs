#![no_std]
#![no_main]
#![cfg_attr(feature = "aio", feature(alloc_error_handler))]

use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    pac,
    pka::{curve::NIST_P256, EcdsaPublicKey, EcdsaSignature, Pka},
    rcc,
};

// Message hash
const HASH: [u32; 8] = [
    0x44acf6b7, 0xe36c1342, 0xc2c58972, 0x04fe0950, 0x4e1e2efb, 0x1a900377, 0xdbc4e7a6, 0xa133ec56,
];

const PRIVATE_KEY: [u32; 8] = [
    0x519b423d, 0x715f8b58, 0x1f4fa8ee, 0x59f4771a, 0x5b44c813, 0x0b4e3eac, 0xca54a56d, 0xda72b464,
];

// Note: in real-world use this should be a one-time random number (nonce).
// This fixed value is for testing purposes only.
const INTEGER: [u32; 8] = [
    0x94a1bbb1, 0x4b906a61, 0xa280f245, 0xf9e93c7f, 0x3b4a6247, 0x824f5d33, 0xb9670787, 0x642a68de,
];

const SIGNATURE: EcdsaSignature<8> = EcdsaSignature {
    r_sign: [
        0xf3ac8061, 0xb514795b, 0x8843e3d6, 0x629527ed, 0x2afd6b1f, 0x6a555a7a, 0xcabb5e6f,
        0x79c8c2ac,
    ],
    s_sign: [
        0x8bf77819, 0xca05a6b2, 0x786c7626, 0x2bf7371c, 0xef97b218, 0xe96f175a, 0x3ccdda2a,
        0xcc058903,
    ],
};

const PUB_KEY: EcdsaPublicKey<8> = EcdsaPublicKey {
    curve_pt_x: [
        0x1ccbe91c, 0x075fc7f4, 0xf033bfa2, 0x48db8fcc, 0xd3565de9, 0x4bbfb12f, 0x3c59ff46,
        0xc271bf83,
    ],
    curve_pt_y: [
        0xce4014c6, 0x8811f9a2, 0x1a1fdb2c, 0x0e6113e0, 0x6db7ca93, 0xb7404e78, 0xdc7ccd5c,
        0xa89a4ca9,
    ],
};

#[cfg(feature = "aio")]
async fn aio_ecdsa_sign_inner() {
    let mut pka: Pka = unsafe { Pka::steal() };
    let mut output_signature: EcdsaSignature<8> = EcdsaSignature {
        r_sign: [0; 8],
        s_sign: [0; 8],
    };
    pka.aio_ecdsa_sign(
        &NIST_P256,
        &INTEGER,
        &PRIVATE_KEY,
        &HASH,
        &mut output_signature,
    )
    .await
    .unwrap();
    assert_eq!(output_signature, SIGNATURE);
}

#[cfg(feature = "aio")]
async fn aio_ecdsa_verify_inner() {
    let mut pka: Pka = unsafe { Pka::steal() };
    pka.aio_ecdsa_verify(&NIST_P256, &SIGNATURE, &PUB_KEY, &HASH)
        .await
        .unwrap();
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Pka {
        let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
        let mut rcc = dp.RCC;

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut rcc);

        #[cfg(feature = "aio")]
        {
            let start: usize = stm32wl_hal::cortex_m_rt::heap_start() as usize;
            let size: usize = 2048; // in bytes
            unsafe { ate::ALLOCATOR.init(start, size) };
            unsafe { Pka::unmask_irq() };
        }

        Pka::new(dp.PKA, &mut rcc)
    }

    #[test]
    fn ecdsa_sign(pka: &mut Pka) {
        let mut output_signature: EcdsaSignature<8> = EcdsaSignature {
            r_sign: [0; 8],
            s_sign: [0; 8],
        };
        pka.ecdsa_sign(
            &NIST_P256,
            &INTEGER,
            &PRIVATE_KEY,
            &HASH,
            &mut output_signature,
        )
        .unwrap();
        assert_eq!(output_signature, SIGNATURE);
    }

    #[test]
    fn ecdsa_verify(pka: &mut Pka) {
        pka.ecdsa_verify(&NIST_P256, &SIGNATURE, &PUB_KEY, &HASH)
            .unwrap();
    }

    #[test]
    #[cfg(feature = "aio")]
    fn aio_ecdsa_sign(_pka: &mut Pka) {
        let mut executor = ate::Executor::new();
        executor.spawn(ate::Task::new(aio_ecdsa_sign_inner()));
        executor.run();
    }

    #[test]
    #[cfg(feature = "aio")]
    fn aio_ecdsa_verify(_pka: &mut Pka) {
        let mut executor = ate::Executor::new();
        executor.spawn(ate::Task::new(aio_ecdsa_verify_inner()));
        executor.run();
    }
}
