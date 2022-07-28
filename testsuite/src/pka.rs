#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use nucleo_wl55jc_bsp::hal::{
    cortex_m,
    pac::{self, DWT},
    pka::{
        curve::NIST_P256, EcdsaPublicKey, EcdsaSignError, EcdsaSignature, EcdsaVerifyError, Pka,
    },
    rcc,
};
use panic_probe as _;

const FREQ: u32 = 48_000_000;
const CYC_PER_MICRO: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:us}", DWT::cycle_count() / CYC_PER_MICRO);

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

const R_SIGN: [u32; 8] = [
    0xf3ac8061, 0xb514795b, 0x8843e3d6, 0x629527ed, 0x2afd6b1f, 0x6a555a7a, 0xcabb5e6f, 0x79c8c2ac,
];
const S_SIGN: [u32; 8] = [
    0x8bf77819, 0xca05a6b2, 0x786c7626, 0x2bf7371c, 0xef97b218, 0xe96f175a, 0x3ccdda2a, 0xcc058903,
];

const CURVE_PT_X: [u32; 8] = [
    0x1ccbe91c, 0x075fc7f4, 0xf033bfa2, 0x48db8fcc, 0xd3565de9, 0x4bbfb12f, 0x3c59ff46, 0xc271bf83,
];

const CURVE_PT_Y: [u32; 8] = [
    0xce4014c6, 0x8811f9a2, 0x1a1fdb2c, 0x0e6113e0, 0x6db7ca93, 0xb7404e78, 0xdc7ccd5c, 0xa89a4ca9,
];

const SIGNATURE: EcdsaSignature<8> = EcdsaSignature {
    r_sign: &R_SIGN,
    s_sign: &S_SIGN,
};

const PUB_KEY: EcdsaPublicKey<8> = EcdsaPublicKey {
    curve_pt_x: &CURVE_PT_X,
    curve_pt_y: &CURVE_PT_Y,
};

const fn swap32(i: [u32; 8]) -> [u32; 8] {
    [
        i[0].swap_bytes(),
        i[1].swap_bytes(),
        i[2].swap_bytes(),
        i[3].swap_bytes(),
        i[4].swap_bytes(),
        i[5].swap_bytes(),
        i[6].swap_bytes(),
        i[7].swap_bytes(),
    ]
}

fn into_bytes(i: [u32; 8]) -> [u8; 32] {
    unsafe { core::mem::transmute::<[u32; 8], [u8; 32]>(i) }
}

// for use with rust-crypto
const HASH_SWAP: [u32; 8] = swap32(HASH);
const INTEGER_SWAP: [u32; 8] = swap32(INTEGER);
const PRIVATE_KEY_SWAP: [u32; 8] = swap32(PRIVATE_KEY);
const R_SIGN_SWAP: [u32; 8] = swap32(R_SIGN);
const S_SIGN_SWAP: [u32; 8] = swap32(S_SIGN);
const CURVE_PT_X_SWAP: [u32; 8] = swap32(CURVE_PT_X);
const CURVE_PT_Y_SWAP: [u32; 8] = swap32(CURVE_PT_Y);

#[inline(always)]
fn stopwatch<F>(f: F) -> u32
where
    F: FnOnce() -> (),
{
    let start: u32 = DWT::cycle_count();
    f();
    let end: u32 = DWT::cycle_count();
    end.wrapping_sub(start)
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Pka {
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());

        cortex_m::interrupt::free(|cs| unsafe {
            rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs)
        });

        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        cp.DWT.set_cycle_count(0);

        Pka::new(dp.PKA, &mut dp.RCC)
    }

    #[test]
    fn ecdsa_sign(pka: &mut Pka) {
        {
            let mut r_sign: [u32; 8] = [0; 8];
            let mut s_sign: [u32; 8] = [0; 8];
            let elapsed: u32 = stopwatch(|| {
                unwrap!(pka.ecdsa_sign(
                    &NIST_P256,
                    &INTEGER,
                    &PRIVATE_KEY,
                    &HASH,
                    &mut r_sign,
                    &mut s_sign,
                ))
            });
            defmt::assert_eq!(r_sign, R_SIGN);
            defmt::assert_eq!(s_sign, S_SIGN);

            defmt::info!("Approximate cycles per PKA p256 sign: {}", elapsed);
        }

        // rust-crypto p256 for comparsion
        {
            use ecdsa::hazmat::SignPrimitive;
            use p256::{elliptic_curve::ops::Reduce, Scalar};

            let hash_bytes: [u8; 32] = into_bytes(HASH_SWAP);
            let private_key_bytes: [u8; 32] = into_bytes(PRIVATE_KEY_SWAP);
            let integer_bytes: [u8; 32] = into_bytes(INTEGER_SWAP);

            let prehashed_message_as_scalar: Scalar =
                Scalar::from_be_bytes_reduced(hash_bytes.into());
            let static_scalar: Scalar = Scalar::from_be_bytes_reduced(private_key_bytes.into());

            let ephemeral_secret =
                defmt::unwrap!(p256::SecretKey::from_be_bytes(&integer_bytes).ok());
            let ephemeral_scalar: Scalar =
                Scalar::from_be_bytes_reduced(ephemeral_secret.to_be_bytes());

            let start: u32 = DWT::cycle_count();
            let (signature, _) = unwrap!(static_scalar
                .try_sign_prehashed(ephemeral_scalar, prehashed_message_as_scalar.to_bytes())
                .ok());
            let elapsed: u32 = DWT::cycle_count().wrapping_sub(start);
            defmt::info!("Approximate cycles per rust-crypto p256 sign: {}", elapsed);

            let (r, s) = signature.split_bytes();
            defmt::assert_eq!(r.as_slice(), into_bytes(R_SIGN_SWAP));
            defmt::assert_eq!(s.as_slice(), into_bytes(S_SIGN_SWAP));
        }
    }

    #[test]
    fn ecdsa_sign_nb(pka: &mut Pka) {
        unwrap!(pka.ecdsa_sign_start(&NIST_P256, &INTEGER, &PRIVATE_KEY, &HASH));
        let mut r_sign: [u32; 8] = [0; 8];
        let mut s_sign: [u32; 8] = [0; 8];
        unwrap!(nb::block!(pka.ecdsa_sign_result(&mut r_sign, &mut s_sign)));
        defmt::assert_eq!(r_sign, R_SIGN);
        defmt::assert_eq!(s_sign, S_SIGN);
    }

    #[test]
    fn ecdsa_sign_ram_error(pka: &mut Pka) {
        unwrap!(pka.ecdsa_verify_start(&NIST_P256, &SIGNATURE, &PUB_KEY, &HASH));
        let mut r_sign: [u32; 8] = [0; 8];
        let mut s_sign: [u32; 8] = [0; 8];
        defmt::assert_eq!(
            pka.ecdsa_sign(
                &NIST_P256,
                &INTEGER,
                &PRIVATE_KEY,
                &HASH,
                &mut r_sign,
                &mut s_sign,
            ),
            Err(EcdsaSignError::Ram)
        );
        unwrap!(nb::block!(pka.ecdsa_verify_result()));
    }

    #[test]
    fn ecdsa_sign_mode_error(pka: &mut Pka) {
        unwrap!(pka.ecdsa_verify_start(&NIST_P256, &SIGNATURE, &PUB_KEY, &HASH));
        let mut r_sign: [u32; 8] = [0; 8];
        let mut s_sign: [u32; 8] = [0; 8];
        defmt::assert_eq!(
            nb::block!(pka.ecdsa_sign_result(&mut r_sign, &mut s_sign)),
            Err(EcdsaSignError::Mode { mode: 0b100110 })
        );
        unwrap!(nb::block!(pka.ecdsa_verify_result()));
    }

    #[test]
    fn ecdsa_verify(pka: &mut Pka) {
        {
            let elapsed: u32 =
                stopwatch(|| unwrap!(pka.ecdsa_verify(&NIST_P256, &SIGNATURE, &PUB_KEY, &HASH)));

            defmt::info!("Approximate cycles per PKA p256 verify: {}", elapsed);
        }

        // rust-crypto p256 for comparsion
        {
            use ecdsa::{hazmat::VerifyPrimitive, Signature};
            use p256::{elliptic_curve::ops::Reduce, PublicKey, Scalar};

            let hash_bytes: [u8; 32] = into_bytes(HASH_SWAP);

            let mut key: [u8; 65] = [0; 65];
            key[0] = 0x04;
            key[1..33].copy_from_slice(&into_bytes(CURVE_PT_X_SWAP));
            key[33..65].copy_from_slice(&into_bytes(CURVE_PT_Y_SWAP));

            let public_key: PublicKey = unwrap!(PublicKey::from_sec1_bytes(&key).ok());

            let prehashed_message_as_scalar: Scalar =
                Scalar::from_be_bytes_reduced(hash_bytes.into());

            let signature: Signature<_> =
                unwrap!(
                    Signature::from_scalars(into_bytes(R_SIGN_SWAP), into_bytes(S_SIGN_SWAP)).ok()
                );

            let start: u32 = DWT::cycle_count();
            let result = public_key
                .as_affine()
                .verify_prehashed(prehashed_message_as_scalar.to_bytes(), &signature);
            let elapsed: u32 = DWT::cycle_count().wrapping_sub(start);
            defmt::info!(
                "Approximate cycles per rust-crypto p256 verify: {}",
                elapsed
            );

            defmt::assert!(result.is_ok(), "Result is Err variant");
        }
    }

    #[test]
    fn ecdsa_verify_nb(pka: &mut Pka) {
        unwrap!(pka.ecdsa_verify_start(&NIST_P256, &SIGNATURE, &PUB_KEY, &HASH));
        unwrap!(nb::block!(pka.ecdsa_verify_result()));
    }

    #[test]
    fn ecdsa_verify_ram_err(pka: &mut Pka) {
        unwrap!(pka.ecdsa_sign_start(&NIST_P256, &INTEGER, &PRIVATE_KEY, &HASH));
        defmt::assert_eq!(
            pka.ecdsa_verify(&NIST_P256, &SIGNATURE, &PUB_KEY, &HASH),
            Err(EcdsaVerifyError::Ram)
        );
        let mut r_sign: [u32; 8] = [0; 8];
        let mut s_sign: [u32; 8] = [0; 8];
        unwrap!(nb::block!(pka.ecdsa_sign_result(&mut r_sign, &mut s_sign)));
        defmt::assert_eq!(r_sign, R_SIGN);
        defmt::assert_eq!(s_sign, S_SIGN);
    }

    #[test]
    fn ecdsa_verify_mode_err(pka: &mut Pka) {
        unwrap!(pka.ecdsa_sign_start(&NIST_P256, &INTEGER, &PRIVATE_KEY, &HASH));
        defmt::assert_eq!(
            nb::block!(pka.ecdsa_verify_result()),
            Err(EcdsaVerifyError::Mode { mode: 0b100100 })
        );
        let mut r_sign: [u32; 8] = [0; 8];
        let mut s_sign: [u32; 8] = [0; 8];
        unwrap!(nb::block!(pka.ecdsa_sign_result(&mut r_sign, &mut s_sign)));
        defmt::assert_eq!(r_sign, R_SIGN);
        defmt::assert_eq!(s_sign, S_SIGN);
    }

    #[test]
    fn ecdsa_verify_invalid_err(pka: &mut Pka) {
        let mut s_sign: [u32; 8] = S_SIGN;
        s_sign[0] ^= 0b1;
        let signature: EcdsaSignature<8> = EcdsaSignature {
            s_sign: &s_sign,
            r_sign: &R_SIGN,
        };
        defmt::assert_eq!(
            pka.ecdsa_verify(&NIST_P256, &signature, &PUB_KEY, &HASH),
            Err(EcdsaVerifyError::Invalid)
        )
    }

    // key from module-level documentation in PKA
    #[test]
    fn ecdsa_doc_keypair(pka: &mut Pka) {
        const PRIV_KEY: [u32; 8] = [
            0x49ac8727, 0xcee87484, 0xfe6dfda5, 0x10238ad4, 0x11ace8fe, 0x593a8cb7, 0x0492d659,
            0xdb81802a,
        ];

        let mut r_sign: [u32; 8] = [0; 8];
        let mut s_sign: [u32; 8] = [0; 8];
        let nonce: [u32; 8] = [0x87654321; 8];
        let hash: [u32; 8] = [0x12345678; 8];
        unwrap!(pka.ecdsa_sign(
            &NIST_P256,
            &nonce,
            &PRIV_KEY,
            &hash,
            &mut r_sign,
            &mut s_sign,
        ));

        let sig: EcdsaSignature<8> = EcdsaSignature {
            r_sign: &r_sign,
            s_sign: &s_sign,
        };

        const CURVE_PT_X: [u32; 8] = [
            0xfa655759, 0xdec39028, 0x96460a43, 0x2bae1d00, 0x9126e1b4, 0x88789ff4, 0xef6b9a9b,
            0xde1bc363,
        ];
        const CURVE_PT_Y: [u32; 8] = [
            0x8fa02ac4, 0xc421ca88, 0x4f0651f4, 0xe985e3cf, 0xd0af4069, 0xcc87f3a8, 0x8a8e95e7,
            0x556ced97,
        ];

        let pub_key: EcdsaPublicKey<8> = EcdsaPublicKey {
            curve_pt_x: &CURVE_PT_X,
            curve_pt_y: &CURVE_PT_Y,
        };

        unwrap!(pka.ecdsa_verify(&NIST_P256, &sig, &pub_key, &hash));
    }
}
