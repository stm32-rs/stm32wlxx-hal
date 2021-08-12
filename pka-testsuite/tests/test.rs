#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    pac::{self, DWT},
    pka::{curve::NIST_P256, EcdsaPublicKey, EcdsaSignature, Pka},
    rcc,
    util::reset_cycle_count,
};

const FREQ: u32 = 48_000_000;
const CYC_PER_MICRO: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:µs}", DWT::get_cycle_count() / CYC_PER_MICRO);

// const uint8_t SigVer_Msg[] = {
//     0x59, 0x05, 0x23, 0x88, 0x77, 0xc7, 0x74, 0x21, 0xf7, 0x3e, 0x43, 0xee, 0x3d, 0xa6, 0xf2, 0xd9,
//     0xe2, 0xcc, 0xad, 0x5f, 0xc9, 0x42, 0xdc, 0xec, 0x0c, 0xbd, 0x25, 0x48, 0x29, 0x35, 0xfa, 0xaf,
//     0x41, 0x69, 0x83, 0xfe, 0x16, 0x5b, 0x1a, 0x04, 0x5e, 0xe2, 0xbc, 0xd2, 0xe6, 0xdc, 0xa3, 0xbd,
//     0xf4, 0x6c, 0x43, 0x10, 0xa7, 0x46, 0x1f, 0x9a, 0x37, 0x96, 0x0c, 0xa6, 0x72, 0xd3, 0xfe, 0xb5,
//     0x47, 0x3e, 0x25, 0x36, 0x05, 0xfb, 0x1d, 0xdf, 0xd2, 0x80, 0x65, 0xb5, 0x3c, 0xb5, 0x85, 0x8a,
//     0x8a, 0xd2, 0x81, 0x75, 0xbf, 0x9b, 0xd3, 0x86, 0xa5, 0xe4, 0x71, 0xea, 0x7a, 0x65, 0xc1, 0x7c,
//     0xc9, 0x34, 0xa9, 0xd7, 0x91, 0xe9, 0x14, 0x91, 0xeb, 0x37, 0x54, 0xd0, 0x37, 0x99, 0x79, 0x0f,
//     0xe2, 0xd3, 0x08, 0xd1, 0x61, 0x46, 0xd5, 0xc9, 0xb0, 0xd0, 0xde, 0xbd, 0x97, 0xd7, 0x9c, 0xe8
//   };
//   const uint32_t SigVer_Msg_len = 128;

//   /* Result of hashing SigVer_Msg (You can verify using "openssl dgst -sha256" or "sha256sum" utilities)*/
//   const uint8_t SigVer_Hash_Msg[] = {
//     0x44, 0xac, 0xf6, 0xb7, 0xe3, 0x6c, 0x13, 0x42, 0xc2, 0xc5, 0x89, 0x72, 0x04, 0xfe, 0x09, 0x50,
//     0x4e, 0x1e, 0x2e, 0xfb, 0x1a, 0x90, 0x03, 0x77, 0xdb, 0xc4, 0xe7, 0xa6, 0xa1, 0x33, 0xec, 0x56
//   };
//   const uint32_t SigVer_Hash_Msg_len = 32;

//   const uint8_t SigVer_d[] = {
//     0x51, 0x9b, 0x42, 0x3d, 0x71, 0x5f, 0x8b, 0x58, 0x1f, 0x4f, 0xa8, 0xee, 0x59, 0xf4, 0x77, 0x1a,
//     0x5b, 0x44, 0xc8, 0x13, 0x0b, 0x4e, 0x3e, 0xac, 0xca, 0x54, 0xa5, 0x6d, 0xda, 0x72, 0xb4, 0x64
//   };
//   const uint32_t SigVer_d_len = 32;

//   const uint8_t SigVer_Qx[] = {
//     0x1c, 0xcb, 0xe9, 0x1c, 0x07, 0x5f, 0xc7, 0xf4, 0xf0, 0x33, 0xbf, 0xa2, 0x48, 0xdb, 0x8f, 0xcc,
//     0xd3, 0x56, 0x5d, 0xe9, 0x4b, 0xbf, 0xb1, 0x2f, 0x3c, 0x59, 0xff, 0x46, 0xc2, 0x71, 0xbf, 0x83
//   };
//   const uint32_t SigVer_Qx_len = 32;

//   const uint8_t SigVer_Qy[] = {
//     0xce, 0x40, 0x14, 0xc6, 0x88, 0x11, 0xf9, 0xa2, 0x1a, 0x1f, 0xdb, 0x2c, 0x0e, 0x61, 0x13, 0xe0,
//     0x6d, 0xb7, 0xca, 0x93, 0xb7, 0x40, 0x4e, 0x78, 0xdc, 0x7c, 0xcd, 0x5c, 0xa8, 0x9a, 0x4c, 0xa9
//   };
//   const uint32_t SigVer_Qy_len = 32;

//   const uint8_t SigVer_k[] = {
//     0x94, 0xa1, 0xbb, 0xb1, 0x4b, 0x90, 0x6a, 0x61, 0xa2, 0x80, 0xf2, 0x45, 0xf9, 0xe9, 0x3c, 0x7f,
//     0x3b, 0x4a, 0x62, 0x47, 0x82, 0x4f, 0x5d, 0x33, 0xb9, 0x67, 0x07, 0x87, 0x64, 0x2a, 0x68, 0xde
//   };
//   const uint32_t SigVer_k_len = 32;

//   const uint8_t SigVer_R[] = {
//     0xf3, 0xac, 0x80, 0x61, 0xb5, 0x14, 0x79, 0x5b, 0x88, 0x43, 0xe3, 0xd6, 0x62, 0x95, 0x27, 0xed,
//     0x2a, 0xfd, 0x6b, 0x1f, 0x6a, 0x55, 0x5a, 0x7a, 0xca, 0xbb, 0x5e, 0x6f, 0x79, 0xc8, 0xc2, 0xac
//   };
//   const uint32_t SigVer_R_len = 32;

//   const uint8_t SigVer_S[] = {
//     0x8b, 0xf7, 0x78, 0x19, 0xca, 0x05, 0xa6, 0xb2, 0x78, 0x6c, 0x76, 0x26, 0x2b, 0xf7, 0x37, 0x1c,
//     0xef, 0x97, 0xb2, 0x18, 0xe9, 0x6f, 0x17, 0x5a, 0x3c, 0xcd, 0xda, 0x2a, 0xcc, 0x05, 0x89, 0x03
//   };
//   const uint32_t SigVer_S_len = 32;

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

#[defmt_test::tests]
mod tests {
    use stm32wl_hal::pka::{EcdsaSignError, EcdsaSignature, EcdsaVerifyError};

    use super::*;

    #[init]
    fn init() -> Pka {
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC);

        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        reset_cycle_count(&mut cp.DWT);

        Pka::new(dp.PKA, &mut dp.RCC)
    }

    #[test]
    fn ecdsa_sign(pka: &mut Pka) {
        let mut r_sign: [u32; 8] = [0; 8];
        let mut s_sign: [u32; 8] = [0; 8];
        unwrap!(pka.ecdsa_sign(
            &NIST_P256,
            &INTEGER,
            &PRIVATE_KEY,
            &HASH,
            &mut r_sign,
            &mut s_sign,
        ));
        defmt::assert_eq!(r_sign, R_SIGN);
        defmt::assert_eq!(s_sign, S_SIGN);
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
        unwrap!(pka.ecdsa_verify(&NIST_P256, &SIGNATURE, &PUB_KEY, &HASH))
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
