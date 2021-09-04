#![no_std]
#![no_main]

// NIST AESAVS Nov 15 2002 B.1
pub const ECB_PT_CT_128: [([u32; 4], [u32; 4]); 7] = [
    (
        [0xf34481ec, 0x3cc627ba, 0xcd5dc3fb, 0x08f273e6],
        [0x0336763e, 0x966d9259, 0x5a567cc9, 0xce537f5e],
    ),
    (
        [0x9798c464, 0x0bad75c7, 0xc3227db9, 0x10174e72],
        [0xa9a1631b, 0xf4996954, 0xebc09395, 0x7b234589],
    ),
    (
        [0x96ab5c2f, 0xf612d9df, 0xaae8c31f, 0x30c42168],
        [0xff4f8391, 0xa6a40ca5, 0xb25d23be, 0xdd44a597],
    ),
    (
        [0x6a118a87, 0x4519e64e, 0x9963798a, 0x503f1d35],
        [0xdc43be40, 0xbe0e5371, 0x2f7e2bf5, 0xca707209],
    ),
    (
        [0xcb9fceec, 0x81286ca3, 0xe989bd97, 0x9b0cb284],
        [0x92beedab, 0x1895a94f, 0xaa69b632, 0xe5cc47ce],
    ),
    (
        [0xb26aeb18, 0x74e47ca8, 0x358ff223, 0x78f09144],
        [0x459264f4, 0x798f6a78, 0xbacb89c1, 0x5ed3d601],
    ),
    (
        [0x58c8e00b, 0x2631686d, 0x54eab84b, 0x91f0aca1],
        [0x08a4e2ef, 0xec8a8e33, 0x12ca7460, 0xb9040bbf],
    ),
];

// NIST AESAVS Nov 15 2002 B.1
pub const ECB_PT_CT_256: [([u32; 4], [u32; 4]); 5] = [
    (
        [0x014730f8, 0x0ac625fe, 0x84f026c6, 0x0bfd547d],
        [0x5c9d844e, 0xd46f9885, 0x085e5d6a, 0x4f94c7d7],
    ),
    (
        [0x0b24af36, 0x193ce466, 0x5f2825d7, 0xb4749c98],
        [0xa9ff75bd, 0x7cf6613d, 0x3731c77c, 0x3b6d0c04],
    ),
    (
        [0x761c1fe4, 0x1a18acf2, 0x0d241650, 0x611d90f1],
        [0x623a52fc, 0xea5d443e, 0x48d9181a, 0xb32c7421],
    ),
    (
        [0x8a560769, 0xd605868a, 0xd80d819b, 0xdba03771],
        [0x38f2c7ae, 0x10612415, 0xd27ca190, 0xd27da8b4],
    ),
    (
        [0x91fbef2d, 0x15a97816, 0x060bee1f, 0xeaa49afe],
        [0x1bc704f1, 0xbce135ce, 0xb810341b, 0x216d7abe],
    ),
];

// NIST AESAVS Nov 15 2002 B.1
pub const ECB_KEY_CT_128: [([u32; 4], [u32; 4]); 21] = [
    (
        [0x10a58869, 0xd74be5a3, 0x74cf867c, 0xfb473859],
        [0x6d251e69, 0x44b051e0, 0x4eaa6fb4, 0xdbf78465],
    ),
    (
        [0xcaea65cd, 0xbb75e916, 0x9ecd22eb, 0xe6e54675],
        [0x6e292011, 0x90152df4, 0xee058139, 0xdef610bb],
    ),
    (
        [0xa2e2fa9b, 0xaf7d2082, 0x2ca9f054, 0x2f764a41],
        [0xc3b44b95, 0xd9d2f256, 0x70eee9a0, 0xde099fa3],
    ),
    (
        [0xb6364ac4, 0xe1de1e28, 0x5eaf144a, 0x2415f7a0],
        [0x5d9b0557, 0x8fc944b3, 0xcf1ccf0e, 0x746cd581],
    ),
    (
        [0x64cf9c7a, 0xbc50b888, 0xaf65f49d, 0x521944b2],
        [0xf7efc89d, 0x5dba5781, 0x04016ce5, 0xad659c05],
    ),
    (
        [0x47d6742e, 0xefcc0465, 0xdc96355e, 0x851b64d9],
        [0x0306194f, 0x666d1836, 0x24aa230a, 0x8b264ae7],
    ),
    (
        [0x3eb39790, 0x678c56be, 0xe34bbcde, 0xccf6cdb5],
        [0x858075d5, 0x36d79cce, 0xe571f7d7, 0x204b1f67],
    ),
    (
        [0x64110a92, 0x4f0743d5, 0x00ccadae, 0x72c13427],
        [0x35870c6a, 0x57e9e923, 0x14bcb808, 0x7cde72ce],
    ),
    (
        [0x18d81265, 0x16f8a12a, 0xb1a36d9f, 0x04d68e51],
        [0x6c68e9be, 0x5ec41e22, 0xc825b7c7, 0xaffb4363],
    ),
    (
        [0xf5303579, 0x68578480, 0xb398a3c2, 0x51cd1093],
        [0xf5df3999, 0x0fc688f1, 0xb07224cc, 0x03e86cea],
    ),
    (
        [0xda84367f, 0x325d42d6, 0x01b43269, 0x64802e8e],
        [0xbba071bc, 0xb470f8f6, 0x586e5d3a, 0xdd18bc66],
    ),
    (
        [0xe37b1c6a, 0xa2846f6f, 0xdb413f23, 0x8b089f23],
        [0x43c9f7e6, 0x2f5d288b, 0xb27aa40e, 0xf8fe1ea8],
    ),
    (
        [0x6c002b68, 0x2483e0ca, 0xbcc731c2, 0x53be5674],
        [0x3580d19c, 0xff44f101, 0x4a7c966a, 0x69059de5],
    ),
    (
        [0x143ae8ed, 0x6555aba9, 0x6110ab58, 0x893a8ae1],
        [0x806da864, 0xdd29d48d, 0xeafbe764, 0xf8202aef],
    ),
    (
        [0xb69418a8, 0x5332240d, 0xc8249235, 0x3956ae0c],
        [0xa303d940, 0xded8f0ba, 0xff6f7541, 0x4cac5243],
    ),
    (
        [0x71b5c08a, 0x1993e136, 0x2e4d0ce9, 0xb22b78d5],
        [0xc2dabd11, 0x7f8a3eca, 0xbfbb11d1, 0x2194d9d0],
    ),
    (
        [0xe234cdca, 0x2606b81f, 0x29408d5f, 0x6da21206],
        [0xfff60a47, 0x40086b3b, 0x9c56195b, 0x98d91a7b],
    ),
    (
        [0x13237c49, 0x074a3da0, 0x78dc1d82, 0x8bb78c6f],
        [0x8146a08e, 0x2357f0ca, 0xa30ca8c9, 0x4d1a0544],
    ),
    (
        [0x3071a2a4, 0x8fe6cbd0, 0x4f1a1290, 0x98e308f8],
        [0x4b98e06d, 0x356deb07, 0xebb824e5, 0x713f7be3],
    ),
    (
        [0x90f42ec0, 0xf68385f2, 0xffc5dfc0, 0x3a654dce],
        [0x7a20a53d, 0x460fc9ce, 0x0423a7a0, 0x764c6cf2],
    ),
    (
        [0xfebd9a24, 0xd8b65c1c, 0x787d50a4, 0xed3619a9],
        [0xf4a70d8a, 0xf877f9b0, 0x2b4c40df, 0x57d45b17],
    ),
];

// NIST AESAVS Nov 15 2002 B.1
pub const ECB_KEY_CT_256: [([u32; 8], [u32; 4]); 16] = [
    (
        [
            0xc47b0294, 0xdbbbee0f, 0xec4757f2, 0x2ffeee35, 0x87ca4730, 0xc3d33b69, 0x1df38bab,
            0x076bc558,
        ],
        [0x46f2fb34, 0x2d6f0ab4, 0x77476fc5, 0x01242c5f],
    ),
    (
        [
            0x28d46cff, 0xa1585331, 0x94214a91, 0xe712fc2b, 0x45b51807, 0x6675affd, 0x910edeca,
            0x5f41ac64,
        ],
        [0x4bf3b0a6, 0x9aeb6657, 0x794f2901, 0xb1440ad4],
    ),
    (
        [
            0xc1cc358b, 0x449909a1, 0x9436cfbb, 0x3f852ef8, 0xbcb5ed12, 0xac705832, 0x5f56e609,
            0x9aab1a1c,
        ],
        [0x35206527, 0x2169abf9, 0x85684392, 0x7d0674fd],
    ),
    (
        [
            0x984ca75f, 0x4ee8d706, 0xf46c2d98, 0xc0bf4a45, 0xf5b00d79, 0x1c2dfeb1, 0x91b5ed8e,
            0x420fd627,
        ],
        [0x4307456a, 0x9e67813b, 0x452e15fa, 0x8fffe398],
    ),
    (
        [
            0xb43d08a4, 0x47ac8609, 0xbaadae4f, 0xf12918b9, 0xf68fc165, 0x3f126922, 0x2f123981,
            0xded7a92f,
        ],
        [0x46634466, 0x07354989, 0x477a5c6f, 0x0f007ef4],
    ),
    (
        [
            0x1d85a181, 0xb54cde51, 0xf0e09809, 0x5b2962fd, 0xc93b51fe, 0x9b88602b, 0x3f54130b,
            0xf76a5bd9,
        ],
        [0x531c2c38, 0x344578b8, 0x4d50b3c9, 0x17bbb6e1],
    ),
    (
        [
            0xdc0eba1f, 0x2232a787, 0x9ded34ed, 0x8428eeb8, 0x769b056b, 0xbaf8ad77, 0xcb65c354,
            0x1430b4cf,
        ],
        [0xfc6aec90, 0x63234800, 0x05c58e7e, 0x1ab004ad],
    ),
    (
        [
            0xf8be9ba6, 0x15c5a952, 0xcabbca24, 0xf68f8593, 0x039624d5, 0x24c816ac, 0xda2c9183,
            0xbd917cb9,
        ],
        [0xa3944b95, 0xca0b5204, 0x3584ef02, 0x151926a8],
    ),
    (
        [
            0x797f8b3d, 0x176dac5b, 0x7e34a2d5, 0x39c4ef36, 0x7a16f863, 0x5f626473, 0x7591c5c0,
            0x7bf57a3e,
        ],
        [0xa74289fe, 0x73a4c123, 0xca189ea1, 0xe1b49ad5],
    ),
    (
        [
            0x6838d40c, 0xaf927749, 0xc13f0329, 0xd331f448, 0xe202c73e, 0xf52c5f73, 0xa37ca635,
            0xd4c47707,
        ],
        [0xb91d4ea4, 0x488644b5, 0x6cf0812f, 0xa7fcf5fc],
    ),
    (
        [
            0xccd1bc3c, 0x659cd3c5, 0x9bc43748, 0x4e3c5c72, 0x4441da8d, 0x6e90ce55, 0x6cd57d07,
            0x52663bbc,
        ],
        [0x304f81ab, 0x61a80c2e, 0x743b94d5, 0x002a126b],
    ),
    (
        [
            0x13428b5e, 0x4c005e06, 0x36dd3384, 0x05d173ab, 0x135dec2a, 0x25c22c5d, 0xf0722d69,
            0xdcc43887,
        ],
        [0x649a7154, 0x5378c783, 0xe368c9ad, 0xe7114f6c],
    ),
    (
        [
            0x07eb03a0, 0x8d291d1b, 0x07408bf3, 0x512ab40c, 0x91097ac7, 0x7461aad4, 0xbb859647,
            0xf74f00ee,
        ],
        [0x47cb030d, 0xa2ab051d, 0xfc6c4bf6, 0x910d12bb],
    ),
    (
        [
            0x90143ae2, 0x0cd78c5d, 0x8ebdd6cb, 0x9dc17624, 0x27a96c78, 0xc639bccc, 0x41a61424,
            0x564eafe1,
        ],
        [0x798c7c00, 0x5dee432b, 0x2c8ea5df, 0xa381ecc3],
    ),
    (
        [
            0xb7a5794d, 0x52737475, 0xd53d5a37, 0x7200849b, 0xe0260a67, 0xa2b22ced, 0x8bbef128,
            0x82270d07,
        ],
        [0x637c31dc, 0x2591a076, 0x36f646b7, 0x2daabbe7],
    ),
    (
        [
            0xfca02f3d, 0x5011cfc5, 0xc1e23165, 0xd413a049, 0xd4526a99, 0x1827424d, 0x896fe343,
            0x5e0bf68e,
        ],
        [0x179a49c7, 0x12154bbf, 0xfbe6e7a8, 0x4a18e220],
    ),
];

const ECB_NUM_128: u32 = (ECB_PT_CT_128.len() + ECB_KEY_CT_128.len()) as u32;
const ECB_NUM_256: u32 = (ECB_PT_CT_256.len() + ECB_KEY_CT_256.len()) as u32;

use defmt::unwrap;
use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{aes::Aes, cortex_m::peripheral::DWT, pac, rcc, util::reset_cycle_count};

const FREQ: u32 = 48_000_000;
const CYC_PER_US: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:µs}", DWT::get_cycle_count() / CYC_PER_US);

const ZERO_16B: [u32; 4] = [0; 4];
const ZERO_32B: [u32; 8] = [0; 8];

#[inline(always)]
fn stopwatch<F>(f: F) -> u32
where
    F: FnOnce() -> (),
{
    let start: u32 = DWT::get_cycle_count();
    f();
    let end: u32 = DWT::get_cycle_count();
    end.wrapping_sub(start)
}

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
    fn encrypt_ecb_128(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for (plaintext, ciphertext) in ECB_PT_CT_128.iter() {
            let mut output_ciphertext: [u32; 4] = [0; 4];
            total_elapsed += stopwatch(|| {
                unwrap!(aes.encrypt_ecb(&ZERO_16B, plaintext, &mut output_ciphertext))
            });
            defmt::assert_eq!(&output_ciphertext, ciphertext);
        }

        for (key, ciphertext) in ECB_KEY_CT_128.iter() {
            let mut output_ciphertext: [u32; 4] = [0; 4];
            total_elapsed +=
                stopwatch(|| unwrap!(aes.encrypt_ecb(key, &ZERO_16B, &mut output_ciphertext)));
            defmt::assert_eq!(&output_ciphertext, ciphertext);
        }

        defmt::info!(
            "Approximate cycles per 128-bit encrypt: {}",
            total_elapsed / ECB_NUM_128
        );
    }

    #[test]
    fn encrypt_ecb_256(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for (plaintext, ciphertext) in ECB_PT_CT_256.iter() {
            let mut output_ciphertext: [u32; 4] = [0; 4];
            total_elapsed += stopwatch(|| {
                unwrap!(aes.encrypt_ecb(&ZERO_32B, plaintext, &mut output_ciphertext))
            });
            defmt::assert_eq!(&output_ciphertext, ciphertext);
        }

        for (key, ciphertext) in ECB_KEY_CT_256.iter() {
            let mut output_ciphertext: [u32; 4] = [0; 4];
            total_elapsed +=
                stopwatch(|| unwrap!(aes.encrypt_ecb(key, &ZERO_16B, &mut output_ciphertext)));
            defmt::assert_eq!(&output_ciphertext, ciphertext);
        }

        defmt::info!(
            "Approximate cycles per 256-bit encrypt: {}",
            total_elapsed / ECB_NUM_256
        );
    }

    #[test]
    fn encrypt_ecb_inplace_128(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for (plaintext, ciphertext) in ECB_PT_CT_128.iter() {
            let mut output_ciphertext: [u32; 4] = *plaintext;
            total_elapsed +=
                stopwatch(|| unwrap!(aes.encrypt_ecb_inplace(&ZERO_16B, &mut output_ciphertext)));

            defmt::assert_eq!(&output_ciphertext, ciphertext);
        }

        for (key, ciphertext) in ECB_KEY_CT_128.iter() {
            let mut output_ciphertext: [u32; 4] = ZERO_16B;
            total_elapsed +=
                stopwatch(|| unwrap!(aes.encrypt_ecb_inplace(key, &mut output_ciphertext)));

            defmt::assert_eq!(&output_ciphertext, ciphertext);
        }

        defmt::info!(
            "Approximate cycles per 128-bit encrypt: {}",
            total_elapsed / ECB_NUM_128
        );
    }

    #[test]
    fn encrypt_ecb_inplace_256(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for (plaintext, ciphertext) in ECB_PT_CT_256.iter() {
            let mut output_ciphertext: [u32; 4] = *plaintext;
            total_elapsed +=
                stopwatch(|| unwrap!(aes.encrypt_ecb_inplace(&ZERO_32B, &mut output_ciphertext)));
            defmt::assert_eq!(&output_ciphertext, ciphertext);
        }

        for (key, ciphertext) in ECB_KEY_CT_256.iter() {
            let mut output_ciphertext: [u32; 4] = ZERO_16B;
            total_elapsed +=
                stopwatch(|| unwrap!(aes.encrypt_ecb_inplace(key, &mut output_ciphertext)));
            defmt::assert_eq!(&output_ciphertext, ciphertext);
        }

        defmt::info!(
            "Approximate cycles per 256-bit encrypt: {}",
            total_elapsed / ECB_NUM_256
        );
    }

    #[test]
    fn decrypt_ecb_128(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for (plaintext, ciphertext) in ECB_PT_CT_128.iter() {
            let mut output_plaintext: [u32; 4] = [0; 4];
            total_elapsed += stopwatch(|| {
                unwrap!(aes.decrypt_ecb(&ZERO_16B, ciphertext, &mut output_plaintext))
            });
            defmt::assert_eq!(&output_plaintext, plaintext);
        }

        for (key, ciphertext) in ECB_KEY_CT_128.iter() {
            let mut output_plaintext: [u32; 4] = [0; 4];
            total_elapsed +=
                stopwatch(|| unwrap!(aes.decrypt_ecb(key, ciphertext, &mut output_plaintext)));
            defmt::assert_eq!(output_plaintext, ZERO_16B);
        }

        defmt::info!(
            "Average cycles per 128-bit decrypt: {}",
            total_elapsed / ECB_NUM_128
        );
    }

    #[test]
    fn decrypt_ecb_inplace_128(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for (plaintext, ciphertext) in ECB_PT_CT_128.iter() {
            let mut output_plaintext: [u32; 4] = *ciphertext;
            total_elapsed +=
                stopwatch(|| unwrap!(aes.decrypt_ecb_inplace(&ZERO_16B, &mut output_plaintext)));
            defmt::assert_eq!(&output_plaintext, plaintext);
        }

        for (key, ciphertext) in ECB_KEY_CT_128.iter() {
            let mut output_plaintext: [u32; 4] = *ciphertext;
            total_elapsed +=
                stopwatch(|| unwrap!(aes.decrypt_ecb_inplace(key, &mut output_plaintext)));
            defmt::assert_eq!(output_plaintext, ZERO_16B);
        }

        defmt::info!(
            "Average cycles per 128-bit decrypt: {}",
            total_elapsed / ECB_NUM_128
        );
    }

    #[test]
    fn decrypt_ecb_256(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for (plaintext, ciphertext) in ECB_PT_CT_256.iter() {
            let mut output_plaintext: [u32; 4] = [0; 4];
            total_elapsed += stopwatch(|| {
                unwrap!(aes.decrypt_ecb(&ZERO_32B, ciphertext, &mut output_plaintext))
            });
            defmt::assert_eq!(&output_plaintext, plaintext);
        }

        for (key, ciphertext) in ECB_KEY_CT_256.iter() {
            let mut output_plaintext: [u32; 4] = [0; 4];
            total_elapsed +=
                stopwatch(|| unwrap!(aes.decrypt_ecb(key, ciphertext, &mut output_plaintext)));
            defmt::assert_eq!(output_plaintext, ZERO_16B);
        }

        defmt::info!(
            "Average cycles per 256-bit decrypt: {}",
            total_elapsed / ECB_NUM_256
        );
    }

    #[test]
    fn decrypt_ecb_inplace_256(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for (plaintext, ciphertext) in ECB_PT_CT_256.iter() {
            let mut output_plaintext: [u32; 4] = *ciphertext;
            total_elapsed +=
                stopwatch(|| unwrap!(aes.decrypt_ecb_inplace(&ZERO_32B, &mut output_plaintext)));
            defmt::assert_eq!(&output_plaintext, plaintext);
        }

        for (key, ciphertext) in ECB_KEY_CT_256.iter() {
            let mut output_plaintext: [u32; 4] = *ciphertext;
            total_elapsed +=
                stopwatch(|| unwrap!(aes.decrypt_ecb_inplace(key, &mut output_plaintext)));
            defmt::assert_eq!(output_plaintext, ZERO_16B);
        }

        defmt::info!(
            "Average cycles per 256-bit decrypt: {}",
            total_elapsed / ECB_NUM_256
        );
    }
}
