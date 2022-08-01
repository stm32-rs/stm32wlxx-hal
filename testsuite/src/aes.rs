// Test vector sources:
//
// * ECB: NIST AESAVS Nov 15 2002 B.1
// * GCM: CAVS 14.0

#![no_std]
#![no_main]

use core::mem::size_of;
use defmt::unwrap;
use defmt_rtt as _; // global logger
use hex_literal::hex;
use nucleo_wl55jc_bsp::hal::{
    aes::{Aes, AesWrapClk, SwapMode},
    cortex_m::{self, peripheral::DWT},
    pac, rcc,
};
use panic_probe as _;

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

struct Gcm<const KEYSIZE: usize> {
    key: [u32; KEYSIZE],
    iv: [u32; 3],
    pt: &'static [u8],
    aad: &'static [u8],
    ct: &'static [u8],
    tag: [u32; 4],
}

const GCM_128: [Gcm<4>; 60] = [
    // [Keylen = 128]
    // [IVlen = 96]
    // [PTlen = 0]
    // [AADlen = 0]
    // [Taglen = 128]
    Gcm {
        key: [0x11754cd7, 0x2aec309b, 0xf52f7687, 0x212e8957],
        iv: [0x3c819d9a, 0x9bed0876, 0x15030b65],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x250327c6, 0x74aaf477, 0xaef26757, 0x48cf6971],
    },
    Gcm {
        key: [0xca47248a, 0xc0b6f837, 0x2a97ac43, 0x508308ed],
        iv: [0xffd2b598, 0xfeabc901, 0x9262d2be],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x60d20404, 0xaf527d24, 0x8d893ae4, 0x95707d1a],
    },
    Gcm {
        key: [0xdb1ad0bd, 0x1cf6db0b, 0x5d86efdd, 0x8914b218],
        iv: [0x36fad6ac, 0xb3c98e01, 0x38aeb9b1],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x5ee2ba73, 0x7d3f2a94, 0x4b335a81, 0xf6653cce],
    },
    Gcm {
        key: [0x1c7135af, 0x627c04c3, 0x2957f33f, 0x9ac08590],
        iv: [0x355c094f, 0xa09c8e92, 0x81178d34],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xb6ab2c7d, 0x906c9d9e, 0xc4c1498d, 0x2cbb5029],
    },
    Gcm {
        key: [0x6ca2c112, 0x05a6e55a, 0xb504dbf3, 0x491f8bdc],
        iv: [0xb1008b65, 0x0a2fee64, 0x2175c60d],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x7a9a225d, 0x5f9a0ebf, 0xe0e69f37, 0x1871a672],
    },
    Gcm {
        key: [0x69f2ca78, 0xbb5690ac, 0xc6587302, 0x628828d5],
        iv: [0x701da282, 0xcb6b6018, 0xdabd00d3],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xab1d40dd, 0xa1798d56, 0x687892e2, 0x159decfd],
    },
    Gcm {
        key: [0xdcf4e339, 0xc487b679, 0x7aaca931, 0x725f7bbd],
        iv: [0x2c1d955e, 0x35366760, 0xead8817c],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x32b542c5, 0xf344ccec, 0xeb460a02, 0x938d6b0c],
    },
    Gcm {
        key: [0x7658cdbb, 0x81572a23, 0xa78ee459, 0x6f844ee9],
        iv: [0x1c3baae9, 0xb9065961, 0x842cbe52],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x70c7123f, 0xc819aa06, 0x0ed2d3c1, 0x59b6ea41],
    },
    Gcm {
        key: [0x281a570b, 0x1e8f265e, 0xe09303ec, 0xae0cc46d],
        iv: [0x8c2941f7, 0x3cf8713a, 0xd5bc13df],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xa42e5e5f, 0x6fb00a9f, 0x1206b302, 0xedbfd87c],
    },
    Gcm {
        key: [0xcd332a98, 0x6f82d98c, 0x21527813, 0x1ad387b7],
        iv: [0x1d12b259, 0xf44b873d, 0x3942bc11],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x34238023, 0x648185d7, 0xef0cfcf5, 0x836e93cc],
    },
    Gcm {
        key: [0x80e1d98d, 0x10b27237, 0x386f0291, 0x89ec0448],
        iv: [0x239ebab2, 0xf524fd62, 0xc554a190],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x4c0f29d9, 0x63f0ed68, 0xdccf3449, 0x6cf43d00],
    },
    Gcm {
        key: [0x40650cdb, 0x61e3e19a, 0x1a98fb4e, 0x05377d35],
        iv: [0x69f0a81a, 0xaf6bb848, 0x6282f1b9],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x2657e12d, 0xec21c3ec, 0xf071af61, 0x79529fb4],
    },
    Gcm {
        key: [0x1e89a6cd, 0x7528cce1, 0xe2b2b5f7, 0xfd2b6b52],
        iv: [0xe11fd427, 0xa782d543, 0xf78efc60],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xeeedff87, 0x4c8edeea, 0x53e8be2a, 0x13afd81b],
    },
    Gcm {
        key: [0x2a7ad614, 0x6676057d, 0xb777dea4, 0x683d0d45],
        iv: [0xed721ea6, 0x7456d459, 0x4aafbd51],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xee3cab57, 0x78888439, 0xd90fa718, 0xb75738ad],
    },
    Gcm {
        key: [0xa364f494, 0xa4cd0147, 0xc3473107, 0x4dc1a85b],
        iv: [0x4aa8470d, 0xd404e405, 0x4b30093a],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xd8a7bba3, 0xa451902e, 0x3adc0106, 0x0c3c91a7],
    },
    // [Keylen = 128]
    // [IVlen = 96]
    // [PTlen = 128]
    // [AADlen = 0]
    // [Taglen = 128]
    Gcm {
        key: [0x7fddb574, 0x53c241d0, 0x3efbed3a, 0xc44e371c],
        iv: [0xee283a3f, 0xc75575e3, 0x3efd4887],
        pt: &hex!("d5de42b461646c255c87bd2962d3b9a2"),
        aad: &[],
        ct: &hex!("2ccda4a5415cb91e135c2a0f78c9b2fd"),
        tag: [0xb36d1df9, 0xb9d5e596, 0xf83e8b7f, 0x52971cb3],
    },
    Gcm {
        key: [0xab72c77b, 0x97cb5fe9, 0xa382d9fe, 0x81ffdbed],
        iv: [0x54cc7dc2, 0xc37ec006, 0xbcc6d1da],
        pt: &hex!("007c5e5b3e59df24a7c355584fc1518d"),
        aad: &[],
        ct: &hex!("0e1bde206a07a9c2c1b65300f8c64997"),
        tag: [0x2b440134, 0x6697138c, 0x7a4891ee, 0x59867d0c],
    },
    Gcm {
        key: [0x77b0a58a, 0x1e60541e, 0x5ea3d4d4, 0x2007940e],
        iv: [0xae7a2790, 0x4d95fe80, 0x0e83b345],
        pt: &hex!("6931a3ea07a9e95207334f0274a454dd"),
        aad: &[],
        ct: &hex!("76e39fad4000a07d35d879b785bd7fca"),
        tag: [0x5cb37247, 0x12f129f8, 0x6b7927f1, 0x3b45c835],
    },
    Gcm {
        key: [0xcaaa3f6f, 0xd31822ed, 0x2d2125f2, 0x25b0169f],
        iv: [0x7f6d9041, 0x483e8c14, 0x12fa552a],
        pt: &hex!("84c907b11ae3b79fc4451d1bf17f4a99"),
        aad: &[],
        ct: &hex!("fdb4aafa3519d3c055be8b347764ea33"),
        tag: [0x89e43bfe, 0xad01692c, 0x4ebe6565, 0x86e3fbe3],
    },
    Gcm {
        key: [0x02c8e81d, 0xebc563e9, 0x9cd262bf, 0xc64b0e11],
        iv: [0xb49057c9, 0x778d8c02, 0xfe00d029],
        pt: &hex!("ca2a51e9d05e96e6f1d14ced36811c5c"),
        aad: &[],
        ct: &hex!("5db602fb31bb9268d233bee0dd6b87ae"),
        tag: [0x789d2be2, 0xcc70b7c3, 0x89b31912, 0xe1c0a041],
    },
    Gcm {
        key: [0x4e625a3e, 0xdc61f0cb, 0x2f002da8, 0xf8a70245],
        iv: [0x66d632dd, 0x5ca10b08, 0xd4d8f97b],
        pt: &hex!("0b76d498add6e09c96d7694e5d620bd5"),
        aad: &[],
        ct: &hex!("17bdc7ef5649bec9cf6c565ce33cf889"),
        tag: [0x3f7944ba, 0xd062605f, 0x937ff6d6, 0x598a7651],
    },
    Gcm {
        key: [0x41ab3fc4, 0x88f8d4a8, 0x20e65b9d, 0x41a87de3],
        iv: [0x9b5d27d7, 0x5a0571e9, 0x3f581885],
        pt: &hex!("5ed0836e0a52777599800d4fe754ccbe"),
        aad: &[],
        ct: &hex!("88c0eb8c33a10a22e7561866566b191f"),
        tag: [0x83e88580, 0x2a594a8b, 0x008a94aa, 0x7ef06907],
    },
    Gcm {
        key: [0x00471842, 0x40a5948e, 0xd55701ea, 0xc2c4c26c],
        iv: [0xa3ab8da2, 0x2648c245, 0x3cdef55b],
        pt: &hex!("89ee9502871be15ee4a8c47ab123bfc9"),
        aad: &[],
        ct: &hex!("8b5cb59e7ad2e15c40d5fbcde28a0d17"),
        tag: [0x538e79f8, 0x80e2f65c, 0x72148f5a, 0xde4080a1],
    },
    Gcm {
        key: [0x735c5a4f, 0xf2438852, 0xdf3530c2, 0x3590ac28],
        iv: [0x7bee7c69, 0x38f1ae59, 0x671e2ddb],
        pt: &hex!("479e8d3bf0de4ce7cd4377d2ed3925cd"),
        aad: &[],
        ct: &hex!("2ca09b58178fbbfb82556599b92329a3"),
        tag: [0x2e3cf289, 0x5f111ec2, 0xa86508c3, 0x6a24e45d],
    },
    Gcm {
        key: [0x016dbb38, 0xdaa76dfe, 0x7da384eb, 0xf1240364],
        iv: [0x0793ef3a, 0xda782f78, 0xc98affe3],
        pt: &hex!("4b34a9ec5763524b191d5616c547f6b7"),
        aad: &[],
        ct: &hex!("609aa3f4541bc0fe9931daad2ee15d0c"),
        tag: [0x33afec59, 0xc45baf68, 0x9a5e1b13, 0xae423619],
    },
    Gcm {
        key: [0x2d176607, 0x883aface, 0x75011d14, 0x818f1be6],
        iv: [0x02162c36, 0x35bf6d54, 0x3e1cc148],
        pt: &hex!("71905ad5df601d056effd80dd7333662"),
        aad: &[],
        ct: &hex!("1b68598e1676d2cfd37aa00396fa9676"),
        tag: [0x5d060aa8, 0xa729774d, 0xa001aa9f, 0xdef2b3d2],
    },
    Gcm {
        key: [0x94fd0269, 0xa0ce8131, 0x33626f93, 0xc4af7e6f],
        iv: [0x11fc3928, 0x028dfa34, 0xdb06a1bc],
        pt: &hex!("a1aefec976cd87cf8a4c21bbe902f7b4"),
        aad: &[],
        ct: &hex!("b1baf8c58cdec88238b1b0ab0b40337d"),
        tag: [0x882f865d, 0xf7da529f, 0x768d4944, 0xe8387f69],
    },
    Gcm {
        key: [0xa7bec5e2, 0x4f0db262, 0x9a257d02, 0xfdfaea02],
        iv: [0x9d2ec94b, 0x92732779, 0x3583b818],
        pt: &hex!("a17bc5d428700f94c641e74aaacf2c5d"),
        aad: &[],
        ct: &hex!("d460fda5b24425b5caa8176c8c67b3a9"),
        tag: [0x0df72434, 0x0b8ca56e, 0x8dea6bbe, 0xb4b55c35],
    },
    Gcm {
        key: [0x39d945a0, 0x0e05d70a, 0x16e61334, 0xd2010209],
        iv: [0x1f931448, 0xe9013ec4, 0xec61af0c],
        pt: &hex!("9dd90ebfc054da214cbb30db7f75c692"),
        aad: &[],
        ct: &hex!("e4cb765408697cf85917a7a9264086e4"),
        tag: [0xfe9a1fe7, 0xa58d66e3, 0xb922693a, 0x163c1ff4],
    },
    Gcm {
        key: [0x6620ca65, 0xf72de7b8, 0x65de7319, 0x28a4723e],
        iv: [0xe6428b6b, 0x77e9b699, 0x3b809aef],
        pt: &hex!("7044f7c27d776f6a7d43abea35908de4"),
        aad: &[],
        ct: &hex!("a1c5634a07d05ca909dba87bf02228e4"),
        tag: [0xd8b40a60, 0xa6523733, 0x7db05b04, 0x5de8074c],
    },
    // [Keylen = 128]
    // [IVlen = 96]
    // [PTlen = 0]
    // [AADlen = 128]
    // [Taglen = 128]
    Gcm {
        key: [0x77be6370, 0x8971c4e2, 0x40d1cb79, 0xe8d77feb],
        iv: [0xe0e00f19, 0xfed7ba01, 0x36a797f3],
        pt: &[],
        aad: &hex!("7a43ec1d9c0a5a78a0b16533a6213cab"),
        ct: &[],
        tag: [0x209fcc8d, 0x3675ed93, 0x8e9c7166, 0x709dd946],
    },
    Gcm {
        key: [0x7680c5d3, 0xca615475, 0x8e510f4d, 0x25b98820],
        iv: [0xf8f105f9, 0xc3df4965, 0x780321f8],
        pt: &[],
        aad: &hex!("c94c410194c765e3dcc7964379758ed3"),
        ct: &[],
        tag: [0x94dca8ed, 0xfcf90bb7, 0x4b153c8d, 0x48a17930],
    },
    Gcm {
        key: [0xa82bb1ed, 0xc7c01a36, 0x89006f34, 0xbfed783e],
        iv: [0x963836b6, 0x7b188bec, 0xf9ba1411],
        pt: &[],
        aad: &hex!("9d115bb9bbd119fb777b6316065a9ac8"),
        ct: &[],
        tag: [0xc491889f, 0xa3eca454, 0x4ba0d51b, 0x8e0f3837],
    },
    Gcm {
        key: [0xb9782d0a, 0x5986c63f, 0x352d3bc4, 0xc7ecc96d],
        iv: [0x4541e15b, 0x92edea44, 0xeceb1f2a],
        pt: &[],
        aad: &hex!("f1a9f0723429c5b26185ac3ea7e13d7a"),
        ct: &[],
        tag: [0x74d0d369, 0x49f02766, 0x70f9ddc5, 0x79e94f3a],
    },
    Gcm {
        key: [0x59b95785, 0xb30f2056, 0x79fc4f3f, 0x9a90102f],
        iv: [0x1908787c, 0xc1e1880a, 0x6ef5dd17],
        pt: &[],
        aad: &hex!("39852d3182944a5177db277b63910702"),
        ct: &[],
        tag: [0x8f9a96c0, 0x13992485, 0xb43e2b62, 0x745ad173],
    },
    Gcm {
        key: [0x34dd7926, 0xab13d407, 0x8160d87d, 0xe2e3c724],
        iv: [0xc11ccdaf, 0x798ab03a, 0xf2d97ef9],
        pt: &[],
        aad: &hex!("af698717a6d790b3bfc39195857bb5ff"),
        ct: &[],
        tag: [0x48116050, 0xbbd91182, 0x70d0be25, 0x2d29d5d4],
    },
    Gcm {
        key: [0x8ec86fab, 0x55aaab0e, 0x77455e9c, 0xd3dbc78e],
        iv: [0x15fd90a9, 0x867e14f0, 0xd63b53b9],
        pt: &[],
        aad: &hex!("e7509e276209a6d3ecfabb53ccdcd236"),
        ct: &[],
        tag: [0xd96d6ac0, 0xd309cebe, 0xdeba2af9, 0xf262132f],
    },
    Gcm {
        key: [0x66b2473d, 0x9e012166, 0x6d47633f, 0x7008eb1c],
        iv: [0xc1716c68, 0xa24d5777, 0x0b867e51],
        pt: &[],
        aad: &hex!("c20f686317d67e53dd79bae5c46dc111"),
        ct: &[],
        tag: [0x9a086168, 0x09cf1524, 0x7dfeb975, 0x6ba4f609],
    },
    Gcm {
        key: [0x5b262a9d, 0x00904d30, 0xa2587caa, 0xde091381],
        iv: [0xf7bc154c, 0xa562e8f2, 0xc1845598],
        pt: &[],
        aad: &hex!("23112d078c9914fa3dfe5218cd191016"),
        ct: &[],
        tag: [0x98854d19, 0x3a06dbe3, 0x2ce4497e, 0xec5c9a8b],
    },
    Gcm {
        key: [0x2e4fb9cc, 0x320188a6, 0xf1fa89a7, 0xa252273a],
        iv: [0x7a6d4ee6, 0x9c7256c1, 0x4fba8f5e],
        pt: &[],
        aad: &hex!("80ba4a202a68c3590d6557912c6f878e"),
        ct: &[],
        tag: [0x92803132, 0x73befb8a, 0xfa0bceca, 0x5a966d85],
    },
    Gcm {
        key: [0x5ea94973, 0xd8616daf, 0xa7f31db0, 0x716d1729],
        iv: [0xa05b6266, 0x9d250e61, 0xb077d28a],
        pt: &[],
        aad: &hex!("9620baf2f58d013f8a4c4871989c1b17"),
        ct: &[],
        tag: [0x7e550398, 0xdee72825, 0x6d6928cd, 0xaac43b73],
    },
    Gcm {
        key: [0x910385f6, 0xf07f9e57, 0xe483c47d, 0xd5206bcc],
        iv: [0x518f56e3, 0x3658df31, 0x1d42d9fe],
        pt: &[],
        aad: &hex!("5d157909a2a4607117e77da0e4493b88"),
        ct: &[],
        tag: [0xa7041ea4, 0xa1d74d9e, 0x66b9571b, 0x59b6a1d8],
    },
    Gcm {
        key: [0xcab3af7a, 0x15b430e0, 0x34e793bb, 0x30db8ab2],
        iv: [0x963a56e2, 0xe12f3870, 0x62e18498],
        pt: &[],
        aad: &hex!("a094a1dd1121d3aa52c81e8f10bf9f0c"),
        ct: &[],
        tag: [0x1a31d295, 0x601eb3c8, 0x2a54b234, 0x984ffdf5],
    },
    Gcm {
        key: [0x89c949e9, 0xc804af01, 0x4d5604b3, 0x9459f2c8],
        iv: [0xd1b104c8, 0x15bf1e94, 0xe28c8f16],
        pt: &[],
        aad: &hex!("82adcd638d3fa9d9f3e84100d61e0777"),
        ct: &[],
        tag: [0x88db9d62, 0x172ed043, 0xaa10f16d, 0x227dc41b],
    },
    Gcm {
        key: [0xa4d994c4, 0xac5ac0f0, 0x29132457, 0x14fbe235],
        iv: [0xa9472dad, 0xcca8d7e0, 0xe3b8084d],
        pt: &[],
        aad: &hex!("eb318b9e17575203dd29ebed20ec82f9"),
        ct: &[],
        tag: [0x323df7f3, 0x3694106f, 0x56739de0, 0x973216a3],
    },
    // [Keylen = 128]
    // [IVlen = 96]
    // [PTlen = 128]
    // [AADlen = 128]
    // [Taglen = 128]
    Gcm {
        key: [0xc939cc13, 0x397c1d37, 0xde6ae0e1, 0xcb7c423c],
        iv: [0xb3d8cc01, 0x7cbb89b3, 0x9e0f67e2],
        pt: &hex!("c3b3c41f113a31b73d9a5cd432103069"),
        aad: &hex!("24825602bd12a984e0092d3e448eda5f"),
        ct: &hex!("93fe7d9e9bfd10348a5606e5cafa7354"),
        tag: [0x0032a1dc, 0x85f1c978, 0x6925a2e7, 0x1d8272dd],
    },
    Gcm {
        key: [0x599eb65e, 0x6b2a2a7f, 0xcc40e51c, 0x4f6e3257],
        iv: [0xd407301c, 0xfa29af85, 0x25981c17],
        pt: &hex!("a6c9e0f248f07a3046ece12125666921"),
        aad: &hex!("10e72efe048648d40139477a2016f8ce"),
        ct: &hex!("1be9359a543fd7ec3c4bc6f3c9395e89"),
        tag: [0xe2e9c07d, 0x4c3c10a6, 0x137ca433, 0xda42f9a8],
    },
    Gcm {
        key: [0x2d265491, 0x712fe6d7, 0x087a5545, 0x852f4f44],
        iv: [0xc59868b8, 0x701fbf88, 0xe6343262],
        pt: &hex!("301873be69f05a84f22408aa0862d19a"),
        aad: &hex!("67105634ac9fbf849970dc416de7ad30"),
        ct: &hex!("98b03c77a67831bcf16b1dd96c324e1c"),
        tag: [0x39152e26, 0xbdc4d17e, 0x8c00493f, 0xa0be92f2],
    },
    Gcm {
        key: [0x1fd1e536, 0xa1c39c75, 0xfd583bc8, 0xe3372029],
        iv: [0x281f2552, 0xf8c34fb9, 0xb3ec85aa],
        pt: &hex!("f801e0839619d2c1465f0245869360da"),
        aad: &hex!("bf12a140d86727f67b860bcf6f34e55f"),
        ct: &hex!("35371f2779f4140dfdb1afe79d563ed9"),
        tag: [0xcc2b0b0f, 0x1f8b3db5, 0xdc1b41ce, 0x73f5c221],
    },
    Gcm {
        key: [0x7b0345f6, 0xdcf469ec, 0xf9b17efa, 0x39de5359],
        iv: [0xb15d6fcd, 0xe5e6cf1f, 0xa99ba145],
        pt: &hex!("822ae01a0372b6aa46c2e5bf19db92f2"),
        aad: &hex!("72e9cb26885154d4629e7bc91279bb19"),
        ct: &hex!("382e440694b0c93be8dd438e37635194"),
        tag: [0x2fa042bf, 0xf9a9cd35, 0xe343b520, 0x017841bb],
    },
    Gcm {
        key: [0x9db91a40, 0x020cdb07, 0xf8876930, 0x9a6ac40b],
        iv: [0xf89e1b7e, 0x598cc253, 0x5a5c8659],
        pt: &hex!("f4a5003db4a4ebbc2fdb8c6756830391"),
        aad: &hex!("70910598e7abd4f0503ecd9e21bdafb5"),
        ct: &hex!("40d7fc4ccc8147581f40655a07f23ee9"),
        tag: [0x243331b4, 0x8404859c, 0x66af4d7b, 0x2ee44109],
    },
    Gcm {
        key: [0xe2f48398, 0x9b349efb, 0x59ae0a7c, 0xadc74b7a],
        iv: [0x3338343f, 0x9b97ebb7, 0x84e75027],
        pt: &hex!("14d80ad66e8f5f2e6c43c3109e023a93"),
        aad: &hex!("8b12987e600ff58df54f1f5e62e59e61"),
        ct: &hex!("43c2d68384d486e9788950bbb8cd8fd1"),
        tag: [0x47d7e914, 0x4ff0ed4a, 0xa3300a94, 0x4a007882],
    },
    Gcm {
        key: [0x5c115508, 0x4cc0ede7, 0x6b3bc22e, 0x9f7574ef],
        iv: [0x9549e4ba, 0x69a61cad, 0x7856efc1],
        pt: &hex!("d1448fa852b84408e2dad8381f363de7"),
        aad: &hex!("e98e9d9c618e46fef32660976f854ee3"),
        ct: &hex!("f78b60ca125218493bea1c50a2e12ef4"),
        tag: [0xd72da7f5, 0xc6cf0bca, 0x7242c718, 0x35809449],
    },
    Gcm {
        key: [0x23525037, 0x40a4e1b2, 0x2dcc9c00, 0x2f53bd11],
        iv: [0x474ecccc, 0x3182e03c, 0x80a7be74],
        pt: &hex!("dc1c35bc78b985f2d2b1a13ce635dd69"),
        aad: &hex!("a1bc98dacec4b6aa7fee6dfa0802f21a"),
        ct: &hex!("3f6f4daf6d07743b9bd2a069d3710834"),
        tag: [0xb9c2b319, 0xadbd743f, 0x5e4ffd44, 0x304a1b5f],
    },
    Gcm {
        key: [0xfc1f971b, 0x514a1678, 0x65341b82, 0x8a4295d6],
        iv: [0x8851ea68, 0xd20ce0be, 0xff1e3a98],
        pt: &hex!("2fec17b1a9570f6651bbe9a657d82bce"),
        aad: &hex!("ece8d5f63aebda80ebde4b750637f654"),
        ct: &hex!("2d27e5fa08e218f02b2e36dfad87a50e"),
        tag: [0xeb996677, 0x4c588a31, 0xb71c4d8d, 0xaa495e9e],
    },
    Gcm {
        key: [0x00ef3c67, 0x62be3fba, 0xb38154d9, 0x02ff43b5],
        iv: [0xc3c1c307, 0x9cda49a7, 0x5a53b3cc],
        pt: &hex!("be425e008e9b0c083b19a2d945c2ede9"),
        aad: &hex!("714fa1d6904187b3c5c08a30dffc86e8"),
        ct: &hex!("c961a1758dcf91e539658372db18968e"),
        tag: [0xeaf9bda9, 0xb3322f50, 0x1f7329cb, 0x61c1c428],
    },
    Gcm {
        key: [0x2d70b956, 0x9943cc49, 0xcdef8495, 0xbdb6f0e6],
        iv: [0xb401d0f5, 0x0880a621, 0x1fde9d9c],
        pt: &hex!("47a87a387944f739bd3cb03e0e8be499"),
        aad: &hex!("592e7276bda066327f2b3cd8cc39f571"),
        ct: &hex!("c1b2af4d273231e71e7e066c206bf567"),
        tag: [0xc68d8d3c, 0xf8b89e6b, 0x15f623d6, 0x0fef60bd],
    },
    Gcm {
        key: [0x775cb7f8, 0xdc73f04f, 0xe4f9d221, 0x26bb7b57],
        iv: [0x81ceb17d, 0xeee19b81, 0x53ff927c],
        pt: &hex!("8242c6c0eed6d5d1ab69cd11dbe361d0"),
        aad: &hex!("97e07cd65065d1edc863192de98bc62c"),
        ct: &hex!("580f063ab1a4801d279e4ee773200abe"),
        tag: [0x29e4d7e0, 0x54a6b0a4, 0xe0113357, 0x3fbe632b],
    },
    Gcm {
        key: [0x58ba3cb7, 0xc0a0cf57, 0x75002bf3, 0xb112d051],
        iv: [0xbb923c93, 0xddca303a, 0xb131238d],
        pt: &hex!("6b93d2d92de05b53769ec398ab8097dc"),
        aad: &hex!("0898ea55c0ca0594806e2dc78be15c27"),
        ct: &hex!("d0564006b1897bf21922fef4f6386fd4"),
        tag: [0x3a92f3c9, 0xe3ae6b0c, 0x69dcb886, 0x8d4de27c],
    },
    Gcm {
        key: [0x955b761d, 0xe8e98f37, 0xacb41259, 0xfa308442],
        iv: [0xa103db8a, 0x0825e606, 0xb70427fc],
        pt: &hex!("d18344c86caffc4237d2daae47817b13"),
        aad: &hex!("c2d0d8b77a6fd03ced080e0f89de8a4b"),
        ct: &hex!("065d228c1289007a682aa847a36b6f30"),
        tag: [0xfb367f47, 0x922d67c8, 0x4bf47aab, 0xb2b98421],
    },
];

const GCM_256: [Gcm<8>; 60] = [
    // [Keylen = 256]
    // [IVlen = 96]
    // [PTlen = 0]
    // [AADlen = 0]
    // [Taglen = 128]
    Gcm {
        key: [
            0xb52c505a, 0x37d78eda, 0x5dd34f20, 0xc22540ea, 0x1b58963c, 0xf8e5bf8f, 0xfa85f9f2,
            0x492505b4,
        ],
        iv: [0x516c3392, 0x9df5a328, 0x4ff463d7],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xbdc1ac88, 0x4d332457, 0xa1d2664f, 0x168c76f0],
    },
    Gcm {
        key: [
            0x5fe0861c, 0xdc2690ce, 0x69b3658c, 0x7f26f845, 0x8eec1c92, 0x43c5ba08, 0x45305d89,
            0x7e96ca0f,
        ],
        iv: [0x770ac1a5, 0xa3d476d5, 0xd96944a1],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x196d691e, 0x1047093c, 0xa4b3d2ef, 0x4baba216],
    },
    Gcm {
        key: [
            0x7620b79b, 0x17b21b06, 0xd97019aa, 0x70e1ca10, 0x5e1c03d2, 0xa0cf8b20, 0xb5a0ce5c,
            0x3903e548,
        ],
        iv: [0x60f56eb7, 0xa4b38d4f, 0x03395511],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xf570c382, 0x02d94564, 0xbab39f75, 0x617bc87a],
    },
    Gcm {
        key: [
            0x7e2db003, 0x21189476, 0xd144c5f2, 0x7e787087, 0x302a48b5, 0xf7786cd9, 0x1e936416,
            0x28c2328b,
        ],
        iv: [0xea9d525b, 0xf01de7b2, 0x234b606a],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xdb9df5f1, 0x4f6c9f2a, 0xe81fd421, 0x412ddbbb],
    },
    Gcm {
        key: [
            0xa23dfb84, 0xb5976b46, 0xb1830d93, 0xbcf61941, 0xcae5e409, 0xe4f5551d, 0xc684bdce,
            0xf9876480,
        ],
        iv: [0x5aa34590, 0x8048de10, 0xa2bd3d32],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xf2821764, 0x9230bd7a, 0x40a9a4dd, 0xabc67c43],
    },
    Gcm {
        key: [
            0xdfe928f8, 0x6430b78a, 0xdd7bb769, 0x6023e615, 0x3d76977e, 0x56103b18, 0x0253490a,
            0xffb9431c,
        ],
        iv: [0x1dd0785a, 0xf9f58979, 0xa10bd62d],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xa55eb09e, 0x9edef58d, 0x9f671d72, 0x207f8b3c],
    },
    Gcm {
        key: [
            0x34048db8, 0x1591ee68, 0x224956bd, 0x6989e163, 0x0fcf068d, 0x7ff726ae, 0x81e5b29f,
            0x548cfcfb,
        ],
        iv: [0x1621d34c, 0xff2a5b25, 0x0c7b76fc],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x4992ec3d, 0x57cccfa5, 0x8fd8916c, 0x59b70b11],
    },
    Gcm {
        key: [
            0xa1114f87, 0x49c72b8c, 0xef62e750, 0x3f1ad921, 0xd33eeede, 0x32b0b5b8, 0xe0d6807a,
            0xa233d0ad,
        ],
        iv: [0xa190ed3f, 0xf2e238be, 0x56f90bd6],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xc8464d95, 0xd540fb19, 0x1156fbbc, 0x1608842a],
    },
    Gcm {
        key: [
            0xddbb99dc, 0x3102d311, 0x02c0e14b, 0x23851860, 0x5766c5b2, 0x3d9bea52, 0xc7c5a771,
            0x042c85a0,
        ],
        iv: [0x95d15ed7, 0x5c6a109a, 0xac1b1d86],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x813d1da3, 0x775cacd7, 0x8e96d86f, 0x036cff96],
    },
    Gcm {
        key: [
            0x1faa506b, 0x8f13a2e6, 0x660af78d, 0x92915adf, 0x333658f7, 0x48f4e48f, 0xa20135a2,
            0x9e9abe5f,
        ],
        iv: [0xe50f278d, 0x3662c99d, 0x750f60d3],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xaec7ece6, 0x6b7344af, 0xd6f6cc74, 0x19cf6027],
    },
    Gcm {
        key: [
            0xf30b5942, 0xfaf57d4c, 0x13e7a824, 0x95aedf1b, 0x4e603539, 0xb2e15993, 0x17cc6e53,
            0x225a2493,
        ],
        iv: [0x336c388e, 0x18e6abf9, 0x2bb739a9],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xddaf8ef4, 0xcb2f8a6d, 0x401f3be5, 0xff0baf6a],
    },
    Gcm {
        key: [
            0xdaf4d9c1, 0x2c5d29fc, 0x3fa93653, 0x2c96196e, 0x56ae842e, 0x47063a4b, 0x29bfff2a,
            0x35ed9280,
        ],
        iv: [0x5381f211, 0x97e093b9, 0x6cdac4fa],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x7f1832c7, 0xf7cd7812, 0xa004b79c, 0x3d399473],
    },
    Gcm {
        key: [
            0x6b524754, 0x149c8140, 0x1d29a4b8, 0xa6f4a478, 0x33372806, 0xb2d4083f, 0xf17f2db3,
            0xbfc17bca,
        ],
        iv: [0xac7d3d61, 0x8ab69055, 0x5ec24408],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0xdb07a885, 0xe2bd39da, 0x74116d06, 0xc316a5c9],
    },
    Gcm {
        key: [
            0xcff08330, 0x3ff40a1f, 0x66c4aed1, 0xac7f5062, 0x8fe7e931, 0x1f5d037e, 0xbf49f4a4,
            0xb9f0223f,
        ],
        iv: [0x45d46e1b, 0xaadcfbc8, 0xf0e922ff],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x1687c6d4, 0x59ea481b, 0xf88e4b22, 0x63227906],
    },
    Gcm {
        key: [
            0x3954f60c, 0xddbb39d2, 0xd8b058ad, 0xf545d5b8, 0x2490c8ae, 0x9283afa5, 0x27868904,
            0x1d415a3a,
        ],
        iv: [0x8fb3d98e, 0xf24fba03, 0x746ac84f],
        pt: &[],
        aad: &[],
        ct: &[],
        tag: [0x7fb13085, 0x5dfe7a37, 0x3313361f, 0x33f55237],
    },
    // [Keylen = 256]
    // [IVlen = 96]
    // [PTlen = 0]
    // [AADlen = 128]
    // [Taglen = 128]
    Gcm {
        key: [
            0x78dc4e0a, 0xaf52d935, 0xc3c01eea, 0x57428f00, 0xca1fd475, 0xf5da86a4, 0x9c8dd73d,
            0x68c8e223,
        ],
        iv: [0xd79cf22d, 0x504cc793, 0xc3fb6c8a],
        pt: &[],
        aad: &hex!("b96baa8c1c75a671bfb2d08d06be5f36"),
        ct: &[],
        tag: [0x3e5d486a, 0xa2e30b22, 0xe040b857, 0x23a06e76],
    },
    Gcm {
        key: [
            0x4457ff33, 0x683cca6c, 0xa493878b, 0xdc003738, 0x93a97634, 0x12eef8cd, 0xdb54f913,
            0x18e0da88,
        ],
        iv: [0x699d1f29, 0xd7b8c553, 0x00bb1fd2],
        pt: &[],
        aad: &hex!("6749daeea367d0e9809e2dc2f309e6e3"),
        ct: &[],
        tag: [0xd60c74d2, 0x517fde4a, 0x74e0cd47, 0x09ed43a9],
    },
    Gcm {
        key: [
            0x4d01c96e, 0xf9d98d4f, 0xb4e9b61b, 0xe5efa772, 0xc9788545, 0xb3eac39e, 0xb1cacb99,
            0x7a5f0792,
        ],
        iv: [0x32124a4d, 0x9e576aea, 0x2589f238],
        pt: &[],
        aad: &hex!("d72bad0c38495eda50d55811945ee205"),
        ct: &[],
        tag: [0x6d6397c9, 0xe2030f5b, 0x8053bfe5, 0x10f3f2cf],
    },
    Gcm {
        key: [
            0x8378193a, 0x4ce64180, 0x814bd605, 0x91d1054a, 0x04dbc4da, 0x02afde45, 0x3799cd68,
            0x88ee0c6c,
        ],
        iv: [0xbd8b4e35, 0x2c7f6987, 0x8a475435],
        pt: &[],
        aad: &hex!("1c6b343c4d045cbba562bae3e5ff1b18"),
        ct: &[],
        tag: [0x0833967a, 0x6a53ba24, 0xe75c0372, 0xa6a17bda],
    },
    Gcm {
        key: [
            0x22fc82db, 0x5b606998, 0xad45099b, 0x7978b5b4, 0xf9dd4ea6, 0x017e5737, 0x0ac56141,
            0xcaaabd12,
        ],
        iv: [0x880d05c5, 0xee599e5f, 0x151e302f],
        pt: &[],
        aad: &hex!("3e3eb5747e390f7bc80e748233484ffc"),
        ct: &[],
        tag: [0x2e122a47, 0x8e644632, 0x86f8b489, 0xdcdd09c8],
    },
    Gcm {
        key: [
            0xfc00960d, 0xdd698d35, 0x728c5ac6, 0x07596b51, 0xb3f89741, 0xd14c25b8, 0xbadac919,
            0x76120d99,
        ],
        iv: [0xa424a32a, 0x237f0df5, 0x30f05e30],
        pt: &[],
        aad: &hex!("cfb7e05e3157f0c90549d5c786506311"),
        ct: &[],
        tag: [0xdcdcb9e4, 0x004b852a, 0x0da12bdf, 0x255b4ddd],
    },
    Gcm {
        key: [
            0x69749943, 0x092f5605, 0xbf971e18, 0x5c191c61, 0x8261b2c7, 0xcc1693cd, 0xa1080ca2,
            0xfd8d5111,
        ],
        iv: [0xbd0d62c0, 0x2ee68206, 0x9bd1e128],
        pt: &[],
        aad: &hex!("6967dce878f03b643bf5cdba596a7af3"),
        ct: &[],
        tag: [0x378f796a, 0xe543e1b2, 0x9115cc18, 0xacd193f4],
    },
    Gcm {
        key: [
            0xfc4875db, 0x84819834, 0xb1cb4382, 0x8d2f0ae3, 0x473aa380, 0x111c2737, 0xe82a9ab1,
            0x1fea1f19,
        ],
        iv: [0xda6a684d, 0x3ff63a2d, 0x109decd6],
        pt: &[],
        aad: &hex!("91b6fa2ab4de44282ffc86c8cde6e7f5"),
        ct: &[],
        tag: [0x504e81d2, 0xe7877e4d, 0xad6f31cd, 0xeb07bdbd],
    },
    Gcm {
        key: [
            0x9f9fe7d2, 0xa26dcf59, 0xd684f1c0, 0x945b5ffa, 0xfe0a4746, 0x845ed317, 0xd35f3ed7,
            0x6c93044d,
        ],
        iv: [0x13b59971, 0xcd4dd36b, 0x19ac7104],
        pt: &[],
        aad: &hex!("190a6934f45f89c90067c2f62e04c53b"),
        ct: &[],
        tag: [0x4f636a29, 0x4bfbf51f, 0xc0e131d6, 0x94d5c222],
    },
    Gcm {
        key: [
            0xab9155d7, 0xd81ba6f3, 0x3193695c, 0xf4566a9b, 0x6e97a3e4, 0x09f57159, 0xae6ca496,
            0x55cca071,
        ],
        iv: [0x26a9f8d6, 0x65d163dd, 0xb92d035d],
        pt: &[],
        aad: &hex!("4a203ac26b951a1f673c6605653ec02d"),
        ct: &[],
        tag: [0x437ea77a, 0x3879f010, 0x691e288d, 0x6269a996],
    },
    Gcm {
        key: [
            0x0f1c62dd, 0x80b4a6d0, 0x9ee9d787, 0xb1b04327, 0xaa361529, 0xffa34075, 0x60414ac4,
            0x7b7ef7bc,
        ],
        iv: [0xc87613a3, 0xb70d2a04, 0x8f32cb9a],
        pt: &[],
        aad: &hex!("8f23d404be2d9e888d219f1b40aa29e8"),
        ct: &[],
        tag: [0x36d8a309, 0xacbb8716, 0xc9c08c7f, 0x5de4911e],
    },
    Gcm {
        key: [
            0xf3e954a3, 0x8956df89, 0x0255f017, 0x09e457b3, 0x3f4bfe7e, 0xcb36d0ee, 0x50f25004,
            0x71eebcde,
        ],
        iv: [0x9799abd3, 0xc52110c7, 0x04b0f36a],
        pt: &[],
        aad: &hex!("ddb70173f44157755b6c9b7058f40cb7"),
        ct: &[],
        tag: [0xb323ae3a, 0xbcb415c7, 0xf420876c, 0x980f4858],
    },
    Gcm {
        key: [
            0x06253165, 0x34fbd82f, 0xe8fdea50, 0xfa573c46, 0x2022c42f, 0x79e8b213, 0x60e5a6dc,
            0xe66dde28,
        ],
        iv: [0xda64a674, 0x907cd6cf, 0x248f5fbb],
        pt: &[],
        aad: &hex!("f24d48e04f5a0d987ba7c745b73b0364"),
        ct: &[],
        tag: [0xdf360b81, 0x0f27e794, 0x673a8bb2, 0xdc0d68b0],
    },
    Gcm {
        key: [
            0x28f045ac, 0x7c4fe5d4, 0xb01a9dcd, 0x5f1ad3ef, 0xff1c4f17, 0x0fc8ab87, 0x58d97292,
            0x868d5828,
        ],
        iv: [0x5d85de95, 0xb0bdc445, 0x14143919],
        pt: &[],
        aad: &hex!("601d2158f17ab3c7b4dcb6950fbdcdde"),
        ct: &[],
        tag: [0x42c3f527, 0x418cf2c3, 0xf5d5010c, 0xcba8f271],
    },
    Gcm {
        key: [
            0x19310eed, 0x5f5f44eb, 0x47075c10, 0x5eb31e36, 0xbbfd1310, 0xf741b9ba, 0xa66a8113,
            0x8d357242,
        ],
        iv: [0xa1247120, 0x138fa4f0, 0xe96c992c],
        pt: &[],
        aad: &hex!("29d746414333e0f72b4c3f44ec6bfe42"),
        ct: &[],
        tag: [0xd5997e2f, 0x956df3fa, 0x2c2388e2, 0x0f30c480],
    },
    // [Keylen = 256]
    // [IVlen = 96]
    // [PTlen = 128]
    // [AADlen = 0]
    // [Taglen = 128]
    Gcm {
        key: [
            0x31bdadd9, 0x6698c204, 0xaa9ce144, 0x8ea94ae1, 0xfb4a9a0b, 0x3c9d773b, 0x51bb1822,
            0x666b8f22,
        ],
        iv: [0x0d18e06c, 0x7c725ac9, 0xe362e1ce],
        pt: &hex!("2db5168e932556f8089a0622981d017d"),
        aad: &[],
        ct: &hex!("fa4362189661d163fcd6a56d8bf0405a"),
        tag: [0xd636ac1b, 0xbedd5cc3, 0xee727dc2, 0xab4a9489],
    },
    Gcm {
        key: [
            0x460fc864, 0x972261c2, 0x560e1eb8, 0x8761ff1c, 0x992b9824, 0x97bd2ac3, 0x6c04071c,
            0xbb8e5d99,
        ],
        iv: [0x8a4a16b9, 0xe210eb68, 0xbcb6f58d],
        pt: &hex!("99e4e926ffe927f691893fb79a96b067"),
        aad: &[],
        ct: &hex!("133fc15751621b5f325c7ff71ce08324"),
        tag: [0xec4e87e0, 0xcf74a136, 0x18d0b686, 0x36ba9fa7],
    },
    Gcm {
        key: [
            0xf78a2ba3, 0xc5bd164d, 0xe134a030, 0xca09e994, 0x63ea7e96, 0x7b92c4b0, 0xa0870796,
            0x480297e5,
        ],
        iv: [0x2bb92fcb, 0x726c278a, 0x2fa35a88],
        pt: &hex!("f562509ed139a6bbe7ab545ac616250c"),
        aad: &[],
        ct: &hex!("e2f787996e37d3b47294bf7ebba5ee25"),
        tag: [0x00f613ee, 0xe9bdad6c, 0x9ee7765d, 0xb1cb45c0],
    },
    Gcm {
        key: [
            0x48e6af21, 0x2da13865, 0x00454c94, 0xa201640c, 0x2151b280, 0x79240e40, 0xd72d2a5f,
            0xd7d54234,
        ],
        iv: [0xef0ff062, 0x220eb817, 0xdc2ece94],
        pt: &hex!("c7afeecec1408ad155b177c2dc7138b0"),
        aad: &[],
        ct: &hex!("9432a620e6a22307e06a321d66846fd4"),
        tag: [0xe3ea4991, 0x92f2cd8d, 0x3ab3edfc, 0x55897415],
    },
    Gcm {
        key: [
            0x79cd8d75, 0x0fc8ea62, 0xa2714edc, 0xd9b32867, 0xc7c4da90, 0x6c56e23a, 0x644552f5,
            0xb812e75a,
        ],
        iv: [0x9bbfdb81, 0x015d2b57, 0xdead2de5],
        pt: &hex!("f980ad8c55ebd31ee6f98f44e92bff55"),
        aad: &[],
        ct: &hex!("41a34d1e759c859e91b8cf5d3ded1970"),
        tag: [0x68cd9840, 0x6d5b3225, 0x71e750c3, 0x0aa49834],
    },
    Gcm {
        key: [
            0x130ae450, 0xc18efb85, 0x1057aaa7, 0x9575a0a0, 0x90194be8, 0xb2c95469, 0xa0e8e380,
            0xa8f48f42,
        ],
        iv: [0xb2691153, 0x96f81b39, 0xe0c38f47],
        pt: &hex!("036cf36280dee8355c82abc4c1fdb778"),
        aad: &[],
        ct: &hex!("09f7568fd8181652e556f0dda5a49ed5"),
        tag: [0xd10b6194, 0x7cae275b, 0x7034f525, 0x9ba6fc28],
    },
    Gcm {
        key: [
            0x9c712128, 0x9aefc670, 0x90cabed5, 0x3ad11658, 0xbe72a537, 0x2761b9d7, 0x35e81d2b,
            0xfc0e3267,
        ],
        iv: [0xade1702d, 0x2051b8dd, 0x203b5419],
        pt: &hex!("b95bcaa2b31403d76859a4c301c50b56"),
        aad: &[],
        ct: &hex!("628285e6489090dde1b9a60674785003"),
        tag: [0x9f516af3, 0xf3b93d61, 0x0edbc5ba, 0x6e2d115f],
    },
    Gcm {
        key: [
            0x0400b428, 0x97011fc2, 0x0fd2280a, 0x52ef905d, 0x6ebf1b05, 0x5b48c970, 0x67bd786d,
            0x678ec4ea,
        ],
        iv: [0x0abfb0a4, 0x1496b453, 0x358409d9],
        pt: &hex!("20c8230191e35f4e9b269d59cf5521f6"),
        aad: &[],
        ct: &hex!("dd8c38087daffbbb3ebb57ebf5ee5f78"),
        tag: [0xbfb07aa5, 0x049ee350, 0xec6fb139, 0x7f37087b],
    },
    Gcm {
        key: [
            0x56690798, 0x978c154f, 0xf250ba78, 0xe463765f, 0x2f0ce697, 0x09a4551b, 0xd8cb3add,
            0xeda087b6,
        ],
        iv: [0xcf37c286, 0xc18ad4ea, 0x3d0ba6a0],
        pt: &hex!("2d328124a8d58d56d0775eed93de1a88"),
        aad: &[],
        ct: &hex!("3b0a0267f6ecde3a78b30903ebd4ca6e"),
        tag: [0x1fd20064, 0x09fc6363, 0x79f3d406, 0x7eca0988],
    },
    Gcm {
        key: [
            0x8a02a33b, 0xdf87e784, 0x5d7a8ae3, 0xc8727e70, 0x4f4fd08c, 0x1f208328, 0x2d8cb3a5,
            0xd3cedee9,
        ],
        iv: [0x599f5896, 0x851c968e, 0xd808323b],
        pt: &hex!("4ade8b32d56723fb8f65ce40825e27c9"),
        aad: &[],
        ct: &hex!("cb9133796b9075657840421a46022b63"),
        tag: [0xa79e453c, 0x6fad8a5a, 0x4c2a8e87, 0x821c7f88],
    },
    Gcm {
        key: [
            0x23aaa78a, 0x5915b14f, 0x00cf285f, 0x38ee275a, 0x2db97cb4, 0xab14d1aa, 0xc8b9a73f,
            0xf1e66467,
        ],
        iv: [0x4a675ec9, 0xbe1aab96, 0x32dd9f59],
        pt: &hex!("56659c06a00a2e8ed1ac60572eee3ef7"),
        aad: &[],
        ct: &hex!("e6c01723bfbfa398d9c9aac8c683bb12"),
        tag: [0x4a2f78a9, 0x975d4a1b, 0x5f503a4a, 0x2cb71553],
    },
    Gcm {
        key: [
            0xfe647f72, 0xe95c4690, 0x27f4d777, 0x8429a2e8, 0xe90d0902, 0x68d4fa7d, 0xf44f65c0,
            0xaf84190a,
        ],
        iv: [0x4f40ae2a, 0x83a9b480, 0xe4686c90],
        pt: &hex!("31fd6cce3f0d2b0d18e0af01c4b5609e"),
        aad: &[],
        ct: &hex!("54c769fd542f0d3022f1335a7c410b61"),
        tag: [0x106cb7cb, 0xcd967da6, 0xcad64603, 0x9c753474],
    },
    Gcm {
        key: [
            0xfce20551, 0x5f0551b1, 0x797128a2, 0x132d8e00, 0x2ea5ab1b, 0xeb99c5e7, 0xe8329398,
            0xcf478e10,
        ],
        iv: [0x20209a0d, 0x4a3b9bfd, 0xdeef39a0],
        pt: &hex!("7d663e31a2f6ffef17e536684dae2e87"),
        aad: &[],
        ct: &hex!("6529712030fb659dc11ab719f6a4c402"),
        tag: [0x58699464, 0xd062aba5, 0x05508c57, 0x6c4e07dd],
    },
    Gcm {
        key: [
            0xcd33003f, 0xf18f6f33, 0x69dd9a35, 0x381261ba, 0x660ce0a7, 0x69864475, 0x152e6770,
            0x66540337,
        ],
        iv: [0x20bffe90, 0x64ce76d2, 0x75204138],
        pt: &hex!("acaf53d4dd2fe12cd44450b0d9adcc92"),
        aad: &[],
        ct: &hex!("a669fda0444b180165f90815dc992b33"),
        tag: [0x6e31f5a5, 0x6c4790ce, 0xdcc2368c, 0x51d0639b],
    },
    Gcm {
        key: [
            0x381873b5, 0xf9579d82, 0x41f0c61f, 0x0d9e327b, 0xb9f67869, 0x1714aaa4, 0x8ea7d926,
            0x78d43fe7,
        ],
        iv: [0x3fc8bec2, 0x3603158e, 0x012d65e5],
        pt: &hex!("7b622e9b408fe91f6fa800ecef838d36"),
        aad: &[],
        ct: &hex!("8ca4de5b4e2ab22431a009f3ddd01bae"),
        tag: [0xb3a7f80e, 0x3edf3226, 0x22731550, 0x164cd747],
    },
    // [Keylen = 256]
    // [IVlen = 96]
    // [PTlen = 128]
    // [AADlen = 128]
    // [Taglen = 128]
    Gcm {
        key: [
            0x92e11dcd, 0xaa866f5c, 0xe790fd24, 0x501f9250, 0x9aacf4cb, 0x8b1339d5, 0x0c9c1240,
            0x935dd08b,
        ],
        iv: [0xac93a1a6, 0x145299bd, 0xe902f21a],
        pt: &hex!("2d71bcfa914e4ac045b2aa60955fad24"),
        aad: &hex!("1e0889016f67601c8ebea4943bc23ad6"),
        ct: &hex!("8995ae2e6df3dbf96fac7b7137bae67f"),
        tag: [0xeca5aa77, 0xd51d4a0a, 0x14d9c51e, 0x1da474ab],
    },
    Gcm {
        key: [
            0x7da3bcca, 0xffb34641, 0x78ca7c72, 0x2379836d, 0xb50ce0bf, 0xb47640b9, 0x57216386,
            0x5332e486,
        ],
        iv: [0xc04fd2e7, 0x01c3dc62, 0xb68738b3],
        pt: &hex!("fd671cab1ee21f0df6bb610bf94f0e69"),
        aad: &hex!("fec0311013202e4ffdc4204926ae0ddf"),
        ct: &hex!("6be61b17b7f7d494a7cdf270562f37ba"),
        tag: [0x5e702a38, 0x323fe116, 0x0b780d17, 0xadad3e96],
    },
    Gcm {
        key: [
            0xa359b958, 0x4beec189, 0x527f8842, 0xdda6b6d4, 0xc6a5db2f, 0x88963571, 0x5fa3bcd7,
            0x967c0a71,
        ],
        iv: [0x8616c4cd, 0xe11b34a9, 0x44caba32],
        pt: &hex!("33a46b7539d64c6e1bdb91ba221e3007"),
        aad: &hex!("e1796fca20cb3d3ab0ade69b2a18891e"),
        ct: &hex!("b0d316e95f3f3390ba10d0274965c62b"),
        tag: [0xaeaedcf8, 0xa012cc32, 0xef25a627, 0x90e9334c],
    },
    Gcm {
        key: [
            0x8c83238e, 0x7b3b5827, 0x8200b549, 0x40d779d0, 0xa0750673, 0xaab0bf2f, 0x5808dd15,
            0xdc1a8c49,
        ],
        iv: [0x70f8f4eb, 0xe408f61a, 0x35077956],
        pt: &hex!("6e57f8572dd5b2247410f0d4c7424186"),
        aad: &hex!("e1cbf83924f1b8d1014b97db56c25a15"),
        ct: &hex!("4a11acb9611251df01f79f16f8201ffb"),
        tag: [0x9732be4a, 0xd0569586, 0x753d90fa, 0xbb06f62c],
    },
    Gcm {
        key: [
            0xfe21919b, 0xb320af87, 0x44c9e862, 0xb5b7cf8b, 0x81ad3ad1, 0xfb0e7d7d, 0x710a688d,
            0x3eed154b,
        ],
        iv: [0x38bc3917, 0xaa1925f4, 0x0850c082],
        pt: &hex!("aea53b1ea79a71c3a4b83c92a0c979f1"),
        aad: &hex!("f24102fa7e6b819bb3ff47f90844db9c"),
        ct: &hex!("2fb8b697bf8f7a2eea25fe702a3ae0a9"),
        tag: [0x5be77e82, 0x7737ad7c, 0x4f79e0e3, 0x43fe010d],
    },
    Gcm {
        key: [
            0x499e8a3f, 0x39ac4abc, 0x62dd4e1a, 0x6133042e, 0x74785972, 0xb6b501bf, 0xaffefc8b,
            0xb29fd312,
        ],
        iv: [0x5c728dbb, 0xef9dcc0f, 0xf483e891],
        pt: &hex!("b44014c7fc6b3f15d126a881fbe2bd2b"),
        aad: &hex!("82300dab592f840ae991efa3623a6203"),
        ct: &hex!("578fe5e1aef7619f392c027c838a239e"),
        tag: [0x49fdc724, 0xf05eb56e, 0xa9e3fd14, 0xb61ad567],
    },
    Gcm {
        key: [
            0x2775d3e7, 0xa8fc665b, 0xb9a59edc, 0x22eb136a, 0xdd194824, 0xed8f2adb, 0x44917740,
            0x4c739716,
        ],
        iv: [0x73f16c05, 0x4e166696, 0xdf679a2e],
        pt: &hex!("c9f3bce40310b6c0a3fd62742e4f3617"),
        aad: &hex!("23199a1c9b7244913952ca4f7e7444f4"),
        ct: &hex!("72c85c10756266d00a9a4340b2cb3137"),
        tag: [0x5881e456, 0x5b42394e, 0x62d5daf0, 0xd1ebc593],
    },
    Gcm {
        key: [
            0x425a341c, 0x67e6d873, 0x870f54e2, 0xcc5a2984, 0xc734e817, 0x29c0dbaa, 0xeee05030,
            0x9f1ce674,
        ],
        iv: [0x0c09b7b4, 0xe9e09731, 0x7b791433],
        pt: &hex!("76dda644b3faca509b37def0319f30cc"),
        aad: &hex!("4300a721547846761e4bf8df2b6ec1d6"),
        ct: &hex!("1dd80daa0fc9e47e43897c64a6663f5e"),
        tag: [0x5d69b34d, 0x8c3b12f7, 0x83faaea7, 0xe93685db],
    },
    Gcm {
        key: [
            0xdd5c4898, 0x8a6e9f9f, 0x60be801b, 0xa5c090f2, 0x24a1b53d, 0x6601ec58, 0x58eab7b7,
            0x784a8d5e,
        ],
        iv: [0x43562d48, 0xcd4110a6, 0x6d9ca64e],
        pt: &hex!("2cda2761fd0be2b03f9714fce8d0e303"),
        aad: &hex!("55e568309fc6cb0fb0e0e7d2511d4116"),
        ct: &hex!("f2cfb6f5446e7aa172adfcd66b92a98d"),
        tag: [0xe099c64d, 0x2966e780, 0xce7d2eaa, 0xe97f47d8],
    },
    Gcm {
        key: [
            0x2bdad9c3, 0xe5de6e4e, 0x101b7f16, 0xe727c690, 0xdb95eacf, 0x4b0ccbde, 0xc7aab6fb,
            0x9fc80486,
        ],
        iv: [0xa5cf3967, 0xd244074d, 0x2153c576],
        pt: &hex!("84c867ec36cc6fe3487f5192fdfd390b"),
        aad: &hex!("6bdae72b5ed0e4d1f10064ebd02cf85c"),
        ct: &hex!("53c8fa437c1b5fa91abbd6508b3878ce"),
        tag: [0x7859593d, 0x127324be, 0x8b9cf1d4, 0x3ead4d82],
    },
    Gcm {
        key: [
            0x01e92afd, 0xb5d956be, 0x12d38b09, 0x252966c5, 0x728d26f3, 0xc72e54bb, 0x62bbc55a,
            0xe590e716,
        ],
        iv: [0x886e5536, 0x4eeb90e8, 0x7ac79bbe],
        pt: &hex!("6c6570385f3d6d937e54a3a2e95bc9eb"),
        aad: &hex!("c76aabb7f44b942a81feb50249d2131a"),
        ct: &hex!("423b749a507f437b431114962180d352"),
        tag: [0x54d85932, 0x0a492813, 0x68297da7, 0xd4e37326],
    },
    Gcm {
        key: [
            0x46921319, 0x217598cb, 0x64256fe4, 0x9abca1f1, 0x8a9d1dbc, 0xa360f863, 0x0afb5c61,
            0x37cb42b5,
        ],
        iv: [0x290827cf, 0x98141576, 0x0ec3b37a],
        pt: &hex!("480d32b191c2e201aed03680f93ea2da"),
        aad: &hex!("535ee80b12f581baaf8027e6e3900e31"),
        ct: &hex!("89ace4f73583fb1ac260dea99b54055e"),
        tag: [0x7b8b8358, 0x363c175a, 0x66e6fb48, 0xd1bc2222],
    },
    Gcm {
        key: [
            0xe18cd9b0, 0x1b59bc0d, 0xe1502efb, 0x74c36429, 0x97fe7dfb, 0x8d80c8a7, 0x3caffe77,
            0x26807d33,
        ],
        iv: [0xbd087b38, 0x4c40841b, 0x3839ba02],
        pt: &hex!("62f7f3a12b8c5f6747fcfe192d850b19"),
        aad: &hex!("fe69f837961b1d83f27fbf68e6791a1c"),
        ct: &hex!("bacfccf6397424e96caf761e71dd3e3a"),
        tag: [0x9c9a5b65, 0x420f83e7, 0x66c7c051, 0x680e8e58],
    },
    Gcm {
        key: [
            0x68ee463b, 0x3153d9a0, 0x42e5e368, 0x5def6f90, 0xf7659a20, 0x3441de33, 0x7fb94831,
            0xcbeae9b2,
        ],
        iv: [0x9c4a9254, 0xc485236c, 0xf838de7e],
        pt: &hex!("73731054514f3fb0102c7a1df809f212"),
        aad: &hex!("d55820e7acbb27d23c7df32938cf7d42"),
        ct: &hex!("13b7823cac37f40eb811e3c966d16a67"),
        tag: [0x76288c33, 0xa66ff645, 0x1e2cec6c, 0x4ba4935e],
    },
    Gcm {
        key: [
            0x64bd594d, 0xaf279e31, 0x72f9aa71, 0x3b35b7fc, 0xe8f43083, 0x792bc7d1, 0xf1091913,
            0x1f400a7b,
        ],
        iv: [0x339a2c40, 0xe9d9507c, 0x34228649],
        pt: &hex!("2b794cb4c98450463a3e225ab33f3f30"),
        aad: &hex!("2b9544807b362ebfd88146e2b02c9270"),
        ct: &hex!("434d703b8d1069ad8036288b7c2d1ae6"),
        tag: [0x7d31e397, 0xc0c943cb, 0xb16cfb95, 0x39a6a17d],
    },
];

const NUM_ECB_128: u32 = (ECB_PT_CT_128.len() + ECB_KEY_CT_128.len()) as u32;
const NUM_ECB_256: u32 = (ECB_PT_CT_256.len() + ECB_KEY_CT_256.len()) as u32;
const NUM_GCM_128: u32 = GCM_128.len() as u32;
const NUM_GCM_256: u32 = GCM_256.len() as u32;

const FREQ: u32 = 48_000_000;
const CYC_PER_US: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:us}", DWT::cycle_count() / CYC_PER_US);

const ZERO_16B: [u32; 4] = [0; 4];
const ZERO_32B: [u32; 8] = [0; 8];

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

fn align_bytes(bytes: &[u8], buf: &mut [u32]) -> usize {
    const CHUNK_SIZE: usize = size_of::<u32>();
    defmt::assert_eq!(bytes.len() % CHUNK_SIZE, 0);
    defmt::assert!(buf.len() * CHUNK_SIZE >= bytes.len());

    bytes
        .chunks_exact(CHUNK_SIZE)
        .enumerate()
        .for_each(|(idx, chunk)| buf[idx] = u32::from_be_bytes(unwrap!(chunk.try_into().ok())));

    bytes.len() / CHUNK_SIZE
}

fn half_word_swap<const N: usize>(input: [u32; N]) -> [u32; N] {
    let mut buf: [u32; N] = [0; N];
    for idx in 0..N {
        buf[idx] = (input[idx] & 0x0000_FFFF) << 16 | (input[idx] & 0xFFFF_0000) >> 16;
    }
    buf
}

fn byte_swap<const N: usize>(input: [u32; N]) -> [u32; N] {
    let mut buf: [u32; N] = [0; N];
    for idx in 0..N {
        buf[idx] = input[idx].swap_bytes();
    }
    buf
}

fn bit_swap<const N: usize>(input: [u32; N]) -> [u32; N] {
    let mut buf: [u32; N] = [0; N];
    for idx in 0..N {
        buf[idx] = {
            let mut v: u32 = input[idx];
            let mut r: u32 = input[idx];

            let mut s: usize = 8 * core::mem::size_of::<u32>() - 1;

            v >>= 1;
            while v != 0 {
                r <<= 1;
                r |= v & 1;
                v >>= 1;
                s -= 1;
            }

            r << s
        };
    }
    buf
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Aes {
        defmt::info!("init");
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());

        cortex_m::interrupt::free(|cs| unsafe {
            rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs)
        });
        defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);

        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        cp.DWT.set_cycle_count(0);

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
            total_elapsed / NUM_ECB_128
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
            total_elapsed / NUM_ECB_256
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
            total_elapsed / NUM_ECB_128
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
            total_elapsed / NUM_ECB_256
        );
    }

    #[test]
    fn encrypt_gcm_inplace_128(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for gcm in GCM_128.iter() {
            let mut tag: [u32; 4] = [0; 4];
            let mut buf: [u8; 16] = [0; 16];
            defmt::assert!(buf.len() >= gcm.pt.len());
            gcm.pt
                .iter()
                .enumerate()
                .for_each(|(idx, &byte)| buf[idx] = byte);

            total_elapsed += stopwatch(|| {
                unwrap!(aes.encrypt_gcm_inplace(
                    &gcm.key,
                    &gcm.iv,
                    &gcm.aad,
                    &mut buf[..gcm.pt.len()],
                    &mut tag
                ))
            });

            defmt::assert_eq!(tag, gcm.tag);
            defmt::assert_eq!(&buf[..gcm.pt.len()], gcm.ct);
        }

        defmt::info!(
            "Approximate cycles per 128-bit encrypt: {}",
            total_elapsed / NUM_GCM_128
        );
    }

    // included for benchmarking reference
    #[test]
    fn soft_encrypt_gcm_inplace_128() {
        use aes_gcm::{AeadInPlace, KeyInit, Nonce};

        let mut total_elapsed: u32 = 0;

        for gcm in GCM_128.iter() {
            let mut buf: [u8; 16] = [0; 16];
            defmt::assert!(buf.len() >= gcm.pt.len());
            gcm.pt
                .iter()
                .enumerate()
                .for_each(|(idx, &byte)| buf[idx] = byte);

            let mut key: [u8; 16] = [0; 16];
            let mut iv: [u8; 12] = [0; 12];
            let mut tag: [u8; 16] = [0; 16];

            gcm.key.iter().enumerate().for_each(|(idx, dw)| {
                key[(idx * 4)..((idx + 1) * 4)].copy_from_slice(&dw.to_be_bytes())
            });
            gcm.iv.iter().enumerate().for_each(|(idx, dw)| {
                iv[(idx * 4)..((idx + 1) * 4)].copy_from_slice(&dw.to_be_bytes())
            });
            gcm.tag.iter().enumerate().for_each(|(idx, dw)| {
                tag[(idx * 4)..((idx + 1) * 4)].copy_from_slice(&dw.to_be_bytes())
            });

            let start: u32 = DWT::cycle_count();
            let cipher = aes_gcm::Aes128Gcm::new(key.as_ref().into());
            let nonce = Nonce::from_slice(iv.as_ref().into());
            let result_tag = unwrap!(cipher
                .encrypt_in_place_detached(nonce, &gcm.aad, &mut buf[..gcm.pt.len()])
                .ok());
            total_elapsed += DWT::cycle_count().wrapping_sub(start);

            let result_tag: [u8; 16] = unwrap!(result_tag.try_into());

            defmt::assert_eq!(result_tag, tag);

            defmt::assert_eq!(&buf[..gcm.pt.len()], gcm.ct);
        }

        defmt::info!(
            "Approximate cycles per 128-bit encrypt: {}",
            total_elapsed / NUM_GCM_128
        );
    }

    #[test]
    fn encrypt_gcm_inplace_u32_128(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for gcm in GCM_128.iter() {
            let mut tag: [u32; 4] = [0; 4];
            let mut buf: [u32; 4] = [0; 4];
            let mut aad: [u32; 4] = [0; 4];
            let mut expected: [u32; 4] = [0; 4];
            let pt_len: usize = align_bytes(gcm.pt, &mut buf);
            let aad_len: usize = align_bytes(gcm.aad, &mut aad);
            align_bytes(gcm.ct, &mut expected);

            total_elapsed += stopwatch(|| {
                unwrap!(aes.encrypt_gcm_inplace_u32(
                    &gcm.key,
                    &gcm.iv,
                    &aad[..aad_len],
                    &mut buf[..pt_len],
                    &mut tag
                ))
            });

            defmt::assert_eq!(tag, gcm.tag);
            defmt::assert_eq!(buf, expected);
        }

        defmt::info!(
            "Approximate cycles per 128-bit encrypt: {}",
            total_elapsed / NUM_GCM_128
        );
    }

    #[test]
    fn encrypt_gcm_inplace_256(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for gcm in GCM_256.iter() {
            let mut tag: [u32; 4] = [0; 4];
            let mut buf: [u8; 16] = [0; 16];
            defmt::assert!(buf.len() >= gcm.pt.len());
            gcm.pt
                .iter()
                .enumerate()
                .for_each(|(idx, &byte)| buf[idx] = byte);

            total_elapsed += stopwatch(|| {
                unwrap!(aes.encrypt_gcm_inplace(
                    &gcm.key,
                    &gcm.iv,
                    &gcm.aad,
                    &mut buf[..gcm.pt.len()],
                    &mut tag
                ))
            });

            defmt::assert_eq!(tag, gcm.tag);
            defmt::assert_eq!(&buf[..gcm.pt.len()], gcm.ct);
        }

        defmt::info!(
            "Approximate cycles per 256-bit encrypt: {}",
            total_elapsed / NUM_GCM_256
        );
    }
    #[test]
    fn encrypt_gcm_inplace_u32_256(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for gcm in GCM_256.iter() {
            let mut tag: [u32; 4] = [0; 4];
            let mut buf: [u32; 4] = [0; 4];
            let mut aad: [u32; 4] = [0; 4];
            let mut expected: [u32; 4] = [0; 4];
            let pt_len: usize = align_bytes(gcm.pt, &mut buf);
            let aad_len: usize = align_bytes(gcm.aad, &mut aad);
            align_bytes(gcm.ct, &mut expected);

            total_elapsed += stopwatch(|| {
                unwrap!(aes.encrypt_gcm_inplace_u32(
                    &gcm.key,
                    &gcm.iv,
                    &aad[..aad_len],
                    &mut buf[..pt_len],
                    &mut tag
                ))
            });

            defmt::assert_eq!(tag, gcm.tag);
            defmt::assert_eq!(buf, expected);
        }

        defmt::info!(
            "Approximate cycles per 256-bit encrypt: {}",
            total_elapsed / NUM_GCM_256
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
            total_elapsed / NUM_ECB_128
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
            total_elapsed / NUM_ECB_128
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
            total_elapsed / NUM_ECB_256
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
            total_elapsed / NUM_ECB_256
        );
    }

    #[test]
    fn decrypt_gcm_inplace_128(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for gcm in GCM_128.iter() {
            let mut tag: [u32; 4] = [0; 4];
            let mut buf: [u8; 16] = [0; 16];
            defmt::assert!(buf.len() >= gcm.pt.len());
            gcm.ct
                .iter()
                .enumerate()
                .for_each(|(idx, &byte)| buf[idx] = byte);

            total_elapsed += stopwatch(|| {
                unwrap!(aes.decrypt_gcm_inplace(
                    &gcm.key,
                    &gcm.iv,
                    &gcm.aad,
                    &mut buf[..gcm.pt.len()],
                    &mut tag
                ))
            });

            defmt::assert_eq!(tag, gcm.tag);
            defmt::assert_eq!(&buf[..gcm.pt.len()], gcm.pt);
        }

        defmt::info!(
            "Approximate cycles per 128-bit decrypt: {}",
            total_elapsed / NUM_GCM_128
        );
    }

    #[test]
    fn decrypt_gcm_inplace_128_u32(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for gcm in GCM_128.iter() {
            let mut tag: [u32; 4] = [0; 4];
            let mut buf: [u32; 4] = [0; 4];
            let mut aad: [u32; 4] = [0; 4];
            let mut expected: [u32; 4] = [0; 4];
            let ct_len: usize = align_bytes(gcm.ct, &mut buf);
            let aad_len: usize = align_bytes(gcm.aad, &mut aad);
            align_bytes(gcm.pt, &mut expected);

            total_elapsed += stopwatch(|| {
                unwrap!(aes.decrypt_gcm_inplace_u32(
                    &gcm.key,
                    &gcm.iv,
                    &aad[..aad_len],
                    &mut buf[..ct_len],
                    &mut tag
                ))
            });

            defmt::assert_eq!(tag, gcm.tag);
            defmt::assert_eq!(buf, expected);
        }

        defmt::info!(
            "Approximate cycles per 128-bit decrypt: {}",
            total_elapsed / NUM_GCM_128
        );
    }

    #[test]
    fn decrypt_gcm_inplace_256(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for gcm in GCM_256.iter() {
            let mut tag: [u32; 4] = [0; 4];
            let mut buf: [u8; 16] = [0; 16];
            defmt::assert!(buf.len() >= gcm.pt.len());
            gcm.ct
                .iter()
                .enumerate()
                .for_each(|(idx, &byte)| buf[idx] = byte);

            total_elapsed += stopwatch(|| {
                unwrap!(aes.decrypt_gcm_inplace(
                    &gcm.key,
                    &gcm.iv,
                    &gcm.aad,
                    &mut buf[..gcm.pt.len()],
                    &mut tag
                ))
            });

            defmt::assert_eq!(tag, gcm.tag);
            defmt::assert_eq!(&buf[..gcm.pt.len()], gcm.pt);
        }

        defmt::info!(
            "Approximate cycles per 256-bit decrypt: {}",
            total_elapsed / NUM_GCM_256
        );
    }

    #[test]
    fn decrypt_gcm_inplace_256_u32(aes: &mut Aes) {
        let mut total_elapsed: u32 = 0;

        for gcm in GCM_256.iter() {
            let mut tag: [u32; 4] = [0; 4];
            let mut buf: [u32; 4] = [0; 4];
            let mut aad: [u32; 4] = [0; 4];
            let mut expected: [u32; 4] = [0; 4];
            let ct_len: usize = align_bytes(gcm.ct, &mut buf);
            let aad_len: usize = align_bytes(gcm.aad, &mut aad);
            align_bytes(gcm.pt, &mut expected);

            total_elapsed += stopwatch(|| {
                unwrap!(aes.decrypt_gcm_inplace_u32(
                    &gcm.key,
                    &gcm.iv,
                    &aad[..aad_len],
                    &mut buf[..ct_len],
                    &mut tag
                ))
            });

            defmt::assert_eq!(tag, gcm.tag);
            defmt::assert_eq!(buf, expected);
        }

        defmt::info!(
            "Approximate cycles per 256-bit decrypt: {}",
            total_elapsed / NUM_GCM_256
        );
    }

    #[test]
    fn gcm_inplace_normal_use(aes: &mut Aes) {
        const ASSOCIATED_DATA: &[u8; 13] = b"Hello, World!";
        const PLAINTEXT: &[u8; 445] = b"Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.";

        // purpose of this test is to check encryption on data that does not
        // match the native hardware size
        defmt::assert_ne!(ASSOCIATED_DATA.len() % 16, 0);
        defmt::assert_ne!(PLAINTEXT.len() % 16, 0);

        const IV: [u32; 3] = [0; 3];

        // static to prevent the stack from going crazy
        static mut BUF: [u8; 445] = *PLAINTEXT;
        let mut encrypt_tag: [u32; 4] = [0; 4];
        let encrypt_elapsed: u32 = stopwatch(|| {
            unwrap!(aes.encrypt_gcm_inplace(
                &ZERO_16B,
                &IV,
                ASSOCIATED_DATA,
                unsafe { &mut BUF },
                &mut encrypt_tag
            ))
        });
        defmt::info!(
            "Encrypting {} bytes: {} cycles",
            PLAINTEXT.len(),
            encrypt_elapsed
        );

        assert_ne!(unsafe { &BUF }, PLAINTEXT);
        defmt::assert_ne!(encrypt_tag, ZERO_16B);

        let mut decrypt_tag: [u32; 4] = [0; 4];
        let decrypt_elapsed: u32 = stopwatch(|| {
            unwrap!(aes.decrypt_gcm_inplace(
                &ZERO_16B,
                &IV,
                ASSOCIATED_DATA,
                unsafe { &mut BUF },
                &mut decrypt_tag
            ))
        });

        assert_eq!(unsafe { &BUF }, PLAINTEXT);
        defmt::assert_eq!(decrypt_tag, encrypt_tag);

        defmt::info!(
            "Decrypting {} bytes: {} cycles",
            PLAINTEXT.len(),
            decrypt_elapsed
        );
    }

    #[test]
    fn gcm_inplace_all_sizes(aes: &mut Aes) {
        const DATA: [u8; 16] = [0x55; 16];
        const IV: [u32; 3] = [0; 3];

        for x in 0..DATA.len() {
            defmt::debug!("{}-length buffer", x);
            let mut buf: [u8; 16] = DATA;
            let mut encrypt_tag: [u32; 4] = [0; 4];
            unwrap!(aes.encrypt_gcm_inplace(&ZERO_16B, &IV, &[], &mut buf[..x], &mut encrypt_tag));
            if x != 0 {
                defmt::assert_ne!(DATA[..x], buf[..x]);
            }
            let mut decrypt_tag: [u32; 4] = [0; 4];
            unwrap!(aes.decrypt_gcm_inplace(&ZERO_16B, &IV, &[], &mut buf[..x], &mut decrypt_tag));

            defmt::assert_eq!(encrypt_tag, decrypt_tag);
            defmt::assert_eq!(DATA[..x], buf[..x]);
        }

        for x in 0..DATA.len() {
            defmt::debug!("{}-length AAD", x);
            let mut encrypt_tag: [u32; 4] = [0; 4];
            unwrap!(aes.encrypt_gcm_inplace(&ZERO_16B, &IV, &DATA[..x], &mut [], &mut encrypt_tag));
            let mut decrypt_tag: [u32; 4] = [0; 4];
            unwrap!(aes.decrypt_gcm_inplace(&ZERO_16B, &IV, &DATA[..x], &mut [], &mut decrypt_tag));

            defmt::assert_eq!(encrypt_tag, decrypt_tag);
        }
    }

    #[test]
    fn ecb_half_word_swap(aes: &mut Aes) {
        let pt: [u32; 4] = half_word_swap(ECB_PT_CT_128[0].0);
        let ct: [u32; 4] = half_word_swap(ECB_PT_CT_128[0].1);

        aes.set_dataswap(SwapMode::HalfWord);

        let mut output_ciphertext: [u32; 4] = [0; 4];
        unwrap!(aes.encrypt_ecb(&ZERO_16B, &pt, &mut output_ciphertext));

        defmt::assert_eq!(output_ciphertext, ct);
    }

    #[test]
    fn ecb_byte_swap(aes: &mut Aes) {
        let pt: [u32; 4] = byte_swap(ECB_PT_CT_128[0].0);
        let ct: [u32; 4] = byte_swap(ECB_PT_CT_128[0].1);

        aes.set_dataswap(SwapMode::Byte);

        let mut output_ciphertext: [u32; 4] = [0; 4];
        unwrap!(aes.encrypt_ecb(&ZERO_16B, &pt, &mut output_ciphertext));

        defmt::assert_eq!(output_ciphertext, ct);
    }

    #[test]
    fn ecb_bit_swap(aes: &mut Aes) {
        let pt: [u32; 4] = bit_swap(ECB_PT_CT_128[0].0);
        let ct: [u32; 4] = bit_swap(ECB_PT_CT_128[0].1);

        aes.set_dataswap(SwapMode::Bit);

        let mut output_ciphertext: [u32; 4] = [0; 4];
        unwrap!(aes.encrypt_ecb(&ZERO_16B, &pt, &mut output_ciphertext));

        defmt::assert_eq!(output_ciphertext, ct);
    }

    #[test]
    fn aes_wrap_clk() {
        let mut dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
        unsafe { Aes::pulse_reset(&mut dp.RCC) };
        unsafe { Aes::disable_clock(&mut dp.RCC) };

        let mut aeswrap: AesWrapClk = unsafe { Aes::new_no_init(dp.AES) }.into();

        for (plaintext, ciphertext) in ECB_PT_CT_128.iter() {
            let mut output_plaintext: [u32; 4] = [0; 4];
            unwrap!(aeswrap.with_clk(&mut dp.RCC, |aes| aes.decrypt_ecb(
                &ZERO_16B,
                ciphertext,
                &mut output_plaintext
            )));
            defmt::assert_eq!(&output_plaintext, plaintext);
        }

        for (key, ciphertext) in ECB_KEY_CT_128.iter() {
            let mut output_plaintext: [u32; 4] = [0; 4];

            unwrap!(aeswrap.with_clk(&mut dp.RCC, |aes| aes.decrypt_ecb(
                key,
                ciphertext,
                &mut output_plaintext
            )));
            defmt::assert_eq!(output_plaintext, ZERO_16B);
        }
    }
}
