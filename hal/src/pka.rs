//! Public key accelerator

use crate::pac;

use core::{
    mem::size_of,
    ptr::{read_volatile, write_volatile},
    sync::atomic::{compiler_fence, Ordering::SeqCst},
};

/// PKA errors.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum Error {
    /// Address access is out of range (unmapped address).
    Address,
    /// An AHB access to the PKA RAM occurred while the PKA core was computing
    /// and using its internal RAM.
    /// (AHB PKA_RAM access are not allowed while PKA operation is in progress).
    Ram,
}

/// Errors from an ECDSA signing operation.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum EcdsaSignError {
    /// Address access is out of range (unmapped address).
    Address,
    /// An AHB access to the PKA RAM occurred while the PKA core was computing
    /// and using its internal RAM.
    /// (AHB PKA_RAM access are not allowed while PKA operation is in progress).
    Ram,
    /// Signature part R is equal to 0.
    Rzero,
    /// Signature part S is equal to 0.
    Szero,
    /// Unknown result code.
    Unknown {
        /// Unknown result code bits.
        bits: u32,
    },
}

impl From<Error> for EcdsaSignError {
    fn from(pka: Error) -> Self {
        match pka {
            Error::Address => EcdsaSignError::Address,
            Error::Ram => EcdsaSignError::Ram,
        }
    }
}

/// Errors from an ECDSA verify operation.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum EcdsaVerifyError {
    /// Address access is out of range (unmapped address).
    Address,
    /// An AHB access to the PKA RAM occurred while the PKA core was computing
    /// and using its internal RAM.
    /// (AHB PKA_RAM access is not allowed while a PKA operation is in progress).
    Ram,
    /// Invalid signature.
    Invalid,
}

impl From<Error> for EcdsaVerifyError {
    fn from(pka: Error) -> Self {
        match pka {
            Error::Address => EcdsaVerifyError::Address,
            Error::Ram => EcdsaVerifyError::Ram,
        }
    }
}

/// PKA operation codes.
#[derive(Debug)]
#[repr(u8)]
#[allow(dead_code)]
enum PkaOpcode {
    /// Montgomery parameter computation then modular exponentiation.
    MontgomeryParameterExponentiation = 0b000000,
    /// Montgomery parameter computation only.
    MontgomeryParameter = 0b000001,
    /// Modular exponentiation only (Montgomery parameter must be loaded first).
    ModularExponentiation = 0b000010,
    /// Montgomery parameter computation then ECC scalar multiplication.
    MontgomeryParameterEcc = 0b100000,
    /// ECC scalar multiplication only (Montgomery parameter must be loaded first).
    EccScalar = 0b100010,
    /// ECDSA signing.
    EcdsaSign = 0b100100,
    /// ECDSA verification.
    EcdsaVerify = 0b100110,
    /// Point on elliptic curve Fp check.
    Point = 0b101000,
    /// RSA CRT exponentiation.
    RsaCrt = 0b000111,
    /// Modular inversion.
    ModularInversion = 0b001000,
    /// Arithmetic addition.
    ArithmeticAdd = 0b001001,
    /// Arithmetic subtraction.
    ArithmeticSub = 0b001010,
    /// Arithmetic multiplication.
    ArithmeticMul = 0b001011,
    /// Arithmetic comparison.
    ArithmeticCmp = 0b001100,
    /// Modular reduction.
    ModularRed = 0b001101,
    /// Modular addition.
    ModularAdd = 0b001110,
    /// Modular subtraction.
    ModularSub = 0b001111,
    /// Montgomery multiplication.
    MontgomeryMul = 0b010000,
}

impl From<PkaOpcode> for u8 {
    fn from(x: PkaOpcode) -> Self {
        x as u8
    }
}

/// PKA driver.
///
/// Created with [`Pka::new`].
pub struct Pka {
    pka: pac::PKA,
}

impl Pka {
    const BASE: usize = 0x5800_2000;
    const RAM_BASE: usize = Self::BASE + 0x400;
    const RAM_NUM_DW: usize = 894;

    // ECDSA sign input addresses
    const ECDSA_SIGN_N_LEN: usize = Self::BASE + 0x400;
    const ECDSA_SIGN_P_LEN: usize = Self::BASE + 0x404;
    const ECDSA_SIGN_A_SIGN: usize = Self::BASE + 0x408;
    const ECDSA_SIGN_A: usize = Self::BASE + 0x40C;
    const ECDSA_SIGN_P: usize = Self::BASE + 0x460;
    const ECDSA_SIGN_K: usize = Self::BASE + 0x508;
    const ECDSA_SIGN_X: usize = Self::BASE + 0x55C;
    const ECDSA_SIGN_Y: usize = Self::BASE + 0x5B0;
    const ECDSA_SIGN_Z: usize = Self::BASE + 0xDE8;
    const ECDSA_SIGN_D: usize = Self::BASE + 0xE3C;
    const ECDSA_SIGN_N: usize = Self::BASE + 0xE94;

    // ECDSA sign output addresses
    const ECDSA_SIGN_OUT_R: usize = Self::BASE + 0x700;
    const ECDSA_SIGN_OUT_S: usize = Self::BASE + 0x754;
    const ECDSA_SIGN_OUT_RESULT: usize = Self::BASE + 0xEE8;

    // ECDSA verify input addresses
    const ECDSA_VERIFY_N_LEN: usize = Self::BASE + 0x404;
    const ECDSA_VERIFY_P_LEN: usize = Self::BASE + 0x4B4;
    const ECDSA_VERIFY_A_SIGN: usize = Self::BASE + 0x45C;
    const ECDSA_VERIFY_A: usize = Self::BASE + 0x460;
    const ECDSA_VERIFY_P: usize = Self::BASE + 0x4B8;
    const ECDSA_VERIFY_X: usize = Self::BASE + 0x5E8;
    const ECDSA_VERIFY_Y: usize = Self::BASE + 0x63C;
    const ECDSA_VERIFY_XQ: usize = Self::BASE + 0xF40;
    const ECDSA_VERIFY_YQ: usize = Self::BASE + 0xF94;
    const ECDSA_VERIFY_R: usize = Self::BASE + 0x1098;
    const ECDSA_VERIFY_S: usize = Self::BASE + 0xA44;
    const ECDSA_VERIFY_Z: usize = Self::BASE + 0xFE8;
    const ECDSA_VERIFY_N: usize = Self::BASE + 0xD5C;

    // ECDSA verify output addresses
    const ECDSA_VERIFY_OUT: usize = Self::BASE + 0x5B0;

    /// Create a new PKA driver from a PKA peripheral.
    ///
    /// This will enable clocks and reset the PKA peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{pac, pka::Pka};
    ///
    /// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rcc = dp.RCC;
    ///
    /// let mut pka = Pka::new(dp.PKA, &mut rcc);
    /// ```
    pub fn new(pka: pac::PKA, rcc: &mut pac::RCC) -> Pka {
        Self::enable_clock(rcc);
        rcc.ahb3rstr.modify(|_, w| w.pkarst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.pkarst().clear_bit());

        debug_assert_eq!(pka.cr.read().bits(), 0);
        debug_assert_eq!(pka.sr.read().bits(), 0);

        // When the PKA peripheral reset signal is released PKA RAM is cleared
        // automatically, taking 894 clock cycles.
        // During this time the setting of EN bit in PKA_CR is ignored
        pka.cr.write(|w| w.en().set_bit());
        while pka.cr.read().en().bit_is_clear() {
            pka.cr.write(|w| w.en().set_bit());
        }

        debug_assert_eq!(pka.cr.read().bits(), 1);
        debug_assert_eq!(pka.sr.read().bits(), 0);

        Pka { pka }
    }

    /// Free the PKA peripheral from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{pac, pka::Pka};
    ///
    /// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rcc = dp.RCC;
    /// let pka = dp.PKA;
    ///
    /// let mut pka_driver = Pka::new(pka, &mut rcc);
    /// // ... use PKA
    /// let pka = pka_driver.free();
    /// ```
    pub fn free(self) -> pac::PKA {
        self.pka
    }

    /// Steal the PKA peripheral from whatever is currently using it.
    ///
    /// This will **not** initialize the PKA (unlike [`new`]).
    ///
    /// # Safety
    ///
    /// This will create a new PKA peripheral, bypassing the singleton checks
    /// that normally occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// the PKA peripheral.
    /// You are also responsible for ensuring the PKA has been setup correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::pka::Pka;
    ///
    /// // ... setup happens here
    ///
    /// let pka = unsafe { Pka::steal() };
    /// ```
    ///
    /// [`new`]: Pka::new
    pub unsafe fn steal() -> Pka {
        let dp: pac::Peripherals = pac::Peripherals::steal();
        Pka { pka: dp.PKA }
    }

    /// Disable the PKA clock.
    pub fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.pkaen().disabled());
    }

    /// Enable the PKA clock.
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.pkaen().enabled());
        rcc.ahb3enr.read(); // selay after an RCC peripheral clock enabling
    }

    fn clear_all_faults(&mut self) {
        #[rustfmt::skip]
        self.pka.clrfr.write(|w| {
            w
                .addrerrfc().set_bit()
                .ramerrfc().set_bit()
                .procendfc().set_bit()
        });
    }

    fn zero_all_ram(&mut self) {
        for dw in 0..Self::RAM_NUM_DW {
            debug_assert_eq!(self.pka.sr.read().bits(), 0);
            unsafe { write_volatile((dw * 4 + Self::RAM_BASE) as *mut u32, 0) }
        }
    }

    unsafe fn write_buf_to_ram(&mut self, offset: usize, buf: &[u32]) {
        compiler_fence(SeqCst);
        debug_assert_eq!(offset % 4, 0);
        debug_assert!(offset + buf.len() * size_of::<u32>() < 0x5800_33FF);
        buf.iter().rev().enumerate().for_each(|(idx, &dw)| {
            write_volatile((offset + idx * size_of::<u32>()) as *mut u32, dw)
        });
        compiler_fence(SeqCst)
    }

    unsafe fn read_buf_from_ram(&mut self, offset: usize, buf: &mut [u32]) {
        compiler_fence(SeqCst);
        debug_assert_eq!(offset % 4, 0);
        debug_assert!(offset + buf.len() * size_of::<u32>() < 0x5800_33FF);
        buf.iter_mut().rev().enumerate().for_each(|(idx, dw)| {
            *dw = read_volatile((offset + idx * size_of::<u32>()) as *const u32);
        });
        compiler_fence(SeqCst)
    }

    fn ecdsa_sign_set<const MODULUS_SIZE: usize, const PRIME_ORDER_SIZE: usize>(
        &mut self,
        curve: &EllipticCurve<MODULUS_SIZE, PRIME_ORDER_SIZE>,
        nonce: &[u32; PRIME_ORDER_SIZE],
        private_key: &[u32; PRIME_ORDER_SIZE],
        hash: &[u32; PRIME_ORDER_SIZE],
    ) {
        debug_assert!(self.pka.cr.read().en().bit_is_set());
        debug_assert_eq!(self.pka.sr.read().bits(), 0);
        let n_length: u32 = (PRIME_ORDER_SIZE * size_of::<u32>() * 8) as u32;
        let p_length: u32 = (MODULUS_SIZE * size_of::<u32>() * 8) as u32;

        unsafe {
            write_volatile(Self::ECDSA_SIGN_N_LEN as *mut u32, n_length);
            write_volatile(Self::ECDSA_SIGN_P_LEN as *mut u32, p_length);
            write_volatile(Self::ECDSA_SIGN_A_SIGN as *mut u32, curve.coef_sign.into());
            self.write_buf_to_ram(Self::ECDSA_SIGN_A, &curve.coef);
            self.write_buf_to_ram(Self::ECDSA_SIGN_P, &curve.modulus);
            self.write_buf_to_ram(Self::ECDSA_SIGN_K, nonce);
            self.write_buf_to_ram(Self::ECDSA_SIGN_X, &curve.base_point_x);
            self.write_buf_to_ram(Self::ECDSA_SIGN_Y, &curve.base_point_y);
            self.write_buf_to_ram(Self::ECDSA_SIGN_Z, hash);
            self.write_buf_to_ram(Self::ECDSA_SIGN_D, private_key);
            self.write_buf_to_ram(Self::ECDSA_SIGN_N, &curve.prime_order);
        }
    }

    fn process(&mut self, opcode: PkaOpcode) -> Result<(), Error> {
        debug_assert!(self.pka.sr.read().procendf().bit_is_clear());

        #[rustfmt::skip]
        self.pka.cr.write(|w| unsafe {
            w
                .addrerrie().clear_bit()
                .ramerrie().clear_bit()
                .procendie().clear_bit()
                .mode().bits(opcode.into())
                .start().set_bit()
                .en().set_bit()
        });

        let mut attempts = 0;
        loop {
            let sr = self.pka.sr.read();
            if sr.addrerrf().bit_is_set() {
                return Err(Error::Address);
            } else if sr.ramerrf().bit_is_set() {
                return Err(Error::Ram);
            } else if sr.procendf().bit_is_set() {
                break;
            } else {
                attempts += 1;
            }

            // TODO: Return a timeout error.
            // TODO: Accept a timeout argument.
            if attempts >= 2_000_000 {
                panic!(
                    "TOO LONG, sr=0x{:X} busy={} addrerrf={} ramerrf={} procendf={}",
                    sr.bits(),
                    sr.busy().bits(),
                    sr.addrerrf().bits(),
                    sr.ramerrf().bits(),
                    sr.procendf().bits()
                )
            }
        }

        debug_assert!(self.pka.sr.read().busy().bit_is_clear());
        self.clear_all_faults();
        Ok(())
    }

    /// ECDSA (Ellipctic Curve Digital Signature Algorithm) signing.
    pub fn ecdsa_sign<const MODULUS_SIZE: usize, const PRIME_ORDER_SIZE: usize>(
        &mut self,
        curve: &EllipticCurve<MODULUS_SIZE, PRIME_ORDER_SIZE>,
        nonce: &[u32; PRIME_ORDER_SIZE],
        private_key: &[u32; PRIME_ORDER_SIZE],
        hash: &[u32; PRIME_ORDER_SIZE],
        sig_buf: &mut EcdsaSignature<MODULUS_SIZE>,
    ) -> Result<(), EcdsaSignError> {
        self.ecdsa_sign_set(curve, nonce, private_key, hash);
        if let Err(e) = self.process(PkaOpcode::EcdsaSign) {
            self.zero_all_ram();
            Err(e.into())
        } else {
            unsafe {
                self.read_buf_from_ram(Self::ECDSA_SIGN_OUT_R, &mut sig_buf.r_sign);
                self.read_buf_from_ram(Self::ECDSA_SIGN_OUT_S, &mut sig_buf.s_sign);
            }

            let result: u32 = unsafe { read_volatile(Self::ECDSA_SIGN_OUT_RESULT as *const u32) };
            if result != 0 {
                // Reference manual table 163 "ECDSA sign - Outputs":
                // If error output is different from zero the content of the PKA
                // memory should be cleared to avoid leaking information about
                // the private key.
                self.zero_all_ram();
            }
            match result {
                0 => Ok(()),
                1 => Err(EcdsaSignError::Rzero),
                2 => Err(EcdsaSignError::Szero),
                _ => Err(EcdsaSignError::Unknown { bits: result }),
            }
        }
    }

    fn ecdsa_verify_set<const MODULUS_SIZE: usize, const PRIME_ORDER_SIZE: usize>(
        &mut self,
        curve: &EllipticCurve<MODULUS_SIZE, PRIME_ORDER_SIZE>,
        sig: &EcdsaSignature<MODULUS_SIZE>,
        pub_key: &EcdsaPublicKey<MODULUS_SIZE>,
        hash: &[u32; PRIME_ORDER_SIZE],
    ) {
        debug_assert!(self.pka.cr.read().en().bit_is_set());
        debug_assert_eq!(self.pka.sr.read().bits(), 0);
        let n_length: u32 = (PRIME_ORDER_SIZE * size_of::<u32>() * 8) as u32;
        let p_length: u32 = (MODULUS_SIZE * size_of::<u32>() * 8) as u32;

        unsafe {
            write_volatile(Self::ECDSA_VERIFY_N_LEN as *mut u32, n_length);
            write_volatile(Self::ECDSA_VERIFY_P_LEN as *mut u32, p_length);
            write_volatile(
                Self::ECDSA_VERIFY_A_SIGN as *mut u32,
                curve.coef_sign.into(),
            );
            self.write_buf_to_ram(Self::ECDSA_VERIFY_A, &curve.coef);
            self.write_buf_to_ram(Self::ECDSA_VERIFY_P, &curve.modulus);
            self.write_buf_to_ram(Self::ECDSA_VERIFY_X, &curve.base_point_x);
            self.write_buf_to_ram(Self::ECDSA_VERIFY_Y, &curve.base_point_y);
            self.write_buf_to_ram(Self::ECDSA_VERIFY_XQ, &pub_key.curve_pt_x);
            self.write_buf_to_ram(Self::ECDSA_VERIFY_YQ, &pub_key.curve_pt_y);
            self.write_buf_to_ram(Self::ECDSA_VERIFY_R, &sig.r_sign);
            self.write_buf_to_ram(Self::ECDSA_VERIFY_S, &sig.s_sign);
            self.write_buf_to_ram(Self::ECDSA_VERIFY_Z, hash);
            self.write_buf_to_ram(Self::ECDSA_VERIFY_N, &curve.prime_order);
        }
    }

    /// ECDSA (Ellipctic Curve Digital Signature Algorithm) verification.
    pub fn ecdsa_verify<const MODULUS_SIZE: usize, const PRIME_ORDER_SIZE: usize>(
        &mut self,
        curve: &EllipticCurve<MODULUS_SIZE, PRIME_ORDER_SIZE>,
        sig: &EcdsaSignature<MODULUS_SIZE>,
        pub_key: &EcdsaPublicKey<MODULUS_SIZE>,
        hash: &[u32; PRIME_ORDER_SIZE],
    ) -> Result<(), EcdsaVerifyError> {
        self.zero_all_ram();
        self.ecdsa_verify_set(curve, sig, pub_key, hash);
        self.process(PkaOpcode::EcdsaVerify)?;

        let result: u32 = unsafe { read_volatile(Self::ECDSA_VERIFY_OUT as *const u32) };
        match result {
            0 => Ok(()),
            _ => Err(EcdsaVerifyError::Invalid),
        }
    }
}

/// Sign bit for ECDSA coefficient signing and verification.
#[repr(u32)]
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Clone, Copy)]
pub enum Sign {
    /// Positive.
    Pos = 0,
    /// Negative.
    Neg = 1,
}

impl From<Sign> for u32 {
    fn from(s: Sign) -> Self {
        s as u32
    }
}

/// ECDSA signature.
#[derive(Debug, PartialEq, Eq)]
pub struct EcdsaSignature<const MODULUS_SIZE: usize> {
    /// Signature part r.
    pub r_sign: [u32; MODULUS_SIZE],
    /// Signature part s.
    pub s_sign: [u32; MODULUS_SIZE],
}

/// ECDSA public key.
pub struct EcdsaPublicKey<const MODULUS_SIZE: usize> {
    /// Public-key curve point xQ.
    pub curve_pt_x: [u32; MODULUS_SIZE],
    /// Public-key curve point yQ.
    pub curve_pt_y: [u32; MODULUS_SIZE],
}

/// Elliptic curve.
///
/// Used to ECDSA signing and verification.
#[derive(Debug, PartialEq, Eq)]
pub struct EllipticCurve<const MODULUS_SIZE: usize, const PRIME_ORDER_SIZE: usize> {
    /// Curve coefficient a sign.
    ///
    /// **Note:** 0 for positive, 1 for negative.
    pub coef_sign: Sign,
    /// Curve coefficient |a|.
    ///
    /// **Note:** Absolute value, |a| < p.
    pub coef: [u32; MODULUS_SIZE],
    /// Curve modulus value p.
    ///
    /// **Note:** Odd integer prime, 0 < p < 2<sup>640</sup>
    pub modulus: [u32; MODULUS_SIZE],
    /// Curve base point G coordinate x.
    ///
    /// **Note:** x < p
    pub base_point_x: [u32; MODULUS_SIZE],
    /// Curve base point G coordinate y.
    ///
    /// **Note:** y < p
    pub base_point_y: [u32; MODULUS_SIZE],
    /// Curve prime order n.
    ///
    /// **Note:** Integer prime.
    pub prime_order: [u32; PRIME_ORDER_SIZE],
}

/// Pre-defined elliptic curves.
pub mod curve {
    use super::{
        EllipticCurve,
        Sign::{Neg, Pos},
    };

    /// nist P-256
    pub const NIST_P256: EllipticCurve<8, 8> = EllipticCurve {
        coef_sign: Neg,
        coef: [
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000003,
        ],
        modulus: [
            0xffffffff, 0x00000001, 0x00000000, 0x00000000, 0x00000000, 0xffffffff, 0xffffffff,
            0xffffffff,
        ],
        base_point_x: [
            0x6b17d1f2, 0xe12c4247, 0xf8bce6e5, 0x63a440f2, 0x77037d81, 0x2deb33a0, 0xf4a13945,
            0xd898c296,
        ],
        base_point_y: [
            0x4fe342e2, 0xfe1a7f9b, 0x8ee7eb4a, 0x7c0f9e16, 0x2bce3357, 0x6b315ece, 0xcbb64068,
            0x37bf51f5,
        ],
        prime_order: [
            0xffffffff, 0x00000000, 0xffffffff, 0xffffffff, 0xbce6faad, 0xa7179e84, 0xf3b9cac2,
            0xfc632551,
        ],
    };

    /// brainpool P224r1
    pub const P224R1: EllipticCurve<7, 7> = EllipticCurve {
        coef_sign: Pos,
        coef: [
            0x68A5E62C, 0xA9CE6C1C, 0x299803A6, 0xC1530B51, 0x4E182AD8, 0xB0042A59, 0xCAD29F43,
        ],
        modulus: [
            0xD7C134AA, 0x26436686, 0x2A183025, 0x75D1D787, 0xB09F0757, 0x97DA89F5, 0x7EC8C0FF,
        ],
        base_point_x: [
            0x0D9029AD, 0x2C7E5CF4, 0x340823B2, 0xA87DC68C, 0x9E4CE317, 0x4C1E6EFD, 0xEE12C07D,
        ],
        base_point_y: [
            0x58AA56F7, 0x72C0726F, 0x24C6B89E, 0x4ECDAC24, 0x354B9E99, 0xCAA3F6D3, 0x761402CD,
        ],
        prime_order: [
            0xD7C134AA, 0x26436686, 0x2A183025, 0x75D0FB98, 0xD116BC4B, 0x6DDEBCA3, 0xA5A7939F,
        ],
    };

    #[allow(missing_docs)]
    pub const P224R1_B: [u32; 7] = [
        0x2580F63C, 0xCFE44138, 0x870713B1, 0xA92369E3, 0x3E2135D2, 0x66DBB372, 0x386C400B,
    ];

    /// nist P-192
    #[rustfmt::skip]
    pub const NIST_P192: EllipticCurve<6, 6> = EllipticCurve {
        coef_sign: Neg,
        coef: [0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000003],
        modulus: [0xFFFFFFFF; 6],
        base_point_x: [0x188DA80E, 0xB03090F6, 0x7CBF20EB, 0x43A18800, 0xF4FF0AFD, 0x82FF1012],
        base_point_y: [0x07192B95, 0xFFC8DA78, 0x631011ED, 0x6B24CDD5, 0x73F977A1, 0x1E794811],
        prime_order: [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x99DEF836, 0x146BC9B1, 0xB4D22831],
    };

    #[allow(missing_docs)]
    pub const NIST_P192_B: [u32; 6] = [
        0x64210519, 0xE59C80E7, 0x0FA7E9AB, 0x72243049, 0xFEB8DEEC, 0xC146B9B1,
    ];
}
