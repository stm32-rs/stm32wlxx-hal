//! Public key accelerator
//!
//! Quickstart:
//!
//! * [ECDSA signing](Pka::ecdsa_sign)
//! * [ECDSA verify](Pka::ecdsa_verify)
//!
//! # Alternatives
//!
//! The [p256-cortex-m4] crate offers an assembly implementation of P256 that is
//! about 10x faster then the hardware.
//!
//! # ECDSA key pair generation
//!
//! Generate a private key for [`NIST_P256`](curve::NIST_P256):
//!
//! ```console
//! $ openssl ecparam -genkey -name prime256v1 -out key.pem
//! $ openssl ec -in key.pem -noout -text
//! read EC key
//! Private-Key: (256 bit)
//! priv:
//!     49:ac:87:27:ce:e8:74:84:fe:6d:fd:a5:10:23:8a:
//!     d4:11:ac:e8:fe:59:3a:8c:b7:04:92:d6:59:db:81:
//!     80:2a
//! pub:
//!     04:fa:65:57:59:de:c3:90:28:96:46:0a:43:2b:ae:
//!     1d:00:91:26:e1:b4:88:78:9f:f4:ef:6b:9a:9b:de:
//!     1b:c3:63:8f:a0:2a:c4:c4:21:ca:88:4f:06:51:f4:
//!     e9:85:e3:cf:d0:af:40:69:cc:87:f3:a8:8a:8e:95:
//!     e7:55:6c:ed:97
//! ASN1 OID: prime256v1
//! NIST CURVE: P-256
//! ```
//!
//! The first `04` on the public key is encoding information, expressed in rust
//! that keypair becomes this:
//!
//! ```
//! use stm32wlxx_hal::pka::EcdsaPublicKey;
//!
//! const PRIV_KEY: [u32; 8] = [
//!     0x49ac8727, 0xcee87484, 0xfe6dfda5, 0x10238ad4, 0x11ace8fe, 0x593a8cb7, 0x0492d659,
//!     0xdb81802a,
//! ];
//!
//! const CURVE_PT_X: [u32; 8] = [
//!     0xfa655759, 0xdec39028, 0x96460a43, 0x2bae1d00, 0x9126e1b4, 0x88789ff4, 0xef6b9a9b,
//!     0xde1bc363,
//! ];
//! const CURVE_PT_Y: [u32; 8] = [
//!     0x8fa02ac4, 0xc421ca88, 0x4f0651f4, 0xe985e3cf, 0xd0af4069, 0xcc87f3a8, 0x8a8e95e7,
//!     0x556ced97,
//! ];
//! let pub_key: EcdsaPublicKey<8> = EcdsaPublicKey {
//!     curve_pt_x: &CURVE_PT_X,
//!     curve_pt_y: &CURVE_PT_Y,
//! };
//! ```
//!
//! Use this command to find the name of other curves:
//!
//! ```bash
//! openssl ecparam -list_curves
//! ```
//!
//! [p256-cortex-m4]: https://crates.io/crates/p256-cortex-m4

use crate::pac::{self, pka::cr::MODE_A};
use core::{
    mem::size_of,
    ptr::{read_volatile, write_volatile},
};
use nb;

/// Errors from an ECDSA signing operation.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    /// PKA mode does not match the expected mode.
    Mode {
        /// Actual mode bits
        mode: u8,
    },
    /// Unknown result code.
    Unknown {
        /// Unknown result code bits.
        bits: u32,
    },
}

impl EcdsaSignError {
    const fn from_raw(raw: u32) -> nb::Result<(), EcdsaSignError> {
        match raw {
            0 => Ok(()),
            1 => Err(nb::Error::Other(EcdsaSignError::Rzero)),
            2 => Err(nb::Error::Other(EcdsaSignError::Szero)),
            _ => Err(nb::Error::Other(EcdsaSignError::Unknown { bits: raw })),
        }
    }

    const fn mode(mode: u8) -> nb::Result<(), EcdsaSignError> {
        Err(nb::Error::Other(EcdsaSignError::Mode { mode }))
    }
}

/// Errors from an ECDSA verify operation.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EcdsaVerifyError {
    /// Address access is out of range (unmapped address).
    Address,
    /// An AHB access to the PKA RAM occurred while the PKA core was computing
    /// and using its internal RAM.
    /// (AHB PKA_RAM access is not allowed while a PKA operation is in progress).
    Ram,
    /// Invalid signature.
    Invalid,
    /// PKA mode does not match the expected mode.
    Mode {
        /// Actual mode bits
        mode: u8,
    },
}

impl EcdsaVerifyError {
    const fn from_raw(raw: u32) -> nb::Result<(), EcdsaVerifyError> {
        match raw {
            0 => Ok(()),
            _ => Err(nb::Error::Other(EcdsaVerifyError::Invalid)),
        }
    }

    const fn mode(mode: u8) -> nb::Result<(), EcdsaVerifyError> {
        Err(nb::Error::Other(EcdsaVerifyError::Mode { mode }))
    }
}

/// PKA operation codes.
#[derive(Debug)]
#[repr(u8)]
#[allow(dead_code)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

const BASE: usize = 0x5800_2000;
const RAM_BASE: usize = BASE + 0x400;
const RAM_NUM_DW: usize = 894;

// ECDSA sign input addresses
const ECDSA_SIGN_N_LEN: usize = BASE + 0x400;
const ECDSA_SIGN_P_LEN: usize = BASE + 0x404;
const ECDSA_SIGN_A_SIGN: usize = BASE + 0x408;
const ECDSA_SIGN_A: usize = BASE + 0x40C;
const ECDSA_SIGN_P: usize = BASE + 0x460;
const ECDSA_SIGN_K: usize = BASE + 0x508;
const ECDSA_SIGN_X: usize = BASE + 0x55C;
const ECDSA_SIGN_Y: usize = BASE + 0x5B0;
const ECDSA_SIGN_Z: usize = BASE + 0xDE8;
const ECDSA_SIGN_D: usize = BASE + 0xE3C;
const ECDSA_SIGN_N: usize = BASE + 0xE94;

// ECDSA sign output addresses
const ECDSA_SIGN_OUT_R: usize = BASE + 0x700;
const ECDSA_SIGN_OUT_S: usize = BASE + 0x754;
const ECDSA_SIGN_OUT_RESULT: usize = BASE + 0xEE8;

// ECDSA verify input addresses
const ECDSA_VERIFY_N_LEN: usize = BASE + 0x404;
const ECDSA_VERIFY_P_LEN: usize = BASE + 0x4B4;
const ECDSA_VERIFY_A_SIGN: usize = BASE + 0x45C;
const ECDSA_VERIFY_A: usize = BASE + 0x460;
const ECDSA_VERIFY_P: usize = BASE + 0x4B8;
const ECDSA_VERIFY_X: usize = BASE + 0x5E8;
const ECDSA_VERIFY_Y: usize = BASE + 0x63C;
const ECDSA_VERIFY_XQ: usize = BASE + 0xF40;
const ECDSA_VERIFY_YQ: usize = BASE + 0xF94;
const ECDSA_VERIFY_R: usize = BASE + 0x1098;
const ECDSA_VERIFY_S: usize = BASE + 0xA44;
const ECDSA_VERIFY_Z: usize = BASE + 0xFE8;
const ECDSA_VERIFY_N: usize = BASE + 0xD5C;

// ECDSA verify output addresses
const ECDSA_VERIFY_OUT: usize = BASE + 0x5B0;

/// PKA driver.
#[derive(Debug)]
pub struct Pka {
    pka: pac::PKA,
}

impl Pka {
    /// Create a new PKA driver from a PKA peripheral.
    ///
    /// This will enable clocks and reset the PKA peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{pac, pka::Pka};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut pka = Pka::new(dp.PKA, &mut dp.RCC);
    /// ```
    pub fn new(pka: pac::PKA, rcc: &mut pac::RCC) -> Pka {
        Self::enable_clock(rcc);
        unsafe { Self::pulse_reset(rcc) };

        // When the PKA peripheral reset signal is released PKA RAM is cleared
        // automatically, taking 894 clock cycles.
        // During this time the setting of EN bit in PKA_CR is ignored
        pka.cr.write(|w| w.en().set_bit());
        while pka.cr.read().en().bit_is_clear() {
            pka.cr.write(|w| w.en().set_bit());
        }

        Pka { pka }
    }

    /// Returns `true` if the PKA is enabled.
    #[inline]
    pub fn is_enabled(&mut self) -> bool {
        self.pka.cr.read().en().is_enabled()
    }

    /// Free the PKA peripheral from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{pac, pka::Pka};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let pka: pac::PKA = dp.PKA;
    /// let pka: Pka = Pka::new(pka, &mut dp.RCC);
    /// // ... use PKA
    /// let pka: pac::PKA = pka.free();
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
    /// use stm32wlxx_hal::pka::Pka;
    ///
    /// // ... setup happens here
    ///
    /// let pka = unsafe { Pka::steal() };
    /// ```
    ///
    /// [`new`]: Pka::new
    pub unsafe fn steal() -> Pka {
        Pka {
            pka: pac::Peripherals::steal().PKA,
        }
    }

    /// Disable the PKA clock.
    ///
    /// # Safety
    ///
    /// 1. You are responsible for ensuring the PKA bus is in a state where the
    ///    clock can be disabled without entering an error state.
    /// 2. You cannot use the PKA bus while the clock is disabled.
    /// 3. You are responsible for re-enabling the clock before resuming use
    ///    of the PKA bus.
    /// 4. You are responsible for setting up anything that may have lost state
    ///    while the clock was disabled.
    #[inline]
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.pkaen().disabled());
    }

    /// Enable the PKA clock.
    #[inline]
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.pkaen().enabled());
        rcc.ahb3enr.read(); // delay after an RCC peripheral clock enabling
    }

    /// Reset the PKA.
    ///
    /// # Safety
    ///
    /// 1. The PKA must not be in-use.
    /// 2. You are responsible for setting up the PKA after a reset.
    #[inline]
    pub unsafe fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.ahb3rstr.modify(|_, w| w.pkarst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.pkarst().clear_bit());
    }

    /// Unmask the PKA IRQ in the NVIC.
    ///
    /// # Safety
    ///
    /// This can break mask-based critical sections.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # #[cfg(not(feature = "stm32wl5x_cm0p"))]
    /// unsafe { stm32wlxx_hal::pka::Pka::unmask_irq() };
    /// ```
    #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    #[inline]
    pub unsafe fn unmask_irq() {
        pac::NVIC::unmask(pac::Interrupt::PKA)
    }

    #[inline]
    fn clear_all_flags(&mut self) {
        self.pka.clrfr.write(|w| {
            w.addrerrfc().set_bit();
            w.ramerrfc().set_bit();
            w.procendfc().set_bit()
        });
    }

    fn zero_ram(&mut self) {
        (0..RAM_NUM_DW)
            .into_iter()
            .for_each(|dw| unsafe { write_volatile((dw * 4 + RAM_BASE) as *mut u32, 0) });
    }

    unsafe fn write_ram(&mut self, offset: usize, buf: &[u32]) {
        // asserts are for internal correctness, should not be accessible by users
        debug_assert_eq!(offset % 4, 0);
        debug_assert!(offset + buf.len() * size_of::<u32>() < 0x5800_33FF);
        buf.iter().rev().enumerate().for_each(|(idx, &dw)| {
            write_volatile((offset + idx * size_of::<u32>()) as *mut u32, dw)
        });
    }

    unsafe fn read_ram(&mut self, offset: usize, buf: &mut [u32]) {
        // asserts are for internal correctness, should not be accessible by users
        debug_assert_eq!(offset % 4, 0);
        debug_assert!(offset + buf.len() * size_of::<u32>() < 0x5800_33FF);
        buf.iter_mut().rev().enumerate().for_each(|(idx, dw)| {
            *dw = read_volatile((offset + idx * size_of::<u32>()) as *const u32);
        });
    }

    #[inline]
    fn start_process(&mut self, mode: MODE_A) {
        self.pka.cr.write(|w| {
            w.addrerrie().enabled();
            w.ramerrie().enabled();
            w.procendie().enabled();
            w.mode().variant(mode);
            w.start().set_bit();
            w.en().set_bit()
        });
    }

    /// ECDSA (Elliptic Curve Digital Signature Algorithm) signing.
    ///
    /// This is the blocking ECDSA sign method, equivalent to calling
    /// [`ecdsa_sign_start`](Self::ecdsa_sign_start) then polling
    /// [`ecdsa_sign_result`](Self::ecdsa_sign_result).
    ///
    /// ```no_run
    /// # let mut pka = unsafe { stm32wlxx_hal::pka::Pka::steal() };
    /// # let curve = stm32wlxx_hal::pka::curve::NIST_P256;
    /// # let nonce: [u32; 8] = [0; 8];
    /// # let priv_key: [u32; 8] = [0; 8];
    /// # let hash: [u32; 8] = [0; 8];
    /// # let mut r_sign: [u32; 8] = [0; 8];
    /// # let mut s_sign: [u32; 8] = [0; 8];
    /// // blocking
    /// pka.ecdsa_sign(&curve, &nonce, &priv_key, &hash, &mut r_sign, &mut s_sign);
    ///
    /// // non-blocking
    /// pka.ecdsa_sign_start(&curve, &nonce, &priv_key, &hash)?;
    /// nb::block!(pka.ecdsa_sign_result(&mut r_sign, &mut s_sign))?;
    /// # Ok::<(), stm32wlxx_hal::pka::EcdsaSignError>(())
    /// ```
    ///
    /// # Computation Times
    ///
    /// | Modulus Length (bits) | Cycles   | Seconds at 48MHz |
    /// |-----------------------|----------|------------------|
    /// | 160                   | 1760000  | 0.037            |
    /// | 192                   | 2664000  | 0.056            |
    /// | 256                   | 5249000  | 0.109            |
    /// | 320                   | 9016000  | 0.188            |
    /// | 384                   | 14596000 | 0.304            |
    /// | 512                   | 30618000 | 0.638            |
    /// | 521                   | 35540000 | 0.740            |
    pub fn ecdsa_sign<const MODULUS_SIZE: usize, const PRIME_ORDER_SIZE: usize>(
        &mut self,
        curve: &EllipticCurve<MODULUS_SIZE, PRIME_ORDER_SIZE>,
        nonce: &[u32; PRIME_ORDER_SIZE],
        priv_key: &[u32; PRIME_ORDER_SIZE],
        hash: &[u32; PRIME_ORDER_SIZE],
        r_sign: &mut [u32; MODULUS_SIZE],
        s_sign: &mut [u32; MODULUS_SIZE],
    ) -> Result<(), EcdsaSignError> {
        self.ecdsa_sign_start(curve, nonce, priv_key, hash)?;
        nb::block!(self.ecdsa_sign_result(r_sign, s_sign))
    }

    /// Start an ECDSA signing operation.
    ///
    /// This will enable all the PKA IRQs.
    ///
    /// Use the [`ecdsa_sign_result`](Self::ecdsa_sign_result) method to poll
    /// for completion, or to get the result in an interrupt handler.
    pub fn ecdsa_sign_start<const MODULUS_SIZE: usize, const PRIME_ORDER_SIZE: usize>(
        &mut self,
        curve: &EllipticCurve<MODULUS_SIZE, PRIME_ORDER_SIZE>,
        nonce: &[u32; PRIME_ORDER_SIZE],
        priv_key: &[u32; PRIME_ORDER_SIZE],
        hash: &[u32; PRIME_ORDER_SIZE],
    ) -> Result<(), EcdsaSignError> {
        self.zero_ram();
        let n_length: u32 = (PRIME_ORDER_SIZE * size_of::<u32>() * 8) as u32;
        let p_length: u32 = (MODULUS_SIZE * size_of::<u32>() * 8) as u32;

        unsafe {
            write_volatile(ECDSA_SIGN_N_LEN as *mut u32, n_length);
            write_volatile(ECDSA_SIGN_P_LEN as *mut u32, p_length);
            write_volatile(ECDSA_SIGN_A_SIGN as *mut u32, curve.coef_sign.into());
            self.write_ram(ECDSA_SIGN_A, &curve.coef);
            self.write_ram(ECDSA_SIGN_P, &curve.modulus);
            self.write_ram(ECDSA_SIGN_K, nonce);
            self.write_ram(ECDSA_SIGN_X, &curve.base_point_x);
            self.write_ram(ECDSA_SIGN_Y, &curve.base_point_y);
            self.write_ram(ECDSA_SIGN_Z, hash);
            self.write_ram(ECDSA_SIGN_D, priv_key);
            self.write_ram(ECDSA_SIGN_N, &curve.prime_order);
        }
        let sr = self.pka.sr.read();
        if sr.addrerrf().bit_is_set() {
            self.clear_all_flags();
            Err(EcdsaSignError::Address)
        } else if sr.ramerrf().bit_is_set() {
            self.clear_all_flags();
            Err(EcdsaSignError::Ram)
        } else {
            self.start_process(MODE_A::Ecdsasign);
            Ok(())
        }
    }

    /// Get the result of an ECDSA sign operation.
    ///
    /// Use this after starting an ECDSA sign operation with
    /// [`ecdsa_sign_start`](Self::ecdsa_sign_start).
    pub fn ecdsa_sign_result<const MODULUS_SIZE: usize>(
        &mut self,
        r_sign: &mut [u32; MODULUS_SIZE],
        s_sign: &mut [u32; MODULUS_SIZE],
    ) -> nb::Result<(), EcdsaSignError> {
        let mode = self.pka.cr.read().mode();
        if !mode.is_ecdsasign() {
            return EcdsaSignError::mode(mode.bits());
        }
        let sr = self.pka.sr.read();
        if sr.addrerrf().bit_is_set() {
            self.clear_all_flags();
            Err(nb::Error::Other(EcdsaSignError::Address))
        } else if sr.ramerrf().bit_is_set() {
            self.clear_all_flags();
            Err(nb::Error::Other(EcdsaSignError::Ram))
        } else if sr.procendf().is_in_progress() {
            Err(nb::Error::WouldBlock)
        } else {
            self.clear_all_flags();

            unsafe {
                self.read_ram(ECDSA_SIGN_OUT_R, r_sign);
                self.read_ram(ECDSA_SIGN_OUT_S, s_sign);
            }

            let result: u32 = unsafe { read_volatile(ECDSA_SIGN_OUT_RESULT as *const u32) };
            if result != 0 {
                // Reference manual table 163 "ECDSA sign - Outputs":
                // If error output is different from zero the content of the PKA
                // memory should be cleared to avoid leaking information about
                // the private key.
                self.zero_ram();
            }
            EcdsaSignError::from_raw(result)
        }
    }

    /// ECDSA (Elliptic Curve Digital Signature Algorithm) verification.
    ///
    /// This is the blocking ECDSA verify method, equivalent to calling
    /// [`ecdsa_verify_start`](Self::ecdsa_verify_start) then polling
    /// [`ecdsa_verify_result`](Self::ecdsa_verify_result).
    ///
    /// ```no_run
    /// # let mut pka = unsafe { stm32wlxx_hal::pka::Pka::steal() };
    /// # let curve = stm32wlxx_hal::pka::curve::NIST_P256;
    /// # let r_sign: [u32; 8] = [0; 8];
    /// # let s_sign: [u32; 8] = [0; 8];
    /// # let curve_pt_x: [u32; 8] = [0; 8];
    /// # let curve_pt_y: [u32; 8] = [0; 8];
    /// # let sig = stm32wlxx_hal::pka::EcdsaSignature { r_sign: &r_sign, s_sign: &s_sign };
    /// # let pub_key = stm32wlxx_hal::pka::EcdsaPublicKey { curve_pt_x: &curve_pt_x, curve_pt_y: &curve_pt_y };
    /// # let hash: [u32; 8] = [0; 8];
    /// // blocking
    /// pka.ecdsa_verify(&curve, &sig, &pub_key, &hash)?;
    ///
    /// // non-blocking
    /// pka.ecdsa_verify_start(&curve, &sig, &pub_key, &hash)?;
    /// nb::block!(pka.ecdsa_verify_result())?;
    /// # Ok::<(), stm32wlxx_hal::pka::EcdsaVerifyError>(())
    /// ```
    ///
    /// # Computation Times
    ///
    /// | Modulus Length (bits) | Cycles   | Seconds at 48MHz |
    /// |-----------------------|----------|------------------|
    /// | 160                   | 3500000  | 0.073            |
    /// | 192                   | 5350000  | 0.112            |
    /// | 256                   | 10498000 | 0.219            |
    /// | 320                   | 18126000 | 0.378            |
    /// | 384                   | 29118000 | 0.607            |
    /// | 512                   | 61346000 | 1.278            |
    /// | 521                   | 71588000 | 1.491            |
    pub fn ecdsa_verify<const MODULUS_SIZE: usize, const PRIME_ORDER_SIZE: usize>(
        &mut self,
        curve: &EllipticCurve<MODULUS_SIZE, PRIME_ORDER_SIZE>,
        sig: &EcdsaSignature<MODULUS_SIZE>,
        pub_key: &EcdsaPublicKey<MODULUS_SIZE>,
        hash: &[u32; PRIME_ORDER_SIZE],
    ) -> Result<(), EcdsaVerifyError> {
        self.ecdsa_verify_start(curve, sig, pub_key, hash)?;
        nb::block!(self.ecdsa_verify_result())
    }

    /// Start an ECDSA verify operation.
    ///
    /// This will enable all the PKA IRQs.
    ///
    /// Use the [`ecdsa_verify_result`](Self::ecdsa_verify_result) method to
    /// poll for completion, or to get the result in an interrupt handler.
    pub fn ecdsa_verify_start<const MODULUS_SIZE: usize, const PRIME_ORDER_SIZE: usize>(
        &mut self,
        curve: &EllipticCurve<MODULUS_SIZE, PRIME_ORDER_SIZE>,
        sig: &EcdsaSignature<MODULUS_SIZE>,
        pub_key: &EcdsaPublicKey<MODULUS_SIZE>,
        hash: &[u32; PRIME_ORDER_SIZE],
    ) -> Result<(), EcdsaVerifyError> {
        self.zero_ram();
        let n_length: u32 = (PRIME_ORDER_SIZE * size_of::<u32>() * 8) as u32;
        let p_length: u32 = (MODULUS_SIZE * size_of::<u32>() * 8) as u32;

        unsafe {
            write_volatile(ECDSA_VERIFY_N_LEN as *mut u32, n_length);
            write_volatile(ECDSA_VERIFY_P_LEN as *mut u32, p_length);
            write_volatile(ECDSA_VERIFY_A_SIGN as *mut u32, curve.coef_sign.into());
            self.write_ram(ECDSA_VERIFY_A, &curve.coef);
            self.write_ram(ECDSA_VERIFY_P, &curve.modulus);
            self.write_ram(ECDSA_VERIFY_X, &curve.base_point_x);
            self.write_ram(ECDSA_VERIFY_Y, &curve.base_point_y);
            self.write_ram(ECDSA_VERIFY_XQ, pub_key.curve_pt_x);
            self.write_ram(ECDSA_VERIFY_YQ, pub_key.curve_pt_y);
            self.write_ram(ECDSA_VERIFY_R, sig.r_sign);
            self.write_ram(ECDSA_VERIFY_S, sig.s_sign);
            self.write_ram(ECDSA_VERIFY_Z, hash);
            self.write_ram(ECDSA_VERIFY_N, &curve.prime_order);
        }
        let sr = self.pka.sr.read();
        if sr.addrerrf().bit_is_set() {
            self.clear_all_flags();
            Err(EcdsaVerifyError::Address)
        } else if sr.ramerrf().bit_is_set() {
            self.clear_all_flags();
            Err(EcdsaVerifyError::Ram)
        } else {
            self.start_process(MODE_A::Ecdsaverif);
            Ok(())
        }
    }

    /// Get the result of an ECDSA verify operation.
    ///
    /// Use this after starting an ECDSA verify operation with
    /// [`ecdsa_verify_start`](Self::ecdsa_verify_start).
    pub fn ecdsa_verify_result(&mut self) -> nb::Result<(), EcdsaVerifyError> {
        let mode = self.pka.cr.read().mode();
        if !mode.is_ecdsaverif() {
            return EcdsaVerifyError::mode(mode.bits());
        }
        let sr = self.pka.sr.read();
        if sr.addrerrf().bit_is_set() {
            self.clear_all_flags();
            Err(nb::Error::Other(EcdsaVerifyError::Address))
        } else if sr.ramerrf().bit_is_set() {
            self.clear_all_flags();
            Err(nb::Error::Other(EcdsaVerifyError::Ram))
        } else if sr.procendf().is_in_progress() {
            Err(nb::Error::WouldBlock)
        } else {
            self.clear_all_flags();

            let result: u32 = unsafe { read_volatile(ECDSA_VERIFY_OUT as *const u32) };
            EcdsaVerifyError::from_raw(result)
        }
    }
}

/// Sign bit for ECDSA coefficient signing and verification.
#[repr(u32)]
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
pub struct EcdsaSignature<'a, const MODULUS_SIZE: usize> {
    /// Signature part r.
    pub r_sign: &'a [u32; MODULUS_SIZE],
    /// Signature part s.
    pub s_sign: &'a [u32; MODULUS_SIZE],
}

#[cfg(feature = "defmt")]
impl<'a, const MODULUS_SIZE: usize> defmt::Format for EcdsaSignature<'a, MODULUS_SIZE> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "EcdsaSignature {{ r_sign: {}, s_sign: {} }}",
            self.r_sign.as_ref(),
            self.s_sign.as_ref()
        )
    }
}

/// ECDSA public key.
#[derive(Debug, PartialEq, Eq)]
pub struct EcdsaPublicKey<'a, const MODULUS_SIZE: usize> {
    /// Public-key curve point xQ.
    pub curve_pt_x: &'a [u32; MODULUS_SIZE],
    /// Public-key curve point yQ.
    pub curve_pt_y: &'a [u32; MODULUS_SIZE],
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
