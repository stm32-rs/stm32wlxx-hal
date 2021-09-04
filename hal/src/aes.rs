//! Advanced encryption standard

use crate::pac;
use pac::aes::cr::KEYSIZE_A as KeySize;

/// Algorithm modes.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[allow(dead_code)]
enum Algorithm {
    /// Electronic codebook chaining algorithm
    Ecb,
    /// Cipher block chaining algorithm
    Cbc,
    /// Counter mode chaining algorithm
    Ctr,
    /// Galois counter mode - Galois message authentication code
    Gcm,
    /// Counter with Cipher Mode
    Ccm,
}

impl Algorithm {
    /// Bit 16
    pub(crate) const fn chmod2(&self) -> bool {
        matches!(self, Algorithm::Ccm)
    }

    /// Bits 6:5
    pub(crate) const fn chmod10(&self) -> u8 {
        match self {
            Algorithm::Ecb => 0b00,
            Algorithm::Cbc => 0b01,
            Algorithm::Ctr => 0b10,
            Algorithm::Gcm => 0b11,
            Algorithm::Ccm => 0b00,
        }
    }
}

#[repr(u8)]
#[allow(dead_code)]
enum Mode {
    Encryption = 0b00,
    KeyDerivation = 0b01,
    Decryption = 0b10,
    /// ST does not document this!
    /// ST uses this in their HAL implementation and it passes NIST tests...
    KeyDerivationDecryption = 0b11,
}

impl Mode {
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

impl From<Mode> for u8 {
    fn from(m: Mode) -> Self {
        m as u8
    }
}

/// AES errors.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Unexpected read operation from the AES_DOUTR register
    /// during computation or data input phase.
    Read,
    /// Unexpected write operation to the AES_DINR register
    /// during computation or data output phase.
    Write,
}

/// AES driver.
#[derive(Debug)]
pub struct Aes {
    aes: pac::AES,
}

impl Aes {
    /// Create a new AES driver from an AES peripheral.
    ///
    /// This will enable clocks and reset the AES peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut aes: Aes = Aes::new(dp.AES, &mut dp.RCC);
    /// ```
    #[inline]
    pub fn new(aes: pac::AES, rcc: &mut pac::RCC) -> Aes {
        Self::enable_clock(rcc);
        unsafe { Self::pulse_reset(rcc) };

        Aes { aes }
    }

    /// Free the AES peripheral from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut aes: Aes = Aes::new(dp.AES, &mut dp.RCC);
    /// // ... use AES
    /// let aes: pac::AES = aes.free();
    /// ```
    #[inline]
    pub fn free(self) -> pac::AES {
        self.aes
    }

    /// Reset the AES peripheral.
    ///
    /// [`new`](Self::new) will pulse reset for you.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the AES peripheral before calling this function.
    ///
    /// # Example
    ///
    /// See [`steal`](Self::steal).
    #[inline]
    pub unsafe fn pulse_reset(rcc: &mut pac::RCC) {
        rcc.ahb3rstr.modify(|_, w| w.aesrst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.aesrst().clear_bit());
    }

    /// Disable the AES peripheral clock.
    ///
    /// # Safety
    ///
    /// 1. Ensure nothing is using the AES peripheral before disabling the clock.
    /// 2. You are responsible for re-enabling the clock before using the AES peripheral.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut aes: Aes = Aes::new(dp.AES, &mut dp.RCC);
    /// // ... use AES
    ///
    /// // safety: AES is not in use
    /// unsafe { Aes::disable_clock(&mut dp.RCC) };
    ///
    /// // have a low power nap or something
    ///
    /// Aes::enable_clock(&mut dp.RCC);
    /// // ... use AES
    /// ```
    #[inline]
    pub unsafe fn disable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.aesen().disabled());
    }

    /// Enable the AES peripheral clock.
    ///
    /// [`new`](Self::new) will enable clocks for you.
    ///
    /// # Example
    ///
    /// See [`steal`](Self::steal).
    #[inline]
    pub fn enable_clock(rcc: &mut pac::RCC) {
        rcc.ahb3enr.modify(|_, w| w.aesen().enabled());
        rcc.ahb3enr.read(); // delay after an RCC peripheral clock enabling
    }

    /// Steal the AES peripheral from whatever is currently using it.
    ///
    /// This will **not** initialize the AES peripheral (unlike [`new`]).
    ///
    /// # Safety
    ///
    /// 1. Ensure that the code stealing the AES peripheral has exclusive access.
    ///    Singleton checks are bypassed with this method.
    /// 2. You are responsible for resetting the AES peripheral and enabling
    ///    the AES peripheral clock before use.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// // AES cannot be used via registers now
    /// let _: pac::AES = dp.AES;
    ///
    /// // safety: nothing is using the peripheral
    /// unsafe { Aes::pulse_reset(&mut dp.RCC) };
    ///
    /// Aes::enable_clock(&mut dp.RCC);
    ///
    /// // safety
    /// // 1. We have exclusive access
    /// // 2. peripheral has been setup
    /// let aes: Aes = unsafe { Aes::steal() };
    /// ```
    ///
    /// [`new`]: Aes::new
    #[inline]
    pub unsafe fn steal() -> Aes {
        let dp: pac::Peripherals = pac::Peripherals::steal();
        Aes { aes: dp.AES }
    }

    /// Unmask the AES IRQ in the NVIC.
    ///
    /// # Safety
    ///
    /// This can break mask-based critical sections.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    /// unsafe { stm32wl_hal::aes::Aes::unmask_irq() };
    /// ```
    #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    #[cfg_attr(docsrs, doc(cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))))]
    #[inline]
    pub unsafe fn unmask_irq() {
        pac::NVIC::unmask(pac::Interrupt::AES)
    }

    fn set_key(&mut self, key: &[u32]) -> KeySize {
        match key.len() {
            4 => {
                self.aes.cr.write(|w| w.en().disabled().keysize().bits128());
                self.aes.keyr3.write(|w| w.key().bits(key[0]));
                self.aes.keyr2.write(|w| w.key().bits(key[1]));
                self.aes.keyr1.write(|w| w.key().bits(key[2]));
                self.aes.keyr0.write(|w| w.key().bits(key[3]));
                KeySize::BITS128
            }
            8 => {
                self.aes.cr.write(|w| w.en().disabled().keysize().bits256());
                self.aes.keyr7.write(|w| w.key().bits(key[0]));
                self.aes.keyr6.write(|w| w.key().bits(key[1]));
                self.aes.keyr5.write(|w| w.key().bits(key[2]));
                self.aes.keyr4.write(|w| w.key().bits(key[3]));
                self.aes.keyr3.write(|w| w.key().bits(key[4]));
                self.aes.keyr2.write(|w| w.key().bits(key[5]));
                self.aes.keyr1.write(|w| w.key().bits(key[6]));
                self.aes.keyr0.write(|w| w.key().bits(key[7]));
                KeySize::BITS256
            }
            _ => panic!("Key must be 128-bit or 256-bit not {}-bit", key.len() * 32),
        }
    }

    fn poll_completion(&self) -> Result<(), Error> {
        loop {
            let sr = self.aes.sr.read();
            if sr.wrerr().bit_is_set() {
                return Err(Error::Write);
            }
            if sr.rderr().bit_is_set() {
                return Err(Error::Read);
            }
            if sr.ccf().bit_is_set() {
                return Ok(());
            }
        }
    }

    fn set_din(&mut self, din: &[u32; 4]) {
        din.iter()
            .for_each(|dw| self.aes.dinr.write(|w| w.din().bits(*dw)))
    }

    fn dout(&mut self, buf: &mut [u32; 4]) {
        buf.iter_mut()
            .for_each(|dw| *dw = self.aes.doutr.read().bits());
    }

    /// Encrypt using the electronic codebook chaining (ECB) algorithm.
    ///
    /// # Panics
    ///
    /// * Key is not 128-bits long (`[u32; 4]`) or 256-bits long (`[u32; 8]`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut aes: Aes = Aes::new(dp.AES, &mut dp.RCC);
    ///
    /// // this is a bad key, I am just using values from the NIST testsuite
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let plaintext: [u32; 4] = [0xf34481ec, 0x3cc627ba, 0xcd5dc3fb, 0x08f273e6];
    /// let mut ciphertext: [u32; 4] = [0; 4];
    /// aes.encrypt_ecb(&KEY, &plaintext, &mut ciphertext)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn encrypt_ecb(
        &mut self,
        key: &[u32],
        plaintext: &[u32; 4],
        ciphertext: &mut [u32; 4],
    ) -> Result<(), Error> {
        const ALGO: Algorithm = Algorithm::Ecb;
        const CHMOD2: bool = ALGO.chmod2();
        const CHMOD10: u8 = ALGO.chmod10();
        const MODE: u8 = Mode::Encryption.bits();

        let keysize: KeySize = self.set_key(key);

        #[rustfmt::skip]
        self.aes.cr.write(|w|
            w
                .en().enabled()
                .datatype().none()
                .mode().bits(MODE)
                .chmod2().bit(CHMOD2)
                .chmod().bits(CHMOD10)
                .ccfc().clear()
                .errc().clear()
                .ccfie().disabled()
                .errie().disabled()
                .dmainen().disabled()
                .dmaouten().disabled()
                .gcmph().bits(0) // do not care for ECB
                .keysize().variant(keysize)
                .npblb().bits(0) // no padding
        );

        self.set_din(plaintext);
        let ret: Result<(), Error> = self.poll_completion().map(|_| self.dout(ciphertext));

        self.aes.cr.write(|w| w.en().clear_bit());
        ret
    }

    /// Encrypt using the electronic codebook chaining (ECB) algorithm in-place.
    ///
    /// # Panics
    ///
    /// * Key is not 128-bits long (`[u32; 4]`) or 256-bits long (`[u32; 8]`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut aes: Aes = Aes::new(dp.AES, &mut dp.RCC);
    ///
    /// // this is a bad key, I am just using values from the NIST testsuite
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let mut text: [u32; 4] = [0xf34481ec, 0x3cc627ba, 0xcd5dc3fb, 0x08f273e6];
    /// aes.encrypt_ecb_inplace(&KEY, &mut text)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn encrypt_ecb_inplace(&mut self, key: &[u32], buf: &mut [u32; 4]) -> Result<(), Error> {
        const ALGO: Algorithm = Algorithm::Ecb;
        const CHMOD2: bool = ALGO.chmod2();
        const CHMOD10: u8 = ALGO.chmod10();
        const MODE: u8 = Mode::Encryption.bits();

        let keysize: KeySize = self.set_key(key);

        #[rustfmt::skip]
        self.aes.cr.write(|w|
            w
                .en().enabled()
                .datatype().none()
                .mode().bits(MODE)
                .chmod2().bit(CHMOD2)
                .chmod().bits(CHMOD10)
                .ccfc().clear()
                .errc().clear()
                .ccfie().disabled()
                .errie().disabled()
                .dmainen().disabled()
                .dmaouten().disabled()
                .gcmph().bits(0) // do not care for ECB
                .keysize().variant(keysize)
                .npblb().bits(0) // no padding
        );

        self.set_din(buf);
        let ret: Result<(), Error> = self.poll_completion().map(|_| self.dout(buf));

        self.aes.cr.write(|w| w.en().clear_bit());
        ret
    }

    /// Decrypt using the electronic codebook chaining (ECB) algorithm.
    ///
    /// # Panics
    ///
    /// * Key is not 128-bits long (`[u32; 4]`) or 256-bits long (`[u32; 8]`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut aes: Aes = Aes::new(dp.AES, &mut dp.RCC);
    ///
    /// // this is a bad key, I am just using values from the NIST testsuite
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let ciphertext: [u32; 4] = [0x0336763e, 0x966d9259, 0x5a567cc9, 0xce537f5e];
    /// let mut plaintext: [u32; 4] = [0; 4];
    /// aes.decrypt_ecb(&KEY, &ciphertext, &mut plaintext)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn decrypt_ecb(
        &mut self,
        key: &[u32],
        ciphertext: &[u32; 4],
        plaintext: &mut [u32; 4],
    ) -> Result<(), Error> {
        const ALGO: Algorithm = Algorithm::Ecb;
        const CHMOD2: bool = ALGO.chmod2();
        const CHMOD10: u8 = ALGO.chmod10();
        const MODE: u8 = Mode::KeyDerivationDecryption.bits();

        let keysize: KeySize = self.set_key(key);

        #[rustfmt::skip]
        self.aes.cr.write(|w|
            w
                .en().enabled()
                .datatype().none()
                .mode().bits(MODE)
                .chmod2().bit(CHMOD2)
                .chmod().bits(CHMOD10)
                .ccfc().clear()
                .errc().clear()
                .ccfie().disabled()
                .errie().disabled()
                .dmainen().disabled()
                .dmaouten().disabled()
                .gcmph().bits(0) // do not care for ECB
                .keysize().variant(keysize)
                .npblb().bits(0) // no padding
        );

        self.set_din(ciphertext);
        let ret: Result<(), Error> = self.poll_completion().map(|_| self.dout(plaintext));

        self.aes.cr.write(|w| w.en().clear_bit());
        ret
    }

    /// Decrypt using the electronic codebook chaining (ECB) algorithm in-place.
    ///
    /// # Panics
    ///
    /// * Key is not 128-bits long (`[u32; 4]`) or 256-bits long (`[u32; 8]`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut aes: Aes = Aes::new(dp.AES, &mut dp.RCC);
    ///
    /// // this is a bad key, I am just using values from the NIST testsuite
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let mut text: [u32; 4] = [0x0336763e, 0x966d9259, 0x5a567cc9, 0xce537f5e];
    /// aes.decrypt_ecb_inplace(&KEY, &mut text)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn decrypt_ecb_inplace(&mut self, key: &[u32], buf: &mut [u32; 4]) -> Result<(), Error> {
        const ALGO: Algorithm = Algorithm::Ecb;
        const CHMOD2: bool = ALGO.chmod2();
        const CHMOD10: u8 = ALGO.chmod10();
        const MODE: u8 = Mode::KeyDerivationDecryption.bits();
        let keysize: KeySize = self.set_key(key);

        #[rustfmt::skip]
        self.aes.cr.write(|w|
            w
                .en().enabled()
                .datatype().none()
                .mode().bits(MODE)
                .chmod2().bit(CHMOD2)
                .chmod().bits(CHMOD10)
                .ccfc().clear()
                .errc().clear()
                .ccfie().disabled()
                .errie().disabled()
                .dmainen().disabled()
                .dmaouten().disabled()
                .gcmph().bits(0) // do not care for ECB
                .keysize().variant(keysize)
                .npblb().bits(0) // no padding
        );

        self.set_din(buf);
        let ret: Result<(), Error> = self.poll_completion().map(|_| self.dout(buf));

        self.aes.cr.write(|w| w.en().clear_bit());
        ret
    }
}
