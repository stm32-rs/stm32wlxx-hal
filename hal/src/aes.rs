//! Advanced encryption standard

use crate::pac;

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

/// 128-bit AES key.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub struct Key128 {
    key: [u32; 4],
}

impl Key128 {
    /// Create a new 128-bit key from a `u128`.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::aes::Key128;
    ///
    /// const KEY: Key128 = Key128::from_u128(0xAAAAAAAABBBBBBBBCCCCCCCCDDDDDDDD);
    /// assert_eq!(
    ///     KEY,
    ///     Key128::from_u32([0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC, 0xDDDDDDDD])
    /// );
    /// ```
    pub const fn from_u128(key: u128) -> Key128 {
        Key128 {
            key: [
                (key >> 96) as u32,
                (key >> 64) as u32,
                (key >> 32) as u32,
                key as u32,
            ],
        }
    }

    /// Create a new 128-bit key from 4 dwords.
    ///
    /// This is the native key format of the hardware.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::aes::Key128;
    ///
    /// const KEY: Key128 = Key128::from_u32([0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC, 0xDDDDDDDD]);
    /// assert_eq!(KEY, Key128::from_u128(0xAAAAAAAABBBBBBBBCCCCCCCCDDDDDDDD));
    /// ```
    pub const fn from_u32(key: [u32; 4]) -> Key128 {
        Key128 { key }
    }
}

impl From<[u32; 4]> for Key128 {
    fn from(key: [u32; 4]) -> Self {
        Key128::from_u32(key)
    }
}

impl From<u128> for Key128 {
    fn from(key: u128) -> Self {
        Key128::from_u128(key)
    }
}

/// 256-bit AES key.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub struct Key256 {
    key: [u32; 8],
}

impl Key256 {
    /// Create a new 256-bit key from 8 dwords.
    ///
    /// This is the native key format of the hardware.
    pub const fn from_u32(key: [u32; 8]) -> Key256 {
        Key256 { key }
    }
}

/// AES key sizes.
pub enum Key {
    /// 128-bit key
    K128(Key128),
    /// 256-bit key
    K256(Key256),
}

impl From<Key128> for Key {
    fn from(k: Key128) -> Self {
        Key::K128(k)
    }
}

impl From<Key256> for Key {
    fn from(k: Key256) -> Self {
        Key::K256(k)
    }
}

impl Key {
    pub(crate) const fn keysize(&self) -> bool {
        match self {
            Key::K128(_) => false,
            Key::K256(_) => true,
        }
    }

    pub(crate) const fn key(&self) -> &[u32] {
        match self {
            Key::K128(k) => &k.key,
            Key::K256(k) => &k.key,
        }
    }

    /// Returns `true` if the key is a 128-bit key.
    pub fn is_128bit(&self) -> bool {
        matches!(self, Self::K128(..))
    }

    /// Returns `true` if the key is a 256-bit key.
    pub fn is_256bit(&self) -> bool {
        matches!(self, Self::K256(..))
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
#[non_exhaustive] // add timeout at some point
pub enum Error {
    /// Unexpected read operation from the AES_DOUTR register
    /// during computation or data input phase.
    Read,
    /// Unexpected write operation to the AES_DINR register
    /// during computation or data output phase.
    Write,
}

/// AES driver.
pub struct Aes {
    aes: pac::AES,
}

impl Aes {
    /// Create a new AES driver from an AES peripheral.
    ///
    /// This will reset the AES, but it will not enable clocks for the AES.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rcc = dp.RCC;
    ///
    /// // ... setup the system clocks
    ///
    /// rcc.ahb3enr.modify(|_, w| w.aesen().set_bit());
    /// rcc.ahb3enr.read(); // Delay after an RCC peripheral clock enabling
    ///
    /// let mut aes = Aes::new(dp.AES, &mut rcc);
    /// ```
    pub fn new(aes: pac::AES, rcc: &mut pac::RCC) -> Aes {
        rcc.ahb3rstr.modify(|_, w| w.aesrst().set_bit());
        rcc.ahb3rstr.modify(|_, w| w.aesrst().clear_bit());

        Aes { aes }
    }

    /// Free the AES peripheral from the driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rcc = dp.RCC;
    /// let aes = dp.AES;
    ///
    /// // ... setup the system clocks
    ///
    /// rcc.ahb3enr.modify(|_, w| w.aesen().set_bit());
    /// rcc.ahb3enr.read(); // Delay after an RCC peripheral clock enabling
    ///
    /// let mut aes_driver = Aes::new(aes, &mut rcc);
    /// // ... use AES
    /// let aes = aes_driver.free();
    /// ```
    pub fn free(self) -> pac::AES {
        self.aes
    }

    /// Steal the AES peripheral from whatever is currently using it.
    ///
    /// This will **not** initialize the AES peripheral (unlike [`new`]).
    ///
    /// # Safety
    ///
    /// This will create a new AES peripheral, bypassing the singleton checks
    /// that normally occur.
    /// You are responsible for ensuring that the driver has exclusive access to
    /// the AES peripheral.
    /// You are also responsible for ensuring the AES peripheral has been setup
    /// correctly.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::aes::Aes;
    ///
    /// // ... setup happens here
    ///
    /// let aes = unsafe { Aes::steal() };
    /// ```
    ///
    /// [`new`]: Aes::new
    pub unsafe fn steal() -> Aes {
        let dp: pac::Peripherals = pac::Peripherals::steal();
        Aes { aes: dp.AES }
    }

    fn set_key(&mut self, key: &Key) {
        self.aes.keyr0.write(|w| unsafe { w.bits(key.key()[0]) });
        self.aes.keyr1.write(|w| unsafe { w.bits(key.key()[1]) });
        self.aes.keyr2.write(|w| unsafe { w.bits(key.key()[2]) });
        self.aes.keyr3.write(|w| unsafe { w.bits(key.key()[3]) });
        if key.is_256bit() {
            self.aes.keyr4.write(|w| unsafe { w.bits(key.key()[4]) });
            self.aes.keyr5.write(|w| unsafe { w.bits(key.key()[5]) });
            self.aes.keyr6.write(|w| unsafe { w.bits(key.key()[6]) });
            self.aes.keyr7.write(|w| unsafe { w.bits(key.key()[7]) });
        }
    }

    fn poll_completion(&self) -> Result<(), Error> {
        // TODO: timeouts
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

    /// Encrypt using the electronic codebook chaining (ECB) algorithm.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::aes::{Key, Key128};
    /// # let mut aes = unsafe { stm32wl_hal::aes::Aes::steal() };
    ///
    /// // this is a bad key, I am just using values from the NIST testsuite
    /// const KEY: Key = Key::K128(Key128::from_u128(0));
    ///
    /// let plaintext: [u32; 4] = [0xf34481ec, 0x3cc627ba, 0xcd5dc3fb, 0x08f273e6];
    /// let chiphertext: [u32; 4] = aes.encrypt_ecb(&KEY, &plaintext)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn encrypt_ecb(&mut self, key: &Key, plaintext: &[u32; 4]) -> Result<[u32; 4], Error> {
        const ALGO: Algorithm = Algorithm::Ecb;
        const CHMOD2: bool = ALGO.chmod2();
        const CHMOD10: u8 = ALGO.chmod10();
        const MODE: u8 = Mode::Encryption.bits();

        #[rustfmt::skip]
        self.aes.cr.write(|w|
            w
                .en().set_bit()
                .datatype().bits(0b00)
                .mode().bits(MODE)
                .chmod2().bit(CHMOD2)
                .chmod().bits(CHMOD10)
                .ccfc().set_bit()
                .errc().set_bit()
                .ccfie().set_bit()
                .errie().set_bit()
                .dmainen().clear_bit()
                .dmaouten().clear_bit()
                .gcmph().bits(0) // do not care for ECB
                .keysize().bit(key.keysize())
                .npblb().bits(0) // no padding
        );

        self.set_key(key);

        for &dw in plaintext.iter() {
            self.aes.dinr.write(|w| unsafe { w.bits(dw) });
        }

        self.poll_completion()?;

        let mut ciphertext: [u32; 4] = [0; 4];
        for dw in ciphertext.iter_mut() {
            *dw = self.aes.doutr.read().bits();
        }
        self.aes.cr.write(|w| w.en().clear_bit());
        Ok(ciphertext)
    }

    /// Decrypt using the electronic codebook chaining (ECB) algorithm.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::aes::{Key, Key128};
    /// # let mut aes = unsafe { stm32wl_hal::aes::Aes::steal() };
    ///
    /// // this is a bad key, I am just using values from the NIST testsuite
    /// const KEY: Key = Key::K128(Key128::from_u128(0));
    ///
    /// let ciphertext: [u32; 4] = [0x0336763e, 0x966d9259, 0x5a567cc9, 0xce537f5e];
    /// let plaintext: [u32; 4] = aes.decrypt_ecb(&KEY, &ciphertext)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn decrypt_ecb(&mut self, key: &Key, ciphertext: &[u32; 4]) -> Result<[u32; 4], Error> {
        const ALGO: Algorithm = Algorithm::Ecb;
        const CHMOD2: bool = ALGO.chmod2();
        const CHMOD10: u8 = ALGO.chmod10();
        const MODE: u8 = Mode::KeyDerivationDecryption.bits();

        #[rustfmt::skip]
        self.aes.cr.write(|w|
            w
                .en().set_bit()
                .datatype().bits(0b00)
                .mode().bits(MODE)
                .chmod2().bit(CHMOD2)
                .chmod().bits(CHMOD10)
                .ccfc().set_bit()
                .errc().set_bit()
                .ccfie().set_bit()
                .errie().set_bit()
                .dmainen().clear_bit()
                .dmaouten().clear_bit()
                .gcmph().bits(0) // do not care for ECB
                .keysize().bit(key.keysize())
                .npblb().bits(0) // no padding
        );

        self.set_key(key);

        for &dw in ciphertext.iter() {
            self.aes.dinr.write(|w| unsafe { w.bits(dw) });
        }

        self.poll_completion()?;

        let mut plaintext: [u32; 4] = [0; 4];
        for dw in plaintext.iter_mut() {
            *dw = self.aes.doutr.read().bits();
        }
        self.aes.cr.write(|w| w.en().clear_bit());
        Ok(plaintext)
    }
}
