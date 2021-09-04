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

    // expensive copy for the sake of allowing unaligned u8 data
    fn set_din_block(&mut self, block: &[u8]) {
        for chunk in block.chunks(4) {
            let din: u32 = (chunk.get(0).copied().unwrap_or(0) as u32) << 24
                | (chunk.get(1).copied().unwrap_or(0) as u32) << 16
                | (chunk.get(2).copied().unwrap_or(0) as u32) << 8
                | (chunk.get(3).copied().unwrap_or(0) as u32);

            self.aes.dinr.write(|w| w.din().bits(din));
        }
    }

    // expensive copy for the sake of allowing unaligned u8 data
    fn dout_block(&mut self, block: &mut [u8]) {
        for chunk in block.chunks_mut(4) {
            let dout: u32 = self.aes.doutr.read().bits();
            if let Some(byte) = chunk.get_mut(0) {
                *byte = (dout >> 24) as u8
            }
            if let Some(byte) = chunk.get_mut(1) {
                *byte = (dout >> 16) as u8
            }
            if let Some(byte) = chunk.get_mut(2) {
                *byte = (dout >> 8) as u8
            }
            if let Some(byte) = chunk.get_mut(3) {
                *byte = dout as u8
            }
        }
    }

    fn gcm_inplace(
        &mut self,
        mode: Mode,
        key: &[u32],
        iv: &[u32; 3],
        aad: &[u8],
        buf: &mut [u8],
        tag: &mut [u32; 4],
    ) -> Result<(), Error> {
        const ALGO: Algorithm = Algorithm::Gcm;
        const CHMOD2: bool = ALGO.chmod2();
        const CHMOD10: u8 = ALGO.chmod10();
        let mode: u8 = mode.bits();

        // init phase
        let keysize: KeySize = self.set_key(key);
        self.aes.ivr0.write(|w| w.ivi().bits(2));
        self.aes.ivr1.write(|w| w.ivi().bits(iv[2]));
        self.aes.ivr2.write(|w| w.ivi().bits(iv[1]));
        self.aes.ivr3.write(|w| w.ivi().bits(iv[0]));
        #[rustfmt::skip]
        self.aes.cr.write(|w|
            w
                .en().enabled()
                .datatype().none()
                .mode().bits(mode)
                .chmod2().bit(CHMOD2)
                .chmod().bits(CHMOD10)
                .ccfc().clear()
                .errc().clear()
                .ccfie().disabled()
                .errie().disabled()
                .dmainen().disabled()
                .dmaouten().disabled()
                .gcmph().init()
                .keysize().variant(keysize)
                .npblb().bits(0)
        );
        self.poll_completion()?;

        // header phase
        for block in aad.chunks(16) {
            #[rustfmt::skip]
            self.aes.cr.write(|w|
                w
                    .en().enabled()
                    .datatype().none()
                    .mode().bits(mode)
                    .chmod2().bit(CHMOD2)
                    .chmod().bits(CHMOD10)
                    .ccfc().clear()
                    .errc().clear()
                    .ccfie().disabled()
                    .errie().disabled()
                    .dmainen().disabled()
                    .dmaouten().disabled()
                    .gcmph().header()
                    .keysize().variant(keysize)
                    .npblb().bits(16 - (block.len() as u8))
            );
            self.set_din_block(block);
            self.poll_completion()?;
        }

        // payload phase
        for block in buf.chunks_mut(16) {
            #[rustfmt::skip]
            self.aes.cr.write(|w|
                w
                    .en().enabled()
                    .datatype().none()
                    .mode().bits(mode)
                    .chmod2().bit(CHMOD2)
                    .chmod().bits(CHMOD10)
                    .ccfc().clear()
                    .errc().clear()
                    .ccfie().disabled()
                    .errie().disabled()
                    .dmainen().disabled()
                    .dmaouten().disabled()
                    .gcmph().payload()
                    .keysize().variant(keysize)
                    .npblb().bits(16 - (block.len() as u8))
            );
            self.set_din_block(block);
            self.poll_completion()?;
            self.dout_block(block);
        }

        // final phase
        #[rustfmt::skip]
        self.aes.cr.write(|w|
            w
                .en().enabled()
                .datatype().none()
                .mode().bits(mode)
                .chmod2().bit(CHMOD2)
                .chmod().bits(CHMOD10)
                .ccfc().clear()
                .errc().clear()
                .ccfie().disabled()
                .errie().disabled()
                .dmainen().disabled()
                .dmaouten().disabled()
                .gcmph().final_()
                .keysize().variant(keysize)
                .npblb().bits(0)
        );

        // byte length to bit lengths
        // impossible to overflow, not enough RAM for [u8; (u32::MAX >> 3) + 1]
        let aad_len: u32 = (aad.len() as u32) << 3;
        let buf_len: u32 = (buf.len() as u32) << 3;

        self.aes.dinr.write(|w| w.din().bits(0));
        self.aes.dinr.write(|w| w.din().bits(aad_len));
        self.aes.dinr.write(|w| w.din().bits(0));
        self.aes.dinr.write(|w| w.din().bits(buf_len));

        self.poll_completion()?;
        self.dout(tag);
        Ok(())
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
        self.poll_completion()?;
        self.dout(ciphertext);
        Ok(())
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
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let mut text: [u32; 4] = [0xf34481ec, 0x3cc627ba, 0xcd5dc3fb, 0x08f273e6];
    /// aes.encrypt_ecb_inplace(&KEY, &mut text)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn encrypt_ecb_inplace(
        &mut self,
        key: &[u32],
        plaintext: &mut [u32; 4],
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
        self.poll_completion()?;
        self.dout(plaintext);
        Ok(())
    }

    /// Encrypt using the Galois counter mode (GCM) algorithm in-place.
    ///
    /// # Panics
    ///
    /// * Key is not 128-bits long (`[u32; 4]`) or 256-bits long (`[u32; 8]`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     aes::Aes,
    ///     pac,
    ///     rng::{self, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut aes: Aes = Aes::new(dp.AES, &mut dp.RCC);
    /// let mut rng = Rng::new(dp.RNG, rng::Clk::MSI, &mut dp.RCC);
    ///
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let mut iv: [u32; 3] = [0; 3];
    /// rng.try_fill_u32(&mut iv)
    ///     .expect("failed to generate entropy");
    ///
    /// let mut associated_data: [u8; 0] = [];
    /// let mut plaintext: [u8; 13] = b"Hello, World!".clone();
    /// let mut tag: [u32; 4] = [0; 4];
    /// aes.encrypt_gcm_inplace(&KEY, &iv, &associated_data, &mut plaintext, &mut tag)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn encrypt_gcm_inplace(
        &mut self,
        key: &[u32],
        iv: &[u32; 3],
        aad: &[u8],
        plaintext: &mut [u8],
        tag: &mut [u32; 4],
    ) -> Result<(), Error> {
        self.gcm_inplace(Mode::Encryption, key, iv, aad, plaintext, tag)
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
        self.poll_completion()?;
        self.dout(plaintext);
        Ok(())
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
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let mut text: [u32; 4] = [0x0336763e, 0x966d9259, 0x5a567cc9, 0xce537f5e];
    /// aes.decrypt_ecb_inplace(&KEY, &mut text)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn decrypt_ecb_inplace(
        &mut self,
        key: &[u32],
        ciphertext: &mut [u32; 4],
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
        self.poll_completion()?;
        self.dout(ciphertext);
        Ok(())
    }

    /// Decrypt using the Galois counter mode (GCM) algorithm in-place.
    ///
    /// The resulting tag should be compared to the tag sent from the peer
    /// to verify the authenticity of the message.
    ///
    /// # Panics
    ///
    /// * Key is not 128-bits long (`[u32; 4]`) or 256-bits long (`[u32; 8]`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{
    ///     aes::Aes,
    ///     pac,
    ///     rng::{self, Rng},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut aes: Aes = Aes::new(dp.AES, &mut dp.RCC);
    /// let mut rng = Rng::new(dp.RNG, rng::Clk::MSI, &mut dp.RCC);
    ///
    /// const KEY: [u32; 4] = [0; 4];
    /// const IV: [u32; 3] = [0; 3];
    ///
    /// let mut associated_data: [u8; 0] = [];
    /// let mut ciphertext: [u8; 5] = [0xf3, 0x44, 0x81, 0xec, 0x3c];
    /// let mut tag: [u32; 4] = [0; 4];
    /// aes.decrypt_gcm_inplace(&KEY, &IV, &associated_data, &mut ciphertext, &mut tag)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn decrypt_gcm_inplace(
        &mut self,
        key: &[u32],
        iv: &[u32; 3],
        aad: &[u8],
        ciphertext: &mut [u8],
        tag: &mut [u32; 4],
    ) -> Result<(), Error> {
        self.gcm_inplace(Mode::Decryption, key, iv, aad, ciphertext, tag)
    }
}
