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
#[non_exhaustive] // add timeout at some point
pub enum Error {
    /// Unexpected read operation from the AES_DOUTR register
    /// during computation or data input phase.
    Read,
    /// Unexpected write operation to the AES_DINR register
    /// during computation or data output phase.
    Write,
}

impl Error {
    #[cfg_attr(
        not(all(feature = "aio", not(feature = "stm32wl5x_cm0p"))),
        allow(dead_code)
    )]
    pub(crate) fn from_sr(sr: u32) -> Result<(), Self> {
        const RDERR: u32 = 1 << 1;
        const WRERR: u32 = 1 << 2;
        if sr & RDERR != 0 {
            Err(Error::Read)
        } else if sr & WRERR != 0 {
            Err(Error::Write)
        } else {
            Ok(())
        }
    }
}

fn keysize(key: &[u32]) -> KeySize {
    match key.len() {
        4 => KeySize::BITS128,
        8 => KeySize::BITS256,
        _ => panic!("Key must be 128-bit or 256-bit not {}-bit", key.len() * 32),
    }
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
    /// Synchronous (normal) usage:
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let mut aes = Aes::new(dp.AES, &mut dp.RCC);
    /// ```
    ///
    /// Asynchronous usage requires the AES interrupt unmasked in the NVIC:
    ///
    /// ```no_run
    /// use stm32wl_hal::{aes::Aes, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let mut aes = Aes::new(dp.AES, &mut dp.RCC);
    /// # #[cfg(all(not(feature = "stm32wl5x_cm0p"), feature = "rt"))]
    /// unsafe { Aes::unmask_irq() };
    /// ```
    pub fn new(aes: pac::AES, rcc: &mut pac::RCC) -> Aes {
        rcc.ahb3enr.modify(|_, w| w.aesen().set_bit());
        rcc.ahb3enr.read(); // delay after an RCC peripheral clock enabling

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
    pub unsafe fn unmask_irq() {
        pac::NVIC::unmask(pac::Interrupt::AES)
    }

    fn set_key(&mut self, key: &[u32]) {
        self.aes.keyr0.write(|w| w.key().bits(key[0]));
        self.aes.keyr1.write(|w| w.key().bits(key[1]));
        self.aes.keyr2.write(|w| w.key().bits(key[2]));
        self.aes.keyr3.write(|w| w.key().bits(key[3]));
        if key.len() > 4 {
            self.aes.keyr4.write(|w| w.key().bits(key[4]));
            self.aes.keyr5.write(|w| w.key().bits(key[5]));
            self.aes.keyr6.write(|w| w.key().bits(key[6]));
            self.aes.keyr7.write(|w| w.key().bits(key[7]));
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

    fn set_din(&mut self, din: &[u32; 4]) {
        din.iter()
            .for_each(|dw| self.aes.dinr.write(|w| w.din().bits(*dw)))
    }

    fn dout(&self) -> [u32; 4] {
        let mut ret: [u32; 4] = [0; 4];
        ret.iter_mut()
            .for_each(|dw| *dw = self.aes.doutr.read().bits());
        ret
    }

    /// Encrypt using the electronic codebook chaining (ECB) algorithm.
    ///
    /// # Panics
    ///
    /// * Key is not 128-bits long (4 `u32`) or 256-bits long (8 `u32`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut aes = unsafe { stm32wl_hal::aes::Aes::steal() };
    ///
    /// // this is a bad key, I am just using values from the NIST testsuite
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let plaintext: [u32; 4] = [0xf34481ec, 0x3cc627ba, 0xcd5dc3fb, 0x08f273e6];
    /// let chiphertext: [u32; 4] = aes.encrypt_ecb(&KEY, &plaintext)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn encrypt_ecb(&mut self, key: &[u32], plaintext: &[u32; 4]) -> Result<[u32; 4], Error> {
        const ALGO: Algorithm = Algorithm::Ecb;
        const CHMOD2: bool = ALGO.chmod2();
        const CHMOD10: u8 = ALGO.chmod10();
        const MODE: u8 = Mode::Encryption.bits();

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
                .keysize().variant(keysize(key))
                .npblb().bits(0) // no padding
        );

        self.set_key(key);
        self.set_din(plaintext);
        let ret: Result<[u32; 4], Error> = match self.poll_completion() {
            Ok(_) => Ok(self.dout()),
            Err(e) => Err(e),
        };

        self.aes.cr.write(|w| w.en().clear_bit());
        ret
    }

    /// Decrypt using the electronic codebook chaining (ECB) algorithm.
    ///
    /// # Panics
    ///
    /// * Key is not 128-bits long (4 `u32`) or 256-bits long (8 `u32`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut aes = unsafe { stm32wl_hal::aes::Aes::steal() };
    ///
    /// // this is a bad key, I am just using values from the NIST testsuite
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let ciphertext: [u32; 4] = [0x0336763e, 0x966d9259, 0x5a567cc9, 0xce537f5e];
    /// let plaintext: [u32; 4] = aes.decrypt_ecb(&KEY, &ciphertext)?;
    /// # Ok::<(), stm32wl_hal::aes::Error>(())
    /// ```
    pub fn decrypt_ecb(&mut self, key: &[u32], ciphertext: &[u32; 4]) -> Result<[u32; 4], Error> {
        const ALGO: Algorithm = Algorithm::Ecb;
        const CHMOD2: bool = ALGO.chmod2();
        const CHMOD10: u8 = ALGO.chmod10();
        const MODE: u8 = Mode::KeyDerivationDecryption.bits();

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
                .keysize().variant(keysize(key))
                .npblb().bits(0) // no padding
        );

        self.set_key(key);
        self.set_din(ciphertext);
        let ret: Result<[u32; 4], Error> = match self.poll_completion() {
            Ok(_) => Ok(self.dout()),
            Err(e) => Err(e),
        };

        self.aes.cr.write(|w| w.en().clear_bit());
        ret
    }

    /// Encrypt using the electronic codebook chaining (ECB) algorithm.
    ///
    /// # Panics
    ///
    /// * Key is not 128-bits long (4 `u32`) or 256-bits long (8 `u32`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// # async fn doctest() -> Result<(), stm32wl_hal::aes::Error> {
    /// # let mut aes = unsafe { stm32wl_hal::aes::Aes::steal() };
    ///
    /// // this is a bad key, I am just using values from the NIST testsuite
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let plaintext: [u32; 4] = [0xf34481ec, 0x3cc627ba, 0xcd5dc3fb, 0x08f273e6];
    /// let chiphertext: [u32; 4] = aes.aio_encrypt_ecb(&KEY, &plaintext).await?;
    /// # Ok(()) }
    /// ```
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    #[cfg_attr(
        docsrs,
        doc(cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p"))))
    )]
    pub async fn aio_encrypt_ecb(
        &mut self,
        key: &[u32],
        ciphertext: &[u32; 4],
    ) -> Result<[u32; 4], Error> {
        const ALGO: Algorithm = Algorithm::Ecb;
        const CHMOD2: bool = ALGO.chmod2();
        const CHMOD10: u8 = ALGO.chmod10();
        const MODE: u8 = Mode::Encryption.bits();

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
                .ccfie().enabled()
                .errie().enabled()
                .dmainen().disabled()
                .dmaouten().disabled()
                .gcmph().bits(0) // do not care for ECB
                .keysize().variant(keysize(key))
                .npblb().bits(0) // no padding
        );

        self.set_key(key);
        self.set_din(ciphertext);

        let ret: Result<[u32; 4], Error> = match futures::future::poll_fn(aio::poll).await {
            Ok(_) => Ok(self.dout()),
            Err(e) => Err(e),
        };
        self.aes.cr.write(|w| w.en().clear_bit());
        ret
    }

    /// Decrypt using the electronic codebook chaining (ECB) algorithm,
    /// asynchronously.
    ///
    /// # Panics
    ///
    /// * Key is not 128-bits long (4 `u32`) or 256-bits long (8 `u32`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// # async fn doctest() -> Result<(), stm32wl_hal::aes::Error> {
    /// # let mut aes = unsafe { stm32wl_hal::aes::Aes::steal() };
    ///
    /// // this is a bad key, I am just using values from the NIST testsuite
    /// const KEY: [u32; 4] = [0; 4];
    ///
    /// let ciphertext: [u32; 4] = [0x0336763e, 0x966d9259, 0x5a567cc9, 0xce537f5e];
    /// let plaintext: [u32; 4] = aes.aio_decrypt_ecb(&KEY, &ciphertext).await?;
    /// # Ok(()) }
    /// ```
    #[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
    #[cfg_attr(
        docsrs,
        doc(cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p"))))
    )]
    pub async fn aio_decrypt_ecb(
        &mut self,
        key: &[u32],
        ciphertext: &[u32; 4],
    ) -> Result<[u32; 4], Error> {
        const ALGO: Algorithm = Algorithm::Ecb;
        const CHMOD2: bool = ALGO.chmod2();
        const CHMOD10: u8 = ALGO.chmod10();
        const MODE: u8 = Mode::KeyDerivationDecryption.bits();

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
                .ccfie().enabled()
                .errie().enabled()
                .dmainen().disabled()
                .dmaouten().disabled()
                .gcmph().bits(0) // do not care for ECB
                .keysize().variant(keysize(key))
                .npblb().bits(0) // no padding
        );

        self.set_key(key);
        self.set_din(ciphertext);

        let ret: Result<[u32; 4], Error> = match futures::future::poll_fn(aio::poll).await {
            Ok(_) => Ok(self.dout()),
            Err(e) => Err(e),
        };
        self.aes.cr.write(|w| w.en().clear_bit());
        ret
    }
}

#[cfg(all(feature = "aio", not(feature = "stm32wl5x_cm0p")))]
mod aio {
    use core::{
        sync::atomic::{AtomicU32, Ordering::SeqCst},
        task::Poll,
    };
    use futures_util::task::AtomicWaker;

    static AES_WAKER: AtomicWaker = AtomicWaker::new();
    static AES_RESULT: AtomicU32 = AtomicU32::new(0);

    pub fn poll(cx: &mut core::task::Context<'_>) -> Poll<Result<(), super::Error>> {
        AES_WAKER.register(cx.waker());
        match AES_RESULT.load(SeqCst) {
            0 => core::task::Poll::Pending,
            _ => {
                AES_WAKER.take();
                let sr: u32 = AES_RESULT.swap(0, SeqCst);
                Poll::Ready(super::Error::from_sr(sr))
            }
        }
    }

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    mod irq {
        use super::{SeqCst, AES_RESULT, AES_WAKER};
        use crate::pac::{self, interrupt};

        #[interrupt]
        #[allow(non_snake_case)]
        fn AES() {
            debug_assert_eq!(AES_RESULT.load(SeqCst), 0);

            let dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };

            // store result
            AES_RESULT.store(dp.AES.sr.read().bits(), SeqCst);

            // clear and disable IRQs
            #[rustfmt::skip]
            dp.AES.cr.modify(|_, w| {
                w
                    .ccfc().clear()
                    .errc().clear()
                    .ccfie().disabled()
                    .errie().disabled()
            });

            AES_WAKER.wake();
        }
    }
}
