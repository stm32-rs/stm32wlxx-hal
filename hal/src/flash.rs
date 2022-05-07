//! Flash memory
//!
//! Quickstart:
//!
//! 1. [`Flash::unlock`]
//! 2. [`Flash::page_erase`]
//! 3. [`Flash::program_bytes`]

use crate::pac;
use core::{mem::size_of, ops::Range, ptr::write_volatile, slice::ChunksExact};
use num_integer::Integer;

/// Starting address of the flash memory.
pub const FLASH_START: usize = 0x0800_0000;

/// Ending address of the flash memory.
///
/// This is calculated at runtime using the info registers.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::flash::flash_end;
///
/// // valid for the nucleo-wl55jc with 256k flash
/// assert_eq!(flash_end(), 0x0803_FFFF);
/// ```
#[inline]
pub fn flash_end() -> usize {
    const OFFSET: usize = FLASH_START - 1;
    OFFSET + crate::info::flash_size() as usize
}

/// Range of flash memory.
///
/// This is calculated at runtime using the info registers.
///
/// # Example
///
/// ```no_run
/// use core::ops::Range;
/// use stm32wlxx_hal::flash::flash_range;
///
/// // valid for the nucleo-wl55jc with 256k flash
/// assert_eq!(
///     flash_range(),
///     Range {
///         start: 0x0800_0000,
///         end: 0x0804_0000
///     }
/// );
/// assert!(flash_range().contains(&0x0800_0000));
/// assert!(flash_range().contains(&0x0803_FFFF));
/// assert!(!flash_range().contains(&0x0804_0000));
/// ```
#[inline]
pub fn flash_range() -> Range<usize> {
    Range {
        start: FLASH_START,
        end: FLASH_START + crate::info::flash_size() as usize,
    }
}

/// Number of flash pages.
///
/// This is calculated at runtime using the info registers.
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::flash::num_pages;
///
/// // valid for the nucleo-wl55jc with 256k flash
/// assert_eq!(num_pages(), 0x80);
/// ```
#[inline]
pub fn num_pages() -> u8 {
    (crate::info::flash_size_kibibyte() / 2) as u8
}

// status register (SR) flags
mod flags {
    pub const PROGERR: u32 = 1 << 3;
    pub const WRPERR: u32 = 1 << 4;
    pub const PGAERR: u32 = 1 << 5;
    pub const SIZERR: u32 = 1 << 6;
    pub const PGSERR: u32 = 1 << 7;
    pub const MISSERR: u32 = 1 << 8;
    pub const BSY: u32 = 1 << 16;
    pub const PESD: u32 = 1 << 19;
}

/// Page address.
#[derive(Debug, PartialEq, Eq, Clone, Copy, PartialOrd, Ord, Hash)]
pub struct Page {
    idx: u8,
}

impl Page {
    /// Page size in bytes.
    pub const SIZE: usize = 2048;

    /// Create a page address from an index without checking bounds.
    ///
    /// # Safety
    ///
    /// 1. The `idx` argument must be a valid page number, less than the value
    ///    returned by [`num_pages`].
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::flash::Page;
    ///
    /// let page0 = unsafe { Page::from_index_unchecked(0) };
    /// ```
    #[inline]
    pub const unsafe fn from_index_unchecked(idx: u8) -> Self {
        Page { idx }
    }

    /// Create a page address from an index.
    ///
    /// Returns `None` if the value index is greater than the index of the last
    /// page, for example `0x7F` (page 127) on the STM32WLE5.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::flash::Page;
    ///
    /// assert!(Page::from_index(8).is_some());
    /// assert!(Page::from_index(128).is_none());
    /// ```
    pub fn from_index(idx: u8) -> Option<Self> {
        if idx >= num_pages() {
            None
        } else {
            Some(Page { idx })
        }
    }

    /// Create a page address from an offset from the base of the flash memory.
    ///
    /// Returns `None` if the address is out of bounds, or not page aligned.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::flash::Page;
    ///
    /// assert_eq!(Page::from_byte_offset(0), Page::from_index(0));
    /// assert_eq!(Page::from_byte_offset(2048), Page::from_index(1));
    /// assert!(Page::from_byte_offset(2047).is_none());
    /// assert!(Page::from_byte_offset(usize::MAX).is_none());
    /// ```
    pub fn from_byte_offset(offset: usize) -> Option<Self> {
        if offset % Self::SIZE == 0 {
            let idx: usize = offset / Self::SIZE;
            if idx >= usize::from(num_pages()) {
                None
            } else {
                Some(Page { idx: idx as u8 })
            }
        } else {
            None
        }
    }

    /// Create a page address from an absolute address.
    ///
    /// Returns `None` if the address is out of bounds, or not page aligned.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::flash::Page;
    ///
    /// assert_eq!(Page::from_addr(0x0800_0000), Page::from_index(0));
    /// assert_eq!(Page::from_addr(0x0800_0800), Page::from_index(1));
    /// assert!(Page::from_addr(0).is_none());
    /// assert!(Page::from_addr(usize::MAX).is_none());
    /// assert!(Page::from_addr(0x0800_0001).is_none());
    /// ```
    pub fn from_addr(addr: usize) -> Option<Self> {
        if let Some(offset) = addr.checked_sub(FLASH_START) {
            Self::from_byte_offset(offset)
        } else {
            None
        }
    }

    /// Get the page index.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::flash::Page;
    ///
    /// let page7 = unsafe { Page::from_index_unchecked(7) };
    /// assert_eq!(page7.to_index(), 7);
    /// ```
    #[inline]
    pub const fn to_index(self) -> u8 {
        self.idx
    }

    /// Get the page address.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::flash::Page;
    ///
    /// let page127 = unsafe { Page::from_index_unchecked(127) };
    /// let page0 = unsafe { Page::from_index_unchecked(0) };
    ///
    /// assert_eq!(page0.addr(), 0x0800_0000);
    /// assert_eq!(page127.addr(), 0x0803_F800);
    /// ```
    pub const fn addr(&self) -> usize {
        (self.idx as usize) * Self::SIZE + FLASH_START
    }

    /// Get the address range of the page.
    ///
    /// # Example
    ///
    /// ```
    /// use core::ops::Range;
    /// use stm32wlxx_hal::flash::Page;
    ///
    /// let page0 = unsafe { Page::from_index_unchecked(0) };
    /// assert_eq!(
    ///     page0.addr_range(),
    ///     Range {
    ///         start: 0x0800_0000,
    ///         end: 0x0800_0800
    ///     }
    /// );
    /// ```
    pub const fn addr_range(&self) -> Range<usize> {
        Range {
            start: self.addr(),
            end: self.addr() + Page::SIZE,
        }
    }
}

impl From<Page> for AlignedAddr {
    #[inline]
    fn from(page: Page) -> Self {
        AlignedAddr { addr: page.addr() }
    }
}

/// Error for conversions to [`AlignedAddr`].
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AlignedAddrError(pub(crate) ());

impl AlignedAddrError {
    pub(crate) const fn new() -> Self {
        Self(())
    }
}

/// A `u64` aligned flash address.
///
/// An argument of [`Flash::program_bytes`].
///
/// # Example
///
/// Create an aligned flash address by converting from `usize`.
///
/// ```no_run
/// use stm32wlxx_hal::flash::AlignedAddr;
///
/// let addr: AlignedAddr = AlignedAddr::try_from(0x0803_F800_usize)?;
/// # Ok::<(), stm32wlxx_hal::flash::AlignedAddrError>(())
/// ```
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AlignedAddr {
    addr: usize,
}

impl AlignedAddr {
    /// Create a page address from an index without checking bounds.
    ///
    /// # Safety
    ///
    /// 1. The `addr` argument must be a multiple of 8.
    /// 2. The `addr` argument must be a valid flash address, within the range
    ///    returned by [`flash_range`].
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::flash::Page;
    ///
    /// let page0 = unsafe { Page::from_index_unchecked(0) };
    /// ```
    pub const unsafe fn new_unchecked(addr: usize) -> Self {
        Self { addr }
    }
}

impl From<AlignedAddr> for usize {
    #[inline]
    fn from(addr: AlignedAddr) -> Self {
        addr.addr
    }
}

impl From<AlignedAddr> for u32 {
    #[inline]
    fn from(addr: AlignedAddr) -> Self {
        addr.addr as u32
    }
}

impl TryFrom<u32> for AlignedAddr {
    type Error = AlignedAddrError;

    fn try_from(addr: u32) -> Result<Self, Self::Error> {
        Self::try_from(addr as usize)
    }
}

impl TryFrom<usize> for AlignedAddr {
    type Error = AlignedAddrError;

    fn try_from(addr: usize) -> Result<Self, Self::Error> {
        if addr % size_of::<u64>() != 0 || !flash_range().contains(&addr) {
            Err(AlignedAddrError::new())
        } else {
            Ok(AlignedAddr { addr })
        }
    }
}

/// Flash errors.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Busy error.
    ///
    /// A flash programming sequence was started while the previous sequence
    /// was still in-progress.
    Busy,
    /// Program erase suspend error.
    ///
    /// A flash programming sequence was started with a program erase suspend
    /// bit set.
    Suspend,
    /// Overflow error.
    ///
    /// Returned by [`Flash::standard_program_generic`], or [`Flash::program_bytes`]
    /// when the target data and address would exceed the end of flash memory.
    Overflow,
    /// Fast programming data miss error.
    ///
    /// In Fast programming mode, 32 double-words (256 bytes) must be sent to
    /// the flash memory successively and the new data must be sent to the logic
    /// control before the current data is fully programmed.
    ///
    /// This bit is set by hardware when the new data is not present in time.
    Miss,
    /// Programming sequence error.
    ///
    /// This bit is set by hardware when a write access to the flash memory is
    /// performed by the code, while PG or FSTPG have not been set previously.
    ///
    /// This bit is also set by hardware when PROGERR, SIZERR, PGAERR, WRPERR,
    /// MISSERR or FASTERR is set due to a previous programming error.
    Seq,
    /// Size error.
    ///
    /// This bit is set by hardware when the size of the access is a byte (`u8`)
    /// or half-word (`u16`) during a program or a fast program sequence.
    /// Only double-word (`u64`) programming is allowed (consequently: word (`u32`) access).
    Size,
    /// Programming alignment error.
    ///
    /// This bit is set by hardware when the data to program cannot be contained in the same
    /// double-word (`u64`) Flash memory in case of standard programming, or if there is a change
    /// of page during fast programming.
    Align,
    /// Write protection error.
    ///
    /// An address to be erased/programmed belongs to a write-protected part
    /// (by WRP, PCROP or RDP level 1) of the flash memory.
    Wp,
    /// Programming error.
    ///
    /// A 64-bit address to be programmed contains a value different from
    /// `0xFFFF_FFFF_FFFF_FFFF` before programming, except if the data to write
    /// is `0x0000_0000_0000_0000`.
    ///
    /// The erratum states that this will also occur when programming
    /// `0x0000_0000_0000_0000` to a location previously programmed with
    /// `0xFFFF_FFFF_FFFF_FFFF`.
    Prog,
}

/// Flash driver.
#[derive(Debug)]
pub struct Flash<'a> {
    flash: &'a mut pac::FLASH,
}

impl Drop for Flash<'_> {
    fn drop(&mut self) {
        // despite what RM0453 Rev 2 says there is no separate lock for core 2
        // as far as I can tell
        self.flash.cr.modify(|_, w| w.lock().set_bit())
    }
}

impl<'a> Flash<'a> {
    /// Unlock the flash memory for program or erase operations.
    ///
    /// The flash memory will be locked when this struct is dropped.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{flash::Flash, pac};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let mut flash: Flash = Flash::unlock(&mut dp.FLASH);
    /// ```
    pub fn unlock(flash: &'a mut pac::FLASH) -> Self {
        flash.keyr.write(|w| w.key().bits(0x4567_0123));
        flash.keyr.write(|w| w.key().bits(0xCDEF_89AB));
        Self { flash }
    }

    #[inline(always)]
    fn sr(&self) -> u32 {
        c1_c2!(self.flash.sr.read().bits(), self.flash.c2sr.read().bits())
    }

    #[inline(always)]
    fn clear_all_err(&mut self) {
        c1_c2!(
            self.flash.sr.write(|w| {
                w.rderr().clear();
                w.fasterr().clear();
                w.misserr().clear();
                w.pgserr().clear();
                w.sizerr().clear();
                w.pgaerr().clear();
                w.wrperr().clear();
                w.progerr().clear();
                w.operr().clear();
                w.eop().clear()
            }),
            self.flash.c2sr.write(|w| {
                w.rderr().clear();
                w.fasterr().clear();
                w.misserr().clear();
                w.pgserr().clear();
                w.sizerr().clear();
                w.pgaerr().clear();
                w.wrperr().clear();
                w.progerr().clear();
                w.operr().clear();
                w.eop().clear()
            }),
        )
    }

    #[inline(always)]
    fn wait_for_not_busy(&self) -> Result<(), Error> {
        loop {
            let sr: u32 = self.sr();

            // "This bit is set at the beginning of a Flash operation and
            // reset when the operation finishes or when an error occurs."
            if sr & flags::BSY == 0 {
                if sr & flags::PROGERR == flags::PROGERR {
                    return Err(Error::Prog);
                }
                if sr & flags::WRPERR == flags::WRPERR {
                    return Err(Error::Wp);
                }
                if sr & flags::PGAERR == flags::PGAERR {
                    return Err(Error::Align);
                }
                if sr & flags::SIZERR == flags::SIZERR {
                    return Err(Error::Size);
                }
                if sr & flags::MISSERR == flags::MISSERR {
                    return Err(Error::Miss);
                }
                // check last because it can be set with other flags
                if sr & flags::PGSERR == flags::PGSERR {
                    return Err(Error::Seq);
                }

                return Ok(());
            }
        }
    }

    /// Program 8 bytes.
    ///
    /// # Safety
    ///
    /// 1. Do not write to flash memory that is being used for your code.
    /// 2. The destination address must be within the flash memory region.
    /// 3. The `from` and `to` pointers must be aligned to the pointee type.
    #[allow(unused_unsafe)]
    pub unsafe fn standard_program(&mut self, from: *const u64, to: *mut u64) -> Result<(), Error> {
        let sr: u32 = self.sr();
        if sr & flags::BSY != 0 {
            return Err(Error::Busy);
        }
        if sr & flags::PESD != 0 {
            return Err(Error::Suspend);
        }

        self.clear_all_err();

        c1_c2!(
            self.flash.cr.modify(|_, w| w.pg().set_bit()),
            self.flash.c2cr.modify(|_, w| w.pg().set_bit())
        );

        unsafe {
            write_volatile(to as *mut u32, (from as *const u32).read());
            write_volatile(
                (to as *mut u32).offset(1),
                (from as *const u32).offset(1).read(),
            );
        }

        let ret: Result<(), Error> = self.wait_for_not_busy();

        c1_c2!(
            self.flash.cr.modify(|_, w| w.pg().clear_bit()),
            self.flash.c2cr.modify(|_, w| w.pg().clear_bit())
        );

        ret
    }

    /// Program any number of bytes.
    ///
    /// This is the safest possible method for programming.
    ///
    /// # Safety
    ///
    /// 1. Do not write to flash memory that is being used for your code.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     flash::{Flash, Page},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let my_data: [u8; 3] = [0x14, 0x15, 0x16];
    ///
    /// let last_page: Page = Page::from_index(127).unwrap();
    ///
    /// let mut flash: Flash = Flash::unlock(&mut dp.FLASH);
    /// unsafe {
    ///     flash.page_erase(last_page)?;
    ///     flash.program_bytes(&my_data, last_page.into())?;
    /// }
    /// # Ok::<(), stm32wlxx_hal::flash::Error>(())
    /// ```
    pub unsafe fn program_bytes(&mut self, from: &[u8], to: AlignedAddr) -> Result<(), Error> {
        if from.is_empty() {
            return Ok(());
        }

        if !flash_range().contains(&usize::from(to).saturating_add(from.len())) {
            return Err(Error::Overflow);
        }

        let chunks_exact: ChunksExact<u8> = from.chunks_exact(8);
        let remainder: &[u8] = chunks_exact.remainder();
        let remainder_len: usize = remainder.len();

        let last_u64: u64 = chunks_exact
            .remainder()
            .iter()
            .enumerate()
            .fold(0, |acc, (n, byte)| acc | u64::from(*byte) << (8 * n));

        for (n, chunk) in chunks_exact.enumerate() {
            let chunk_u64: u64 = u64::from_le_bytes(chunk.try_into().unwrap());
            let addr: usize = n * size_of::<u64>() + usize::from(to);

            self.standard_program(&chunk_u64, addr as *mut u64)?;
        }

        if remainder_len != 0 {
            let last_addr: usize = (usize::from(to) + from.len()).prev_multiple_of(&8);
            self.standard_program(&last_u64, last_addr as *mut u64)?;
        }

        Ok(())
    }

    /// Program a user-defined type.
    ///
    /// # Safety
    ///
    /// 1. Do not write to flash memory that is being used for your code.
    /// 2. The destination address must be within the flash memory region.
    /// 3. The `from` and `to` pointers must be aligned to `u64`.
    ///    Use `#[repr(align(8))]` to align your structure.
    #[allow(unused_unsafe)]
    pub unsafe fn standard_program_generic<T>(
        &mut self,
        from: *const T,
        to: *mut T,
    ) -> Result<(), Error> {
        let size: isize = size_of::<T>() as isize;
        if size == 0 {
            return Ok(());
        }

        if !flash_range().contains(&(size_of::<T>() + to as usize)) {
            return Err(Error::Overflow);
        }

        let sr: u32 = self.sr();
        if sr & flags::BSY != 0 {
            return Err(Error::Busy);
        }
        if sr & flags::PESD != 0 {
            return Err(Error::Suspend);
        }

        self.clear_all_err();

        c1_c2!(
            self.flash.cr.modify(|_, w| w.pg().set_bit()),
            self.flash.c2cr.modify(|_, w| w.pg().set_bit())
        );

        // Calculate the index of the last double word
        #[allow(unstable_name_collisions)]
        let last_double_word_idx: isize = size.div_ceil(&8) - 1;

        // Write the type as double words and return the number of bytes written
        let written_bytes: isize = (0..last_double_word_idx).fold(0, |acc, n| {
            unsafe {
                write_volatile(
                    (to as *mut u64).offset(n),
                    (from as *const u64).offset(n).read(),
                )
            };
            acc + 8
        });

        // Determine how many bytes are left to write
        let bytes_left: isize = size - written_bytes;

        // Append the left over bytes to a double word,
        // the last few bytes can look random in flash memory since Rust uses memory alignment to make accessing faster.
        let last_double_word: u64 = (0..bytes_left).fold(0, |dw, n| {
            let byte: u8 = (from as *const u8).offset(written_bytes + n).read();
            dw | u64::from(byte) << (8 * n)
        });

        // Write the last double word
        unsafe {
            write_volatile(
                (to as *mut u64).offset(last_double_word_idx),
                (&last_double_word as *const u64).read(),
            )
        };

        let ret: Result<(), Error> = self.wait_for_not_busy();

        c1_c2!(
            self.flash.cr.modify(|_, w| w.pg().clear_bit()),
            self.flash.c2cr.modify(|_, w| w.pg().clear_bit())
        );

        ret
    }

    /// Program 256 bytes.
    ///
    /// # Safety
    ///
    /// 1. Do not write to flash memory that is being used for your code.
    /// 2. The destination address must be within the flash memory region.
    /// 3. The flash clock frequency (HCLK3) must be at least 8 MHz.
    /// 4. The `from` and `to` pointers must be aligned to the pointee type.
    /// 5. The `from` pointer must point to 256 bytes of valid data.
    /// 6. The CPU must execute this from SRAM.
    ///    The compiler may inline this function, because `#[inline(never)]` is
    ///    merely a suggestion.
    #[allow(unused_unsafe)]
    #[cfg_attr(target_os = "none", link_section = ".data")]
    #[inline(never)]
    pub unsafe fn fast_program(&mut self, from: *const u64, to: *mut u64) -> Result<(), Error> {
        let sr: u32 = self.sr();
        if sr & flags::BSY != 0 {
            return Err(Error::Busy);
        }
        if sr & flags::PESD != 0 {
            return Err(Error::Suspend);
        }

        self.clear_all_err();

        c1_c2!(
            self.flash.cr.modify(|_, w| w.fstpg().set_bit()),
            self.flash.c2cr.modify(|_, w| w.fstpg().set_bit())
        );

        let from: *const u32 = from as *const u32;
        let to: *mut u32 = to as *mut u32;

        (0..64)
            .into_iter()
            .for_each(|word| unsafe { write_volatile(to.offset(word), from.offset(word).read()) });

        let ret: Result<(), Error> = self.wait_for_not_busy();

        c1_c2!(
            self.flash.cr.modify(|_, w| w.fstpg().clear_bit()),
            self.flash.c2cr.modify(|_, w| w.fstpg().clear_bit())
        );

        ret
    }

    /// Erases a 2048 byte page, setting all the bits to `1`.
    ///
    /// # Safety
    ///
    /// 1. Do not erase flash memory that is being used for your code.
    ///
    /// # Example
    ///
    /// Erase the last page.
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     flash::{Flash, Page},
    ///     pac,
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let last_page: Page = Page::from_index(127).unwrap();
    ///
    /// let mut flash: Flash = Flash::unlock(&mut dp.FLASH);
    /// unsafe { flash.page_erase(last_page)? };
    /// # Ok::<(), stm32wlxx_hal::flash::Error>(())
    /// ```
    pub unsafe fn page_erase(&mut self, page: Page) -> Result<(), Error> {
        let sr: u32 = self.sr();
        if sr & flags::BSY != 0 {
            return Err(Error::Busy);
        }
        if sr & flags::PESD != 0 {
            return Err(Error::Suspend);
        }

        self.clear_all_err();

        c1_c2!(
            self.flash.cr.modify(|_, w| w
                .per()
                .set_bit()
                .pnb()
                .bits(page.to_index())
                .strt()
                .set_bit()),
            self.flash.c2cr.modify(|_, w| w
                .per()
                .set_bit()
                .pnb()
                .bits(page.to_index())
                .strt()
                .set_bit())
        );

        let ret: Result<(), Error> = self.wait_for_not_busy();

        c1_c2!(
            self.flash.cr.modify(|_, w| w.per().clear_bit()),
            self.flash.c2cr.modify(|_, w| w.per().clear_bit())
        );

        ret
    }

    /// Erases the entire flash memory, setting all the bits to `1`.
    ///
    /// # Safety
    ///
    /// 1. The CPU must execute this from SRAM.
    ///    The compiler may inline this function, because `#[inline(never)]` is
    ///    merely a suggestion.
    #[cfg_attr(target_os = "none", link_section = ".data")]
    #[inline(never)]
    pub unsafe fn mass_erase(&mut self) -> Result<(), Error> {
        let sr: u32 = self.sr();
        if sr & flags::BSY != 0 {
            return Err(Error::Busy);
        }
        if sr & flags::PESD != 0 {
            return Err(Error::Suspend);
        }

        self.clear_all_err();

        c1_c2!(
            self.flash
                .cr
                .modify(|_, w| w.mer().set_bit().strt().set_bit()),
            self.flash
                .c2cr
                .modify(|_, w| w.mer().set_bit().strt().set_bit())
        );

        let ret: Result<(), Error> = self.wait_for_not_busy();

        c1_c2!(
            self.flash.cr.modify(|_, w| w.mer().clear_bit()),
            self.flash.c2cr.modify(|_, w| w.mer().clear_bit())
        );

        ret
    }
}
