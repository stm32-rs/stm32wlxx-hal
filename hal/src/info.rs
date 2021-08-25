//! Device eletronic signature

use core::{
    convert::{TryFrom, TryInto},
    fmt::Display,
    mem::transmute,
    ptr::read,
};

/// 96-bit unique device identifier
///
/// Returned by [`uid`].
///
/// **Note:** There are two UIDs, the other is [`Uid64`].
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Uid {
    uid: [u32; 3],
}

impl From<[u32; 3]> for Uid {
    fn from(uid: [u32; 3]) -> Self {
        Uid { uid }
    }
}

impl From<Uid> for [u32; 3] {
    fn from(uid: Uid) -> Self {
        uid.uid
    }
}

impl From<Uid> for [u8; 12] {
    fn from(uid: Uid) -> Self {
        unsafe { transmute::<[u32; 3], [u8; 12]>(uid.uid) }
    }
}

impl Display for Uid {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Uid")
            .field("coord", &self.coord())
            .field("wafer", &self.wafer())
            .field("lot", &self.lot())
            .finish()
    }
}

impl Uid {
    /// X-Y coordinates on the wafer
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::info::uid;
    ///
    /// let coord: u32 = uid().coord();
    /// ```
    pub const fn coord(&self) -> u32 {
        self.uid[0]
    }

    /// Wafer number
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::info::uid;
    ///
    /// let wafer: u8 = uid().wafer();
    /// ```
    pub const fn wafer(&self) -> u8 {
        self.uid[1] as u8
    }

    /// Lot number
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::info::uid;
    ///
    /// let lot: [u8; 7] = uid().lot();
    /// ```
    pub const fn lot(&self) -> [u8; 7] {
        [
            (self.uid[1] >> 8) as u8,
            (self.uid[1] >> 16) as u8,
            (self.uid[1] >> 24) as u8,
            self.uid[2] as u8,
            (self.uid[2] >> 8) as u8,
            (self.uid[2] >> 16) as u8,
            (self.uid[2] >> 24) as u8,
        ]
    }
}

/// Get the 96-bit unique device identifier
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::info::{uid, Uid};
///
/// let uid: Uid = uid();
/// ```
pub fn uid() -> Uid {
    unsafe {
        [
            read(0x1FFF_7590 as *const u32),
            read(0x1FFF_7594 as *const u32),
            read(0x1FFF_7598 as *const u32),
        ]
    }
    .into()
}

/// Flash size in kibibytes
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::info::flash_size_kibibyte;
///
/// // valid for the NUCLEO-WL55JC2 dev board
/// assert_eq!(flash_size_kibibyte(), 256);
/// ```
pub fn flash_size_kibibyte() -> u16 {
    unsafe { read(0x1FFF_75E0 as *const u16) }
}

/// Flash size in bytes
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::info::flash_size;
///
/// // valid for the NUCLEO-WL55JC2 dev board
/// assert_eq!(flash_size(), 256 * 1024);
/// ```
pub fn flash_size() -> u32 {
    u32::from(flash_size_kibibyte()) << 10
}

/// Physical package type
///
/// Returned by [`package`].
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Package {
    /// [UFBGA73](https://en.wikipedia.org/wiki/Ball_grid_array)
    UFBGA73 = 0b00000,
    /// [WLCSP59](https://en.wikipedia.org/wiki/Wafer-level_packaging)
    WLCSP59 = 0b00010,
    /// [UFQFPN48](https://en.wikipedia.org/wiki/Flat_no-leads_package)
    UFQFPN48 = 0b01010,
}

impl TryFrom<u8> for Package {
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b00000 => Ok(Package::UFBGA73),
            0b00010 => Ok(Package::WLCSP59),
            0b01010 => Ok(Package::UFQFPN48),
            _ => Err(value),
        }
    }
}

impl From<Package> for u8 {
    fn from(p: Package) -> Self {
        p as u8
    }
}

/// Get the package type
///
/// If the value is reserved it will be returned in the `Err` variant of the
/// `Result`.
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::info;
///
/// let package: Result<info::Package, u8> = info::package();
/// // valid for the NUCLEO-WL55JC2 dev board
/// assert_eq!(package, Ok(info::Package::UFBGA73));
/// ```
pub fn package() -> Result<Package, u8> {
    let raw: u16 = unsafe { read(0x1FFF_7500 as *const u16) } & 0xF;
    (raw as u8).try_into()
}

/// IEEE 64-bit unique device ID (UID64)
///
/// Returned by [`uid64`].
///
/// **Note:** There are two UIDs, the other is [`Uid`].
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Uid64 {
    uid: u64,
}

impl Uid64 {
    /// Unique 32-bit device number.
    ///
    /// This is sequential and unique for each individual device.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::info::uid64;
    ///
    /// let dev_num: u32 = uid64().dev_num();
    /// ```
    pub const fn dev_num(&self) -> u32 {
        (self.uid >> 32) as u32
    }

    /// Company ID
    ///
    /// This is `0x0080E1` for STMicroelectronics.
    ///
    /// **Note:** Only the first 24 bits are used.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::info::uid64;
    ///
    /// assert_eq!(uid64().company_id(), 0x0080E1);
    /// ```
    pub const fn company_id(&self) -> u32 {
        ((self.uid as u32) & 0xFFFF_FF00) >> 8
    }

    /// Device ID
    ///
    /// This is always `0x15` for this device.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::info::uid64;
    ///
    /// assert_eq!(uid64().dev_id(), 0x15);
    /// ```
    pub const fn dev_id(&self) -> u8 {
        (self.uid & 0xFF) as u8
    }
}

impl From<u64> for Uid64 {
    fn from(uid: u64) -> Self {
        Uid64 { uid }
    }
}

impl From<Uid64> for u64 {
    fn from(uid: Uid64) -> Self {
        uid.uid
    }
}

impl Display for Uid64 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Uid64")
            .field("dev_num", &self.dev_num())
            .field("company_id", &self.company_id())
            .field("dev_id", &self.dev_id())
            .finish()
    }
}

/// Pointer to the IEEE 64-bit unique device ID (UID64)
pub const UID64: *const u8 = 0x1FFF_7580 as *const u8;

/// Get the IEEE 64-bit unique device ID (UID64)
///
/// # Example
///
/// ```no_run
/// use stm32wl_hal::info;
///
/// let uid64: info::Uid64 = info::uid64();
/// assert_eq!(uid64.dev_id(), 0x15);
/// assert_eq!(uid64.company_id(), 0x0080E1);
/// // uid64.dev_num() is unique
/// ```
pub fn uid64() -> Uid64 {
    let hi: u32 = unsafe { read(0x1FFF_7580 as *const u32) };
    let lo: u32 = unsafe { read(0x1FFF_7584 as *const u32) };
    (((hi as u64) << 32) | (lo as u64)).into()
}
