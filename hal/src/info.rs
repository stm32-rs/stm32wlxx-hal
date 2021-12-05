//! Device electronic signature

use core::{fmt::Display, mem::transmute, ptr::read};

/// CPU core.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Core {
    /// Cortex-M4, core 1.
    Cm4,
    /// Cortex-M0+, core 2.
    Cm0p,
}

impl Core {
    /// Get the CPU core at compile time.
    ///
    /// This is determined by the HAL features.
    ///
    /// For a runtime mechanism use [`Core::from_cpuid()`].
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::info::Core;
    ///
    /// #[cfg(feature = "stm32wl5x_cm4")]
    /// assert_eq!(Core::CT, Core::Cm4);
    ///
    /// #[cfg(feature = "stm32wl5x_cm0p")]
    /// assert_eq!(Core::CT, Core::Cm0p);
    ///
    /// #[cfg(feature = "stm32wle5")]
    /// assert_eq!(Core::CT, Core::Cm4);
    /// ```
    pub const CT: Core = c1_c2!(Core::Cm4, Core::Cm0p);

    /// Get the CPU core at runtime.
    ///
    /// This is determined by the part number field in CPUID register in the
    /// system control block.
    ///
    /// For a compile time mechanism use [`Core::CT`].
    ///
    /// # Example
    ///
    /// ```no_run
    /// # #[cfg(features = "defmt")] {
    /// use stm32wlxx_hal::info::Core;
    ///
    /// match Core::from_device() {
    ///     Core::Cm4 => defmt::info!("Hello world from the Cortex-M4 CPU"),
    ///     Core::Cm0p => defmt::info!("Hello world from the Cortex-M0+ CPU"),
    /// }
    /// # }
    /// ```
    pub fn from_cpuid() -> Core {
        const CPUID: *const u32 = 0xE000ED00 as *const u32;
        let cpuid: u32 = unsafe { CPUID.read_volatile() };

        if cpuid & 0x0000_FFF0 == 0x0000_C240 {
            Core::Cm4
        } else {
            Core::Cm0p
        }
    }

    /// Returns `true` if the core is [`Cm4`].
    ///
    /// [`Cm4`]: Core::Cm4
    pub const fn is_cm4(&self) -> bool {
        matches!(self, Self::Cm4)
    }

    /// Returns `true` if the core is [`Cm0p`].
    ///
    /// [`Cm0p`]: Core::Cm0p
    pub const fn is_cm0p(&self) -> bool {
        matches!(self, Self::Cm0p)
    }
}

impl core::fmt::Display for Core {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Core::Cm4 => write!(f, "Cortex-M4"),
            Core::Cm0p => write!(f, "Cortex-M0+"),
        }
    }
}

/// 96-bit unique device identifier
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
    /// Pointer to the 96-bit unique device identifier device memory.
    pub const PTR: *const u32 = 0x1FFF_7590 as *const u32;

    /// Get the 96-bit unique device identifier
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::info::Uid;
    ///
    /// let uid: Uid = Uid::from_device();
    /// ```
    #[inline]
    pub fn from_device() -> Uid {
        unsafe {
            [
                Self::PTR.read(),
                Self::PTR.offset(1).read(),
                Self::PTR.offset(2).read(),
            ]
        }
        .into()
    }

    /// X-Y coordinates on the wafer
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::info::Uid;
    ///
    /// let coord: u32 = Uid::from_device().coord();
    /// ```
    pub const fn coord(&self) -> u32 {
        self.uid[0]
    }

    /// Wafer number
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::info::Uid;
    ///
    /// let wafer: u8 = Uid::from_device().wafer();
    /// ```
    pub const fn wafer(&self) -> u8 {
        self.uid[1] as u8
    }

    /// Lot number
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::info::Uid;
    ///
    /// let lot: [u8; 7] = Uid::from_device().lot();
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

/// Flash size in kibibytes
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::info::flash_size_kibibyte;
///
/// // valid for the NUCLEO-WL55JC2 dev board
/// assert_eq!(flash_size_kibibyte(), 256);
/// ```
#[inline]
pub fn flash_size_kibibyte() -> u16 {
    unsafe { read(0x1FFF_75E0 as *const u16) }
}

/// Flash size in bytes
///
/// # Example
///
/// ```no_run
/// use stm32wlxx_hal::info::flash_size;
///
/// // valid for the NUCLEO-WL55JC2 dev board
/// assert_eq!(flash_size(), 256 * 1024);
/// ```
#[inline]
pub fn flash_size() -> u32 {
    u32::from(flash_size_kibibyte()) << 10
}

/// Physical package type.
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

impl Package {
    /// Get the package type from device memory.
    ///
    /// If the value is reserved it will be returned in the `Err` variant of the
    /// `Result`.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::info;
    ///
    /// let package: Result<info::Package, u8> = info::Package::from_device();
    /// // valid for the NUCLEO-WL55JC2 dev board
    /// assert_eq!(package, Ok(info::Package::UFBGA73));
    /// ```
    #[inline]
    pub fn from_device() -> Result<Self, u8> {
        let raw: u8 = (unsafe { read(0x1FFF_7500 as *const u32) } & 0xF) as u8;
        raw.try_into()
    }
}

/// IEEE 64-bit unique device ID (UID64).
///
/// **Note:** There are two UIDs, the other is [`Uid`].
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Uid64 {
    uid: u64,
}

impl Uid64 {
    /// Pointer to the IEEE 64-bit unique device ID (UID64) device memory.
    pub const PTR: *const u32 = 0x1FFF_7580 as *const u32;

    /// Get the IEEE 64-bit unique device ID (UID64) from device memory.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::info;
    ///
    /// let uid64: info::Uid64 = info::Uid64::from_device();
    /// assert_eq!(uid64.dev_id(), 0x15);
    /// assert_eq!(uid64.company_id(), 0x0080E1);
    /// // uid64.devnum() is unique
    /// ```
    #[inline]
    pub fn from_device() -> Self {
        let hi: u64 = unsafe { Self::PTR.read() }.into();
        let lo: u64 = unsafe { Self::PTR.offset(1).read() }.into();
        ((hi << 32) | lo).into()
    }

    /// Unique 32-bit device number.
    ///
    /// This is sequential and unique for each individual device.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::info::Uid64;
    ///
    /// let devnum: u32 = Uid64::from_device().devnum();
    /// ```
    pub const fn devnum(&self) -> u32 {
        (self.uid >> 32) as u32
    }

    /// Read the 32-bit device number from device memory.
    ///
    /// This is provided for applications that only need the 32-bit device
    /// number, performing a single device memory read.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::info::Uid64;
    ///
    /// let devnum: u32 = Uid64::read_devnum();
    /// assert_eq!(devnum, Uid64::from_device().devnum());
    /// ```
    #[inline]
    pub fn read_devnum() -> u32 {
        unsafe { Uid64::PTR.read() }
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
    /// use stm32wlxx_hal::info::Uid64;
    ///
    /// assert_eq!(Uid64::from_device().company_id(), 0x0080E1);
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
    /// use stm32wlxx_hal::info::Uid64;
    ///
    /// assert_eq!(Uid64::from_device().dev_id(), 0x15);
    /// ```
    pub const fn dev_id(&self) -> u8 {
        (self.uid & 0xFF) as u8
    }
}

impl From<u64> for Uid64 {
    #[inline]
    fn from(uid: u64) -> Self {
        Uid64 { uid }
    }
}

impl From<Uid64> for u64 {
    #[inline]
    fn from(uid: Uid64) -> Self {
        uid.uid
    }
}

impl Display for Uid64 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Uid64")
            .field("devnum", &self.devnum())
            .field("company_id", &self.company_id())
            .field("dev_id", &self.dev_id())
            .finish()
    }
}
