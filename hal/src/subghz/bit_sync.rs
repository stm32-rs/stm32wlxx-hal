/// Bit synchronization.
///
/// This must be cleared to `0x00` (the reset value) when using packet types
/// other than LoRa.
///
/// Argument of [`set_bit_sync`](crate::subghz::SubGhz::set_bit_sync).
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BitSync {
    val: u8,
}

impl BitSync {
    /// Bit synchronization register reset value.
    pub const RESET: BitSync = BitSync { val: 0x00 };

    /// Create a new [`BitSync`] structure from a raw value.
    ///
    /// Reserved bits will be masked.
    pub const fn from_raw(raw: u8) -> Self {
        Self { val: raw & 0x70 }
    }

    /// Get the raw value of the [`BitSync`] register.
    pub const fn as_raw(&self) -> u8 {
        self.val
    }

    /// LoRa simple bit synchronization enable.
    ///
    /// # Example
    ///
    /// Enable simple bit synchronization.
    ///
    /// ```
    /// use stm32wl_hal::subghz::BitSync;
    ///
    /// const BIT_SYNC: BitSync = BitSync::RESET.set_simple_bit_sync_en(true);
    /// # assert_eq!(u8::from(BIT_SYNC), 0x40u8);
    /// ```
    #[must_use = "set_simple_bit_sync_en returns a new BitSync"]
    pub const fn set_simple_bit_sync_en(mut self, en: bool) -> BitSync {
        self.val |= (en as u8) << 6;
        self
    }

    /// LoRa receive data inversion.
    ///
    /// # Example
    ///
    /// Invert receive data.
    ///
    /// ```
    /// use stm32wl_hal::subghz::BitSync;
    ///
    /// const BIT_SYNC: BitSync = BitSync::RESET.set_rx_data_inv(true);
    /// # assert_eq!(u8::from(BIT_SYNC), 0x20u8);
    /// ```
    #[must_use = "set_rx_data_inv returns a new BitSync"]
    pub const fn set_rx_data_inv(mut self, inv: bool) -> BitSync {
        self.val |= (inv as u8) << 5;
        self
    }

    /// LoRa normal bit synchronization enable.
    ///
    /// # Example
    ///
    /// Enable normal bit synchronization.
    ///
    /// ```
    /// use stm32wl_hal::subghz::BitSync;
    ///
    /// const BIT_SYNC: BitSync = BitSync::RESET.set_norm_bit_sync_en(true);
    /// # assert_eq!(u8::from(BIT_SYNC), 0x10u8);
    /// ```
    #[must_use = "set_norm_bit_sync_en returns a new BitSync"]
    pub const fn set_norm_bit_sync_en(mut self, en: bool) -> BitSync {
        self.val |= (en as u8) << 4;
        self
    }
}

impl From<BitSync> for u8 {
    fn from(bs: BitSync) -> Self {
        bs.val
    }
}

impl Default for BitSync {
    fn default() -> Self {
        Self::RESET
    }
}
