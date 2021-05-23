/// Preamble detection length for [`GenericPacketParams`].
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PreambleDetection {
    /// Preamble detection disabled.
    Disabled = 0x0,
    /// 8-bit preamble detection.
    Bit8 = 0x4,
    /// 16-bit preamble detection.
    Bit16 = 0x5,
    /// 24-bit preamble detection.
    Bit24 = 0x6,
    /// 32-bit preamble detection.
    Bit32 = 0x7,
}

/// Address comparison/filtering for [`GenericPacketParams`].
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AddrComp {
    /// Address comparison/filtering disabled.
    Disabled = 0x0,
    /// Address comparison/filtering on node address.
    Node = 0x1,
    /// Address comparison/filtering on node and broadcast addresses.
    Broadcast = 0x2,
}

/// Payload type for [`GenericPacketParams`].
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PayloadType {
    /// Fixed; payload length and header field not added to packet.
    Fixed = 0b0,
    /// Variable; payload length and header field added to packet.
    Variable = 0b1,
}

/// CRC type definition for [`GenericPacketParams`].
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CrcType {
    /// 1-byte CRC.
    Byte1 = 0x0,
    /// CRC disabled.
    Disabled = 0x1,
    /// 2-byte CRC.
    Byte2 = 0x2,
    /// 1-byte inverted CRC.
    Byte1Inverted = 0x4,
    /// 2-byte inverted CRC.
    Byte2Inverted = 0x6,
}

/// Packet parameters for [`set_packet_params`].
///
/// [`set_packet_params`]: crate::SubGhz::set_packet_params
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GenericPacketParams {
    buf: [u8; 10],
}

impl GenericPacketParams {
    /// Create a new `GenericPacketParams`.
    ///
    /// This is the same as `default`, but in a `const` function.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::GenericPacketParams;
    ///
    /// const PKT_PARAMS: GenericPacketParams = GenericPacketParams::new();
    /// ```
    pub const fn new() -> GenericPacketParams {
        // const variable ensure the compile always optimizes the methods
        const NEW: GenericPacketParams = GenericPacketParams {
            #[rustfmt::skip]
            buf: [
                crate::OpCode::SetPacketParams as u8,
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ],
        }
        .set_preamble_len(1)
        .set_preamble_detection(PreambleDetection::Disabled)
        .set_sync_word_len(0)
        .set_addr_comp(AddrComp::Disabled)
        .set_payload_type(PayloadType::Fixed)
        .set_payload_len(1);

        NEW
    }

    /// Preamble length in number of symbols.
    ///
    /// Values of zero are invalid, and will automatically be set to 1.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::GenericPacketParams;
    ///
    /// const PKT_PARAMS: GenericPacketParams = GenericPacketParams::new().set_preamble_len(0x1234);
    /// # assert_eq!(PKT_PARAMS.as_slice()[1], 0x12);
    /// # assert_eq!(PKT_PARAMS.as_slice()[2], 0x34);
    /// ```
    #[must_use = "preamble_length returns a modified GenericPacketParams"]
    pub const fn set_preamble_len(mut self, mut len: u16) -> GenericPacketParams {
        if len == 0 {
            len = 1
        }
        self.buf[1] = ((len >> 8) & 0xFF) as u8;
        self.buf[2] = (len & 0xFF) as u8;
        self
    }

    /// Preabmle detection length in number of bit symbols.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{GenericPacketParams, PreambleDetection};
    ///
    /// const PKT_PARAMS: GenericPacketParams =
    ///     GenericPacketParams::new().set_preamble_detection(PreambleDetection::Bit8);
    /// # assert_eq!(PKT_PARAMS.as_slice()[3], 0x4);
    /// ```
    #[must_use = "set_preamble_detection returns a modified GenericPacketParams"]
    pub const fn set_preamble_detection(
        mut self,
        pb_det: PreambleDetection,
    ) -> GenericPacketParams {
        self.buf[3] = pb_det as u8;
        self
    }

    /// Sync word length in number of bit symbols.
    ///
    /// Valid values are `0x00` - `0x40` for 0 to 64-bits respectively.
    /// Values that exceed the maximum will saturate at `0x40`.
    ///
    /// # Example
    ///
    /// Set the sync word length to 4 bytes (16 bits).
    ///
    /// ```
    /// use stm32wl_hal_subghz::GenericPacketParams;
    ///
    /// const PKT_PARAMS: GenericPacketParams = GenericPacketParams::new().set_sync_word_len(16);
    /// # assert_eq!(PKT_PARAMS.as_slice()[4], 0x10);
    /// ```
    #[must_use = "set_sync_word_len returns a modified GenericPacketParams"]
    pub const fn set_sync_word_len(mut self, len: u8) -> GenericPacketParams {
        const MAX: u8 = 0x40;
        if len > MAX {
            self.buf[4] = MAX;
        } else {
            self.buf[4] = len;
        }
        self
    }

    /// Address comparison/filtering.
    ///
    /// # Example
    ///
    /// Enable address on the node address.
    ///
    /// ```
    /// use stm32wl_hal_subghz::{AddrComp, GenericPacketParams};
    ///
    /// const PKT_PARAMS: GenericPacketParams =
    ///     GenericPacketParams::new().set_addr_comp(AddrComp::Node);
    /// # assert_eq!(PKT_PARAMS.as_slice()[5], 0x01);
    /// ```
    #[must_use = "set_addr_comp returns a modified GenericPacketParams"]
    pub const fn set_addr_comp(mut self, addr_comp: AddrComp) -> GenericPacketParams {
        self.buf[5] = addr_comp as u8;
        self
    }

    /// Payload type definition.
    ///
    /// **Note:** The reference manual calls this packet type, but that results
    /// in a conflicting variable name for the modulation scheme, which the
    /// reference manual also calls packet type.
    ///
    /// # Example
    ///
    /// Set the payload type to a variable length.
    ///
    /// ```
    /// use stm32wl_hal_subghz::{GenericPacketParams, PayloadType};
    ///
    /// const PKT_PARAMS: GenericPacketParams =
    ///     GenericPacketParams::new().set_payload_type(PayloadType::Variable);
    /// # assert_eq!(PKT_PARAMS.as_slice()[6], 0x01);
    /// ```
    #[must_use = "set_payload_type returns a modified GenericPacketParams"]
    pub const fn set_payload_type(mut self, payload_type: PayloadType) -> GenericPacketParams {
        self.buf[6] = payload_type as u8;
        self
    }

    /// Set the payload length in bytes.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::GenericPacketParams;
    ///
    /// const PKT_PARAMS: GenericPacketParams = GenericPacketParams::new().set_payload_len(12);
    /// # assert_eq!(PKT_PARAMS.as_slice()[7], 12);
    /// ```
    #[must_use = "set_payload_len returns a modified GenericPacketParams"]
    pub const fn set_payload_len(mut self, len: u8) -> GenericPacketParams {
        self.buf[7] = len;
        self
    }

    /// CRC type definition.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{CrcType, GenericPacketParams};
    ///
    /// const PKT_PARAMS: GenericPacketParams =
    ///     GenericPacketParams::new().set_crc_type(CrcType::Byte2Inverted);
    /// # assert_eq!(PKT_PARAMS.as_slice()[8], 0x6);
    /// ```
    #[must_use = "set_payload_len returns a modified GenericPacketParams"]
    pub const fn set_crc_type(mut self, crc_type: CrcType) -> GenericPacketParams {
        self.buf[8] = crc_type as u8;
        self
    }

    /// Whitening enable.
    ///
    /// # Example
    ///
    /// Enable whitening.
    ///
    /// ```
    /// use stm32wl_hal_subghz::GenericPacketParams;
    ///
    /// const PKT_PARAMS: GenericPacketParams = GenericPacketParams::new().set_whitening_enable(true);
    /// # assert_eq!(PKT_PARAMS.as_slice()[9], 1);
    /// ```
    #[must_use = "set_whitening_enable returns a modified GenericPacketParams"]
    pub const fn set_whitening_enable(mut self, en: bool) -> GenericPacketParams {
        self.buf[9] = en as u8;
        self
    }

    /// Extracts a slice containing the packet.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{
    ///     AddrComp, CrcType, GenericPacketParams, PayloadType, PreambleDetection,
    /// };
    ///
    /// const PKT_PARAMS: GenericPacketParams = GenericPacketParams::new()
    ///     .set_preamble_len(8)
    ///     .set_preamble_detection(PreambleDetection::Disabled)
    ///     .set_sync_word_len(2)
    ///     .set_addr_comp(AddrComp::Disabled)
    ///     .set_payload_type(PayloadType::Fixed)
    ///     .set_payload_len(128)
    ///     .set_crc_type(CrcType::Byte2)
    ///     .set_whitening_enable(true);
    ///
    /// assert_eq!(
    ///     PKT_PARAMS.as_slice(),
    ///     &[0x8C, 0x00, 0x08, 0x00, 0x02, 0x00, 0x00, 0x80, 0x02, 0x01]
    /// );
    /// ```
    pub const fn as_slice(&self) -> &[u8] {
        &self.buf
    }
}

impl Default for GenericPacketParams {
    fn default() -> Self {
        Self::new()
    }
}
