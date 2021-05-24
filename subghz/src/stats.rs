use crate::Status;

/// [Typestate] for LoRa stats.
///
/// [Typestate]: https://docs.rust-embedded.org/book/static-guarantees/typestate-programming.html
pub struct LoRaStats;
/// [Typestate] for FSK stats.
///
/// [Typestate]: https://docs.rust-embedded.org/book/static-guarantees/typestate-programming.html
pub struct FskStats;

/// Packet statistics.
///
/// Returned by [`fsk_stats`] and [`lora_stats`].
///
/// [`fsk_stats`]: crate::SubGhz::fsk_stats
/// [`lora_stats`]: crate::SubGhz::lora_stats
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub struct Stats<ModType> {
    status: Status,
    pkt_rx: u16,
    pkt_crc: u16,
    pkt_len_or_hdr_err: u16,
    ty: ModType,
}

impl<ModType> Stats<ModType> {
    const fn from_buf(buf: [u8; 7], ty: ModType) -> Stats<ModType> {
        Stats {
            status: Status::from_raw(buf[0]),
            pkt_rx: u16::from_be_bytes([buf[1], buf[2]]),
            pkt_crc: u16::from_be_bytes([buf[3], buf[4]]),
            pkt_len_or_hdr_err: u16::from_be_bytes([buf[5], buf[6]]),
            ty,
        }
    }

    /// Get the radio status returned with the packet statistics.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{CmdStatus, FskStats, Stats, StatusMode};
    ///
    /// let example_data_from_radio: [u8; 7] = [0x54, 0, 0, 0, 0, 0, 0];
    /// let stats: Stats<FskStats> = Stats::from_raw_fsk(example_data_from_radio);
    /// assert_eq!(stats.status().mode(), Ok(StatusMode::Rx));
    /// assert_eq!(stats.status().cmd(), Ok(CmdStatus::Avaliable));
    /// ```
    pub const fn status(&self) -> Status {
        self.status
    }

    /// Number of packets received.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{FskStats, Stats};
    ///
    /// let example_data_from_radio: [u8; 7] = [0x54, 0, 3, 0, 0, 0, 0];
    /// let stats: Stats<FskStats> = Stats::from_raw_fsk(example_data_from_radio);
    /// assert_eq!(stats.pkt_rx(), 3);
    /// ```
    pub const fn pkt_rx(&self) -> u16 {
        self.pkt_rx
    }

    /// Number of packets received with a payload CRC error
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{LoRaStats, Stats};
    ///
    /// let example_data_from_radio: [u8; 7] = [0x54, 0, 0, 0, 1, 0, 0];
    /// let stats: Stats<LoRaStats> = Stats::from_raw_lora(example_data_from_radio);
    /// assert_eq!(stats.pkt_crc(), 1);
    /// ```
    pub const fn pkt_crc(&self) -> u16 {
        self.pkt_crc
    }
}

impl Stats<FskStats> {
    /// Create a new FSK packet statistics structure from a raw buffer.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{FskStats, Stats};
    ///
    /// let example_data_from_radio: [u8; 7] = [0x54, 0, 0, 0, 0, 0, 0];
    /// let stats: Stats<FskStats> = Stats::from_raw_fsk(example_data_from_radio);
    /// ```
    pub const fn from_raw_fsk(buf: [u8; 7]) -> Stats<FskStats> {
        Self::from_buf(buf, FskStats)
    }

    /// Number of packets received with a payload length error.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{FskStats, Stats};
    ///
    /// let example_data_from_radio: [u8; 7] = [0x54, 0, 0, 0, 0, 0, 1];
    /// let stats: Stats<FskStats> = Stats::from_raw_fsk(example_data_from_radio);
    /// assert_eq!(stats.pkt_len_err(), 1);
    /// ```
    pub const fn pkt_len_err(&self) -> u16 {
        self.pkt_len_or_hdr_err
    }
}

impl Stats<LoRaStats> {
    /// Create a new LoRa packet statistics structure from a raw buffer.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{LoRaStats, Stats};
    ///
    /// let example_data_from_radio: [u8; 7] = [0x54, 0, 0, 0, 0, 0, 0];
    /// let stats: Stats<LoRaStats> = Stats::from_raw_lora(example_data_from_radio);
    /// ```
    pub const fn from_raw_lora(buf: [u8; 7]) -> Stats<LoRaStats> {
        Self::from_buf(buf, LoRaStats)
    }

    /// Number of packets received with a header CRC error.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{LoRaStats, Stats};
    ///
    /// let example_data_from_radio: [u8; 7] = [0x54, 0, 0, 0, 0, 0, 1];
    /// let stats: Stats<LoRaStats> = Stats::from_raw_lora(example_data_from_radio);
    /// assert_eq!(stats.pkt_hdr_err(), 1);
    /// ```
    pub const fn pkt_hdr_err(&self) -> u16 {
        self.pkt_len_or_hdr_err
    }
}
