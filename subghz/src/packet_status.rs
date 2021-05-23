use num_rational::Ratio;

use crate::Status;

/// (G)FSK packet status.
///
/// Returned by [`gfsk_packet_status`].
///
/// [`gfsk_packet_status`]: crate::SubGhz::gfsk_packet_status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GfskPacketStatus {
    buf: [u8; 4],
}

impl From<[u8; 4]> for GfskPacketStatus {
    fn from(buf: [u8; 4]) -> Self {
        GfskPacketStatus { buf }
    }
}

impl GfskPacketStatus {
    /// Get the status.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{
    ///     num_rational::Ratio, CmdStatus, GfskPacketStatus, Status, StatusMode,
    /// };
    ///
    /// let example_data_from_radio: [u8; 4] = [0x54, 0, 0, 0];
    /// let pkt_status: GfskPacketStatus = GfskPacketStatus::from(example_data_from_radio);
    /// let status: Status = pkt_status.status();
    /// assert_eq!(status.mode(), Ok(StatusMode::Rx));
    /// assert_eq!(status.cmd(), Ok(CmdStatus::Avaliable));
    /// ```
    pub const fn status(&self) -> Status {
        Status::from_raw(self.buf[0])
    }

    /// Returns `true` if a preabmle error occured.
    pub const fn preamble_error(&self) -> bool {
        (self.buf[1] & (1 << 7)) != 0
    }

    /// Returns `true` if a synchronization error occured.
    pub const fn sync_err(&self) -> bool {
        (self.buf[1] & (1 << 6)) != 0
    }

    /// Returns `true` if an address error occured.
    pub const fn adrs_err(&self) -> bool {
        (self.buf[1] & (1 << 5)) != 0
    }

    /// Returns `true` if an crc error occured.
    pub const fn crc_err(&self) -> bool {
        (self.buf[1] & (1 << 4)) != 0
    }

    /// Returns `true` if a length error occured.
    pub const fn length_err(&self) -> bool {
        (self.buf[1] & (1 << 3)) != 0
    }

    /// Returns `true` if an abort error occured.
    pub const fn abort_err(&self) -> bool {
        (self.buf[1] & (1 << 2)) != 0
    }

    /// Returns `true` if a packet is received.
    pub const fn pkt_received(&self) -> bool {
        (self.buf[1] & (1 << 1)) != 0
    }

    /// Returns `true` when a packet has been sent.
    pub const fn pkt_sent(&self) -> bool {
        (self.buf[1] & 1) != 0
    }

    /// RSSI level when the synchronization address is detected.
    ///
    /// Units are in dBm.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{num_rational::Ratio, GfskPacketStatus};
    ///
    /// let example_data_from_radio: [u8; 4] = [0, 0, 80, 0];
    /// let pkt_status: GfskPacketStatus = GfskPacketStatus::from(example_data_from_radio);
    /// assert_eq!(pkt_status.rssi_sync().to_integer(), -40);
    /// ```
    pub const fn rssi_sync(&self) -> Ratio<i16> {
        Ratio::new_raw(self.buf[2] as i16, -2)
    }

    /// Return the RSSI level over the received packet.
    ///
    /// Units are in dBm.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{num_rational::Ratio, GfskPacketStatus};
    ///
    /// let example_data_from_radio: [u8; 4] = [0, 0, 0, 100];
    /// let pkt_status: GfskPacketStatus = GfskPacketStatus::from(example_data_from_radio);
    /// assert_eq!(pkt_status.rssi_avg().to_integer(), -50);
    /// ```
    pub const fn rssi_avg(&self) -> Ratio<i16> {
        Ratio::new_raw(self.buf[3] as i16, -2)
    }
}
