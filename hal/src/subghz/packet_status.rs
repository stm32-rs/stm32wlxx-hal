use num_rational::Ratio;

use crate::subghz::Status;

/// (G)FSK packet status.
///
/// Returned by [`fsk_packet_status`].
///
/// [`fsk_packet_status`]: crate::subghz::SubGhz::fsk_packet_status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FskPacketStatus {
    buf: [u8; 4],
}

impl From<[u8; 4]> for FskPacketStatus {
    fn from(buf: [u8; 4]) -> Self {
        FskPacketStatus { buf }
    }
}

impl FskPacketStatus {
    /// Get the status.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::subghz::{CmdStatus, FskPacketStatus, Status, StatusMode};
    ///
    /// let example_data_from_radio: [u8; 4] = [0x54, 0, 0, 0];
    /// let pkt_status: FskPacketStatus = FskPacketStatus::from(example_data_from_radio);
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
    /// use stm32wl_hal::subghz::{num_rational::Ratio, FskPacketStatus};
    ///
    /// let example_data_from_radio: [u8; 4] = [0, 0, 80, 0];
    /// let pkt_status: FskPacketStatus = FskPacketStatus::from(example_data_from_radio);
    /// assert_eq!(pkt_status.rssi_sync().to_integer(), -40);
    /// ```
    pub fn rssi_sync(&self) -> Ratio<i16> {
        Ratio::new(i16::from(self.buf[2]), -2)
    }

    /// Return the RSSI level over the received packet.
    ///
    /// Units are in dBm.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::subghz::{num_rational::Ratio, FskPacketStatus};
    ///
    /// let example_data_from_radio: [u8; 4] = [0, 0, 0, 100];
    /// let pkt_status: FskPacketStatus = FskPacketStatus::from(example_data_from_radio);
    /// assert_eq!(pkt_status.rssi_avg().to_integer(), -50);
    /// ```
    pub fn rssi_avg(&self) -> Ratio<i16> {
        Ratio::new(i16::from(self.buf[3]), -2)
    }
}

/// (G)FSK packet status.
///
/// Returned by [`lora_packet_status`].
///
/// [`lora_packet_status`]: crate::subghz::SubGhz::lora_packet_status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoRaPacketStatus {
    buf: [u8; 4],
}

impl From<[u8; 4]> for LoRaPacketStatus {
    fn from(buf: [u8; 4]) -> Self {
        LoRaPacketStatus { buf }
    }
}

impl LoRaPacketStatus {
    /// Get the status.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::subghz::{CmdStatus, LoRaPacketStatus, Status, StatusMode};
    ///
    /// let example_data_from_radio: [u8; 4] = [0x54, 0, 0, 0];
    /// let pkt_status: LoRaPacketStatus = LoRaPacketStatus::from(example_data_from_radio);
    /// let status: Status = pkt_status.status();
    /// assert_eq!(status.mode(), Ok(StatusMode::Rx));
    /// assert_eq!(status.cmd(), Ok(CmdStatus::Avaliable));
    /// ```
    pub const fn status(&self) -> Status {
        Status::from_raw(self.buf[0])
    }

    /// Average RSSI level over the received packet.
    ///
    /// Units are in dBm.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::subghz::{num_rational::Ratio, LoRaPacketStatus};
    ///
    /// let example_data_from_radio: [u8; 4] = [0, 80, 0, 0];
    /// let pkt_status: LoRaPacketStatus = LoRaPacketStatus::from(example_data_from_radio);
    /// assert_eq!(pkt_status.rssi_pkt().to_integer(), -40);
    /// ```
    pub fn rssi_pkt(&self) -> Ratio<i16> {
        Ratio::new(i16::from(self.buf[1]), -2)
    }

    /// Estimation of SNR over the received packet.
    ///
    /// Units are in dB.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::subghz::{num_rational::Ratio, LoRaPacketStatus};
    ///
    /// let example_data_from_radio: [u8; 4] = [0, 0, 40, 0];
    /// let pkt_status: LoRaPacketStatus = LoRaPacketStatus::from(example_data_from_radio);
    /// assert_eq!(pkt_status.snr_pkt().to_integer(), 10);
    /// ```
    pub fn snr_pkt(&self) -> Ratio<i16> {
        Ratio::new(i16::from(self.buf[2]), 4)
    }

    /// Estimation of RSSI level of the LoRa signal after despreading.
    ///
    /// Units are in dBm.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::subghz::{num_rational::Ratio, LoRaPacketStatus};
    ///
    /// let example_data_from_radio: [u8; 4] = [0, 0, 0, 80];
    /// let pkt_status: LoRaPacketStatus = LoRaPacketStatus::from(example_data_from_radio);
    /// assert_eq!(pkt_status.signal_rssi_pkt().to_integer(), -40);
    /// ```
    pub fn signal_rssi_pkt(&self) -> Ratio<i16> {
        Ratio::new(i16::from(self.buf[3]), -2)
    }
}
