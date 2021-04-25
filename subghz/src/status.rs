/// sub-GHz radio operating mode.
///
/// See `Get_Status` under section 5.8.5 "Communcation status information commands"
/// in the reference manual.
///
/// This is returned by [`Status::mode`].
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub enum StatusMode {
    /// Standby mode with RC 13MHz.
    StandbyRc = 0x2,
    /// Standby mode with HSE32.
    StandbyHse = 0x3,
    /// Frequency Synthesis mode.
    Fs = 0x4,
    /// Receive mode.
    Rx = 0x5,
    /// Transmit mode.
    Tx = 0x6,
}

impl StatusMode {
    /// Create a new `StatusMode` from bits.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::StatusMode;
    ///
    /// assert_eq!(StatusMode::from_bits(0x2), Ok(StatusMode::StandbyRc));
    /// assert_eq!(StatusMode::from_bits(0x3), Ok(StatusMode::StandbyHse));
    /// assert_eq!(StatusMode::from_bits(0x4), Ok(StatusMode::Fs));
    /// assert_eq!(StatusMode::from_bits(0x5), Ok(StatusMode::Rx));
    /// assert_eq!(StatusMode::from_bits(0x6), Ok(StatusMode::Tx));
    /// // Other values are reserved
    /// assert_eq!(StatusMode::from_bits(0), Err(0));
    /// ```
    pub const fn from_bits(bits: u8) -> Result<Self, u8> {
        match bits {
            0x2 => Ok(StatusMode::StandbyRc),
            0x3 => Ok(StatusMode::StandbyHse),
            0x4 => Ok(StatusMode::Fs),
            0x5 => Ok(StatusMode::Rx),
            0x6 => Ok(StatusMode::Tx),
            _ => Err(bits),
        }
    }
}

/// Command status.
///
/// See `Get_Status` under section 5.8.5 "Communcation status information commands"
/// in the reference manual.
///
/// This is returned by [`Status::cmd`].
#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub enum CmdStatus {
    /// data available to host (packet received successfully and data can be retrieved)
    Avaliable = 0x2,
    /// Command time out.
    ///
    /// Command took too long to complete triggering a sub-GHz radio watchdog
    /// timeout.
    Timeout = 0x3,
    /// Command processing error.
    ///
    /// Invalid opcode or incorrect number of parameters.
    ProcessingError = 0x4,
    /// Command execution failure.
    ///
    /// Command successfully received but cannot be executed at this time,
    /// requested operating mode cannot be entered or requested data cannot be
    /// sent.
    ExecutionFailure = 0x5,
    /// Transmit command completed.
    ///
    /// Current packet transmission completed.
    Complete = 0x6,
}

impl CmdStatus {
    /// Create a new `CmdStatus` from bits.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::CmdStatus;
    ///
    /// assert_eq!(CmdStatus::from_bits(0x2), Ok(CmdStatus::Avaliable));
    /// assert_eq!(CmdStatus::from_bits(0x3), Ok(CmdStatus::Timeout));
    /// assert_eq!(CmdStatus::from_bits(0x4), Ok(CmdStatus::ProcessingError));
    /// assert_eq!(CmdStatus::from_bits(0x5), Ok(CmdStatus::ExecutionFailure));
    /// assert_eq!(CmdStatus::from_bits(0x6), Ok(CmdStatus::Complete));
    /// // Other values are reserved
    /// assert_eq!(CmdStatus::from_bits(0), Err(0));
    /// ```
    pub const fn from_bits(bits: u8) -> Result<Self, u8> {
        match bits {
            0x2 => Ok(CmdStatus::Avaliable),
            0x3 => Ok(CmdStatus::Timeout),
            0x4 => Ok(CmdStatus::ProcessingError),
            0x5 => Ok(CmdStatus::ExecutionFailure),
            0x6 => Ok(CmdStatus::Complete),
            _ => Err(bits),
        }
    }
}

/// Radio status.
///
/// This is returned by [`status`].
///
/// [`status`]: crate::SubGhz::status
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Status(u8);

impl From<u8> for Status {
    fn from(x: u8) -> Self {
        Status(x)
    }
}
impl From<Status> for u8 {
    fn from(x: Status) -> Self {
        x.0
    }
}

impl Status {
    /// sub-GHz radio operating mode.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{Status, StatusMode};
    ///
    /// let status: Status = 0xACu8.into();
    /// assert_eq!(status.mode(), Ok(StatusMode::StandbyRc));
    /// ```
    pub const fn mode(&self) -> Result<StatusMode, u8> {
        StatusMode::from_bits((self.0 >> 4) & 0b111)
    }

    /// Command status.
    ///
    /// For some reason `Err(1)` is a pretty common return value for this,
    /// despite being a reserved value.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::{CmdStatus, Status};
    ///
    /// let status: Status = 0xACu8.into();
    /// assert_eq!(status.cmd(), Ok(CmdStatus::Complete));
    /// ```
    pub const fn cmd(&self) -> Result<CmdStatus, u8> {
        CmdStatus::from_bits((self.0 >> 1) & 0b111)
    }
}

impl core::fmt::Display for Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Status")
            .field("mode", &self.mode())
            .field("cmd", &self.cmd())
            .finish()
    }
}
