/// Receiver event which stops the RX timeout timer.
///
/// Used by [`set_rx_timeout_stop`].
///
/// [`set_rx_timeout_stop`]: crate::SubGhz::set_rx_timeout_stop
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
#[repr(u8)]
pub enum RxTimeoutStop {
    /// Receive timeout stopped on synchronization word detection in generic
    /// packet mode or header detection in LoRa packet mode.
    Sync = 0b0,
    /// Receive timeout stopped on preamble detection.
    Preamble = 0b1,
}
