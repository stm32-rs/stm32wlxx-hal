/// RF switch RX.
pub trait RfSwRx {
    /// Set the RF switch to receive.
    fn set_rx(&mut self);
}

/// RF switch TX low-power.
pub trait RfSwTxLp {
    /// Set the RF switch to low-power transmit.
    fn set_tx_lp(&mut self);
}

/// RF switch TX high-power.
pub trait RfSwTxHp {
    /// Set the RF switch to high-power transmit.
    fn set_tx_hp(&mut self);
}
