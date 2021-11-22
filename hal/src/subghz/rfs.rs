use crate::subghz::{PaConfig, TxParams};

/// Contains functionality for the RF switch to switch to receiving mode.
pub trait RfSwRx {
    /// Set the RF switch to receive.
    fn set_rx(&mut self);
}

/// Contains functionality for the RF switch to switch to transmitting mode.
pub trait RfSwTx {
    /// The power amplifier configuration to use.
    const PA_CONFIG: PaConfig;
    /// The transmission parameters to use.
    const TX_PARAMS: TxParams;

    /// Set the RF switch to transmit.
    fn set_tx(&mut self);
}
