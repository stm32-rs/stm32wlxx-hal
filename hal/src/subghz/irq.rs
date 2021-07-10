/// IRQ bit mapping
///
/// See table 37 "IRQ bit mapping and definition" in the reference manual for
/// more information.
#[repr(u16)]
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub enum Irq {
    /// Packet transmission finished.
    ///
    /// * Packet type: LoRa and GFSK
    /// * Operation: TX
    TxDone = (1 << 0),
    /// Packet reception finished.
    ///
    /// * Packet type: LoRa and GFSK
    /// * Operation: RX
    RxDone = (1 << 1),
    /// Preamble detected.
    ///
    /// * Packet type: LoRa and GFSK
    /// * Operation: RX
    PreambleDetected = (1 << 2),
    /// Synchronization word valid.
    ///
    /// * Packet type: GFSK
    /// * Operation: RX
    SyncDetected = (1 << 3),
    /// Header valid.
    ///
    /// * Packet type: LoRa
    /// * Operation: RX
    HeaderValid = (1 << 4),
    /// Header CRC error.
    ///
    /// * Packet type: LoRa
    /// * Operation: RX
    HeaderErr = (1 << 5),
    /// Dual mening error.
    ///
    /// For GFSK RX this indicates a preamble, syncword, address, CRC, or length
    /// error.
    ///
    /// For LoRa RX this indicates a CRC error.
    Err = (1 << 6),
    /// Channel activity detection finished.
    ///
    /// * Packet type: LoRa
    /// * Operation: CAD
    CadDone = (1 << 7),
    /// Channel activity detected.
    ///
    /// * Packet type: LoRa
    /// * Operation: CAD
    CadDetected = (1 << 8),
    /// RX or TX timeout.
    ///
    /// * Packet type: LoRa and GFSK
    /// * Operation: RX and TX
    Timeout = (1 << 9),
}

impl Irq {
    /// Get the bitmask for an IRQ.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::subghz::Irq;
    ///
    /// assert_eq!(Irq::TxDone.mask(), 0x0001u16);
    /// assert_eq!(Irq::Timeout.mask(), 0x0200u16);
    /// ```
    pub const fn mask(self) -> u16 {
        self as u16
    }
}

/// Interrupt lines.
///
/// This is an argument of [`CfgDioIrq::irq_enable`] and
/// [`CfgDioIrq::irq_disable`].
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub enum IrqLine {
    /// Global interrupt.
    Global,
    /// Interrupt line 1.
    Line1,
    /// Interrupt line 2.
    Line2,
    /// Interrupt line 3.
    Line3,
}

impl IrqLine {
    pub(super) const fn offset(&self) -> usize {
        match self {
            IrqLine::Global => 1,
            IrqLine::Line1 => 3,
            IrqLine::Line2 => 5,
            IrqLine::Line3 => 7,
        }
    }
}

/// Argument for [`set_irq_cfg`].
///
/// [`set_irq_cfg`]: crate::subghz::SubGhz::set_irq_cfg
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CfgDioIrq {
    buf: [u8; 9],
}

impl CfgDioIrq {
    /// Create a new `CfgDioIrq`.
    ///
    /// This is the same as `default`, but in a `const` function.
    ///
    /// The default value has all interrupts disabled on all lines.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::subghz::CfgDioIrq;
    ///
    /// const IRQ_CFG: CfgDioIrq = CfgDioIrq::new();
    /// ```
    pub const fn new() -> CfgDioIrq {
        CfgDioIrq {
            buf: [
                super::OpCode::CfgDioIrq as u8,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
            ],
        }
    }

    /// Enable an interrupt.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::subghz::{CfgDioIrq, Irq, IrqLine};
    ///
    /// const IRQ_CFG: CfgDioIrq = CfgDioIrq::new()
    ///     .irq_enable(IrqLine::Global, Irq::TxDone)
    ///     .irq_enable(IrqLine::Global, Irq::Timeout);
    /// # assert_eq!(IRQ_CFG.as_slice()[1], 0x02);
    /// # assert_eq!(IRQ_CFG.as_slice()[2], 0x01);
    /// ```
    #[must_use = "irq_enable returns a modified CfgDioIrq"]
    pub const fn irq_enable(mut self, line: IrqLine, irq: Irq) -> CfgDioIrq {
        let mask: u16 = irq as u16;
        let offset: usize = line.offset();
        self.buf[offset] |= ((mask >> 8) & 0xFF) as u8;
        self.buf[offset + 1] |= (mask & 0xFF) as u8;
        self
    }

    /// Disable an interrupt.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::subghz::{CfgDioIrq, Irq, IrqLine};
    ///
    /// const IRQ_CFG: CfgDioIrq = CfgDioIrq::new()
    ///     .irq_enable(IrqLine::Global, Irq::TxDone)
    ///     .irq_enable(IrqLine::Global, Irq::Timeout)
    ///     .irq_disable(IrqLine::Global, Irq::TxDone)
    ///     .irq_disable(IrqLine::Global, Irq::Timeout);
    /// # assert_eq!(IRQ_CFG, CfgDioIrq::new());
    /// ```
    #[must_use = "irq_disable returns a modified CfgDioIrq"]
    pub const fn irq_disable(mut self, line: IrqLine, irq: Irq) -> CfgDioIrq {
        let mask: u16 = !(irq as u16);
        let offset: usize = line.offset();
        self.buf[offset] &= ((mask >> 8) & 0xFF) as u8;
        self.buf[offset + 1] &= (mask & 0xFF) as u8;
        self
    }

    /// Extracts a slice containing the packet.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::subghz::{CfgDioIrq, Irq, IrqLine};
    ///
    /// const IRQ_CFG: CfgDioIrq = CfgDioIrq::new()
    ///     .irq_enable(IrqLine::Global, Irq::TxDone)
    ///     .irq_enable(IrqLine::Global, Irq::Timeout)
    ///     .irq_enable(IrqLine::Line1, Irq::RxDone)
    ///     .irq_enable(IrqLine::Line2, Irq::PreambleDetected)
    ///     .irq_enable(IrqLine::Line3, Irq::CadDetected);
    ///
    /// assert_eq!(
    ///     IRQ_CFG.as_slice(),
    ///     &[0x08, 0x02, 0x01, 0x00, 0x02, 0x00, 0x04, 0x01, 0x00]
    /// );
    /// ```
    pub const fn as_slice(&self) -> &[u8] {
        &self.buf
    }
}

impl Default for CfgDioIrq {
    fn default() -> Self {
        Self::new()
    }
}
