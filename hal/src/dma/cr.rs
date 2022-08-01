/// Transfer size.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u32)]
pub enum Size {
    /// 8-bit transfer size
    Bits8 = 0b00,
    /// 16-bit transfer size
    Bits16 = 0b01,
    /// 32-bit transfer size
    Bits32 = 0b10,
}

impl Size {
    const fn from_bits(bits: u32) -> Option<Size> {
        match bits {
            0b00 => Some(Size::Bits8),
            0b01 => Some(Size::Bits16),
            0b10 => Some(Size::Bits32),
            _ => None,
        }
    }
}

/// Priority levels.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u32)]
pub enum Priority {
    /// Low priority
    Low = 0b00,
    /// Medium priority
    Medium = 0b01,
    /// High priority
    High = 0b10,
    /// Very high priority
    VeryHigh = 0b11,
}

/// Transfer directions.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Dir {
    /// Read from peripheral
    FromPeriph,
    /// Read from memory
    FromMem,
}

/// Channel configuration register.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Cr {
    val: u32,
}

impl Cr {
    /// Reset value of the register.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    /// assert_eq!(Cr::RESET.raw(), 0);
    /// ```
    pub const RESET: Cr = Cr::new(0);

    /// Reset value + DMA disabled.
    ///
    /// This is equivalent to the reset value, it is provided to make the code
    /// more expressive.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    /// assert_eq!(Cr::DISABLE.enabled(), false);
    /// assert_eq!(Cr::DISABLE, Cr::RESET);
    /// ```
    pub const DISABLE: Cr = Cr::RESET.set_enable(false);

    /// Create a new Cr register from a raw value.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    /// const CR: Cr = Cr::new(0x1234_5678);
    /// ```
    pub const fn new(val: u32) -> Cr {
        Cr { val }
    }

    /// Get the raw value of the register.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    /// const CR: Cr = Cr::new(0x1234_5678);
    /// assert_eq!(CR.raw(), 0x1234_5678);
    /// ```
    pub const fn raw(self) -> u32 {
        self.val
    }

    /// Set the privileged mode bit.
    ///
    /// This bit can only be set and cleared by a privileged software.
    ///
    /// * `false:` disabled
    /// * `true`: enabled
    ///
    /// This bit must not be written when the channel is enabled
    /// ([`enabled`] = true). It is read-only when the channel is enabled.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.privileged(), false);
    ///
    /// let cr = cr.set_privileged(true);
    /// assert_eq!(cr.privileged(), true);
    ///
    /// let cr = cr.set_privileged(false);
    /// assert_eq!(cr.privileged(), false);
    /// ```
    ///
    /// [`enabled`]: crate::dma::Cr::enabled
    #[must_use = "set_privileged returns a modified Cr"]
    pub const fn set_privileged(mut self, privileged: bool) -> Cr {
        if privileged {
            self.val |= 1 << 20;
        } else {
            self.val &= !(1 << 20);
        }
        self
    }

    /// Returns `true` if privileged mode is enabled.
    pub const fn privileged(&self) -> bool {
        (self.val >> 20) & 0b1 != 0
    }

    /// Set the DMA destination security bit.
    ///
    /// This bit can only be read, set or cleared by a secure software.
    /// It must be a privileged software if the channel is in privileged mode.
    ///
    /// This bit is cleared by hardware when the securely written data bit 17
    /// ([`set_secure`]) is cleared
    /// (on a secure reconfiguration of the channel as non-secure).
    ///
    /// A non-secure write of 1 to this secure configuration bit has no impact
    /// on the register setting and an illegal access pulse is asserted.
    ///
    /// Destination (peripheral or memory) of the DMA transfer is set with
    /// [`set_dir_from_mem`] or [`set_dir_from_periph`].
    ///
    /// * `false`: non-secure DMA transfer to the destination
    /// * `true`: secure DMA transfer to the destination
    ///
    /// This bit must not be written when the channel is enabled
    /// ([`enabled`] = true). It is read-only when the channel is enabled.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.dest_sec(), false);
    ///
    /// let cr = cr.set_dest_sec(true);
    /// assert_eq!(cr.dest_sec(), true);
    ///
    /// let cr = cr.set_dest_sec(false);
    /// assert_eq!(cr.dest_sec(), false);
    /// ```
    ///
    /// [`enabled`]: crate::dma::Cr::enabled
    /// [`set_dir_from_mem`]: crate::dma::Cr::set_dir_from_mem
    /// [`set_dir_from_periph`]: crate::dma::Cr::set_dir_from_periph
    /// [`set_secure`]: crate::dma::Cr::set_secure
    #[must_use = "set_dest_sec returns a modified Cr"]
    pub const fn set_dest_sec(mut self, dsec: bool) -> Cr {
        if dsec {
            self.val |= 1 << 19;
        } else {
            self.val &= !(1 << 19);
        }
        self
    }

    /// Returns `true` if destination secure mode is enabled.
    ///
    /// This bit can only be read by a secure software.
    /// A non-secure read to this secure configuration bit returns 0.
    pub const fn dest_sec(&self) -> bool {
        (self.val >> 19) & 0b1 != 0
    }

    /// Set the DMA source security bit.
    ///
    /// This bit can only be read, set or cleared by a secure software.
    /// It must be a privileged software if the channel is in privileged mode.
    ///
    /// This bit is cleared by hardware when the securely written data bit 17
    /// ([`set_secure`]) is cleared
    /// (on a secure reconfiguration of the channel as non-secure).
    ///
    /// A non-secure write of 1 to this secure configuration bit has no impact
    /// on the register setting and an illegal access pulse is asserted.
    ///
    /// Source (peripheral or memory) of the DMA transfer is set with
    /// [`set_dir_from_mem`] or [`set_dir_from_periph`].
    ///
    /// This bit must not be written when the channel is enabled
    /// ([`enabled`] = true). It is read-only when the channel is enabled.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.src_sec(), false);
    ///
    /// let cr = cr.set_src_sec(true);
    /// assert_eq!(cr.src_sec(), true);
    ///
    /// let cr = cr.set_src_sec(false);
    /// assert_eq!(cr.src_sec(), false);
    /// ```
    ///
    /// [`enabled`]: crate::dma::Cr::enabled
    /// [`set_dir_from_mem`]: crate::dma::Cr::set_dir_from_mem
    /// [`set_dir_from_periph`]: crate::dma::Cr::set_dir_from_periph
    /// [`set_secure`]: crate::dma::Cr::set_secure
    #[must_use = "set_src_sec returns a modified Cr"]
    pub const fn set_src_sec(mut self, ssec: bool) -> Cr {
        if ssec {
            self.val |= 1 << 18;
        } else {
            self.val &= !(1 << 18);
        }
        self
    }

    /// Returns `true` if source secure mode is enabled.
    ///
    /// This bit can only be read by a secure software.
    /// A non-secure read to this secure configuration bit returns 0.
    pub const fn src_sec(&self) -> bool {
        (self.val >> 18) & 0b1 != 0
    }

    /// Set the secure mode bit.
    ///
    /// This bit can only be set or cleared by a secure software.
    ///
    /// * `false`: non-secure channel
    /// * `true`: secure channel
    ///
    /// This bit must not be written when the channel is enabled
    /// ([`enabled`] = true). It is read-only when the channel is enabled.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.secure(), false);
    ///
    /// let cr = cr.set_secure(true);
    /// assert_eq!(cr.secure(), true);
    ///
    /// let cr = cr.set_secure(false);
    /// assert_eq!(cr.secure(), false);
    /// ```
    ///
    /// [`enabled`]: crate::dma::Cr::enabled
    #[must_use = "set_secm returns a modified Cr"]
    pub const fn set_secure(mut self, sec: bool) -> Cr {
        if sec {
            self.val |= 1 << 17;
        } else {
            self.val &= !(1 << 17);
        }
        self
    }

    /// Returns `true` if the secure mode bit is set.
    pub const fn secure(&self) -> bool {
        (self.val >> 17) & 0b1 != 0
    }

    /// Set the memory-to-memory mode bit.
    ///
    /// If enabled (`true`) the DMA channels operate without being triggered
    /// by a request from a peripheral.
    ///
    /// Note: The memory-to-memory mode must not be used in circular mode.
    /// Before enabling a channel in memory-to-memory mode, the software must
    /// clear the `CIRC` bit of the `DMA_CCRx` register.
    ///
    /// **Note:** This field is set and cleared by software
    /// (privileged/secure software if the channel is in privileged/secure mode).
    ///
    /// This bit must not be written when the channel is enabled
    /// ([`enabled`] = true). It is read-only when the channel is enabled.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.mem2mem(), false);
    ///
    /// let cr = cr.set_mem2mem(true);
    /// assert_eq!(cr.mem2mem(), true);
    ///
    /// let cr = cr.set_mem2mem(false);
    /// assert_eq!(cr.mem2mem(), false);
    /// ```
    ///
    /// [`enabled`]: crate::dma::Cr::enabled
    #[must_use = "set_mem2mem returns a modified Cr"]
    pub const fn set_mem2mem(mut self, en: bool) -> Cr {
        if en {
            self.val |= 1 << 14;
        } else {
            self.val &= !(1 << 14);
        }
        self
    }

    /// Get the memory-to-memory mode bit.
    pub const fn mem2mem(&self) -> bool {
        (self.val >> 14) & 0b1 != 0
    }

    /// Set the priority level.
    ///
    /// **Note:** This field is set and cleared by software
    /// (privileged/secure software if the channel is in privileged/secure mode).
    ///
    /// This bit must not be written when the channel is enabled
    /// ([`enabled`] = true). It is read-only when the channel is enabled.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::{Cr, Priority};
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.priority(), Priority::Low);
    ///
    /// let cr = cr.set_priority(Priority::VeryHigh);
    /// assert_eq!(cr.priority(), Priority::VeryHigh);
    ///
    /// let cr = cr.set_priority(Priority::High);
    /// assert_eq!(cr.priority(), Priority::High);
    ///
    /// let cr = cr.set_priority(Priority::Medium);
    /// assert_eq!(cr.priority(), Priority::Medium);
    ///
    /// let cr = cr.set_priority(Priority::Low);
    /// assert_eq!(cr.priority(), Priority::Low);
    /// ```
    ///
    /// [`enabled`]: crate::dma::Cr::enabled
    #[must_use = "set_priority returns a modified Cr"]
    pub const fn set_priority(mut self, priority: Priority) -> Cr {
        self.val &= !(0b11 << 12);
        self.val |= ((priority as u32) & 0b11) << 12;
        self
    }

    /// Get the priority level.
    #[allow(clippy::wildcard_in_or_patterns)]
    pub const fn priority(&self) -> Priority {
        match (self.val >> 12) & 0b11 {
            0b00 => Priority::Low,
            0b01 => Priority::Medium,
            0b10 => Priority::High,
            0b11 | _ => Priority::VeryHigh,
        }
    }

    /// Defines the data size of each DMA transfer to the identified memory.
    ///
    /// In memory-to-memory mode, this field identifies the memory source if
    /// [`dir`] = [`FromMem`] and the memory destination if
    /// [`dir`] = [`FromPeriph`].
    ///
    /// In peripheral-to-peripheral mode, this field identifies the peripheral
    /// source if [`dir`] = [`FromMem`] and the peripheral destination if
    /// [`dir`] = [`FromPeriph`].
    ///
    /// **Note:** This field is set and cleared by software
    /// (privileged/secure software if the channel is in privileged/secure mode).
    ///
    /// This bit must not be written when the channel is enabled
    /// ([`enabled`] = true). It is read-only when the channel is enabled.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::{Cr, Size};
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.mem_size(), Some(Size::Bits8));
    ///
    /// let cr = cr.set_mem_size(Size::Bits32);
    /// assert_eq!(cr.mem_size(), Some(Size::Bits32));
    ///
    /// let cr = cr.set_mem_size(Size::Bits16);
    /// assert_eq!(cr.mem_size(), Some(Size::Bits16));
    ///
    /// let cr = cr.set_mem_size(Size::Bits8);
    /// assert_eq!(cr.mem_size(), Some(Size::Bits8));
    /// ```
    ///
    /// [`dir`]: crate::dma::Cr::dir
    /// [`enabled`]: crate::dma::Cr::enabled
    /// [`FromMem`]: crate::dma::Dir::FromPeriph
    /// [`FromPeriph`]: crate::dma::Dir::FromPeriph
    #[must_use = "set_mem_size returns a modified Cr"]
    pub const fn set_mem_size(mut self, size: Size) -> Cr {
        self.val &= !(0b11 << 10);
        self.val |= ((size as u32) & 0b11) << 10;
        self
    }

    /// Get the memory DMA transfer size.
    pub const fn mem_size(&self) -> Option<Size> {
        Size::from_bits((self.val >> 10) & 0b11)
    }

    /// Defines the data size of each DMA transfer to the identified peripheral.
    ///
    /// In memory-to-memory mode, this field identifies the memory destination
    /// if [`dir`] = [`FromMem`] and the memory source if
    /// [`dir`] = [`FromPeriph`].
    ///
    /// In peripheral-to-peripheral mode, this field identifies the peripheral
    /// destination if [`dir`] = [`FromMem`] and the peripheral source if
    /// [`dir`] = [`FromPeriph`].
    ///
    /// **Note:** This field is set and cleared by software
    /// (privileged/secure software if the channel is in privileged/secure mode).
    ///
    /// This bit must not be written when the channel is enabled
    /// ([`enabled`] = true). It is read-only when the channel is enabled.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::{Cr, Size};
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.periph_size(), Some(Size::Bits8));
    ///
    /// let cr = cr.set_periph_size(Size::Bits32);
    /// assert_eq!(cr.periph_size(), Some(Size::Bits32));
    ///
    /// let cr = cr.set_periph_size(Size::Bits16);
    /// assert_eq!(cr.periph_size(), Some(Size::Bits16));
    ///
    /// let cr = cr.set_periph_size(Size::Bits8);
    /// assert_eq!(cr.periph_size(), Some(Size::Bits8));
    /// ```
    ///
    /// [`dir`]: crate::dma::Cr::dir
    /// [`enabled`]: crate::dma::Cr::enabled
    /// [`FromMem`]: crate::dma::Dir::FromPeriph
    /// [`FromPeriph`]: crate::dma::Dir::FromPeriph
    #[must_use = "set_periph_size returns a modified Cr"]
    pub const fn set_periph_size(mut self, size: Size) -> Cr {
        self.val &= !(0b11 << 8);
        self.val |= ((size as u32) & 0b11) << 8;
        self
    }

    /// Get the peripheral DMA transfer size.
    pub const fn periph_size(&self) -> Option<Size> {
        Size::from_bits((self.val >> 8) & 0b11)
    }

    /// Defines the increment mode for each DMA transfer to the identified
    /// memory.
    ///
    /// In memory-to-memory mode, this field identifies the memory source if
    /// [`dir`] = [`FromMem`] and the memory destination if
    /// [`dir`] = [`FromPeriph`].
    ///
    /// In peripheral-to-peripheral mode, this field identifies the peripheral
    /// source if [`dir`] = [`FromMem`] and the peripheral destination if
    /// [`dir`] = [`FromPeriph`].
    ///
    /// **Note:** This field is set and cleared by software
    /// (privileged/secure software if the channel is in privileged/secure mode).
    ///
    /// This bit must not be written when the channel is enabled
    /// ([`enabled`] = true). It is read-only when the channel is enabled.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.mem_inc(), false);
    ///
    /// let cr = cr.set_mem_inc(true);
    /// assert_eq!(cr.mem_inc(), true);
    ///
    /// let cr = cr.set_mem_inc(false);
    /// assert_eq!(cr.mem_inc(), false);
    /// ```
    ///
    /// [`dir`]: crate::dma::Cr::dir
    /// [`enabled`]: crate::dma::Cr::enabled
    /// [`FromMem`]: crate::dma::Dir::FromPeriph
    /// [`FromPeriph`]: crate::dma::Dir::FromPeriph
    #[must_use = "set_mem_inc returns a modified Cr"]
    pub const fn set_mem_inc(mut self, inc: bool) -> Cr {
        if inc {
            self.val |= 1 << 7
        } else {
            self.val &= !(1 << 7)
        }
        self
    }

    /// Get the memory increment bit.
    pub const fn mem_inc(&self) -> bool {
        (self.val >> 7) & 0b1 != 0
    }

    /// Defines the increment mode for each DMA transfer to the identified peripheral.
    ///
    /// In memory-to-memory mode, this field identifies the memory destination
    /// if [`dir`] = [`FromMem`] and the memory source if
    /// [`dir`] = [`FromPeriph`].
    ///
    /// In peripheral-to-peripheral mode, this field identifies the peripheral
    /// destination if [`dir`] = [`FromMem`] and the peripheral source if
    /// [`dir`] = [`FromPeriph`].
    ///
    /// **Note:** This field is set and cleared by software
    /// (privileged/secure software if the channel is in privileged/secure mode).
    ///
    /// This bit must not be written when the channel is enabled
    /// ([`enabled`] = true). It is read-only when the channel is enabled.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.periph_inc(), false);
    ///
    /// let cr = cr.set_periph_inc(true);
    /// assert_eq!(cr.periph_inc(), true);
    ///
    /// let cr = cr.set_periph_inc(false);
    /// assert_eq!(cr.periph_inc(), false);
    /// ```
    ///
    /// [`dir`]: crate::dma::Cr::dir
    /// [`enabled`]: crate::dma::Cr::enabled
    /// [`FromMem`]: crate::dma::Dir::FromPeriph
    /// [`FromPeriph`]: crate::dma::Dir::FromPeriph
    #[must_use = "set_periph_inc returns a modified Cr"]
    pub const fn set_periph_inc(mut self, inc: bool) -> Cr {
        if inc {
            self.val |= 1 << 6
        } else {
            self.val &= !(1 << 6)
        }
        self
    }

    /// Get the peripheral increment bit.
    pub const fn periph_inc(&self) -> bool {
        (self.val >> 6) & 0b1 != 0
    }

    /// Set the circular mode bit.
    ///
    /// In circular mode, after the last data transfer, the `DMA_CNDTRx` register
    /// is automatically reloaded with the initially programmed value.
    /// The current internal address registers are reloaded with the base
    /// address values from the `DMA_CPARx` and `DMA_CMARx` registers.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.circ(), false);
    ///
    /// let cr = cr.set_circ(true);
    /// assert_eq!(cr.circ(), true);
    ///
    /// let cr = cr.set_circ(false);
    /// assert_eq!(cr.circ(), false);
    /// ```
    #[must_use = "set_circ returns a modified Cr"]
    pub const fn set_circ(mut self, circ: bool) -> Cr {
        if circ {
            self.val |= 1 << 5
        } else {
            self.val &= !(1 << 5)
        }
        self
    }

    /// Get the circular mode bit.
    pub const fn circ(&self) -> bool {
        (self.val >> 5) & 0b1 != 0
    }

    /// Set the transfer direction from memory.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::{Cr, Dir};
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.dir(), Dir::FromPeriph);
    ///
    /// let cr = cr.set_dir_from_mem();
    /// assert_eq!(cr.dir(), Dir::FromMem);
    /// ```
    #[must_use = "set_dir_from_mem returns a modified Cr"]
    pub const fn set_dir_from_mem(self) -> Cr {
        self.set_dir(Dir::FromMem)
    }

    /// Set the transfer direction from peripheral.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::{Cr, Dir};
    ///
    /// let cr = Cr::RESET.set_dir_from_mem();
    /// assert_eq!(cr.dir(), Dir::FromMem);
    ///
    /// let cr = cr.set_dir_from_periph();
    /// assert_eq!(cr.dir(), Dir::FromPeriph);
    /// ```
    #[must_use = "set_dir_from_periph returns a modified Cr"]
    pub const fn set_dir_from_periph(self) -> Cr {
        self.set_dir(Dir::FromPeriph)
    }

    /// Set the transfer direction.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::{Cr, Dir};
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.dir(), Dir::FromPeriph);
    ///
    /// let cr = cr.set_dir(Dir::FromMem);
    /// assert_eq!(cr.dir(), Dir::FromMem);
    ///
    /// let cr = cr.set_dir(Dir::FromPeriph);
    /// assert_eq!(cr.dir(), Dir::FromPeriph);
    /// ```
    #[must_use = "set_dir returns a modified Cr"]
    pub const fn set_dir(mut self, dir: Dir) -> Cr {
        match dir {
            Dir::FromPeriph => self.val &= !(1 << 4),
            Dir::FromMem => self.val |= 1 << 4,
        }
        self
    }

    /// Get the transfer direction.
    pub const fn dir(&self) -> Dir {
        match (self.val >> 4) & 0b1 != 0 {
            true => Dir::FromMem,
            false => Dir::FromPeriph,
        }
    }

    /// Enable the transfer error interrupt.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.xfer_err_irq_en(), false);
    ///
    /// let cr = cr.set_xfer_err_irq_en(true);
    /// assert_eq!(cr.xfer_err_irq_en(), true);
    ///
    /// let cr = cr.set_xfer_err_irq_en(false);
    /// assert_eq!(cr.xfer_err_irq_en(), false);
    /// ```
    #[must_use = "set_xfer_err_irq_en returns a modified Cr"]
    pub const fn set_xfer_err_irq_en(mut self, en: bool) -> Cr {
        match en {
            true => self.val |= 1 << 3,
            false => self.val &= !(1 << 3),
        }
        self
    }

    /// Returns `true` if the transfer error interrupt is enabled.
    pub const fn xfer_err_irq_en(&self) -> bool {
        (self.val >> 3) & 0b1 != 0
    }

    /// Enable the transfer half-complete interrupt.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.xfer_hlf_irq_en(), false);
    ///
    /// let cr = cr.set_xfer_hlf_irq_en(true);
    /// assert_eq!(cr.xfer_hlf_irq_en(), true);
    ///
    /// let cr = cr.set_xfer_hlf_irq_en(false);
    /// assert_eq!(cr.xfer_hlf_irq_en(), false);
    /// ```
    #[must_use = "set_xfer_hlf_irq_en returns a modified Cr"]
    pub const fn set_xfer_hlf_irq_en(mut self, en: bool) -> Cr {
        match en {
            true => self.val |= 1 << 2,
            false => self.val &= !(1 << 2),
        }
        self
    }

    /// Returns `true` if the transfer half-complete interrupt is enabled.
    pub const fn xfer_hlf_irq_en(&self) -> bool {
        (self.val >> 2) & 0b1 != 0
    }

    /// Enable the transfer complete interrupt.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.xfer_cpl_irq_en(), false);
    ///
    /// let cr = cr.set_xfer_cpl_irq_en(true);
    /// assert_eq!(cr.xfer_cpl_irq_en(), true);
    ///
    /// let cr = cr.set_xfer_cpl_irq_en(false);
    /// assert_eq!(cr.xfer_cpl_irq_en(), false);
    /// ```
    #[must_use = "set_xfer_cpl_irq_en returns a modified Cr"]
    pub const fn set_xfer_cpl_irq_en(mut self, en: bool) -> Cr {
        match en {
            true => self.val |= 1 << 1,
            false => self.val &= !(1 << 1),
        }
        self
    }

    /// Returns `true` if the transfer complete interrupt is enabled.
    pub const fn xfer_cpl_irq_en(&self) -> bool {
        (self.val >> 1) & 0b1 != 0
    }

    /// Set the enable bit for the DMA channel.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.enabled(), false);
    ///
    /// let cr = cr.set_enable(true);
    /// assert_eq!(cr.enabled(), true);
    ///
    /// let cr = cr.set_enable(false);
    /// assert_eq!(cr.enabled(), false);
    /// ```
    #[must_use = "set_enable returns a modified Cr"]
    pub const fn set_enable(self, en: bool) -> Cr {
        if en {
            self.enable()
        } else {
            self.disable()
        }
    }

    /// Enable the DMA peripheral.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.enabled(), false);
    ///
    /// let cr = cr.enable();
    /// assert_eq!(cr.enabled(), true);
    ///
    /// let cr = cr.disable();
    /// assert_eq!(cr.enabled(), false);
    /// ```
    #[must_use = "enable returns a modified Cr"]
    pub const fn enable(mut self) -> Cr {
        self.val |= 0b1;
        self
    }

    /// Disable the DMA peripheral.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::dma::Cr;
    ///
    /// let cr = Cr::RESET;
    /// assert_eq!(cr.enabled(), false);
    ///
    /// let cr = cr.enable();
    /// assert_eq!(cr.enabled(), true);
    ///
    /// let cr = cr.disable();
    /// assert_eq!(cr.enabled(), false);
    /// ```
    #[must_use = "disable returns a modified Cr"]
    pub const fn disable(mut self) -> Cr {
        self.val &= !0b1;
        self
    }

    /// Returns `true` if the DMA channel is enabled.
    pub const fn enabled(&self) -> bool {
        self.val & 0b1 != 0
    }
}

impl Default for Cr {
    fn default() -> Cr {
        Cr::RESET
    }
}

impl From<u32> for Cr {
    fn from(raw: u32) -> Cr {
        Cr::new(raw)
    }
}

impl From<Cr> for u32 {
    fn from(reg: Cr) -> u32 {
        reg.raw()
    }
}

impl core::fmt::Display for Cr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Cr")
            .field("privileged", &self.privileged())
            .field("mem2mem", &self.mem2mem())
            .field("priority", &self.priority())
            .field("mem_size", &self.mem_size())
            .field("periph_size", &self.periph_size())
            .field("mem_inc", &self.mem_inc())
            .field("periph_inc", &self.periph_inc())
            .field("circ", &self.circ())
            .field("dir", &self.dir())
            .field("xfer_err_irq_en", &self.xfer_err_irq_en())
            .field("xfer_hlf_irq_en", &self.xfer_hlf_irq_en())
            .field("xfer_cpl_irq_en", &self.xfer_cpl_irq_en())
            .field("enabled", &self.enabled())
            .finish()
    }
}
