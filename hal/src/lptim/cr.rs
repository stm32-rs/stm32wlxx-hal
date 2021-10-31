/// Control register.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cr {
    val: u32,
}

impl Cr {
    /// Reset value of the register.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Cr;
    /// assert_eq!(Cr::RESET.raw(), 0);
    /// ```
    pub const RESET: Cr = Cr::new(0);

    /// Reset value + timer disabled.
    ///
    /// This is equivalent to the reset value, it is provided to make the code
    /// more expressive.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Cr;
    /// assert_eq!(Cr::DISABLE.enabled(), false);
    /// assert_eq!(Cr::DISABLE, Cr::RESET);
    /// ```
    pub const DISABLE: Cr = Cr::RESET.set_enable(false);

    /// Create a new Cr register from a raw value.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Cr;
    /// const CR: Cr = Cr::new(0b1);
    /// ```
    pub const fn new(val: u32) -> Cr {
        Cr { val }
    }

    /// Get the raw value of the register.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Cr;
    /// const CR: Cr = Cr::new(0x1234_5678);
    /// assert_eq!(CR.raw(), 0x1234_5678);
    /// ```
    pub const fn raw(self) -> u32 {
        self.val
    }

    /// Set the counter reset.
    ///
    /// This bit is cleared by hardware.
    #[must_use = "set_cnt_rst returns a modified Cr"]
    pub const fn set_cnt_rst(mut self) -> Cr {
        self.val |= 1 << 3;
        self
    }

    /// Returns `true` if the counter reset bit is set.
    pub const fn cnt_rst(&self) -> bool {
        self.val & (1 << 3) != 0
    }

    /// Start the timer in continuous mode.
    #[must_use = "set_continuous returns a modified Cr"]
    pub const fn set_continuous(mut self) -> Cr {
        self.val |= 1 << 2;
        self
    }

    /// Start the timer in single-shot mode.
    #[must_use = "set_single returns a modified Cr"]
    pub const fn set_single(mut self) -> Cr {
        self.val |= 1 << 1;
        self
    }

    /// Set the enable bit for the timer.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Cr;
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

    /// Enable the LPTIM peripheral.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Cr;
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

    /// Disable the LPTIM peripheral.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Cr;
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

    /// Returns `true` if the timer is enabled.
    pub const fn enabled(&self) -> bool {
        self.val & 0b1 != 0
    }
}

impl From<u32> for Cr {
    fn from(val: u32) -> Self {
        Self { val }
    }
}

impl From<Cr> for u32 {
    fn from(cr: Cr) -> Self {
        cr.val
    }
}

impl Default for Cr {
    fn default() -> Self {
        Cr::RESET
    }
}
