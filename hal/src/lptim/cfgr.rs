/// Timer clock prescaler.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Prescaler {
    /// /1
    Div1 = 0b000,
    /// /2
    Div2 = 0b001,
    /// /4
    Div4 = 0b010,
    /// /8
    Div8 = 0b011,
    /// /16
    Div16 = 0b100,
    /// /32
    Div32 = 0b101,
    /// /64
    Div64 = 0b110,
    /// /128
    Div128 = 0b111,
}

impl Default for Prescaler {
    /// Reset value of the prescaler.
    fn default() -> Self {
        Prescaler::Div1
    }
}

/// LPTIM1 and LPTIM2 trigger selection.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TrgSel {
    /// RTC alarm A.
    RtcAlarmA = 0b001,
    /// RTC alarm B.
    RtcAlarmB = 0b010,
    /// TAMP1 input detection.
    Tamp1 = 0b011,
    /// TAMP2 input detection.
    Tamp2 = 0b100,
    /// TAMP3 input detection.
    Tamp3 = 0b101,
    /// COMP1_OUT.
    Comp1 = 0b110,
    /// COMP2_OUT.
    Comp2 = 0b111,
}

impl From<TrgSel> for u32 {
    fn from(sel: TrgSel) -> Self {
        sel as u32
    }
}

/// LPTIM3 trigger selection.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TrgSel3 {
    /// LPTIM1_OUT.
    LpTim1 = 0b001,
    /// LPTIM2_OUT.
    LpTim2 = 0b010,
}

impl From<TrgSel3> for u32 {
    fn from(sel: TrgSel3) -> Self {
        sel as u32
    }
}

/// Trigger polarity.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TrgPol {
    /// Software trigger.
    Soft = 0b00,
    /// Rising edge is active edge.
    Rising = 0b01,
    /// Falling edge is active edge.
    Falling = 0b10,
    /// Both edges are active edges.
    Both = 0b11,
}

/// Filter for triggers and external clocks.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Filter {
    /// Any level change is considered valid.
    Any = 0b00,
    /// Level must be stable for at least 2 clock periods
    /// before it is considered as valid.
    Clk2 = 0b01,
    /// Level must be stable for at least 4 clock periods
    /// before it is considered as valid.
    Clk4 = 0b10,
    /// Level must be stable for at least 8 clock periods
    /// before it is considered as valid.
    Clk8 = 0b11,
}

impl Prescaler {
    /// Get the prescaler divisor.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Prescaler;
    ///
    /// assert_eq!(Prescaler::Div1.div(), 1);
    /// assert_eq!(Prescaler::Div2.div(), 2);
    /// assert_eq!(Prescaler::Div4.div(), 4);
    /// assert_eq!(Prescaler::Div8.div(), 8);
    /// assert_eq!(Prescaler::Div16.div(), 16);
    /// assert_eq!(Prescaler::Div32.div(), 32);
    /// assert_eq!(Prescaler::Div64.div(), 64);
    /// assert_eq!(Prescaler::Div128.div(), 128);
    /// ```
    pub const fn div(&self) -> u8 {
        match self {
            Prescaler::Div1 => 1,
            Prescaler::Div2 => 2,
            Prescaler::Div4 => 4,
            Prescaler::Div8 => 8,
            Prescaler::Div16 => 16,
            Prescaler::Div32 => 32,
            Prescaler::Div64 => 64,
            Prescaler::Div128 => 128,
        }
    }
}

/// Configuration register.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cfgr {
    val: u32,
}

impl Cfgr {
    /// Reset value of the register.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Cfgr;
    /// assert_eq!(Cfgr::RESET.raw(), 0);
    /// ```
    pub const RESET: Cfgr = Cfgr::new(0);

    /// Create a new `Cfgr` register from a raw value.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Cfgr;
    /// const CFGR: Cfgr = Cfgr::new(0);
    /// ```
    #[must_use]
    pub const fn new(val: u32) -> Cfgr {
        Cfgr { val }
    }

    /// Get the raw value of the register.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Cfgr;
    /// const CFGR: Cfgr = Cfgr::new(0x1234_5678);
    /// assert_eq!(CFGR.raw(), 0x1234_5678);
    /// ```
    #[must_use]
    pub const fn raw(self) -> u32 {
        self.val
    }

    /// Set the waveform polarity.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::Cfgr;
    ///
    /// let cfgr: Cfgr = Cfgr::RESET;
    /// assert_eq!(cfgr.wavepol(), false);
    ///
    /// let cfgr: Cfgr = cfgr.set_wavepol(true);
    /// assert_eq!(cfgr.wavepol(), true);
    ///
    /// let cfgr: Cfgr = cfgr.set_wavepol(false);
    /// assert_eq!(cfgr.wavepol(), false);
    /// ```
    #[inline]
    #[must_use = "set_wavepol returns a modified Cfgr"]
    pub const fn set_wavepol(mut self, wavepol: bool) -> Self {
        if wavepol {
            self.val |= 1 << 21;
        } else {
            self.val &= !(1 << 21);
        }
        self
    }

    /// Get the waveform polarity.
    #[inline]
    #[must_use]
    pub const fn wavepol(&self) -> bool {
        self.val & (1 << 21) != 0
    }

    /// Set the trigger polarity.
    #[inline]
    #[must_use = "set_trg_pol returns a modified Cfgr"]
    pub const fn set_trg_pol(mut self, trg_pol: TrgPol) -> Self {
        self.val &= !(0b11 << 17);
        self.val |= (trg_pol as u32) << 17;
        self
    }

    /// Set the trigger source.
    #[inline]
    #[must_use = "set_trg_sel returns a modified Cfgr"]
    pub const fn set_trg_sel(mut self, trigger: u32) -> Self {
        self.val &= !(0b111 << 13);
        self.val |= (trigger & 0b111) << 13;
        self
    }

    /// Set the trigger filter.
    #[inline]
    #[must_use = "set_trg_filter returns a modified Cfgr"]
    pub const fn set_trg_filter(mut self, filter: Filter) -> Self {
        self.val &= !(0b111 << 6);
        self.val |= ((filter as u32) & 0b111) << 6;
        self
    }

    /// Get the prescaler value.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::{Cfgr, Prescaler};
    ///
    /// assert_eq!(Cfgr::default().prescaler(), Prescaler::default());
    /// ```
    #[inline]
    #[must_use]
    pub const fn prescaler(&self) -> Prescaler {
        match (self.val >> 9) & 0b111 {
            0b000 => Prescaler::Div1,
            0b001 => Prescaler::Div2,
            0b010 => Prescaler::Div4,
            0b011 => Prescaler::Div8,
            0b100 => Prescaler::Div16,
            0b101 => Prescaler::Div32,
            0b110 => Prescaler::Div64,
            _ => Prescaler::Div128,
        }
    }

    /// Set the prescaler value.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::lptim::{Cfgr, Prescaler};
    ///
    /// let cfgr: Cfgr = Cfgr::RESET;
    ///
    /// let cfgr: Cfgr = cfgr.set_prescaler(Prescaler::Div1);
    /// assert_eq!(cfgr.prescaler(), Prescaler::Div1);
    ///
    /// let cfgr: Cfgr = cfgr.set_prescaler(Prescaler::Div2);
    /// assert_eq!(cfgr.prescaler(), Prescaler::Div2);
    ///
    /// let cfgr: Cfgr = cfgr.set_prescaler(Prescaler::Div4);
    /// assert_eq!(cfgr.prescaler(), Prescaler::Div4);
    ///
    /// let cfgr: Cfgr = cfgr.set_prescaler(Prescaler::Div8);
    /// assert_eq!(cfgr.prescaler(), Prescaler::Div8);
    ///
    /// let cfgr: Cfgr = cfgr.set_prescaler(Prescaler::Div16);
    /// assert_eq!(cfgr.prescaler(), Prescaler::Div16);
    ///
    /// let cfgr: Cfgr = cfgr.set_prescaler(Prescaler::Div32);
    /// assert_eq!(cfgr.prescaler(), Prescaler::Div32);
    ///
    /// let cfgr: Cfgr = cfgr.set_prescaler(Prescaler::Div64);
    /// assert_eq!(cfgr.prescaler(), Prescaler::Div64);
    ///
    /// let cfgr: Cfgr = cfgr.set_prescaler(Prescaler::Div128);
    /// assert_eq!(cfgr.prescaler(), Prescaler::Div128);
    /// ```
    #[inline]
    #[must_use = "set_prescaler returns a modified Cfgr"]
    pub const fn set_prescaler(mut self, pres: Prescaler) -> Self {
        self.val &= !(0b111 << 9);
        self.val |= (pres as u32) << 9;
        self
    }
}

impl From<u32> for Cfgr {
    fn from(val: u32) -> Self {
        Self { val }
    }
}

impl From<Cfgr> for u32 {
    fn from(cr: Cfgr) -> Self {
        cr.val
    }
}

impl Default for Cfgr {
    fn default() -> Self {
        Cfgr::RESET
    }
}
