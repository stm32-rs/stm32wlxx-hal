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

impl Prescaler {
    /// Get the prescaler divisor.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::lptim::Prescaler;
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
    /// use stm32wl_hal::lptim::Cfgr;
    /// assert_eq!(Cfgr::RESET.raw(), 0);
    /// ```
    pub const RESET: Cfgr = Cfgr::new(0);

    /// Cfgreate a new Cfgr register from a raw value.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::lptim::Cfgr;
    /// const CFGR: Cfgr = Cfgr::new(0);
    /// ```
    pub const fn new(val: u32) -> Cfgr {
        Cfgr { val }
    }

    /// Get the raw value of the register.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::lptim::Cfgr;
    /// const CFGR: Cfgr = Cfgr::new(0x1234_5678);
    /// assert_eq!(CFGR.raw(), 0x1234_5678);
    /// ```
    pub const fn raw(self) -> u32 {
        self.val
    }

    /// Get the prescaler value.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal::lptim::{Cfgr, Prescaler};
    ///
    /// assert_eq!(Cfgr::default().prescaler(), Prescaler::default());
    /// ```
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
    /// use stm32wl_hal::lptim::{Cfgr, Prescaler};
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
