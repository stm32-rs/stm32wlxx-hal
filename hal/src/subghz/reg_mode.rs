/// Radio power supply selection.
#[derive(Debug, Default, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum RegMode {
    /// Linear dropout regulator
    #[default]
    Ldo = 0b0,
    /// Switch mode power supply.
    ///
    /// Used in standby with HSE32, FS, RX, and TX modes.
    Smps = 0b1,
}
