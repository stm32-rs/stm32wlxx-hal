/// Clock in standby mode.
///
/// Used by [`set_standby`].
///
/// [`set_standby`]: crate::SubGhz::set_standby
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Clone, Copy)]
#[repr(u8)]
pub enum StandbyClk {
    /// RC 13 MHz used in standby mode.
    Rc = 0b0,
    /// HSE32 used in standby mode.
    Hse32 = 0b1,
}
