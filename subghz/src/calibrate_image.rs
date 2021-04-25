/// Image calibration.
///
/// An argument of [`calibrate_image`].
///
/// [`calibrate_image`]: crate::SubGhz::calibrate_image
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct CalibrateImage(pub(crate) u8, pub(crate) u8);

impl CalibrateImage {
    /// Image calibration for the 430 - 440 MHz ISM band.
    pub const ISM_430_440: CalibrateImage = CalibrateImage(0x6B, 0x6F);

    /// Image calibration for the 470 - 510 MHz ISM band.
    pub const ISM_470_510: CalibrateImage = CalibrateImage(0x75, 0x81);

    /// Image calibration for the 779 - 787 MHz ISM band.
    pub const ISM_779_787: CalibrateImage = CalibrateImage(0xC1, 0xC5);

    /// Image calibration for the 863 - 870 MHz ISM band.
    pub const ISM_863_870: CalibrateImage = CalibrateImage(0xD7, 0xDB);

    /// Image calibration for the 902 - 928 MHz ISM band.
    pub const ISM_902_928: CalibrateImage = CalibrateImage(0xE1, 0xE9);

    /// Create a new `CalibrateImage` structure from raw values.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::CalibrateImage;
    ///
    /// const CAL: CalibrateImage = CalibrateImage::new(0xE1, 0xE9);
    /// assert_eq!(CAL, CalibrateImage::ISM_902_928);
    /// ```
    pub const fn new(f1: u8, f2: u8) -> CalibrateImage {
        CalibrateImage(f1, f2)
    }

    /// Create a new `CalibrateImage` structure from two frequencies.
    ///
    /// # Arguments
    ///
    /// The units for `freq1` and `freq2` are in MHz.
    ///
    /// # Panics
    ///
    /// * Panics if `freq1` is less than `freq2`.
    /// * Panics if `freq1` or `freq1` is not a multiple of 4MHz.
    /// * Panics if `freq1` or `freq2` are greater than `1020`.
    ///
    /// # Example
    ///
    /// Create an image calibration for the 430 - 440 MHz ISM band.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::CalibrateImage;
    ///
    /// let cal: CalibrateImage = CalibrateImage::from_freq(428, 444);
    /// assert_eq!(cal, CalibrateImage::ISM_430_440);
    /// ```
    pub fn from_freq(freq1: u16, freq2: u16) -> CalibrateImage {
        assert!(freq2 >= freq1);
        assert_eq!(freq1 % 4, 0);
        assert_eq!(freq2 % 4, 0);
        assert!(freq1 <= 1020);
        assert!(freq2 <= 1020);
        CalibrateImage((freq1 / 4) as u8, (freq2 / 4) as u8)
    }
}

impl Default for CalibrateImage {
    fn default() -> Self {
        CalibrateImage::new(0xE1, 0xE9)
    }
}
