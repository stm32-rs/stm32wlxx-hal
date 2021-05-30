use core::time::Duration;

use crate::ValueError;

const fn abs_diff(a: u64, b: u64) -> u64 {
    if a > b {
        a - b
    } else {
        b - a
    }
}

/// Timeout argument.
///
/// This is used by:
/// * [`set_rx`]
/// * [`set_tx`]
/// * [`TcxoMode`]
///
/// Each timeout has 3 bytes, with a resolution of 15.625µs per bit, giving a
/// range of 0s to 262.143984375s.
///
/// [`set_rx`]: crate::SubGhz::set_rx
/// [`set_tx`]: crate::SubGhz::set_tx
/// [`TcxoMode`]: crate::TcxoMode
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Hash)]
pub struct Timeout {
    bits: u32,
}

impl Timeout {
    /// Disable the timeout (0s timeout).
    ///
    /// # Example
    ///
    /// ```
    /// use core::time::Duration;
    /// use stm32wl_hal_subghz::Timeout;
    ///
    /// const TIMEOUT: Timeout = Timeout::DISABLED;
    /// assert_eq!(TIMEOUT.as_duration(), Duration::from_secs(0));
    /// ```
    pub const DISABLED: Timeout = Timeout { bits: 0x0 };

    /// Minimum timeout, 15.625µs.
    ///
    /// # Example
    ///
    /// ```
    /// use core::time::Duration;
    /// use stm32wl_hal_subghz::Timeout;
    ///
    /// const TIMEOUT: Timeout = Timeout::MIN;
    /// assert_eq!(TIMEOUT.into_bits(), 1);
    /// ```
    pub const MIN: Timeout = Timeout { bits: 1 };

    /// Maximum timeout, 262.143984375s.
    ///
    /// # Example
    ///
    /// ```
    /// use core::time::Duration;
    /// use stm32wl_hal_subghz::Timeout;
    ///
    /// const TIMEOUT: Timeout = Timeout::MAX;
    /// assert_eq!(TIMEOUT.as_duration(), Duration::from_nanos(262_143_984_375));
    /// ```
    pub const MAX: Timeout = Timeout { bits: 0x00FF_FFFF };

    /// Timeout resolution in nanoseconds, 15.625µs.
    pub const RESOLUTION_NANOS: u64 = 15_625;

    /// Timeout resolution, 15.625µs.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::Timeout;
    ///
    /// assert_eq!(
    ///     Timeout::RESOLUTION.as_nanos(),
    ///     Timeout::RESOLUTION_NANOS as u128
    /// );
    /// ```
    pub const RESOLUTION: Duration = Duration::from_nanos(Self::RESOLUTION_NANOS);

    /// Create a new timeout from a [`Duration`].
    ///
    /// This will return the nearest timeout value possible, or a
    /// [`ValueError`] if the value is out of bounds.
    ///
    /// This is not _that_ useful right now, it is simply future proofing for a
    /// time when `Result::unwrap` is avaliable for `const fn`.
    ///
    /// # Example
    ///
    /// Value within bounds:
    ///
    /// ```
    /// use core::time::Duration;
    /// use stm32wl_hal_subghz::{Timeout, ValueError};
    ///
    /// const MIN: Duration = Timeout::RESOLUTION;
    /// assert_eq!(Timeout::from_duration(MIN).unwrap(), Timeout::MIN);
    /// ```
    ///
    /// Value too low:
    ///
    /// ```
    /// use core::time::Duration;
    /// use stm32wl_hal_subghz::{Timeout, ValueError};
    ///
    /// const LOWER_LIMIT_NANOS: u128 = 7813;
    /// const TOO_LOW_NANOS: u128 = LOWER_LIMIT_NANOS - 1;
    /// const TOO_LOW_DURATION: Duration = Duration::from_nanos(TOO_LOW_NANOS as u64);
    /// assert_eq!(
    ///     Timeout::from_duration(TOO_LOW_DURATION),
    ///     Err(ValueError::too_low(TOO_LOW_NANOS, LOWER_LIMIT_NANOS))
    /// );
    /// ```
    ///
    /// Value too high:
    ///
    /// ```
    /// use core::time::Duration;
    /// use stm32wl_hal_subghz::{Timeout, ValueError};
    ///
    /// const UPPER_LIMIT_NANOS: u128 = Timeout::MAX.as_nanos() as u128 + 7812;
    /// const TOO_HIGH_NANOS: u128 = UPPER_LIMIT_NANOS + 1;
    /// const TOO_HIGH_DURATION: Duration = Duration::from_nanos(TOO_HIGH_NANOS as u64);
    /// assert_eq!(
    ///     Timeout::from_duration(TOO_HIGH_DURATION),
    ///     Err(ValueError::too_high(TOO_HIGH_NANOS, UPPER_LIMIT_NANOS))
    /// );
    /// ```
    pub const fn from_duration(duration: Duration) -> Result<Timeout, ValueError<u128>> {
        // developers note: at the time of development many methods in
        // in `core::Duration` were not `const fn`, which leads to the hacks
        // you see here.
        let nanos: u128 = duration.as_nanos();
        const UPPER_LIMIT: u128 =
            Timeout::MAX.as_nanos() as u128 + (Timeout::RESOLUTION_NANOS as u128) / 2;
        const LOWER_LIMIT: u128 = (((Timeout::RESOLUTION_NANOS as u128) + 1) / 2) as u128;

        if nanos > UPPER_LIMIT {
            Err(ValueError::too_high(nanos, UPPER_LIMIT))
        } else if nanos < LOWER_LIMIT {
            Err(ValueError::too_low(nanos, LOWER_LIMIT))
        } else {
            // safe to truncate here because of previous bounds check.
            let duration_nanos: u64 = nanos as u64;

            let div_floor: u64 = duration_nanos / Self::RESOLUTION_NANOS;
            let div_ceil: u64 = 1 + (duration_nanos - 1) / Self::RESOLUTION_NANOS;

            let timeout_ceil: Timeout = Timeout::from_raw(div_ceil as u32);
            let timeout_floor: Timeout = Timeout::from_raw(div_floor as u32);

            let error_ceil: u64 = abs_diff(timeout_ceil.as_nanos(), duration_nanos);
            let error_floor: u64 = abs_diff(timeout_floor.as_nanos(), duration_nanos);

            if error_ceil < error_floor {
                Ok(timeout_ceil)
            } else {
                Ok(timeout_floor)
            }
        }
    }

    /// Create a new timeout from a [`Duration`].
    ///
    /// This will return the nearest timeout value possible, saturating at the
    /// limits.
    ///
    /// # Example
    ///
    /// ```
    /// use core::time::Duration;
    /// use stm32wl_hal_subghz::Timeout;
    ///
    /// const DURATION_MAX_NS: u64 = 262_143_984_376;
    ///
    /// assert_eq!(
    ///     Timeout::from_duration_sat(Duration::from_millis(0)),
    ///     Timeout::MIN
    /// );
    /// assert_eq!(
    ///     Timeout::from_duration_sat(Duration::from_nanos(DURATION_MAX_NS)),
    ///     Timeout::MAX
    /// );
    /// assert_eq!(
    ///     Timeout::from_duration_sat(Timeout::RESOLUTION).into_bits(),
    ///     1
    /// );
    /// ```
    pub const fn from_duration_sat(duration: Duration) -> Timeout {
        // developers note: at the time of development many methods in
        // in `core::Duration` were not `const fn`, which leads to the hacks
        // you see here.
        let nanos: u128 = duration.as_nanos();
        const UPPER_LIMIT: u128 = Timeout::MAX.as_nanos() as u128;

        if nanos > UPPER_LIMIT {
            Timeout::MAX
        } else if nanos < (Timeout::RESOLUTION_NANOS as u128) {
            Timeout::from_raw(1)
        } else {
            // safe to truncate here because of previous bounds check.
            let duration_nanos: u64 = duration.as_nanos() as u64;

            let div_floor: u64 = duration_nanos / Self::RESOLUTION_NANOS;
            let div_ceil: u64 = 1 + (duration_nanos - 1) / Self::RESOLUTION_NANOS;

            let timeout_ceil: Timeout = Timeout::from_raw(div_ceil as u32);
            let timeout_floor: Timeout = Timeout::from_raw(div_floor as u32);

            let error_ceil: u64 = abs_diff(timeout_ceil.as_nanos(), duration_nanos);
            let error_floor: u64 = abs_diff(timeout_floor.as_nanos(), duration_nanos);

            if error_ceil < error_floor {
                timeout_ceil
            } else {
                timeout_floor
            }
        }
    }

    /// Create a timeout from raw bits, where each bit has the resolution of
    /// [`Timeout::RESOLUTION`].
    ///
    /// **Note:** Only the first 24 bits of the `u32` are used, the `bits`
    /// argument will be masked.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::Timeout;
    ///
    /// assert_eq!(Timeout::from_raw(u32::MAX), Timeout::MAX);
    /// assert_eq!(Timeout::from_raw(0x00_FF_FF_FF), Timeout::MAX);
    /// assert_eq!(Timeout::from_raw(1).as_duration(), Timeout::RESOLUTION);
    /// assert_eq!(Timeout::from_raw(0), Timeout::DISABLED);
    /// ```
    pub const fn from_raw(bits: u32) -> Timeout {
        Timeout {
            bits: bits & 0x00FF_FFFF,
        }
    }

    /// Get the number of nanoseconds in the timeout.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::Timeout;
    ///
    /// assert_eq!(Timeout::MAX.as_nanos(), 262_143_984_375);
    /// assert_eq!(Timeout::DISABLED.as_nanos(), 0);
    /// assert_eq!(Timeout::from_raw(1).as_nanos(), 15_625);
    /// ```
    pub const fn as_nanos(&self) -> u64 {
        (self.bits as u64) * Timeout::RESOLUTION_NANOS
    }

    /// Get the timeout as a [`Duration`].
    ///
    /// # Example
    ///
    /// ```
    /// use core::time::Duration;
    /// use stm32wl_hal_subghz::Timeout;
    ///
    /// assert_eq!(
    ///     Timeout::MAX.as_duration(),
    ///     Duration::from_nanos(262_143_984_375)
    /// );
    /// assert_eq!(Timeout::DISABLED.as_duration(), Duration::from_nanos(0));
    /// assert_eq!(Timeout::from_raw(1).as_duration(), Timeout::RESOLUTION);
    /// ```
    pub const fn as_duration(&self) -> Duration {
        Duration::from_nanos((self.bits as u64) * Timeout::RESOLUTION_NANOS)
    }

    /// Get the bit value for the timeout.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::Timeout;
    ///
    /// assert_eq!(Timeout::from_raw(u32::MAX).into_bits(), 0x00FF_FFFF);
    /// assert_eq!(Timeout::from_raw(1).into_bits(), 1);
    /// ```
    pub const fn into_bits(self) -> u32 {
        self.bits
    }

    /// Get the byte value for the timeout.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wl_hal_subghz::Timeout;
    ///
    /// assert_eq!(Timeout::from_raw(u32::MAX).as_bytes(), [0xFF, 0xFF, 0xFF]);
    /// assert_eq!(Timeout::from_raw(1).as_bytes(), [0, 0, 1]);
    /// ```
    pub const fn as_bytes(self) -> [u8; 3] {
        [
            ((self.bits >> 16) & 0xFF) as u8,
            ((self.bits >> 8) & 0xFF) as u8,
            (self.bits & 0xFF) as u8,
        ]
    }
}

impl From<Timeout> for Duration {
    fn from(to: Timeout) -> Self {
        to.as_duration()
    }
}

impl From<Timeout> for [u8; 3] {
    fn from(to: Timeout) -> Self {
        to.as_bytes()
    }
}
