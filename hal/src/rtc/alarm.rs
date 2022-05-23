use chrono::Timelike;
use num_traits::FromPrimitive;

/// Alarm day.
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum AlarmDay {
    /// Day of the month.
    Day(u8),
    /// Weekday.
    Weekday(chrono::Weekday),
}

impl From<chrono::Weekday> for AlarmDay {
    #[inline]
    fn from(wd: chrono::Weekday) -> Self {
        Self::Weekday(wd)
    }
}

/// Alarm settings.
///
/// By default the date, hour, minute, and second are compared, but not the
/// sub-seconds.
///
/// # Example
///
/// Compare only seconds:
///
/// ```
/// use stm32wlxx_hal::rtc::Alarm;
///
/// const NO_COMPARE: Alarm = Alarm::DEFAULT
///     .set_days_mask(true)
///     .set_hours_mask(true)
///     .set_minutes_mask(true)
///     .set_seconds_mask(false);
/// # assert_eq!(NO_COMPARE.hours_mask(), true);
/// # assert_eq!(NO_COMPARE.minutes_mask(), true);
/// # assert_eq!(NO_COMPARE.seconds_mask(), false);
/// ```
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Alarm {
    pub(super) val: u32,
    pub(super) ss: u32,
    pub(super) ss_mask: u8,
}

impl From<chrono::NaiveTime> for Alarm {
    fn from(time: chrono::NaiveTime) -> Self {
        Self::DEFAULT
            .set_hours(time.hour() as u8)
            .set_minutes(time.minute() as u8)
            .set_seconds(time.second() as u8)
    }
}

impl From<Alarm> for chrono::NaiveTime {
    fn from(alarm: Alarm) -> Self {
        Self::from_hms(
            alarm.hours().into(),
            alarm.minutes().into(),
            alarm.seconds().into(),
        )
    }
}

const fn const_min(a: u8, b: u8) -> u32 {
    if a < b {
        a as u32
    } else {
        b as u32
    }
}

impl Default for Alarm {
    #[inline]
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl Alarm {
    /// Default alarm settings, as a constant.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// assert_eq!(Alarm::DEFAULT, Alarm::default());
    /// ```
    pub const DEFAULT: Self = Self {
        val: 0,
        ss: 0,
        ss_mask: 0,
    };

    /// Set the seconds value of the alarm.
    ///
    /// If the seconds value is greater than 59 it will be truncated.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::default();
    /// assert_eq!(alarm.seconds(), 0);
    ///
    /// let alarm: Alarm = alarm.set_seconds(31);
    /// assert_eq!(alarm.seconds(), 31);
    ///
    /// let alarm: Alarm = alarm.set_seconds(60);
    /// assert_eq!(alarm.seconds(), 59);
    /// ```
    #[must_use = "set_seconds returns a modified Alarm"]
    pub const fn set_seconds(mut self, seconds: u8) -> Self {
        let seconds: u32 = const_min(seconds, 59);

        let st: u32 = seconds / 10;
        let su: u32 = seconds % 10;

        self.val &= !0x7F;
        self.val |= st << 4 | su;
        self
    }

    /// Get the seconds value of the alarm.
    #[must_use]
    pub const fn seconds(&self) -> u8 {
        let st: u32 = self.val >> 4 & 0x7;
        let su: u32 = self.val & 0xF;
        (st * 10 + su) as u8
    }

    /// Set the alarm seconds mask.
    ///
    /// * `false`: Alarm is set if the seconds match.
    /// * `true`: Seconds are "do not care" in the alarm comparison.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::default();
    /// assert_eq!(alarm.seconds_mask(), false);
    ///
    /// let alarm: Alarm = alarm.set_seconds_mask(true);
    /// assert_eq!(alarm.seconds_mask(), true);
    ///
    /// let alarm: Alarm = alarm.set_seconds_mask(false);
    /// assert_eq!(alarm.seconds_mask(), false);
    /// ```
    #[must_use = "set_seconds_mask returns a modified Alarm"]
    pub const fn set_seconds_mask(mut self, mask: bool) -> Self {
        if mask {
            self.val |= 1 << 7;
        } else {
            self.val &= !(1 << 7);
        }
        self
    }

    /// Return `true` if the seconds mask is set.
    #[must_use]
    pub const fn seconds_mask(&self) -> bool {
        self.val & 1 << 7 != 0
    }

    /// Set the minutes value of the alarm.
    ///
    /// If the minutes value is greater than 59 it will be truncated.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::default();
    /// assert_eq!(alarm.minutes(), 0);
    ///
    /// let alarm: Alarm = alarm.set_minutes(31);
    /// assert_eq!(alarm.minutes(), 31);
    ///
    /// let alarm: Alarm = alarm.set_minutes(60);
    /// assert_eq!(alarm.minutes(), 59);
    /// ```
    #[must_use = "set_minutes returns a modified Alarm"]
    pub const fn set_minutes(mut self, minutes: u8) -> Self {
        let minutes: u32 = const_min(minutes, 59);

        let mnt: u32 = minutes / 10;
        let mnu: u32 = minutes % 10;

        self.val &= !0x7F << 8;
        self.val |= mnt << 12 | mnu << 8;
        self
    }

    /// Get the minutes value of the alarm.
    #[must_use]
    pub const fn minutes(&self) -> u8 {
        let mnt: u32 = self.val >> 12 & 0x7;
        let mnu: u32 = self.val >> 8 & 0xF;
        (mnt * 10 + mnu) as u8
    }

    /// Set the alarm minutes mask.
    ///
    /// * `false`: Alarm is set if the minutes match.
    /// * `true`: Minutes are "do not care" in the alarm comparison.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::default();
    /// assert_eq!(alarm.minutes_mask(), false);
    ///
    /// let alarm: Alarm = alarm.set_minutes_mask(true);
    /// assert_eq!(alarm.minutes_mask(), true);
    ///
    /// let alarm: Alarm = alarm.set_minutes_mask(false);
    /// assert_eq!(alarm.minutes_mask(), false);
    /// ```
    #[must_use = "set_minutes_mask returns a modified Alarm"]
    pub const fn set_minutes_mask(mut self, mask: bool) -> Self {
        if mask {
            self.val |= 1 << 15;
        } else {
            self.val &= !(1 << 15);
        }
        self
    }

    /// Return `true` if the minutes mask is set.
    #[must_use]
    pub const fn minutes_mask(&self) -> bool {
        self.val & 1 << 15 != 0
    }

    /// Set the hours value of the alarm.
    ///
    /// If the hours value is greater than 23 it will be truncated.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::default();
    /// assert_eq!(alarm.hours(), 0);
    ///
    /// let alarm: Alarm = alarm.set_hours(15);
    /// assert_eq!(alarm.hours(), 15);
    ///
    /// let alarm: Alarm = alarm.set_hours(24);
    /// assert_eq!(alarm.hours(), 23);
    /// ```
    #[must_use = "set_hours returns a modified Alarm"]
    pub const fn set_hours(mut self, hours: u8) -> Self {
        let hours: u32 = const_min(hours, 23);

        let ht: u32 = hours / 10;
        let hu: u32 = hours % 10;

        self.val &= !0x3F << 16;
        self.val |= ht << 20 | hu << 16;
        self
    }

    /// Get the hours value of the alarm.
    #[must_use]
    pub const fn hours(&self) -> u8 {
        let ht: u32 = self.val >> 20 & 0x3;
        let hu: u32 = self.val >> 16 & 0xF;
        (ht * 10 + hu) as u8
    }

    /// Set the alarm hours mask.
    ///
    /// * `false`: Alarm is set if the hours match.
    /// * `true`: Hours are "do not care" in the alarm comparison.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::default();
    /// assert_eq!(alarm.hours_mask(), false);
    ///
    /// let alarm: Alarm = alarm.set_hours_mask(true);
    /// assert_eq!(alarm.hours_mask(), true);
    ///
    /// let alarm: Alarm = alarm.set_hours_mask(false);
    /// assert_eq!(alarm.hours_mask(), false);
    /// ```
    #[must_use = "set_hours_mask returns a modified Alarm"]
    pub const fn set_hours_mask(mut self, mask: bool) -> Self {
        if mask {
            self.val |= 1 << 23;
        } else {
            self.val &= !(1 << 23);
        }
        self
    }

    /// Return `true` if the hours mask is set.
    #[must_use]
    pub const fn hours_mask(&self) -> bool {
        self.val & 1 << 23 != 0
    }

    /// Set the day for the alarm.
    ///
    /// This is mutually exclusive with [`set_weekday`](Self::set_weekday).
    ///
    /// If the day value is greater than 31 it will be truncated.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::{Alarm, AlarmDay};
    ///
    /// let alarm: Alarm = Alarm::default();
    /// assert_eq!(alarm.day(), AlarmDay::Day(0));
    ///
    /// let alarm: Alarm = alarm.set_days(14);
    /// assert_eq!(alarm.day(), AlarmDay::Day(14));
    ///
    /// let alarm: Alarm = alarm.set_days(32);
    /// assert_eq!(alarm.day(), AlarmDay::Day(31));
    /// ```
    #[must_use = "set_days returns a modified Alarm"]
    pub const fn set_days(mut self, day: u8) -> Self {
        let day: u32 = const_min(day, 31);

        let dt: u32 = day / 10;
        let du: u32 = day % 10;

        self.val &= !(0x7F << 24);
        self.val |= dt << 28 | du << 24;
        self
    }

    /// Set the weekday for the alarm.
    ///
    /// This is mutually exclusive with [`set_days`](Self::set_days).
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::{
    ///     chrono::Weekday,
    ///     rtc::{Alarm, AlarmDay},
    /// };
    ///
    /// let alarm: Alarm = Alarm::default();
    /// assert_eq!(alarm.day(), AlarmDay::Day(0));
    ///
    /// let alarm: Alarm = alarm.set_weekday(Weekday::Mon);
    /// assert_eq!(alarm.day(), AlarmDay::Weekday(Weekday::Mon));
    ///
    /// let alarm: Alarm = alarm.set_weekday(Weekday::Tue);
    /// assert_eq!(alarm.day(), AlarmDay::Weekday(Weekday::Tue));
    ///
    /// let alarm: Alarm = alarm.set_weekday(Weekday::Wed);
    /// assert_eq!(alarm.day(), AlarmDay::Weekday(Weekday::Wed));
    ///
    /// let alarm: Alarm = alarm.set_weekday(Weekday::Thu);
    /// assert_eq!(alarm.day(), AlarmDay::Weekday(Weekday::Thu));
    ///
    /// let alarm: Alarm = alarm.set_weekday(Weekday::Fri);
    /// assert_eq!(alarm.day(), AlarmDay::Weekday(Weekday::Fri));
    ///
    /// let alarm: Alarm = alarm.set_weekday(Weekday::Sat);
    /// assert_eq!(alarm.day(), AlarmDay::Weekday(Weekday::Sat));
    ///
    /// let alarm: Alarm = alarm.set_weekday(Weekday::Sun);
    /// assert_eq!(alarm.day(), AlarmDay::Weekday(Weekday::Sun));
    /// # let alarm: Alarm = alarm.set_days(9);
    /// # assert_eq!(alarm.day(), AlarmDay::Day(9));
    /// ```
    #[must_use = "set_weekday returns a modified Alarm"]
    pub fn set_weekday(mut self, wd: chrono::Weekday) -> Self {
        self.val &= !(0xF << 24);
        self.val |= 1 << 30 | wd.number_from_monday() << 24;
        self
    }

    /// Get the weekday or day of the alarm.
    #[must_use]
    pub fn day(&self) -> AlarmDay {
        if self.val & 1 << 30 != 0 {
            let wd: u32 = (self.val >> 24) & 0xF;
            AlarmDay::Weekday(chrono::Weekday::from_u32(wd.saturating_sub(1)).unwrap())
        } else {
            let dt: u32 = (self.val >> 28) & 0x3;
            let du: u32 = (self.val >> 24) & 0xF;

            AlarmDay::Day((dt * 10 + du) as u8)
        }
    }

    /// Set the alarm day / weekday mask.
    ///
    /// * `false`: Alarm is set if the days / weekdays match.
    /// * `true`: Days / weekdays are "do not care" in the alarm comparison.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::default();
    /// assert_eq!(alarm.days_mask(), false);
    ///
    /// let alarm: Alarm = alarm.set_days_mask(true);
    /// assert_eq!(alarm.days_mask(), true);
    ///
    /// let alarm: Alarm = alarm.set_days_mask(false);
    /// assert_eq!(alarm.days_mask(), false);
    /// ```
    #[must_use = "set_days_mask returns a modified Alarm"]
    pub const fn set_days_mask(mut self, mask: bool) -> Self {
        if mask {
            self.val |= 1 << 31;
        } else {
            self.val &= !(1 << 31);
        }
        self
    }

    /// Return `true` if the day mask is set.
    #[must_use]
    pub const fn days_mask(&self) -> bool {
        self.val & 1 << 31 != 0
    }

    /// Set the sub-seconds value.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::default();
    /// assert_eq!(alarm.subseconds(), 0);
    ///
    /// let alarm: Alarm = alarm.set_subseconds(1);
    /// assert_eq!(alarm.subseconds(), 1);
    ///
    /// let alarm: Alarm = alarm.set_subseconds(u32::MAX);
    /// assert_eq!(alarm.subseconds(), u32::MAX);
    /// ```
    #[must_use = "set_subseconds returns a modified Alarm"]
    pub const fn set_subseconds(mut self, ss: u32) -> Self {
        self.ss = ss;
        self
    }

    /// Get the sub-seconds value.
    #[must_use]
    pub const fn subseconds(&self) -> u32 {
        self.ss
    }

    /// Set the sub-seconds mask.
    ///
    /// * 0: No comparison on sub seconds for the alarm.
    ///   The alarm is set when the seconds unit is incremented
    ///   (assuming that the rest of the fields match).
    /// * 1: SS\[31:1\] are "do not care" in the alarm comparison.
    ///   Only SS\[0\] is compared.
    /// * 2: SS\[31:2\] are "do not care" in the alarm comparison.
    ///   Only SS\[1:0\] are compared.
    /// * ...
    /// * 31: SS\[31\] is "do not care" in the alarm comparison.
    ///   Only SS\[30:0\] are compared.
    /// * 32 to 63: All 32 SS bits are compared and must match to activate the
    ///   alarm.
    ///
    /// Values greater than 63 will be truncated.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::default();
    /// assert_eq!(alarm.subseconds_mask(), 0);
    ///
    /// let alarm: Alarm = alarm.set_subseconds_mask(17);
    /// assert_eq!(alarm.subseconds_mask(), 17);
    ///
    /// let alarm: Alarm = alarm.set_subseconds_mask(128);
    /// assert_eq!(alarm.subseconds_mask(), 63);
    /// ```
    #[must_use = "set_subseconds_mask returns a modified Alarm"]
    pub const fn set_subseconds_mask(mut self, mask: u8) -> Self {
        if mask > 0x3F {
            self.ss_mask = 0x3F
        } else {
            self.ss_mask = mask;
        }
        self
    }

    /// Get the sub-seconds mask.
    #[must_use]
    pub const fn subseconds_mask(&self) -> u8 {
        self.ss_mask
    }
}

#[cfg(test)]
mod tests {
    use super::Alarm;
    use chrono::NaiveTime;

    #[test]
    fn chrono_convert() {
        let ndt: NaiveTime = NaiveTime::from_hms(12, 34, 56);
        let alarm: Alarm = ndt.into();
        assert_eq!(alarm.hours(), 12);
        assert_eq!(alarm.minutes(), 34);
        assert_eq!(alarm.seconds(), 56);
        assert_eq!(NaiveTime::from(alarm), ndt);
    }
}
