//! Real-time clock.

use crate::{pac, rcc::lsi_hz};
use chrono::{Datelike, NaiveDate, NaiveDateTime, NaiveTime, Timelike};
use num_traits::FromPrimitive;
use pac::{
    rcc::{
        bdcr::RTCSEL_A,
        csr::LSIPRE_A::{DIV1, DIV128},
    },
    rtc::cr::WUCKSEL_A,
};

/// RTC clock selection
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Clk {
    /// LSE oscillator clock selected.
    Lse = RTCSEL_A::LSE as u8,
    /// LSI oscillator clock selected.
    Lsi = RTCSEL_A::LSI as u8,
    /// HSE32 oscillator clock divided by 32 selected.
    Hse = RTCSEL_A::HSE32 as u8,
}

/// Status (interrupt) masks.
///
/// Used for [`Rtc::clear_status`].
pub mod stat {
    /// SSR underflow flag
    pub const SSRU: u32 = 1 << 6;
    /// Internal timestamp flag
    pub const ITS: u32 = 1 << 5;
    /// Timestamp overflow flag
    pub const TSOV: u32 = 1 << 4;
    /// Timestamp flag
    pub const TS: u32 = 1 << 3;
    /// Wakeup timer flag
    pub const WUT: u32 = 1 << 2;
    /// Alarm B flag
    pub const ALRB: u32 = 1 << 1;
    /// Alarm A flag
    pub const ALRA: u32 = 1 << 0;

    /// All status flags.
    pub const ALL: u32 = SSRU | ITS | TSOV | TS | WUT | ALRB | ALRA;
}

/// Alarm day.
#[derive(Debug)]
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
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Alarm {
    val: u32,
    ss: u32,
}

impl From<chrono::NaiveTime> for Alarm {
    fn from(time: chrono::NaiveTime) -> Self {
        let hour: u32 = time.hour();
        let ht: u32 = hour / 10;
        let hu: u32 = hour % 10;

        let minute: u32 = time.minute();
        let mnt: u32 = minute / 10;
        let mnu: u32 = minute % 10;

        let second: u32 = time.second();
        let st: u32 = second / 10;
        let su: u32 = second % 10;

        Self {
            val: ht << 20 | hu << 16 | mnt << 12 | mnu << 8 | st << 4 | su,
            ss: 0,
        }
    }
}

const fn const_min(a: u8, b: u8) -> u32 {
    if a < b {
        a as u32
    } else {
        b as u32
    }
}

impl Alarm {
    pub const ZERO: Self = Self { val: 0, ss: 0 };

    /// Set the seconds value of the alarm.
    ///
    /// If the seconds value is greater than 59 it will be truncated.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::ZERO;
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

    /// Set the alarm second mask.
    ///
    /// * `true`: Alarm is set if the seconds match.
    /// * `false`: Seconds are "do not care" in the alarm comparison.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::ZERO;
    /// assert_eq!(alarm.second_mask(), false);
    ///
    /// let alarm: Alarm = alarm.set_second_mask(true);
    /// assert_eq!(alarm.second_mask(), true);
    ///
    /// let alarm: Alarm = alarm.set_second_mask(false);
    /// assert_eq!(alarm.second_mask(), false);
    /// ```
    #[must_use = "set_second_mask returns a modified Alarm"]
    pub const fn set_second_mask(mut self, mask: bool) -> Self {
        if mask {
            self.val |= 1 << 7;
        } else {
            self.val &= !(1 << 7);
        }
        self
    }

    /// Return `true` if the second mask is set.
    #[must_use]
    pub const fn second_mask(&self) -> bool {
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
    /// let alarm: Alarm = Alarm::ZERO;
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

    /// Set the alarm minute mask.
    ///
    /// * `true`: Alarm is set if the minutes match.
    /// * `false`: Minutes are "do not care" in the alarm comparison.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::ZERO;
    /// assert_eq!(alarm.minute_mask(), false);
    ///
    /// let alarm: Alarm = alarm.set_minute_mask(true);
    /// assert_eq!(alarm.minute_mask(), true);
    ///
    /// let alarm: Alarm = alarm.set_minute_mask(false);
    /// assert_eq!(alarm.minute_mask(), false);
    /// ```
    #[must_use = "set_minute_mask returns a modified Alarm"]
    pub const fn set_minute_mask(mut self, mask: bool) -> Self {
        if mask {
            self.val |= 1 << 15;
        } else {
            self.val &= !(1 << 15);
        }
        self
    }

    /// Return `true` if the minute mask is set.
    #[must_use]
    pub const fn minute_mask(&self) -> bool {
        self.val & 1 << 15 != 0
    }

    /// Set the alarm hour mask.
    ///
    /// * `true`: Alarm is set if the hours match.
    /// * `false`: Hours are "do not care" in the alarm comparison.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::rtc::Alarm;
    ///
    /// let alarm: Alarm = Alarm::ZERO;
    /// assert_eq!(alarm.hour_mask(), false);
    ///
    /// let alarm: Alarm = alarm.set_hour_mask(true);
    /// assert_eq!(alarm.hour_mask(), true);
    ///
    /// let alarm: Alarm = alarm.set_hour_mask(false);
    /// assert_eq!(alarm.hour_mask(), false);
    /// ```
    #[must_use = "set_hour_mask returns a modified Alarm"]
    pub const fn set_hour_mask(mut self, mask: bool) -> Self {
        if mask {
            self.val |= 1 << 23;
        } else {
            self.val &= !(1 << 23);
        }
        self
    }

    /// Return `true` if the hour mask is set.
    #[must_use]
    pub const fn hour_mask(&self) -> bool {
        self.val & 1 << 23 != 0
    }

    #[must_use = "set_day returns a modified Alarm"]
    pub const fn set_day(mut self, day: u8) -> Self {
        let day: u32 = const_min(day, 31);

        let dt: u32 = day / 10;
        let du: u32 = day % 10;

        self.val &= !(0x7F << 24);
        self.val |= dt << 28 | du << 24;
        self
    }

    #[must_use = "set_weekday returns a modified Alarm"]
    pub fn set_weekday(mut self, wd: chrono::Weekday) -> Self {
        self.val &= !(0xF << 24);
        self.val |= 1 << 30 | wd.number_from_monday() << 24;
        self
    }

    #[must_use]
    pub fn day(&self) -> AlarmDay {
        if 1 << 30 != 0 {
            let wd: u32 = (self.val >> 24) & 0xF;
            AlarmDay::Weekday(chrono::Weekday::from_u32(wd.saturating_sub(1)).unwrap())
        } else {
            let dt: u32 = (self.val >> 28) & 0x3;
            let du: u32 = (self.val >> 24) & 0xF;

            AlarmDay::Day((dt * 10 + du) as u8)
        }
    }

    #[must_use = "set_day_mask returns a modified Alarm"]
    pub const fn set_day_mask(mut self, mask: bool) -> Self {
        if mask {
            self.val |= 1 << 31;
        } else {
            self.val &= !(1 << 31);
        }
        self
    }
}

/// Real-time clock driver.
#[derive(Debug)]
pub struct Rtc {
    rtc: pac::RTC,
}

impl Rtc {
    /// Create a new real-time clock driver.
    ///
    /// This will **not** setup the source clock.
    ///
    /// # Safety
    ///
    /// This function _could_ be considered unsafe because it is not a
    /// pure function.
    /// The RTC is in the backup domain; system resets will not reset the RTC.
    /// You are responsible for resetting the backup domain if required.
    ///
    /// # Panics
    ///
    /// * (debug) clock source is not ready.
    ///
    /// # Example
    ///
    /// LSE clock source (this depends on HW, example valid for NUCLEO board):
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rcc::pulse_reset_backup_domain,
    ///     rtc::{Clk, Rtc},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// unsafe { pulse_reset_backup_domain(&mut dp.RCC, &mut dp.PWR) };
    /// dp.PWR.cr1.modify(|_, w| w.dbp().enabled());
    /// dp.RCC.bdcr.modify(|_, w| w.lseon().on());
    /// while dp.RCC.bdcr.read().lserdy().is_not_ready() {}
    ///
    /// let rtc: Rtc = Rtc::new(dp.RTC, Clk::Lse, &mut dp.PWR, &mut dp.RCC);
    /// ```
    ///
    /// LSI clock source:
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rcc::{enable_lsi, pulse_reset_backup_domain},
    ///     rtc::{Clk, Rtc},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// unsafe { pulse_reset_backup_domain(&mut dp.RCC, &mut dp.PWR) };
    /// enable_lsi(&mut dp.RCC);
    ///
    /// let rtc: Rtc = Rtc::new(dp.RTC, Clk::Lsi, &mut dp.PWR, &mut dp.RCC);
    /// ```
    ///
    /// HSE clock source (this depends on HW, example valid for NUCLEO board):
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{
    ///     pac,
    ///     rcc::pulse_reset_backup_domain,
    ///     rtc::{Clk, Rtc},
    /// };
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// unsafe { pulse_reset_backup_domain(&mut dp.RCC, &mut dp.PWR) };
    /// dp.RCC
    ///     .cr
    ///     .modify(|_, w| w.hseon().enabled().hsebyppwr().vddtcxo());
    /// while dp.RCC.cr.read().hserdy().is_not_ready() {}
    ///
    /// let rtc: Rtc = Rtc::new(dp.RTC, Clk::Hse, &mut dp.PWR, &mut dp.RCC);
    /// ```
    pub fn new(rtc: pac::RTC, clk: Clk, pwr: &mut pac::PWR, rcc: &mut pac::RCC) -> Rtc {
        pwr.cr1.modify(|_, w| w.dbp().enabled());

        match clk {
            Clk::Lse => {
                debug_assert!(rcc.bdcr.read().lserdy().is_ready());
                rcc.bdcr.modify(|_, w| w.rtcsel().lse().rtcen().enabled());
            }
            Clk::Lsi => {
                debug_assert!(rcc.csr.read().lsirdy().is_ready());
                rcc.bdcr.modify(|_, w| w.rtcsel().lsi().rtcen().enabled());
            }
            Clk::Hse => {
                debug_assert!(rcc.cr.read().hserdy().is_ready());
                rcc.bdcr.modify(|_, w| w.rtcsel().hse32().rtcen().enabled());
            }
        }

        Self::apbclken(rcc);

        let mut rtc: Rtc = Rtc { rtc };
        rtc.disable_write_protect();
        rtc.configure_prescaler(rcc);

        rtc
    }

    /// Create a new real-time clock driver preserving backup domain values.
    ///
    /// Unlike [`new`](Self::new) this will enable the LSE clock source if not
    /// already enabled.  This function assumes the LSE clock source will be
    /// used because it is the only clock source that is preserved in the
    /// shutdown mode.
    ///
    /// The RTC calendar may not be initialized, this can occur if this function
    /// is called after power loss or after a backup domain reset.
    ///
    /// # Safety
    ///
    /// 1. This function relies on global hardware state in the backup domain.
    ///    The backup domain is **not** reset with normal system resets.
    ///    Reset the backup domain before calling this function if determinism
    ///    is required.
    pub unsafe fn renew(rtc: pac::RTC, pwr: &mut pac::PWR, rcc: &mut pac::RCC) -> Rtc {
        pwr.cr1.modify(|_, w| w.dbp().enabled());
        Self::apbclken(rcc);

        let bdcr = rcc.bdcr.read();
        if bdcr.rtcsel().is_lse() && bdcr.rtcen().is_enabled() && bdcr.lseon().is_on() {
            while rcc.bdcr.read().lserdy().is_not_ready() {}
            Rtc { rtc }
        } else {
            rcc.bdcr.modify(|_, w| w.lseon().on());
            while rcc.bdcr.read().lserdy().is_not_ready() {}
            rcc.bdcr.modify(|_, w| w.rtcsel().lse().rtcen().enabled());

            let mut rtc: Rtc = Rtc { rtc };
            rtc.disable_write_protect();
            rtc.configure_prescaler(rcc);
            rtc
        }
    }

    #[inline(always)]
    fn apbclken(rcc: &mut pac::RCC) {
        #[cfg(not(feature = "stm32wl5x_cm0p"))]
        rcc.apb1enr1.modify(|_, w| w.rtcapben().set_bit());
        #[cfg(feature = "stm32wl5x_cm0p")]
        rcc.c2apb1enr1.modify(|_, w| w.rtcapben().set_bit());
    }

    /// Source clock frequency in hertz.
    #[inline]
    pub fn hz(rcc: &pac::RCC) -> u32 {
        match rcc.bdcr.read().rtcsel().variant() {
            RTCSEL_A::NOCLOCK => 0,
            RTCSEL_A::LSE => 32_768,
            RTCSEL_A::LSI => lsi_hz(rcc).into(),
            RTCSEL_A::HSE32 => 1_000_000,
        }
    }

    /// Read the RTC status (interrupt) register.
    #[inline]
    pub fn status() -> pac::rtc::sr::R {
        // saftey: atomic read with no side-effects
        unsafe { (*pac::RTC::PTR).sr.read() }
    }

    /// Read the RTC masked status (interrupt) register.
    #[inline]
    pub fn masked_status() -> pac::rtc::misr::R {
        // saftey: atomic read with no side-effects
        unsafe { (*pac::RTC::PTR).misr.read() }
    }

    /// Clear status (interrupt) flags.
    ///
    /// Status flag masks can be found in [`stat`].
    #[inline]
    pub fn clear_status(mask: u32) {
        // safety: mask is masked with valid register fields
        unsafe { (*pac::RTC::PTR).scr.write(|w| w.bits(mask & stat::ALL)) }
    }

    // configure prescaler for a 1Hz clock
    //
    // RM0453 Rev 2 page 996:
    // When both prescalers are used, it is recommended to configure the
    // asynchronous prescaler to a high value to minimize consumption.
    //
    // async is 7 bit (128 max)
    // sync is 15-bit (32_768 max)
    fn configure_prescaler(&mut self, rcc: &mut pac::RCC) {
        let (a_pre, s_pre): (u8, u16) = match rcc.bdcr.read().rtcsel().variant() {
            RTCSEL_A::NOCLOCK => unreachable!(),
            // (127 + 1) × (255 + 1) = 32_768 Hz
            RTCSEL_A::LSE => (127, 255),
            RTCSEL_A::LSI => match rcc.csr.read().lsipre().variant() {
                // (99 + 1) × (319 + 1) = 32_000 Hz
                DIV1 => (99, 319),
                // (124 + 1) × (1 + 1) = 250 Hz
                DIV128 => (124, 1),
            },
            // (99 + 1) × (9_999 + 1) = 1_000_000 Hz
            RTCSEL_A::HSE32 => (99, 9_999),
        };

        // enter initialization mode
        self.rtc.icsr.modify(|_, w| w.init().init_mode());
        while self.rtc.icsr.read().initf().is_not_allowed() {}

        // enable shadow register bypass
        self.rtc.cr.modify(|_, w| w.bypshad().set_bit());

        self.rtc
            .prer
            .write(|w| w.prediv_s().bits(s_pre).prediv_a().bits(a_pre));

        // exit initialization mode
        self.rtc.icsr.modify(|_, w| w.init().free_running_mode())
    }

    /// Set the date and time.
    ///
    /// The value will take some duration to apply after this function returns:
    ///
    /// * LPCAL=0: the counting restarts after 4 RTCCLK clock cycles
    /// * LPCAL=1: the counting restarts after up to 2 RTCCLK + 1 ck_apre
    ///
    /// # Panics
    ///
    /// * Year is greater than or equal to 2100.
    /// * Year is less than 2000.
    /// * Backup domain write protection is enabled.
    pub fn set_date_time(&mut self, date_time: chrono::NaiveDateTime) {
        // safety: atomic read with no side effects
        assert!(unsafe { (*pac::PWR::PTR).cr1.read().dbp().bit_is_set() });

        // enter initialization mode
        self.rtc.icsr.modify(|_, w| w.init().init_mode());
        while self.rtc.icsr.read().initf().is_not_allowed() {}

        let hour: u8 = date_time.hour() as u8;
        let ht: u8 = hour / 10;
        let hu: u8 = hour % 10;

        let minute: u8 = date_time.minute() as u8;
        let mnt: u8 = minute / 10;
        let mnu: u8 = minute % 10;

        let second: u8 = date_time.second() as u8;
        let st: u8 = second / 10;
        let su: u8 = second % 10;

        self.rtc.tr.write(|w| {
            w.pm().clear_bit(); // 24h format
            w.ht().bits(ht);
            w.hu().bits(hu);
            w.mnt().bits(mnt);
            w.mnu().bits(mnu);
            w.st().bits(st);
            w.su().bits(su)
        });

        let year: i32 = date_time.year();
        assert!((2000..2100).contains(&year));
        let yt: u8 = ((year - 2000) / 10) as u8;
        let yu: u8 = ((year - 2000) % 10) as u8;

        let wdu: u8 = date_time.weekday().number_from_monday() as u8;

        let month: u8 = date_time.month() as u8;
        let mt: bool = month > 9;
        let mu: u8 = month % 10;

        let day: u8 = date_time.day() as u8;
        let dt: u8 = day / 10;
        let du: u8 = day % 10;

        self.rtc.dr.write(|w| unsafe {
            w.yt().bits(yt);
            w.yu().bits(yu);
            w.wdu().bits(wdu);
            w.mt().bit(mt);
            w.mu().bits(mu);
            w.dt().bits(dt);
            w.du().bits(du)
        });

        // exit initialization mode
        self.rtc.icsr.modify(|_, w| w.init().free_running_mode());
    }

    /// Returns `None` if the calendar is uninitialized.
    #[inline]
    pub fn calendar_initialized(&self) -> Option<()> {
        use pac::rtc::icsr::INITS_A;
        match self.rtc.icsr.read().inits().variant() {
            INITS_A::NOTINITALIZED => None,
            INITS_A::INITALIZED => Some(()),
        }
    }

    /// Calendar Date
    ///
    /// Returns `None` if the calendar has not been initialized.
    pub fn date(&self) -> Option<NaiveDate> {
        self.calendar_initialized()?;
        let data = self.rtc.dr.read();
        let year: i32 = 2000 + (data.yt().bits() as i32) * 10 + (data.yu().bits() as i32);
        let month: u8 = data.mt().bits() as u8 * 10 + data.mu().bits();
        let day: u8 = data.dt().bits() * 10 + data.du().bits();
        NaiveDate::from_ymd_opt(year, month.into(), day.into())
    }

    fn ss_to_us(&self, ss: u32) -> u32 {
        // running in BCD mode, only 15:0 are used
        let ss: u32 = ss & 0xFFFF;

        let pre_s: u32 = self.rtc.prer.read().prediv_s().bits().into();
        // RM0453 Rev 2 page 1012
        // SS can be larger than PREDIV_S only after a shift operation.
        // In that case, the correct time/date is one second less than as
        // indicated by RTC_TR/RTC_DR.
        debug_assert!(ss <= pre_s);

        // RM0453 Rev 2 page 1012
        // SS[15:0] is the value in the synchronous prescaler counter.
        // The fraction of a second is given by the formula below:
        // Second fraction = (PREDIV_S - SS) / (PREDIV_S + 1)
        (((pre_s - ss) * 100_000) / (pre_s + 1)) * 10
    }

    /// Current Time
    ///
    /// Returns `None` if the calendar has not been initialized.
    pub fn time(&self) -> Option<NaiveTime> {
        loop {
            self.calendar_initialized()?;
            let ss: u32 = self.rtc.ssr.read().ss().bits();
            let data = self.rtc.tr.read();

            // If an RTCCLK edge occurs during read we may see inconsistent values
            // so read ssr again and see if it has changed
            // see RM0453 Rev 2 32.3.10 page 1002 "Reading the calendar"
            let ss_after: u32 = self.rtc.ssr.read().ss().bits();
            if ss == ss_after {
                let mut hour: u8 = data.ht().bits() * 10 + data.hu().bits();
                if data.pm().is_pm() {
                    hour += 12;
                }
                let minute: u8 = data.mnt().bits() * 10 + data.mnu().bits();
                let second: u8 = data.st().bits() * 10 + data.su().bits();
                let micro: u32 = self.ss_to_us(ss);

                return NaiveTime::from_hms_micro_opt(
                    hour as u32,
                    minute as u32,
                    second as u32,
                    micro,
                );
            }
        }
    }

    /// Calendar Date and Time
    ///
    /// Returns `None` if the calendar has not been initialized.
    pub fn date_time(&self) -> Option<NaiveDateTime> {
        loop {
            self.calendar_initialized()?;
            let ss: u32 = self.rtc.ssr.read().ss().bits();
            let dr = self.rtc.dr.read();
            let tr = self.rtc.tr.read();

            // If an RTCCLK edge occurs during a read we may see inconsistent values
            // so read ssr again and see if it has changed
            // see RM0453 Rev 2 32.3.10 page 1002 "Reading the calendar"
            let ss_after: u32 = self.rtc.ssr.read().ss().bits();
            if ss == ss_after {
                let year: i32 = 2000 + (dr.yt().bits() as i32) * 10 + (dr.yu().bits() as i32);
                let month: u8 = dr.mt().bits() as u8 * 10 + dr.mu().bits();
                let day: u8 = dr.dt().bits() * 10 + dr.du().bits();

                let date: NaiveDate = NaiveDate::from_ymd_opt(year, month as u32, day as u32)?;

                let mut hour: u8 = tr.ht().bits() * 10 + tr.hu().bits();
                if tr.pm().is_pm() {
                    hour += 12;
                }
                let minute: u8 = tr.mnt().bits() * 10 + tr.mnu().bits();
                let second: u8 = tr.st().bits() * 10 + tr.su().bits();
                let micro: u32 = self.ss_to_us(ss);

                let time = NaiveTime::from_hms_micro_opt(
                    hour as u32,
                    minute as u32,
                    second as u32,
                    micro,
                )?;

                return Some(date.and_time(time));
            }
        }
    }

    /// Setup the periodic wakeup timer for `sec + 1` seconds.
    ///
    /// `sec` can only go up to 2<sup>17</sup> (36 hours), values greater than
    /// this will be set to the maximum.
    ///
    /// # Example
    ///
    /// Setup the wakeup timer to go off in 1 hour, without interrupts.
    ///
    /// ```no_run
    /// # use stm32wlxx_hal::{pac, rtc};
    /// # let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// # let mut rtc = rtc::Rtc::new(dp.RTC, rtc::Clk::Lse, &mut dp.PWR, &mut dp.RCC);
    /// rtc.setup_wakeup_timer(3599, false);
    /// ```
    pub fn setup_wakeup_timer(&mut self, sec: u32, irq_en: bool) {
        // The following sequence is required to configure or change the wakeup
        // timer auto-reload value (WUT[15:0] in RTC_WUTR):

        // 1. Clear WUTE in RTC_CR to disable the wakeup timer.
        self.rtc.cr.modify(|_, w| w.wute().clear_bit());
        // 2. Poll WUTWF until it is set in RTC_ICSR to make sure the access to
        // wakeup auto-reload counter and to WUCKSEL[2:0] bits is allowed.
        // This step must be skipped in calendar initialization mode.
        // If WUCKSEL[2] = 0:
        //  WUTWF is set around 1 ck_wut + 1 RTCCLK cycles after WUTE bit is cleared.
        // If WUCKSEL[2] = 1:
        //  WUTWF is set up to 1 ck_apre + 1 RTCCLK cycles after WUTE bit is cleared.
        while self.rtc.icsr.read().wutwf().bit_is_clear() {}
        // 3. Program the wakeup auto-reload value WUT[15:0], WUTOCLR[15:0]
        // and the wakeup clock selection (WUCKSEL[2:0] bits in RTC_CR).
        // Set WUTE in RTC_CR to enable the timer again.
        // The wakeup timer restarts down-counting.
        // If WUCKSEL[2] = 0:
        //  WUTWF is cleared around 1 ck_wut + 1 RTCCLK cycles after WUTE bit is set.
        // If WUCKSEL[2] = 1:
        //  WUTWF is cleared up to 1 ck_apre + 1 RTCCLK cycles after WUTE bit is set.
        let (wucksel, sec): (WUCKSEL_A, u16) = match u16::try_from(sec) {
            Ok(sec) => (WUCKSEL_A::CLOCKSPARE, sec),
            Err(_) => (
                WUCKSEL_A::CLOCKSPAREWITHOFFSET,
                u16::try_from(sec - (1 << 16) - 1).unwrap_or(u16::MAX),
            ),
        };

        self.rtc
            .cr
            .modify(|_, w| w.wucksel().variant(wucksel).wutie().bit(irq_en));
        self.rtc.wutr.write(|w| w.wut().bits(sec).wutoclr().bits(0));
        self.rtc.cr.modify(|_, w| w.wute().set_bit());
    }

    pub fn set_alarm_a(&mut self, alarm: Alarm, irq_en: bool) {
        self.rtc.cr.modify(|_, w| w.alrae().clear_bit());
        self.rtc.alrmar.write(|w| unsafe { w.bits(alarm.val) });
        self.rtc
            .cr
            .modify(|_, w| w.alrae().set_bit().alraie().bit(irq_en));
    }

    pub fn set_alarm_b(&mut self, alarm: Alarm, irq_en: bool) {
        self.rtc.cr.modify(|_, w| w.alrbe().clear_bit());
        self.rtc.alrmbr.write(|w| unsafe { w.bits(alarm.val) });
        self.rtc
            .cr
            .modify(|_, w| w.alrbe().set_bit().alrbie().bit(irq_en));
    }

    /// Disable the wakeup timer.
    #[inline]
    pub fn disable_wakeup_timer(&mut self) {
        self.rtc.cr.modify(|_, w| w.wute().clear_bit());
    }

    /// Disable the RTC write protection.
    #[inline]
    pub fn disable_write_protect(&mut self) {
        self.rtc.wpr.write(|w| w.key().deactivate1());
        self.rtc.wpr.write(|w| w.key().deactivate2());
    }

    /// Enable the RTC write protection.
    ///
    /// # Safety
    ///
    /// * You must call [`disable_write_protect`] before using any other
    ///   `&mut self` RTC method.
    ///
    /// [`disable_write_protect`]: Self::disable_write_protect
    #[inline]
    pub unsafe fn enable_write_protect(&mut self) {
        self.rtc.wpr.write(|w| w.key().activate());
    }
}
