//! Real-time clock

mod alarm;

pub use alarm::{Alarm, AlarmDay};

use crate::{pac, rcc::lsi_hz};
use chrono::{Datelike, NaiveDate, NaiveDateTime, NaiveTime, Timelike};
use core::cmp::min;
use pac::{
    rcc::{
        bdcr::RTCSEL_A,
        csr::LSIPRE_A::{Div1, Div128},
    },
    rtc::cr::WUCKSEL_A,
};

/// RTC clock selection
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Clk {
    /// LSE oscillator clock selected.
    Lse = RTCSEL_A::Lse as u8,
    /// LSI oscillator clock selected.
    Lsi = RTCSEL_A::Lsi as u8,
    /// HSE32 oscillator clock divided by 32 selected.
    Hse = RTCSEL_A::Hse32 as u8,
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

    /// Alarm A & B flags.
    pub const ALR_ALL: u32 = ALRA | ALRB;
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
    /// HSE clock source (this depends on your hardware):
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
            RTCSEL_A::NoClock => 0,
            RTCSEL_A::Lse => 32_768,
            RTCSEL_A::Lsi => lsi_hz(rcc).into(),
            RTCSEL_A::Hse32 => 1_000_000,
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
            RTCSEL_A::NoClock => unreachable!(),
            // (127 + 1) × (255 + 1) = 32_768 Hz
            RTCSEL_A::Lse => (127, 255),
            RTCSEL_A::Lsi => match rcc.csr.read().lsipre().variant() {
                // (99 + 1) × (319 + 1) = 32_000 Hz
                Div1 => (99, 319),
                // (124 + 1) × (1 + 1) = 250 Hz
                Div128 => (124, 1),
            },
            // (99 + 1) × (9_999 + 1) = 1_000_000 Hz
            RTCSEL_A::Hse32 => (99, 9_999),
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
        self.rtc.icsr.read().inits().is_initalized().then(|| ())
    }

    /// Calibrate the RTC using the low-power mode.
    ///
    /// This does not poll for completion, use [`recalibration_pending`] if you
    /// need to wait for completion.
    ///
    /// The calibration argument is in units of 0.9537 ppm.
    /// The calibration range is -487.1 ppm to +488.5 ppm (-511 to 512), values
    /// outside of this range will saturate.
    ///
    /// [`recalibration_pending`]: Self::recalibration_pending
    pub fn calibrate_lp(&mut self, calibration: i16) {
        while self.recalibration_pending() {}
        let (calp, calm): (bool, u16) = if let Ok(calibration_pos) = u16::try_from(calibration) {
            (true, 512_u16.saturating_sub(calibration_pos))
        } else {
            (false, min(calibration.abs_diff(0), 511))
        };
        self.rtc
            .calr
            .write(|w| w.calp().bit(calp).lpcal().set_bit().calm().bits(calm))
    }

    /// Returns `true` if recalibration is pending.
    #[inline]
    pub fn recalibration_pending(&self) -> bool {
        self.rtc.icsr.read().recalpf().is_pending()
    }

    /// Calendar Date
    ///
    /// Returns `None` if the calendar has not been initialized.
    pub fn date(&self) -> Option<NaiveDate> {
        self.calendar_initialized()?;
        let data = self.rtc.dr.read();
        let year: i32 = 2000 + (data.yt().bits() as i32) * 10 + (data.yu().bits() as i32);
        let month: u8 = data.mt().bit() as u8 * 10 + data.mu().bits();
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
                let month: u8 = dr.mt().bit() as u8 * 10 + dr.mu().bits();
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
            Ok(sec) => (WUCKSEL_A::ClockSpare, sec),
            Err(_) => (
                WUCKSEL_A::ClockSpareWithOffset,
                u16::try_from(sec - (1 << 16) - 1).unwrap_or(u16::MAX),
            ),
        };

        self.rtc
            .cr
            .modify(|_, w| w.wucksel().variant(wucksel).wutie().bit(irq_en));
        self.rtc.wutr.write(|w| w.wut().bits(sec).wutoclr().bits(0));
        self.rtc.cr.modify(|_, w| w.wute().set_bit());
    }

    /// Returns `true` if the wakeup timer is enabled.
    #[inline]
    pub fn is_wakeup_timer_en(&self) -> bool {
        self.rtc.cr.read().wute().bit_is_set()
    }

    /// Disable the wakeup timer.
    #[inline]
    pub fn disable_wakeup_timer(&mut self) {
        self.rtc.cr.modify(|_, w| w.wute().clear_bit());
    }

    /// Returns the value of the wakeup timer as calculated by the RTC logic in RTC cycles
    pub fn wakeup_period_cycles(&self) -> u32 {
        let wutr = self.rtc.wutr.read().wut().bits();
        let wucksel_extension = self.rtc.cr.read().wucksel().is_clock_spare_with_offset();
        if wucksel_extension {
            u32::from(wutr) + 0x1_0000
        } else {
            u32::from(wutr)
        }
    }

    /// Set alarm A.
    ///
    /// This will disable the alarm if previously enabled.
    ///
    /// This will not enable the alarm after setup.
    /// To enable the alarm use [`set_alarm_a_en`](Self::set_alarm_a_en).
    pub fn set_alarm_a(&mut self, alarm: &Alarm) {
        self.rtc.cr.modify(|_, w| w.alrae().clear_bit());
        self.rtc.alrmar.write(|w| unsafe { w.bits(alarm.val) });
        self.rtc.alrmassr.write(|w| w.maskss().bits(alarm.ss_mask));
        self.rtc.alrabinr().write(|w| w.ss().bits(alarm.ss));
    }

    /// Returns `true` if alarm A is enabled.
    #[inline]
    #[must_use]
    pub fn is_alarm_a_en(&self) -> bool {
        self.rtc.cr.read().alrae().is_enabled()
    }

    /// Get the value of alarm A.
    #[inline]
    #[must_use]
    pub fn alarm_a(&self) -> Alarm {
        Alarm {
            val: self.rtc.alrmar.read().bits(),
            ss: self.rtc.alrabinr().read().ss().bits(),
            ss_mask: self.rtc.alrmassr.read().maskss().bits(),
        }
    }

    /// Set the alarm A enable, and alarm A interrupt enable.
    #[inline]
    pub fn set_alarm_a_en(&mut self, en: bool, irq_en: bool) {
        self.rtc
            .cr
            .modify(|_, w| w.alrae().bit(en).alraie().bit(irq_en));
    }

    /// Set alarm B.
    ///
    /// This will disable the alarm if previously enabled.
    ///
    /// This will not enable the alarm after setup.
    /// To enable the alarm use [`set_alarm_b_en`](Self::set_alarm_b_en).
    pub fn set_alarm_b(&mut self, alarm: &Alarm) {
        self.rtc.cr.modify(|_, w| w.alrbe().clear_bit());
        self.rtc.alrmbr.write(|w| unsafe { w.bits(alarm.val) });
        self.rtc.alrmbssr.write(|w| w.maskss().bits(alarm.ss_mask));
        self.rtc.alrbbinr().write(|w| w.ss().bits(alarm.ss));
    }

    /// Returns `true` if alarm B is enabled.
    #[inline]
    #[must_use]
    pub fn is_alarm_b_en(&self) -> bool {
        self.rtc.cr.read().alrbe().is_enabled()
    }

    /// Get the value of alarm B.
    #[inline]
    #[must_use]
    pub fn alarm_b(&self) -> Alarm {
        Alarm {
            val: self.rtc.alrmbr.read().bits(),
            ss: self.rtc.alrbbinr().read().ss().bits(),
            ss_mask: self.rtc.alrmbssr.read().maskss().bits(),
        }
    }

    /// Set the alarm B enable, and alarm B interrupt enable.
    #[inline]
    pub fn set_alarm_b_en(&mut self, en: bool, irq_en: bool) {
        self.rtc
            .cr
            .modify(|_, w| w.alrbe().bit(en).alrbie().bit(irq_en));
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
