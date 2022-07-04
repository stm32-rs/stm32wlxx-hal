#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use nucleo_wl55jc_bsp::hal::{
    chrono::{Datelike, Duration, NaiveDate, NaiveDateTime, NaiveTime, Timelike},
    cortex_m,
    pac::{self, DWT},
    rcc::{self, pulse_reset_backup_domain, setup_lsi, LsiPre},
    rtc::{self, Alarm, Rtc},
};
use panic_probe as _;

const FREQ: u32 = 48_000_000;
const CYC_PER_MILLI: u32 = FREQ / 1000;
const CYC_PER_MICRO: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:us}", DWT::cycle_count() / CYC_PER_MICRO);

fn test_set_date_time_with_clk(clk: rtc::Clk) -> Rtc {
    let mut dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
    let mut rtc: Rtc = Rtc::new(dp.RTC, clk, &mut dp.PWR, &mut dp.RCC);

    dp.PWR.cr1.modify(|_, w| w.dbp().enabled());
    let rtc_src_freq: u32 = Rtc::hz(&dp.RCC);
    defmt::assert_ne!(rtc_src_freq, 0);

    // check the hz method works without dtb
    // (it should but I do not trust the reference manual)
    dp.PWR.cr1.modify(|_, w| w.dbp().disabled());
    let rtc_src_freq_no_dpb: u32 = Rtc::hz(&dp.RCC);
    defmt::assert_eq!(rtc_src_freq, rtc_src_freq_no_dpb);
    dp.PWR.cr1.modify(|_, w| w.dbp().enabled());

    let four_rtc_clk_cycles: u32 = (FREQ / rtc_src_freq) * 4;

    defmt::trace!("four_rtc_clk_cycles={}", four_rtc_clk_cycles);

    let date: NaiveDate = NaiveDate::from_ymd(2021, 10, 20);
    let set_dt: NaiveDateTime = date.and_hms(12, 02, 05);
    rtc.set_date_time(set_dt);
    let start: u32 = DWT::cycle_count();

    // wait 4 RTC clock cycles for the datetime to apply
    loop {
        let elapsed: u32 = DWT::cycle_count() - start;
        if elapsed >= four_rtc_clk_cycles {
            break;
        }
    }

    let rtc_time: NaiveTime = unwrap!(rtc.time());
    defmt::assert_eq!(rtc_time.hour(), set_dt.hour());
    defmt::assert_eq!(rtc_time.minute(), set_dt.minute());
    defmt::assert_eq!(rtc_time.second(), set_dt.second());

    let rtc_date: NaiveDate = unwrap!(rtc.date());
    defmt::assert_eq!(rtc_date.year(), set_dt.year());
    defmt::assert_eq!(rtc_date.month(), set_dt.month());
    defmt::assert_eq!(rtc_date.day(), set_dt.day());

    // delay 10ms
    const TEN_MILLIS: u32 = CYC_PER_MILLI * 10;
    let start: u32 = DWT::cycle_count();
    loop {
        let elapsed: u32 = DWT::cycle_count() - start;
        if elapsed > TEN_MILLIS {
            break;
        }
    }

    let rtc_date_time: NaiveDateTime = unwrap!(rtc.date_time());
    defmt::assert_eq!(rtc_date_time.hour(), set_dt.hour());
    defmt::assert_eq!(rtc_date_time.minute(), set_dt.minute());
    defmt::assert_eq!(rtc_date_time.second(), set_dt.second());
    defmt::assert_eq!(rtc_date_time.year(), set_dt.year());
    defmt::assert_eq!(rtc_date_time.month(), set_dt.month());
    defmt::assert_eq!(rtc_date_time.day(), set_dt.day());

    let before: i64 = set_dt.timestamp_millis();
    let after: i64 = rtc_date_time.timestamp_millis();
    defmt::debug!(
        "Timestamp before {} after {} Δ {}",
        before,
        after,
        after - before
    );
    defmt::assert!(after > before);

    rtc
}

#[defmt_test::tests]
mod tests {
    use super::*;

    struct TestArgs {
        rcc: pac::RCC,
        pwr: pac::PWR,
    }

    #[init]
    fn init() -> TestArgs {
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());

        cortex_m::interrupt::free(|cs| unsafe {
            rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs)
        });
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        cp.DWT.set_cycle_count(0);
        defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);

        TestArgs {
            rcc: dp.RCC,
            pwr: dp.PWR,
        }
    }

    #[test]
    fn set_date_time_lse(ta: &mut TestArgs) {
        unsafe { pulse_reset_backup_domain(&mut ta.rcc, &mut ta.pwr) };
        ta.pwr.cr1.modify(|_, w| w.dbp().enabled());
        ta.rcc.bdcr.modify(|_, w| w.lseon().on());
        while ta.rcc.bdcr.read().lserdy().is_not_ready() {}

        test_set_date_time_with_clk(rtc::Clk::Lse);
    }

    // must come directly after set_date_time_lse
    #[test]
    fn renew(ta: &mut TestArgs) {
        {
            let mut dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
            let rtc: Rtc = unsafe { Rtc::renew(dp.RTC, &mut dp.PWR, &mut dp.RCC) };
            let date: NaiveDate = unwrap!(rtc.date());
            assert_eq!(date, NaiveDate::from_ymd(2021, 10, 20));
        }

        unsafe { pulse_reset_backup_domain(&mut ta.rcc, &mut ta.pwr) };

        {
            let mut dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
            let rtc: Rtc = unsafe { Rtc::renew(dp.RTC, &mut dp.PWR, &mut dp.RCC) };
            defmt::assert!(rtc.date().is_none());
        }
    }

    #[test]
    fn set_date_time_lsi(ta: &mut TestArgs) {
        unsafe { pulse_reset_backup_domain(&mut ta.rcc, &mut ta.pwr) };
        unsafe { setup_lsi(&mut ta.rcc, LsiPre::Div1) };
        test_set_date_time_with_clk(rtc::Clk::Lsi);
    }

    #[test]
    fn set_date_time_hse(ta: &mut TestArgs) {
        unsafe { pulse_reset_backup_domain(&mut ta.rcc, &mut ta.pwr) };
        ta.rcc
            .cr
            .modify(|_, w| w.hseon().enabled().hsebyppwr().vddtcxo());
        while ta.rcc.cr.read().hserdy().is_not_ready() {}
        test_set_date_time_with_clk(rtc::Clk::Hse);
    }

    #[test]
    fn wakeup_timer(ta: &mut TestArgs) {
        unsafe { pulse_reset_backup_domain(&mut ta.rcc, &mut ta.pwr) };
        ta.pwr.cr1.modify(|_, w| w.dbp().enabled());
        ta.rcc.bdcr.modify(|_, w| w.lseon().on());
        while ta.rcc.bdcr.read().lserdy().is_not_ready() {}
        let mut rtc: Rtc = test_set_date_time_with_clk(rtc::Clk::Lse);

        rtc.setup_wakeup_timer(0, false);

        let start: u32 = DWT::cycle_count();
        loop {
            let elapsed_micros: u32 = DWT::cycle_count().wrapping_sub(start) / CYC_PER_MICRO;
            if Rtc::status().wutf().bit_is_set() {
                defmt::info!("elapsed: {=u32:us}", elapsed_micros);
                // 100ms tolerance
                defmt::assert!(elapsed_micros > 900_000 && elapsed_micros < 1_100_000);
                return;
            } else if elapsed_micros > 3 * 1000 * 1000 {
                defmt::panic!("Timeout! Elapsed: {=u32:us}", elapsed_micros);
            }
        }
    }

    #[test]
    fn alarm(ta: &mut TestArgs) {
        unsafe { pulse_reset_backup_domain(&mut ta.rcc, &mut ta.pwr) };
        ta.pwr.cr1.modify(|_, w| w.dbp().enabled());
        ta.rcc.bdcr.modify(|_, w| w.lseon().on());
        while ta.rcc.bdcr.read().lserdy().is_not_ready() {}
        let mut rtc: Rtc = test_set_date_time_with_clk(rtc::Clk::Lse);

        defmt::assert!(!rtc.is_alarm_a_en());

        let alarm: Alarm = Alarm::from(unwrap!(rtc.time()) + Duration::seconds(1))
            .set_days_mask(true)
            .set_hours_mask(true)
            .set_minutes_mask(true)
            .set_seconds_mask(false);

        rtc.set_alarm_a(&alarm);
        rtc.set_alarm_a_en(true, false);

        let start: u32 = DWT::cycle_count();
        loop {
            let elapsed_micros: u32 = DWT::cycle_count().wrapping_sub(start) / CYC_PER_MICRO;
            if Rtc::status().alraf().bit_is_set() {
                defmt::info!("elapsed: {=u32:us}", elapsed_micros);
                // 100ms tolerance
                defmt::assert!(elapsed_micros > 900_000 && elapsed_micros < 1_100_000);
                break;
            } else if elapsed_micros > 2 * 1000 * 1000 {
                defmt::info!("{:08X}", Rtc::status().bits());
                defmt::panic!("Timeout! Elapsed: {=u32:us}", elapsed_micros);
            }
        }

        defmt::assert!(rtc.is_alarm_a_en());
        defmt::assert_eq!(rtc.alarm_a(), alarm);
    }
}
