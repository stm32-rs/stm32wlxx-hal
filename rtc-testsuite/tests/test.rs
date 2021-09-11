#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use panic_probe as _;
use stm32wl_hal::{
    chrono::{Datelike, NaiveDate, NaiveDateTime, NaiveTime, Timelike},
    pac::{self, DWT},
    rcc::{self, pulse_reset_backup_domain, setup_lsi, LsiPre},
    rtc::{self, Rtc},
    util::reset_cycle_count,
};

const FREQ: u32 = 48_000_000;
const CYC_PER_MILLI: u32 = FREQ / 1000;
const CYC_PER_MICRO: u32 = FREQ / 1000 / 1000;

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:µs}", DWT::get_cycle_count() / CYC_PER_MICRO);

fn test_set_date_time_with_clk(clk: rtc::Clk) {
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
    let start: u32 = DWT::get_cycle_count();

    // wait 4 RTC clock cycles for the datetime to apply
    loop {
        let elapsed: u32 = DWT::get_cycle_count() - start;
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
    let start: u32 = DWT::get_cycle_count();
    loop {
        let elapsed: u32 = DWT::get_cycle_count() - start;
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

        unsafe { rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC) };
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        reset_cycle_count(&mut cp.DWT);
        defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);

        TestArgs {
            rcc: dp.RCC,
            pwr: dp.PWR,
        }
    }

    #[test]
    fn test_set_date_time_lse(ta: &mut TestArgs) {
        unsafe { pulse_reset_backup_domain(&mut ta.rcc, &mut ta.pwr) };
        ta.pwr.cr1.modify(|_, w| w.dbp().enabled());
        ta.rcc.bdcr.modify(|_, w| w.lseon().on());
        while ta.rcc.bdcr.read().lserdy().is_not_ready() {}

        test_set_date_time_with_clk(rtc::Clk::Lse)
    }

    #[test]
    fn test_set_date_time_lsi(ta: &mut TestArgs) {
        unsafe { pulse_reset_backup_domain(&mut ta.rcc, &mut ta.pwr) };
        unsafe { setup_lsi(&mut ta.rcc, LsiPre::DIV1) };
        test_set_date_time_with_clk(rtc::Clk::Lsi)
    }

    #[test]
    fn test_set_date_time_hse(ta: &mut TestArgs) {
        unsafe { pulse_reset_backup_domain(&mut ta.rcc, &mut ta.pwr) };
        ta.rcc
            .cr
            .modify(|_, w| w.hseon().enabled().hsebyppwr().vddtcxo());
        while ta.rcc.cr.read().hserdy().is_not_ready() {}
        test_set_date_time_with_clk(rtc::Clk::Hse)
    }
}
