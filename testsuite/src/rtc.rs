#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use nucleo_wl55jc_bsp::hal::{
    cortex_m,
    pac::{self, DWT},
    rcc::{self, pulse_reset_backup_domain, setup_lsi, LsiPre},
    rtc::{self, Rtc},
    time::{Date, Month, PrimitiveDateTime, Time},
    util::reset_cycle_count,
};
use panic_probe as _;

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

    let date: Date = unwrap!(Date::from_calendar_date(2021, Month::October, 24).ok());
    let time: Time = unwrap!(Time::from_hms(08, 52, 36).ok());
    let set_dt: PrimitiveDateTime = PrimitiveDateTime::new(date, time);
    rtc.set_date_time(set_dt);
    let start: u32 = DWT::get_cycle_count();

    // wait 4 RTC clock cycles for the datetime to apply
    loop {
        let elapsed: u32 = DWT::get_cycle_count() - start;
        if elapsed >= four_rtc_clk_cycles {
            break;
        }
    }

    let rtc_time: Time = unwrap!(rtc.time());
    defmt::assert_eq!(rtc_time.hour(), set_dt.hour());
    defmt::assert_eq!(rtc_time.minute(), set_dt.minute());
    defmt::assert_eq!(rtc_time.second(), set_dt.second());

    let rtc_date: Date = unwrap!(rtc.date());
    defmt::assert_eq!(rtc_date.year(), set_dt.year());
    defmt::assert_eq!(u8::from(rtc_date.month()), u8::from(set_dt.month()));
    defmt::assert_eq!(rtc_date.day(), set_dt.day());

    // delay 10ms
    const TEN_MILLIS: u32 = CYC_PER_MILLI * 10;
    let start: u32 = DWT::get_cycle_count();
    loop {
        let elapsed: u32 = DWT::get_cycle_count().wrapping_sub(start);
        if elapsed > TEN_MILLIS {
            defmt::trace!("delayed {} CPU CLK cycles", elapsed);
            break;
        }
    }

    let rtc_date_time: PrimitiveDateTime = unwrap!(rtc.date_time());
    defmt::assert_eq!(rtc_date_time.hour(), set_dt.hour());
    defmt::assert_eq!(rtc_date_time.minute(), set_dt.minute());
    defmt::assert_eq!(rtc_date_time.second(), set_dt.second());
    defmt::assert_eq!(rtc_date_time.year(), set_dt.year());
    defmt::assert_eq!(u8::from(rtc_date_time.month()), u8::from(set_dt.month()));
    defmt::assert_eq!(rtc_date_time.day(), set_dt.day());

    let before: i128 = set_dt.assume_utc().unix_timestamp_nanos();
    let after: i128 = rtc_date_time.assume_utc().unix_timestamp_nanos();
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

        cortex_m::interrupt::free(|cs| unsafe {
            rcc::set_sysclk_msi_max(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC, cs)
        });
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
    fn set_date_time_lse(ta: &mut TestArgs) {
        unsafe { pulse_reset_backup_domain(&mut ta.rcc, &mut ta.pwr) };
        ta.pwr.cr1.modify(|_, w| w.dbp().enabled());
        ta.rcc.bdcr.modify(|_, w| w.lseon().on());
        while ta.rcc.bdcr.read().lserdy().is_not_ready() {}

        test_set_date_time_with_clk(rtc::Clk::Lse)
    }

    // must come directly after set_date_time_lse
    #[test]
    fn renew(ta: &mut TestArgs) {
        {
            let mut dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
            let rtc: Rtc = unsafe { Rtc::renew(dp.RTC, &mut dp.PWR, &mut dp.RCC) };
            let date: Date = unwrap!(rtc.date());
            let expected_date: Date =
                unwrap!(Date::from_calendar_date(2021, Month::October, 24).ok());
            assert_eq!(date, expected_date);
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
        unsafe { setup_lsi(&mut ta.rcc, LsiPre::DIV1) };
        test_set_date_time_with_clk(rtc::Clk::Lsi)
    }

    #[test]
    fn set_date_time_hse(ta: &mut TestArgs) {
        unsafe { pulse_reset_backup_domain(&mut ta.rcc, &mut ta.pwr) };
        ta.rcc
            .cr
            .modify(|_, w| w.hseon().enabled().hsebyppwr().vddtcxo());
        while ta.rcc.cr.read().hserdy().is_not_ready() {}
        test_set_date_time_with_clk(rtc::Clk::Hse)
    }
}
