use core::time::Duration;
use stm32wl_hal_subghz::{Timeout, ValueError};

#[test]
fn saturate() {
    assert_eq!(
        Timeout::from_duration_sat(Duration::from_secs(u64::MAX)),
        Timeout::MAX
    );
}

#[test]
fn rounding() {
    const NANO1: Duration = Duration::from_nanos(1);
    let res_sub_1_ns: Duration = Timeout::RESOLUTION - NANO1;
    let res_add_1_ns: Duration = Timeout::RESOLUTION + NANO1;
    assert_eq!(Timeout::from_duration_sat(res_sub_1_ns).into_bits(), 1);
    assert_eq!(Timeout::from_duration_sat(res_add_1_ns).into_bits(), 1);
}

#[test]
fn lower_limit() {
    let low: Duration = (Timeout::RESOLUTION + Duration::from_nanos(1)) / 2;
    assert_eq!(Timeout::from_duration(low), Ok(Timeout::from_bits(1)));

    let too_low: Duration = low - Duration::from_nanos(1);
    assert_eq!(
        Timeout::from_duration(too_low),
        Err(ValueError::too_low(too_low.as_nanos(), low.as_nanos()))
    );
}

#[test]
fn upper_limit() {
    let high: Duration = Timeout::MAX.as_duration() + Timeout::RESOLUTION / 2;
    assert_eq!(
        Timeout::from_duration(high),
        Ok(Timeout::from_bits(0xFFFFFF))
    );

    let too_high: Duration = high + Duration::from_nanos(1);
    assert_eq!(
        Timeout::from_duration(too_high),
        Err(ValueError::too_high(too_high.as_nanos(), high.as_nanos()))
    );
}
