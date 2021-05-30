use stm32wl_hal_subghz::RfFreq;

#[test]
fn max() {
    assert_eq!(RfFreq::from_raw(u32::MAX).freq(), 4_095_999_999);
}

#[test]
fn min() {
    assert_eq!(RfFreq::from_raw(u32::MIN).freq(), 0);
}
