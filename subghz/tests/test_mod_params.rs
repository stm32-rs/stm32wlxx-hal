use stm32wl_hal_subghz::{FskBandwidth, FskBitrate, FskFdev, LoRaBandwidth};

#[test]
fn fsk_bw_ord() {
    assert!((FskBandwidth::Bw4 as u8) > (FskBandwidth::Bw5 as u8));
    assert!(FskBandwidth::Bw4 < FskBandwidth::Bw5);
    assert!(FskBandwidth::Bw5 > FskBandwidth::Bw4);
}

#[test]
fn lora_bw_ord() {
    assert!((LoRaBandwidth::Bw10 as u8) > (LoRaBandwidth::Bw15 as u8));
    assert!(LoRaBandwidth::Bw10 < LoRaBandwidth::Bw15);
    assert!(LoRaBandwidth::Bw15 > LoRaBandwidth::Bw10);
}

#[test]
fn fsk_bitrate_ord() {
    assert!(FskBitrate::from_bps(9600) > FskBitrate::from_bps(4800));
    assert!(FskBitrate::from_bps(4800) < FskBitrate::from_bps(9600));
}

#[test]
fn fsk_bitrate_as_bps_limits() {
    const ZERO: FskBitrate = FskBitrate::from_bits(0);
    const ONE: FskBitrate = FskBitrate::from_bits(1);
    const MAX: FskBitrate = FskBitrate::from_bits(u32::MAX);

    assert_eq!(ZERO.as_bps(), 0);
    assert_eq!(ONE.as_bps(), 1_024_000_000);
    assert_eq!(MAX.as_bps(), 61);
}

#[test]
fn fsk_bitrate_from_bps_limits() {
    const ZERO: FskBitrate = FskBitrate::from_bps(0);
    const ONE: FskBitrate = FskBitrate::from_bps(1);
    const MAX: FskBitrate = FskBitrate::from_bps(u32::MAX);

    assert_eq!(ZERO.as_bps(), 61);
    assert_eq!(ONE.as_bps(), 61);
    assert_eq!(MAX.as_bps(), 0);
}

#[test]
fn fsk_fdev_ord() {
    assert!(FskFdev::from_hertz(30_000) > FskFdev::from_hertz(20_000));
    assert!(FskFdev::from_hertz(20_000) < FskFdev::from_hertz(30_000));
}

#[test]
fn fsk_fdev_as_hertz_limits() {
    const ZERO: FskFdev = FskFdev::from_bits(0);
    const ONE: FskFdev = FskFdev::from_bits(1);
    const MAX: FskFdev = FskFdev::from_bits(u32::MAX);

    assert_eq!(ZERO.as_hertz(), 0);
    assert_eq!(ONE.as_hertz(), 0);
    assert_eq!(MAX.as_hertz(), 15_999_999);
}

#[test]
fn fsk_fdev_from_hertz_limits() {
    const ZERO: FskFdev = FskFdev::from_hertz(0);
    const ONE: FskFdev = FskFdev::from_hertz(1);
    const MAX: FskFdev = FskFdev::from_hertz(u32::MAX);

    assert_eq!(ZERO.as_hertz(), 0);
    assert_eq!(ONE.as_hertz(), 0);
    assert_eq!(MAX.as_hertz(), 6_967_294);
}
