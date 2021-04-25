use stm32wl_hal_subghz::{GfskBandwidth, GfskBitrate, GfskFdev, LoRaBandwidth};

#[test]
fn gfsk_bw_ord() {
    assert!((GfskBandwidth::Bw4 as u8) > (GfskBandwidth::Bw5 as u8));
    assert!(GfskBandwidth::Bw4 < GfskBandwidth::Bw5);
    assert!(GfskBandwidth::Bw5 > GfskBandwidth::Bw4);
}

#[test]
fn lora_bw_ord() {
    assert!((LoRaBandwidth::Bw10 as u8) > (LoRaBandwidth::Bw15 as u8));
    assert!(LoRaBandwidth::Bw10 < LoRaBandwidth::Bw15);
    assert!(LoRaBandwidth::Bw15 > LoRaBandwidth::Bw10);
}

#[test]
fn gfsk_bitrate_ord() {
    assert!(GfskBitrate::from_bps(9600) > GfskBitrate::from_bps(4800));
    assert!(GfskBitrate::from_bps(4800) < GfskBitrate::from_bps(9600));
}

#[test]
fn gfsk_bitrate_as_bps_limits() {
    const ZERO: GfskBitrate = GfskBitrate::from_bits(0);
    const ONE: GfskBitrate = GfskBitrate::from_bits(1);
    const MAX: GfskBitrate = GfskBitrate::from_bits(u32::MAX);

    assert_eq!(ZERO.as_bps(), 0);
    assert_eq!(ONE.as_bps(), 1_024_000_000);
    assert_eq!(MAX.as_bps(), 61);
}

#[test]
fn gfsk_bitrate_from_bps_limits() {
    const ZERO: GfskBitrate = GfskBitrate::from_bps(0);
    const ONE: GfskBitrate = GfskBitrate::from_bps(1);
    const MAX: GfskBitrate = GfskBitrate::from_bps(u32::MAX);

    assert_eq!(ZERO.as_bps(), 61);
    assert_eq!(ONE.as_bps(), 61);
    assert_eq!(MAX.as_bps(), 0);
}

#[test]
fn gfsk_fdev_ord() {
    assert!(GfskFdev::from_hertz(30_000) > GfskFdev::from_hertz(20_000));
    assert!(GfskFdev::from_hertz(20_000) < GfskFdev::from_hertz(30_000));
}

#[test]
fn gfsk_fdev_as_hertz_limits() {
    const ZERO: GfskFdev = GfskFdev::from_bits(0);
    const ONE: GfskFdev = GfskFdev::from_bits(1);
    const MAX: GfskFdev = GfskFdev::from_bits(u32::MAX);

    assert_eq!(ZERO.as_hertz(), 0);
    assert_eq!(ONE.as_hertz(), 0);
    assert_eq!(MAX.as_hertz(), 15_999_999);
}

#[test]
fn gfsk_fdev_from_hertz_limits() {
    const ZERO: GfskFdev = GfskFdev::from_hertz(0);
    const ONE: GfskFdev = GfskFdev::from_hertz(1);
    const MAX: GfskFdev = GfskFdev::from_hertz(u32::MAX);

    assert_eq!(ZERO.as_hertz(), 0);
    assert_eq!(ONE.as_hertz(), 0);
    assert_eq!(MAX.as_hertz(), 6_967_294);
}
