#![no_std]

use core::time::Duration;

use stm32wl_hal::subghz::{
    AddrComp, CrcType, FskBandwidth, FskBitrate, FskFdev, FskModParams, FskPulseShape,
    GenericPacketParams, PaConfig, PaSel, PayloadType, PreambleDetection, RampTime, RfFreq,
    TcxoMode, TcxoTrim, Timeout, TxParams,
};

pub const DATA: &str = "HELLO WORLD!";
pub const DATA_LEN: u8 = DATA.len() as u8;
pub const DATA_BYTES: &[u8] = DATA.as_bytes();
pub const PREAMBLE_LEN: u16 = 5 * 8;

pub const RF_FREQ: RfFreq = RfFreq::from_frequency(434_000_000);

pub const SYNC_WORD: [u8; 8] = [0x79, 0x80, 0x0C, 0xC0, 0x29, 0x95, 0xF8, 0x4A];
pub const SYNC_WORD_LEN: u8 = SYNC_WORD.len() as u8;
pub const SYNC_WORD_LEN_BITS: u8 = SYNC_WORD_LEN * 8;

pub const PACKET_PARAMS: GenericPacketParams = GenericPacketParams::new()
    .set_preamble_len(PREAMBLE_LEN)
    .set_preamble_detection(PreambleDetection::Bit8)
    .set_sync_word_len(SYNC_WORD_LEN_BITS)
    .set_addr_comp(AddrComp::Disabled)
    .set_payload_type(PayloadType::Fixed)
    .set_payload_len(DATA_LEN)
    .set_crc_type(CrcType::Byte2)
    .set_whitening_enable(true);

pub const MOD_PARAMS: FskModParams = FskModParams::new()
    .set_bitrate(FskBitrate::from_bps(50_000))
    .set_pulse_shape(FskPulseShape::None)
    .set_bandwidth(FskBandwidth::Bw58)
    .set_fdev(FskFdev::from_hertz(25_000));

// configuration for +10 dBm output power
// see table 35 "PA optimal setting and operating modes"
pub const PA_CONFIG: PaConfig = PaConfig::new()
    .set_pa_duty_cycle(0x1)
    .set_hp_max(0x0)
    .set_pa(PaSel::Lp);

pub const TCXO_MODE: TcxoMode = TcxoMode::new()
    .set_txco_trim(TcxoTrim::Volts1pt7)
    .set_timeout(Timeout::from_duration_sat(Duration::from_millis(10)));

pub const TX_PARAMS: TxParams = TxParams::new()
    .set_power(0x0D)
    .set_ramp_time(RampTime::Micros40);
