use stm32wl_hal_subghz::{CmdStatus, LoRaStats, Stats, StatusMode};

#[test]
fn mixed() {
    let example_data_from_radio: [u8; 7] = [0x54, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
    let stats: Stats<LoRaStats> = Stats::from_raw_lora(example_data_from_radio);
    assert_eq!(stats.status().mode(), Ok(StatusMode::Rx));
    assert_eq!(stats.status().cmd(), Ok(CmdStatus::Avaliable));
    assert_eq!(stats.pkt_rx(), 0x0102);
    assert_eq!(stats.pkt_crc(), 0x0304);
    assert_eq!(stats.pkt_hdr_err(), 0x0506);
}
