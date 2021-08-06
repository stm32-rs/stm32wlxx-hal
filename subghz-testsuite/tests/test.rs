#![no_std]
#![no_main]

#[cfg(feature = "aio")]
extern crate alloc;

use defmt::unwrap;
use defmt_rtt as _; // global logger
use panic_probe as _;

use nucleo_wl55jc_bsp as bsp;
use static_assertions as sa;

use bsp::{
    hal::{
        dma::{AllDma, DmaCh},
        gpio::{PortA, PortC},
        pac::{self, DWT},
        rcc,
        rng::{self, Rng},
        subghz::{
            AddrComp, CalibrateImage, CfgIrq, CmdStatus, CodingRate, CrcType, FskBandwidth,
            FskBitrate, FskFdev, FskModParams, FskPulseShape, GenericPacketParams, HeaderType, Irq,
            LoRaBandwidth, LoRaModParams, LoRaPacketParams, LoRaSyncWord, Ocp, PaConfig, PaSel,
            PacketType, PreambleDetection, RampTime, RegMode, RfFreq, SpreadingFactor, StandbyClk,
            Status, StatusMode, SubGhz, TcxoMode, TcxoTrim, Timeout, TxParams,
        },
    },
    RfSwitch,
};

const FREQ: u32 = 48_000_000;
const CYC_PER_US: u32 = FREQ / 1000 / 1000;
const CYC_PER_MS: u32 = FREQ / 1000;
const CYC_PER_SEC: u32 = FREQ;

use core::{ptr::write_volatile, time::Duration};

const PING_DATA: &str = "PING";
const PONG_DATA: &str = "PONG";
const DATA_LEN: u8 = PING_DATA.len() as u8;
sa::const_assert_eq!(PING_DATA.len(), PONG_DATA.len());
const PING_DATA_BYTES: &[u8] = PING_DATA.as_bytes();
const PONG_DATA_BYTES: &[u8] = PONG_DATA.as_bytes();
const PREAMBLE_LEN: u16 = 5 * 8;

const RF_FREQ: RfFreq = RfFreq::from_frequency(434_000_000);

const SYNC_WORD: [u8; 8] = [0x79, 0x80, 0x0C, 0xC0, 0x29, 0x95, 0xF8, 0x4A];
const SYNC_WORD_LEN: u8 = SYNC_WORD.len() as u8;
const SYNC_WORD_LEN_BITS: u8 = SYNC_WORD_LEN * 8;

const TX_BUF_OFFSET: u8 = 128;
const RX_BUF_OFFSET: u8 = 0;

const FSK_PACKET_PARAMS: GenericPacketParams = GenericPacketParams::new()
    .set_preamble_len(PREAMBLE_LEN)
    .set_preamble_detection(PreambleDetection::Bit8)
    .set_sync_word_len(SYNC_WORD_LEN_BITS)
    .set_addr_comp(AddrComp::Disabled)
    .set_header_type(HeaderType::Fixed)
    .set_payload_len(DATA_LEN)
    .set_crc_type(CrcType::Byte2)
    .set_whitening_enable(true);

const LORA_PACKET_PARAMS: LoRaPacketParams = LoRaPacketParams::new()
    .set_crc_en(true)
    .set_preamble_len(PREAMBLE_LEN)
    .set_payload_len(DATA_LEN)
    .set_invert_iq(false)
    .set_header_type(HeaderType::Fixed);

const FSK_MOD_PARAMS: FskModParams = FskModParams::new()
    .set_bitrate(FskBitrate::from_bps(50_000))
    .set_pulse_shape(FskPulseShape::None)
    .set_bandwidth(FskBandwidth::Bw58)
    .set_fdev(FskFdev::from_hertz(25_000));

const LORA_MOD_PARAMS: LoRaModParams = LoRaModParams::new()
    .set_bw(LoRaBandwidth::Bw125)
    .set_cr(CodingRate::Cr45)
    .set_ldro_en(true)
    .set_sf(SpreadingFactor::Sf7);

// configuration for +10 dBm output power
// see table 35 "PA optimal setting and operating modes"
const PA_CONFIG: PaConfig = PaConfig::new()
    .set_pa_duty_cycle(0x1)
    .set_hp_max(0x0)
    .set_pa(PaSel::Lp);

const TCXO_MODE: TcxoMode = TcxoMode::new()
    .set_txco_trim(TcxoTrim::Volts1pt7)
    .set_timeout(Timeout::from_duration_sat(Duration::from_millis(10)));

const TX_PARAMS: TxParams = TxParams::new()
    .set_power(0x0D)
    .set_ramp_time(RampTime::Micros40);

#[cfg(feature = "aio")]
async fn aio_buffer_io_inner() {
    let mut sg = unsafe {
        let dma = AllDma::steal();
        SubGhz::steal_with_dma(dma.d1c1, dma.d2c1)
    };
    const DATA: [u8; 255] = [0xA5; 255];
    let mut buf: alloc::vec::Vec<u8> = alloc::vec![0; 255];
    unwrap!(sg.aio_write_buffer(0, &DATA).await);
    unwrap!(sg.aio_read_buffer(0, &mut buf).await);
    defmt::assert_eq!(DATA.as_ref(), buf.as_slice());
}

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:µs}", DWT::get_cycle_count() / CYC_PER_US);

#[cfg(feature = "aio")]
async fn aio_wait_irq_inner() {
    let mut sg = unsafe {
        let dma = AllDma::steal();
        SubGhz::steal_with_dma(dma.d1c1, dma.d2c1)
    };

    unwrap!(sg.aio_set_standby(StandbyClk::Rc).await);
    let status: Status = unwrap!(sg.aio_status().await);
    defmt::assert_eq!(status.mode(), Ok(StatusMode::StandbyRc));

    unwrap!(sg.aio_set_tcxo_mode(&TCXO_MODE).await);
    unwrap!(sg.aio_set_regulator_mode(RegMode::Ldo).await);
    unwrap!(sg.aio_set_sync_word(&SYNC_WORD).await);
    unwrap!(sg.aio_set_packet_type(PacketType::Fsk).await);
    unwrap!(sg.aio_set_fsk_mod_params(&FSK_MOD_PARAMS).await);
    unwrap!(sg.aio_set_packet_params(&FSK_PACKET_PARAMS).await);
    unwrap!(sg.aio_calibrate_image(CalibrateImage::ISM_430_440).await);
    unwrap!(sg.aio_set_rf_frequency(&RF_FREQ).await);

    const IRQ_CFG: CfgIrq = CfgIrq::new()
        .irq_enable(Irq::RxDone)
        .irq_enable(Irq::Timeout);
    unwrap!(sg.aio_set_irq_cfg(&IRQ_CFG).await);

    let status: Status = unwrap!(sg.status());
    defmt::assert_ne!(status.cmd(), Ok(CmdStatus::Timeout));

    // this will fail to RX immediately (15 micros timeout)
    unwrap!(sg.aio_set_rx(Timeout::MIN).await);

    let (_, irq) = unwrap!(sg.aio_wait_irq().await);
    defmt::assert_eq!(Irq::Timeout.mask(), irq);
}

fn tx_or_panic(sg: &mut SubGhz<DmaCh>, rfs: &mut RfSwitch) {
    rfs.set_tx_lp();
    unwrap!(sg.set_tx(Timeout::DISABLED));
    let start_cc: u32 = DWT::get_cycle_count();
    loop {
        let status: Status = unwrap!(sg.status());
        if status.cmd() == Ok(CmdStatus::Complete) {
            rfs.set_rx();
            defmt::info!("TX done");
            break;
        }

        let elapsed_s: u32 = DWT::get_cycle_count().wrapping_sub(start_cc) / CYC_PER_SEC;
        defmt::assert!(
            elapsed_s < 1,
            "Timeout waiting for TX completion status={}",
            status
        );
    }
}

/// This test should be run simultaneously on two boards.
///
/// Both radios transmit `b"PING"`.
///
/// The first radio to recieve `b"PING"` transmits `b"PONG"` in reply.
fn ping_pong(sg: &mut SubGhz<DmaCh>, rng: &mut Rng, rfs: &mut RfSwitch, pkt: PacketType) {
    unwrap!(sg.set_standby(StandbyClk::Rc));
    let status: Status = unwrap!(sg.status());
    defmt::assert_eq!(status.mode(), Ok(StatusMode::StandbyRc));

    unwrap!(sg.set_tcxo_mode(&TCXO_MODE));
    unwrap!(sg.set_regulator_mode(RegMode::Ldo));
    unwrap!(sg.set_buffer_base_address(TX_BUF_OFFSET, RX_BUF_OFFSET));
    unwrap!(sg.set_pa_config(&PA_CONFIG));
    unwrap!(sg.set_pa_ocp(Ocp::Max60m));
    unwrap!(sg.set_tx_params(&TX_PARAMS));

    let status: Status = unwrap!(sg.status());
    defmt::assert_eq!(status.mode(), Ok(StatusMode::StandbyRc));

    unwrap!(sg.set_packet_type(pkt));
    match pkt {
        PacketType::Fsk => {
            unwrap!(sg.set_sync_word(&SYNC_WORD));
            unwrap!(sg.set_fsk_mod_params(&FSK_MOD_PARAMS));
            unwrap!(sg.set_packet_params(&FSK_PACKET_PARAMS));
        }
        PacketType::LoRa => {
            unwrap!(sg.set_lora_sync_word(LoRaSyncWord::Public));
            unwrap!(sg.set_lora_mod_params(&LORA_MOD_PARAMS));
            unwrap!(sg.set_lora_packet_params(&LORA_PACKET_PARAMS));
        }
        PacketType::Bpsk => defmt::todo!(),
        PacketType::Msk => defmt::todo!(),
    }

    unwrap!(sg.calibrate_image(CalibrateImage::ISM_430_440));
    unwrap!(sg.set_rf_frequency(&RF_FREQ));

    unwrap!(sg.write_buffer(TX_BUF_OFFSET, PING_DATA_BYTES));

    const IRQ_CFG: CfgIrq = CfgIrq::new()
        .irq_enable(Irq::RxDone)
        .irq_enable(Irq::Timeout);
    unwrap!(sg.set_irq_cfg(&IRQ_CFG));

    const MAX_ATTEMPTS: u32 = 100;
    let mut attempt: u32 = 0;
    loop {
        defmt::info!("Attempt: {}/{}", attempt, MAX_ATTEMPTS);

        // 45 - 300 ms
        let timeout: Timeout = {
            let rand_u8: u8 = unwrap!(rng.try_u8());
            let millis: u64 = u64::from(rand_u8).saturating_add(45);
            let dur: Duration = Duration::from_millis(millis);
            Timeout::from_duration_sat(dur)
        };

        defmt::debug!("RX for {:?} ms", timeout.as_duration().as_millis());
        unwrap!(sg.set_rx(timeout));

        let start_cc: u32 = DWT::get_cycle_count();
        loop {
            let (status, irq_status) = unwrap!(sg.irq_status());

            if irq_status != 0 {
                defmt::debug!("IRQ status: 0x{:04X}", irq_status);
            }

            let elapsed_ms: u32 = DWT::get_cycle_count().wrapping_sub(start_cc) / CYC_PER_MS;

            if irq_status & Irq::Timeout.mask() != 0 {
                defmt::info!("RX timeout {} ms", elapsed_ms);
                defmt::assert_eq!(status.mode(), Ok(StatusMode::StandbyRc));
                unwrap!(sg.clear_irq_status(irq_status));

                tx_or_panic(sg, rfs);
                break;
            } else if irq_status & Irq::RxDone.mask() != 0 {
                defmt::info!("RX done");
                defmt::assert_eq!(status.mode(), Ok(StatusMode::StandbyRc));
                unwrap!(sg.clear_irq_status(irq_status));

                let (status, len, ptr) = unwrap!(sg.rx_buffer_status());
                defmt::assert_eq!(len, DATA_LEN);
                defmt::assert_eq!(status.cmd(), Ok(CmdStatus::Avaliable));
                defmt::assert_eq!(ptr, RX_BUF_OFFSET);
                defmt::assert_eq!(irq_status, 0b10);
                let mut data_buf: [u8; DATA_LEN as usize] = [0; DATA_LEN as usize];
                unwrap!(sg.read_buffer(ptr, &mut data_buf));
                match data_buf.as_ref() {
                    PING_DATA_BYTES => {
                        defmt::info!("received PING");

                        // respond with pong
                        unwrap!(sg.write_buffer(TX_BUF_OFFSET, PONG_DATA_BYTES));

                        // assume the other radio will get it in 10 transmissions
                        for _ in 0..10 {
                            tx_or_panic(sg, rfs);
                        }
                    }
                    PONG_DATA_BYTES => {
                        defmt::info!("received PONG");
                    }
                    _ => defmt::panic!(
                        "Unknown data: {:?} PING={:?} PONG={:?}",
                        data_buf,
                        PING_DATA_BYTES,
                        PONG_DATA_BYTES
                    ),
                }

                rfs.set_rx();
                return;
            } else {
                defmt::assert_eq!(irq_status, 0);
                defmt::assert_ne!(status.cmd(), Ok(CmdStatus::Avaliable));
                defmt::assert_ne!(status.cmd(), Ok(CmdStatus::ProcessingError));
                defmt::assert_ne!(status.cmd(), Ok(CmdStatus::ExecutionFailure));

                defmt::assert!(
                    elapsed_ms < 1_000,
                    "Timeout waiting for RX completion status={}",
                    status
                );
            }
        }

        attempt = attempt.saturating_add(1);
        defmt::assert!(attempt < MAX_ATTEMPTS);
    }
}

#[defmt_test::tests]
mod tests {
    use super::*;

    struct TestArgs {
        sg: SubGhz<DmaCh>,
        rng: Rng,
        rfs: RfSwitch,
    }

    #[init]
    fn init() -> TestArgs {
        let mut cp: pac::CorePeripherals = unwrap!(pac::CorePeripherals::take());
        let mut dp: pac::Peripherals = unwrap!(pac::Peripherals::take());

        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC);
        defmt::assert_eq!(rcc::sysclk_hz(&dp.RCC), FREQ);

        let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
        let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
        let mut rfs: RfSwitch = RfSwitch::new(gpioc.pc3, gpioc.pc4, gpioc.pc5);
        rfs.set_rx();

        let dma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);

        #[cfg(feature = "aio")]
        {
            let start: usize = bsp::hal::cortex_m_rt::heap_start() as usize;
            let size: usize = 2048; // in bytes
            unsafe { ate::ALLOCATOR.init(start, size) };
        }

        Rng::set_clock_source(&mut dp.RCC, rng::ClkSrc::MSI);
        let rng: Rng = Rng::new(dp.RNG, &mut dp.RCC);

        let mut sg: SubGhz<DmaCh> = SubGhz::new_with_dma(dp.SPI3, dma.d1c1, dma.d2c1, &mut dp.RCC);
        sg.enable_spi_debug(gpioa.pa4, gpioa.pa5, gpioa.pa6, gpioa.pa7);

        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        // reset the cycle counter
        const DWT_CYCCNT: usize = 0xE0001004;
        unsafe { write_volatile(DWT_CYCCNT as *mut u32, 0) };

        TestArgs { sg, rng, rfs }
    }

    #[test]
    #[cfg(feature = "aio")]
    fn aio_buffer_io(_: &mut TestArgs) {
        let mut executor = ate::Executor::new();
        executor.spawn(ate::Task::new(aio_buffer_io_inner()));
        executor.run();
    }

    #[test]
    fn buffer_io(ta: &mut TestArgs) {
        const DATA: [u8; 255] = [0x5A; 255];
        static mut BUF: [u8; 255] = [0; 255];
        unwrap!(ta.sg.write_buffer(0, &DATA));
        unwrap!(ta.sg.read_buffer(0, unsafe { &mut BUF }));
        defmt::assert_eq!(DATA.as_ref(), unsafe { BUF }.as_ref());
    }

    #[test]
    #[cfg(feature = "aio")]
    fn aio_wait_irq(_: &mut TestArgs) {
        let mut executor = ate::Executor::new();
        executor.spawn(ate::Task::new(aio_wait_irq_inner()));
        executor.run();
    }

    #[test]
    fn fsk_ping_pong(ta: &mut TestArgs) {
        let sg: &mut SubGhz<DmaCh> = &mut ta.sg;
        let rng: &mut Rng = &mut ta.rng;
        let rfs: &mut RfSwitch = &mut ta.rfs;

        ping_pong(sg, rng, rfs, PacketType::Fsk);
    }

    #[test]
    fn lora_ping_pong(ta: &mut TestArgs) {
        let sg: &mut SubGhz<DmaCh> = &mut ta.sg;
        let rng: &mut Rng = &mut ta.rng;
        let rfs: &mut RfSwitch = &mut ta.rfs;

        ping_pong(sg, rng, rfs, PacketType::LoRa);
    }
}
