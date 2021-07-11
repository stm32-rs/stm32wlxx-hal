#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;

use bsp::hal;
use nucleo_wl55jc_bsp as bsp;

use hal::{
    pac,
    subghz::{
        CalibrateImage, CmdStatus, Ocp, PacketType, RegMode, StandbyClk, Status, StatusMode,
        SubGhz, Timeout,
    },
};

#[defmt_test::tests]
mod tests {
    use bsp::{hal::gpio::PortC, RfSwitch};
    use subghz_testsuite_assets::{
        DATA_BYTES, MOD_PARAMS, PACKET_PARAMS, PA_CONFIG, RF_FREQ, SYNC_WORD, TCXO_MODE, TX_PARAMS,
    };

    use super::*;

    #[init]
    fn init() -> SubGhz {
        let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();

        let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
        let mut rfs: RfSwitch = RfSwitch::new(gpioc.pc3, gpioc.pc4, gpioc.pc5);
        rfs.set_tx_lp();

        SubGhz::new(dp.SPI3, &mut dp.RCC)
    }

    #[test]
    fn fsk_tx(sg: &mut SubGhz) {
        sg.set_standby(StandbyClk::Rc).unwrap();
        let status: Status = sg.status().unwrap();
        assert_eq!(status.mode(), Ok(StatusMode::StandbyRc));

        sg.set_tcxo_mode(&TCXO_MODE).unwrap();
        sg.set_regulator_mode(RegMode::Ldo).unwrap();
        sg.set_buffer_base_address(0, 0).unwrap();
        sg.set_pa_config(&PA_CONFIG).unwrap();
        sg.set_pa_ocp(Ocp::Max60m).unwrap();
        sg.set_tx_params(&TX_PARAMS).unwrap();
        sg.set_sync_word(&SYNC_WORD).unwrap();

        let status: Status = sg.status().unwrap();
        assert_eq!(status.mode(), Ok(StatusMode::StandbyRc));

        sg.set_packet_type(PacketType::Fsk).unwrap();
        sg.set_fsk_mod_params(&MOD_PARAMS).unwrap();
        sg.set_packet_params(&PACKET_PARAMS).unwrap();
        sg.calibrate_image(CalibrateImage::ISM_430_440).unwrap();
        sg.set_rf_frequency(&RF_FREQ).unwrap();

        sg.write_buffer(0, DATA_BYTES).unwrap();
        sg.set_tx(Timeout::DISABLED).unwrap();

        let mut loop_count: u32 = 0;
        loop {
            let status: Status = sg.status().unwrap();
            if status.cmd() == Ok(CmdStatus::Complete) {
                break;
            }
            loop_count = loop_count.saturating_add(1);
            assert!(
                loop_count < 10000,
                "Timeout waiting for completion status={}",
                status
            );
        }
    }
}
