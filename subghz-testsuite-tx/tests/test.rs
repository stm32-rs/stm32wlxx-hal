#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;

use stm32wl_hal::{
    pac,
    subghz::{
        CalibrateImage, CmdStatus, Ocp, PacketType, RegMode, StandbyClk, Status, StatusMode,
        SubGhz, Timeout,
    },
};

#[defmt_test::tests]
mod tests {
    use subghz_testsuite_assets::{
        DATA_BYTES, MOD_PARAMS, PACKET_PARAMS, PA_CONFIG, RF_FREQ, SYNC_WORD, TCXO_MODE, TX_PARAMS,
    };

    use super::*;

    #[init]
    fn init() -> SubGhz {
        let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
        let gpioc = &dp.GPIOC;
        let spi3 = dp.SPI3;
        let mut rcc = dp.RCC;
        let pwr = dp.PWR;

        rcc.apb3enr.modify(|_, w| w.subghzspien().set_bit());
        rcc.apb3enr.read(); // Delay after an RCC peripheral clock enabling
        rcc.csr.modify(|_, w| w.rfrst().set_bit());
        rcc.csr.modify(|_, w| w.rfrst().clear_bit());
        pwr.subghzspicr.write(|w| w.nss().clear_bit());
        pwr.subghzspicr.write(|w| w.nss().set_bit());
        pwr.cr3.modify(|_, w| w.ewrfbusy().set_bit());
        pwr.scr.write(|w| w.cwrfbusyf().set_bit());

        // GPIO setup for the RF switch on the NUCLEO-WL55JC2
        #[rustfmt::skip]
        rcc.ahb2enr.modify(|_, w|
            w
                .gpioaen().set_bit()
                .gpioben().set_bit()
                .gpiocen().set_bit()
        );
        rcc.ahb2enr.read(); // Delay after an RCC peripheral clock enabling

        #[rustfmt::skip]
        gpioc.moder.write(|w| 
            w
                .moder3().output()
                .moder4().output()
                .moder5().output()
        );
        // Drive FE_CTRL1, FE_CTRL2, FE_CTRL3 high.
        // Configures the RF switch for low power transmit
        gpioc
            .odr
            .write(|w| w.odr3().set_bit().odr4().set_bit().odr5().set_bit());

        // TODO: set clocks to 48MHz so the tests execute fater

        SubGhz::new(spi3, &mut rcc)
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
