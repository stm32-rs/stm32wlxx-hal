#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;

use stm32wl_hal::{
    pac,
    subghz::{
        CalibrateImage, CmdStatus, Ocp, PacketType, RampTime, RegMode, StandbyClk, Status,
        StatusMode, SubGhz, Timeout,
    },
};

#[defmt_test::tests]
mod tests {
    use stm32wl_hal::subghz::{CfgDioIrq, Irq, IrqLine, TxParams};
    use subghz_testsuite_assets::{
        DATA_BYTES, DATA_LEN, MOD_PARAMS, PACKET_PARAMS, PA_CONFIG, RF_FREQ, SYNC_WORD, TCXO_MODE,
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
        rcc.ahb2enr.modify(|_, w| {
            w
                .gpioaen().set_bit()
                .gpioben().set_bit()
                .gpiocen().set_bit()
        });
        rcc.ahb2enr.read(); // Delay after an RCC peripheral clock enabling

        const GPIO_MODE_OUTPUT: u8 = 0b01;
        #[rustfmt::skip]
        gpioc.moder.write(|w| unsafe {
            w
                .moder3().bits(GPIO_MODE_OUTPUT)
                .moder4().bits(GPIO_MODE_OUTPUT)
                .moder5().bits(GPIO_MODE_OUTPUT)
        });
        // Drive FE_CTRL1, FE_CTRL3 high.
        // Drive FE_CTRL2 low.
        // Configures the RF switch for receive
        gpioc
            .odr
            .write(|w| w.odr3().set_bit().odr4().set_bit().odr5().clear_bit());

        // TODO: set clocks to 48MHz so the tests execute fater

        SubGhz::new(spi3, &mut rcc)
    }

    #[test]
    fn fsk_rx(sg: &mut SubGhz) {
        sg.set_standby(StandbyClk::Rc).unwrap();
        let status: Status = sg.status().unwrap();
        assert_eq!(status.mode(), Ok(StatusMode::StandbyRc));

        sg.set_tcxo_mode(&TCXO_MODE).unwrap();
        sg.set_hse_in_trim(0).unwrap();
        sg.set_regulator_mode(RegMode::Ldo).unwrap();
        sg.set_buffer_base_address(0, 0).unwrap();
        sg.set_pa_config(&PA_CONFIG).unwrap();
        sg.set_pa_ocp(Ocp::Max60m).unwrap();
        const TX_PARAMS: TxParams = TxParams::new()
            .set_power(0x0D)
            .set_ramp_time(RampTime::Micros40);
        sg.set_tx_params(&TX_PARAMS).unwrap();
        sg.set_sync_word(SYNC_WORD).unwrap();

        let status: Status = sg.status().unwrap();
        assert_eq!(status.mode(), Ok(StatusMode::StandbyRc));

        sg.set_packet_type(PacketType::Fsk).unwrap();
        sg.set_fsk_mod_params(&MOD_PARAMS).unwrap();
        sg.set_packet_params(&PACKET_PARAMS).unwrap();
        sg.calibrate_image(CalibrateImage::ISM_902_928).unwrap();
        sg.set_rf_frequency(&RF_FREQ).unwrap();

        const IRQ_CFG: CfgDioIrq = CfgDioIrq::new()
            .irq_enable(IrqLine::Global, Irq::RxDone)
            .irq_enable(IrqLine::Global, Irq::Timeout);
        sg.set_irq_cfg(&IRQ_CFG).unwrap();

        sg.set_rx(Timeout::DISABLED).unwrap();

        // infinite loop while waiting for incoming data
        loop {
            let (status, len, ptr) = sg.rx_buffer_status().unwrap();
            let (_, irq_status) = sg.irq_status().unwrap();
            if irq_status & Irq::RxDone.mask() != 0 {
                assert_eq!(len, DATA_LEN);
                assert_eq!(status.cmd(), Ok(CmdStatus::Avaliable));
                let mut data_buf: [u8; DATA_BYTES.len()] = [0; DATA_BYTES.len()];
                sg.read_buffer(ptr, &mut data_buf).unwrap();
                assert_eq!(data_buf, DATA_BYTES);
                break;
            }
            assert_eq!(len, 0);
            assert_eq!(status.mode(), Ok(StatusMode::Rx));
            assert_ne!(status.cmd(), Ok(CmdStatus::Avaliable));
            assert_ne!(status.cmd(), Ok(CmdStatus::Timeout));
            assert_ne!(status.cmd(), Ok(CmdStatus::ProcessingError));
            assert_ne!(status.cmd(), Ok(CmdStatus::ExecutionFailure));
            assert_ne!(status.cmd(), Ok(CmdStatus::Complete));
            assert_eq!(irq_status, 0);
        }
    }
}
