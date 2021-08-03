#![no_std]
#![no_main]
#![cfg_attr(feature = "aio", feature(alloc_error_handler))]

use defmt_rtt as _; // global logger
use panic_probe as _;

use nucleo_wl55jc_bsp as bsp;

use bsp::{hal, RfSwitch};

use hal::{
    dma::{AllDma, DmaCh},
    gpio::{PortA, PortC},
    pac,
    subghz::{
        CalibrateImage, CfgDioIrq, CmdStatus, Irq, IrqLine, Ocp, PacketType, RegMode, StandbyClk,
        Status, StatusMode, SubGhz, Timeout,
    },
};

#[cfg(feature = "aio")]
async fn aio_buffer_io_inner() {
    let mut sg = unsafe {
        let dma = AllDma::steal();
        SubGhz::steal_with_dma(dma.d1c1, dma.d2c1)
    };
    const DATA: [u8; 255] = [0xA5; 255];
    let mut buf: [u8; 255] = [0; 255];
    sg.aio_write_buffer(0, &DATA).await.unwrap();
    sg.aio_read_buffer(0, &mut buf).await.unwrap();
    assert_eq!(DATA, buf);
}

#[defmt_test::tests]
mod tests {
    use subghz_testsuite_assets::{
        DATA_BYTES, DATA_LEN, MOD_PARAMS, PACKET_PARAMS, PA_CONFIG, RF_FREQ, SYNC_WORD, TCXO_MODE,
        TX_PARAMS,
    };

    use super::*;

    #[init]
    fn init() -> SubGhz<DmaCh> {
        let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();

        let gpioa: PortA = PortA::split(dp.GPIOA, &mut dp.RCC);
        let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
        let mut rfs: RfSwitch = RfSwitch::new(gpioc.pc3, gpioc.pc4, gpioc.pc5);
        rfs.set_rx();

        let dma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
        let tx_dma: DmaCh = dma.d1c1;
        let rx_dma: DmaCh = dma.d2c1;

        #[cfg(feature = "aio")]
        {
            let start: usize = stm32wl_hal::cortex_m_rt::heap_start() as usize;
            let size: usize = 2048; // in bytes
            unsafe { ate::ALLOCATOR.init(start, size) };
        }

        let mut sg: SubGhz<DmaCh> = SubGhz::new_with_dma(dp.SPI3, tx_dma, rx_dma, &mut dp.RCC);
        sg.enable_spi_debug(gpioa.pa4, gpioa.pa5, gpioa.pa6, gpioa.pa7);

        sg
    }

    #[test]
    #[cfg(feature = "aio")]
    fn aio_buffer_io(_: &mut SubGhz<DmaCh>) {
        let mut executor = ate::Executor::new();
        executor.spawn(ate::Task::new(aio_buffer_io_inner()));
        executor.run();
    }

    #[test]
    fn fsk_rx(sg: &mut SubGhz<DmaCh>) {
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

        const IRQ_CFG: CfgDioIrq = CfgDioIrq::new()
            .irq_enable(IrqLine::Global, Irq::RxDone)
            .irq_enable(IrqLine::Global, Irq::Timeout);
        sg.set_irq_cfg(&IRQ_CFG).unwrap();

        sg.set_rx(Timeout::DISABLED).unwrap();

        // infinite loop while waiting for incoming data
        loop {
            let (status, irq_status) = sg.irq_status().unwrap();
            if irq_status & Irq::RxDone.mask() != 0 {
                let (status, len, ptr) = sg.rx_buffer_status().unwrap();
                assert_eq!(len, DATA_LEN);
                assert_eq!(status.cmd(), Ok(CmdStatus::Avaliable));
                assert_eq!(ptr, 0);
                assert_eq!(irq_status, 0b10);
                let mut data_buf: [u8; DATA_BYTES.len()] = [0; DATA_BYTES.len()];
                sg.read_buffer(ptr, &mut data_buf).unwrap();
                assert_eq!(data_buf, DATA_BYTES);
                break;
            }
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
