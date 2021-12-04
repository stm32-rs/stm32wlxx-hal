use core::time::Duration;

use radio::modulation::lora;
use radio::modulation::lora::LoRaChannel;
use radio::{BasicInfo, Busy, Receive, Transmit};

use crate::spi::Spi3;
use crate::subghz;
use crate::subghz::rfs::{RfSwRx, RfSwTx};
use crate::subghz::{
    CalibrateImage, CfgIrq, CodingRate, FallbackMode, HeaderType, Irq, LoRaModParams,
    LoRaPacketParams, LoRaSyncWord, Ocp, PaConfig, PacketType, RampTime, RegMode, RfFreq,
    SpreadingFactor, StandbyClk, SubGhz, TcxoMode, TcxoTrim, Timeout, TxParams,
};

const IRQ_CFG: CfgIrq = CfgIrq::new()
    .irq_enable_all(Irq::RxDone)
    .irq_enable_all(Irq::Timeout)
    .irq_enable_all(Irq::TxDone)
    .irq_enable_all(Irq::Err);

const PREAMBLE_LEN: u16 = 5 * 8;
const TX_BUF_OFFSET: u8 = 0;
const RX_BUF_OFFSET: u8 = 128;

const PA_CONFIG: PaConfig = PaConfig::LP_10;
const TX_PARAMS: TxParams = TxParams::LP_10.set_ramp_time(RampTime::Micros40);

const TCXO_MODE: TcxoMode = TcxoMode::new()
    .set_txco_trim(TcxoTrim::Volts1pt7)
    .set_timeout(Timeout::from_millis_sat(10));

/// Allowed transmission time before timeout.
const TX_TIMEOUT: Timeout = Timeout::from_duration_sat(Duration::from_millis(5000));

/// Allowed receiving time before timeout.
const RX_TIMEOUT: Timeout = Timeout::from_duration_sat(Duration::from_millis(600));

/// Sx126x radio.
#[derive(Debug)]
pub struct Sx126x<MISO, MOSI, RFS> {
    sg: SubGhz<MISO, MOSI>,
    rfs: RFS,
}

impl<MISO, MOSI, RFS> Sx126x<MISO, MOSI, RFS>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = subghz::Error>,
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Write<u8, Error = subghz::Error>,
    RFS: RfSwRx + RfSwTx,
{
    /// Creates a new Sx126x radio.
    pub fn new(sg: SubGhz<MISO, MOSI>, rfs: RFS) -> Self {
        Sx126x { sg, rfs }
    }

    /// Returns the internal Sub-GHz radio peripheral.
    pub fn as_subghz(&self) -> &SubGhz<MISO, MOSI> {
        &self.sg
    }

    /// Returns a mutable reference to the internal Sub-GHz radio peripheral.
    pub fn as_mut_subghz(&mut self) -> &mut SubGhz<MISO, MOSI> {
        &mut self.sg
    }
}

impl<MISO, MOSI, RFS> Transmit for Sx126x<MISO, MOSI, RFS>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = subghz::Error>,
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Write<u8, Error = subghz::Error>,
    RFS: RfSwRx + RfSwTx,
{
    type Error = Error;

    fn start_transmit(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        // TODO: Check current modulation
        let lora_packet_params = LoRaPacketParams::new()
            .set_crc_en(true)
            .set_preamble_len(PREAMBLE_LEN)
            .set_payload_len(data.len() as u8)
            .set_invert_iq(false)
            .set_header_type(HeaderType::Fixed);

        self.sg.set_lora_packet_params(&lora_packet_params)?;
        self.sg.write_buffer(TX_BUF_OFFSET, data)?;
        self.rfs.set_tx();
        self.sg.set_tx(TX_TIMEOUT)?;

        Ok(())
    }

    fn check_transmit(&mut self) -> Result<bool, Self::Error> {
        let (_, irq_status) = self.sg.irq_status()?;
        if irq_status & Irq::Timeout.mask() != 0 {
            self.sg.clear_irq_status(irq_status)?;
            Err(Error::Timeout)
        } else if irq_status & Irq::TxDone.mask() != 0 {
            self.sg.clear_irq_status(irq_status)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }
}

impl<MISO, MOSI, RFS> Receive for Sx126x<MISO, MOSI, RFS>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = subghz::Error>,
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Write<u8, Error = subghz::Error>,
    RFS: RfSwRx + RfSwTx,
{
    type Error = Error;
    type Info = BasicInfo;

    fn start_receive(&mut self) -> Result<(), Self::Error> {
        self.rfs.set_rx();
        self.sg.set_rx(RX_TIMEOUT)?;
        Ok(())
    }

    fn check_receive(&mut self, _: bool) -> Result<bool, Self::Error> {
        let (_, irq_status) = self.sg.irq_status()?;
        if irq_status & Irq::Timeout.mask() != 0 {
            self.sg.clear_irq_status(irq_status)?;
            Err(Error::Timeout)
        } else if irq_status & Irq::TxDone.mask() != 0 {
            self.sg.clear_irq_status(irq_status)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn get_received(&mut self, buf: &mut [u8]) -> Result<(usize, Self::Info), Self::Error> {
        let (_, len, ptr) = self.sg.rx_buffer_status()?;
        let size = usize::from(len);
        let data: &mut [u8] = &mut buf[..size];
        self.sg.read_buffer(ptr, data)?;
        // TODO: get info
        let info = BasicInfo::default();
        Ok((size, info))
    }
}

/// All supported modulations for the Sx126x.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum Channel {
    LoRa(LoRaChannel),
}

impl From<LoRaChannel> for Channel {
    fn from(channel: LoRaChannel) -> Self {
        Channel::LoRa(channel)
    }
}

impl<MISO, MOSI, RFS> radio::Channel for Sx126x<MISO, MOSI, RFS>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = subghz::Error>,
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Write<u8, Error = subghz::Error>,
{
    type Channel = Channel;
    type Error = Error;

    fn set_channel(&mut self, channel: &Self::Channel) -> Result<(), Self::Error> {
        match channel {
            Channel::LoRa(channel) => {
                let rf_freq = RfFreq::from_frequency(channel.freq_khz * 1000);
                let lora_mod_params = LoRaModParams::new()
                    .set_bw((channel.bw_khz as u32 * 1000).try_into()?)
                    .set_cr(channel.cr.into())
                    .set_ldro_en(true)
                    .set_sf(channel.sf.into());

                self.sg.set_standby(StandbyClk::Rc)?;
                self.sg.set_tcxo_mode(&TCXO_MODE)?;
                self.sg.set_tx_rx_fallback_mode(FallbackMode::Standby)?;
                self.sg.set_regulator_mode(RegMode::Ldo)?;
                self.sg
                    .set_buffer_base_address(TX_BUF_OFFSET, RX_BUF_OFFSET)?;
                self.sg.set_pa_config(&PA_CONFIG)?;
                self.sg.set_pa_ocp(Ocp::Max60m)?;
                self.sg.set_tx_params(&TX_PARAMS)?;
                self.sg.set_packet_type(PacketType::LoRa)?;
                self.sg.set_lora_sync_word(LoRaSyncWord::Public)?;
                self.sg.set_lora_mod_params(&lora_mod_params)?;
                self.sg.calibrate_image(CalibrateImage::ISM_430_440)?;
                self.sg.set_rf_frequency(&rf_freq)?;
                self.sg.set_irq_cfg(&IRQ_CFG)?;

                Ok(())
            }
        }
    }
}

impl<MISO, MOSI, RFS> Busy for Sx126x<MISO, MOSI, RFS> {
    type Error = Error;

    fn is_busy(&mut self) -> Result<bool, Self::Error> {
        Ok(subghz::rfbusys())
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    SubGhz(subghz::Error),
    Bandwidth(subghz::BandwidthError),
    Timeout,
}

impl From<subghz::Error> for Error {
    fn from(err: subghz::Error) -> Self {
        Error::SubGhz(err)
    }
}

impl From<subghz::BandwidthError> for Error {
    fn from(err: subghz::BandwidthError) -> Self {
        Error::Bandwidth(err)
    }
}

impl From<lora::CodingRate> for CodingRate {
    fn from(cr: lora::CodingRate) -> Self {
        match cr {
            lora::CodingRate::Cr4_5 => CodingRate::Cr45,
            lora::CodingRate::Cr4_6 => CodingRate::Cr46,
            lora::CodingRate::Cr4_7 => CodingRate::Cr47,
            lora::CodingRate::Cr4_8 => CodingRate::Cr48,
            _ => todo!("implement CodingRate"),
        }
    }
}

impl From<lora::SpreadingFactor> for SpreadingFactor {
    fn from(sf: lora::SpreadingFactor) -> Self {
        match sf {
            lora::SpreadingFactor::Sf5 => SpreadingFactor::Sf5,
            lora::SpreadingFactor::Sf6 => SpreadingFactor::Sf6,
            lora::SpreadingFactor::Sf7 => SpreadingFactor::Sf7,
            lora::SpreadingFactor::Sf8 => SpreadingFactor::Sf8,
            lora::SpreadingFactor::Sf9 => SpreadingFactor::Sf9,
            lora::SpreadingFactor::Sf10 => SpreadingFactor::Sf10,
            lora::SpreadingFactor::Sf11 => SpreadingFactor::Sf11,
            lora::SpreadingFactor::Sf12 => SpreadingFactor::Sf12,
            _ => todo!("implement SpreadingFactor"),
        }
    }
}
