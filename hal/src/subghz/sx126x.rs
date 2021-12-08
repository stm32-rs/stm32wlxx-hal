use radio::modulation::lora;
use radio::modulation::lora::LoRaChannel;
use radio::{BasicInfo, Busy, Receive, Transmit};

use crate::spi::Spi3;
use crate::subghz;
use crate::subghz::rfs::{RfSwRx, RfSwTx};
use crate::subghz::{
    CalibrateImage, CfgIrq, CodingRate, FallbackMode, HeaderType, Irq, LoRaModParams,
    LoRaPacketParams, LoRaSyncWord, Ocp, PaConfig, PacketType, RegMode, RfFreq, SpreadingFactor,
    StandbyClk, SubGhz, TcxoMode, TcxoTrim, Timeout, TxParams,
};

const IRQ_CFG: CfgIrq = CfgIrq::new()
    .irq_enable_all(Irq::RxDone)
    .irq_enable_all(Irq::Timeout)
    .irq_enable_all(Irq::TxDone)
    .irq_enable_all(Irq::Err);

const TX_BUF_OFFSET: u8 = 128;
const RX_BUF_OFFSET: u8 = 0;

/// Sx126x radio.
#[derive(Debug)]
pub struct Sx126x<MISO, MOSI, RFS> {
    sg: SubGhz<MISO, MOSI>,
    rfs: RFS,
    config: SxConfig,
}

impl<MISO, MOSI, RFS> Sx126x<MISO, MOSI, RFS>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = subghz::Error>,
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Write<u8, Error = subghz::Error>,
    RFS: RfSwRx + RfSwTx,
{
    /// Creates a new Sx126x radio.
    pub fn new(sg: SubGhz<MISO, MOSI>, rfs: RFS, config: SxConfig) -> Self {
        Sx126x { sg, rfs, config }
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
    type Error = Sx126xError;

    fn start_transmit(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        // TODO: Check current modulation
        let lora_packet_params = LoRaPacketParams::new()
            .set_crc_en(true)
            .set_preamble_len(8)
            .set_payload_len(data.len() as u8)
            .set_invert_iq(false)
            .set_header_type(HeaderType::Variable);

        self.sg.set_lora_packet_params(&lora_packet_params)?;
        self.sg.write_buffer(TX_BUF_OFFSET, data)?;
        self.rfs.set_tx();
        self.sg.set_tx(self.config.tx_timeout)?;

        Ok(())
    }

    fn check_transmit(&mut self) -> Result<bool, Self::Error> {
        let (_, irq_status) = self.sg.irq_status()?;
        if irq_status & Irq::Timeout.mask() != 0 {
            self.sg.clear_irq_status(irq_status)?;
            Err(Sx126xError::Timeout)
        } else if irq_status & Irq::Err.mask() != 0 {
            self.sg.clear_irq_status(irq_status)?;
            Err(Sx126xError::Tx)
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
    type Error = Sx126xError;
    type Info = BasicInfo;

    fn start_receive(&mut self) -> Result<(), Self::Error> {
        self.rfs.set_rx();
        self.sg.set_rx(self.config.rx_timeout)?;
        Ok(())
    }

    fn check_receive(&mut self, _: bool) -> Result<bool, Self::Error> {
        let (_, irq_status) = self.sg.irq_status()?;
        if irq_status & Irq::Timeout.mask() != 0 {
            self.sg.clear_irq_status(irq_status)?;
            Err(Sx126xError::Timeout)
        } else if irq_status & Irq::Err.mask() != 0 {
            self.sg.clear_irq_status(irq_status)?;
            Err(Sx126xError::Rx)
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
    /// LoRa modulation.
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
    type Error = Sx126xError;

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
                self.sg.set_tcxo_mode(&self.config.tcxo_mode)?;
                self.sg.set_standby(StandbyClk::Hse)?;
                self.sg.set_tx_rx_fallback_mode(FallbackMode::StandbyHse)?;
                self.sg.set_regulator_mode(RegMode::Ldo)?;
                self.sg
                    .set_buffer_base_address(TX_BUF_OFFSET, RX_BUF_OFFSET)?;
                self.sg.set_pa_config(&self.config.pa_config)?;
                self.sg.set_pa_ocp(Ocp::Max60m)?;
                self.sg.set_tx_params(&self.config.tx_params)?;
                self.sg.set_packet_type(PacketType::LoRa)?;
                self.sg.set_lora_sync_word(LoRaSyncWord::Public)?;
                self.sg.set_lora_mod_params(&lora_mod_params)?;
                self.sg.calibrate_image(self.config.calibrate_image)?;
                self.sg.set_rf_frequency(&rf_freq)?;
                self.sg.set_irq_cfg(&IRQ_CFG)?;

                Ok(())
            }
        }
    }
}

impl<MISO, MOSI, RFS> Busy for Sx126x<MISO, MOSI, RFS> {
    type Error = Sx126xError;

    fn is_busy(&mut self) -> Result<bool, Self::Error> {
        Ok(subghz::rfbusys())
    }
}

/// The configuration for the Sx126x.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SxConfig {
    preamble_len: u16,
    pa_config: PaConfig,
    tx_params: TxParams,
    tcxo_mode: TcxoMode,
    tx_timeout: Timeout,
    rx_timeout: Timeout,
    calibrate_image: CalibrateImage,
}

impl SxConfig {
    /// Create a new `SxConfig` struct.
    ///
    /// This is the same as `default`, but in a `const` function.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::subghz::SxConfig;
    ///
    /// const SX_CONFIG: SxConfig = SxConfig::new();
    /// ```
    pub const fn new() -> Self {
        SxConfig {
            preamble_len: 8,
            pa_config: PaConfig::new(),
            tx_params: TxParams::new(),
            tcxo_mode: TcxoMode::new()
                .set_txco_trim(TcxoTrim::Volts1pt7)
                .set_timeout(Timeout::from_millis_sat(10)),
            tx_timeout: Timeout::DISABLED,
            rx_timeout: Timeout::DISABLED,
            calibrate_image: CalibrateImage::ISM_863_870,
        }
    }

    /// Set the preamble length.
    pub const fn set_preamble_len(mut self, preamble_len: u16) -> Self {
        self.preamble_len = preamble_len;
        self
    }

    /// Set the power amplifier configuration.
    pub const fn set_pa_config(mut self, pa_config: PaConfig) -> Self {
        self.pa_config = pa_config;
        self
    }

    /// Set the transmit parameters.
    pub const fn set_tx_params(mut self, tx_params: TxParams) -> Self {
        self.tx_params = tx_params;
        self
    }

    /// Set the TCXO trim and HSE32 ready timeout.
    pub const fn set_tcxo_mode(mut self, tcxo_mode: TcxoMode) -> Self {
        self.tcxo_mode = tcxo_mode;
        self
    }

    /// Set the transmit timeout.
    pub const fn set_tx_timeout(mut self, timeout: Timeout) -> Self {
        self.tx_timeout = timeout;
        self
    }

    /// Set the receive timeout.
    pub const fn set_rx_timeout(mut self, timeout: Timeout) -> Self {
        self.rx_timeout = timeout;
        self
    }

    /// Set the image calibration.
    pub const fn set_calibrate_image(mut self, calibrate_image: CalibrateImage) -> Self {
        self.calibrate_image = calibrate_image;
        self
    }
}

impl Default for SxConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Errors that can occur during communication with the SX126x.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sx126xError {
    /// Internal radio error.
    SubGhz(subghz::Error),
    /// Unsupported bandwidth.
    Bandwidth(subghz::BandwidthError),
    /// A timeout occurred.
    Timeout,
    /// Something went wrong during transmit.
    Tx,
    /// Something went wrong during receive.
    Rx,
}

impl From<subghz::Error> for Sx126xError {
    fn from(err: subghz::Error) -> Self {
        Sx126xError::SubGhz(err)
    }
}

impl From<subghz::BandwidthError> for Sx126xError {
    fn from(err: subghz::BandwidthError) -> Self {
        Sx126xError::Bandwidth(err)
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
