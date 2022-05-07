//! Sub-GHz radio operating in the 150 - 960 MHz ISM band
//!
//! The main radio type is [`SubGhz`].
//!
//! ## LoRa user notice
//!
//! The Sub-GHz radio may have an undocumented erratum, see this ST community
//! post for more information: [link]
//!
//! [link]: https://community.st.com/s/question/0D53W00000hR8kpSAC/stm32wl55-erratum-clairification

mod bit_sync;
mod cad_params;
mod calibrate;
mod fallback_mode;
mod hse_trim;
mod irq;
mod lora_sync_word;
mod mod_params;
mod ocp;
mod op_error;
mod pa_config;
mod packet_params;
mod packet_status;
mod packet_type;
mod pkt_ctrl;
mod pmode;
mod pwr_ctrl;
mod reg_mode;
mod rf_frequency;
mod rx_timeout_stop;
mod sleep_cfg;
mod smps;
mod standby_clk;
mod stats;
mod status;
mod tcxo_mode;
mod timeout;
mod tx_params;
mod value_error;

use crate::{
    dma::DmaCh,
    pac,
    spi::{BaudRate, SgMiso, SgMosi, Spi3},
};

pub use bit_sync::BitSync;
pub use cad_params::{CadParams, ExitMode, NbCadSymbol};
pub use calibrate::{Calibrate, CalibrateImage};
pub use fallback_mode::FallbackMode;
pub use hse_trim::HseTrim;
pub use irq::{CfgIrq, Irq, IrqLine};
pub use lora_sync_word::LoRaSyncWord;
pub use mod_params::BpskModParams;
pub use mod_params::{CodingRate, LoRaBandwidth, LoRaModParams, SpreadingFactor};
pub use mod_params::{FskBandwidth, FskBitrate, FskFdev, FskModParams, FskPulseShape};
pub use ocp::Ocp;
pub use op_error::OpError;
pub use pa_config::{PaConfig, PaSel};
pub use packet_params::{
    AddrComp, BpskPacketParams, CrcType, GenericPacketParams, HeaderType, LoRaPacketParams,
    PreambleDetection,
};
pub use packet_status::{FskPacketStatus, LoRaPacketStatus};
pub use packet_type::PacketType;
pub use pkt_ctrl::{InfSeqSel, PktCtrl};
pub use pmode::PMode;
pub use pwr_ctrl::{CurrentLim, PwrCtrl};
pub use reg_mode::RegMode;
pub use rf_frequency::RfFreq;
pub use rx_timeout_stop::RxTimeoutStop;
pub use sleep_cfg::{SleepCfg, Startup};
pub use smps::SmpsDrv;
pub use standby_clk::StandbyClk;
pub use stats::{FskStats, LoRaStats, Stats};
pub use status::{CmdStatus, Status, StatusMode};
pub use tcxo_mode::{TcxoMode, TcxoTrim};
pub use timeout::Timeout;
pub use tx_params::{RampTime, TxParams};
pub use value_error::ValueError;

use crate::Ratio;

use embedded_hal::blocking::spi::{Transfer, Write};

/// Passthrough for SPI errors (for now)
pub type Error = crate::spi::Error;

struct Nss {
    _priv: (),
}

impl Nss {
    #[inline(always)]
    pub fn new() -> Nss {
        Self::clear();
        Nss { _priv: () }
    }

    /// Clear NSS, enabling SPI transactions
    #[inline(always)]
    fn clear() {
        unsafe { (*pac::PWR::PTR).subghzspicr.write(|w| w.nss().clear_bit()) }
    }

    /// Set NSS, disabling SPI transactions
    #[inline(always)]
    fn set() {
        unsafe { (*pac::PWR::PTR).subghzspicr.write(|w| w.nss().set_bit()) }
    }
}

impl Drop for Nss {
    fn drop(&mut self) {
        Self::set()
    }
}

fn baud_rate(rcc: &pac::RCC) -> BaudRate {
    // see RM0453 rev 1 section 7.2.13 page 291
    // The sub-GHz radio SPI clock is derived from the PCLK3 clock.
    // The SUBGHZSPI_SCK frequency is obtained by PCLK3 divided by two.
    // The SUBGHZSPI_SCK clock maximum speed must not exceed 16 MHz.
    if crate::rcc::hclk3_hz(rcc) > 32_000_000 {
        BaudRate::Div4
    } else {
        BaudRate::Div2
    }
}

/// Wakeup the radio from sleep mode.
///
/// # Safety
///
/// 1. This must not be called when the SubGHz radio is in use.
/// 2. This must not be called when the SubGHz SPI bus is in use.
///
/// # Example
///
/// See [`SubGhz::set_sleep`]
#[inline]
pub unsafe fn wakeup() {
    Nss::clear();
    // RM0453 rev 2 page 171 section 5.7.2 "Sleep mode"
    // on a firmware request via the sub-GHz radio SPI NSS signal
    // (keeping sub-GHz radio SPI NSS low for at least 20 μs)
    //
    // I have found this to be a more reliable mechanism for ensuring NSS is
    // pulled low for long enough to wake the radio.
    while rfbusys() {}
    Nss::set();
}

/// Unmask the SubGHz IRQ in the NVIC.
///
/// # Safety
///
/// This can break mask-based critical sections.
///
/// # Example
///
/// ```no_run
/// unsafe { stm32wlxx_hal::subghz::unmask_irq() };
/// ```
#[inline]
pub unsafe fn unmask_irq() {
    pac::NVIC::unmask(pac::Interrupt::RADIO_IRQ_BUSY)
}

/// Mask the SubGHz IRQ in the NVIC.
///
/// # Example
///
/// ```no_run
/// stm32wlxx_hal::subghz::mask_irq();
/// ```
#[inline]
pub fn mask_irq() {
    pac::NVIC::mask(pac::Interrupt::RADIO_IRQ_BUSY)
}

/// Returns `true` if the radio is busy.
///
/// This may not be set immediately after NSS going low.
///
/// See RM0461 Rev 4 section 5.3 page 181 "Radio busy management" for more
/// details.
#[inline]
pub fn rfbusys() -> bool {
    // safety: atmoic read with no side-effects
    unsafe { (*pac::PWR::PTR).sr2.read().rfbusys().is_busy() }
}

/// Returns `true` if the radio is busy or NSS is low.
///
/// See RM0461 Rev 4 section 5.3 page 181 "Radio busy management" for more
/// details.
#[inline]
pub fn rfbusyms() -> bool {
    // saftey: atomic read with no side-effects
    unsafe { (*pac::PWR::PTR).sr2.read().rfbusyms().is_busy() }
}

#[inline]
fn hold_reset(rcc: &mut pac::RCC) {
    rcc.csr.modify(|_, w| w.rfrst().set_bit());
}

#[inline]
fn release_reset(rcc: &mut pac::RCC) {
    rcc.csr.modify(|_, w| w.rfrst().clear_bit());
}

#[inline]
fn pulse_reset(rcc: &mut pac::RCC) {
    hold_reset(rcc);
    release_reset(rcc);
}

/// Sub-GHz radio peripheral.
///
/// # Example
///
/// GFSK setup
///
/// ```no_run
/// use static_assertions as sa;
/// use stm32wlxx_hal::{
///     dma::{AllDma, Dma1Ch1, Dma1Ch2},
///     pac,
///     subghz::{
///         AddrComp, CalibrateImage, CfgIrq, CrcType, FallbackMode, FskBandwidth, FskBitrate,
///         FskFdev, FskModParams, FskPulseShape, GenericPacketParams, HeaderType, Irq, Ocp,
///         PaConfig, PacketType, PreambleDetection, RampTime, RegMode, RfFreq, StandbyClk, SubGhz,
///         TcxoMode, TcxoTrim, Timeout, TxParams,
///     },
/// };
///
/// const IRQ_CFG: CfgIrq = CfgIrq::new()
///     .irq_enable_all(Irq::RxDone)
///     .irq_enable_all(Irq::Timeout)
///     .irq_enable_all(Irq::TxDone)
///     .irq_enable_all(Irq::Err);
///
/// const PREAMBLE_LEN: u16 = 5 * 8;
/// const RF_FREQ: RfFreq = RfFreq::from_frequency(434_000_000);
/// const SYNC_WORD: [u8; 8] = [0x79, 0x80, 0x0C, 0xC0, 0x29, 0x95, 0xF8, 0x4A];
/// const SYNC_WORD_LEN: u8 = SYNC_WORD.len() as u8;
/// const SYNC_WORD_LEN_BITS: u8 = SYNC_WORD_LEN * 8;
/// const TX_BUF_OFFSET: u8 = 0;
/// const RX_BUF_OFFSET: u8 = 128;
///
/// const FSK_PACKET_PARAMS: GenericPacketParams = GenericPacketParams::new()
///     .set_preamble_len(PREAMBLE_LEN)
///     .set_preamble_detection(PreambleDetection::Bit8)
///     .set_sync_word_len(SYNC_WORD_LEN_BITS)
///     .set_addr_comp(AddrComp::Disabled)
///     .set_header_type(HeaderType::Variable)
///     .set_payload_len(128)
///     .set_crc_type(CrcType::Byte2)
///     .set_whitening_enable(true);
///
/// const FSK_MOD_PARAMS: FskModParams = FskModParams::new()
///     .set_bitrate(FskBitrate::from_bps(20_000))
///     .set_pulse_shape(FskPulseShape::Bt03)
///     .set_bandwidth(FskBandwidth::Bw58)
///     .set_fdev(FskFdev::from_hertz(10_000));
///
/// sa::const_assert!(FSK_MOD_PARAMS.is_valid_worst_case());
///
/// const PA_CONFIG: PaConfig = PaConfig::LP_10;
/// const TX_PARAMS: TxParams = TxParams::LP_10.set_ramp_time(RampTime::Micros40);
///
/// const TCXO_MODE: TcxoMode = TcxoMode::new()
///     .set_txco_trim(TcxoTrim::Volts1pt7)
///     .set_timeout(Timeout::from_millis_sat(10));
///
/// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
///
/// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
/// let mut sg: SubGhz<Dma1Ch1, Dma1Ch2> =
///     SubGhz::new_with_dma(dp.SPI3, dma.d1.c1, dma.d1.c2, &mut dp.RCC);
///
/// sg.set_standby(StandbyClk::Rc)?;
/// sg.set_tcxo_mode(&TCXO_MODE)?;
/// sg.set_tx_rx_fallback_mode(FallbackMode::Standby)?;
/// sg.set_regulator_mode(RegMode::Ldo)?;
/// sg.set_buffer_base_address(TX_BUF_OFFSET, RX_BUF_OFFSET)?;
/// sg.set_pa_config(&PA_CONFIG)?;
/// sg.set_pa_ocp(Ocp::Max60m)?;
/// sg.set_tx_params(&TX_PARAMS)?;
/// sg.set_packet_type(PacketType::Fsk)?;
/// sg.set_sync_word(&SYNC_WORD)?;
/// sg.set_fsk_mod_params(&FSK_MOD_PARAMS)?;
/// sg.set_packet_params(&FSK_PACKET_PARAMS)?;
/// sg.calibrate_image(CalibrateImage::ISM_430_440)?;
/// sg.set_rf_frequency(&RF_FREQ)?;
/// sg.set_irq_cfg(&IRQ_CFG)?;
/// # Ok::<(), stm32wlxx_hal::subghz::Error>(())
/// ```
///
/// Basic RX, requires setup.
///
/// ```no_run
/// # let mut sg = unsafe { stm32wlxx_hal::subghz::SubGhz::steal() };
/// use stm32wlxx_hal::subghz::{Irq, Timeout};
///
/// // if you have an RF switch put it into RX mode on this line
/// sg.set_rx(Timeout::DISABLED)?;
///
/// loop {
///     let (status, irq_status) = sg.irq_status()?;
///     sg.clear_irq_status(irq_status)?;
///
///     if irq_status & Irq::RxDone.mask() != 0 {
///         let (status, len, ptr) = sg.rx_buffer_status()?;
///         // this is a little large for the stack
///         let mut buf: [u8; 128] = [0; 128];
///         let data: &mut [u8] = &mut buf[..usize::from(len)];
///         sg.read_buffer(ptr, data)?;
///         // ... do things with the data
///         break;
///     } else {
///         // ... handle other IRQs
///     }
/// }
/// # Ok::<(), stm32wlxx_hal::subghz::Error>(())
/// ```
///
/// Basic TX, requires setup.
///
/// ```no_run
/// # const TX_BUF_OFFSET: u8 = 0;
/// # let mut sg = unsafe { stm32wlxx_hal::subghz::SubGhz::steal() };
/// use stm32wlxx_hal::subghz::{Irq, Timeout};
///
/// sg.write_buffer(TX_BUF_OFFSET, b"Hello, World!")?;
/// // if you have an RF switch put it into TX mode on this line
/// sg.set_tx(Timeout::from_millis_sat(100))?;
///
/// loop {
///     let (status, irq_status) = sg.irq_status()?;
///     sg.clear_irq_status(irq_status)?;
///
///     if irq_status & Irq::TxDone.mask() != 0 {
///         // nothing is required upon TX done
///         // you may want to put the radio into RX mode after TX completion
///         break;
///     } else {
///         // ... handle other IRQs
///     }
/// }
/// # Ok::<(), stm32wlxx_hal::subghz::Error>(())
/// ```
#[derive(Debug)]
pub struct SubGhz<MISO, MOSI> {
    spi: Spi3<MISO, MOSI>,
}

impl<MISO, MOSI> SubGhz<MISO, MOSI> {
    /// Disable the SPI3 (SubGHz SPI) clock.
    ///
    /// # Safety
    ///
    /// 1. You are responsible for ensuring the SPI bus is in a state where the
    ///    clock can be disabled without entering an error state.
    /// 2. You cannot use the SPI bus while the clock is disabled.
    /// 3. You are responsible for re-enabling the clock before resuming use
    ///    of the SPI bus.
    /// 4. You are responsible for setting up anything that may have lost state
    ///    while the clock was disabled.
    pub unsafe fn disable_spi_clock(rcc: &mut pac::RCC) {
        Spi3::<SgMiso, SgMosi>::disable_clock(rcc)
    }

    /// Enable the SPI3 (SubGHz SPI) clock.
    pub fn enable_spi_clock(rcc: &mut pac::RCC) {
        Spi3::<SgMiso, SgMosi>::enable_clock(rcc)
    }

    fn poll_not_busy(&self) {
        // TODO: this is a terrible timeout
        let mut count: u32 = 1_000_000;
        while rfbusys() {
            count -= 1;
            if count == 0 {
                let dp = unsafe { pac::Peripherals::steal() };
                panic!(
                    "rfbusys timeout pwr.sr2=0x{:X} pwr.subghzspicr=0x{:X} pwr.cr1=0x{:X}",
                    dp.PWR.sr2.read().bits(),
                    dp.PWR.subghzspicr.read().bits(),
                    dp.PWR.cr1.read().bits(),
                );
            }
        }
    }

    /// Free the SPI3 peripheral and DMA channels from the SubGhz driver.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{pac, subghz::SubGhz};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let sg = SubGhz::new(dp.SPI3, &mut dp.RCC);
    /// let (spi, _, _) = sg.free();
    /// ```
    pub fn free(self) -> (pac::SPI3, MISO, MOSI) {
        self.spi.free()
    }
}

impl<MISO, MOSI> SubGhz<MISO, MOSI>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = Error>
        + embedded_hal::blocking::spi::Write<u8, Error = Error>,
{
    fn read(&mut self, opcode: OpCode, data: &mut [u8]) -> Result<(), Error> {
        self.poll_not_busy();
        {
            let _nss: Nss = Nss::new();
            self.spi.write(&[opcode as u8])?;
            self.spi.transfer(data)?;
        }
        self.poll_not_busy();
        Ok(())
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Error> {
        self.poll_not_busy();
        {
            let _nss: Nss = Nss::new();
            self.spi.write(data)?;
        }
        self.poll_not_busy();
        Ok(())
    }

    /// Read one byte from the sub-Ghz radio.
    fn read_1(&mut self, opcode: OpCode) -> Result<u8, Error> {
        let mut buf: [u8; 1] = [0; 1];
        self.read(opcode, &mut buf)?;
        Ok(buf[0])
    }

    /// Read a fixed number of bytes from the sub-Ghz radio.
    fn read_n<const N: usize>(&mut self, opcode: OpCode) -> Result<[u8; N], Error> {
        let mut buf: [u8; N] = [0; N];
        self.read(opcode, &mut buf)?;
        Ok(buf)
    }
}

impl SubGhz<SgMiso, SgMosi> {
    /// Create a new sub-GHz radio driver from a peripheral.
    ///
    /// This will reset the radio and the SPI bus, and enable the peripheral
    /// clock.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{pac, subghz::SubGhz};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let sg = SubGhz::new(dp.SPI3, &mut dp.RCC);
    /// ```
    pub fn new(spi: pac::SPI3, rcc: &mut pac::RCC) -> SubGhz<SgMiso, SgMosi> {
        pulse_reset(rcc);
        // safety: radio has been reset
        unsafe { Self::new_no_reset(spi, rcc) }
    }

    /// Create a new sub-GHz radio driver from a peripheral.
    ///
    /// Same as `new`, but without a reset.
    /// This is useful when waking up from standby to handle a radio interrupt.
    ///
    /// # Safety
    ///
    /// This will not reset the radio, the radio will be in an unknown state
    /// when constructed with this method.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{pac, subghz::SubGhz};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let sg = unsafe { SubGhz::new_no_reset(dp.SPI3, &mut dp.RCC) };
    /// ```
    #[allow(unused_unsafe)]
    pub unsafe fn new_no_reset(spi: pac::SPI3, rcc: &mut pac::RCC) -> SubGhz<SgMiso, SgMosi> {
        let spi: Spi3<SgMiso, SgMosi> = Spi3::new(spi, baud_rate(rcc), rcc);
        unsafe { wakeup() };
        Self { spi }
    }

    /// Steal the SubGHz peripheral from whatever is currently using it.
    ///
    /// This will **not** initialize the SPI bus (unlike [`new`]).
    ///
    /// # Safety
    ///
    /// This will create a new `SPI3` peripheral, bypassing the singleton checks
    /// that normally occur.
    /// You are responsible for ensuring that the radio has exclusive access to
    /// these peripherals.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::subghz::SubGhz;
    ///
    /// // ... setup happens here
    ///
    /// let sg = unsafe { SubGhz::steal() };
    /// ```
    ///
    /// [`new`]: SubGhz::new
    pub unsafe fn steal() -> SubGhz<SgMiso, SgMosi> {
        SubGhz { spi: Spi3::steal() }
    }
}

impl<MISO: DmaCh, MOSI: DmaCh> SubGhz<MISO, MOSI> {
    /// Create a new sub-GHz radio driver from a peripheral and two DMA
    /// channels.
    ///
    /// This will reset the radio and the SPI bus, and enable the peripheral
    /// clock.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dma::AllDma, pac, subghz::SubGhz};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    ///
    /// let sg = SubGhz::new_with_dma(dp.SPI3, dma.d1.c1, dma.d2.c1, &mut dp.RCC);
    /// ```
    pub fn new_with_dma(
        spi: pac::SPI3,
        miso_dma: MISO,
        mosi_dma: MOSI,
        rcc: &mut pac::RCC,
    ) -> Self {
        pulse_reset(rcc);
        // safety: radio has been reset
        unsafe { Self::new_with_dma_no_reset(spi, miso_dma, mosi_dma, rcc) }
    }

    /// Create a new sub-GHz radio driver from a peripheral and two DMA
    /// channels.
    ///
    /// Same as `new_with_dma`, but without a reset.
    /// This is useful when waking up from standby to handle a radio interrupt.
    ///
    /// # Safety
    ///
    /// This will not reset the radio, the radio will be in an unknown state
    /// when constructed with this method.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wlxx_hal::{dma::AllDma, pac, subghz::SubGhz};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
    ///
    /// let sg = unsafe { SubGhz::new_with_dma_no_reset(dp.SPI3, dma.d1.c1, dma.d2.c1, &mut dp.RCC) };
    /// ```
    #[allow(unused_unsafe)]
    pub unsafe fn new_with_dma_no_reset(
        spi: pac::SPI3,
        miso_dma: MISO,
        mosi_dma: MOSI,
        rcc: &mut pac::RCC,
    ) -> Self {
        release_reset(rcc);
        let spi: Spi3<MISO, MOSI> =
            Spi3::new_with_dma(spi, miso_dma, mosi_dma, baud_rate(rcc), rcc);
        unsafe { wakeup() };
        SubGhz { spi }
    }

    /// Steal the SubGHz peripheral from whatever is currently using it.
    ///
    /// This will **not** initialize the SPI bus, or the DMA channels
    /// (unlike [`new_with_dma`]).
    ///
    /// # Safety
    ///
    /// This will create a new `SPI3` peripheral, bypassing the singleton checks
    /// that normally occur.
    /// You are responsible for ensuring that the radio has exclusive access to
    /// these peripherals.
    ///
    /// # Example
    ///
    /// ```
    /// use stm32wlxx_hal::{
    ///     dma::{AllDma, Dma1Ch1, Dma2Ch1},
    ///     subghz::SubGhz,
    /// };
    ///
    /// // ... setup happens here
    ///
    /// let sg: SubGhz<Dma1Ch1, Dma2Ch1> = unsafe {
    ///     let dma = AllDma::steal();
    ///     SubGhz::steal_with_dma(dma.d1.c1, dma.d2.c1)
    /// };
    /// ```
    ///
    /// [`new_with_dma`]: SubGhz::new_with_dma
    pub unsafe fn steal_with_dma(miso_dma: MISO, mosi_dma: MOSI) -> Self {
        SubGhz {
            spi: Spi3::steal_with_dma(miso_dma, mosi_dma),
        }
    }
}

// 5.8.2
/// Synchronous buffer access commands
impl<MISO, MOSI> SubGhz<MISO, MOSI>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = Error>
        + embedded_hal::blocking::spi::Write<u8, Error = Error>,
{
    /// Write the radio buffer at the given offset.
    pub fn write_buffer(&mut self, offset: u8, data: &[u8]) -> Result<(), Error> {
        self.poll_not_busy();
        {
            let _nss: Nss = Nss::new();
            self.spi.write(&[OpCode::WriteBuffer as u8, offset])?;
            self.spi.write(data)?;
        }
        self.poll_not_busy();

        Ok(())
    }

    /// Read the radio buffer at the given offset.
    ///
    /// The offset and length of a received packet is provided by
    /// [`rx_buffer_status`](Self::rx_buffer_status).
    pub fn read_buffer(&mut self, offset: u8, buf: &mut [u8]) -> Result<Status, Error> {
        let mut status_buf: [u8; 1] = [0];

        self.poll_not_busy();
        {
            let _nss: Nss = Nss::new();
            self.spi.write(&[OpCode::ReadBuffer as u8, offset])?;
            self.spi.transfer(&mut status_buf)?;
            self.spi.transfer(buf)?;
        }
        self.poll_not_busy();

        Ok(status_buf[0].into())
    }
}

// helper to pack register writes into a single buffer to avoid multiple DMA
// transfers
macro_rules! wr_reg {
    [$reg:ident, $($data:expr),+] => {
        &[
            OpCode::WriteRegister as u8,
            Register::$reg.address().to_be_bytes()[0],
            Register::$reg.address().to_be_bytes()[1],
            $($data),+
        ]
    };
}

// 5.8.2
/// Register access
impl<MISO, MOSI> SubGhz<MISO, MOSI>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = Error>
        + embedded_hal::blocking::spi::Write<u8, Error = Error>,
{
    // register write with variable length data
    fn write_register(&mut self, register: Register, data: &[u8]) -> Result<(), Error> {
        let addr: [u8; 2] = register.address().to_be_bytes();

        self.poll_not_busy();
        {
            let _nss: Nss = Nss::new();
            self.spi
                .write(&[OpCode::WriteRegister as u8, addr[0], addr[1]])?;
            self.spi.write(data)?;
        }
        self.poll_not_busy();

        Ok(())
    }

    /// Set the LoRa bit synchronization.
    pub fn set_bit_sync(&mut self, bs: BitSync) -> Result<(), Error> {
        self.write(wr_reg![GBSYNC, bs.as_bits()])
    }

    /// Set the generic packet control register.
    pub fn set_pkt_ctrl(&mut self, pkt_ctrl: PktCtrl) -> Result<(), Error> {
        self.write(wr_reg![GPKTCTL1A, pkt_ctrl.as_bits()])
    }

    /// Set the initial value for generic packet whitening.
    ///
    /// This sets the first 8 bits, the 9th bit is set with
    /// [`set_pkt_ctrl`](Self::set_pkt_ctrl).
    pub fn set_init_whitening(&mut self, init: u8) -> Result<(), Error> {
        self.write(wr_reg![GWHITEINIRL, init])
    }

    /// Set the initial value for generic packet CRC polynomial.
    pub fn set_crc_polynomial(&mut self, polynomial: u16) -> Result<(), Error> {
        let bytes: [u8; 2] = polynomial.to_be_bytes();
        self.write(wr_reg![GCRCINIRH, bytes[0], bytes[1]])
    }

    /// Set the generic packet CRC polynomial.
    pub fn set_initial_crc_polynomial(&mut self, polynomial: u16) -> Result<(), Error> {
        let bytes: [u8; 2] = polynomial.to_be_bytes();
        self.write(wr_reg![GCRCPOLRH, bytes[0], bytes[1]])
    }

    /// Set the synchronization word registers.
    pub fn set_sync_word(&mut self, sync_word: &[u8; 8]) -> Result<(), Error> {
        self.write_register(Register::GSYNC7, sync_word)
    }

    /// Set the LoRa synchronization word registers.
    pub fn set_lora_sync_word(&mut self, sync_word: LoRaSyncWord) -> Result<(), Error> {
        let bytes: [u8; 2] = sync_word.bytes();
        self.write(wr_reg![LSYNCH, bytes[0], bytes[1]])
    }

    /// Set the RX gain control.
    pub fn set_rx_gain(&mut self, pmode: PMode) -> Result<(), Error> {
        self.write(wr_reg![RXGAINC, pmode as u8])
    }

    /// Set the power amplifier over current protection.
    pub fn set_pa_ocp(&mut self, ocp: Ocp) -> Result<(), Error> {
        self.write(wr_reg![PAOCP, ocp as u8])
    }

    /// Restart the radio RTC.
    ///
    /// This is used to workaround an erratum for [`set_rx_duty_cycle`].
    ///
    /// [`set_rx_duty_cycle`]: crate::subghz::SubGhz::set_rx_duty_cycle
    pub fn restart_rtc(&mut self) -> Result<(), Error> {
        self.write(wr_reg![RTCCTLR, 0b1])
    }

    /// Set the radio real-time-clock period.
    ///
    /// This is used to workaround an erratum for [`set_rx_duty_cycle`].
    ///
    /// [`set_rx_duty_cycle`]: crate::subghz::SubGhz::set_rx_duty_cycle
    pub fn set_rtc_period(&mut self, period: Timeout) -> Result<(), Error> {
        let tobits: u32 = period.into_bits();
        self.write(wr_reg![
            RTCPRDR2,
            (tobits >> 16) as u8,
            (tobits >> 8) as u8,
            tobits as u8
        ])
    }

    /// Set the HSE32 crystal OSC_IN load capacitor trimming.
    pub fn set_hse_in_trim(&mut self, trim: HseTrim) -> Result<(), Error> {
        self.write(wr_reg![HSEINTRIM, trim.into()])
    }

    /// Set the HSE32 crystal OSC_OUT load capacitor trimming.
    pub fn set_hse_out_trim(&mut self, trim: HseTrim) -> Result<(), Error> {
        self.write(wr_reg![HSEOUTTRIM, trim.into()])
    }

    /// Set the SMPS clock detection enabled.
    ///
    /// SMPS clock detection must be enabled fore enabling the SMPS.
    pub fn set_smps_clock_det_en(&mut self, en: bool) -> Result<(), Error> {
        self.write(wr_reg![SMPSC0, (en as u8) << 6])
    }

    /// Set the power current limiting.
    pub fn set_pwr_ctrl(&mut self, pwr_ctrl: PwrCtrl) -> Result<(), Error> {
        self.write(wr_reg![PC, pwr_ctrl.as_bits()])
    }

    /// Set the maximum SMPS drive capability.
    pub fn set_smps_drv(&mut self, drv: SmpsDrv) -> Result<(), Error> {
        self.write(wr_reg![SMPSC2, (drv as u8) << 1])
    }

    /// Set the node address.
    ///
    /// Used with [`GenericPacketParams::set_addr_comp`] to filter packets based
    /// on node address.
    pub fn set_node_addr(&mut self, addr: u8) -> Result<(), Error> {
        self.write(wr_reg![NODE, addr])
    }

    /// Set the broadcast address.
    ///
    /// Used with [`GenericPacketParams::set_addr_comp`] to filter packets based
    /// on broadcast address.
    pub fn set_broadcast_addr(&mut self, addr: u8) -> Result<(), Error> {
        self.write(wr_reg![BROADCAST, addr])
    }

    /// Set both the broadcast address and node address.
    ///
    /// This is a combination of [`set_node_addr`] and [`set_broadcast_addr`]
    /// in a single SPI transfer.
    ///
    /// [`set_node_addr`]: Self::set_node_addr
    /// [`set_broadcast_addr`]: Self::set_broadcast_addr
    pub fn set_addrs(&mut self, node: u8, broadcast: u8) -> Result<(), Error> {
        self.write(wr_reg![NODE, node, broadcast])
    }
}

// 5.8.3
/// Operating mode commands
impl<MISO, MOSI> SubGhz<MISO, MOSI>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = Error>
        + embedded_hal::blocking::spi::Write<u8, Error = Error>,
{
    /// Put the radio into sleep mode.
    ///
    /// This command is only accepted in standby mode.
    /// The cfg argument allows some optional functions to be maintained
    /// in sleep mode.
    ///
    /// # Safety
    ///
    /// 1. After the `set_sleep` command, the sub-GHz radio NSS must not go low
    ///    for 500 μs.
    ///    No reason is provided, the reference manual (RM0453 rev 2) simply
    ///    says "you must".
    /// 2. The radio cannot be used while in sleep mode.
    /// 3. The radio must be woken up with [`wakeup`] before resuming use.
    ///
    /// # Example
    ///
    /// Put the radio into sleep mode.
    ///
    /// ```no_run
    /// # let cp = unsafe { stm32wlxx_hal::pac::CorePeripherals::steal() };
    /// # let dp = unsafe { stm32wlxx_hal::pac::Peripherals::steal() };
    /// # let mut sg = unsafe { stm32wlxx_hal::subghz::SubGhz::steal() };
    /// # let mut delay = new_delay(cp.SYST, &dp.RCC);
    /// use stm32wlxx_hal::{
    ///     subghz::{wakeup, SleepCfg, StandbyClk},
    ///     util::new_delay,
    /// };
    ///
    /// sg.set_standby(StandbyClk::Rc)?;
    /// unsafe { sg.set_sleep(SleepCfg::default())? };
    /// delay.delay_us(500);
    /// unsafe { wakeup() };
    /// # Ok::<(), stm32wlxx_hal::subghz::Error>(())
    /// ```
    pub unsafe fn set_sleep(&mut self, cfg: SleepCfg) -> Result<(), Error> {
        // poll for busy before, but not after
        // radio idles with busy high while in sleep mode
        self.poll_not_busy();
        {
            let _nss: Nss = Nss::new();
            self.spi.write(&[OpCode::SetSleep as u8, u8::from(cfg)])?;
        }
        Ok(())
    }

    /// Put the radio into standby mode.
    pub fn set_standby(&mut self, standby_clk: StandbyClk) -> Result<(), Error> {
        self.write(&[OpCode::SetStandby as u8, u8::from(standby_clk)])
    }

    /// Put the subghz radio into frequency synthesis mode.
    ///
    /// The RF-PLL frequency must be set with [`set_rf_frequency`] before using
    /// this command.
    ///
    /// Check the datasheet for more information, this is a test command but
    /// I honestly do not see any use for it.  Please update this description
    /// if you know more than I do.
    ///
    /// [`set_rf_frequency`]: crate::subghz::SubGhz::set_rf_frequency
    pub fn set_fs(&mut self) -> Result<(), Error> {
        self.write(&[OpCode::SetFs.into()])
    }

    /// Setup the sub-GHz radio for TX.
    pub fn set_tx(&mut self, timeout: Timeout) -> Result<(), Error> {
        let tobits: u32 = timeout.into_bits();
        self.write(&[
            OpCode::SetTx.into(),
            (tobits >> 16) as u8,
            (tobits >> 8) as u8,
            tobits as u8,
        ])
    }

    /// Setup the sub-GHz radio for RX.
    pub fn set_rx(&mut self, timeout: Timeout) -> Result<(), Error> {
        let tobits: u32 = timeout.into_bits();
        self.write(&[
            OpCode::SetRx.into(),
            (tobits >> 16) as u8,
            (tobits >> 8) as u8,
            tobits as u8,
        ])
    }

    /// Allows selection of the receiver event which stops the RX timeout timer.
    pub fn set_rx_timeout_stop(&mut self, rx_timeout_stop: RxTimeoutStop) -> Result<(), Error> {
        self.write(&[
            OpCode::SetStopRxTimerOnPreamble.into(),
            rx_timeout_stop.into(),
        ])
    }

    /// Put the radio in non-continuous RX mode.
    ///
    /// This command must be sent in Standby mode.
    /// This command is only functional with FSK and LoRa packet type.
    ///
    /// The following steps are performed:
    /// 1. Save sub-GHz radio configuration.
    /// 2. Enter Receive mode and listen for a preamble for the specified `rx_period`.
    /// 3. Upon the detection of a preamble, the `rx_period` timeout is stopped
    ///    and restarted with the value 2 × `rx_period` + `sleep_period`.
    ///    During this new period, the sub-GHz radio looks for the detection of
    ///    a synchronization word when in (G)FSK modulation mode,
    ///    or a header when in LoRa modulation mode.
    /// 4. If no packet is received during the listen period defined by
    ///    2 × `rx_period` + `sleep_period`, the sleep mode is entered for a
    ///    duration of `sleep_period`. At the end of the receive period,
    ///    the sub-GHz radio takes some time to save the context before starting
    ///    the sleep period.
    /// 5. After the sleep period, a new listening period is automatically
    ///    started. The sub-GHz radio restores the sub-GHz radio configuration
    ///    and continuous with step 2.
    ///
    /// The listening mode is terminated in one of the following cases:
    /// * if a packet is received during the listening period: the sub-GHz radio
    ///   issues a [`RxDone`] interrupt and enters standby mode.
    /// * if [`set_standby`] is sent during the listening period or after the
    ///   sub-GHz has been requested to exit sleep mode by sub-GHz radio SPI NSS
    ///
    /// # Erratum
    ///
    /// When a preamble is detected the radio should restart the RX timeout
    /// with a value of 2 × `rx_period` + `sleep_period`.
    /// Instead the radio erroneously uses `sleep_period`.
    ///
    /// To workaround this use [`restart_rtc`] and [`set_rtc_period`] to
    /// reprogram the radio timeout to  2 × `rx_period` + `sleep_period`.
    ///
    /// Use code similar to this in the [`PreambleDetected`] interrupt handler.
    ///
    /// ```no_run
    /// # let rx_period: Timeout = Timeout::from_millis_sat(100);
    /// # let sleep_period: Timeout = Timeout::from_millis_sat(100);
    /// # let mut sg = unsafe { stm32wlxx_hal::subghz::SubGhz::steal() };
    /// use stm32wlxx_hal::subghz::Timeout;
    ///
    /// let period: Timeout = rx_period
    ///     .saturating_add(rx_period)
    ///     .saturating_add(sleep_period);
    ///
    /// sg.set_rtc_period(period)?;
    /// sg.restart_rtc()?;
    /// # Ok::<(), stm32wlxx_hal::subghz::Error>(())
    /// ```
    ///
    /// Please read the erratum for more details.
    ///
    /// [`PreambleDetected`]: crate::subghz::Irq::PreambleDetected
    /// [`restart_rtc`]: crate::subghz::SubGhz::restart_rtc
    /// [`RxDone`]: crate::subghz::Irq::RxDone
    /// [`set_rf_frequency`]: crate::subghz::SubGhz::set_rf_frequency
    /// [`set_rtc_period`]: crate::subghz::SubGhz::set_rtc_period
    /// [`set_standby`]: crate::subghz::SubGhz::set_standby
    pub fn set_rx_duty_cycle(
        &mut self,
        rx_period: Timeout,
        sleep_period: Timeout,
    ) -> Result<(), Error> {
        let rx_period_bits: u32 = rx_period.into_bits();
        let sleep_period_bits: u32 = sleep_period.into_bits();
        self.write(&[
            OpCode::SetRxDutyCycle.into(),
            (rx_period_bits >> 16) as u8,
            (rx_period_bits >> 8) as u8,
            rx_period_bits as u8,
            (sleep_period_bits >> 16) as u8,
            (sleep_period_bits >> 8) as u8,
            sleep_period_bits as u8,
        ])
    }

    /// Channel Activity Detection (CAD) with LoRa packets.
    ///
    /// The channel activity detection (CAD) is a specific LoRa operation mode,
    /// where the sub-GHz radio searches for a LoRa radio signal.
    /// After the search is completed, the Standby mode is automatically
    /// entered, CAD is done and IRQ is generated.
    /// When a LoRa radio signal is detected, the CAD detected IRQ is also
    /// generated.
    ///
    /// The length of the search must be configured with [`set_cad_params`]
    /// prior to calling `set_cad`.
    ///
    /// [`set_cad_params`]: crate::subghz::SubGhz::set_cad_params
    pub fn set_cad(&mut self) -> Result<(), Error> {
        self.write(&[OpCode::SetCad.into()])
    }

    /// Generate a continuous transmit tone at the RF-PLL frequency.
    ///
    /// The sub-GHz radio remains in continuous transmit tone mode until a mode
    /// configuration command is received.
    pub fn set_tx_continuous_wave(&mut self) -> Result<(), Error> {
        self.write(&[OpCode::SetTxContinuousWave as u8])
    }

    /// Generate an infinite preamble at the RF-PLL frequency.
    ///
    /// The preamble is an alternating 0s and 1s sequence in generic (G)FSK and
    /// (G)MSK modulations.
    /// The preamble is symbol 0 in LoRa modulation.
    /// The sub-GHz radio remains in infinite preamble mode until a mode
    /// configuration command is received.
    pub fn set_tx_continuous_preamble(&mut self) -> Result<(), Error> {
        self.write(&[OpCode::SetTxContinuousPreamble as u8])
    }
}

// 5.8.4
/// Radio configuration commands
impl<MISO, MOSI> SubGhz<MISO, MOSI>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = Error>
        + embedded_hal::blocking::spi::Write<u8, Error = Error>,
{
    /// Set the packet type (modulation scheme).
    pub fn set_packet_type(&mut self, packet_type: PacketType) -> Result<(), Error> {
        self.write(&[OpCode::SetPacketType as u8, packet_type as u8])
    }

    /// Get the packet type.
    pub fn packet_type(&mut self) -> Result<Result<PacketType, u8>, Error> {
        let pkt_type: [u8; 2] = self.read_n(OpCode::GetPacketType)?;
        Ok(PacketType::from_raw(pkt_type[1]))
    }

    /// Set the radio carrier frequency.
    pub fn set_rf_frequency(&mut self, freq: &RfFreq) -> Result<(), Error> {
        self.write(freq.as_slice())
    }

    /// Set the transmit output power and the PA ramp-up time.
    pub fn set_tx_params(&mut self, params: &TxParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Power amplifier configuration.
    ///
    /// Used to customize the maximum output power and efficiency.
    pub fn set_pa_config(&mut self, pa_config: &PaConfig) -> Result<(), Error> {
        self.write(pa_config.as_slice())
    }

    /// Operating mode to enter after a successful packet transmission or
    /// packet reception.
    pub fn set_tx_rx_fallback_mode(&mut self, fm: FallbackMode) -> Result<(), Error> {
        self.write(&[OpCode::SetTxRxFallbackMode as u8, fm as u8])
    }

    /// Set channel activity detection (CAD) parameters.
    pub fn set_cad_params(&mut self, params: &CadParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the data buffer base address for the packet handling in TX and RX.
    ///
    /// There is a single buffer for both TX and RX.
    /// The buffer is not memory mapped, it is accessed via the
    /// [`read_buffer`](SubGhz::read_buffer) and
    /// [`write_buffer`](SubGhz::write_buffer) methods.
    pub fn set_buffer_base_address(&mut self, tx: u8, rx: u8) -> Result<(), Error> {
        self.write(&[OpCode::SetBufferBaseAddress as u8, tx, rx])
    }

    /// Set the (G)FSK modulation parameters.
    pub fn set_fsk_mod_params(&mut self, params: &FskModParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the LoRa modulation parameters.
    pub fn set_lora_mod_params(&mut self, params: &LoRaModParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the BPSK modulation parameters.
    pub fn set_bpsk_mod_params(&mut self, params: &BpskModParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the generic (FSK) packet parameters.
    pub fn set_packet_params(&mut self, params: &GenericPacketParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the BPSK packet parameters.
    pub fn set_bpsk_packet_params(&mut self, params: &BpskPacketParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the LoRa packet parameters.
    pub fn set_lora_packet_params(&mut self, params: &LoRaPacketParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the number of LoRa symbols to be received before starting the
    /// reception of a LoRa packet.
    ///
    /// Packet reception is started after `n` + 1 symbols are detected.
    pub fn set_lora_symb_timeout(&mut self, n: u8) -> Result<(), Error> {
        self.write(&[OpCode::SetLoRaSymbTimeout.into(), n])
    }
}

// 5.8.5
/// Communication status and information commands
impl<MISO, MOSI> SubGhz<MISO, MOSI>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = Error>
        + embedded_hal::blocking::spi::Write<u8, Error = Error>,
{
    /// Get the radio status.
    ///
    /// The hardware (or documentation) appears to have many bugs where this
    /// will return reserved values.
    /// See this thread in the ST community for details: [link]
    ///
    /// [link]: https://community.st.com/s/question/0D53W00000hR9GQSA0/stm32wl55-getstatus-command-returns-reserved-cmdstatus
    pub fn status(&mut self) -> Result<Status, Error> {
        Ok(self.read_1(OpCode::GetStatus)?.into())
    }

    /// Get the RX buffer status.
    ///
    /// The return tuple is (status, payload_length, buffer_pointer).
    pub fn rx_buffer_status(&mut self) -> Result<(Status, u8, u8), Error> {
        let data: [u8; 3] = self.read_n(OpCode::GetRxBufferStatus)?;
        Ok((data[0].into(), data[1], data[2]))
    }

    /// Returns information on the last received (G)FSK packet.
    pub fn fsk_packet_status(&mut self) -> Result<FskPacketStatus, Error> {
        Ok(FskPacketStatus::from(self.read_n(OpCode::GetPacketStatus)?))
    }

    /// Returns information on the last received LoRa packet.
    pub fn lora_packet_status(&mut self) -> Result<LoRaPacketStatus, Error> {
        Ok(LoRaPacketStatus::from(
            self.read_n(OpCode::GetPacketStatus)?,
        ))
    }

    /// Get the instantaneous signal strength during packet reception.
    ///
    /// The units are in dbm.
    pub fn rssi_inst(&mut self) -> Result<(Status, Ratio<i16>), Error> {
        let data: [u8; 2] = self.read_n(OpCode::GetRssiInst)?;
        let status: Status = data[0].into();
        let rssi: Ratio<i16> = Ratio::new_raw(i16::from(data[1]), -2);

        Ok((status, rssi))
    }

    /// (G)FSK packet stats.
    pub fn fsk_stats(&mut self) -> Result<Stats<FskStats>, Error> {
        let data: [u8; 7] = self.read_n(OpCode::GetStats)?;
        Ok(Stats::from_raw_fsk(data))
    }

    /// LoRa packet stats.
    pub fn lora_stats(&mut self) -> Result<Stats<LoRaStats>, Error> {
        let data: [u8; 7] = self.read_n(OpCode::GetStats)?;
        Ok(Stats::from_raw_lora(data))
    }

    /// Reset the stats as reported in [`lora_stats`](SubGhz::lora_stats) and
    /// [`fsk_stats`](SubGhz::fsk_stats).
    pub fn reset_stats(&mut self) -> Result<(), Error> {
        const RESET_STATS: [u8; 7] = [0x00; 7];
        self.write(&RESET_STATS)
    }
}

// 5.8.6
/// IRQ commands
impl<MISO, MOSI> SubGhz<MISO, MOSI>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = Error>
        + embedded_hal::blocking::spi::Write<u8, Error = Error>,
{
    /// Set the interrupt configuration.
    pub fn set_irq_cfg(&mut self, cfg: &CfgIrq) -> Result<(), Error> {
        self.write(cfg.as_slice())
    }

    /// Get the IRQ status.
    pub fn irq_status(&mut self) -> Result<(Status, u16), Error> {
        let data: [u8; 3] = self.read_n(OpCode::GetIrqStatus)?;
        let irq_status: u16 = u16::from_be_bytes([data[1], data[2]]);
        Ok((data[0].into(), irq_status))
    }

    /// Clear the IRQ status.
    pub fn clear_irq_status(&mut self, mask: u16) -> Result<(), Error> {
        self.write(&[OpCode::ClrIrqStatus as u8, (mask >> 8) as u8, mask as u8])
    }
}

// 5.8.7
/// Miscellaneous commands
impl<MISO, MOSI> SubGhz<MISO, MOSI>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = Error>
        + embedded_hal::blocking::spi::Write<u8, Error = Error>,
{
    /// Calibrate one or several blocks at any time when in standby mode.
    pub fn calibrate(&mut self, cal: u8) -> Result<(), Error> {
        // bit 7 is reserved and must be kept at reset value.
        self.write(&[OpCode::Calibrate as u8, cal & 0x7F])
    }

    /// Calibrate the image at the given frequencies.
    ///
    /// Requires the radio to be in standby mode.
    pub fn calibrate_image(&mut self, cal: CalibrateImage) -> Result<(), Error> {
        self.write(&[OpCode::CalibrateImage as u8, cal.0, cal.1])
    }

    /// Set the radio power supply.
    pub fn set_regulator_mode(&mut self, reg_mode: RegMode) -> Result<(), Error> {
        self.write(&[OpCode::SetRegulatorMode as u8, reg_mode as u8])
    }

    /// Get the radio operational errors.
    pub fn op_error(&mut self) -> Result<(Status, u16), Error> {
        let data: [u8; 3] = self.read_n(OpCode::GetError)?;
        Ok((data[0].into(), u16::from_be_bytes([data[1], data[2]])))
    }

    /// Clear all errors as reported by [`op_error`](SubGhz::op_error).
    pub fn clear_error(&mut self) -> Result<(), Error> {
        self.write(&[OpCode::ClrError as u8, 0x00])
    }
}

// 5.8.8
/// Set TCXO mode command
impl<MISO, MOSI> SubGhz<MISO, MOSI>
where
    Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error = Error>
        + embedded_hal::blocking::spi::Write<u8, Error = Error>,
{
    /// Set the TCXO trim and HSE32 ready timeout.
    pub fn set_tcxo_mode(&mut self, tcxo_mode: &TcxoMode) -> Result<(), Error> {
        self.write(tcxo_mode.as_slice())
    }
}

/// sub-GHz radio opcodes.
///
/// See Table 41 "Sub-GHz radio SPI commands overview"
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub(crate) enum OpCode {
    Calibrate = 0x89,
    CalibrateImage = 0x98,
    CfgDioIrq = 0x08,
    ClrError = 0x07,
    ClrIrqStatus = 0x02,
    GetError = 0x17,
    GetIrqStatus = 0x12,
    GetPacketStatus = 0x14,
    GetPacketType = 0x11,
    GetRssiInst = 0x15,
    GetRxBufferStatus = 0x13,
    GetStats = 0x10,
    GetStatus = 0xC0,
    ReadBuffer = 0x1E,
    RegRegister = 0x1D,
    ResetStats = 0x00,
    SetBufferBaseAddress = 0x8F,
    SetCad = 0xC5,
    SetCadParams = 0x88,
    SetFs = 0xC1,
    SetLoRaSymbTimeout = 0xA0,
    SetModulationParams = 0x8B,
    SetPacketParams = 0x8C,
    SetPacketType = 0x8A,
    SetPaConfig = 0x95,
    SetRegulatorMode = 0x96,
    SetRfFrequency = 0x86,
    SetRx = 0x82,
    SetRxDutyCycle = 0x94,
    SetSleep = 0x84,
    SetStandby = 0x80,
    SetStopRxTimerOnPreamble = 0x9F,
    SetTcxoMode = 0x97,
    SetTx = 0x83,
    SetTxContinuousPreamble = 0xD2,
    SetTxContinuousWave = 0xD1,
    SetTxParams = 0x8E,
    SetTxRxFallbackMode = 0x93,
    WriteBuffer = 0x0E,
    WriteRegister = 0x0D,
}

impl From<OpCode> for u8 {
    fn from(opcode: OpCode) -> Self {
        opcode as u8
    }
}

#[repr(u16)]
#[allow(clippy::upper_case_acronyms)]
pub(crate) enum Register {
    /// Generic bit synchronization.
    GBSYNC = 0x06AC,
    /// Generic packet control.
    GPKTCTL1A = 0x06B8,
    /// Generic whitening.
    GWHITEINIRL = 0x06B9,
    /// Generic CRC initial.
    GCRCINIRH = 0x06BC,
    /// Generic CRC polynomial.
    GCRCPOLRH = 0x06BE,
    /// Generic synchronization word 7.
    GSYNC7 = 0x06C0,
    /// Node address.
    NODE = 0x06CD,
    /// Broadcast address.
    BROADCAST = 0x06CE,
    /// LoRa synchronization word MSB.
    LSYNCH = 0x0740,
    /// LoRa synchronization word LSB.
    #[allow(dead_code)]
    LSYNCL = 0x0741,
    /// Receiver gain control.
    RXGAINC = 0x08AC,
    /// PA over current protection.
    PAOCP = 0x08E7,
    /// RTC control.
    RTCCTLR = 0x0902,
    /// RTC period MSB.
    RTCPRDR2 = 0x0906,
    /// RTC period mid-byte.
    #[allow(dead_code)]
    RTCPRDR1 = 0x0907,
    /// RTC period LSB.
    #[allow(dead_code)]
    RTCPRDR0 = 0x0908,
    /// HSE32 OSC_IN capacitor trim.
    HSEINTRIM = 0x0911,
    /// HSE32 OSC_OUT capacitor trim.
    HSEOUTTRIM = 0x0912,
    /// SMPS control 0.
    SMPSC0 = 0x0916,
    /// Power control.
    PC = 0x091A,
    /// SMPS control 2.
    SMPSC2 = 0x0923,
}

impl Register {
    pub const fn address(self) -> u16 {
        self as u16
    }
}
