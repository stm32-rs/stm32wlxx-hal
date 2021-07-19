//! Sub-GHz radio operating in the 150 - 960 MHz ISM band
//!
//! ## LoRa user notice
//!
//! The Sub-GHz radio may have an undocumented erratum, see this ST community
//! post for more information: [link]
//!
//! [link]: https://community.st.com/s/question/0D53W00000hR8kpSAC/stm32wl55-erratum-clairification

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
mod reg_mode;
mod rf_frequency;
mod rx_timeout_stop;
mod sleep_cfg;
mod standby_clk;
mod stats;
mod status;
mod tcxo_mode;
mod timeout;
mod tx_params;
mod value_error;

use crate::gpio::{
    pins,
    sealed::{SubGhzSpiMiso, SubGhzSpiMosi, SubGhzSpiNss, SubGhzSpiSck},
};
use crate::pac;
use crate::spi::{BaudDiv, Spi3};

pub use cad_params::{CadParams, ExitMode, NbCadSymbol};
pub use calibrate::{Calibrate, CalibrateImage};
pub use fallback_mode::FallbackMode;
pub use hse_trim::HseTrim;
pub use irq::{CfgDioIrq, Irq, IrqLine};
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
pub use reg_mode::RegMode;
pub use rf_frequency::RfFreq;
pub use rx_timeout_stop::RxTimeoutStop;
pub use sleep_cfg::{SleepCfg, Startup};
pub use standby_clk::StandbyClk;
pub use stats::{FskStats, LoRaStats, Stats};
pub use status::{CmdStatus, Status, StatusMode};
pub use tcxo_mode::{TcxoMode, TcxoTrim};
pub use timeout::Timeout;
pub use tx_params::{RampTime, TxParams};
pub use value_error::ValueError;

pub use num_rational;

use num_rational::Ratio;

use embedded_hal::blocking::spi::{Transfer, Write};

/// Passthrough for SPI errors (for now)
pub type Error = crate::spi::Error;

#[derive(Debug)]
struct DebugPins {
    a4: pins::A4,
    a5: pins::A5,
    a6: pins::A6,
    a7: pins::A7,
}

impl DebugPins {
    pub const fn new(a4: pins::A4, a5: pins::A5, a6: pins::A6, a7: pins::A7) -> DebugPins {
        DebugPins { a4, a5, a6, a7 }
    }

    pub const fn free(self) -> (pins::A4, pins::A5, pins::A6, pins::A7) {
        (self.a4, self.a5, self.a6, self.a7)
    }
}

/// Sub-GHz radio peripheral
#[derive(Debug)]
pub struct SubGhz {
    spi: Spi3,
    debug_pins: Option<DebugPins>,
}

impl SubGhz {
    /// Create a new sub-GHz radio driver from a peripheral.
    ///
    /// This will reset the radio and the SPI bus, and enable the peripheral
    /// clock.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{pac, subghz::SubGhz};
    ///
    /// let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    /// let mut rcc = dp.RCC;
    ///
    /// let sg = SubGhz::new(dp.SPI3, &mut rcc);
    /// ```
    pub fn new(spi: pac::SPI3, rcc: &mut pac::RCC) -> SubGhz {
        Self::enable_spi_clock(rcc);
        rcc.csr.modify(|_, w| w.rfrst().set_bit());
        rcc.csr.modify(|_, w| w.rfrst().clear_bit());

        let spi: Spi3 = Spi3::new(spi, BaudDiv::DIV2, rcc);

        Self::clear_nss();
        Self::set_nss();

        // clear RF busy
        unsafe { pac::Peripherals::steal() }
            .PWR
            .scr
            .write(|w| w.cwrfbusyf().set_bit());

        SubGhz {
            spi,
            debug_pins: None,
        }
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
    /// use stm32wl_hal::subghz::SubGhz;
    ///
    /// // ... setup happens here
    ///
    /// let sg = unsafe { SubGhz::steal() };
    /// ```
    ///
    /// [`new`]: SubGhz::new
    pub unsafe fn steal() -> SubGhz {
        SubGhz {
            spi: Spi3::steal(),
            debug_pins: None,
        }
    }

    pub fn free(self) -> pac::SPI3 {
        self.spi.free()
    }

    /// Disable the SPI3 (SubGHz SPI) clock.
    pub fn disable_spi_clock(rcc: &mut pac::RCC) {
        Spi3::disable_clock(rcc)
    }

    /// Enable the SPI3 (SubGHz SPI) clock.
    pub fn enable_spi_clock(rcc: &mut pac::RCC) {
        Spi3::enable_clock(rcc)
    }

    /// Enable debug of the SubGHz SPI bus over physical pins.
    ///
    /// * A4: NSS
    /// * A5: SCK
    /// * A6: MISO
    /// * A7: MOSI
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{gpio::PortA, pac, subghz::SubGhz};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let mut gpioa = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let mut sg = SubGhz::new(dp.SPI3, &mut dp.RCC);
    /// sg.enable_spi_debug(gpioa.pa4, gpioa.pa5, gpioa.pa6, gpioa.pa7);
    /// ```
    pub fn enable_spi_debug(
        &mut self,
        mut a4: pins::A4,
        mut a5: pins::A5,
        mut a6: pins::A6,
        mut a7: pins::A7,
    ) {
        a4.set_subghz_spi_nss_af();
        a5.set_subghz_spi_sck_af();
        a6.set_subghz_spi_miso_af();
        a7.set_subghz_spi_mosi_af();
        self.debug_pins = Some(DebugPins::new(a4, a5, a6, a7))
    }

    /// Disable debug of the SubGHz SPI bus over physical pins.
    ///
    /// This will return `None` if debug was not previously enabled with
    /// [`enable_spi_debug`](crate::subghz::SubGhz::enable_spi_debug).
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{gpio::PortA, pac, subghz::SubGhz};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let mut gpioa = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let mut sg = SubGhz::new(dp.SPI3, &mut dp.RCC);
    /// sg.enable_spi_debug(gpioa.pa4, gpioa.pa5, gpioa.pa6, gpioa.pa7);
    ///
    /// let (a4, a5, a6, a7) = sg.disable_spi_debug().unwrap();
    /// ```
    pub fn disable_spi_debug(&mut self) -> Option<(pins::A4, pins::A5, pins::A6, pins::A7)> {
        self.debug_pins.take().and_then(|f| Some(f.free()))
    }

    /// Return `true` if debug of the SubGHz SPI bus over physical pins is
    /// enabled.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use stm32wl_hal::{gpio::PortA, pac, subghz::SubGhz};
    ///
    /// let mut dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    ///
    /// let mut gpioa = PortA::split(dp.GPIOA, &mut dp.RCC);
    /// let mut sg = SubGhz::new(dp.SPI3, &mut dp.RCC);
    ///
    /// assert!(!sg.spi_debug_enabled());
    /// sg.enable_spi_debug(gpioa.pa4, gpioa.pa5, gpioa.pa6, gpioa.pa7);
    /// assert!(sg.spi_debug_enabled());
    /// ```
    pub fn spi_debug_enabled(&self) -> bool {
        self.debug_pins.is_some()
    }

    /// Returns `true` if RFBUSYS is high.
    ///
    /// This indicates the radio is busy.
    ///
    /// See section 6.3 "Radio busy management" for more details.
    pub fn rfbusys(&self) -> bool {
        let dp = unsafe { pac::Peripherals::steal() };
        dp.PWR.sr2.read().rfbusys().bit_is_set()
    }

    fn poll_not_busy(&self) {
        // TODO: this is a terrible timeout
        let mut count: u32 = 1_000_000;
        while self.rfbusys() {
            count -= 1;
            if count == 0 {
                let dp = unsafe { pac::Peripherals::steal() };
                panic!(
                    "pwr.sr2=0x{:X} pwr.subghzspicr=0x{:X} pwr.cr1=0x{:X}",
                    dp.PWR.sr2.read().bits(),
                    dp.PWR.subghzspicr.read().bits(),
                    dp.PWR.cr1.read().bits(),
                );
            }
        }
    }

    /// Clear NSS, enabling SPI transactions
    #[inline(always)]
    fn clear_nss() {
        unsafe { pac::Peripherals::steal() }
            .PWR
            .subghzspicr
            .write(|w| w.nss().clear_bit())
    }

    /// Set NSS, disabling SPI transactions
    #[inline(always)]
    fn set_nss() {
        unsafe { pac::Peripherals::steal() }
            .PWR
            .subghzspicr
            .write(|w| w.nss().set_bit())
    }

    /// Helper to ensure NSS is always set high; even upon an error
    #[inline(always)]
    fn with_nss<F>(&mut self, mut f: F) -> Result<(), Error>
    where
        F: FnMut(&mut Spi3) -> Result<(), Error>,
    {
        Self::clear_nss();
        let result = f(&mut self.spi);
        Self::set_nss();
        result
    }

    fn read(&mut self, opcode: OpCode, data: &mut [u8]) -> Result<(), Error> {
        self.poll_not_busy();

        self.with_nss(|spi| {
            spi.write(&[opcode as u8])?;
            spi.transfer(data)?;
            Ok(())
        })?;

        self.poll_not_busy();

        Ok(())
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Error> {
        self.poll_not_busy();
        self.with_nss(|spi| spi.write(data))?;
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

// 5.8.2
/// Buffer access commands.
impl SubGhz {
    pub fn write_buffer(&mut self, offset: u8, data: &[u8]) -> Result<(), Error> {
        self.poll_not_busy();

        self.with_nss(|spi| {
            spi.write(&[OpCode::WriteBuffer as u8, offset])?;
            spi.write(data)?;
            Ok(())
        })?;

        self.poll_not_busy();
        Ok(())
    }

    pub fn read_buffer(&mut self, offset: u8, buf: &mut [u8]) -> Result<Status, Error> {
        let mut status_buf: [u8; 1] = [0];
        self.poll_not_busy();

        self.with_nss(|spi| {
            spi.write(&[OpCode::ReadBuffer as u8, offset])?;
            spi.transfer(&mut status_buf)?;
            spi.transfer(buf)?;
            Ok(())
        })?;

        self.poll_not_busy();
        Ok(status_buf[0].into())
    }
}

// 5.8.2
/// Register access.
impl SubGhz {
    fn write_register(&mut self, register: Register, data: &[u8]) -> Result<(), Error> {
        let addr: [u8; 2] = register.address().to_be_bytes();
        self.poll_not_busy();

        self.with_nss(|spi| {
            spi.write(&[OpCode::WriteRegister as u8, addr[0], addr[1]])?;
            spi.write(data)?;
            Ok(())
        })?;

        self.poll_not_busy();
        Ok(())
    }

    /// Set the initial value for generic packet whitening.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// sg.set_initial_whitening(0xA5);
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_initial_whitening(&mut self, init: u8) -> Result<(), Error> {
        self.write_register(Register::GWHITEINIRL, &[init])
    }

    /// Set the initial value for generic packet CRC polynomial.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// sg.set_crc_polynomial(0x1D0F);
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_crc_polynomial(&mut self, polynomial: u16) -> Result<(), Error> {
        self.write_register(Register::GCRCINIRH, &polynomial.to_be_bytes())
    }

    /// Set the generic packet CRC polynomial.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// sg.set_initial_crc_polynomial(0x1021);
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_initial_crc_polynomial(&mut self, polynomial: u16) -> Result<(), Error> {
        self.write_register(Register::GCRCPOLRH, &polynomial.to_be_bytes())
    }

    /// Set the synchronization word registers.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// const SYNC_WORD: [u8; 8] = [0x79, 0x80, 0x0C, 0xC0, 0x29, 0x95, 0xF8, 0x4A];
    ///
    /// sg.set_sync_word(&SYNC_WORD);
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_sync_word(&mut self, sync_word: &[u8; 8]) -> Result<(), Error> {
        self.write_register(Register::GSYNC7, sync_word)
    }

    /// Set the LoRa synchronization word registers.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{LoRaSyncWord, PacketType};
    ///
    /// sg.set_packet_type(PacketType::LoRa)?;
    /// sg.set_lora_sync_word(LoRaSyncWord::Public)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_lora_sync_word(&mut self, sync_word: LoRaSyncWord) -> Result<(), Error> {
        self.write_register(Register::LSYNCH, &sync_word.bytes())
    }

    /// Set the power amplifier over current protection.
    ///
    /// # Example
    ///
    /// Maximum 60mA for LP PA mode.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::Ocp;
    ///
    /// sg.set_pa_ocp(Ocp::Max60m)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// Maximum 60mA for HP PA mode.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::Ocp;
    ///
    /// sg.set_pa_ocp(Ocp::Max140m)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_pa_ocp(&mut self, ocp: Ocp) -> Result<(), Error> {
        self.write_register(Register::PAOCP, &[ocp as u8])
    }

    /// Set the HSE32 crystal OSC_IN load capaitor trimming.
    ///
    /// # Example
    ///
    /// Set the trim to the lowest value.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::HseTrim;
    ///
    /// sg.set_hse_in_trim(HseTrim::MIN);
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_hse_in_trim(&mut self, trim: HseTrim) -> Result<(), Error> {
        self.write_register(Register::HSEINTRIM, &[trim.into()])
    }

    /// Set the HSE32 crystal OSC_OUT load capaitor trimming.
    ///
    /// # Example
    ///
    /// Set the trim to the lowest value.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::HseTrim;
    ///
    /// sg.set_hse_out_trim(HseTrim::MIN);
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_hse_out_trim(&mut self, trim: HseTrim) -> Result<(), Error> {
        self.write_register(Register::HSEOUTTRIM, &[trim.into()])
    }
}

// 5.8.3
/// Operating mode commands.
impl SubGhz {
    /// Put the radio into sleep mode.
    ///
    /// This command is only accepted in standby mode.
    /// The cfg argument allows some optional functions to be maintained
    /// in sleep mode.
    ///
    /// # Example
    ///
    /// Put the radio into sleep mode.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{SleepCfg, StandbyClk};
    ///
    /// sg.set_standby(StandbyClk::Rc)?;
    /// sg.set_sleep(SleepCfg::default())?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_sleep(&mut self, cfg: SleepCfg) -> Result<(), Error> {
        self.write(&[OpCode::SetSleep as u8, u8::from(cfg)])
    }

    /// Put the radio into standby mode.
    ///
    /// # Examples
    ///
    /// Put the radio into standby mode using the RC 13MHz clock.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::StandbyClk;
    ///
    /// sg.set_standby(StandbyClk::Rc)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// Put the radio into standby mode using the HSE32 clock.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::StandbyClk;
    ///
    /// sg.set_standby(StandbyClk::Hse32)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
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
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::RfFreq;
    ///
    /// sg.set_rf_frequency(&RfFreq::from_frequency(915_000_000))?;
    /// sg.set_fs()?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// [`set_rf_frequency`]: crate::subghz::SubGhz::set_rf_frequency
    pub fn set_fs(&mut self) -> Result<(), Error> {
        self.write(&[OpCode::SetFs.into()])
    }

    /// Set the sub-GHz radio in TX mode.
    ///
    /// # Example
    ///
    /// Transmit with no timeout.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::Timeout;
    ///
    /// sg.set_tx(Timeout::DISABLED)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_tx(&mut self, timeout: Timeout) -> Result<(), Error> {
        let tobits: u32 = timeout.into_bits();
        self.write(&[
            OpCode::SetTx.into(),
            ((tobits >> 16) & 0xFF) as u8,
            ((tobits >> 8) & 0xFF) as u8,
            (tobits & 0xFF) as u8,
        ])
    }

    /// Set the sub-GHz radio in RX mode.
    ///
    /// # Example
    ///
    /// Receive with a 1 second timeout.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use core::time::Duration;
    /// use stm32wl_hal::subghz::Timeout;
    ///
    /// sg.set_rx(Timeout::from_duration_sat(Duration::from_secs(1)))?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_rx(&mut self, timeout: Timeout) -> Result<(), Error> {
        let tobits: u32 = timeout.into_bits();
        self.write(&[
            OpCode::SetRx.into(),
            ((tobits >> 16) & 0xFF) as u8,
            ((tobits >> 8) & 0xFF) as u8,
            (tobits & 0xFF) as u8,
        ])
    }

    /// Allows selection of the receiver event which stops the RX timeout timer.
    ///
    /// # Example
    ///
    /// Set the RX timeout timer to stop on preamble detection.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::RxTimeoutStop;
    ///
    /// sg.set_rx_timeout_stop(RxTimeoutStop::Preamble)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
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
    ///    and restarted with the value 2 x `rx_period` + `sleep_period`.
    ///    During this new period, the sub-GHz radio looks for the detection of
    ///    a synchronization word when in (G)FSK modulation mode,
    ///    or a header when in LoRa modulation mode.
    /// 4. If no packet is received during the listen period defined by
    ///    2 x `rx_period` + `sleep_period`, the sleep mode is entered for a
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
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use core::time::Duration;
    /// use stm32wl_hal::subghz::{StandbyClk, Timeout};
    ///
    /// const RX_PERIOD: Timeout = Timeout::from_duration_sat(Duration::from_millis(100));
    /// const SLEEP_PERIOD: Timeout = Timeout::from_duration_sat(Duration::from_secs(1));
    ///
    /// sg.set_standby(StandbyClk::Rc)?;
    /// sg.set_rx_duty_cycle(RX_PERIOD, SLEEP_PERIOD)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// [`RxDone`]: crate::subghz::Irq::RxDone
    /// [`set_rf_frequency`]: crate::subghz::SubGhz::set_rf_frequency
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
            ((rx_period_bits >> 16) & 0xFF) as u8,
            ((rx_period_bits >> 8) & 0xFF) as u8,
            (rx_period_bits & 0xFF) as u8,
            ((sleep_period_bits >> 16) & 0xFF) as u8,
            ((sleep_period_bits >> 8) & 0xFF) as u8,
            (sleep_period_bits & 0xFF) as u8,
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
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use core::time::Duration;
    /// use stm32wl_hal::subghz::{CadParams, ExitMode, NbCadSymbol, StandbyClk, Timeout};
    ///
    /// const RX_PERIOD: Timeout = Timeout::from_duration_sat(Duration::from_millis(100));
    /// const SLEEP_PERIOD: Timeout = Timeout::from_duration_sat(Duration::from_secs(1));
    /// const CAD_PARAMS: CadParams = CadParams::new()
    ///     .set_num_symbol(NbCadSymbol::S4)
    ///     .set_det_peak(0x18)
    ///     .set_det_min(0x10)
    ///     .set_exit_mode(ExitMode::Standby);
    ///
    /// sg.set_standby(StandbyClk::Rc)?;
    /// sg.set_cad_params(&CAD_PARAMS)?;
    /// sg.set_cad()?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// [`set_cad_params`]: crate::subghz::SubGhz::set_cad_params
    pub fn set_cad(&mut self) -> Result<(), Error> {
        self.write(&[OpCode::SetCad.into()])
    }

    /// Generate a continuous transmit tone at the RF-PLL frequency.
    ///
    /// The sub-GHz radio remains in continuous transmit tone mode until a mode
    /// configuration command is received.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// sg.set_tx_continuous_wave()?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
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
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// sg.set_tx_continuous_preamble()?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_tx_continuous_preamble(&mut self) -> Result<(), Error> {
        self.write(&[OpCode::SetTxContinuousPreamble as u8])
    }
}

// 5.8.4
/// Radio configuration commands.
impl SubGhz {
    /// Set the packet type (modulation scheme).
    ///
    /// # Examples
    ///
    /// FSK (frequency shift keying):
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::PacketType;
    ///
    /// sg.set_packet_type(PacketType::Fsk)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// LoRa (long range):
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::PacketType;
    ///
    /// sg.set_packet_type(PacketType::LoRa)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// BPSK (binary phase shift keying):
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::PacketType;
    ///
    /// sg.set_packet_type(PacketType::Bpsk)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// MSK (minimum shift keying):
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::PacketType;
    ///
    /// sg.set_packet_type(PacketType::Msk)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_packet_type(&mut self, packet_type: PacketType) -> Result<(), Error> {
        self.write(&[OpCode::SetPacketType as u8, packet_type as u8])
    }

    /// Get the packet type.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::PacketType;
    ///
    /// sg.set_packet_type(PacketType::LoRa)?;
    /// assert_eq!(sg.packet_type()?, Ok(PacketType::LoRa));
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn packet_type(&mut self) -> Result<Result<PacketType, u8>, Error> {
        let pkt_type: [u8; 2] = self.read_n(OpCode::GetPacketType)?;
        Ok(PacketType::from_raw(pkt_type[1]))
    }

    /// Set the radio carrier frequency.
    ///
    /// # Example
    ///
    /// Set the frequency to 915MHz (Australia and North America).
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::RfFreq;
    ///
    /// sg.set_rf_frequency(&RfFreq::F915)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_rf_frequency(&mut self, freq: &RfFreq) -> Result<(), Error> {
        self.write(freq.as_slice())
    }

    /// Set the transmit output power and the PA ramp-up time.
    ///
    /// # Example
    ///
    /// Set the output power to +10 dBm (low power mode) and a ramp up time of
    /// 40 microseconds.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{PaConfig, PaSel, RampTime, TxParams};
    ///
    /// const TX_PARAMS: TxParams = TxParams::new()
    ///     .set_ramp_time(RampTime::Micros40)
    ///     .set_power(0x0D);
    /// const PA_CONFIG: PaConfig = PaConfig::new()
    ///     .set_pa(PaSel::Lp)
    ///     .set_pa_duty_cycle(0x1)
    ///     .set_hp_max(0x0);
    ///
    /// sg.set_pa_config(&PA_CONFIG)?;
    /// sg.set_tx_params(&TX_PARAMS)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_tx_params(&mut self, params: &TxParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Power amplifier configuation.
    ///
    /// Used to customize the maximum output power and efficiency.
    ///
    /// # Example
    ///
    /// Set the output power to +22 dBm (high power mode) and a ramp up time of
    /// 200 microseconds.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{PaConfig, PaSel, RampTime, TxParams};
    ///
    /// const TX_PARAMS: TxParams = TxParams::new()
    ///     .set_ramp_time(RampTime::Micros200)
    ///     .set_power(0x16);
    /// const PA_CONFIG: PaConfig = PaConfig::new()
    ///     .set_pa(PaSel::Hp)
    ///     .set_pa_duty_cycle(0x4)
    ///     .set_hp_max(0x7);
    ///
    /// sg.set_pa_config(&PA_CONFIG)?;
    /// sg.set_tx_params(&TX_PARAMS)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_pa_config(&mut self, pa_config: &PaConfig) -> Result<(), Error> {
        self.write(pa_config.as_slice())
    }

    /// Operating mode to enter after a successful packet transmission or
    /// packet reception.
    ///
    /// # Example
    ///
    /// Set the fallback mode to standby mode.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::FallbackMode;
    ///
    /// sg.set_tx_rx_fallback_mode(FallbackMode::Standby)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_tx_rx_fallback_mode(&mut self, fm: FallbackMode) -> Result<(), Error> {
        self.write(&[OpCode::SetTxRxFallbackMode.into(), fm.into()])
    }

    /// Set channel activity detection (CAD) parameters.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use core::time::Duration;
    /// use stm32wl_hal::subghz::{CadParams, ExitMode, NbCadSymbol, StandbyClk, Timeout};
    ///
    /// const RX_PERIOD: Timeout = Timeout::from_duration_sat(Duration::from_millis(100));
    /// const SLEEP_PERIOD: Timeout = Timeout::from_duration_sat(Duration::from_secs(1));
    /// const CAD_PARAMS: CadParams = CadParams::new()
    ///     .set_num_symbol(NbCadSymbol::S4)
    ///     .set_det_peak(0x18)
    ///     .set_det_min(0x10)
    ///     .set_exit_mode(ExitMode::Standby);
    ///
    /// sg.set_standby(StandbyClk::Rc)?;
    /// sg.set_cad_params(&CAD_PARAMS)?;
    /// sg.set_cad()?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_cad_params(&mut self, params: &CadParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the data buffer base address for the packet handling in TX and RX.
    ///
    /// There is a 256B TX buffer and a 256B RX buffer.
    /// These buffers are not memory mapped, they are accessed via the
    /// [`read_buffer`] and [`write_buffer`] methods.
    ///
    /// # Example
    ///
    /// Set the TX and RX buffer base to the start.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// sg.set_buffer_base_address(0, 0)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// [`read_buffer`]: SubGhz::read_buffer
    /// [`write_buffer`]: SubGhz::write_buffer
    pub fn set_buffer_base_address(&mut self, tx: u8, rx: u8) -> Result<(), Error> {
        self.write(&[OpCode::SetBufferBaseAddress as u8, tx, rx])
    }

    /// Set the (G)FSK modulation parameters.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{
    ///     FskBandwidth, FskBitrate, FskFdev, FskModParams, FskPulseShape, PacketType,
    /// };
    ///
    /// const BITRATE: FskBitrate = FskBitrate::from_bps(32_000);
    /// const PULSE_SHAPE: FskPulseShape = FskPulseShape::Bt03;
    /// const BW: FskBandwidth = FskBandwidth::Bw9;
    /// const FDEV: FskFdev = FskFdev::from_hertz(31_250);
    ///
    /// const MOD_PARAMS: FskModParams = FskModParams::new()
    ///     .set_bitrate(BITRATE)
    ///     .set_pulse_shape(PULSE_SHAPE)
    ///     .set_bandwidth(BW)
    ///     .set_fdev(FDEV);
    ///
    /// sg.set_packet_type(PacketType::Fsk)?;
    /// sg.set_fsk_mod_params(&MOD_PARAMS)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_fsk_mod_params(&mut self, params: &FskModParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the LoRa modulation parameters.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{
    ///     CodingRate, LoRaBandwidth, LoRaModParams, PacketType, SpreadingFactor,
    /// };
    ///
    /// const MOD_PARAMS: LoRaModParams = LoRaModParams::new()
    ///     .set_sf(SpreadingFactor::Sf7)
    ///     .set_bw(LoRaBandwidth::Bw125)
    ///     .set_cr(CodingRate::Cr45)
    ///     .set_ldro_en(false);
    ///
    /// sg.set_packet_type(PacketType::LoRa)?;
    /// sg.set_lora_mod_params(&MOD_PARAMS)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_lora_mod_params(&mut self, params: &LoRaModParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the BPSK modulation parameters.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{BpskModParams, FskBitrate, PacketType};
    ///
    /// const MOD_PARAMS: BpskModParams = BpskModParams::new().set_bitrate(FskBitrate::from_bps(600));
    ///
    /// sg.set_packet_type(PacketType::Bpsk)?;
    /// sg.set_bpsk_mod_params(&MOD_PARAMS)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_bpsk_mod_params(&mut self, params: &BpskModParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the generic (FSK) packet parameters.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{
    ///     AddrComp, CrcType, GenericPacketParams, HeaderType, PacketType, PreambleDetection,
    /// };
    ///
    /// const PKT_PARAMS: GenericPacketParams = GenericPacketParams::new()
    ///     .set_preamble_len(8)
    ///     .set_preamble_detection(PreambleDetection::Disabled)
    ///     .set_sync_word_len(2)
    ///     .set_addr_comp(AddrComp::Disabled)
    ///     .set_header_type(HeaderType::Fixed)
    ///     .set_payload_len(128)
    ///     .set_crc_type(CrcType::Byte2)
    ///     .set_whitening_enable(true);
    ///
    /// sg.set_packet_type(PacketType::Fsk)?;
    /// sg.set_packet_params(&PKT_PARAMS)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_packet_params(&mut self, params: &GenericPacketParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the BPSK packet parameters.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{BpskPacketParams, PacketType};
    ///
    /// sg.set_packet_type(PacketType::Bpsk)?;
    /// sg.set_bpsk_packet_params(&BpskPacketParams::new().set_payload_len(64))?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_bpsk_packet_params(&mut self, params: &BpskPacketParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the LoRa packet parameters.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{HeaderType, LoRaPacketParams, PacketType};
    ///
    /// const PKT_PARAMS: LoRaPacketParams = LoRaPacketParams::new()
    ///     .set_preamble_len(5 * 8)
    ///     .set_header_type(HeaderType::Fixed)
    ///     .set_payload_len(64)
    ///     .set_crc_en(true)
    ///     .set_invert_iq(true);
    ///
    /// sg.set_packet_type(PacketType::LoRa)?;
    /// sg.set_lora_packet_params(&PKT_PARAMS)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_lora_packet_params(&mut self, params: &LoRaPacketParams) -> Result<(), Error> {
        self.write(params.as_slice())
    }

    /// Set the number of LoRa symbols to be received before starting the
    /// reception of a LoRa packet.
    ///
    /// Packet reception is started after `n` + 1 symbols are detected.
    ///
    /// # Example
    ///
    /// Start reception after a single LoRa word is detected
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    ///
    /// // ... setup the radio for LoRa RX
    ///
    /// sg.set_lora_symb_timeout(0)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_lora_symb_timeout(&mut self, n: u8) -> Result<(), Error> {
        self.write(&[OpCode::SetLoRaSymbTimeout.into(), n])
    }
}

// 5.8.5
/// Communication status and information commands
impl SubGhz {
    /// Get the radio status.
    ///
    /// The hardware (or documentation) appears to have many bugs where this
    /// will return reserved values.
    /// See this thread in the ST community for details: [link]
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::Status;
    ///
    /// let status: Status = sg.status()?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// [link]: https://community.st.com/s/question/0D53W00000hR9GQSA0/stm32wl55-getstatus-command-returns-reserved-cmdstatus
    pub fn status(&mut self) -> Result<Status, Error> {
        Ok(self.read_1(OpCode::GetStatus)?.into())
    }

    /// Get the RX buffer status.
    ///
    /// The return tuple is (status, payload_length, buffer_pointer).
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{CmdStatus, Timeout};
    ///
    /// sg.set_rx(Timeout::DISABLED)?;
    /// loop {
    ///     let (status, len, ptr) = sg.rx_buffer_status()?;
    ///
    ///     if status.cmd() == Ok(CmdStatus::Avaliable) {
    ///         let mut buf: [u8; 256] = [0; 256];
    ///         let data: &mut [u8] = &mut buf[..usize::from(len)];
    ///         sg.read_buffer(ptr, data)?;
    ///         // ... do things with the data
    ///         break;
    ///     }
    /// }
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn rx_buffer_status(&mut self) -> Result<(Status, u8, u8), Error> {
        let data: [u8; 3] = self.read_n(OpCode::GetRxBufferStatus)?;
        Ok((data[0].into(), data[1], data[2]))
    }

    /// Returns information on the last received (G)FSK packet.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use std::fmt::Write;
    /// # let mut uart = String::new();
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{CmdStatus, Timeout};
    ///
    /// sg.set_rx(Timeout::DISABLED)?;
    /// loop {
    ///     let pkt_status = sg.fsk_packet_status()?;
    ///
    ///     if pkt_status.status().cmd() == Ok(CmdStatus::Avaliable) {
    ///         let rssi = pkt_status.rssi_avg();
    ///         writeln!(&mut uart, "Avg RSSI: {} dBm", rssi);
    ///         break;
    ///     }
    /// }
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn fsk_packet_status(&mut self) -> Result<FskPacketStatus, Error> {
        Ok(FskPacketStatus::from(self.read_n(OpCode::GetPacketStatus)?))
    }

    /// Returns information on the last received LoRa packet.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use std::fmt::Write;
    /// # let mut uart = String::new();
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{CmdStatus, Timeout};
    ///
    /// sg.set_rx(Timeout::DISABLED)?;
    /// loop {
    ///     let pkt_status = sg.lora_packet_status()?;
    ///
    ///     if pkt_status.status().cmd() == Ok(CmdStatus::Avaliable) {
    ///         let snr = pkt_status.snr_pkt();
    ///         writeln!(&mut uart, "SNR: {} dB", snr);
    ///         break;
    ///     }
    /// }
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn lora_packet_status(&mut self) -> Result<LoRaPacketStatus, Error> {
        Ok(LoRaPacketStatus::from(
            self.read_n(OpCode::GetPacketStatus)?,
        ))
    }

    /// Get the instantaneous signal strength during packet reception.
    ///
    /// The units are in dbm.
    ///
    /// # Example
    ///
    /// Log the instantaneous signal strength to UART.
    ///
    /// ```no_run
    /// # use std::fmt::Write;
    /// # let mut uart = String::new();
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{CmdStatus, Timeout};
    ///
    /// sg.set_rx(Timeout::DISABLED)?;
    /// let (_, rssi) = sg.rssi_inst()?;
    /// writeln!(&mut uart, "RSSI: {} dBm", rssi);
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn rssi_inst(&mut self) -> Result<(Status, Ratio<i16>), Error> {
        let data: [u8; 2] = self.read_n(OpCode::GetRssiInst)?;
        let status: Status = data[0].into();
        let rssi: Ratio<i16> = Ratio::new(i16::from(data[1]), -2);

        Ok((status, rssi))
    }

    /// (G)FSK packet stats.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{FskStats, Stats};
    ///
    /// let stats: Stats<FskStats> = sg.fsk_stats()?;
    /// // ... use stats
    /// sg.reset_stats();
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn fsk_stats(&mut self) -> Result<Stats<FskStats>, Error> {
        let data: [u8; 7] = self.read_n(OpCode::GetStats)?;
        Ok(Stats::from_raw_fsk(data))
    }

    /// LoRa packet stats.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{LoRaStats, Stats};
    ///
    /// let stats: Stats<LoRaStats> = sg.lora_stats()?;
    /// // ... use stats
    /// sg.reset_stats();
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn lora_stats(&mut self) -> Result<Stats<LoRaStats>, Error> {
        let data: [u8; 7] = self.read_n(OpCode::GetStats)?;
        Ok(Stats::from_raw_lora(data))
    }

    /// Reset the stats as reported in [`lora_stats`] and [`fsk_stats`].
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    ///
    /// sg.reset_stats();
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// [`lora_stats`]: crate::subghz::SubGhz::lora_stats
    /// [`fsk_stats`]: crate::subghz::SubGhz::fsk_stats
    pub fn reset_stats(&mut self) -> Result<(), Error> {
        const RESET_STATS: [u8; 7] = [0x00; 7];
        self.write(&RESET_STATS)
    }
}

// 5.8.6
/// IRQ commands.
impl SubGhz {
    /// Set the interrupt configuration.
    ///
    /// # Example
    ///
    /// Enable TX and timeout interrupts globally.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{CfgDioIrq, Irq, IrqLine};
    ///
    /// const IRQ_CFG: CfgDioIrq = CfgDioIrq::new()
    ///     .irq_enable(IrqLine::Global, Irq::TxDone)
    ///     .irq_enable(IrqLine::Global, Irq::Timeout);
    /// sg.set_irq_cfg(&IRQ_CFG)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_irq_cfg(&mut self, cfg: &CfgDioIrq) -> Result<(), Error> {
        self.write(cfg.as_slice())
    }

    /// Get the IRQ status.
    ///
    /// # Example
    ///
    /// Wait for TX to complete or time out.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::Irq;
    ///
    /// loop {
    ///     let (_, irq_status) = sg.irq_status()?;
    ///     if irq_status & Irq::TxDone.mask() != 0 {
    ///         // handle TX done
    ///         break;
    ///     }
    ///     if irq_status & Irq::Timeout.mask() != 0 {
    ///         // handle timeout
    ///         break;
    ///     }
    /// }
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn irq_status(&mut self) -> Result<(Status, u16), Error> {
        let data: [u8; 3] = self.read_n(OpCode::GetIrqStatus)?;
        let irq_status: u16 = u16::from_be_bytes([data[1], data[2]]);
        Ok((data[0].into(), irq_status))
    }

    /// Clear the IRQ status.
    ///
    /// # Example
    ///
    /// Clear the [`TxDone`] and [`RxDone`] interrupts.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::Irq;
    ///
    /// sg.clear_irq_status(Irq::TxDone.mask() | Irq::RxDone.mask())?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// [`TxDone`]: crate::subghz::Irq::TxDone
    /// [`RxDone`]: crate::subghz::Irq::RxDone
    pub fn clear_irq_status(&mut self, mask: u16) -> Result<(), Error> {
        self.write(&[
            OpCode::ClrIrqStatus as u8,
            ((mask >> 8) & 0xFF) as u8,
            (mask & 0xFF) as u8,
        ])
    }
}

// 5.8.7
/// Miscellaneous commands
impl SubGhz {
    /// Calibrate one or several blocks at any time when in standby mode.
    ///
    /// The blocks to calibrate are defined by `cal` argument.
    /// When the calibration is ongoing, BUSY is set.
    /// A falling edge on BUSY indicates the end of all enabled calibrations.
    ///
    /// This function will not poll for BUSY.
    ///
    /// # Example
    ///
    /// Calibrate the RC 13 MHz and PLL.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{Calibrate, StandbyClk};
    ///
    /// sg.set_standby(StandbyClk::Rc)?;
    /// sg.calibrate(Calibrate::Rc13M.mask() | Calibrate::Pll.mask())?;
    /// while sg.rfbusys() {
    ///     // ... insert some timeout code here
    /// }
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn calibrate(&mut self, cal: u8) -> Result<(), Error> {
        // bit 7 is reserved and must be kept at reset value.
        self.write(&[OpCode::Calibrate as u8, cal & 0x7F])
    }

    /// Calibrate the image at the given frequencies.
    ///
    /// Requires the radio to be in standby mode.
    ///
    /// # Example
    ///
    /// Calibrate the image for the 430 - 440 MHz ISM band.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::{CalibrateImage, StandbyClk};
    ///
    /// sg.set_standby(StandbyClk::Rc)?;
    /// sg.calibrate_image(CalibrateImage::ISM_430_440)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn calibrate_image(&mut self, cal: CalibrateImage) -> Result<(), Error> {
        self.write(&[OpCode::CalibrateImage as u8, cal.0, cal.1])
    }

    /// Set the radio power supply.
    ///
    /// # Examples
    ///
    /// Use the linear dropout regulator (LDO):
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::RegMode;
    ///
    /// sg.set_regulator_mode(RegMode::Ldo)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// Use the switch mode power supply (SPMS):
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::RegMode;
    ///
    /// sg.set_regulator_mode(RegMode::Smps)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn set_regulator_mode(&mut self, reg_mode: RegMode) -> Result<(), Error> {
        self.write(&[OpCode::SetRegulatorMode as u8, reg_mode as u8])
    }

    /// Get the radio operational errors.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::OpError;
    ///
    /// let (status, error_mask) = sg.op_error()?;
    /// if error_mask & OpError::PllLockError.mask() != 0 {
    ///     // ... handle PLL lock error
    /// }
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    pub fn op_error(&mut self) -> Result<(Status, u16), Error> {
        let data: [u8; 3] = self.read_n(OpCode::GetError)?;
        Ok((data[0].into(), u16::from_le_bytes([data[1], data[2]])))
    }

    /// Clear all errors as reported by [`op_error`].
    ///
    /// # Example
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use stm32wl_hal::subghz::OpError;
    ///
    /// let (status, error_mask) = sg.op_error()?;
    /// // ignore all errors
    /// if error_mask != 0 {
    ///     sg.clear_error()?;
    /// }
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
    ///
    /// [`op_error`]: crate::subghz::SubGhz::op_error
    pub fn clear_error(&mut self) -> Result<(), Error> {
        self.write(&[OpCode::ClrError as u8, 0x00])
    }
}

// 5.8.8
/// Set TCXO mode command
impl SubGhz {
    /// Set the TCXO trim and HSE32 ready timeout.
    ///
    /// # Example
    ///
    /// Setup the TCXO with 1.7V trim and a 10ms timeout.
    ///
    /// ```no_run
    /// # let mut sg = unsafe { stm32wl_hal::subghz::SubGhz::steal() };
    /// use core::time::Duration;
    /// use stm32wl_hal::subghz::{TcxoMode, TcxoTrim, Timeout};
    ///
    /// const TCXO_MODE: TcxoMode = TcxoMode::new()
    ///     .set_txco_trim(TcxoTrim::Volts1pt7)
    ///     .set_timeout(Timeout::from_duration_sat(Duration::from_millis(10)));
    /// sg.set_tcxo_mode(&TCXO_MODE)?;
    /// # Ok::<(), stm32wl_hal::subghz::Error>(())
    /// ```
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
#[allow(dead_code)]
#[allow(clippy::upper_case_acronyms)]
pub(crate) enum Register {
    /// Generic CRC initial.
    GCRCINIRH = 0x06BC,
    /// Generic CRC polynomial.
    GCRCPOLRH = 0x06BE,
    /// Generic whitening.
    GWHITEINIRL = 0x06B9,
    /// PA over current protection.
    PAOCP = 0x08E7,
    /// LoRa synchronization word MSB.
    LSYNCH = 0x0740,
    /// LoRa synchronization word LSB.
    LSYNCL = 0x0741,
    /// Generic synchronization word 7.
    GSYNC7 = 0x06C0,
    /// HSE32 OSC_IN capacitor trim.
    HSEINTRIM = 0x0911,
    /// HSE32 OSC_OUT capacitor trim.
    HSEOUTTRIM = 0x0912,
}

impl Register {
    pub const fn address(self) -> u16 {
        self as u16
    }
}
