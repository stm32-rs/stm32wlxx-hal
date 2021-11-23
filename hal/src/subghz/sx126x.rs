use embedded_hal::blocking::delay::DelayUs;
use lorawan_device::radio::{LoRaChannel, LoRaInfo, LoRaState};
use radio::{Busy, Channel, Receive, State, Transmit};

use crate::spi::Spi3;
use crate::subghz;
use crate::subghz::{Irq, SubGhz, Timeout};
use crate::subghz::rfs::{RfSwRx, RfSwTx};

/// Sx126x radio.
pub struct Sx126x<MISO, MOSI, RFS> {
    sg: SubGhz<MISO, MOSI>,
    rfs: RFS,
}

impl<MISO, MOSI, RFS> Sx126x<MISO, MOSI, RFS>
    where Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error=subghz::Error>,
          Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Write<u8, Error=subghz::Error>,
          RFS: RfSwRx + RfSwTx
{
    /// Creates a new Sx126x radio.
    pub fn new(sg: SubGhz<MISO, MOSI>, rfs: RFS) -> Self {
        // TODO: Setup
        Sx126x {
            sg,
            rfs,
        }
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
    where Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error=subghz::Error>,
          Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Write<u8, Error=subghz::Error>,
          RFS: RfSwRx + RfSwTx
{
    type Error = Error;

    fn start_transmit(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.sg.write_buffer(0, data)?;
        self.rfs.set_tx();
        self.sg.set_tx(Timeout::from_millis_sat(100))?;
        Ok(())
    }

    fn check_transmit(&mut self) -> Result<bool, Self::Error> {
        let (_, irq_status) = self.sg.irq_status()?;
        self.sg.clear_irq_status(irq_status)?;
        Ok(irq_status & Irq::TxDone.mask() != 0)
    }
}

impl<MISO, MOSI, RFS> Receive for Sx126x<MISO, MOSI, RFS>
    where Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Transfer<u8, Error=subghz::Error>,
          Spi3<MISO, MOSI>: embedded_hal::blocking::spi::Write<u8, Error=subghz::Error>,
          RFS: RfSwRx + RfSwTx
{
    type Error = Error;
    type Info = LoRaInfo;

    fn start_receive(&mut self) -> Result<(), Self::Error> {
        self.rfs.set_rx();
        self.sg.set_rx(Timeout::from_millis_sat(100))?;
        Ok(())
    }

    fn check_receive(&mut self, _: bool) -> Result<bool, Self::Error> {
        let (_, irq_status) = self.sg.irq_status()?;
        self.sg.clear_irq_status(irq_status)?;
        Ok(irq_status & Irq::RxDone.mask() != 0)
    }

    fn get_received(&mut self, buf: &mut [u8]) -> Result<(usize, Self::Info), Self::Error> {
        let (_, len, ptr) = self.sg.rx_buffer_status()?;
        let size = usize::from(len);
        let data: &mut [u8] = &mut buf[..size];
        self.sg.read_buffer(ptr, data)?;
        // TODO: get info
        let info = LoRaInfo::default();
        Ok((size, info))
    }
}

impl<MISO, MOSI, RFS> Channel for Sx126x<MISO, MOSI, RFS> {
    type Channel = LoRaChannel;
    type Error = Error;

    fn set_channel(&mut self, _channel: &Self::Channel) -> Result<(), Self::Error> {
        todo!()
    }
}

impl<MISO, MOSI, RFS> State for Sx126x<MISO, MOSI, RFS> {
    type State = LoRaState;
    type Error = Error;

    fn set_state(&mut self, _state: Self::State) -> Result<(), Self::Error> {
        todo!()
    }

    fn get_state(&mut self) -> Result<Self::State, Self::Error> {
        todo!()
    }
}

impl<MISO, MOSI, RFS> Busy for Sx126x<MISO, MOSI, RFS> {
    type Error = Error;

    fn is_busy(&mut self) -> Result<bool, Self::Error> {
        Ok(subghz::rfbusyms())
    }
}

impl<MISO, MOSI, RFS> DelayUs<u32> for Sx126x<MISO, MOSI, RFS> {
    fn delay_us(&mut self, _us: u32) {
        todo!()
    }
}

#[derive(Debug)]
pub enum Error {
    SubGhz(subghz::Error),
}

impl From<subghz::Error> for Error {
    fn from(err: subghz::Error) -> Self {
        Error::SubGhz(err)
    }
}
