use embedded_hal::blocking::delay::DelayUs;
use lorawan_device::radio::{LoRaChannel, LoRaInfo, LoRaState};
use radio::{Busy, Channel, Receive, State, Transmit};

use crate::subghz::{rfbusyms, SubGhz};
use crate::subghz::rfs::{RfSwRx, RfSwTx};

/// Sx126x radio.
pub struct Sx126x<MISO, MOSI, RFS> {
    subghz: SubGhz<MISO, MOSI>,
    rfs: RFS,
}

impl<MISO, MOSI, RFS> Sx126x<MISO, MOSI, RFS>
    where RFS: RfSwRx + RfSwTx
{
    /// Creates a new Sx126x radio.
    pub fn new(subghz: SubGhz<MISO, MOSI>, rfs: RFS) -> Self {
        Sx126x {
            subghz,
            rfs,
        }
    }

    /// Returns the internal Sub-GHz radio peripheral.
    pub fn as_subghz(&self) -> &SubGhz<MISO, MOSI> {
        &self.subghz
    }

    /// Returns a mutable reference to the internal Sub-GHz radio peripheral.
    pub fn as_mut_subghz(&mut self) -> &mut SubGhz<MISO, MOSI> {
        &mut self.subghz
    }
}

impl<MISO, MOSI, RFS> Transmit for Sx126x<MISO, MOSI, RFS>
    where RFS: RfSwRx + RfSwTx
{
    type Error = Error;

    fn start_transmit(&mut self, _data: &[u8]) -> Result<(), Self::Error> {
        self.rfs.set_tx();
        todo!()
    }

    fn check_transmit(&mut self) -> Result<bool, Self::Error> {
        todo!()
    }
}

impl<MISO, MOSI, RFS> Receive for Sx126x<MISO, MOSI, RFS>
    where RFS: RfSwRx + RfSwTx
{
    type Error = Error;
    type Info = LoRaInfo;

    fn start_receive(&mut self) -> Result<(), Self::Error> {
        self.rfs.set_rx();
        todo!()
    }

    fn check_receive(&mut self, _restart: bool) -> Result<bool, Self::Error> {
        todo!()
    }

    fn get_received(&mut self, _buff: &mut [u8]) -> Result<(usize, Self::Info), Self::Error> {
        todo!()
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
        Ok(rfbusyms())
    }
}

impl<MISO, MOSI, RFS> DelayUs<u32> for Sx126x<MISO, MOSI, RFS> {
    fn delay_us(&mut self, _us: u32) {
        todo!()
    }
}

#[derive(Debug)]
pub enum Error {}
