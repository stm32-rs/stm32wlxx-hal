use embedded_hal::blocking::delay::DelayUs;
use lorawan_device::radio::{LoRaChannel, LoRaInfo, LoRaState};
use radio::{Busy, Channel, Receive, State, Transmit};
use crate::subghz::{rfbusyms, SubGhz};

pub struct Sx126x<MISO, MOSI> {
    subghz: SubGhz<MISO, MOSI>,
}

impl<MISO, MOSI> Sx126x<MISO, MOSI> {
    pub fn new(subghz: SubGhz<MISO, MOSI>) -> Self {
        Sx126x { subghz }
    }

    pub fn as_subghz(&self) -> &SubGhz<MISO, MOSI> {
        &self.subghz
    }

    pub fn as_mut_subghz(&mut self) -> &mut SubGhz<MISO, MOSI> {
        &mut self.subghz
    }
}

impl<MISO, MOSI> Transmit for Sx126x<MISO, MOSI> {
    type Error = Error;

    fn start_transmit(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        todo!()
    }

    fn check_transmit(&mut self) -> Result<bool, Self::Error> {
        todo!()
    }
}

impl<MISO, MOSI> Receive for Sx126x<MISO, MOSI> {
    type Error = Error;
    type Info = LoRaInfo;

    fn start_receive(&mut self) -> Result<(), Self::Error> {
        todo!()
    }

    fn check_receive(&mut self, restart: bool) -> Result<bool, Self::Error> {
        todo!()
    }

    fn get_received(&mut self, buff: &mut [u8]) -> Result<(usize, Self::Info), Self::Error> {
        todo!()
    }
}

impl<MISO, MOSI> Channel for Sx126x<MISO, MOSI> {
    type Channel = LoRaChannel;
    type Error = Error;

    fn set_channel(&mut self, channel: &Self::Channel) -> Result<(), Self::Error> {
        todo!()
    }
}

impl<MISO, MOSI> State for Sx126x<MISO, MOSI> {
    type State = LoRaState;
    type Error = Error;

    fn set_state(&mut self, state: Self::State) -> Result<(), Self::Error> {
        todo!()
    }

    fn get_state(&mut self) -> Result<Self::State, Self::Error> {
        todo!()
    }
}

impl<MISO, MOSI> Busy for Sx126x<MISO, MOSI> {
    type Error = Error;

    fn is_busy(&mut self) -> Result<bool, Self::Error> {
        Ok(rfbusyms())
    }
}

impl<MISO, MOSI> DelayUs<u32> for Sx126x<MISO, MOSI> {
    fn delay_us(&mut self, us: u32) {
        todo!()
    }
}

#[derive(Debug)]
pub enum Error {}
