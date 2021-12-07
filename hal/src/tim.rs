//! Timers

use embedded_hal::blocking::delay::DelayUs;

use crate::pac;
use crate::rcc::apb1timx;

/// Timer for delays.
#[derive(Debug)]
pub struct Tim2 {
    tim2: pac::TIM2,
}

impl Tim2 {
    /// Constructs a new timer.
    pub fn new(tim2: pac::TIM2, rcc: &mut pac::RCC) -> Self {
        rcc.apb1enr1.modify(|_, w| w.tim2en().enabled());
        rcc.apb1enr1.read(); // wait for clock to be ready
        rcc.apb1rstr1.modify(|_, w| w.tim2rst().set_bit());
        rcc.apb1rstr1.modify(|_, w| w.tim2rst().clear_bit());

        // Prescaler for 1 microsecond delay
        tim2.psc.write(|w| {
            w.psc()
                .bits((apb1timx(rcc) / 1_000_000).to_integer() as u16 - 1)
        });

        tim2.cr1.write(|w| w.dir().up().cen().enabled());
        tim2.egr.write(|w| w.ug().set_bit());

        Tim2 { tim2 }
    }
}

impl DelayUs<u32> for Tim2 {
    fn delay_us(&mut self, us: u32) {
        // Write Auto-Reload Register (ARR)
        // Note: Make it impossible to set the ARR value to 0, since this
        // would cause an infinite loop.
        self.tim2.arr.write(|w| w.arr().bits(us.max(1)));

        // Trigger update event (UEV) in the event generation register (EGR)
        // in order to immediately apply the config
        self.tim2.cr1.modify(|_, w| w.urs().set_bit());
        self.tim2.egr.write(|w| w.ug().set_bit());
        self.tim2.cr1.modify(|_, w| w.urs().clear_bit());

        // Configure the counter in one-pulse mode (counter stops counting at
        // the next updateevent, clearing the CEN bit) and enable the counter.
        self.tim2.cr1.write(|w| w.opm().set_bit().cen().set_bit());

        // Wait for CEN bit to clear
        while self.tim2.cr1.read().cen().is_enabled() { /* wait */ }
    }
}
