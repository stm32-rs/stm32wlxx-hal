//! Timers

use crate::pac;

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

        // 1 microsecond delay
        tim2.psc.write(|w| w.psc().bits(48 - 1));

        tim2.cr1.write(|w| w.dir().up().cen().enabled());
        tim2.egr.write(|w| w.ug().set_bit());

        Tim2 { tim2 }
    }

    /// Delays for `us` microseconds.
    pub fn delay_us(&mut self, us: u32) {
        // TODO: GitHub Copilot gave me this, does it work?
        self.tim2.cnt.write(|w| w.cnt().bits(0));
        self.tim2.arr.write(|w| w.arr().bits(us));
        self.tim2.egr.write(|w| w.ug().set_bit());
        while self.tim2.sr.read().uif().bit_is_clear() {}
    }
}
