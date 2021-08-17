//! STM32WL Hardware Abstraction Layer
#![cfg_attr(not(test), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]

#[cfg(any(
    all(feature = "stm32wl5x_cm0p", feature = "stm32wl5x_cm4"),
    all(feature = "stm32wl5x_cm0p", feature = "stm32wle5"),
    all(feature = "stm32wl5x_cm4", feature = "stm32wle5"),
))]
compile_error!("Multile chip features activated. You must activate exactly one of the following features: stm32wl5x_cm0p, stm32wl5x_cm4, stm32wle5");

cfg_if::cfg_if! {
    if #[cfg(feature = "stm32wl5x_cm0p")] {
        /// Peripheral access crate
        pub use stm32wl::stm32wl5x_cm0p as pac;
    } else if #[cfg(feature = "stm32wl5x_cm4")] {
        /// Peripheral access crate
        pub use stm32wl::stm32wl5x_cm4 as pac;
    } else if #[cfg(feature = "stm32wle5")] {
        /// Peripheral access crate
        pub use stm32wl::stm32wle5 as pac;
    } else {
        core::compile_error!("You must select your hardware with a feature flag");
    }
}

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

pub mod adc;
pub mod aes;
pub mod dac;
pub mod dma;
pub mod gpio;
pub mod i2c;
pub mod info;
pub mod pka;
pub mod rcc;
pub mod rng;
pub mod spi;
pub mod subghz;
pub mod uart;
pub mod util;

mod ratio;
pub use ratio::Ratio;

#[cfg(feature = "rt")]
#[cfg_attr(docsrs, doc(cfg(feature = "rt")))]
/// Startup code and minimal runtime for Cortex-M microcontrollers
pub use cortex_m_rt;

/// Cortex-M CPU peripherals
pub use cortex_m;
/// Embedded-hal trait abstractions
pub use embedded_hal;
