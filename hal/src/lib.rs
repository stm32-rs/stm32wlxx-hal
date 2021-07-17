//! STM32WL HAL
#![cfg_attr(not(test), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]

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

pub mod aes;
pub mod dma;
pub mod gpio;
pub mod pka;
pub mod rcc;
pub mod rng;
pub mod spi;
pub mod subghz;

#[cfg(feature = "rt")]
#[cfg_attr(docsrs, doc(cfg(feature = "rt")))]
/// Startup code and minimal runtime for Cortex-M microcontrollers
pub use cortex_m_rt as rt;

/// Cortex-M CPU peripherals
pub use cortex_m as cm;
/// Embedded-hal trait abstractions
pub use embedded_hal as eh;
