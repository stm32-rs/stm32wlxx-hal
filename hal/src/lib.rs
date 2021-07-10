//! STM32WL HAL.
#![cfg_attr(not(test), no_std)]

cfg_if::cfg_if! {
    if #[cfg(feature = "stm32wl5x_cm0p")] {
        /// Peripheral access crate.
        pub use stm32wl::stm32wl5x_cm0p as pac;
    } else if #[cfg(feature = "stm32wl5x_cm4")] {
        /// Peripheral access crate.
        pub use stm32wl::stm32wl5x_cm4 as pac;
    } else if #[cfg(feature = "stm32wle5")] {
        /// Peripheral access crate.
        pub use stm32wl::stm32wle5 as pac;
    } else {
        core::compile_error!("You must select your hardware with a feature flag");
    }
}

pub mod aes;
pub mod gpio;
pub mod pka;
pub mod rcc;
pub mod rng;
pub mod spi;
pub mod subghz;

/// Cortex-M CPU peripherals.
pub use cortex_m;
/// Embedded-hal trait abstractions.
pub use embedded_hal;
