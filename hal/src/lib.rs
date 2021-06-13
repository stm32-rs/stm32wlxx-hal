//! STM32WL HAL.
#![cfg_attr(not(test), no_std)]
#![forbid(missing_docs, unsafe_code)]

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

pub use stm32wl_hal_aes as aes;
pub use stm32wl_hal_gpio as gpio;
pub use stm32wl_hal_pka as pka;
pub use stm32wl_hal_rcc as rcc;
pub use stm32wl_hal_rng as rng;
pub use stm32wl_hal_subghz as subghz;
