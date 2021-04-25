//! STM32WL HAL.
#![cfg_attr(not(test), no_std)]

#[cfg(feature = "stm32wl5x_cm0p")]
pub use stm32wl::stm32wl5x_cm0p as pac;

#[cfg(feature = "stm32wl5x_cm4")]
pub use stm32wl::stm32wl5x_cm4 as pac;

#[cfg(feature = "stm32wle5")]
pub use stm32wl::stm32wle5 as pac;

pub use stm32wl_hal_pka as pka;
pub use stm32wl_hal_subghz as subghz;
