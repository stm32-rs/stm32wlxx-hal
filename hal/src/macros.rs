#![macro_use]
#![allow(unknown_lints, unused_macro_rules)]

macro_rules! typestate {
    ($name:ident, $doc:expr) => {
        paste::paste! {
            #[doc = "[Typestate] for " $doc "."]
            ///
            /// [Typestate]: https://docs.rust-embedded.org/book/static-guarantees/typestate-programming.html
            #[derive(Debug, PartialEq, Eq, Clone, Copy)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub struct $name {
                _priv: (),
            }
        }

        impl $name {
            #[allow(dead_code)]
            pub(crate) const fn new() -> Self {
                Self { _priv: () }
            }
        }
    };
    ($name:ident) => {
        paste::paste! {
            #[derive(Debug, PartialEq, Eq, Clone, Copy)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            struct $name {
                _priv: (),
            }
        }

        impl $name {
            #[allow(dead_code)]
            pub(crate) const fn new() -> Self {
                Self { _priv: () }
            }
        }
    };
}

// helper for conditional compilation
#[cfg(any(feature = "stm32wl5x_cm4", feature = "stm32wle5"))]
macro_rules! c1_c2 {
    ($c1:expr, $c2:expr) => {
        $c1
    };
    ($c1:expr, $c2:expr,) => {
        $c1
    };
}

#[cfg(feature = "stm32wl5x_cm0p")]
macro_rules! c1_c2 {
    ($c1:expr, $c2:expr) => {
        $c2
    };
    ($c1:expr, $c2:expr,) => {
        $c2
    };
}
