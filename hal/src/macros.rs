#![macro_use]

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
