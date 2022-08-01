# NUCLEO-WL55JC Board Support Package

Board support for the NUCLEO-WL55JC development board.

This crate extends the [stm32wlxx-hal] with board specific hardware, see that crate for more information.

## Usage

```toml
[dependencies.nucleo-wl55jc-bsp]
version = "0.6.1"
features = [
    # required: build for core 1
    # This is future proofing for when the HAL has APIs for core 2
    "stm32wl5x_cm4",
    # optional: use the cortex-m-rt interrupt interface
    "rt",
    # optional: use defmt
    "defmt",
    # optional: enable conversions with embedded-time types
    "embedded-time",
    # optional: use the real time clock (RTC)
    "chrono",
]
```

## Flashing

To flash this board with various rust utilities such as `probe-run`, `cargo-embed`, and `cargo-flash` use the `--connected-under-reset` flag.

⚠️ You must use recent versions of `probe-rs` based tools to avoid bugs with the STM32WL ⚠️

* `cargo-embed` >=0.12.0
* `cargo-flash` >=0.12.0
* `probe-run` >=0.3.1

[stm32wlxx-hal]: https://github.com/stm32-rs/stm32wlxx-hal
