# NUCLEO-WL55JC Board Support Package

Board support for the NUCLEO-WL55JC development board.

This crate extends the [stm32wlxx-hal] with board specific hardware, see that crate for more information.

## Flashing

To flash this board with various rust utilities such as `probe-run`, `cargo-embed`, and `cargo-flash` use the `--connected-under-reset` flag.

Flashing binaries larger than 64k with `probe-rs` based tools will hard fault, see [#74] for more information.

[stm32wlxx-hal]: https://github.com/stm32-rs/stm32wlxx-hal
[#74]: https://github.com/stm32-rs/stm32wlxx-hal/issues/74
