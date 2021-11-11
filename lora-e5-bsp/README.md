# LoRa-E5 Board Support Package

Board support for the seeed LoRa-E5 development kit.

This crate extends the [stm32wlxx-hal] with board specific hardware, see that crate for more information.

## Flashing

This board is a pain to get working for the first time because the flash protection bits are set.

Check these resources to unlock the board for programming:

* [How to program a LoRa-E5](https://forum.seeedstudio.com/t/how-to-program-a-lora-e5/257491)
* [seeed-lora/LoRa-E5-LoRaWAN-End-Node](https://github.com/seeed-lora/LoRa-E5-LoRaWAN-End-Node#getting-started)

To flash this board with various rust utilities such as `probe-run`, `cargo-embed`, and `cargo-flash` remove the `--connected-under-reset` flag. This flag is required for the NUCLEO board, but will cause timeout errors with the LoRa-E5 development board.

Flashing binaries larger than 64k with `probe-rs` based tools will hard fault, see [#74] for more information.

[stm32wlxx-hal]: https://github.com/stm32-rs/stm32wlxx-hal
[#74]: https://github.com/stm32-rs/stm32wlxx-hal/issues/74
