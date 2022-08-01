# LoRa-E5 Board Support Package

Board support for the seeed LoRa-E5 development kit.

This crate extends the [stm32wlxx-hal] with board specific hardware, see that crate for more information.

## Usage

```toml
[dependencies.lora-e5-bsp]
version = "0.6.1"
features = [
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

This board is a pain to get working for the first time because the flash protection bits are set.

Check these resources to unlock the board for programming:

* [How to program a LoRa-E5](https://forum.seeedstudio.com/t/how-to-program-a-lora-e5/257491)
* [seeed-lora/LoRa-E5-LoRaWAN-End-Node](https://github.com/seeed-lora/LoRa-E5-LoRaWAN-End-Node#getting-started)
* [stm32wl-unlock](https://github.com/newAM/stm32wl-unlock/)

To flash this board with various rust utilities such as `probe-run`, `cargo-embed`, and `cargo-flash` remove the `--connected-under-reset` flag. This flag is required for the NUCLEO board, but will cause timeout errors with the LoRa-E5 development board.

⚠️ You must use recent versions of `probe-rs` based tools to avoid bugs with the STM32WL ⚠️

* `cargo-embed` >=0.12.0
* `cargo-flash` >=0.12.0
* `probe-run` >=0.3.1

[stm32wlxx-hal]: https://github.com/stm32-rs/stm32wlxx-hal
