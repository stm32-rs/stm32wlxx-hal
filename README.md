# stm32wlxx-hal

[![CI](https://github.com/stm32-rs/stm32wlxx-hal/workflows/CI/badge.svg)](https://github.com/stm32-rs/stm32wlxx-hal/actions?query=branch%3Amain)
[![stable-docs](https://img.shields.io/badge/docs-stable-blue)](https://docs.rs/stm32wlxx-hal/)
[![nightly-docs](https://img.shields.io/badge/docs-nightly-black)](https://stm32-rs.github.io/stm32wlxx-hal/stm32wlxx_hal/index.html)
[![crates.io](https://img.shields.io/crates/v/stm32wlxx-hal.svg)](https://crates.io/crates/stm32wlxx-hal)
[![rustc](https://img.shields.io/badge/rustc-1.60+-blue.svg)](https://doc.rust-lang.org/cargo/reference/manifest.html#the-rust-version-field)

Embedded rust HAL (hardware abstraction layer) for the STM32WL series.

This is still in development, the code that exists today covers basic usage of:

* SubGHz LoRa TX + RX
* SubGHz (G)FSK TX + RX
* SPI
* GPIO
* UART
* I2C
* Low-power timers
* ADC
* DAC
* PKA ECDSA signing + verification
* Secure random number generation
* AES ECB encryption + decryption
* RTC date and time

## Usage

```toml
[dependencies.stm32wlxx-hal]
version = "0.6.1"
features = [
    # use exactly one to match your target hardware
    "stm32wl5x_cm0p",
    "stm32wl5x_cm4",
    "stm32wle5",
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

## Examples

All examples run on the NUCLEO-WL55JC2. Examples are located in the `examples` crate. The arguments got long for this, so a `run-ex` cargo alias is provided.

```bash
DEFMT_LOG=trace cargo run-ex gpio-blink
```

The on-target tests are also excellent reference material.

### System Level Example

The testsuites and examples are a good starting point, but they demonstrate features independent of each-other. A system-level example using multiple features simultaneously is provided in a separate repo: [stm32wl-lightswitch-demo](https://github.com/newAM/stm32wl-lightswitch-demo)

## Unit Tests

Off-target unit tests use the built-in cargo framework. You must specify the target device as a feature.

```bash
cargo test --features stm32wl5x_cm4
```

## On-Target Tests

See [testsuite/README.md](https://github.com/stm32-rs/stm32wlxx-hal/blob/main/testsuite/README.md).

## Reference Documentation

* [stm32wl5x reference manual](https://www.st.com/resource/en/reference_manual/rm0453-stm32wl5x-advanced-armbased-32bit-mcus-with-subghz-radio-solution-stmicroelectronics.pdf)
* [stm32wlex reference manual](https://www.st.com/resource/en/reference_manual/rm0461-stm32wlex-advanced-armbased-32bit-mcus-with-subghz-radio-solution-stmicroelectronics.pdf)
* [stm32wl55cc datasheet](https://www.st.com/resource/en/datasheet/stm32wl55cc.pdf)
* [stm32wle5c8 datasheet](https://www.st.com/resource/en/datasheet/stm32wle5c8.pdf)
* [stm32wl55xx stm32wl54xx erratum](https://www.st.com/resource/en/errata_sheet/es0500-stm32wl55xx-stm32wl54xx-device-errata-stmicroelectronics.pdf)
* [stm32wle5xx stm32wle4xx erratum](https://www.st.com/resource/en/errata_sheet/es0506-stm32wle5xx-stm32wle4xx-device-errata-stmicroelectronics.pdf)
