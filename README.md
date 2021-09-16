# stm32wl-hal

[![CI](https://github.com/newAM/stm32wl-hal/workflows/CI/badge.svg)](https://github.com/newAM/stm32wl-hal/actions?query=branch%3Amain)
[![docs](https://img.shields.io/badge/docs-gh--pages-blue)](https://newam.github.io/stm32wl-hal/stm32wl_hal/index.html)

Embedded rust HAL (hardware abstraction layer) for the STM32WL.

⚠️ This is a **work in progress**, it is unstable (lots of API changes) and
incomplete.

The code that exists today covers basic usage of:

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

See [hal/README.md](hal/README.md).

## Examples

All examples run on the NUCLEO-WL55JC2.
Examples are located in the `examples` crate.
The arguments got long for this, so a `run-ex` cargo alias is provided.

```bash
cargo run-ex gpio-blink
```

The testsuites are also excellent reference material, they are automatically
tested for every commit and are guaranteed to work.

The testsuites and examples are a good starting point, but they demonstrate
features independent of each-other.
A system-level example using multiple features simultaneously is provided in a
separate repo:
[stm32wl-lightswitch-demo](https://github.com/newAM/stm32wl-lightswitch-demo)

## Unit Tests

Off-target unit tests use the built-in cargo framework.
The only abnormal part of this is that you must specify the target device as a
feature.

```bash
cargo test --features stm32wl5x_cm4
```

## On-Target Tests

Workspaces with `testsuite` in the name denote an on-target test.

On-target tests use the NUCLEO-WL55JC2, you can find a place purchase this
[here](https://www.st.com/en/evaluation-tools/nucleo-wl55jc.html#sample-buy).

### On-Target Quickstart

The on-target tests use [defmt-test].

* `cargo install --git https://github.com/newAM/probe-run.git`
  ([probe-run], [newAM/probe-run])
    * **Note:** My fork contains unreleased fixes for the stm32wl,
      see [#74] for details.
* `rustup target add --toolchain stable thumbv7m-none-eabi` ([rustup])
* (Linux users only) udev rules are available at [newAM/nucleo-wl55jc2-rs]
* Connect the nucleo board to your PC via USB.
* `cargo test -p pka-testsuite --target thumbv7em-none-eabi`

Sample output:
```console
$ cargo test -p pka-testsuite --target thumbv7em-none-eabi
(HOST) INFO  flashing program (43.04 KiB)
(HOST) INFO  success!
────────────────────────────────────────────────────────────────────────────────
 INFO  (1/2) running `ecdsa_sign`...
└─ test::tests::__defmt_test_entry @ tests/test.rs:49
 INFO  (2/2) running `ecdsa_verify`...
└─ test::tests::__defmt_test_entry @ tests/test.rs:49
 INFO  all tests passed!
└─ test::tests::__defmt_test_entry @ tests/test.rs:49
────────────────────────────────────────────────────────────────────────────────
(HOST) INFO  device halted without error
```

#### SubGhz Tests

The subghz on-target tests require two nucleo boards, one for transmitting,
and one for receiving.  Run the `subghz-testsuite` twice on two different boards.

Assuming both boards are connected to the same system you will have to pass a
specific probe to each.

```console
$ probe-run --list-probes
The following devices were found:
[0]: STLink V3 (VID: 0483, PID: 374e, Serial: 001D00145553500A20393256, STLink)
[1]: STLink V3 (VID: 0483, PID: 374e, Serial: 001600345553500A20393256, STLink)
$ cargo test -p subghz-testsuite --target thumbv7em-none-eabi -- --probe 001D00145553500A20393256
$ cargo test -p subghz-testsuite --target thumbv7em-none-eabi -- --probe 001600345553500A20393256
```

## Reference Documentation

* [stm32wl5x reference manual](https://www.st.com/resource/en/reference_manual/rm0453-stm32wl5x-advanced-armbased-32bit-mcus-with-subghz-radio-solution-stmicroelectronics.pdf)
* [stm32wlex reference manual](https://www.st.com/resource/en/reference_manual/rm0461-stm32wlex-advanced-armbased-32bit-mcus-with-subghz-radio-solution-stmicroelectronics.pdf)
* [stm32wle5c8 datasheet](https://www.st.com/resource/en/datasheet/stm32wle5c8.pdf)
* [stm32wl55cc datasheet](https://www.st.com/resource/en/datasheet/stm32wl55cc.pdf)
* [stm32wl55xx stm32wl54xx erratum](https://www.st.com/resource/en/errata_sheet/es0500-stm32wl55xx-stm32wl54xx-device-errata-stmicroelectronics.pdf)
* [stm32wle5xx stm32wle4xx erratum](https://www.st.com/resource/en/errata_sheet/es0506-stm32wle5xx-stm32wle4xx-device-errata-stmicroelectronics.pdf)

[defmt-test]: https://crates.io/crates/defmt-test
[flip-link]: https://github.com/knurling-rs/flip-link
[newAM/nucleo-wl55jc2-rs]: https://github.com/newAM/nucleo-wl55jc2-rs
[newAM/probe-run]: https://github.com/newAM/probe-run
[probe-run]: https://github.com/knurling-rs/probe-run
[rustup]: https://rustup.rs/
[#74]: https://github.com/newAM/stm32wl-hal/issues/74
