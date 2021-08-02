# stm32wl-hal

Embedded rust HAL (hardware abstraction layer) for the STM32WL.

This is a **work in progress**, it is unstable, incomplete, and (mostly) untested.

**Use at your own risk.**

There should be enough code for very basic:

* LoRa TX
* (G)FSK TX
* (G)FSK RX
* SPI
* ECDSA signing (polling + async)
* ECDSA verification (polling + async)
* Cryptographically secure random number generation (polling + async)
* AES encryption (polling + async)
* AES decryption (polling + async)

## Why upload incomplete code?

This is better than nothing for other people developing on these new chips.

I am also hoping to find other people using these chips with rust who want to
collaborate on creating a HAL :)

## On-Target Tests

Workspaces with `testsuite` in the name denote an on-target test.

On-target tests use the NUCLEO-WL55JC2, you can find a place purchase this
[here](https://www.st.com/en/evaluation-tools/nucleo-wl55jc.html#sample-buy).

### On-Target Quickstart

The on-target tests use [defmt-test].

* `cargo install flip-link` ([flip-link])
* `cargo install --git https://github.com/knurling-rs/probe-run.git`
  ([probe-run], [newAM/probe-run])
    * **Note:** My fork contains unreleased fixes for the stm32wl,
      see #74 for details.
* `rustup target add --toolchain stable thumbv7m-none-eabi` ([rustup])
* (Linux users only) udev rules are available at [newAM/nucleo-wl55jc2-rs]
* Connect the nucleo board to your PC via USB.
* `cargo test -p pka-testsuite --target thumbv7em-none-eabi`

Sample output:
```text
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

These tests require two nucleo boards, one for transmitting, and one for
receiving.

Run `subghz-testsuite-rx` before `subghz-testsuite-tx`.

Assuming both are connected to the same system you will have to pass a specific
probe to each.

```text
$ probe-run --list-probes
The following devices were found:
[0]: STLink V3 (VID: 0483, PID: 374e, Serial: 001D00145553500A20393256, STLink)
[1]: STLink V3 (VID: 0483, PID: 374e, Serial: 002900205553500A20393256, STLink)
$ cargo test -p subghz-testsuite-rx --target thumbv7em-none-eabi -- --probe 001D00145553500A20393256
$ cargo test -p subghz-testsuite-tx --target thumbv7em-none-eabi -- --probe 002900205553500A20393256
```

## Unit Tests

Off-target unit tests use the built-in cargo framework.
The only difference is you must specify the MCU as a feature.

```bash
cargo test --features stm32wl5x_cm4
```

## Examples

All examples run on the NUCLEO-WL55JC2.
Examples are located in the `examples` crate.
The arguments got long for this, so a `run-ex` cargo alias is provided.

```bash
cargo run-ex gpio-blink
```

## async

There is opt-in experimental `async` code in the hal.
The `async` code is more experimental and unstable then the rest of this crate
(which is already experimental and unstable).

There is no asynchronous executor provided, you must build your own.

`async` support may be dropped at any time for any reason.

Enabling `async` requires:
1. nightly rust (non-async code compiles on stable)
2. `--features aio`

For example, if you wanted to run the AES testsuite with AIO you would use:
```text
cargo +nightly test -p aes-testsuite --target thumbv7em-none-eabi --features aio
```

[defmt-test]: https://crates.io/crates/defmt-test
[flip-link]: https://github.com/knurling-rs/flip-link
[newAM/nucleo-wl55jc2-rs]: https://github.com/newAM/nucleo-wl55jc2-rs
[newAM/probe-run]: https://github.com/newAM/probe-run
[probe-run]: https://github.com/knurling-rs/probe-run
[rustup]: https://rustup.rs/
