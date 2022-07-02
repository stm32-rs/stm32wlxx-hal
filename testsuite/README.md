# Testsuite

This workspace contains on-target tests using [defmt-test].

The tests run on the NUCLEO-WL55JC2, you can find a place purchase this
[here](https://www.st.com/en/evaluation-tools/nucleo-wl55jc.html#sample-buy).

These tests will run automatically as part of CI for every pull-request.

## Quickstart

* `rustup target add --toolchain stable thumbv7em-none-eabi` ([rustup])
* `cargo install probe-run` ([probe-run])
  * ⚠️ You must use version >=0.3.1 to avoid bugs with the STM32WL ⚠️
* Linux users: Add udev rules

```text
# /etc/udev/rules.d/99-stm.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666"
```

* Linux users: run `sudo udevadm control --reload-rules && sudo udevadm trigger`
* Connect the nucleo board to your PC via USB
* `cargo test -p testsuite --target thumbv7em-none-eabi --bin pka`

## Sample output

```console
$ DEFMT_LOG=info cargo test -p testsuite --target thumbv7em-none-eabi --bin pka
    Finished dev [optimized + debuginfo] target(s) in 0.01s
     Running `probe-run --chip STM32WLE5JCIx --connect-under-reset target/thumbv7em-none-eabi/debug/pka`
(HOST) INFO  flashing program (17.31 KiB)
(HOST) INFO  success!
────────────────────────────────────────────────────────────────────────────────
0.000008 INFO  (1/10) running `ecdsa_sign`...
└─ pka::tests::__defmt_test_entry @ testsuite/src/pka.rs:60
0.108633 INFO  (2/10) running `ecdsa_sign_nb`...
└─ pka::tests::__defmt_test_entry @ testsuite/src/pka.rs:60
0.217258 INFO  (3/10) running `ecdsa_sign_ram_error`...
└─ pka::tests::__defmt_test_entry @ testsuite/src/pka.rs:60
0.436387 INFO  (4/10) running `ecdsa_sign_mode_error`...
└─ pka::tests::__defmt_test_entry @ testsuite/src/pka.rs:60
0.655516 INFO  (5/10) running `ecdsa_verify`...
└─ pka::tests::__defmt_test_entry @ testsuite/src/pka.rs:60
0.874646 INFO  (6/10) running `ecdsa_verify_nb`...
└─ pka::tests::__defmt_test_entry @ testsuite/src/pka.rs:60
1.093774 INFO  (7/10) running `ecdsa_verify_ram_err`...
└─ pka::tests::__defmt_test_entry @ testsuite/src/pka.rs:60
1.202398 INFO  (8/10) running `ecdsa_verify_mode_err`...
└─ pka::tests::__defmt_test_entry @ testsuite/src/pka.rs:60
1.311021 INFO  (9/10) running `ecdsa_verify_invalid_err`...
└─ pka::tests::__defmt_test_entry @ testsuite/src/pka.rs:60
1.527159 INFO  (10/10) running `ecdsa_doc_keypair`...
└─ pka::tests::__defmt_test_entry @ testsuite/src/pka.rs:60
1.845845 INFO  all tests passed!
└─ pka::tests::__defmt_test_entry @ testsuite/src/pka.rs:60
────────────────────────────────────────────────────────────────────────────────
  (HOST) INFO  device halted without error
```

## SubGhz Tests

The subghz on-target tests require two nucleo boards, one for transmitting,
and one for receiving.
Run the `subghz` binary twice on two different boards.

Assuming both boards are connected to the same system you will have to pass a
specific probe to each.

```console
$ probe-run --list-probes
The following devices were found:
[0]: STLink V3 (VID: 0483, PID: 374e, Serial: 001D00145553500A20393256, STLink)
[1]: STLink V3 (VID: 0483, PID: 374e, Serial: 001600345553500A20393256, STLink)
$ DEFMT_LOG=info cargo test -p testsuite --target thumbv7em-none-eabi --bin subghz -- --probe 001D00145553500A20393256
$ DEFMT_LOG=info cargo test -p testsuite --target thumbv7em-none-eabi --bin subghz -- --probe 001600345553500A20393256
```

[defmt-test]: https://crates.io/crates/defmt-test
[probe-run]: https://github.com/knurling-rs/probe-run
[rustup]: https://rustup.rs/
