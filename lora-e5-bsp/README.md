# LoRa-E5 Board Support Package

Board support for the seeed LoRa-E5 development kit.

## Usage

This board is a pain to get working for the first time because of the
pre-programmed firmware.

Check these resources to unlock the board for programming:

* [How to program a LoRa-E5](https://forum.seeedstudio.com/t/how-to-program-a-lora-e5/257491)
* [seeed-lora/LoRa-E5-LoRaWAN-End-Node](https://github.com/seeed-lora/LoRa-E5-LoRaWAN-End-Node#getting-started)

### Flashing

To flash this board with various rust utilities
(e.g. probe-run, cargo-embed, cargo-flash)
remove the `--connected-under-reset` flag.
This flag is required for the NUCLEO board, but will cause timeout errors with
the LoRa-E5 development board.
