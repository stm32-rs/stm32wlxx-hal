#!/bin/bash
set -euo pipefail

for testsuite in *-testsuite; do
  if [[ "$testsuite" != "subghz-testsuite" ]]; then
    echo "Running $testsuite"
    cargo test -p "$testsuite" --target thumbv7em-none-eabi -- --probe 0034001A5553500B20393256
    echo "Done $testsuite"
  fi
done 

echo "Running subghz-testsuite"
cargo test -p subghz-testsuite --target thumbv7em-none-eabi -- --probe 002900205553500A20393256 &
cargo test -p subghz-testsuite --target thumbv7em-none-eabi -- --probe 0034001A5553500B20393256 &

wait
echo "Done subghz-testsuite"
