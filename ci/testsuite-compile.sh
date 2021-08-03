#!/usr/bin/env bash
set -euo pipefail

echo "Running testsuite compile check"

# https://stackoverflow.com/a/246128
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "$SCRIPT_DIR/.."

for testsuite in *-testsuite
do
    if [[ -v 1 ]]; then
        cargo test \
            -p "$testsuite" \
            --target thumbv7em-none-eabi \
            --features "$1" \
            --no-run
    else
        cargo test \
            -p "$testsuite" \
            --target thumbv7em-none-eabi \
            --no-run
    fi
done
