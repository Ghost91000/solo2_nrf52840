#!/usr/bin/env bash
# Build the FIDO2 firmware for the SuperMini (nice!nano-compatible clone) and
# package it as a UF2 for the stock nice!nano / Adafruit bootloader.
#
#   ./build-uf2.sh              # release build, board-supermini
#   FEATURES=board-supermini,ccid ./build-uf2.sh   # + contact CCID (hurts NFC)
#
# Requires: rustup toolchain 1.94.1 with the thumbv7em-none-eabihf target,
# flip-link (cargo install flip-link), arm-none-eabi-gcc (littlefs2-sys builds
# C code) and uf2conv.py (fetched automatically into .uf2-tools/ if missing).
#
# Output: ubikey.uf2 in this directory. Flash it by double-tapping RESET on the
# board and dragging the file onto the "NICENANO" USB drive.
set -euo pipefail
cd "$(dirname "$0")"

# Machine-specific toolchain paths (rustup GNU host, MinGW, xpack ARM GCC,
# libclang for bindgen). Present on this checkout; harmless if missing when the
# environment is already set up.
if [ -f toolchain-env.sh ]; then
    # shellcheck disable=SC1091
    . ./toolchain-env.sh
fi

FEATURES=${FEATURES:-board-supermini}
ELF=../../target/thumbv7em-none-eabihf/release/runner-nrf52840dk
UF2CONV=${UF2CONV:-.uf2-tools/uf2conv.py}
UF2_FAMILY=0xADA52840          # generic nRF52840 family id (nice!nano UF2)
APP_ORIGIN=0x26000             # must match memory.x.supermini

if [ ! -f "$UF2CONV" ]; then
    mkdir -p "$(dirname "$UF2CONV")"
    echo "—— fetching uf2conv.py + uf2families.json"
    curl -sSL -o "$UF2CONV" \
        https://raw.githubusercontent.com/microsoft/uf2/master/utils/uf2conv.py
    # uf2conv.py loads the family table next to itself (for -f <family id>).
    curl -sSL -o "$(dirname "$UF2CONV")/uf2families.json" \
        https://raw.githubusercontent.com/microsoft/uf2/master/utils/uf2families.json
fi

echo "—— cargo build ($FEATURES)"
cargo build --release --no-default-features --features "$FEATURES"

OBJCOPY=$(command -v llvm-objcopy || command -v arm-none-eabi-objcopy)
SIZE=$(command -v llvm-size || command -v arm-none-eabi-size)
echo "—— objcopy ($OBJCOPY)"
"$OBJCOPY" -O binary "$ELF" ubikey.bin
"$SIZE" "$ELF" | tail -1

echo "—— uf2conv"
python "$UF2CONV" ubikey.bin -c -f "$UF2_FAMILY" -b "$APP_ORIGIN" -o ubikey.uf2
ls -l ubikey.uf2 ubikey.bin
echo "—— done: $(pwd)/ubikey.uf2 (double-tap RESET → drag to NICENANO)"
