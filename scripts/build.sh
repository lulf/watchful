#!/usr/bin/env bash
set -e
pushd firmware
cargo clean
cargo build --release
cargo objcopy --release -- -O binary watchful.bin
popd
./tools/mcuboot/imgtool.py create --align 4 --version 1.0.0 --header-size 32 --slot-size 475136 --pad-header ./firmware/watchful.bin watchful-image.bin
adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application watchful-image.bin watchful-dfu.zip
