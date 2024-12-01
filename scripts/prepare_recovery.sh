#!/usr/bin/env sh

pushd infinitime-recovery/boot && cargo objcopy --release -- -O ihex ../../boot-recovery.hex && popd
pushd infinitime-recovery/app && cargo objcopy --release -- -O ihex ../../app-recovery.hex && popd

mergehex -m boot-recovery.hex app-recovery.hex -o infinitime-recovery.bin

tools/mcuboot/imgtool.py create --align 4 --version 1.0.0 --header-size 32 --slot-size 475136 --pad-header infinitime-recovery.bin infinitime-recovery-image.bin

#probe-rs erase --chip nRF52832_xxAA
#probe-rs download blinky-image.bin --base-address 0x8000 --binary-format Binary --chip nRF52832_xxAA
#probe-rs download bootloader-1.0.1.bin --base-address 0 --binary-format Binary --chip nRF52832_xxAA
