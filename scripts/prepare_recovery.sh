#!/usr/bin/env sh
set -e

pushd infinitime-recovery/boot && cargo clean && cargo objcopy --release && cp target/thumbv7em-none-eabi/release/recovery-boot ../../boot-recovery.elf && popd
pushd infinitime-recovery/app && cargo clean && cargo objcopy --release && cp target/thumbv7em-none-eabi/release/recovery-app ../../app-recovery.elf && popd

#mergehex -m boot-recovery.elf app-recovery.elf -o infinitime-recovery.elf
#mergehex -m boot-recovery.hex app-recovery.hex -o infinitime-recovery.bin
mergehex -m boot-recovery.elf app-recovery.elf -o infinitime-recovery.elf
arm-none-eabi-objcopy -I elf32-littlearm infinitime-recovery.elf -O binary infinitime-recovery.bin

tools/mcuboot/imgtool.py create --align 4 --version 1.0.0 --header-size 32 --slot-size 475136 --pad-header infinitime-recovery.bin infinitime-recovery-image.bin

adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application infinitime-recovery-image.bin infinitime-recovery-dfu.zip

#probe-rs erase --chip nRF52832_xxAA
#probe-rs download blinky-image.bin --base-address 0x8000 --binary-format Binary --chip nRF52832_xxAA
#probe-rs download bootloader-1.0.1.bin --base-address 0 --binary-format Binary --chip nRF52832_xxAA
