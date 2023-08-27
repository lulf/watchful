set -e
pushd firmware/boot && cargo objcopy --release -- -O binary ../../reloader/app/bootloader.bin && popd
pushd firmware/app && cargo objcopy --release -- -O binary ../../reloader/app/application.bin && popd
pushd reloader/boot && cargo objcopy --release -- -O ihex ../../boot.hex && popd
pushd reloader/app && cargo objcopy --release -- -O ihex ../../app.hex && popd

mergehex -m boot.hex app.hex -o reloader.bin

imgtool create --align 4 --version 1.0.0 --header-size 32 --slot-size 475136 --pad-header reloader.bin image.bin

probe-rs erase --chip nRF52832_xxAA
probe-rs download image.bin --format Binary --base-address 0x8000 --chip nRF52832_xxAA
probe-rs download mcuboot.bin --format Binary --chip nRF52832_xxAA
