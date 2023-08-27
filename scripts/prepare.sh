pushd firmware/boot && cargo objcopy --release -- -O binary ../../reloader/app/bootloader.bin && popd
pushd firmware/app && cargo objcopy --release -- -O binary ../../reloader/app/application.bin && popd
pushd reloader/boot && cargo objcopy --release -- -O ihex ../reloader-boot.hex && popd
pushd reloader/app && cargo objcopy --release -- -O ihex ../reloader-app.hex && popd

mergehex -m reloader/reloader-boot.hex reloader/reloader-app.hex -o reloader.bin
imgtool create --align 4 --version 1.0.0 --header-size 32 --slot-size 475136 --pad-header reloader.bin watchful-reloader-image.bin
adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application watchful-reloader-image.bin watchful-reloader-dfu.zip
