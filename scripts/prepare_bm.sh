set -e
pushd reloader/boot && cargo objcopy --release -- -O ihex ../../boot.hex && popd
pushd reloader/app && cargo objcopy --release -- -O ihex ../../app.hex && popd

mergehex -m boot.hex app.hex -o reloader.bin

probe-rs erase --chip nRF52832_xxAA
probe-rs download reloader.bin --format Binary --chip nRF52832_xxAA
