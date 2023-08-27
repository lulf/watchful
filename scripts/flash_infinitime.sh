probe-rs erase --chip nRF52832_xxAA
#probe-rs download pinetime-mcuboot-app-image-1.13.0.bin --format Binary --base-address 0x8000 --chip nRF52832_xxAA
probe-rs download pinetime-mcuboot-app-image-1.11.0.bin --format Binary --base-address 0x8000 --chip nRF52832_xxAA
probe-rs download mcuboot.bin --format Binary --chip nRF52832_xxAA
