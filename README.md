# Watchful

Real Time Async Enterprise Watch OS written in Rust!

Firmware for Pinetime based on [Embassy](https://embassy.dev).

TODO:

* [x] Implement Nordic DFU protocol (version supported by Gadgetbridge and SDK 15.x) (moved to [nrf-dfu-target](https://crates.io/crates/nrf-dfu-target) crate)
* [x] Synchronize watch with phone
* [x] Use external flash for firmware and persistence
* [x] Update firmware over DFU and validate in UI
* [ ]
* [ ] Support swapping to/from Inifinitime + MCUBoot
* [ ] Support installing from Infinitime 'factory firmware'
* [ ] Show clock in view mode
* [ ] Low power in idle mode
* [ ] Menu for starting workout or finding phone
* [ ] Workout tracking mode
* [ ] Record and export of workout data
* [ ] Finding phone mode 


Resources:
* Updating to watch-os: https://www.youtube.com/watch?v=lPasAt1LJmo
* Bootloading process: https://github.com/InfiniTimeOrg/pinetime-mcuboot-bootloader
* Memory of factoru: https://wiki.pine64.org/wiki/PineTime_bootloader_improvements
