# Watchful

(Anagram for _ulfwatch_)

Real Time Async Enterprise Watch OS written in Rust!

Firmware for Pinetime based on [Embassy](https://embassy.dev). The goal is to provide a firmware for the PineTime that is written in Rust and can function as watch OS.

![PineTime on my arm running Watchful](image.png)

## Getting started

The recommended way to run Watchful is to get a [PineTime Development Kit](https://pine64.com/product/pinetime-dev-kit/), to which you can connect a debug probe. To run Watchful, you need to download the latest [S132 SoftDevice](https://www.nordicsemi.com/Products/Development-software/s132/download). For flashing and running with the debug probe, `probe-rs` is recommended.

To run Watchful:

``` 4d
# Installing the softdevice
probe-rs download path-to-softdevice.hex --format Hex --chip nRF52832_xxAA

# Flashing the bootloader
cargo flash --manifest-path firmware/boot/Cargo.toml --release

# Flashing the OS
cargo flash --manifest-path firmware/app/Cargo.toml --release
```

## Updating firmware

Once you have Watchful running, you can use an app such as nRF Connect on Android or iOS using the DFU functionality with the [latest release](https://github.com/lulf/watchful/releases).

## *DANGER* Reflashing your sealed PineTime from InfiniTime to Watchful

If you want to reflash your sealed PineTime to Watchful, it is possible. But there is a chance to brick your PineTime, so don't do this unless you've tried it a few times on a devkit and feel confident. Also consider the fact that once you go to Watchful, there is no way to go back at the moment.

*I take no responsibilty of broken PineTimes, you are hereby warned :)*

With that out of the way, 

* Update your PineTime to Infinity 1.13.0
* Download the `watchful-reloader` from the [latest release](https://github.com/lulf/watchful/releases)
* Use whatever [update mechanism that works with InfiniTime](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/doc/gettingStarted/updating-software.md).

TODO:

* [x] Implement Nordic DFU protocol (version supported by Gadgetbridge and SDK 15.x) (moved to [nrf-dfu-target](https://crates.io/crates/nrf-dfu-target) crate)
* [x] Synchronize watch with phone
* [x] Use external flash for firmware and persistence
* [x] Update firmware over DFU and validate in UI
* [x] Support installing from Infinitime 'factory firmware'
* [ ] Support swapping back to Inifinitime + MCUBoot
* [x] Show clock in view mode
* [x] View for viewing and validating firmware
* [ ] Low power in idle mode
* [ ] Menu for starting workout or finding phone
* [ ] Workout tracking mode
* [ ] Record and export of workout data
* [ ] Finding phone mode 
