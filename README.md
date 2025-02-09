[![CI](https://github.com/lulf/watchful/actions/workflows/ci.yaml/badge.svg)](https://github.com/lulf/watchful/actions/workflows/ci.yaml)

# Watchful

(Anagram for _ulfwatch_)

Real Time Async Enterprise Watch OS written in Rust!

Firmware for Pinetime based on [Embassy](https://embassy.dev). The goal is to provide a firmware for the PineTime that is written in Rust and can function as watch OS.

<img src="image.png" alt="PineTime on my arm running Watchful" style="width:200px;"/>

## Features 

* Basic UI with menus using [`embedded-graphics`](https://crates.io/crates/embedded-graphics).
* Automatically synchronizes time with using BLE standard Current Time Service.
* Rollback to previous firmware if reset or crashing before new firmware is validated in watch UI.
* Compatible with existing InfiniTime bootloader.
* Implements Nordic and InfiniTime DFU protocols so you can update from a phone app such as nRF Connect to perform firmware updates.

## Getting started

If you have InfiniTime running already, it's easy to try out Watchful. You can use the same app you use to update InfiniTime to try out Watchful (such as GadgetBridge). 

Pick the `watchful-dfu.zip` from the [latest release](https://github.com/lulf/watchful/releases) and upload it to your watch. To revert back to InfiniTime, head into the menu -> settings -> reset, and mcuboot will do the rest.

To permanently mark Watchful as your OS, go to settings -> firmware and press `validate`. If you want to move back to InfiniTime later, you can use the DFU mechanism with the standard InfiniTime DFU image.

## Developing

The recommended way to develop Watchful is to get a [PineTime Development Kit](https://pine64.com/product/pinetime-dev-kit/), to which you can connect a debug probe. For flashing and running with the debug probe, `probe-rs` is recommended.


### Running without MCUBoot

While developing, running without the bootloader is the quickest way to iterate:

``` 4d
cd firmware
cargo run --release --features panic-probe,baremetal
```

### Building an image

To build an MCUBoot compatible image:

```
cd firmware
cargo build --release
cargo objcopy --release -- -O binary watchful.bin
../tools/mcuboot/imgtool.py create --align 4 --version 1.0.0 --header-size 32 --slot-size 475136 --pad-header watchful.bin watchful-image.bin
```

You can also download the latest non-released image from the CI.

### Flashing MCUBoot

Flashing MCUBoot requires the mcuboot image:

```
probe-rs erase --chip nRF52832_xxAA
probe-rs download tools/mcuboot/mcuboot.bin --binary-format Binary --chip nRF52832_xxAA
```

You can also build the MCUBoot image yourself from the [pinetime-mcuboot-bootloader](https://github.com/InfiniTimeOrg/pinetime-mcuboot-bootloader).

### Flashing Watchful Image

To flash the watchful image, use the artifact from your own build, CI or a release:

```
probe-rs download watchful-image.bin --binary-format Binary --base-address 0x8000 --chip nRF52832_xxAA
```

## Recovering from older versions of Watchful

NOTE: If you've used watchful before 0.2.6, it now has switched from using `embassy-boot` to `mcuboot` as provided by default on InfiniTime. To achieve that, the [nrf-softdevice](https://github.com/embassy-rs/nrf-softdevice/) has been replaced with [trouble](https://github.com/embassy-rs/trouble).

The `infinitime-recovery` app allows you to move from previous versions of Watchful to the new.

## License

Code in `tools/mcuboot` are subject to mcuboot licensing.

Watchful is licensed under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
