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

NOTE: Some features that has been removed after recent switch to mcuboot, but will be added again soon:

* DFU: Implements Nordic DFU protocol so you can update from a phone app such as nRF Connect to perform firmware updates.


## Getting started

If you have InfiniTime running already, it's easy to try out Watchful. You can use the same app you use to update InfiniTime to try out Watchful (such as GadgetBridge). 

Pick the `watchful-dfu.zip` from the [latest release](https://github.com/lulf/watchful/releases) and upload it to your watch. To revert back to InfiniTime, head into the menu -> settings -> reset, and mcuboot will do the rest.

## Developing

The recommended way to develop Watchful is to get a [PineTime Development Kit](https://pine64.com/product/pinetime-dev-kit/), to which you can connect a debug probe. For flashing and running with the debug probe, `probe-rs` is recommended.

To run Watchful:

``` 4d
cd firmware
cargo run --release --features panic-probe,baremetal
```

## Recovering from older versions of Watchful

NOTE: If you've used watchful before, it now has switched from using `embassy-boot` to `mcuboot` as provided by default on InfiniTime. To achieve that, the [nrf-softdevice](https://github.com/embassy-rs/nrf-softdevice/) has been replaced with [trouble](https://github.com/embassy-rs/trouble). 

The `infinitime-recovery` app allows you to move from previous versions of Watchful to the new.

## License

Code in `tools/mcuboot` are subject to mcuboot licensing.

Watchful is licensed under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
