[![CI](https://github.com/lulf/watchful/actions/workflows/ci.yaml/badge.svg)](https://github.com/lulf/watchful/actions/workflows/ci.yaml)

# Watchful

(Anagram for _ulfwatch_)

Real Time Async Enterprise Watch OS written in Rust!

Firmware for Pinetime based on [Embassy](https://embassy.dev). The goal is to provide a firmware for the PineTime that is written in Rust and can function as watch OS.

<img src="image.png" alt="PineTime on my arm running Watchful" style="width:200px;"/>

## Features 

* Basic UI with menus using [`embedded-graphics`](https://crates.io/crates/embedded-graphics).
* Implements Nordic DFU protocol so you can update from a phone app such as nRF Connect to perform firmware updates.
* Automatically synchronizes time with using BLE standard Current Time Service.
* Rollback to previous firmware if reset or crashing before new firmware is validated in watch UI.
* Compatible with existing InfiniTime bootloader.

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

## Updating firmware

Once you have Watchful running, you can use an app such as GadgetBridge nRF Connect on Android or iOS using the DFU functionality with the [latest release](https://github.com/lulf/watchful/releases).

## *DANGER* Reflashing your sealed PineTime from InfiniTime to Watchful

If you want to reflash your sealed PineTime to Watchful, it is possible. But there is a chance to brick your PineTime, so don't do this unless you've tried it a few times on a devkit and feel confident. Also consider the fact that once you go to Watchful, there is no way to go back at the moment.

*I take no responsibilty of broken PineTimes, you are hereby warned :)*

With that out of the way, 

* Update your PineTime to Infinity 1.13.0
* Download the `watchful-reloader` from the [latest release](https://github.com/lulf/watchful/releases)
* Use whatever [update mechanism that works with InfiniTime](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/doc/gettingStarted/updating-software.md).
