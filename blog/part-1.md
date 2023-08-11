# Async Rust on the Pinetime - Part 1

In this blog post series, we'll go through the development of a complex Embassy application for a real world use case (Well, almost real world!). One of the most common questions I've seen asked in the Embassy chat is "How do I get started with an Embassy project?". The series will cover things such as:

* Developing firmware based on Embassy
* Using the embassy-boot bootloader
* Using Bluetooth Low Energy (BLE) 
* Writing an async driver
* Driving multiple SPI devices on the same bus

The first part will focus on the basics.

## Background

First a bit more context about the project. A while back, I got myself a [PineTime](https://www.pine64.org/pinetime/) smart watch. One of the reasons is that you can also buy a [development kit](https://pine64.com/product/pinetime-dev-kit/) and attach a debug probe. However, programming the watch was something I planned for later. The assembled version comes with [InfiniTime](https://github.com/InfiniTimeOrg/InfiniTime), which is based on [FreeRTOS](https://www.freertos.org/index.html) and [MCUBoot](https://docs.mcuboot.com/). The watch works well enough with InfiniTime, but I managed to come up with a few excuses of writing my own:

* Some features not working the way I want or lacking in InfiniTime
* Because I can!

When I think of it, probably the last reason is the only valid one. 

## The Hardware

The good news is that the PineTime uses the Nordic nRF52832, which is probably one of the best supported MCUs in Embassy. The bad news is that the best BLE support for Rust still relies on the [Softdevice](), which is in maintenance mode. Hopefully, new projects like [Bleps]() will gain traction.

As for the rest of the peripherals, the [Display]() is already supported with `embedded-graphics`. The external flash required writing an async driver (which I'll cover in the next blog post).

## The Goal

There are other features I want:

* Showing the time (duh!)
* A way to track pulse during workouts
* A way to ping my phone when I've lost it

There are also a few other requirements:

* Firmware updates: ideally using the same phone apps as the InfiniTime, which uses the nRF DFU protocol.
* Using Rust and Embassy, because I'm convinced it is superior to anything else.
* Can be installed from InfiniTime (without bricking the device!).
* (Optional) Can revert back to the InfiniTime firmware.

## Starting the project

When starting out a new Embassy project, I often base the scaffolding on a previous project. However, I've found that the Embassy examples are good to look at for dependencies and features required in Cargo.toml, as well as the main application.

There are a growing list of example projects you can look at:

* [embassy-start](https://github.com/titanclass/embassy-start)
* [official examples](https://github.com/embassy-rs/embassy/tree/main/examples)
* [drogue-iot](https://github.com/drogue-iot/drogue-device/tree/main/examples)
* [this project](https://github.com/lulf/pinetime-embassy)

The examples depend directly on the in-tree embassy crates, so it might be slightly confusing as a starting point. You need to make sure the following is in place:

* Modify Cargo.toml with `[patch.crates]` section for all embassy dependencies pointing to the revision of Embassy you want to use.
* A `.cargo/config.toml` file which defines the default build target and commands for running firmware using a debug probe.
* A `rust-toolchain.toml` file which pins the toolchain version to the same as Embassy.

It can be hard to select which Embassy revision to use, but generally starting from the latest revision on the main branch works, due to automated testing of every PR and push to the Embassy main branch. When updating, search and replace of the revision in the Cargo.toml to another revision.

### Project layout

Since this particular project also requires a bootloader (embassy-boot, for DFU), and a reflasher tool (I'll come back to what that is for), I'm keeping the .cargo in the workspace folder, and the `[patch.crates-io]` section in the workspace `Cargo.toml`. This just makes it quicker to update, but there is a drawback: cargo features could potentially conflict, in which case we might change to not using workspaces at a later point.

Since I know I'm going to need DFU, I'll lay everything out with that in mind from the beginning. This is a personal taste, but I find it easier to debug bootloading and firmware updates while the application is somewhat manageable.

All in all, the project has the following structure at the time of writing:

```
firmware/.cargo/config.toml
firmware/Cargo.toml

firmware/app/Cargo.toml
firmware/app/memory.x
firmware/app/build.rs
firmware/app/src/main.rs

firmware/bootloader/Cargo.toml
firmware/bootloader/memory.x
firmware/bootloader/build.rs
firmware/bootloader/src/main.rs

pinetime-flash/Cargo.toml
pinetime-flash/src/lib.rs
```

Note the `memory.x`linker scripts. It is important that regions in the app and bootloader linker scripts match! The `build.rs` files are necessary to make the linker pick up additional arguments (such as the linker script).

I also ended up with one additional crate. Keeping utilities and additional crates in the same repo is useful during development and also allows for creating unit tests that doesn't require running on the device. Once the crate is generally useful, move it out and let it live its own life in the true spirit of open source!

## Development tools

* `cargo-flash`

Generally, I start with flashing the bootloader first:

* cargo flash --manifest-path firmware/bootloader/Cargo.toml --chip nRF52832_xxAA

## Summary


## BLE

The excellent [nrf-softdevice]() crate is well supported to use with Embassy, even if the blob is no longer developed. Since I've made several BLE applications with Embassy for the microbit, this part was the easiest. However, I 
