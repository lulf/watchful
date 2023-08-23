# Rust on my Pinetime 

In this blog post series, we'll go through the development of a complex Embassy application for a real world use case (Well, almost real world!). One of the most common questions I've seen asked in the Embassy chat is "How do I get started with an Embassy project?". The series will cover things such as:

* Developing firmware based on Embassy
* Using the embassy-boot bootloader
* Using Bluetooth Low Energy (BLE) 
* Writing an async driver
* Driving multiple SPI devices on the same bus

The first part will focus on the basics of creating a standalone Embassy project.

## Background

First a bit more context about the project. A while back, I got myself a [PineTime](https://www.pine64.org/pinetime/) smart watch. One of the reasons is that you can also buy a [development kit](https://pine64.com/product/pinetime-dev-kit/) and attach a debug probe. However, programming the watch was something I planned for later. The assembled version comes with [InfiniTime](https://github.com/InfiniTimeOrg/InfiniTime), which is based on [FreeRTOS](https://www.freertos.org/index.html) and [MCUBoot](https://docs.mcuboot.com/). The watch works well enough with InfiniTime, but I managed to come up with a few excuses of writing my own:

* Some features not working the way I want or lacking in InfiniTime
* Because I can!

When I think of it, probably the last reason is the only valid one. 

## The Hardware

The good news is that the PineTime uses the Nordic nRF52832, which is probably one of the best supported MCUs in Embassy. The bad news is that the best BLE support for Rust still relies on the [Softdevice](https://infocenter.nordicsemi.com/topic/struct_nrf52/struct/nrf52_softdevices.html), which is in maintenance mode. Hopefully, new projects like [Bleps](https://github.com/bjoernQ/bleps) will gain traction.

As for the rest of the peripherals, the [Display](https://crates.io/crates/st7789) is already supported with `embedded-graphics`. The external flash required writing an async driver (which I'll cover in the next blog post).

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

## Getting started

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

Since I know I'm going to need DFU, I'll lay everything out with that in mind from the beginning. This is a personal taste, but I find it easier to debug bootloading and firmware updates while the application is somewhat small.

And once the bootloading and DFU parts are written, I get to test it every time I update with some new feature while development, hopefully uncovering real bugs.

```
firmware/ # Embassy firmware
firmware/app/  # Application part
firmware/boot/ # Bootloader part

reloader/ # Firmware that swaps InfiniTime with Embassy
reloader/app/
reloader/boot/

pinetime-flash/ # Driver for external Flash
watchful-ui/ # The UI of the watch firmware which can also run on a simulator
```

Originally I ended up with one additional crate, `nrf-dfu`, which was later moved. Keeping utilities and additional crates in the same repo is useful during development and also allows for creating unit tests that doesn't require running on the device. Once the crate is generally useful, move it out and let it live its own life in the true spirit of open source!


## Development tools

In general `probe-rs` is all you need for flashing and debugging. I use:

* `probe-rs run --release --chip nRF52832_xxAA` to run the application with RTT logging.
* `cargo flash --release --chip nRF52832_xxAA` to run the application without attaching RTT.


## Firmware

The firmware is the main part of the application. It implements the main features:

* User interface
* Touch gestures
* BLE connectivity
* DFU
* Time synchronization
* ...

### Flash driver

At the time of development, the flash chip in the PineTime did not have any Rust driver I could use (That I was able to find). Luckily, the PineTime Wiki had a link to [the datasheet](), and that got me going.

If you need to write code that uses flash in embedded Rust, consider relying on the traits from the `embedded-storage` and `embedded-storage-async `crates, which define an API that seems to map well to most flash implementations.

By doing so, your driver will be able to work with Embassy but also any other bare metal Rust implementation.

## The Reloader

Ok, great! I have a firmware that is working, can do DFU over BLE and swap firmwares using embassy-boot. It works great on my devkit. Now, how do I get this on my sealed PineTime with no debug probe attached? That's what the reloader is for.
The reloader is a special firmware that can be loaded with MCUBoot, with the purpose of flashing the embassy bootloader and firmware to replace both MCUBoot and InfiniTime. But it's a one shot chance, if it fails during the process, the device might get bricked. Sounds risky! On positive side, the development kit allowed testing this thoroughly before attempting to reflash my watch.

The reloader includes the main bootloader, application and nRF softdevice and is tasked with flashing each of these to the appropriate location: 

* The softdevice is written at the start of the internal flash
* The bootloader is written to the later part of the internal flash
* The application is written to the DFU partition in the external flash
* Write the address of the bootloader in the UICR flash region
* Mark the bootloader state as requiring a firmware swap

Once the reloader has run, it will reset and the bootloader will copy the application firmware from the external flash into the internal flash in the active partition.

You might wonder why the reloader has a bootloader. If you look at the linker scripts, you can see that the app portion of the reloader is located at a higher flash address. The reason for that is that the reloader will need to overwrite both the MCUBoot and the application sections of the flash, and therefor must be located in a 'safe' area.

To build the reloader, first a hex file of both the boot and app is made:

```
cd app && cargo objcopy --release -- -O ihex app.hex
cd boot && cargo objcopy --release -- -O ihex boot.hex
mergehex -m boot/boot.hex app/app.hex -o combined.bin
```

Then, the `imgtool.py` tool from MCUBoot is used to create an "image" that can be loaded by MCUBoot:

```
imgtool.py create --align 4 --version 1.0.0 --header-size 32 --slot-size 475136 --pad-header combined.bin watchful-reloader-image.bin
```

The final step is to create a nRF DFU package with the image: 

```
adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application watchful-reloader-image.bin watchful-reloader-dfu.zip
```

