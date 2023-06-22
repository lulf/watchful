#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::spis::MODE_3;
use embassy_nrf::{bind_interrupts, peripherals, spim};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565 as Rgb;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_text::alignment::HorizontalAlignment;
use embedded_text::style::{HeightMode, TextBoxStyleBuilder};
use embedded_text::TextBox;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => spim::InterruptHandler<peripherals::TWISPI0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    info!("Hello world");
    // Medium backlight
    let _backlight = Output::new(p.P0_22, Level::Low, OutputDrive::Standard);

    // Reset pin
    let rst = Output::new(p.P0_26, Level::Low, OutputDrive::Standard);

    // Keep low while driving display
    let _cs = Output::new(p.P0_25, Level::Low, OutputDrive::Standard);

    // Data/clock
    let dc = Output::new(p.P0_18, Level::Low, OutputDrive::Standard);

    // create a DisplayInterface from SPI and DC pin, with no manual CS control
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M8;
    config.mode = MODE_3;

    let spim = spim::Spim::new_txonly(p.TWISPI0, Irqs, p.P0_02, p.P0_03, config);

    let di = display_interface_spi::SPIInterfaceNoCS::new(spim, dc);
    // create the ILI9486 display driver from the display interface and optional RST pin
    let mut display = mipidsi::Builder::st7789(di).init(&mut Delay, Some(rst)).unwrap();

    let text = "10:42";

    let character_style = MonoTextStyle::new(&FONT_6X10, Rgb::YELLOW);
    let textbox_style = TextBoxStyleBuilder::new()
        .height_mode(HeightMode::FitToText)
        .alignment(HorizontalAlignment::Justified)
        .paragraph_spacing(6)
        .build();

    let bounds = Rectangle::new(Point::zero(), Size::new(128, 0));

    let text_box = TextBox::with_textbox_style(text, bounds, character_style, textbox_style);

    info!("Rendering started");
    loop {
        text_box.draw(&mut display).unwrap();
        Timer::after(Duration::from_micros(5)).await;
    }
}
/*
    let state = WatchState::Idle;
    loop {
        match state {
            WatchState::Idle => {
                info!("Idle state");
                // Idle task wait for reactions
                // select(wait_for_button, wait_for_touch, timeout)
                state = WatchState::ViewTime
            }
            WatchState::ViewTime => {
                // select(wait_for_button, wait_for_touch, timeout)
                state = WatchState::ViewMenu;
            }
            WatchState::ViewMenu => {
                // select(wait_for_button, wait_for_touch, timeout)
                state = WatchState::Workout;
            }
            WatchState::Workout => {
                // start pulse reading
                // start accelerometer reading
                // display exercise view
                // refresh display until timout (go black)
            }
            WatchState::FindPhone => {
                // try connecting to phone over BLE
                // tell phone to make some noise
            }
        }
        Timer::after(Duration::from_secs(5)).await;
        // Main is the 'idle task'
    }

}

pub enum WatchState {
    Idle,
    ViewTime,
    ViewMenu,
    FindPhone,
    Workout,
}

pub enum ViewState {}

pub type Signal = Signal<CriticalSectionRawMutex, Command>;
pub enum Command {
    Stop,
}

struct HealthTracker {}

impl HealthTracker {
    pub async fn run(&mut self) {}
}

#[embassy_executor::task]
async fn exercise(signal: StopSignal, tracker: HealthTracker) {
    loop {
        match select(signal.wait(), tracker.run()).await {
            Either::First(command) => match command {
                Command::Stop => break,
            },
            Either::Second(_) => {}
        }
    }
}
*/
