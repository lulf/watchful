#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::info;
use display_interface_spi::SPIInterfaceNoCS;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::peripherals::{P0_18, P0_26, TWISPI0};
use embassy_nrf::spim::Spim;
use embassy_nrf::spis::MODE_3;
use embassy_nrf::{bind_interrupts, peripherals, spim};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::image::{Image, ImageRawLE};
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10};
use embedded_graphics::mono_font::iso_8859_15::FONT_9X18_BOLD;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::{BinaryColor, Rgb565 as Rgb};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
//use embedded_text::alignment::HorizontalAlignment;
use embedded_text::style::{HeightMode, TextBoxStyleBuilder};
use embedded_text::TextBox;
use mipidsi::models::ST7789;
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};
use u8g2_fonts::{fonts, FontRenderer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => spim::InterruptHandler<peripherals::TWISPI0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    info!("Hello world");
    // Button enable
    let _btn_enable = Output::new(p.P0_15, Level::High, OutputDrive::Standard);

    let mut btn = Input::new(p.P0_13, Pull::Down);

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
    let mut display = mipidsi::Builder::st7789(di)
        .with_display_size(240, 240)
        .init(&mut Delay, Some(rst))
        .unwrap();

    display.set_orientation(mipidsi::Orientation::Portrait(false)).unwrap();

    /*
    let raw_image_data = ImageRawLE::new(include_bytes!("../assets/ferris.raw"), 86);
    let ferris = Image::new(&raw_image_data, Point::new(34, 8));

    // draw image on black background
    display.clear(Rgb::BLACK).unwrap();
    ferris.draw(&mut display).unwrap();
    loop {}*/

    /*
    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb::YELLOW);
    let textbox_style = TextBoxStyleBuilder::new()
        .height_mode(HeightMode::Exact(embedded_text::style::VerticalOverdraw::Hidden))
        .alignment(HorizontalAlignment::Center)
        .vertical_alignment(embedded_text::alignment::VerticalAlignment::Middle)
        .paragraph_spacing(6)
        .build();

    let bounds = Rectangle::new(Point::zero(), Size::new(240, 240));

    let text_box = TextBox::with_textbox_style(text, bounds, character_style, textbox_style);
    text_box.draw(&mut display).unwrap();
    */
    //let font = FontRenderer::new::<fonts::u8g2_font_haxrcorp4089_t_cyrillic>();

    let mut state = WatchState::Idle;
    loop {
        match state {
            WatchState::Idle => {
                // TODO: Power save
                display.clear(Rgb::WHITE).unwrap();
                btn.wait_for_any_edge().await;
                if btn.is_high() {
                    info!("Button pressed");
                    state = WatchState::ViewTime;
                } else {
                    info!("Button not pressed");
                }
                // Idle task wait for reactions
                // select(wait_for_button, wait_for_touch, timeout)
            }
            WatchState::ViewTime => {
                display.clear(Rgb::BLACK).unwrap();
                display_time(&mut display).await;
                Timer::after(Duration::from_secs(5)).await;
                state = WatchState::Idle;
            } /*  WatchState::ViewMenu => {
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
              }*/
        }
        // Main is the 'idle task'
    }
}

pub enum WatchState {
    Idle,
    ViewTime,
    //  ViewMenu,
    //  FindPhone,
    //  Workout,
}

type Display =
    mipidsi::Display<SPIInterfaceNoCS<Spim<'static, TWISPI0>, Output<'static, P0_18>>, ST7789, Output<'static, P0_26>>;

async fn display_time(display: &mut Display) {
    //mipidsi::Display<DI, MODEL, RST>) {
    let text = "10:42";
    let font = FontRenderer::new::<fonts::u8g2_font_spleen32x64_mu>();

    font.render_aligned(
        text,
        display.bounding_box().center() + Point::new(0, 0),
        VerticalPosition::Baseline,
        HorizontalAlignment::Center,
        FontColor::Transparent(Rgb::YELLOW),
        display,
    )
    .unwrap();
}

/*
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
