#![no_std]

use ::core::fmt::Write as _;
use embedded_graphics::pixelcolor::Rgb565 as Rgb;
use embedded_graphics::prelude::{DrawTarget, *};
use embedded_graphics::primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::{Text, TextStyleBuilder};
use embedded_text::style::TextBoxStyleBuilder;
use embedded_text::TextBox;
use u8g2_fonts::{fonts, U8g2TextStyle};

mod core;

const WIDTH: u32 = 240;
const HEIGHT: u32 = 240;
const GRID_ITEMS: u32 = 3;

#[derive(Clone, Copy)]
pub struct FirmwareDetails {
    name: &'static str,
    version: &'static str,
    commit: &'static str,
    build_timestamp: &'static str,
    validated: bool,
}

fn watch_text_style(color: Rgb) -> U8g2TextStyle<Rgb> {
    //U8g2TextStyle::new(fonts::u8g2_font_unifont_t_symbols, Rgb::YELLOW)
    U8g2TextStyle::new(fonts::u8g2_font_inb57_mn, color) //u8g2_font_logisoso58_tn, color) //u8g2_font_logisoso78_tn, color) //u8g2_font_logisoso92_tn, color) //u8g2_font_fub49_tn, color) //u8g2_font_spleen16x32_mf, color)
}

fn menu_text_style(color: Rgb) -> U8g2TextStyle<Rgb> {
    //U8g2TextStyle::new(fonts::u8g2_font_unifont_t_symbols, Rgb::YELLOW)
    U8g2TextStyle::new(fonts::u8g2_font_spleen16x32_mf, color)
}

fn text_text_style(color: Rgb) -> U8g2TextStyle<Rgb> {
    U8g2TextStyle::new(fonts::u8g2_font_unifont_t_symbols, color)
}

impl FirmwareDetails {
    pub const fn new(
        name: &'static str,
        version: &'static str,
        commit: &'static str,
        build_timestamp: &'static str,
        validated: bool,
    ) -> Self {
        Self {
            name,
            version,
            commit,
            build_timestamp,
            validated,
        }
    }

    pub fn draw<D: DrawTarget<Color = Rgb>>(&self, display: &mut D) -> Result<(), D::Error> {
        let start = Point::new(0, 0);
        let end = Size::new(WIDTH as u32, 2 * (HEIGHT as u32 / GRID_ITEMS as u32) - 20);

        let bounds = Rectangle::new(start, end);

        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(embedded_text::style::HeightMode::FitToText)
            .alignment(embedded_text::alignment::HorizontalAlignment::Justified)
            .paragraph_spacing(6)
            .build();

        let character_style = text_text_style(Rgb::YELLOW);

        let mut info: heapless::String<256> = heapless::String::new();
        write!(
            info,
            "Name: {}\nVersion: {}\nCommit: {}\nBuild: {}",
            self.name, self.version, self.commit, self.build_timestamp
        )
        .unwrap();

        TextBox::with_textbox_style(&info, bounds, character_style, textbox_style).draw(display)?;
        Ok(())
    }
}

pub struct WatchView {
    time: time::PrimitiveDateTime,
}

impl WatchView {
    pub fn new(time: time::PrimitiveDateTime) -> Self {
        Self { time }
    }
    pub fn draw<D: DrawTarget<Color = Rgb>>(&self, display: &mut D) -> Result<(), D::Error> {
        display.clear(Rgb::BLACK)?;
        let character_style = watch_text_style(Rgb::BLUE);
        let text_style = TextStyleBuilder::new()
            .alignment(embedded_graphics::text::Alignment::Center)
            .baseline(embedded_graphics::text::Baseline::Alphabetic)
            .build();

        let mut text: heapless::String<16> = heapless::String::new();
        write!(text, "{:02}\n{:02}", self.time.hour(), self.time.minute()).unwrap();
        Text::with_text_style(&text, display.bounding_box().center(), character_style, text_style).draw(display)?;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MenuChoice {
    Workout,
    FindPhone,
    Settings,
    Firmware,
}

#[derive(Clone, Copy)]
pub struct FirmwareView {
    details: FirmwareDetails,
    item: MenuItem<'static>,
}

impl FirmwareView {
    pub fn new(details: FirmwareDetails) -> Self {
        let valid = details.validated;
        Self {
            details,
            item: MenuItem::new(if valid { "Validated" } else { "Validate" }, 2),
        }
    }

    pub fn draw<D: DrawTarget<Color = Rgb>>(&self, display: &mut D) -> Result<(), D::Error> {
        display.clear(Rgb::BLACK)?;

        // First draw the firmware info.
        self.details.draw(display)?;

        // Then a clickable item to mark it as working.
        self.item.draw(display)?;
        Ok(())
    }

    pub fn validated(&self, pos: Point) -> bool {
        self.item.is_contained_by(pos)
    }
}

#[derive(Clone, Copy)]
pub enum MenuView<'a> {
    Main {
        workout: MenuItem<'a>,
        find_phone: MenuItem<'a>,
        settings: MenuItem<'a>,
    },
    Settings {
        firmware: MenuItem<'a>,
    },
}

impl<'a> MenuView<'a> {
    pub fn main() -> Self {
        Self::Main {
            workout: MenuItem::new("Workout", 0),
            find_phone: MenuItem::new("Find Phone", 1),
            settings: MenuItem::new("Settings", 2),
        }
    }

    pub fn settings() -> Self {
        Self::Settings {
            firmware: MenuItem::new("Firmware", 0),
        }
    }

    pub fn draw<D: DrawTarget<Color = Rgb>>(&self, display: &mut D) -> Result<(), D::Error> {
        display.clear(Rgb::BLACK)?;

        match self {
            Self::Main {
                workout,
                find_phone,
                settings,
            } => {
                workout.draw(display)?;
                find_phone.draw(display)?;
                settings.draw(display)?;
            }

            Self::Settings { firmware } => {
                firmware.draw(display)?;
            }
        }

        Ok(())
    }

    pub fn intersects(&self, pos: Point) -> Option<MenuChoice> {
        match self {
            Self::Main {
                workout,
                find_phone,
                settings,
            } => {
                if workout.is_contained_by(pos) {
                    Some(MenuChoice::Workout)
                } else if find_phone.is_contained_by(pos) {
                    Some(MenuChoice::FindPhone)
                } else if settings.is_contained_by(pos) {
                    Some(MenuChoice::Settings)
                } else {
                    None
                }
            }
            Self::Settings { firmware } => {
                if firmware.is_contained_by(pos) {
                    Some(MenuChoice::Firmware)
                } else {
                    None
                }
            }
        }
    }
}

#[derive(Clone, Copy)]
pub struct MenuItem<'a> {
    text: &'a str,
    idx: u32,
}

impl<'a> MenuItem<'a> {
    pub fn new(text: &'a str, idx: u32) -> Self {
        Self { text, idx }
    }

    pub fn draw<D: DrawTarget<Color = Rgb>>(&self, display: &mut D) -> Result<(), D::Error> {
        let line_style = PrimitiveStyleBuilder::new()
            .stroke_color(Rgb::BLUE)
            .stroke_width(1)
            .fill_color(Rgb::CSS_GRAY)
            .build();
        let (start, end) = self.placement();
        Rectangle::with_corners(start, end)
            .into_styled(line_style)
            .draw(display)?;

        Text::with_text_style(
            self.text,
            Point::new(
                (WIDTH as i32) / 2,
                self.idx as i32 * (HEIGHT as i32 / GRID_ITEMS as i32) + 47,
            ),
            menu_text_style(Rgb::BLUE),
            TextStyleBuilder::new()
                .alignment(embedded_graphics::text::Alignment::Center)
                .build(),
        )
        .draw(display)?;
        Ok(())
    }

    fn placement(&self) -> (Point, Point) {
        let start = Point::new(10, (self.idx * HEIGHT / GRID_ITEMS + 10) as i32);
        let end = Point::new(
            start.x + WIDTH as i32 - 20,
            start.y + (HEIGHT as i32 / GRID_ITEMS as i32) - 20,
        );
        (start, end)
    }

    // Check if point is within our range
    pub fn is_contained_by(&self, pos: Point) -> bool {
        let (c1, c2) = self.placement();
        c1.x <= pos.x && c1.y <= pos.y && c2.x >= pos.x && c2.y >= pos.y
    }
}
