#![no_std]

use core::fmt::Write as _;

use embedded_graphics::image::Image;
use embedded_graphics::pixelcolor::Rgb565 as Rgb;
use embedded_graphics::prelude::{DrawTarget, *};
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::{Text, TextStyleBuilder};
use embedded_iconoir::prelude::*;
use embedded_layout::layout::linear::{spacing, LinearLayout};
use embedded_layout::prelude::*;
use embedded_text::style::TextBoxStyleBuilder;
use embedded_text::TextBox;
use u8g2_fonts::{fonts, U8g2TextStyle};

const WIDTH: u32 = 240;
const HEIGHT: u32 = 240;
const GRID_ITEMS: u32 = 3;

fn watch_text_style(color: Rgb) -> U8g2TextStyle<Rgb> {
    //U8g2TextStyle::new(fonts::u8g2_font_unifont_t_symbols, Rgb::YELLOW)
    //U8g2TextStyle::new(fonts::u8g2_font_inb57_mn, color)
    U8g2TextStyle::new(fonts::u8g2_font_logisoso58_tn, color) //u8g2_font_logisoso78_tn, color) //u8g2_font_logisoso92_tn, color) //u8g2_font_fub49_tn, color) //u8g2_font_spleen16x32_mf, color)
}

fn menu_text_style(color: Rgb) -> U8g2TextStyle<Rgb> {
    //U8g2TextStyle::new(fonts::u8g2_font_unifont_t_symbols, Rgb::YELLOW)
    U8g2TextStyle::new(fonts::u8g2_font_spleen16x32_mf, color)
}

fn date_text_style(color: Rgb) -> U8g2TextStyle<Rgb> {
    U8g2TextStyle::new(fonts::u8g2_font_spleen12x24_mf, color)
}

fn text_text_style(color: Rgb) -> U8g2TextStyle<Rgb> {
    U8g2TextStyle::new(fonts::u8g2_font_unifont_t_symbols, color)
}

pub enum ButtonEvent {
    ShortPress,
    LongPress,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum InputEvent {
    Touch(TouchGesture),
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum TouchGesture {
    SingleTap(Point),
    DoubleTap(Point),
    SwipeUp(Point),
    SwipeDown(Point),
    SwipeLeft(Point),
    SwipeRight(Point),
}

pub struct TimeView {
    time: time::PrimitiveDateTime,
    battery_level: u32,
    battery_charging: bool,
}

impl TimeView {
    pub fn new(time: time::PrimitiveDateTime, battery_level: u32, battery_charging: bool) -> Self {
        Self {
            time,
            battery_level,
            battery_charging,
        }
    }
    pub fn draw<D: DrawTarget<Color = Rgb>>(&self, display: &mut D) -> Result<(), D::Error> {
        display.clear(Rgb::BLACK)?;

        let mut buf: heapless::String<16> = heapless::String::new();
        write!(buf, "{:02}:{:02}", self.time.hour(), self.time.minute()).unwrap();
        let hm = Text::with_text_style(
            &buf,
            display.bounding_box().center(),
            watch_text_style(Rgb::CSS_DARK_CYAN),
            TextStyleBuilder::new()
                .alignment(embedded_graphics::text::Alignment::Center)
                .baseline(embedded_graphics::text::Baseline::Alphabetic)
                .build(),
        );

        let mut buf: heapless::String<16> = heapless::String::new();
        write!(buf, "{}", self.time.weekday()).unwrap();
        buf.truncate(3);
        write!(buf, " {}", self.time.day()).unwrap();
        let date = Text::with_text_style(
            &buf,
            display.bounding_box().center(),
            date_text_style(Rgb::CSS_DARK_CYAN),
            TextStyleBuilder::new()
                .alignment(embedded_graphics::text::Alignment::Center)
                .baseline(embedded_graphics::text::Baseline::Alphabetic)
                .build(),
        );

        let display_area = display.bounding_box();
        LinearLayout::vertical(Chain::new(date).append(hm))
            .with_spacing(spacing::FixedMargin(10))
            .with_alignment(horizontal::Center)
            .arrange()
            .align_to(&display_area, horizontal::Center, vertical::Center)
            .draw(display)?;

        let display_area = display_area.offset(-5);
        let top_right_y = display_area.top_left.y;
        let top_right_x = display_area.top_left.x + display_area.size.width as i32 - 30;
        let pos = Point::new(top_right_x, top_right_y);
        if self.battery_charging {
            Image::new(&icons::size24px::system::BatteryCharging::new(Rgb::CSS_DARK_CYAN), pos).draw(display)?
        } else {
            if self.battery_level > 85 {
                Image::new(&icons::size24px::system::BatteryFull::new(Rgb::CSS_DARK_CYAN), pos).draw(display)?
            } else if self.battery_level > 65 {
                Image::new(&icons::size24px::system::BatterySevenFive::new(Rgb::CSS_DARK_CYAN), pos).draw(display)?
            } else if self.battery_level > 35 {
                Image::new(&icons::size24px::system::BatteryFiveZero::new(Rgb::CSS_DARK_CYAN), pos).draw(display)?
            } else if self.battery_level > 10 {
                Image::new(&icons::size24px::system::BatteryTwoFive::new(Rgb::CSS_DARK_CYAN), pos).draw(display)?
            } else {
                Image::new(&icons::size24px::system::BatteryEmpty::new(Rgb::CSS_DARK_CYAN), pos).draw(display)?
            }
        };

        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MenuAction {
    Workout,
    FindPhone,
    Settings,
    FirmwareSettings,
    ValidateFirmware,
    Reset,
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
        reset: MenuItem<'a>,
    },
    Firmware {
        details: FirmwareDetails,
        item: MenuItem<'static>,
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
            reset: MenuItem::new("Reset", 2),
        }
    }

    pub fn firmware_settings(details: FirmwareDetails) -> Self {
        let valid = details.validated;
        Self::Firmware {
            details,
            item: MenuItem::new(if valid { "Validated" } else { "Validate" }, 2),
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

            Self::Settings { firmware, reset } => {
                firmware.draw(display)?;
                reset.draw(display)?;
            }

            Self::Firmware { details, item } => {
                details.draw(display)?;
                item.draw(display)?;
            }
        }

        Ok(())
    }

    pub fn on_event(&self, input: InputEvent) -> Option<MenuAction> {
        match self {
            Self::Main {
                workout,
                find_phone,
                settings,
            } => {
                if workout.is_clicked(input) {
                    Some(MenuAction::Workout)
                } else if find_phone.is_clicked(input) {
                    Some(MenuAction::FindPhone)
                } else if settings.is_clicked(input) {
                    Some(MenuAction::Settings)
                } else {
                    None
                }
            }
            Self::Settings { firmware, reset } => {
                if firmware.is_clicked(input) {
                    Some(MenuAction::FirmwareSettings)
                } else if reset.is_clicked(input) {
                    Some(MenuAction::Reset)
                } else {
                    None
                }
            }
            Self::Firmware { details: _, item } => {
                if item.is_clicked(input) {
                    Some(MenuAction::ValidateFirmware)
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
            .stroke_color(Rgb::CSS_DARK_CYAN)
            .stroke_width(1)
            .fill_color(Rgb::CSS_DARK_CYAN)
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
            menu_text_style(Rgb::CSS_CORNSILK),
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
    pub fn is_clicked(&self, event: InputEvent) -> bool {
        if let InputEvent::Touch(TouchGesture::SingleTap(pos)) = event {
            let (c1, c2) = self.placement();
            c1.x <= pos.x && c1.y <= pos.y && c2.x >= pos.x && c2.y >= pos.y
        } else {
            false
        }
    }
}

#[derive(Clone, Copy)]
pub struct FirmwareDetails {
    name: &'static str,
    version: &'static str,
    commit: &'static str,
    build_timestamp: &'static str,
    battery_level: u32,
    battery_charging: bool,
    validated: bool,
}

impl FirmwareDetails {
    pub const fn new(
        name: &'static str,
        version: &'static str,
        commit: &'static str,
        build_timestamp: &'static str,
        battery_level: u32,
        battery_charging: bool,
        validated: bool,
    ) -> Self {
        Self {
            name,
            version,
            commit,
            build_timestamp,
            battery_level,
            battery_charging,
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

        let character_style = text_text_style(Rgb::CSS_LIGHT_CORAL);

        let mut info: heapless::String<512> = heapless::String::new();
        write!(
            info,
            "Name: {}\nVersion: {}\nCommit: {}\nBuild: {}\nBattery: {}{}",
            self.name,
            self.version,
            self.commit,
            self.build_timestamp,
            self.battery_level,
            if self.battery_charging { "(Charging)" } else { "" }
        )
        .unwrap();

        TextBox::with_textbox_style(&info, bounds, character_style, textbox_style).draw(display)?;
        Ok(())
    }
}
