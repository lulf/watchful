#![no_std]

use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X9};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565 as Rgb;
use embedded_graphics::prelude::{DrawTarget, *};
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;

const WIDTH: u32 = 240;
const HEIGHT: u32 = 240;
const MENU_ITEMS: u32 = 3;

pub enum DisplayState {
    Watch,
    MenuView,
}

pub struct WatchView {}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MenuChoice {
    Workout,
    FindPhone,
    Settings,
    Firmware,
}

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
        display.clear(Rgb::WHITE)?;

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

pub struct MenuItem<'a> {
    text: &'a str,
    idx: u32,
}

impl<'a> MenuItem<'a> {
    pub fn new(text: &'a str, idx: u32) -> Self {
        Self { text, idx }
    }

    pub fn draw<D: DrawTarget<Color = Rgb>>(&self, display: &mut D) -> Result<(), D::Error> {
        let line_style = PrimitiveStyle::with_stroke(Rgb::YELLOW, 1);
        let (start, end) = self.placement();
        Rectangle::with_corners(start, end)
            .into_styled(line_style)
            .draw(display)?;

        let text_style = MonoTextStyle::new(&FONT_10X20, Rgb::YELLOW);
        Text::with_alignment(
            self.text,
            Point::new(
                (WIDTH as i32) / 2,
                self.idx as i32 * (HEIGHT as i32 / MENU_ITEMS as i32) + 45,
            ),
            text_style,
            embedded_graphics::text::Alignment::Center,
        )
        .draw(display)?;
        Ok(())
    }

    fn placement(&self) -> (Point, Point) {
        let start = Point::new(10, (self.idx * HEIGHT / MENU_ITEMS + 10) as i32);
        let end = Point::new(
            start.x + WIDTH as i32 - 20,
            start.y + (HEIGHT as i32 / MENU_ITEMS as i32) - 20,
        );
        (start, end)
    }

    // Check if point is within our range
    pub fn is_contained_by(&self, pos: Point) -> bool {
        let (c1, c2) = self.placement();
        c1.x <= pos.x && c1.y <= pos.y && c2.x >= pos.x && c2.y >= pos.y
    }
}
