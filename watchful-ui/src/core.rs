use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::{DrawTarget, Point};
use embedded_graphics::primitives::{Primitive, PrimitiveStyle, Rectangle, Styled};
use embedded_graphics::text::{Text, TextStyleBuilder};
use embedded_graphics::Drawable as _;
use micromath::F32Ext;
use u8g2_fonts::U8g2TextStyle;

type COLOR = Rgb565;

pub enum Shape {
    Rectangle(Point, Point),
    Circle(Point, f32),
}

impl Shape {
    pub fn contains(&self, coord: Point) -> bool {
        match self {
            Self::Rectangle(c1, c2) => c1.x <= coord.x && c1.y <= coord.y && c2.x >= coord.x && c2.y >= coord.y,
            Self::Circle(center, radius) => {
                let distance: f32 = (((coord.x - center.x) * (coord.x - center.x)) as f32
                    - ((coord.y - center.y) * (coord.y - center.y)) as f32)
                    .sqrt();
                distance < *radius
            }
        }
    }
}

pub trait Component {
    fn shape(&self) -> Option<Shape> {
        None
    }
    fn on_render<D: DrawTarget<Color = COLOR>>(&self, display: &mut D) -> Result<(), D::Error> {
        Ok(())
    }

    fn on_event(&mut self, event: InputEvent) {}

    fn dispatch(&mut self, event: InputEvent) {
        let InputEvent::Touch(gesture) = event;
        if let Some(shape) = self.shape() {
            if shape.contains(gesture.point()) {
                self.on_event(event);
            }
        }
    }
}

pub enum ButtonEvent {
    Pressed,
    Released,
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

impl TouchGesture {
    pub fn point(&self) -> Point {
        match *self {
            Self::SingleTap(point) => point,
            Self::DoubleTap(point) => point,
            Self::SwipeUp(point) => point,
            Self::SwipeDown(point) => point,
            Self::SwipeLeft(point) => point,
            Self::SwipeRight(point) => point,
        }
    }
}

pub struct Button<'a, F>
where
    F: FnMut(ButtonEvent),
{
    container: Rectangle,
    text: &'a str,
    action: F,
    button_style: PrimitiveStyle<COLOR>,
    text_style: U8g2TextStyle<COLOR>,
}

impl<'a, F> Button<'a, F>
where
    F: Fn(ButtonEvent),
{
    pub fn new(
        container: Rectangle,
        text: &'a str,
        action: F,
        button_style: PrimitiveStyle<COLOR>,
        text_style: U8g2TextStyle<COLOR>,
    ) -> Self {
        Self {
            container,
            text,
            action,
            button_style,
            text_style,
        }
    }
}

impl<'a, F> Component for Button<'a, F>
where
    F: Fn(ButtonEvent),
{
    fn on_render<D: DrawTarget<Color = COLOR>>(&self, display: &mut D) -> Result<(), D::Error> {
        self.container.into_styled(self.button_style).draw(display)?;
        Text::with_text_style(
            self.text,
            self.container.center(),
            self.text_style.clone(),
            TextStyleBuilder::new()
                .alignment(embedded_graphics::text::Alignment::Center)
                .build(),
        )
        .draw(display)?;
        Ok(())
    }
    fn on_event(&mut self, event: InputEvent) {
        if let InputEvent::Touch(TouchGesture::SingleTap(_)) = event {
            (self.action)(ButtonEvent::Pressed);
        }
    }
}

pub struct TextView<'a> {
    text: &'a str,
}

impl<'a> TextView<'a> {
    pub fn new(text: &'a str) -> Self {
        Self { text }
    }
}

impl<'a> Component for TextView<'a> {}

// A vertical view with 2 components sharing the area
pub struct VerticalView<TOP, BOTTOM>
where
    TOP: Component,
    BOTTOM: Component,
{
    top: TOP,
    bottom: BOTTOM,
}

impl<TOP, BOTTOM> VerticalView<TOP, BOTTOM>
where
    TOP: Component,
    BOTTOM: Component,
{
    pub fn new(top: TOP, bottom: BOTTOM) -> Self {
        Self { top, bottom }
    }
}

impl<TOP, BOTTOM> Component for VerticalView<TOP, BOTTOM>
where
    TOP: Component,
    BOTTOM: Component,
{
}

// A vertical view with 3 components sharing the area
pub struct VerticalView3<TOP, MIDDLE, BOTTOM>
where
    TOP: Component,
    MIDDLE: Component,
    BOTTOM: Component,
{
    top: TOP,
    middle: MIDDLE,
    bottom: BOTTOM,
}

impl<TOP, MIDDLE, BOTTOM> VerticalView3<TOP, MIDDLE, BOTTOM>
where
    TOP: Component,
    MIDDLE: Component,
    BOTTOM: Component,
{
    pub fn new(top: TOP, middle: MIDDLE, bottom: BOTTOM) -> Self {
        Self { top, middle, bottom }
    }
}

impl<TOP, MIDDLE, BOTTOM> Component for VerticalView3<TOP, MIDDLE, BOTTOM>
where
    TOP: Component,
    MIDDLE: Component,
    BOTTOM: Component,
{
}
