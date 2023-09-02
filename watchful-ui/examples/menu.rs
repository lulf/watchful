use embedded_graphics::mono_font::ascii::FONT_6X9;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565 as Rgb;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;
use embedded_graphics_simulator::{BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, Window};
use watchful_ui::*;

fn main() -> Result<(), core::convert::Infallible> {
    let mut display = SimulatorDisplay::<Rgb>::new(Size::new(240, 240));

    let view = MenuView::main();
    view.draw(&mut display)?;
    let output_settings = OutputSettingsBuilder::new().scale(1).build();

    Window::new("Main", &output_settings).show_static(&display);

    let view = MenuView::settings();
    view.draw(&mut display)?;
    let output_settings = OutputSettingsBuilder::new().scale(1).build();

    Window::new("Settings", &output_settings).show_static(&display);

    let mut display = SimulatorDisplay::<Rgb>::new(Size::new(240, 240));

    let view = MenuView::firmware_settings(FirmwareDetails::new(
        "watchful-os",
        "0.1.0",
        "abcdefg",
        "2021-02-19T21:32:22.932833758+00:00",
        33,
        false,
        false,
    ));
    view.draw(&mut display)?;
    Window::new("Firmware", &output_settings).show_static(&display);

    let mut display = SimulatorDisplay::<Rgb>::new(Size::new(240, 240));
    let t = time::OffsetDateTime::now_utc();
    let view = TimeView::new(time::PrimitiveDateTime::new(t.date(), t.time()), 5, false);
    view.draw(&mut display)?;
    Window::new("Time", &output_settings).show_static(&display);
    Ok(())
}
