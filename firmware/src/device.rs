use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{AnyPin, Input, Level, Output, OutputDrive};
use embassy_nrf::peripherals::{TWISPI0, TWISPI1};
use embassy_nrf::spim::Spim;
use embassy_nrf::{saadc, twim};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer};
use mipidsi::models::ST7789;

use crate::clock::Clock;
use crate::firmware_validator::FirmwareValidator;

pub type Touchpad<'a> = cst816s::CST816S<I2cDevice<'a, NoopRawMutex, twim::Twim<'a, TWISPI1>>, Input<'a>, Output<'a>>;
pub type Hrs<'a> = hrs3300::Hrs3300<I2cDevice<'a, NoopRawMutex, twim::Twim<'a, TWISPI1>>>;
pub type Display<'a> = mipidsi::Display<
    SPIInterface<SpiDevice<'a, NoopRawMutex, Spim<'a, TWISPI0>, Output<'a>>, Output<'a>>,
    ST7789,
    Output<'a>,
>;

pub struct Device<'a> {
    pub clock: &'a Clock,
    pub screen: Screen<'static>,
    pub button: Button,
    pub battery: Battery<'static>,
    pub touchpad: Touchpad<'static>,
    pub hrs: Hrs<'static>,
    pub firmware_validator: FirmwareValidator<'static>,
}

impl<'a> Device<'a> {}

pub struct Button {
    pin: Input<'static>,
}

impl Button {
    pub fn new(pin: Input<'static>) -> Self {
        Self { pin }
    }
    pub async fn wait(&mut self) {
        self.pin.wait_for_any_edge().await;
        if self.pin.is_high() {
            match select(Timer::after(Duration::from_secs(8)), self.pin.wait_for_falling_edge()).await {
                Either::First(_) => {
                    if self.pin.is_high() {
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                }
                Either::Second(_) => {}
            }
        }
    }
}

pub struct Battery<'a> {
    charging: Input<'a>,
    adc: saadc::Saadc<'a, 1>,
}

impl<'a> Battery<'a> {
    pub fn new(adc: saadc::Saadc<'a, 1>, charging: Input<'a>) -> Self {
        Self { adc, charging }
    }
    pub async fn measure(&mut self) -> u32 {
        let mut buf = [0i16; 1];
        self.adc.sample(&mut buf).await;
        let voltage = buf[0] as u32 * (8 * 600) / 1024;
        //let voltage = buf[0] as u32 * 2000 / 1241;
        approximate_charge(voltage)
    }

    pub fn is_charging(&mut self) -> bool {
        self.charging.is_low()
    }
}

pub enum BacklightLevel {
    Low,
    Medium,
    High,
}

pub struct Backlight<'a> {
    low: Output<'a>,
    med: Output<'a>,
    high: Output<'a>,
    level: BacklightLevel,
}

impl<'a> Backlight<'a> {
    pub fn new(low_pin: AnyPin, med_pin: AnyPin, high_pin: AnyPin) -> Self {
        let backlight_low = Output::new(low_pin, Level::High, OutputDrive::Standard); // Low backlight
        let backlight_med = Output::new(med_pin, Level::High, OutputDrive::Standard); // Medium backlight
        let backlight_high = Output::new(high_pin, Level::High, OutputDrive::Standard); // High backlight
        Self {
            low: backlight_low,
            med: backlight_med,
            high: backlight_high,
            level: BacklightLevel::Medium,
        }
    }

    fn set_level(&mut self, level: BacklightLevel) {
        self.level = level;
    }

    fn off(&mut self) {
        self.low.set_high();
        self.med.set_high();
        self.high.set_high();
    }

    fn on(&mut self) {
        match self.level {
            BacklightLevel::Low => self.set_low(),
            BacklightLevel::Medium => self.set_medium(),
            BacklightLevel::High => self.set_high(),
        }
    }

    fn set_low(&mut self) {
        self.low.set_low();
        self.med.set_high();
        self.high.set_high();
    }

    fn set_medium(&mut self) {
        self.low.set_high();
        self.med.set_low();
        self.high.set_high();
    }

    fn set_high(&mut self) {
        self.low.set_high();
        self.med.set_high();
        self.high.set_low();
    }
}

pub struct Screen<'a> {
    display: Display<'a>,
    backlight: Backlight<'a>,
}

impl<'a> Screen<'a> {
    pub fn new(display: Display<'a>, backlight: Backlight<'a>) -> Self {
        Self { display, backlight }
    }

    pub fn display(&mut self) -> &mut Display<'a> {
        &mut self.display
    }

    pub fn on(&mut self) {
        self.backlight.on();
    }

    pub fn off(&mut self) {
        self.backlight.off();
    }

    pub fn change_brightness(&mut self) {
        match self.backlight.level {
            BacklightLevel::Low => self.backlight.set_level(BacklightLevel::Medium),
            BacklightLevel::Medium => self.backlight.set_level(BacklightLevel::High),
            BacklightLevel::High => self.backlight.set_level(BacklightLevel::Low),
        }
    }
}

fn approximate_charge(voltage_millis: u32) -> u32 {
    let level_approx = &[(3500, 0), (3616, 3), (3723, 22), (3776, 48), (3979, 79), (4180, 100)];
    let approx = |value| {
        if value < level_approx[0].0 {
            level_approx[0].1
        } else {
            let mut ret = level_approx[level_approx.len() - 1].1;
            for i in 1..level_approx.len() {
                let prev = level_approx[i - 1];
                let val = level_approx[i];
                if value < val.0 {
                    ret = prev.1 + (value - prev.0) * (val.1 - prev.1) / (val.0 - prev.0);
                    break;
                }
            }
            ret
        }
    };
    approx(voltage_millis)
}
