use embassy_boot_nrf::FirmwareState;
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{AnyPin, Input, Output};
use embassy_nrf::peripherals::{P0_10, P0_16, P0_18, P0_25, P0_26, P0_28, TWISPI0, TWISPI1};
use embassy_nrf::spim::Spim;
use embassy_nrf::{saadc, twim};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer};
use mipidsi::models::ST7789;

use crate::clock::Clock;

pub type Touchpad<'a> =
    cst816s::CST816S<I2cDevice<'a, NoopRawMutex, twim::Twim<'a, TWISPI1>>, Input<'a, P0_28>, Output<'a, P0_10>>;
pub type Hrs<'a> = hrs3300::Hrs3300<I2cDevice<'a, NoopRawMutex, twim::Twim<'a, TWISPI1>>>;
pub type Display<'a> = mipidsi::Display<
    SPIInterface<SpiDevice<'a, NoopRawMutex, Spim<'a, TWISPI0>, Output<'a, P0_25>>, Output<'a, P0_18>>,
    ST7789,
    Output<'a, P0_26>,
>;

pub struct Device<'a> {
    pub clock: &'a Clock,
    pub screen: Screen<'static>,
    pub button: Button,
    pub battery: Battery<'static>,
    pub firmware: FirmwareState<'a, crate::StatePartition<'static>>,
    pub touchpad: Touchpad<'static>,
    pub hrs: Hrs<'static>,
    pub vibrator: Vibrator<'static>,
    //pub wtimer: Wtimer,
}

impl<'a> Device<'a> {}
/*
pub struct Wtimer {
	pub running: bool,
}

impl Wtimer {
    pub fn new(running: bool) -> Self {
        Self { running }
    }
}
*/
pub struct Vibrator<'a> {
    motor: Output<'a, P0_16>,
}

impl<'a> Vibrator<'a> {
    pub fn new(motor: Output<'a, P0_16>) -> Self {
        Self { motor }
    }

    pub async fn on_for(&mut self, ms: u64) {
        self.motor.set_low();
        Timer::after(Duration::from_millis(ms)).await;
        self.motor.set_high();
    }

    pub fn off(&mut self) {
        self.motor.set_high();
    }
}

pub struct Button {
    pin: Input<'static, AnyPin>,
}

impl Button {
    pub fn new(pin: Input<'static, AnyPin>) -> Self {
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
    charging: Input<'a, AnyPin>,
    adc: saadc::Saadc<'a, 1>,
}

impl<'a> Battery<'a> {
    pub fn new(adc: saadc::Saadc<'a, 1>, charging: Input<'a, AnyPin>) -> Self {
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

pub struct Screen<'a> {
    display: Display<'a>,
    backlight: Output<'a, AnyPin>,
}

impl<'a> Screen<'a> {
    pub fn new(display: Display<'a>, backlight: Output<'a, AnyPin>) -> Self {
        Self { display, backlight }
    }

    pub fn display(&mut self) -> &mut Display<'a> {
        &mut self.display
    }

    pub fn on(&mut self) {
        self.backlight.set_low();
    }

    pub fn off(&mut self) {
        self.backlight.set_high();
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
