#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::info;
use display_interface_spi::SPIInterfaceNoCS;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::interrupt::Priority;
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
use heapless::Vec;
use mipidsi::models::ST7789;
use nrf_softdevice::ble::{gatt_server, peripheral, Connection};
use nrf_softdevice::{gatt_server, raw, Softdevice};
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};
use u8g2_fonts::{fonts, FontRenderer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => spim::InterruptHandler<peripherals::TWISPI0>;
});

const ATT_MTU: usize = 32;

#[nrf_softdevice::gatt_service(uuid = "FE59")]
struct NrfDfuService {
    #[characteristic(uuid = "8EC90001-F315-4F60-9FB8-838830DAEA50", write, notify)]
    control: Vec<u8, ATT_MTU>,

    /// The maximum size of each packet is derived from the Att MTU size of the connection.
    /// The maximum Att MTU size of the DFU Service is 256 bytes (saved in NRF_SDH_BLE_GATT_MAX_MTU_SIZE),
    /// making the maximum size of the DFU Packet characteristic 253 bytes. (3 bytes are used for opcode and handle ID upon writing.)
    #[characteristic(uuid = "8EC90002-F315-4F60-9FB8-838830DAEA50", write, notify)]
    packet: Vec<u8, ATT_MTU>,
}

struct DfuConnection {
    pub connection: Connection,
    pub notify_control: bool,
    pub notify_packet: bool,
}

const NRF_DFU_PROTOCOL_VERSION: u8 = 0x01;

enum ObjectType {
    Invalid = 0,
    Command = 1,
    Data = 2,
}

impl TryFrom<u8> for ObjectType {
    type Error = ();
    fn try_from(t: u8) -> Result<Self, Self::Error> {
        match t {
            0 => Ok(Self::Invalid),
            1 => Ok(Self::Command),
            2 => Ok(Self::Data),
            _ => Err(()),
        }
    }
}

impl Into<u8> for ObjectType {
    fn into(self) -> u8 {
        match self {
            Self::Invalid => 0x00,
            Self::Command => 0x01,
            Self::Data => 0x02,
        }
    }
}

pub enum DfuRequest<'m> {
    ProtocolVersion,
    Create { obj_type: ObjectType, obj_size: u32 },
    SetReceiptNotification { target: u16 },
    Crc,
    Execute,
    Select { obj_type: ObjectType },
    MtuGet,
    Write { data: &'m [u8] },
    Ping { id: u8 },
    HwVersion,
    FwVersion { image_id: u8 },
    Abort,
}

impl DfuRequest {
    fn parse(data: &[u8]) -> Result<(Self, &[u8]), ()> {
        if data.is_empty() {
            Err(())
        } else {
            let op = data[0];
            data = &data[1..];
            let req = match op {
                0x00 => Ok((Self::ProtocolVersion, data)),
                0x01 => {
                    if data.len() < 5 {
                        Err(())
                    } else {
                        let obj_type = ObjectType::try_from(data[0])?;
                        let obj_size = u32::from_le_bytes([data[1], data[2], data[3], data[4]]);

                        Ok((Self::Create {
                            obj_type,
                            obj_size
                        }, &data[5..]))
                    }
                }
                0x02 => {
                    if data.len() < 2 {
                        Err(())
                    } else {
                        let target = u16::from_le_bytes([data[0], data[1]]);
                        Ok((Self::SetReceiptNotification {
                            target
                        }, &data[2..]))
                    }
                }
                0x03 => Ok((Self::Crc, data)),
                0x04 => Ok((Self::Execute, data)),
                0x06 => {
                    if data.len() < 1 {
                        Err(())
                    } else {
                        let obj_type = ObjectType::try_from(data[0])?;
                        Ok((Self::Select {
                            obj_type
                        }, &data[1..]))
                    }
                }
                0x07 => Ok((Self::MtuGet, data)),
                0x08 => Ok((Self::Write {
                        data
                    }, &data[data.len()..])),
                0x09 => {
                    if data.len() < 1 {
                        Err(())

                    } else {
                        let id = data[0];
                        Ok((Self::Ping {
                            id
                        }, &data[1..]))
                    }
                }
                0x0A => Ok((Self::HwVersion, data)),
                0x0B => {
                    if data.len() < 1 {
                        Err(())
                    } else {
                        let image_id = data[0];
                        Ok((Self::FwVersion {
                            image_id
                        }, &data[1..]))
                    }
                }
                0x0C => Ok((Self::Abort, data)),
                _ => Err(()),
            }
        }
    }
}

enum DfuState {

}

impl NrfDfuService {
    fn respond_control(&self, conn: &mut DfuConnection, response: &[u8]) {
        if conn.notify_control {
            self.control_notify(&conn.connection, &Vec::from_slice(response).unwrap());
        }
    }

    fn dispatch(&self, conn: &mut DfuConnection, request: DfuRequest) {
    }

    fn handle(&self, connection: &mut DfuConnection, event: NrfDfuServiceEvent) {
        match event {
            NrfDfuServiceEvent::ControlWrite(data) => {
                if let Ok((request, _)) = DfuRequest::parse(&data) {
                    self.dispatch(connection, request);
                }
            }
            NrfDfuServiceEvent::ControlCccdWrite { notifications } => {
                connection.notify_control = notifications;
            }
            NrfDfuServiceEvent::PacketWrite(data) => {
                let request = DfuRequest::Write {
                    data: &data[..],
                };
                self.dispatch(connection, request);
            }
            NrfDfuServiceEvent::PacketCccdWrite { notifications } => {
                connection.notify_packet = notifications;
            }
        }
    }
}

pub enum DfuOp {
    ProtocolVersion,
    Create,
    SetReceiptNotification,
    Crc,
    Execute,
    Select,
    MtuGet,
    Write,
    Ping,
    HwVersionGet,
    FwVersion,
    Get,
    Abort,
}

impl DfuOp {
    fn parse(data: &[u8]) -> Result<(Self, &[u8]), ()> {
        if data.is_empty() {
            Err(())
        } else {
            let op = Self::try_from(data[0])?;
            Ok((op, &data[1..]))
        }
    }
}

impl TryFrom<u8> for DfuOp {
    type Error = ();
    fn try_from(op: u8) -> Result<Self, Self::Error> {
        match op {
            0x00 => Ok(Self::ProtocolVersion),
            0x01 => Ok(Self::Create),
            0x02 => Ok(Self::SetReceiptNotification),
            0x03 => Ok(Self::Crc),
            0x04 => Ok(Self::Execute),
            0x05 => Ok(Self::Select),
            0x06 => Ok(Self::MtuGet),
            0x07 => Ok(Self::Write),
            0x08 => Ok(Self::Ping),
            0x09 => Ok(Self::HwVersionGet),
            0x0A => Ok(Self::FwVersion),
            0x0B => Ok(Self::Get),
            0x0C => Ok(Self::Abort),
            _ => Err(()),
        }
    }
}

impl Into<u8> for DfuOp {
    fn into(self) -> u8 {
        match self {
            Self::ProtocolVersion => 0x00,
            Self::Create => 0x01,
            Self::SetReceiptNotification => 0x02,
            Self::Crc => 0x03,
            Self::Execute => 0x04,
            Self::Select => 0x05,
            Self::MtuGet => 0x06,
            Self::Write => 0x07,
            Self::Ping => 0x08,
            Self::HwVersionGet => 0x09,
            Self::FwVersion => 0x0A,
            Self::Get => 0x0B,
            Self::Abort => 0x0C,
        }
    }
}

#[nrf_softdevice::gatt_server]
struct PineTimeServer {
    dfu: NrfDfuService,
}

impl PineTimeServer {
    fn handle(&self, conn: &Connection, event: PineTimeServerEvent) {
        match event {
            PineTimeServerEvent::Dfu(event) => self.dfu.handle(conn, event),
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

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
