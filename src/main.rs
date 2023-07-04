#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;

use defmt::{info, warn};
use display_interface_spi::SPIInterfaceNoCS;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals::{P0_18, P0_26, TWISPI0};
use embassy_nrf::spim::Spim;
use embassy_nrf::spis::MODE_3;
use embassy_nrf::{bind_interrupts, pac, peripherals, spim};
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
use nrf_softdevice::ble::gatt_server::NotifyValueError;
use nrf_softdevice::ble::{gatt_server, peripheral, Connection, DisconnectedError};
use nrf_softdevice::{gatt_server, raw, Softdevice};
use static_cell::StaticCell;
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};
use u8g2_fonts::{fonts, FontRenderer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => spim::InterruptHandler<peripherals::TWISPI0>;
});

const ATT_MTU: usize = 32;

#[nrf_softdevice::gatt_service(uuid = "FE59")]
pub struct NrfDfuService {
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

#[derive(Copy, Clone, Debug)]
pub enum ObjectType {
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

struct ReadBuf<'m> {
    data: &'m [u8],
    pos: usize,
}

impl<'m> ReadBuf<'m> {
    fn new(data: &'m [u8]) -> ReadBuf<'m> {
        Self { data, pos: 0 }
    }

    fn decode_u8(&mut self) -> Result<u8, ()> {
        if self.data.len() - self.pos >= 1 {
            let b = self.data[self.pos];
            self.pos += 1;
            Ok(b)
        } else {
            Err(())
        }
    }

    fn decode_u16(&mut self) -> Result<u16, ()> {
        if self.data.len() - self.pos >= 2 {
            let b = u16::from_le_bytes([self.data[self.pos], self.data[self.pos + 1]]);
            self.pos += 2;
            Ok(b)
        } else {
            Err(())
        }
    }

    fn decode_u32(&mut self) -> Result<u32, ()> {
        if self.data.len() - self.pos >= 4 {
            let b = u32::from_le_bytes([
                self.data[self.pos],
                self.data[self.pos + 1],
                self.data[self.pos + 2],
                self.data[self.pos + 3],
            ]);
            self.pos += 4;
            Ok(b)
        } else {
            Err(())
        }
    }

    fn slice(&mut self) -> &'m [u8] {
        let s = &self.data[self.pos..];
        self.pos += s.len();
        s
    }

    fn release(self) -> &'m [u8] {
        &self.data[self.pos..]
    }
}

struct WriteBuf<'m> {
    data: &'m mut [u8],
    pos: usize,
}

impl<'m> WriteBuf<'m> {
    fn new(data: &'m mut [u8]) -> WriteBuf<'m> {
        Self { data, pos: 0 }
    }

    fn encode_u8(&mut self, value: u8) -> Result<(), ()> {
        if self.data.len() - self.pos >= 1 {
            self.data[self.pos] = value;
            Ok(())
        } else {
            Err(())
        }
    }

    fn encode_u16(&mut self, value: u16) -> Result<(), ()> {
        if self.data.len() - self.pos >= 2 {
            let d = value.to_le_bytes();
            self.data[self.pos] = d[0];
            self.data[self.pos + 1] = d[1];
            self.pos += 2;
            Ok(())
        } else {
            Err(())
        }
    }

    fn encode_u32(&mut self, value: u32) -> Result<(), ()> {
        if self.data.len() - self.pos >= 4 {
            let d = value.to_le_bytes();
            self.data[self.pos] = d[0];
            self.data[self.pos + 1] = d[1];
            self.data[self.pos + 2] = d[2];
            self.data[self.pos + 3] = d[3];
            self.pos += 4;
            Ok(())
        } else {
            Err(())
        }
    }

    fn release(self) -> usize {
        self.pos
    }
}

impl<'m> DfuRequest<'m> {
    fn decode(data: &'m [u8]) -> Result<(DfuRequest<'m>, &'m [u8]), ()> {
        let mut data = ReadBuf::new(data);
        let op = data.decode_u8()?;
        let req = match op {
            0x00 => Ok(Self::ProtocolVersion),
            0x01 => {
                let obj_type = ObjectType::try_from(data.decode_u8()?)?;
                let obj_size = data.decode_u32()?;

                Ok(Self::Create { obj_type, obj_size })
            }
            0x02 => {
                let target = data.decode_u16()?;
                Ok(Self::SetReceiptNotification { target })
            }
            0x03 => Ok(Self::Crc),
            0x04 => Ok(Self::Execute),
            0x06 => {
                let obj_type = ObjectType::try_from(data.decode_u8()?)?;
                Ok(Self::Select { obj_type })
            }
            0x07 => Ok(Self::MtuGet),
            0x08 => Ok(Self::Write { data: data.slice() }),
            0x09 => {
                let id = data.decode_u8()?;
                Ok(Self::Ping { id })
            }
            0x0A => Ok(Self::HwVersion),
            0x0B => {
                let image_id = data.decode_u8()?;
                Ok(Self::FwVersion { image_id })
            }
            0x0C => Ok(Self::Abort),
            _ => Err(()),
        }?;

        Ok((req, data.release()))
    }
}

pub enum DfuResponse {
    ProtocolVersion {
        version: u8,
    },
    Crc {
        offset: u32,
        crc: u32,
    },
    Select {
        offset: u32,
        crc: u32,
        max_size: u32,
    },
    Mtu {
        mtu: u16,
    },
    Write {
        crc: u32,
    },
    Ping {
        id: u8,
    },
    HwVersion {
        part: u32,
        variant: u32,
        rom_size: u32,
        ram_size: u32,
        rom_page_size: u32,
    },
    FwVersion {
        ftype: FirmwareType,
        version: u32,
        addr: u32,
        len: u32,
    },
}

impl DfuResponse {
    fn encode(&self, buf: &mut [u8]) -> Result<usize, ()> {
        let mut buf = WriteBuf::new(buf);
        match &self {
            DfuResponse::ProtocolVersion { version } => buf.encode_u8(*version)?,
            DfuResponse::Crc { offset, crc } => {
                buf.encode_u32(*offset)?;
                buf.encode_u32(*crc)?;
            }
            DfuResponse::Select { offset, crc, max_size } => {
                buf.encode_u32(*offset)?;
                buf.encode_u32(*crc)?;
                buf.encode_u32(*max_size)?;
            }
            DfuResponse::Mtu { mtu } => {
                buf.encode_u16(*mtu)?;
            }
            DfuResponse::Write { crc } => {
                buf.encode_u32(*crc)?;
            }
            DfuResponse::Ping { id } => {
                buf.encode_u8(*id)?;
            }
            DfuResponse::HwVersion {
                part,
                variant,
                rom_size,
                ram_size,
                rom_page_size,
            } => {
                buf.encode_u32(*part)?;
                buf.encode_u32(*variant)?;
                buf.encode_u32(*rom_size)?;
                buf.encode_u32(*ram_size)?;
                buf.encode_u32(*rom_page_size)?;
            }
            DfuResponse::FwVersion {
                ftype,
                version,
                addr,
                len,
            } => {
                buf.encode_u8((*ftype).into())?;
                buf.encode_u32(*version)?;
                buf.encode_u32(*addr)?;
                buf.encode_u32(*len)?;
            }
        }
        Ok(buf.release())
    }
}

#[derive(Copy, Clone, Debug)]
pub enum FirmwareType {
    Softdevice,
    Application,
    Bootloader,
    Unknown,
}

impl Into<u8> for FirmwareType {
    fn into(self) -> u8 {
        match self {
            Self::Softdevice => 0x00,
            Self::Application => 0x01,
            Self::Bootloader => 0x02,
            Self::Unknown => 0xFF,
        }
    }
}

enum DfuState {
    WaitForData,
    ReceiveFirmware,
    Validate,
}

pub struct DfuTarget {
    crc_receipt_interval: u16,
    receipt_count: usize,
    objects: [Object; 2],
    current: usize,
}

pub struct Object {
    obj_type: ObjectType,
    offset: u32,
    crc: u32,
    size: u32,
}

const DFU_MTU: u16 = 32;

impl DfuTarget {
    pub fn new() -> Self {
        Self {
            crc_receipt_interval: 0,
            receipt_count: 0,
            objects: [
                Object {
                    obj_type: ObjectType::Command,
                    offset: 0,
                    crc: 0,
                    size: 0,
                },
                Object {
                    obj_type: ObjectType::Data,
                    offset: 0,
                    crc: 0,
                    size: 0,
                },
            ],
            current: 0,
        }
    }

    fn process<'m>(&mut self, request: DfuRequest<'m>) -> Result<Option<DfuResponse>, ()> {
        match request {
            DfuRequest::ProtocolVersion => Ok(Some(DfuResponse::ProtocolVersion {
                version: NRF_DFU_PROTOCOL_VERSION,
            })),
            DfuRequest::Create { obj_type, obj_size } => {
                let idx = match obj_type {
                    ObjectType::Command => Some(0),

                    ObjectType::Data => Some(1),
                    _ => None,
                };
                if let Some(idx) = idx {
                    self.objects[idx] = Object {
                        obj_type,
                        size: obj_size,
                        offset: 0,
                        crc: 0,
                    };
                    self.current = idx;
                }
                Ok(None)
            }
            DfuRequest::SetReceiptNotification { target } => {
                self.crc_receipt_interval = target;
                Ok(None)
            }
            DfuRequest::Crc => Ok(Some(DfuResponse::Crc {
                offset: self.objects[self.current].offset,
                crc: self.objects[self.current].crc,
            })),
            DfuRequest::Execute => {
                // TODO: If transfer complete, validate and swap
                Ok(None)
            }
            DfuRequest::Select { obj_type } => {
                let idx = match obj_type {
                    ObjectType::Command => Some(0),

                    ObjectType::Data => Some(1),
                    _ => None,
                };
                if let Some(idx) = idx {
                    Ok(Some(DfuResponse::Select {
                        offset: self.objects[idx].offset,
                        crc: self.objects[idx].crc,
                        max_size: self.objects[idx].size,
                    }))
                } else {
                    Ok(None)
                }
            }

            DfuRequest::MtuGet => Ok(Some(DfuResponse::Mtu { mtu: DFU_MTU })),
            DfuRequest::Write { data } => {
                self.receipt_count += 1;
                if self.crc_receipt_interval > 0 {
                    let obj = &self.objects[self.current];
                    self.receipt_count = 0;
                    Ok(Some(DfuResponse::Crc {
                        offset: obj.offset,
                        crc: obj.crc,
                    }))
                } else {
                    Ok(None)
                }
            }
            DfuRequest::Ping { id } => Ok(Some(DfuResponse::Ping { id })),
            DfuRequest::HwVersion => {
                let p = unsafe { pac::Peripherals::steal() };
                let part = p.FICR.info.part.read().part().bits();
                let variant = p.FICR.info.variant.read().variant().bits();
                let rom_size = 12345;
                let ram_size = 5678;
                let rom_page_size = 4096;
                Ok(Some(DfuResponse::HwVersion {
                    part,
                    variant,
                    rom_size,
                    ram_size,
                    rom_page_size,
                }))
            }
            DfuRequest::FwVersion { image_id } => {
                let ftype = FirmwareType::Application;
                let version = 0;
                let addr = 0;
                let len = 1024;
                Ok(Some(DfuResponse::FwVersion {
                    ftype,
                    version,
                    addr,
                    len,
                }))
            }
            DfuRequest::Abort => {
                self.objects[0].crc = 0;
                self.objects[0].offset = 0;
                self.objects[1].crc = 0;
                self.objects[1].offset = 0;
                self.receipt_count = 0;
                Ok(None)
            }
        }
    }
}

impl NrfDfuService {
    fn process<F: FnOnce(&DfuConnection, &[u8]) -> Result<(), NotifyValueError>>(
        &self,
        target: &mut DfuTarget,
        conn: &mut DfuConnection,
        request: DfuRequest,
        notify: F,
    ) {
        match target.process(request) {
            Ok(Some(response)) => {
                let mut buf: [u8; 32] = [0; 32];
                match response.encode(&mut buf[..]) {
                    Ok(len) => match notify(&conn, &buf[..len]) {
                        Ok(_) => {
                            info!("Notification sent successfully");
                        }
                        Err(e) => {
                            warn!("Error sending notification: {:?}", e);
                        }
                    },
                    Err(e) => {
                        warn!("Error encoding DFU response");
                    }
                }
            }
            Ok(None) => {
                info!("No response for request");
            }
            Err(_) => {
                warn!("Error processing DFU requst");
            }
        }
    }

    fn handle(&self, target: &mut DfuTarget, connection: &mut DfuConnection, event: NrfDfuServiceEvent) {
        match event {
            NrfDfuServiceEvent::ControlWrite(data) => {
                if let Ok((request, _)) = DfuRequest::decode(&data) {
                    self.process(target, connection, request, |conn, response| {
                        if conn.notify_control {
                            self.control_notify(&conn.connection, &Vec::from_slice(response).unwrap())
                        } else {
                            Ok(())
                        }
                    });
                }
            }
            NrfDfuServiceEvent::ControlCccdWrite { notifications } => {
                connection.notify_control = notifications;
            }
            NrfDfuServiceEvent::PacketWrite(data) => {
                let request = DfuRequest::Write { data: &data[..] };
                self.process(target, connection, request, |conn, response| {
                    if conn.notify_packet {
                        self.packet_notify(&conn.connection, &Vec::from_slice(response).unwrap())
                    } else {
                        Ok(())
                    }
                });
            }
            NrfDfuServiceEvent::PacketCccdWrite { notifications } => {
                connection.notify_packet = notifications;
            }
        }
    }
}

#[nrf_softdevice::gatt_server]
pub struct PineTimeServer {
    dfu: NrfDfuService,
}

impl PineTimeServer {
    fn handle(&self, target: &mut DfuTarget, conn: &mut DfuConnection, event: PineTimeServerEvent) {
        match event {
            PineTimeServerEvent::Dfu(event) => self.dfu.handle(target, conn, event),
        }
    }
}

#[embassy_executor::main]
async fn main(s: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    let sd = enable_softdevice("Pinetime Embassy");

    static GATT: StaticCell<PineTimeServer> = StaticCell::new();
    let server = GATT.init(PineTimeServer::new(sd).unwrap());

    s.spawn(softdevice_task(sd)).unwrap();

    static TARGET: StaticCell<Mutex<CriticalSectionRawMutex, RefCell<DfuTarget>>> = StaticCell::new();
    let target = TARGET.init(Mutex::new(RefCell::new(DfuTarget::new())));

    s.spawn(advertiser_task(s, sd, server, target, "Pinetime Embassy"))
        .unwrap();

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

#[embassy_executor::task(pool_size = "4")]
pub async fn gatt_server_task(
    sd: &'static Softdevice,
    conn: Connection,
    server: &'static PineTimeServer,
    target: &'static Mutex<CriticalSectionRawMutex, RefCell<DfuTarget>>,
) {
    let mut dfu_conn = DfuConnection {
        connection: conn.clone(),
        notify_control: false,
        notify_packet: false,
    };

    loop {
        let target = target.lock().await;
        let mut target = target.borrow_mut();
        let _ = gatt_server::run(&conn, server, |e| server.handle(&mut target, &mut dfu_conn, e)).await;
        info!("Disconnected");
    }
}

use embassy_sync::mutex::Mutex;

#[embassy_executor::task]
pub async fn advertiser_task(
    spawner: Spawner,
    sd: &'static Softdevice,
    server: &'static PineTimeServer,
    target: &'static Mutex<CriticalSectionRawMutex, RefCell<DfuTarget>>,
    name: &'static str,
) {
    let mut adv_data: Vec<u8, 31> = Vec::new();
    #[rustfmt::skip]
    adv_data.extend_from_slice(&[
        0x02, 0x01, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
        0x03, 0x03, 0xFE, 0x59,
        (1 + name.len() as u8), 0x09]).unwrap();

    adv_data.extend_from_slice(name.as_bytes()).ok().unwrap();

    #[rustfmt::skip]
    let scan_data = &[
        0x03, 0x03, 0x0A, 0x18,
    ];

    loop {
        let config = peripheral::Config::default();
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &adv_data[..],
            scan_data,
        };
        info!("Advertising");
        let conn = peripheral::advertise_connectable(sd, adv, &config).await.unwrap();

        info!("connection established");
        if let Err(e) = spawner.spawn(gatt_server_task(sd, conn, server, target)) {
            defmt::info!("Error spawning gatt task: {:?}", e);
        }
    }
}

fn enable_softdevice(name: &'static str) -> &'static mut Softdevice {
    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 4,
            rc_temp_ctiv: 2,
            accuracy: 7,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 2,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 128 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t { attr_tab_size: 32768 }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 1,
            central_sec_count: 1,
            _bitfield_1: Default::default(),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: name.as_ptr() as *const u8 as _,
            current_len: name.len() as u16,
            max_len: name.len() as u16,
            write_perm: unsafe { core::mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(raw::BLE_GATTS_VLOC_STACK as u8),
        }),
        ..Default::default()
    };
    Softdevice::enable(&config)
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
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
