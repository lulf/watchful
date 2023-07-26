#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;

use defmt::{info, warn, Format};
use display_interface_spi::SPIInterfaceNoCS;
use embassy_boot_nrf::{AlignedBuffer, FirmwareUpdater, FirmwareUpdaterConfig};
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::block_on;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::peripherals::{P0_18, P0_26, TWISPI0};
use embassy_nrf::spim::Spim;
use embassy_nrf::spis::MODE_3;
use embassy_nrf::{bind_interrupts, interrupt, pac, peripherals, spim};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::{Channel, DynamicSender, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::image::{Image, ImageRawLE};
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10};
use embedded_graphics::mono_font::iso_8859_15::FONT_9X18_BOLD;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::{BinaryColor, Rgb565 as Rgb};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_storage_async::nor_flash::NorFlash;
//use embedded_text::alignment::HorizontalAlignment;
use embedded_text::style::{HeightMode, TextBoxStyleBuilder};
use embedded_text::TextBox;
use heapless::Vec;
use mipidsi::models::ST7789;
use nrf_dfu::prelude::*;
use nrf_softdevice::ble::gatt_server::NotifyValueError;
use nrf_softdevice::ble::{gatt_client, gatt_server, peripheral, Connection};
use nrf_softdevice::{raw, Flash, Softdevice};
use static_cell::StaticCell;
use u8g2_fonts::types::{FontColor, HorizontalAlignment, VerticalPosition};
use u8g2_fonts::{fonts, FontRenderer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => spim::InterruptHandler<peripherals::TWISPI0>;
});

const MTU: usize = 120;
// Aligned to 4 bytes + 3 bytes for header
const ATT_MTU: usize = MTU + 3;

type Target = DfuTarget<MTU>;

// Executor for processing soc events from softdevice at a higher priority.
static EXECUTOR_SOC: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI0_EGU0() {
    EXECUTOR_SOC.on_interrupt()
}

static CURRENT_TIME: Mutex<CriticalSectionRawMutex, RefCell<time::PrimitiveDateTime>> =
    Mutex::new(RefCell::new(time::PrimitiveDateTime::MIN));

#[embassy_executor::main]
async fn main(s: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    let sd = enable_softdevice("Pinetime Embassy");
    static FLASH: StaticCell<Mutex<NoopRawMutex, Flash>> = StaticCell::new();
    let flash = FLASH.init(Mutex::new(Flash::take(sd)));

    static GATT: StaticCell<PineTimeServer> = StaticCell::new();
    let server = GATT.init(PineTimeServer::new(sd).unwrap());

    interrupt::SWI0_EGU0.set_priority(Priority::P5);
    EXECUTOR_SOC
        .start(interrupt::SWI0_EGU0)
        .spawn(softdevice_soc_task())
        .unwrap();

    s.spawn(softdevice_ble_task(sd)).unwrap();
    s.spawn(watchdog_task()).unwrap();

    static CURRENT_TIME: Mutex<CriticalSectionRawMutex, RefCell<time::PrimitiveDateTime>> =
        Mutex::new(RefCell::new(time::PrimitiveDateTime::MIN));
    s.spawn(advertiser_task(s, sd, server, flash, &CURRENT_TIME, "Pinetime Embassy"))
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
                display_time(&mut display, &CURRENT_TIME).await;
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

async fn display_time(
    display: &mut Display,
    current_time: &Mutex<CriticalSectionRawMutex, RefCell<time::PrimitiveDateTime>>,
) {
    //mipidsi::Display<DI, MODEL, RST>) {
    let current_time = current_time.lock().await;
    let current_time = current_time.borrow();
    let mut text: heapless::String<16> = heapless::String::new();
    use core::fmt::Write;
    write!(text, "{:02}:{:02}", current_time.hour(), current_time.minute()).unwrap();
    let font = FontRenderer::new::<fonts::u8g2_font_spleen32x64_mu>();

    font.render_aligned(
        text.as_ref(),
        display.bounding_box().center() + Point::new(0, 0),
        VerticalPosition::Baseline,
        HorizontalAlignment::Center,
        FontColor::Transparent(Rgb::YELLOW),
        display,
    )
    .unwrap();
}

#[nrf_softdevice::gatt_service(uuid = "FE59")]
pub struct NrfDfuService {
    #[characteristic(uuid = "8EC90001-F315-4F60-9FB8-838830DAEA50", write, notify)]
    control: Vec<u8, ATT_MTU>,

    /// The maximum size of each packet is derived from the Att MTU size of the connection.
    /// The maximum Att MTU size of the DFU Service is 256 bytes (saved in NRF_SDH_BLE_GATT_MAX_MTU_SIZE),
    /// making the maximum size of the DFU Packet characteristic 253 bytes. (3 bytes are used for opcode and handle ID upon writing.)
    #[characteristic(uuid = "8EC90002-F315-4F60-9FB8-838830DAEA50", write_without_response, notify)]
    packet: Vec<u8, ATT_MTU>,
}

struct DfuConnection {
    pub connection: Connection,
    pub notify_control: bool,
    pub notify_packet: bool,
}

impl NrfDfuService {
    fn process<DFU: NorFlash, F: FnOnce(&DfuConnection, &[u8]) -> Result<(), NotifyValueError>>(
        &self,
        target: &mut Target,
        dfu: &mut DFU,
        conn: &mut DfuConnection,
        request: DfuRequest<'_>,
        notify: F,
    ) -> DfuStatus {
        let (response, status) = target.process(request, dfu);
        let mut buf: [u8; 32] = [0; 32];
        match response.encode(&mut buf[..]) {
            Ok(len) => match notify(&conn, &buf[..len]) {
                Ok(_) => {}
                Err(e) => {
                    warn!("Error sending notification: {:?}", e);
                }
            },
            Err(e) => {
                warn!("Error encoding DFU response: {:?}", e);
            }
        }
        status
    }

    fn handle<DFU: NorFlash>(
        &self,
        target: &mut Target,
        dfu: &mut DFU,
        connection: &mut DfuConnection,
        event: NrfDfuServiceEvent,
    ) -> Option<DfuStatus> {
        match event {
            NrfDfuServiceEvent::ControlWrite(data) => {
                if let Ok((request, _)) = DfuRequest::decode(&data) {
                    return Some(self.process(target, dfu, connection, request, |conn, response| {
                        if conn.notify_control {
                            self.control_notify(&conn.connection, &Vec::from_slice(response).unwrap())?;
                        }
                        Ok(())
                    }));
                }
            }
            NrfDfuServiceEvent::ControlCccdWrite { notifications } => {
                connection.notify_control = notifications;
            }
            NrfDfuServiceEvent::PacketWrite(data) => {
                let request = DfuRequest::Write { data: &data[..] };
                return Some(self.process(target, dfu, connection, request, |conn, response| {
                    if conn.notify_packet {
                        self.packet_notify(&conn.connection, &Vec::from_slice(response).unwrap())?;
                    }
                    Ok(())
                }));
            }
            NrfDfuServiceEvent::PacketCccdWrite { notifications } => {
                connection.notify_packet = notifications;
            }
        }
        None
    }
}

#[nrf_softdevice::gatt_server]
pub struct PineTimeServer {
    dfu: NrfDfuService,
}

#[nrf_softdevice::gatt_client(uuid = "1805")]
pub struct CurrentTimeServiceClient {
    #[characteristic(uuid = "2a2b", write, read, notify)]
    current_time: Vec<u8, 10>,
}

type TimeKeeper = DynamicSender<'static, time::PrimitiveDateTime>;

impl CurrentTimeServiceClient {
    async fn get_time(&self) -> Result<time::PrimitiveDateTime, gatt_client::ReadError> {
        let data = self.current_time_read().await?;
        if data.len() == 10 {
            let year = u16::from_le_bytes([data[0], data[1]]);
            let month = data[2];
            let day = data[3];
            let hour = data[4];
            let minute = data[5];
            let second = data[6];
            let _weekday = data[7];
            let secs_frac = data[8];

            if let Ok(month) = month.try_into() {
                let date = time::Date::from_calendar_date(year as i32, month, day);
                let micros = secs_frac as u32 * 1000000 / 256;
                let time = time::Time::from_hms_micro(hour, minute, second, micros);
                if let (Ok(time), Ok(date)) = (time, date) {
                    let dt = time::PrimitiveDateTime::new(date, time);
                    return Ok(dt);
                }
            }
        }
        Err(gatt_client::ReadError::Truncated)
    }
}

impl PineTimeServer {
    fn handle<DFU: NorFlash>(
        &self,
        target: &mut Target,
        dfu: &mut DFU,
        conn: &mut DfuConnection,
        event: PineTimeServerEvent,
    ) -> Option<DfuStatus> {
        match event {
            PineTimeServerEvent::Dfu(event) => self.dfu.handle(target, dfu, conn, event),
        }
    }
}

#[embassy_executor::task(pool_size = "1")]
pub async fn gatt_server_task(
    conn: Connection,
    server: &'static PineTimeServer,
    flash: &'static Mutex<NoopRawMutex, Flash>,
) {
    let p = unsafe { pac::Peripherals::steal() };
    let part = p.FICR.info.part.read().part().bits();
    let variant = p.FICR.info.variant.read().variant().bits();

    let hw_info = HardwareInfo {
        part,
        variant,
        rom_size: 0,
        ram_size: 0,
        rom_page_size: 0,
    };

    let fw_info = FirmwareInfo {
        ftype: FirmwareType::Application,
        version: 1,
        addr: 0,
        len: 0,
    };

    let mut dfu_conn = DfuConnection {
        connection: conn.clone(),
        notify_control: false,
        notify_packet: false,
    };

    info!("Running GATT server");
    let config = FirmwareUpdaterConfig::from_linkerfile(flash);
    let mut target = DfuTarget::new(config.dfu.size(), fw_info, hw_info);
    let mut magic = AlignedBuffer([0; 4]);
    let mut updater = FirmwareUpdater::new(config);
    updater
        .mark_booted(&mut magic.0[..])
        .await
        .expect("Failed to mark current firmware as good");

    let _ = gatt_server::run(&conn, server, |e| {
        let mut config = FirmwareUpdaterConfig::from_linkerfile(flash);
        if let Some(DfuStatus::Done) = server.handle(&mut target, &mut config.dfu, &mut dfu_conn, e) {
            let mut updater = FirmwareUpdater::new(config);
            match block_on(updater.mark_updated(&mut magic.0[..])) {
                Ok(_) => {
                    info!("Firmware updated, resetting");
                    cortex_m::peripheral::SCB::sys_reset();
                }
                Err(e) => {
                    panic!("Error marking firmware updated: {:?}", e);
                }
            }
        }
    })
    .await;
    info!("Disconnected");
}

#[embassy_executor::task]
pub async fn advertiser_task(
    spawner: Spawner,
    sd: &'static Softdevice,
    server: &'static PineTimeServer,
    flash: &'static Mutex<NoopRawMutex, Flash>,
    current_time: &'static Mutex<CriticalSectionRawMutex, RefCell<time::PrimitiveDateTime>>,
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

        info!("Connection established");
        if let Ok(time_client) = gatt_client::discover::<CurrentTimeServiceClient>(&conn).await {
            info!("Found time server on peer, synchronizing time");
            match time_client.get_time().await {
                Ok(time) => {
                    info!("Got time from peer: {:?}", defmt::Debug2Format(&time));
                    let current = current_time.lock().await;
                    let mut current = current.borrow_mut();
                    *current = time;
                }
                Err(e) => {
                    info!("Error retrieving time: {:?}", e);
                }
            }
        }

        if let Err(e) = spawner.spawn(gatt_server_task(conn, server, flash)) {
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
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t {
            att_mtu: ATT_MTU as u16,
        }),
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
async fn softdevice_soc_task() {
    let sd = unsafe { Softdevice::steal() };
    sd.run_soc().await;
}

#[embassy_executor::task]
async fn softdevice_ble_task(sd: &'static Softdevice) {
    sd.run_ble().await;
}

// Keeps our system alive
#[embassy_executor::task]
async fn watchdog_task() {
    let mut handle = unsafe { embassy_nrf::wdt::WatchdogHandle::steal(0) };
    loop {
        handle.pet();
        Timer::after(Duration::from_secs(4)).await;
    }
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
