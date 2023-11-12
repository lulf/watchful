#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;

use defmt::info;
use defmt_rtt as _;
use embassy_boot_nrf::{AlignedBuffer, FirmwareState};
use embassy_embedded_hal::flash::partition::{BlockingPartition, Partition};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals::{P0_05, TWISPI0, TWISPI1};
use embassy_nrf::spim::Spim;
use embassy_nrf::spis::MODE_3;
use embassy_nrf::twim::Twim;
use embassy_nrf::{bind_interrupts, pac, peripherals, saadc, spim, twim};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex as BMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use heapless::Vec;
use nrf_dfu_target::prelude::*;
use nrf_softdevice::ble::{gatt_server, peripheral, Connection};
use nrf_softdevice::{raw, Softdevice};
#[cfg(feature = "panic-probe")]
use panic_probe as _;
use pinetime_flash::XtFlash;
use static_cell::StaticCell;

mod ble;
mod clock;
mod device;
mod display_interface;
mod state;
use crate::clock::clock;
use crate::device::{Battery, Button, Device, Hrs, Screen};
use crate::display_interface::SPIDeviceInterface;
use crate::state::WatchState;

bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => spim::InterruptHandler<peripherals::TWISPI0>;
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1 => twim::InterruptHandler<peripherals::TWISPI1>;
    SAADC => saadc::InterruptHandler;
});

static CLOCK: clock::Clock = clock::Clock::new();

type ExternalFlash = XtFlash<SpiDevice<'static, NoopRawMutex, Spim<'static, TWISPI0>, Output<'static, P0_05>>>;

type InternalFlash = nrf_softdevice::Flash;
type StatePartition<'a> = Partition<'a, NoopRawMutex, InternalFlash>;
type DfuPartition<'a> = BlockingPartition<'a, NoopRawMutex, ExternalFlash>;

static I2C_BUS: StaticCell<BMutex<NoopRawMutex, RefCell<Twim<'static, TWISPI1>>>> = StaticCell::new();
static SPI_BUS: StaticCell<BMutex<NoopRawMutex, RefCell<Spim<'static, TWISPI0>>>> = StaticCell::new();

use core::panic::PanicInfo;

#[cfg(not(feature = "panic-probe"))]
#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::main]
async fn main(s: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    let sd = enable_softdevice("Watchful Embassy");

    static GATT: StaticCell<ble::PineTimeServer> = StaticCell::new();
    let server = GATT.init(ble::PineTimeServer::new(sd).unwrap());

    s.spawn(softdevice_task(sd)).unwrap();
    s.spawn(watchdog_task()).unwrap();
    s.spawn(clock(&CLOCK)).unwrap();

    // Battery measurement
    let mut bat_config = saadc::ChannelConfig::single_ended(p.P0_31);
    bat_config.gain = saadc::Gain::GAIN1_4;
    bat_config.resistor = saadc::Resistor::BYPASS;
    bat_config.reference = saadc::Reference::INTERNAL;
    bat_config.time = saadc::Time::_40US;
    let mut adc_config = saadc::Config::default();
    adc_config.resolution = saadc::Resolution::_10BIT;
    let saadc = saadc::Saadc::new(p.SAADC, Irqs, adc_config, [bat_config]);
    let battery = Battery::new(saadc, Input::new(p.P0_12.degrade(), Pull::Up));

    // Touch peripheral
    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K400;
    let i2c = twim::Twim::new(p.TWISPI1, Irqs, p.P0_06, p.P0_07, twim_config);
    let i2c_bus = I2C_BUS.init(BMutex::new(RefCell::new(i2c)));

    let i2c = I2cDevice::new(i2c_bus);
    let hrs = Hrs::new(i2c);

    // setup touchpad external interrupt pin: P0.28/AIN4 (TP_INT)
    let touch_int = Input::new(p.P0_28, Pull::Up);
    // setup touchpad reset pin: P0.10/NFC2 (TP_RESET)
    let touch_rst = Output::new(p.P0_10, Level::High, OutputDrive::Standard);

    let i2c = I2cDevice::new(i2c_bus);
    let mut touchpad = cst816s::CST816S::new(i2c, touch_int, touch_rst);
    touchpad.setup(&mut embassy_time::Delay).unwrap();

    // Button enable
    let _btn_enable = Output::new(p.P0_15, Level::High, OutputDrive::Standard);

    let btn = Button::new(Input::new(p.P0_13.degrade(), Pull::Down));

    let mut default_config = spim::Config::default();
    default_config.frequency = spim::Frequency::M8;
    default_config.mode = MODE_3;

    let spim = spim::Spim::new(p.TWISPI0, Irqs, p.P0_02, p.P0_04, p.P0_03, default_config);
    let spi_bus = SPI_BUS.init(BMutex::new(RefCell::new(spim)));

    // Create flash device
    let flash_cs = Output::new(p.P0_05, Level::High, OutputDrive::Standard);
    let flash_spi = SpiDevice::new(spi_bus, flash_cs);
    let xt_flash = XtFlash::new(flash_spi).unwrap();
    static EXTERNAL_FLASH: StaticCell<BMutex<NoopRawMutex, RefCell<ExternalFlash>>> = StaticCell::new();
    let external_flash = EXTERNAL_FLASH.init(BMutex::new(RefCell::new(xt_flash)));

    let internal_flash = nrf_softdevice::Flash::take(sd);
    static INTERNAL_FLASH: StaticCell<Mutex<NoopRawMutex, InternalFlash>> = StaticCell::new();
    let internal_flash = INTERNAL_FLASH.init(Mutex::new(internal_flash));

    // DFU setup
    let dfu_config = DfuConfig::new(internal_flash, external_flash);
    let mut magic = AlignedBuffer([0; 4]);
    let fw: FirmwareState<'_, _> = FirmwareState::new(dfu_config.state(), &mut magic.0);

    // Display
    s.spawn(advertiser_task(s, sd, server, dfu_config.clone(), "Watchful Embassy"))
        .unwrap();

    let backlight = Output::new(p.P0_22.degrade(), Level::Low, OutputDrive::Standard); // Medium backlight
    let rst = Output::new(p.P0_26, Level::Low, OutputDrive::Standard);
    let display_cs = Output::new(p.P0_25, Level::High, OutputDrive::Standard); // Keep low while driving display
    let display_spi = SpiDevice::new(spi_bus, display_cs);
    let dc = Output::new(p.P0_18, Level::Low, OutputDrive::Standard); // Data/clock
    let di = SPIDeviceInterface::new(display_spi, dc);
    // create the ILI9486 display driver from the display interface and optional RST pin
    let mut display = mipidsi::Builder::st7789(di)
        .with_display_size(240, 240)
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .init(&mut Delay, Some(rst))
        .unwrap();
    display.set_orientation(mipidsi::Orientation::Portrait(false)).unwrap();

    let screen = Screen::new(display, backlight);
    let mut device: Device<'_> = Device {
        clock: &CLOCK,
        screen,
        button: btn,
        battery,
        firmware: fw,
        touchpad,
        hrs,
    };

    let mut state = WatchState::default();
    state.draw(&mut device).await;
    loop {
        let mut next = state.next(&mut device).await;
        defmt::info!("{:?} -> {:?}", state, next);
        if next != state {
            next.draw(&mut device).await;
        }
        state = next;
    }
}

pub async fn gatt_server_task(conn: Connection, server: &'static ble::PineTimeServer, dfu_config: DfuConfig<'static>) {
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

    let mut conn_handle = ble::ConnectionHandle {
        connection: conn.clone(),
        notify_control: false,
        notify_packet: false,
    };

    info!("Running GATT server");
    let mut dfu = dfu_config.dfu();
    let mut target = DfuTarget::new(dfu.size(), fw_info, hw_info);
    let spawner = Spawner::for_current_executor().await;

    let _ = gatt_server::run(&conn, server, |e| {
        if let Some(DfuStatus::DoneReset) = server.handle(&mut target, &mut dfu, &mut conn_handle, e) {
            let _ = spawner.spawn(finish_dfu(dfu_config.clone()));
        }
    })
    .await;
    info!("Disconnected");
}

#[embassy_executor::task]
pub async fn finish_dfu(config: DfuConfig<'static>) {
    let mut magic = AlignedBuffer([0; 4]);
    let mut state = FirmwareState::new(config.state(), &mut magic.0);
    match state.mark_updated().await {
        Ok(_) => {
            info!("Firmware updated, resetting");
            cortex_m::peripheral::SCB::sys_reset();
        }
        Err(e) => {
            panic!("Error marking firmware updated: {:?}", e);
        }
    }
}

#[embassy_executor::task]
pub async fn advertiser_task(
    _spawner: Spawner,
    sd: &'static Softdevice,
    server: &'static ble::PineTimeServer,
    dfu_config: DfuConfig<'static>,
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
        Timer::after(Duration::from_secs(1)).await;
        info!("Syncing time");
        ble::sync_time(&conn, &CLOCK).await;

        gatt_server_task(conn, server, dfu_config.clone()).await;
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
            att_mtu: crate::ble::ATT_MTU as u16,
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
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
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

#[derive(Clone)]
pub struct DfuConfig<'a> {
    internal: &'a Mutex<NoopRawMutex, InternalFlash>,
    external: &'a BMutex<NoopRawMutex, RefCell<ExternalFlash>>,
    state_start: u32,
    state_end: u32,
    dfu_start: u32,
    dfu_end: u32,
}

impl<'a> DfuConfig<'a> {
    pub fn new(
        internal: &'a Mutex<NoopRawMutex, InternalFlash>,
        external: &'a BMutex<NoopRawMutex, RefCell<ExternalFlash>>,
    ) -> Self {
        extern "C" {
            static __bootloader_state_start: u32;
            static __bootloader_state_end: u32;
            static __bootloader_dfu_start: u32;
            static __bootloader_dfu_end: u32;
        }

        unsafe {
            let dfu_start = &__bootloader_dfu_start as *const u32 as u32;
            let dfu_end = &__bootloader_dfu_end as *const u32 as u32;

            BlockingPartition::new(external, dfu_start, dfu_end - dfu_start);

            let state_start = &__bootloader_state_start as *const u32 as u32;
            let state_end = &__bootloader_state_end as *const u32 as u32;

            Partition::new(internal, state_start, state_end - state_start);
            Self {
                internal,
                external,
                state_start,
                state_end,
                dfu_start,
                dfu_end,
            }
        }
    }

    pub fn state(&self) -> StatePartition<'a> {
        StatePartition::new(self.internal, self.state_start, self.state_end - self.state_start)
    }

    pub fn dfu(&self) -> DfuPartition<'a> {
        DfuPartition::new(self.external, self.dfu_start, self.dfu_end - self.dfu_start)
    }
}
