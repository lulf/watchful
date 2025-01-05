#![cfg_attr(not(test), no_std)]
#![feature(impl_trait_in_assoc_type)]
#![no_main]

use core::cell::RefCell;

use defmt::unwrap;
use defmt_rtt as _;
use device::Backlight;
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::flash::partition::Partition;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::peripherals::{RNG, TWISPI0, TWISPI1};
use embassy_nrf::spim::Spim;
use embassy_nrf::spis::MODE_3;
use embassy_nrf::twim::Twim;
use embassy_nrf::{bind_interrupts, peripherals, rng, saadc, spim, twim};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex as BMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use mipidsi::options::Orientation;
use nrf_sdc::{self as sdc, mpsl};
#[cfg(feature = "panic-probe")]
use panic_probe as _;
use pinetime_flash::XtFlash;
use static_cell::StaticCell;

use crate::firmware_validator::FirmwareValidator;

mod ble;
mod clock;
mod device;
mod firmware_validator;
mod state;
use crate::clock::clock;
use crate::device::{Battery, Button, Device, Hrs, Screen};
use crate::state::WatchState;

bind_interrupts!(struct Irqs {
    TWISPI0 => spim::InterruptHandler<peripherals::TWISPI0>;
    TWISPI1 => twim::InterruptHandler<peripherals::TWISPI1>;
    SAADC => saadc::InterruptHandler;
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO => mpsl::HighPrioInterruptHandler;
    TIMER0 => mpsl::HighPrioInterruptHandler;
    RTC0 => mpsl::HighPrioInterruptHandler;
});

static CLOCK: clock::Clock = clock::Clock::new();

type ExternalFlash = XtFlash<SpiDevice<'static, NoopRawMutex, Spim<'static, TWISPI0>, Output<'static>>>;

type InternalFlash = mpsl::Flash<'static>;
type DfuPartition<'a> = Partition<'a, NoopRawMutex, ExternalFlash>;

static I2C_BUS: StaticCell<BMutex<NoopRawMutex, RefCell<Twim<'static, TWISPI1>>>> = StaticCell::new();
static SPI_BUS: StaticCell<BMutex<NoopRawMutex, RefCell<Spim<'static, TWISPI0>>>> = StaticCell::new();

use core::panic::PanicInfo;

#[cfg(not(feature = "panic-probe"))]
#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static mpsl::MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

fn build_sdc<'d, const N: usize>(
    p: sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<RNG>,
    mpsl: &'d mpsl::MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<sdc::SoftdeviceController<'d>, sdc::Error> {
    sdc::Builder::new()?
        .support_adv()?
        .support_peripheral()?
        .peripheral_count(1)?
        .buffer_cfg(
            ble::L2CAP_MTU as u8,
            ble::L2CAP_MTU as u8,
            ble::L2CAP_TXQ,
            ble::L2CAP_RXQ,
        )?
        .build(p, rng, mpsl, mem)
}

#[embassy_executor::main]
async fn main(s: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    let mpsl_p = mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<mpsl::MultiprotocolServiceLayer> = StaticCell::new();
    static SESSION_MEM: StaticCell<mpsl::SessionMem<1>> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::with_timeslots(
        mpsl_p,
        Irqs,
        lfclk_cfg,
        SESSION_MEM.init(mpsl::SessionMem::new())
    )));
    s.must_spawn(mpsl_task(&*mpsl));
    s.must_spawn(watchdog_task());

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24, p.PPI_CH25, p.PPI_CH26,
        p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let rng = rng::Rng::new(p.RNG, Irqs);

    static SDC_MEM: StaticCell<sdc::Mem<4096>> = StaticCell::new();
    let sdc_mem = SDC_MEM.init(sdc::Mem::new());

    static RNG: StaticCell<rng::Rng<'static, RNG>> = StaticCell::new();
    let rng = RNG.init(rng);

    let sdc = unwrap!(build_sdc(sdc_p, rng, mpsl, sdc_mem));

    s.must_spawn(clock(&CLOCK));

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
    static EXTERNAL_FLASH: StaticCell<Mutex<NoopRawMutex, ExternalFlash>> = StaticCell::new();
    let external_flash = EXTERNAL_FLASH.init(Mutex::new(xt_flash));

    let internal_flash = mpsl::Flash::take(mpsl, p.NVMC);
    static INTERNAL_FLASH: StaticCell<Mutex<NoopRawMutex, InternalFlash>> = StaticCell::new();
    let internal_flash = INTERNAL_FLASH.init(Mutex::new(internal_flash));

    // DFU setup
    let dfu_config = DfuConfig::new(internal_flash, external_flash);
    let firmware_validator = FirmwareValidator::new(internal_flash);

    // BLE
    ble::start(s, sdc, dfu_config);

    // Display
    let backlight = Backlight::new(p.P0_14.degrade(), p.P0_22.degrade(), p.P0_23.degrade());
    let rst = Output::new(p.P0_26, Level::Low, OutputDrive::Standard);
    let display_cs = Output::new(p.P0_25, Level::High, OutputDrive::Standard); // Keep low while driving display
    let display_spi = SpiDevice::new(spi_bus, display_cs);
    let dc = Output::new(p.P0_18, Level::Low, OutputDrive::Standard); // Data/clock
    let di = SPIInterface::new(display_spi, dc);
    let mut display = mipidsi::Builder::new(mipidsi::models::ST7789, di)
        .display_size(240, 240)
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .reset_pin(rst)
        .init(&mut Delay)
        .unwrap();
    display.set_orientation(Orientation::new()).unwrap();

    let screen = Screen::new(display, backlight);
    let mut device: Device<'_> = Device {
        clock: &CLOCK,
        screen,
        button: btn,
        battery,
        touchpad,
        hrs,
        firmware_validator,
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
    #[allow(dead_code)]
    internal: &'a Mutex<NoopRawMutex, InternalFlash>,
    external: &'a Mutex<NoopRawMutex, ExternalFlash>,
    dfu_start: u32,
    dfu_end: u32,
}

impl<'a> DfuConfig<'a> {
    pub fn new(
        internal: &'a Mutex<NoopRawMutex, InternalFlash>,
        external: &'a Mutex<NoopRawMutex, ExternalFlash>,
    ) -> Self {
        let dfu_start = 0x40000;
        let dfu_end = 0xB4000;

        Partition::new(external, dfu_start, dfu_end - dfu_start);
        Self {
            internal,
            external,
            dfu_start,
            dfu_end,
        }
    }

    pub fn dfu(&self) -> DfuPartition<'a> {
        DfuPartition::new(self.external, self.dfu_start, self.dfu_end - self.dfu_start)
    }
}
