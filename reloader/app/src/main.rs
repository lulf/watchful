#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_boot::State as FwState;
use embassy_boot_nrf::{AlignedBuffer, BlockingFirmwareState as FirmwareState, FirmwareUpdaterConfig};
use embassy_embedded_hal::flash::partition::{self, BlockingPartition};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::peripherals::{P0_05, P0_18, P0_25, P0_26, TWISPI0};
use embassy_nrf::spim::Spim;
use embassy_nrf::spis::MODE_3;
use embassy_nrf::{bind_interrupts, pac, peripherals, spim, twim};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::{Mutex, Mutex as BMutex};
use embassy_time::{Delay, Duration, Timer};
use embedded_storage::nor_flash::NorFlash;
use heapless::Vec;
use pinetime_flash::XtFlash;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => spim::InterruptHandler<peripherals::TWISPI0>;

});

type Flash = XtFlash<SpiDevice<'static, NoopRawMutex, Spim<'static, TWISPI0>, Output<'static, P0_05>>>;

static SOFTDEVICE: &[u8] = include_bytes!("../softdevice.bin");
static BOOTLOADER: &[u8] = include_bytes!("../bootloader.bin");
static APPLICATION: &[u8] = include_bytes!("../application.bin");

const BOOTLOADER_DEST: u32 = 0x00077000;
const APPLICATION_DEST: u32 = 0x00000000; // External flash
const SOFTDEVICE_DEST: u32 = 0x00000000;
const BOOTLOADER_STATE: u32 = 0x003FF000;
const UICR_ADDRESS: u32 = 0x10001014;

/*#[link_section = ".uicr_bootloader_start_address"]
#[no_mangle]
pub static UICR_BOOTLOADER_START_ADDRESS: u32 = BOOTLOADER_DEST;*/

#[embassy_executor::main]
async fn main(s: Spawner) {
    let p = embassy_nrf::init(Default::default());

    defmt::info!("Hello! Starting reloader");
    Timer::after(Duration::from_secs(5)).await;

    let mut default_config = spim::Config::default();
    default_config.frequency = spim::Frequency::M8;
    default_config.mode = MODE_3;

    let spim = spim::Spim::new(p.TWISPI0, Irqs, p.P0_02, p.P0_04, p.P0_03, default_config);
    let spi_bus: Mutex<NoopRawMutex, RefCell<_>> = Mutex::new(RefCell::new(spim));

    // Create flash device
    let flash_cs = Output::new(p.P0_05, Level::High, OutputDrive::Standard);
    let flash_spi = SpiDevice::new(&spi_bus, flash_cs);
    let mut external_flash = XtFlash::new(flash_spi).unwrap();
    external_flash.erase(APPLICATION_DEST, 328 * 1024).unwrap();
    external_flash.write(APPLICATION_DEST, APPLICATION).unwrap();

    defmt::info!("Flashed application");
    let mut internal_flash = Nvmc::new(p.NVMC);

    internal_flash.erase(0, 0x26000).unwrap();
    defmt::info!("Erased softdevice section");

    internal_flash.write(0, SOFTDEVICE).unwrap();

    defmt::info!("Flashed softdevice, erasing bootloader at {:x}", BOOTLOADER_DEST);

    internal_flash.erase(BOOTLOADER_DEST, BOOTLOADER_DEST + 32768).unwrap();

    defmt::info!("Erased bootloader, flashing it (size {})", BOOTLOADER.len());

    internal_flash.write(BOOTLOADER_DEST, BOOTLOADER).unwrap();

    defmt::info!("Flashed bootloader");

    //    internal_flash.write(UICR_ADDRESS, BOOTLOADER_DEST).unwrap();
    //   unsafe { (*pac::UICR::ptr()).nrffw[0].write(|f| f.bits(BOOTLOADER_DEST)) };

    // defmt::info!("Wrote UICR");

    let mtx = Mutex::new(RefCell::new(external_flash));
    let part: BlockingPartition<NoopRawMutex, _> = BlockingPartition::new(&mtx, BOOTLOADER_STATE, 4096);
    let mut aligned = [0; 1];
    let mut state = FirmwareState::new(part, &mut aligned);
    match state.mark_updated() {
        Ok(_) => {
            defmt::info!("Marked as updated, resetting in 5 seconds!");
            Timer::after(Duration::from_secs(5)).await;
            cortex_m::peripheral::SCB::sys_reset();
        }
        Err(e) => {
            defmt::info!("Error marking firmware as updated: {:?}", defmt::Debug2Format(&e));

            loop {
                Timer::after(Duration::from_secs(1)).await
            }
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf();
}
