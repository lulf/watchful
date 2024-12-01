#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use core::cell::RefCell;

use embassy_embedded_hal::flash::partition::BlockingPartition;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::spis::MODE_3;
use embassy_nrf::{bind_interrupts, interrupt, pac, peripherals, spim};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_storage::nor_flash::NorFlash;

static MCUBOOT: &[u8] = include_bytes!("../mcuboot.bin");
static RECOVERY: &[u8] = include_bytes!("../recovery.bin");

const MCUBOOT_DEST: u32 = 0x00000000;
const RECOVERY_DEST: u32 = 0x00008000;

#[interrupt]
unsafe fn SWI0_EGU0() {
    EXECUTOR_MED.on_interrupt()
}

static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();

#[repr(align(4))]
struct AlignedBuffer([u8; 4096]);

#[embassy_executor::main]
async fn main(s: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // Medium-priority executor: SWI0_EGU0, priority level 7
    interrupt::SWI0_EGU0.set_priority(Priority::P6);
    let spawner = EXECUTOR_MED.start(interrupt::SWI0_EGU0);
    spawner.spawn(watchdog_task()).unwrap();

    let mut led = Output::new(p.P0_22, Level::Low, OutputDrive::Standard);

    Timer::after(Duration::from_secs(1)).await;

    let mut buf = AlignedBuffer([0; 4096]);
    let mut internal_flash = Nvmc::new(p.NVMC);

    internal_flash.erase(0, 0x8000).unwrap();
    Timer::after(Duration::from_secs(1)).await;

    let magic: &[u8] = &[
        0xf3, 0x95, 0xc2, 0x77, 0x7f, 0xef, 0xd2, 0x60, 0x0f, 0x50, 0x52, 0x35, 0x80, 0x79, 0xb6, 0x2c,
    ];
    let mut pos = MCUBOOT_DEST;
    for chunk in MCUBOOT.chunks(4096) {
        buf.0[..chunk.len()].copy_from_slice(chunk);
        if chunk.len() < buf.0.len() {
            for mut slice in buf.0[chunk.len()..].chunks_mut(magic.len()) {
                let to_copy = slice.len();
                slice[0..to_copy].copy_from_slice(&magic[0..to_copy]);
            }
        }

        internal_flash.write(pos, &buf.0[..]).unwrap();
        pos += chunk.len() as u32;
        Timer::after(Duration::from_millis(200)).await;
    }

    for page in (0x8000..0x39000).step_by(4096) {
        internal_flash.erase(page, 4096).unwrap();
        Timer::after(Duration::from_millis(200)).await;
    }

    let mut pos = RECOVERY_DEST;
    for chunk in RECOVERY.chunks(4096) {
        buf.0[..chunk.len()].copy_from_slice(chunk);
        if chunk.len() < buf.0.len() {
            for mut slice in buf.0[chunk.len()..].chunks_mut(magic.len()) {
                let to_copy = slice.len();
                slice[0..to_copy].copy_from_slice(&magic[0..to_copy]);
            }
        }

        internal_flash.write(pos, &buf.0[..]).unwrap();
        pos += chunk.len() as u32;
        Timer::after(Duration::from_millis(200)).await;
    }
    Timer::after(Duration::from_secs(5)).await;
    cortex_m::peripheral::SCB::sys_reset();
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf();
}

// Keeps our system alive
#[embassy_executor::task]
async fn watchdog_task() {
    let mut handle = unsafe { embassy_nrf::wdt::WatchdogHandle::steal(0) };
    loop {
        handle.pet();
        Timer::after(Duration::from_secs(2)).await;
    }
}
