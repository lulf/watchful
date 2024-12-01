#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_nrf::interrupt;
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::nvmc::Nvmc;
use embassy_time::{Duration, Timer};

#[interrupt]
unsafe fn SWI0_EGU0() {
    EXECUTOR_MED.on_interrupt()
}

static MCUBOOT: &[u8] = include_bytes!("../mcuboot.bin");
static RECOVERY: &[u8] = include_bytes!("../recovery.bin");

static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();

#[repr(align(4))]
struct AlignedBuffer([u8; 4096]);

#[embassy_executor::main]
async fn main(_s: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // Medium-priority executor: SWI0_EGU0, priority level 7
    interrupt::SWI0_EGU0.set_priority(Priority::P6);
    let spawner = EXECUTOR_MED.start(interrupt::SWI0_EGU0);
    spawner.spawn(watchdog_task()).unwrap();

    Timer::after(Duration::from_secs(1)).await;

    let mut buf = AlignedBuffer([0; 4096]);
    let mut internal_flash = Nvmc::new(p.NVMC);
    watchful_infinitime_recovery::recover(
        &mut internal_flash,
        &mut embassy_time::Delay,
        &mut buf.0,
        MCUBOOT,
        RECOVERY,
    )
    .await;
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
