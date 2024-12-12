#![no_std]
#![no_main]

use cortex_m_rt::{entry, exception};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use {defmt_rtt as _, embassy_nrf as _, panic_probe as _};

#[entry]
fn main() -> ! {
    let p = embassy_nrf::init(Default::default());
    let mut led = Output::new(p.P0_22, Level::Low, OutputDrive::Standard);
    led.set_low();
    unsafe {
        extern "C" {
            static __reloader_start: u32;
        }
        let start = &__reloader_start as *const u32 as u32;
        defmt::info!("Loading at {:08x}", start);
        let mut p = cortex_m::Peripherals::steal();
        p.SCB.invalidate_icache();
        p.SCB.vtor.write(start);

        cortex_m::asm::bootload(start as *const u32)
    }
}

#[no_mangle]
#[cfg_attr(target_os = "none", link_section = ".HardFault.user")]
unsafe extern "C" fn HardFault() {
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    const SCB_ICSR: *const u32 = 0xE000_ED04 as *const u32;
    let irqn = core::ptr::read_volatile(SCB_ICSR) as u8 as i16 - 16;

    panic!("DefaultHandler #{:?}", irqn);
}
