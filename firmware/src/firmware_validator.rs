use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;

use crate::InternalFlash;

const VALID_BIT_ADDRESS: u32 = 0x7BFE8;
const VALID_BIT_VALUE: u32 = 1;

pub struct FirmwareValidator<'d> {
    flash: &'d Mutex<NoopRawMutex, InternalFlash>,
}

impl<'d> FirmwareValidator<'d> {
    pub fn new(flash: &'d Mutex<NoopRawMutex, InternalFlash>) -> Self {
        Self { flash }
    }

    pub fn is_valid(&self) -> bool {
        let value_ptr = VALID_BIT_ADDRESS as *const u32;
        let value = unsafe { value_ptr.read_volatile() };
        value == VALID_BIT_VALUE
    }

    pub async fn validate(&self) {
        if !self.is_valid() {
            let mut flash = self.flash.lock().await;
            if let Err(_e) = flash.write(VALID_BIT_ADDRESS, &VALID_BIT_VALUE.to_le_bytes()).await {
                defmt::warn!("Error validating firmware");
            }
        }
    }
}
