use core::cell::RefCell;
use core::ops::Add;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Ticker};

pub struct Clock {
    time: Mutex<ThreadModeRawMutex, RefCell<time::PrimitiveDateTime>>,
}

impl Clock {
    pub const fn new() -> Self {
        Self {
            time: Mutex::new(RefCell::new(time::PrimitiveDateTime::MIN)),
        }
    }

    pub fn set(&self, time: time::PrimitiveDateTime) {
        self.time.lock(|f| *f.borrow_mut() = time)
    }

    pub fn get(&self) -> time::PrimitiveDateTime {
        self.time.lock(|f| f.borrow().clone())
    }

    fn add(&self, duration: time::Duration) {
        self.time.lock(|f| {
            let mut val = f.borrow_mut();
            *val = val.add(duration);
        })
    }
}

#[embassy_executor::task]
pub async fn clock(clock: &'static Clock) {
    const TICK: Duration = Duration::from_secs(1);
    let mut ticker = Ticker::every(TICK);
    loop {
        ticker.next().await;
        clock.add(time::Duration::seconds(1));
    }
}
