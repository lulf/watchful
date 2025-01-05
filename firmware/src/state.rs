use defmt::info;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::prelude::*;
use watchful_ui::{FirmwareDetails, MenuAction, MenuView, TimeView, WorkoutView};

use crate::device::Device;

const IDLE_TIMEOUT: Duration = Duration::from_secs(10);

#[derive(PartialEq, Clone, Copy)]
pub struct Timeout {
    start: Instant,
    duration: Duration,
}

impl Timeout {
    pub fn new(duration: Duration) -> Self {
        Self {
            start: Instant::now(),
            duration,
        }
    }
    pub fn timer(&self) -> Timer {
        Timer::after(self.time_left())
    }

    fn time_left(&self) -> Duration {
        let left = Instant::now() - self.start;
        if left < self.duration {
            self.duration - left
        } else {
            Duration::from_ticks(0)
        }
    }
}

#[derive(PartialEq)]
pub enum WatchState {
    Idle(IdleState),
    Time(TimeState),
    Menu(MenuState),
    //  FindPhone,
    Workout(WorkoutState),
}

impl Default for WatchState {
    fn default() -> Self {
        Self::Idle(IdleState)
    }
}

impl defmt::Format for WatchState {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Self::Idle(_) => defmt::write!(fmt, "Idle"),
            Self::Time(_) => defmt::write!(fmt, "Time"),
            Self::Menu(_) => defmt::write!(fmt, "Menu"),
            Self::Workout(_) => defmt::write!(fmt, "Workout"),
        }
    }
}

impl WatchState {
    pub async fn draw(&mut self, device: &mut Device<'_>) {
        match self {
            WatchState::Idle(state) => state.draw(device).await,
            WatchState::Time(state) => state.draw(device).await,
            WatchState::Menu(state) => state.draw(device).await,
            WatchState::Workout(state) => state.draw(device).await,
        }
    }

    pub async fn next(&mut self, device: &mut Device<'_>) -> WatchState {
        match self {
            WatchState::Idle(state) => state.next(device).await,
            WatchState::Time(state) => state.next(device).await,
            WatchState::Menu(state) => state.next(device).await,
            WatchState::Workout(state) => state.next(device).await,
        }
    }
}

#[derive(PartialEq)]
pub struct IdleState;
impl IdleState {
    pub fn new(_device: &mut Device<'_>) -> Self {
        Self
    }

    pub async fn draw(&mut self, device: &mut Device<'_>) {
        device.screen.off();
    }

    pub async fn next(&mut self, device: &mut Device<'_>) -> WatchState {
        device.button.wait().await;
        WatchState::Time(TimeState::new(device, Timeout::new(IDLE_TIMEOUT)).await)
    }
}

#[derive(PartialEq)]
pub struct TimeState {
    view: TimeView,
    timeout: Timeout,
}

impl TimeState {
    pub async fn new(device: &mut Device<'_>, timeout: Timeout) -> TimeState {
        let now = device.clock.get();
        let battery_level = device.battery.measure().await;
        let charging = device.battery.is_charging();
        Self {
            view: TimeView::new(now, battery_level, charging),
            timeout,
        }
    }

    pub async fn draw(&mut self, device: &mut Device<'_>) {
        self.view.draw(device.screen.display()).unwrap();
        device.screen.on();
    }

    pub async fn next(&mut self, device: &mut Device<'_>) -> WatchState {
        loop {
            match select3(
                Timer::after(Duration::from_secs(2)),
                self.timeout.timer(),
                device.button.wait(),
            )
            .await
            {
                Either3::First(_) => {
                    let t = device.clock.get();
                    let b = device.battery.measure().await;
                    let l = device.battery.is_charging();
                    if t.minute() != self.view.time.minute()
                        || b != self.view.battery_level
                        || l != self.view.battery_charging
                    {
                        return WatchState::Time(TimeState::new(device, self.timeout).await);
                    }
                }
                Either3::Second(_) => {
                    return WatchState::Idle(IdleState::new(device));
                }
                Either3::Third(_) => return WatchState::Menu(MenuState::new(MenuView::main())),
            }
        }
    }
}

#[derive(PartialEq)]
pub struct MenuState {
    view: MenuView,
    timeout: Timeout,
}

impl MenuState {
    pub fn new(view: MenuView) -> Self {
        let timeout = Timeout::new(IDLE_TIMEOUT);
        Self { view, timeout }
    }

    pub async fn draw(&mut self, device: &mut Device<'_>) {
        self.view.draw(device.screen.display()).unwrap();
        device.screen.on();
    }

    pub async fn next(&mut self, device: &mut Device<'_>) -> WatchState {
        match select3(self.timeout.timer(), device.button.wait(), async {
            let selected;
            loop {
                if let Some(evt) = device.touchpad.read_one_touch_event(true) {
                    if let cst816s::TouchGesture::SingleClick = evt.gesture {
                        let touched = Point::new(evt.x, evt.y);
                        if let Some(s) =
                            self.view
                                .on_event(watchful_ui::InputEvent::Touch(watchful_ui::TouchGesture::SingleTap(
                                    touched,
                                )))
                        {
                            selected = s;
                            break;
                        }
                    }
                } else {
                    Timer::after(Duration::from_micros(2)).await;
                }
            }
            selected
        })
        .await
        {
            Either3::First(_) => WatchState::Idle(IdleState::new(device)),
            Either3::Second(_) => {
                if let MenuView::Settings { .. } = &self.view {
                    WatchState::Menu(MenuState::new(MenuView::main()))
                } else if let MenuView::Firmware { .. } = &self.view {
                    WatchState::Menu(MenuState::new(MenuView::settings()))
                } else {
                    WatchState::Time(TimeState::new(device, Timeout::new(IDLE_TIMEOUT)).await)
                }
            }
            Either3::Third(selected) => match selected {
                MenuAction::Workout => {
                    defmt::info!("Not implemented");
                    WatchState::Workout(WorkoutState {})
                }
                MenuAction::FindPhone => {
                    defmt::info!("Not implemented");
                    WatchState::Time(TimeState::new(device, Timeout::new(IDLE_TIMEOUT)).await)
                }
                MenuAction::Settings => WatchState::Menu(MenuState::new(MenuView::settings())),
                MenuAction::Brightness => {
                    device.screen.change_brightness();
                    WatchState::Menu(MenuState::new(MenuView::settings()))
                },
                MenuAction::Reset => {
                    cortex_m::peripheral::SCB::sys_reset();
                }
                MenuAction::FirmwareSettings => {
                    /*
                    let validated = FwState::Boot
                        == device
                            .firmware
                            .get_state()
                            .await
                            .expect("Failed to read firmware state");
                    */
                    let validated = false;
                    WatchState::Menu(MenuState::new(MenuView::firmware_settings(
                        firmware_details(&mut device.battery, validated).await,
                    )))
                }
                MenuAction::ValidateFirmware => {
                    info!("Validate firmware");
                    WatchState::Time(TimeState::new(device, Timeout::new(IDLE_TIMEOUT)).await)
                    /*
                    let validated = FwState::Boot
                        == device
                            .firmware
                            .get_state()
                            .await
                            .expect("Failed to read firmware state");
                    if !validated {
                        device
                            .firmware
                            .mark_booted()
                            .await
                            .expect("Failed to mark current firmware as good");
                        info!("Firmware marked as valid");
                        WatchState::Menu(MenuState::new(MenuView::main()))
                    } else {
                        WatchState::Menu(MenuState::new(MenuView::firmware_settings(
                            firmware_details(&mut device.battery, validated).await,
                        )))
                    }*/
                }
            },
        }
    }
}

#[derive(PartialEq)]
pub struct WorkoutState {}

impl WorkoutState {
    pub async fn draw(&mut self, _device: &mut Device<'_>) {}
    pub async fn next(&mut self, device: &mut Device<'_>) -> WatchState {
        let screen = &mut device.screen;
        let button = &mut device.button;
        let hrs = &mut device.hrs;
        hrs.init().unwrap();
        hrs.enable_hrs().unwrap();
        hrs.enable_oscillator().unwrap();

        let mut seconds = 0;
        let workout = async {
            loop {
                let hr = hrs.read_hrs().unwrap();
                WorkoutView::new(hr, time::Duration::new(seconds, 0))
                    .draw(screen.display())
                    .unwrap();
                screen.on();
                Timer::after(Duration::from_secs(2)).await;
                seconds += 2;
            }
        };

        let next = match select(button.wait(), workout).await {
            Either::First(_) => WatchState::Menu(MenuState::new(MenuView::main())),
            Either::Second(state) => state,
        };
        hrs.disable_oscillator().unwrap();
        hrs.disable_hrs().unwrap();
        next
    }
}

async fn firmware_details(battery: &mut crate::device::Battery<'_>, validated: bool) -> FirmwareDetails {
    const CARGO_NAME: &str = env!("CARGO_PKG_NAME");
    const CARGO_VERSION: &str = env!("CARGO_PKG_VERSION");
    const COMMIT: &str = env!("VERGEN_GIT_SHA");
    const BUILD_TIMESTAMP: &str = env!("VERGEN_BUILD_TIMESTAMP");

    let battery_level = battery.measure().await;
    let battery_charging = battery.is_charging();

    FirmwareDetails::new(
        CARGO_NAME,
        CARGO_VERSION,
        COMMIT,
        BUILD_TIMESTAMP,
        battery_level,
        battery_charging,
        validated,
    )
}
