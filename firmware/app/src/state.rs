use defmt::info;
use embassy_boot::State as FwState;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_time::{Duration, Instant, Timer};
use embassy_time::Ticker;
//use embedded_graphics::{framebuffer,prelude::*};
use embedded_graphics::prelude::*;
use watchful_ui::{FirmwareDetails, MenuAction, MenuView, TimeView, WorkoutView, TimerView};

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
    Timer(TimerState),
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
            Self::Timer(_) => defmt::write!(fmt, "Timer"),
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
            WatchState::Timer(state) => state.draw(device).await,
        }
    }

    pub async fn next(&mut self, device: &mut Device<'_>) -> WatchState {
        match self {
            WatchState::Idle(state) => state.next(device).await,
            WatchState::Time(state) => state.next(device).await,
            WatchState::Menu(state) => state.next(device).await,
            WatchState::Workout(state) => state.next(device).await,
            WatchState::Timer(state) => state.next(device).await,
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
    	let mut ticker = Ticker::every(Duration::from_secs(2));
        loop {
            match select3(
                //Timer::after(Duration::from_secs(2)),
                ticker.next(),
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
                } else if let MenuView::Timer { .. } = &self.view {
                    WatchState::Menu(MenuState::new(MenuView::timers()))                    
                } else if let MenuView::Timers { .. } = &self.view {
                    WatchState::Menu(MenuState::new(MenuView::timers()))
                } else {
                    WatchState::Time(TimeState::new(device, Timeout::new(IDLE_TIMEOUT)).await)
                }
            }
            Either3::Third(selected) => match selected {
            	MenuAction::Timers => WatchState::Menu(MenuState::new(MenuView::timers())),
				MenuAction::TimerOne => {
                    //WatchState::Timer(TimerState { duration: 180, running: false })
                    WatchState::Menu(MenuState::new(MenuView::timer_view(
                    		TimerView::new(time::Duration::seconds(180), false))))
                }
				MenuAction::TimerTwo => {
                    WatchState::Menu(MenuState::new(MenuView::timer_view(
                    		TimerView::new(time::Duration::seconds(120), false))))
                }
				MenuAction::TimerThree => {
                    WatchState::Menu(MenuState::new(MenuView::timer_view(
                    		TimerView::new(time::Duration::seconds(60), false))))
                }
				MenuAction::TimerFour => {
                    WatchState::Menu(MenuState::new(MenuView::timer_view(
                    		TimerView::new(time::Duration::seconds(30), false))))
                }
				MenuAction::ToggleTimer(rem, run) => {
                    WatchState::Timer(TimerState { duration: rem, running: run })
                    //WatchState::Menu(MenuState::new(MenuView::timer_view(
                    	//TimerView::new(time::Duration::seconds(rem), run))))
                }                               
                MenuAction::Workout => {
                    defmt::info!("Not implemented");
                    WatchState::Workout(WorkoutState {})
                }
                MenuAction::FindPhone => {
                    defmt::info!("Not implemented");
                    WatchState::Time(TimeState::new(device, Timeout::new(IDLE_TIMEOUT)).await)
                }
                MenuAction::Settings => WatchState::Menu(MenuState::new(MenuView::settings())),
                MenuAction::Reset => {
                    cortex_m::peripheral::SCB::sys_reset();
                }
                MenuAction::FirmwareSettings => {
                    let validated = FwState::Boot
                        == device
                            .firmware
                            .get_state()
                            .await
                            .expect("Failed to read firmware state");
                    WatchState::Menu(MenuState::new(MenuView::firmware_settings(
                        firmware_details(&mut device.battery, validated).await,
                    )))
                }
                MenuAction::ValidateFirmware => {
                    info!("Validate firmware");
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
                    }
                }
            },
        }
    }
}

#[derive(PartialEq)]
pub struct TimerState {
	duration: i64,
	running: bool
}

impl TimerState {
	//i added embedded_graphics_framebuf
	//and did my graphics write calls to that, then that gets
    pub async fn draw(&mut self, _device: &mut Device<'_>) {}/*
	pub fn on_event(&self, input: InputEvent) {
 		match self {
			if tog.is_clicked(input) {
      			self.running = !self.ruinning;
	       } else {
				None
			}
		}
	}    */
    pub async fn next(&mut self, device: &mut Device<'_>) -> WatchState {
        let screen = &mut device.screen;
        let button = &mut device.button;
    	let mut ticker = Ticker::every(Duration::from_secs(1));
		let mut tv = TimerView::new(time::Duration::seconds(self.duration), self.running);
  		let v = &mut device.vibrator;

		
        let timer = async {
            loop {
            	tv.remaining = time::Duration::seconds(self.duration);
            	tv.draw(screen.display()).unwrap();
            	screen.on();
                if tv.running {
                	self.duration -= 1;
				}
					
                if self.duration <= 1 {
                    break;
                }
                ticker.next().await;
                self.running = tv.running;
            }
            
            tv.remaining = time::Duration::ZERO;
            	tv.draw(screen.display()).unwrap();
            	screen.on();
            	v.on_for(1500).await;
            //loop {}
        };
    	
      	/*
        let timer = async {
            loop {
                TimerView::new(time::Duration::seconds(secs))
                    .draw(screen.display())
                    .unwrap();
                screen.on();
                secs -= 1;

                if secs <= 0 {
                    break;
                }
                ticker.next().await;
            }
        };
      	
      	let mut secs = 60.0;
      	let mut ticker = Ticker::every(Duration::from_millis(100));
        let timer = async {
            loop {
                TimerView::new(time::Duration::checked_seconds_f32(secs)
                	.expect("wrong seconds"))
                    .draw(screen.display())
                    .unwrap();
                screen.on();
                secs -= 0.1;

                if secs <= 0.0 {
                    break;
                }
                ticker.next().await;
            }
        };
        */
        
        let next = match select(button.wait(), timer).await {
            Either::First(_) => WatchState::Menu(MenuState::new(MenuView::main())),
            // this state is not triggered
            Either::Second(_) => WatchState::Menu(MenuState::new(MenuView::timers())),
        };
        next
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
        let mut ticker = Ticker::every(Duration::from_secs(2));
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
                ticker.next().await;
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
