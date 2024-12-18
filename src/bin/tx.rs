#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Level, Output},
    prelude::*,
    rng::Rng,
    rtc_cntl::{sleep::TimerWakeupSource, Rtc},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_wifi::esp_now::BROADCAST_ADDRESS;

// Log every 5 mins
const SLEEP_DURATION_S: u64 = 5 * 60;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();

        config
    });

    let mut led = Output::new(peripherals.GPIO15, Level::High);

    esp_alloc::heap_allocator!(72 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let init = esp_wifi::init(
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    // prepare peripherals
    let wifi = peripherals.WIFI;
    let mut esp_now = esp_wifi::esp_now::EspNow::new(&init, wifi).unwrap();
    let mut rtc = Rtc::new(peripherals.LPWR);

    // get the RTC time as a value to send onwards.
    let now: u64 = rtc
        .current_time()
        .and_utc()
        .timestamp_micros()
        .try_into()
        .expect("current_time is negative");

    // broadcast the message
    let status = esp_now
        .send(&BROADCAST_ADDRESS, &now.to_le_bytes())
        .unwrap()
        .wait();
    println!("Send broadcast status: {:?}", status);

    // enter deep sleep;
    let wake_src = TimerWakeupSource::new(core::time::Duration::from_secs(SLEEP_DURATION_S));
    led.set_low();
    rtc.sleep_deep(&[&wake_src]);
}
