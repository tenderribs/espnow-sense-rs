#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output},
    i2c::master::{Config, I2c},
    prelude::*,
    rng::Rng,
    rtc_cntl::{sleep::TimerWakeupSource, Rtc},
    timer::timg::TimerGroup,
};
use esp_wifi::esp_now::BROADCAST_ADDRESS;
use sensirion_rht::{Addr, Device, Repeatability};

use espnow_sense_rs::{set_rtc_time, DeepSleep};

// default interval between measurements
const SLEEP_DURATION_S: u64 = 5;

// extended deep sleep config during overnight stop
const UTC_DIFF: f32 = 1.0; // timezone diff to UTC. in CH should be +1 or +2, in AU should be +9.5 or +10.5
const BEDTIME_HR: f32 = 22.0; // when it starts. ex. 21.5 is 09:30 PM, is timezone aware
const WAKEUP_HR: f32 = 6.0; // when to wake up from deep sleep, is timezone aware

// validate specified configuration
const _: () = {
    assert!(
        BEDTIME_HR > 12.0 && BEDTIME_HR < 24.0,
        "Bedtime should be in the PM, local time."
    );
    assert!(
        WAKEUP_HR > 0.0 && WAKEUP_HR <= 12.0,
        "Waketime has to be in the AM, local time."
    );
    assert!(UTC_DIFF > 0.0, "CH and AU have positive UTC offsets.");
    assert!(UTC_DIFF < 11.0, "UTC offset is too high.");
};

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    // init esp32
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();

        config
    });
    esp_alloc::heap_allocator!(72 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let delay = Delay::new();
    let mut led = Output::new(peripherals.GPIO15, Level::High);
    let mut tsensor_pwr = Output::new(peripherals.GPIO19, Level::High);

    // prepare peripherals
    let init = esp_wifi::init(
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    // init RTC
    let rtc = Rtc::new(peripherals.LPWR);
    set_rtc_time(&rtc);

    // init esp-now
    let wifi = peripherals.WIFI;
    let mut esp_now = esp_wifi::esp_now::EspNow::new(&init, wifi).unwrap();

    // init temp sensor
    // https://sensirion.com/media/documents/1DA31AFD/65D613A8/Datasheet_STS3x_DIS.pdf
    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .with_sda(peripherals.GPIO0)
        .with_scl(peripherals.GPIO1);
    let mut tsensor = Device::new_sts3x(Addr::A, i2c, delay);

    // read temperature value in single shot mode
    if let Ok(temperature) = tsensor.single_shot(Repeatability::High) {
        // broadcast the message
        let _ = esp_now
            .send(&BROADCAST_ADDRESS, &temperature.as_celsius().to_le_bytes())
            .unwrap()
            .wait();
    }

    // power down peripherals
    led.set_low();
    tsensor_pwr.set_low();

    enter_deep_sleep(rtc);
    panic!("Entering deep sleep failed");
}

fn enter_deep_sleep(mut rtc: Rtc) {
    let ds = DeepSleep::new(BEDTIME_HR, WAKEUP_HR, UTC_DIFF, SLEEP_DURATION_S);
    let duration = ds.sleep_duration(rtc.current_time().time());

    let wake_src =
        TimerWakeupSource::new(core::time::Duration::from_secs(duration));
    rtc.sleep_deep(&[&wake_src]);
}
