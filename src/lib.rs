// make std available during testing
#![no_std]

use core::time::Duration;

use chrono::{NaiveDateTime, NaiveTime, TimeDelta, Timelike};
use esp_hal::rtc_cntl::Rtc;
use esp_println::println;

pub fn set_rtc_time(rtc: &Rtc) -> () {
    // reads the BUILD_TIME env var at compile time passed in build.rs
    let flash_time = env!("BUILD_TIME");
    let fmt = "%Y-%m-%dT%H:%M:%SZ";

    println!("BUILD_TIME={}", flash_time);

    let res = NaiveDateTime::parse_from_str(flash_time, fmt);

    if let Ok(dt) = res {
        rtc.set_current_time(dt);
    } else {
        println!("Couldn't parse BUILD_TIME in fmt {}", fmt);
    }
}

pub struct DeepSleep {
    // RTC runs in UTC, we set target times in local time and specify the offet
    bedtime: NaiveTime,
    waketime: NaiveTime,
    utc_diff: f32,

    default_sleep_s: u64,
}

impl DeepSleep {
    pub fn new(
        bedtime_hr: f32,
        wake_hr: f32,
        utc_diff: f32,
        default_sleep_s: u64,
    ) -> Self {
        Self {
            bedtime: NaiveTime::from_num_seconds_from_midnight_opt(
                (bedtime_hr * 3600.0) as u32,
                0,
            )
            .expect("hours range is checked"),
            waketime: NaiveTime::from_num_seconds_from_midnight_opt(
                (wake_hr * 3600.0) as u32,
                0,
            )
            .expect("hours range is checked"),
            utc_diff,
            default_sleep_s,
        }
    }

    pub fn sleep_duration(&self, utc_now: NaiveTime) -> u64 {
        let utc_offset = TimeDelta::from_std(Duration::from_secs(
            (self.utc_diff * 3600.0) as u64,
        ));

        if let Ok(utc_delta) = utc_offset {
            // convert the time from UTC to local
            let now = utc_now + utc_delta;

            // enter sleep if now it's past bedtime
            if now > self.bedtime {
                let midnight_delta: TimeDelta =
                    NaiveTime::from_hms_opt(23, 59, 59).unwrap() - now;

                return midnight_delta.num_seconds().unsigned_abs()
                    + (self.waketime.num_seconds_from_midnight() as u64);
            }

            // or if now it's in the early hours of the morning
            if now < self.waketime {
                let wake_delta = self.waketime - now;
                return wake_delta.num_seconds().unsigned_abs();
            }
        }

        self.default_sleep_s
    }
}
