#![no_std]

use chrono::NaiveDateTime;
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
