// make std available during testing
#![no_std]

use chrono::{NaiveDateTime, NaiveTime, TimeDelta, Timelike};
use core::time::Duration;
use ds323x::{DateTimeAccess, Ds323x};
use embedded_storage::{ReadStorage, Storage};
use esp_hal::{i2c::master::I2c, rom::md5, rtc_cntl::Rtc, Blocking};
use esp_println::println;
use esp_storage::FlashStorage;

const NVS_ADDR: u32 = 0x9000; // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html
const HASH_OFFSET: u32 = 0x0;
const HASH_ADDR: u32 = NVS_ADDR + HASH_OFFSET;

fn build_datetime() -> NaiveDateTime {
    // reads the BUILD_TIME env var at compile time passed in build.rs
    let flash_time = env!("BUILD_TIME");
    let fmt = "%Y-%m-%dT%H:%M:%SZ";

    println!("BUILD_TIME={}", flash_time);

    NaiveDateTime::parse_from_str(flash_time, fmt)
        .expect("Couldn't parse BUILD_TIME")
}

fn is_first_boot(now_dt: &NaiveDateTime) -> bool {
    // Get MD5 has of build time
    let timestamp = now_dt.and_utc().to_rfc3339(); // nanosecs are zero given by parse_from_str
    let build_hash: [u8; 16] = md5::compute(&timestamp[..]).into();

    // read the cached hash data
    let mut flash = FlashStorage::new();
    let mut cached_hash = [0u8; 16];
    flash
        .read(HASH_ADDR, &mut cached_hash)
        .expect("Cannot read from flash memory.");

    if cached_hash != build_hash {
        // This is a first boot after program reflashing since the build_time has changed.
        // Save the new build_hash in flash memory
        flash
            .write(HASH_ADDR, &build_hash)
            .expect("Cannot write to flash memory.");

        return true;
    }

    false
}

/**
 * Low power TX modules have a soldered LiPo and persist the RTC
 * state internally on chip.
 */
pub fn set_tx_rtc_time(rtc: &Rtc) -> () {
    let dt = build_datetime();

    if is_first_boot(&dt) {
        rtc.set_current_time(dt);
    }
}

/**
 * High power RX module may be restarted and unpowered frequently.
 * Persist the RTC state externally on a DS3231 RTC module and load time
 * from external src if needed.
 */
pub fn set_rx_rtc_time<T>(chip_rtc: &Rtc, i2c: I2c<'_, Blocking, T>) -> ()
where
    T: esp_hal::i2c::master::Instance,
{
    let build_dt = build_datetime();
    println!("Setting RX RTC time");

    let mut ext_rtc = Ds323x::new_ds3231(i2c);

    let now_dt: NaiveDateTime = match is_first_boot(&build_dt) {
        true => {
            ext_rtc.set_datetime(&build_dt).unwrap();

            build_dt
        }
        false => ext_rtc.datetime().unwrap(),
    };

    chip_rtc.set_current_time(now_dt);
    ext_rtc.destroy_ds3231(); // release ownership of i2c
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
