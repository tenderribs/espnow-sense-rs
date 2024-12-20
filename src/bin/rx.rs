#![no_std]
#![no_main]

use embassy_executor::Spawner;

use esp_backtrace as _;
use esp_hal::prelude::*;
use esp_wifi::esp_now::EspNow;
use log::info;

extern crate alloc;

// https://files.waveshare.com/wiki/ESP32-C6-LCD-1.47/ESP32-C6-LCD-1.47_schemetics.pdf

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);
    esp_println::logger::init_logger_from_env();

    let timer0 =
        esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER)
            .split::<esp_hal::timer::systimer::Target>();
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let timer1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();
    let wifi = peripherals.WIFI;
    let mut esp_now: EspNow = EspNow::new(&init, wifi).unwrap();

    loop {
        let recv = esp_now.receive_async().await;

        if let Ok(bytes) = recv.data().try_into() {
            let val = u64::from_le_bytes(bytes);
            info!("received temperature meas {}", val);
        }
    }
}
