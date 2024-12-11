#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    prelude::*,
    rng::Rng,
    time::{self, Duration},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_wifi::{esp_now::BROADCAST_ADDRESS, init};

#[entry]
fn main() -> ! {
    let start = time::now();
    esp_println::logger::init_logger_from_env();

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::Clock80MHz; // underclock
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let init = init(
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let mut esp_now = esp_wifi::esp_now::EspNow::new(&init, wifi).unwrap();

    println!("esp-now version {}", esp_now.version().unwrap());

    let status = esp_now
        .send(&BROADCAST_ADDRESS, b"0123456789")
        .unwrap()
        .wait();
    println!("Send broadcast status: {:?}", status);

    let duration: u64 = time::now().checked_duration_since(start).unwrap().ticks();

    println!("took {}ms", duration / 1000);

    loop {}
}
