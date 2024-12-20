#![no_std]
#![no_main]

use core::cell::RefCell;
use embassy_executor::Spawner;
use embedded_hal_bus::spi::RefCellDevice;
use embedded_sdmmc::SdCard;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    prelude::*,
    rtc_cntl::Rtc,
    spi::{
        master::{Config, Spi},
        SpiMode,
    },
};
use esp_wifi::esp_now::EspNow;
use espnow_sense_rs::set_rtc_time;
use log::info;
extern crate alloc;

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);
    esp_println::logger::init_logger_from_env();

    // init RTC
    let rtc = Rtc::new(peripherals.LPWR);
    set_rtc_time(&rtc);

    // init timers
    let timer0 =
        esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER)
            .split::<esp_hal::timer::systimer::Target>();
    esp_hal_embassy::init(timer0.alarm0);
    let timer1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);

    let delay = Delay::new();

    // init esp-now
    let init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();
    let wifi = peripherals.WIFI;
    let mut esp_now: EspNow = EspNow::new(&init, wifi).unwrap();

    // init SPI based on pinout: https://files.waveshare.com/wiki/ESP32-C6-LCD-1.47/ESP32-C6-LCD-1.47_schemetics.pdf
    let spi = Spi::new_with_config(
        peripherals.SPI2,
        Config {
            frequency: 400.kHz(),
            mode: SpiMode::Mode0,
            ..Config::default()
        },
    )
    .with_sck(peripherals.GPIO7)
    .with_mosi(peripherals.GPIO6)
    .with_miso(peripherals.GPIO5)
    .with_cs(peripherals.GPIO4)
    .into_async();

    // https://github.com/syrtcevvi/rust-esp32-embedded-sdmmc/blob/master/examples/esp-hal/src/main.rs
    let spi = RefCell::new(spi);
    let sd_spi_device = RefCellDevice::new(&spi, DummyCsPin, delay).unwrap();
    let sd_card = SdCard::new(sd_spi_device, delay);

    loop {
        let recv = esp_now.receive_async().await;

        if let Ok(bytes) = recv.data().try_into() {
            let val = f64::from_le_bytes(bytes);
            info!("recv T meas {}", val);
        }
    }
}

// explanation: https://docs.rs/embedded-sdmmc/latest/embedded_sdmmc/struct.SdCard.html
pub struct DummyCsPin;
impl embedded_hal::digital::ErrorType for DummyCsPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for DummyCsPin {
    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
