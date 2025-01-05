#![no_std]
#![no_main]

use alloc::format;
use chrono::{Datelike, Timelike};
use core::cell::RefCell;
use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};

use embedded_hal_bus::spi::RefCellDevice;
use embedded_sdmmc::{Mode, SdCard, VolumeIdx, VolumeManager};
use esp_backtrace as _;
use esp_hal::{
    delay::Delay, gpio::{Level, Output}, i2c::{self, master::I2c}, peripherals::TIMG0, prelude::*, rng::Rng, rtc_cntl::Rtc, spi::{self, master::Spi, SpiMode}
};
use esp_wifi::esp_now::EspNow;
use espnow_sense_rs::set_rx_rtc_time;
use log::info;
use mipidsi::{
    models::ST7789,
    options::{ColorInversion, Orientation, Rotation},
    Builder, Display,
};

extern crate alloc;

const TX_DEVS: [TxDevice; 2] = [
    TxDevice {
        name: "Dev 1",
        addr: "[54, 32, 04, 33, 6b, 04]",
        plot_color: Rgb565::CSS_LIGHT_SKY_BLUE,
    },
    TxDevice {
        name: "Dev 2",
        addr: "[54, 32, 04, 33, 69, 90]",
        plot_color: Rgb565::CSS_LIGHT_GREEN,
    },
];

async fn load_clock(
    spawner: Spawner,
    timg0: TIMG0,
    rng: Rng) -> Result<Clock

#[main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);
    esp_println::logger::init_logger_from_env();

    let sd_cs = Output::new(peripherals.GPIO4, Level::High);

    let lcd_cs = Output::new(peripherals.GPIO14, Level::High);
    let lcd_dc = Output::new(peripherals.GPIO15, Level::High);
    let lcd_rst = Output::new(peripherals.GPIO21, Level::High);
    let mut lcd_bl = Output::new(peripherals.GPIO22, Level::Low);

    // init RTC
    let rtc = Rtc::new(peripherals.LPWR);

    // I2C bus to access the external RTC
    let i2c: I2c<'_, esp_hal::Blocking> =
        I2c::new(peripherals.I2C0, i2c::master::Config::default())
            .with_scl(peripherals.GPIO0)
            .with_sda(peripherals.GPIO1);

    set_rx_rtc_time(&rtc, i2c);

    // init timers
    let timer0 =
        esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER)
            .split::<esp_hal::timer::systimer::Target>();
    esp_hal_embassy::init(timer0.alarm0);
    let timer1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);

    let mut delay = Delay::new();

    // init SPI based on pinout: https://files.waveshare.com/wiki/ESP32-C6-LCD-1.47/ESP32-C6-LCD-1.47_schemetics.pdf
    let spi_driver = Spi::new_with_config(
        peripherals.SPI2,
        spi::master::Config {
            frequency: 400.kHz(),
            mode: SpiMode::Mode0,
            ..spi::master::Config::default()
        },
    )
    .with_sck(peripherals.GPIO7)
    .with_miso(peripherals.GPIO5)
    .with_mosi(peripherals.GPIO6)
    .into_async();
    // https://github.com/syrtcevvi/rust-esp32-embedded-sdmmc/blob/master/examples/esp-hal/src/main.rs
    let spi_bus = RefCell::new(spi_driver);

    // prepare SD card access as a device over a shared SPI bus via refcell
    let lcd_spi_dev = RefCellDevice::new(&spi_bus, lcd_cs, delay).unwrap();
    let di = SPIInterface::new(lcd_spi_dev, lcd_dc);
    let mut display = Builder::new(ST7789, di)
        .reset_pin(lcd_rst)
        .invert_colors(ColorInversion::Normal)
        .orientation(Orientation::default().rotate(Rotation::Deg270))
        .init(&mut delay)
        .expect("Cannot init display");

    let sd_spi_device = RefCellDevice::new(&spi_bus, sd_cs, delay).unwrap();
    // open SD card when inserted
    let sd_card = SdCard::new(sd_spi_device, delay);
    let mut volume_mgr = VolumeManager::new(sd_card, SdRtc::new(&rtc));

    // open CSV file for writing
    let mut volume0 = volume_mgr
        .open_volume(VolumeIdx(0))
        .expect("Can't open first partition on SD card");
    let mut root_dir =
        volume0.open_root_dir().expect("Cannot open root directory");
    let mut file = root_dir
        .open_file_in_dir("LOG.CSV", Mode::ReadWriteCreateOrAppend)
        .expect("Cannot open CSV file for appending");

    // init esp-now
    let init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();
    let wifi = peripherals.WIFI;
    let mut esp_now: EspNow = EspNow::new(&init, wifi).unwrap();

    // start LCD background task
    spawner.spawn(lcd_background_task()).ok();

    // receive ESP-Now data and write to SD-Card
    loop {
        let recv = esp_now.receive_async().await;
        let src_addr = format!("{:02x?}", &recv.info.src_address[..6]);

        if let Ok(bytes) = recv.data().try_into() {
            let val = f64::from_le_bytes(bytes);

            // construct line
            let timestamp: alloc::string::String =
                rtc.current_time().and_utc().to_rfc3339();
            let line = format!(
                "{}, {}, {}, {}\r\n",
                timestamp, src_addr, val, recv.info.rx_control.rssi
            );
            info!("{}", line);

            // append line to file
            if let Ok(()) = file.seek_from_end(0) {
                if let Ok(()) = file.write(line.as_bytes()) {
                    file.flush().unwrap();
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn lcd_background_task() {
    // Turn on the backlight
    lcd_bl.set_high();

    // Clear the display initially
    display.clear(Rgb565::WHITE).unwrap();

    loop {
        // Wait 10 seconds before updating
        Timer::after(Duration::from_secs(10)).await;

        // Perform any display updates here
        display.clear(Rgb565::CYAN).unwrap(); // Example update
    }
}

struct TxDevice<'a> {
    addr: &'a str,
    plot_color: Rgb565,
    name: &'a str,
}

impl TxDevice<'_> {
    fn addr_str(&self) -> alloc::string::String {
        format!("{:02x?}", &self.addr[..6])
    }
}

// shadow the Rtc struct to implement the TimeSource trait.
struct SdRtc<'a> {
    rtc: &'a Rtc<'a>,
}

impl<'a> SdRtc<'a> {
    pub fn new(rtc: &'a Rtc<'a>) -> Self {
        Self { rtc }
    }
}

impl<'a> embedded_sdmmc::TimeSource for SdRtc<'a> {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        let now = self.rtc.current_time();

        embedded_sdmmc::Timestamp {
            year_since_1970: (now.year() - 1970).unsigned_abs() as u8,
            zero_indexed_month: now.month().wrapping_sub(1) as u8,
            zero_indexed_day: now.day().wrapping_sub(1) as u8,
            hours: now.hour() as u8,
            minutes: now.minute() as u8,
            seconds: now.second() as u8,
        }
    }
}
