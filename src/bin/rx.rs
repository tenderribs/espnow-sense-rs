#![no_std]
#![no_main]

use alloc::format;
use chrono::{Datelike, NaiveDateTime, Timelike};
use core::cell::RefCell;
use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    pubsub::{self, PubSubChannel},
};
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};
use embedded_hal_bus::spi::RefCellDevice;
use embedded_sdmmc::{Mode, SdCard, VolumeIdx, VolumeManager};
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output},
    i2c::{self, master::I2c},
    prelude::*,
    rtc_cntl::Rtc,
    spi::{self, master::Spi, SpiMode},
};
use esp_println::println;
use esp_wifi::esp_now::EspNow;
use espnow_sense_rs::set_rx_rtc_time;
use heapless::{FnvIndexMap, Vec};
use mipidsi::{
    models::ST7789,
    options::{ColorInversion, Orientation, Rotation},
    Builder,
};

extern crate alloc;

static CHANNEL: MeasChannel = MeasChannel::new();

const TX_DEVS: [[u8; 6]; 3] = [
    [0x54, 0x32, 0x04, 0x33, 0x69, 0x90], // 54:32:04:33:69:90
    [0x54, 0x32, 0x04, 0x33, 0x6b, 0x04], // 54:32:04:33:6b:04
    [0x54, 0x32, 0x04, 0x33, 0x66, 0x3c], // 54:32:04:33:66:3c
];

type MeasChannel = PubSubChannel<CriticalSectionRawMutex, Measurement, 4, 2, 1>;
// type MeasSubscriber =
//     Subscriber<'static, CriticalSectionRawMutex, Measurement, 4, 2, 1>;

#[esp_hal_embassy::main]
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

    // access external RTC via I2c
    let i2c = I2c::new(peripherals.I2C0, i2c::master::Config::default())
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
    // https://github.com/syrtcevvi/rust-esp32-embedded-sdmmc/blob/master/examples/esp-hal/src/main.rs
    let spi_bus = Spi::new_with_config(
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
    let spi_bus = RefCell::new(spi_bus);

    let _lcd_spi_dev = RefCellDevice::new(&spi_bus, lcd_cs, delay).unwrap();
    let _sd_spi_device = RefCellDevice::new(&spi_bus, sd_cs, delay).unwrap(); // prepare SD card access as a device over a shared SPI bus via refcell

    // init esp-now
    let init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();
    let wifi = peripherals.WIFI;
    let mut esp_now: EspNow = EspNow::new(&init, wifi).unwrap();

    let pub0 = CHANNEL.publisher().unwrap();

    spawner.spawn(display_task()).ok();
    spawner.spawn(sd_card_task()).ok();

    loop {
        let recv = esp_now.receive_async().await;
        println!("received message");
        // only process comms from whitelisted TX devices
        if !TX_DEVS.iter().any(|&txdev| txdev == recv.info.src_address) {
            println!("rejecting message");
            continue;
        }

        println!("proceeding with message");

        // if let Ok(bytes) = recv.data().try_into() {
        //     let value = f32::from_le_bytes(bytes);
        //     println!("parsed bytes as f32");

        //     let meas = Measurement {
        //         timestamp: rtc.current_time(),
        //         src_addr: recv.info.src_address,
        //         value,
        //     };

        //     pub0.publish_immediate(meas);
        // }
    }
}

#[embassy_executor::task]
async fn display_task() {
    println!("starting display task");
    // let di = SPIInterface::new(lcd_spi_dev, lcd_dc);
    // let mut display = Builder::new(ST7789, di)
    //     .reset_pin(lcd_rst)
    //     .invert_colors(ColorInversion::Normal)
    //     .orientation(Orientation::default().rotate(Rotation::Deg270))
    //     .init(&mut delay)
    //     .expect("Cannot init display");
    // display.clear(Rgb565::WHITE).unwrap();

    type Points = Vec<Measurement, 50>;

    // create hashmap to store 50 measurements per TX device
    let mut ds: FnvIndexMap<[u8; 6], Points, 4> = FnvIndexMap::new();
    assert!(TX_DEVS.len() < 4);
    TX_DEVS.iter().for_each(|&txdev| {
        ds.insert(txdev, Points::new()).unwrap();
    });

    let mut sub = CHANNEL.subscriber().unwrap();
    loop {
        if let pubsub::WaitResult::Message(_meas) = sub.next_message().await {
            // process message
            println!("received value in display task");
        }
    }
}

#[embassy_executor::task]
async fn sd_card_task() {
    println!("starting SD card task");
    let mut sub = CHANNEL.subscriber().unwrap();

    // let sd_card = SdCard::new(sd_spi_device, delay); // open SD card when inserted
    // let mut volume_mgr = VolumeManager::new(sd_card, SdRtc::new(&rtc));

    // // open CSV file for writing
    // let mut volume0 = volume_mgr
    //     .open_volume(VolumeIdx(0))
    //     .expect("Can't open first partition on SD card");
    // let mut root_dir =
    //     volume0.open_root_dir().expect("Cannot open root directory");
    // let mut file = root_dir
    //     .open_file_in_dir("LOG.CSV", Mode::ReadWriteCreateOrAppend)
    //     .expect("Cannot open CSV file for appending");

    loop {
        if let pubsub::WaitResult::Message(meas) = sub.next_message().await {
            let src_addr = format!("{:02x?}", &meas.src_addr[..6]);

            let _line = format!(
                "{}, {}, {}\r\n",
                meas.timestamp.and_utc().to_rfc3339(),
                src_addr,
                meas.value
            );

            println!("received value in sd card task");

            // // append line
            // if let Ok(()) = file.seek_from_end(0) {
            //     if let Ok(()) = file.write(line.as_bytes()) {
            //         file.flush().unwrap();
            //     }
            // }
        }
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

#[derive(Clone, Debug)]
struct Measurement {
    timestamp: NaiveDateTime,
    value: f32,
    src_addr: [u8; 6],
}

struct PlotPoint {
    timestamp: NaiveDateTime,
    value: f32,
}

impl From<Measurement> for PlotPoint {
    fn from(m: Measurement) -> Self {
        Self {
            timestamp: m.timestamp,
            value: m.value,
        }
    }
}
