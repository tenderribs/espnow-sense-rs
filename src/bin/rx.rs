#![no_std]
#![no_main]

use alloc::{format, string::String, sync::Arc};
use chrono::{Datelike, NaiveDateTime, Timelike};
use core::cell::RefCell;
use critical_section::Mutex;
use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    pubsub::{self, PubSubChannel},
};
use embedded_graphics::{
    mono_font::{
        ascii::{FONT_6X10, FONT_8X13},
        iso_8859_13::FONT_10X20,
        MonoTextStyle,
    },
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use embedded_hal_bus::spi::CriticalSectionDevice;
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
use heapless::FnvIndexMap;
use mipidsi::{
    models::ST7789,
    options::{ColorInversion, Orientation, Rotation},
    Builder,
};
use static_cell::StaticCell;

type SpiDevice<'a> =
    CriticalSectionDevice<'a, Spi<'a, esp_hal::Async>, Output<'a>, Delay>;

extern crate alloc;

static CHANNEL: Channel = Channel::new();
static SPI_BUS: StaticCell<Mutex<RefCell<Spi<'static, esp_hal::Async>>>> =
    StaticCell::new();

const TX_DEVS: [TxDevice; 3] = [
    TxDevice {
        name: "Dev1",
        mac_addr: [0x54, 0x32, 0x04, 0x33, 0x69, 0x90],
    },
    TxDevice {
        name: "Dev2",
        mac_addr: [0x54, 0x32, 0x04, 0x33, 0x6b, 0x04],
    },
    TxDevice {
        name: "Dev3",
        mac_addr: [0x54, 0x32, 0x04, 0x33, 0x66, 0x3c],
    },
];

type Channel = PubSubChannel<CriticalSectionRawMutex, Message, 4, 2, 1>;

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
    let lcd_pins = LcdPins {
        dc: Output::new(peripherals.GPIO15, Level::High),
        rst: Output::new(peripherals.GPIO21, Level::High),
        bl: Output::new(peripherals.GPIO22, Level::Low),
    };

    // init RTC
    let rtc: Arc<Rtc<'static>> = Arc::new(Rtc::new(peripherals.LPWR));

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

    let delay = Delay::new();

    let spi_bus = Spi::new_with_config(
        peripherals.SPI2,
        spi::master::Config {
            frequency: 400.kHz(),
            mode: SpiMode::Mode0,
            ..spi::master::Config::default()
        },
    )
    // pinout: https://files.waveshare.com/wiki/ESP32-C6-LCD-1.47/ESP32-C6-LCD-1.47_schemetics.pdf
    .with_sck(peripherals.GPIO7)
    .with_miso(peripherals.GPIO5)
    .with_mosi(peripherals.GPIO6)
    .into_async();

    let spi_bus = SPI_BUS.init(Mutex::new(RefCell::new(spi_bus)));

    let sd_spi_dev = CriticalSectionDevice::new(
        unsafe { &*core::ptr::addr_of!(spi_bus) }, // unsafe is justifyable because StaticCell guarantees static lifetime
        sd_cs,
        delay,
    )
    .unwrap();

    let lcd_spi_dev = CriticalSectionDevice::new(
        unsafe { &*core::ptr::addr_of!(spi_bus) }, // unsafe is justifyable because StaticCell guarantees static lifetime
        lcd_cs,
        delay,
    )
    .unwrap();

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

    // start background tasks
    spawner.must_spawn(lcd_task(lcd_spi_dev, lcd_pins, delay));
    spawner.must_spawn(sd_card_task(sd_spi_dev, delay, Arc::clone(&rtc)));

    // enter main foreground loop
    loop {
        let recv = esp_now.receive_async().await;

        if !TX_DEVS
            .iter()
            .any(|txd| txd.mac_addr == recv.info.src_address)
        {
            continue;
        }

        if let Ok(bytes) = recv.data().try_into() {
            let value = f32::from_le_bytes(bytes);

            let msg = Message {
                meas: Measurement {
                    timestamp: rtc.current_time(),
                    value,
                },
                mac_addr: recv.info.src_address,
            };

            pub0.publish_immediate(msg);
        }
    }
}

#[embassy_executor::task]
async fn lcd_task(
    spi_dev: SpiDevice<'static>,
    mut pins: LcdPins<'static>,
    mut delay: Delay,
) {
    let di = SPIInterface::new(spi_dev, pins.dc);
    let mut display = Builder::new(ST7789, di)
        .reset_pin(pins.rst)
        .invert_colors(ColorInversion::Inverted)
        .orientation(Orientation::default().rotate(Rotation::Deg270))
        .init(&mut delay)
        .expect("Cannot init display");

    display.clear(Rgb565::BLACK).unwrap();
    pins.bl.set_high();

    let style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);

    let mut history = FnvIndexMap::<[u8; 6], Option<NaiveDateTime>, 4>::new();
    TX_DEVS.iter().for_each(|txd| {
        history
            .insert(txd.mac_addr, None)
            .expect("unable to init history");
    });

    let mut sub = CHANNEL.subscriber().unwrap();

    loop {
        if let pubsub::WaitResult::Message(msg) = sub.next_message().await {
            // update last seen timestamp
            if let Some(last_seen) = history.get_mut(&msg.mac_addr) {
                *last_seen = Some(msg.meas.timestamp)
            }
        }

        display.clear(Rgb565::BLACK).unwrap();

        // update the screen
        for (i, (mac_addr, last_seen)) in history.iter().enumerate() {
            // find human readable name defined in TX_DEVS
            let name =
                match TX_DEVS.iter().find(|txd| txd.mac_addr == *mac_addr) {
                    Some(txd) => txd.name,
                    _ => continue,
                };

            // create human readable string of date
            let seen = match last_seen {
                Some(t) => format!("{}", t.and_utc().format("%m-%d %H:%M UTC")),
                None => String::from("unavailable"),
            };

            let contents = format!("{}: {}", name, seen);

            Text::new(&contents[..], Point::new(20, i as i32 * 20 + 60), style)
                .draw(&mut display)
                .unwrap();
        }
    }
}

#[embassy_executor::task]
async fn sd_card_task(
    spi_dev: SpiDevice<'static>,
    delay: Delay,
    rtc: Arc<Rtc<'static>>,
) {
    println!("starting SD card task");
    let mut sub = CHANNEL.subscriber().unwrap();

    let sd_card = SdCard::new(spi_dev, delay); // open SD card when inserted
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

    loop {
        if let pubsub::WaitResult::Message(msg) = sub.next_message().await {
            // let a = meas.tx_dev.mac_addr;
            let src_addr = pretty_mac(msg.mac_addr);

            let line = format!(
                "{}, {}, {}\r\n",
                msg.meas.timestamp.and_utc().to_rfc3339(),
                src_addr,
                msg.meas.value
            );

            // append line
            if let Ok(()) = file.seek_from_end(0) {
                if let Ok(()) = file.write(line.as_bytes()) {
                    file.flush().unwrap();
                }
            }
        }
    }
}

fn pretty_mac(addr: [u8; 6]) -> String {
    format!(
        "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
    )
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
struct Message {
    mac_addr: [u8; 6],
    meas: Measurement,
}

#[derive(Clone, Debug)]
struct Measurement {
    timestamp: NaiveDateTime,
    value: f32,
}

struct TxDevice {
    name: &'static str,
    mac_addr: [u8; 6],
}

struct LcdPins<'a> {
    dc: Output<'a>,
    rst: Output<'a>,
    bl: Output<'a>,
}
