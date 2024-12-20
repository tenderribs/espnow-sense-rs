## Installation

```sh
# Install toolchain deps
sudo apt install llvm-dev libclang-dev clang


# Install Toolchain
rustup toolchain install stable --component rust-src

# Set the target
rustup target add riscv32imac-unknown-none-elf # For ESP32-C6 and ESP32-H2

# Install the flasher if not already done
sudo apt-get install libudev-dev
cargo install cargo-espflash espflash
```

## TX Side

Inside `src/bin/tx.rs`, you can modify parameters governing the sleep schedule. The microcontroller wakes up from deep sleep every `SLEEP_DURATION_S` seconds and broadcasts a temperature measurement. To further conserve power during the night, an extended deep sleep period is entered between `BEDTIME_HR` and `WAKEUP_HR`. These times are specified in the local timezone and require an explicit UTC offset to convert from the RTC's datetime in UTC.

```rs
// default interval between measurements
const SLEEP_DURATION_S: u64 = 5 * 60;

// extended deep sleep config during overnight stop
const UTC_DIFF: f32 = 1.0; // timezone diff to UTC. in CH should be +1 or +2, in AU should be +9.5 or +10.5
const BEDTIME_HR: f32 = 22.0; // when it starts. ex. 21.5 is 09:30 PM, is timezone aware
const WAKEUP_HR: f32 = 6.0; // when to wake up from deep sleep, is timezone aware
```

## Relevant Documentation:
- [ESP32-C6-LCD-1.47_schemetics.pdf](https://files.waveshare.com/wiki/ESP32-C6-LCD-1.47/ESP32-C6-LCD-1.47_schemetics.pdf)
- [Temp: Datasheet_STS3x_DIS.pdf](https://sensirion.com/media/documents/1DA31AFD/65D613A8/Datasheet_STS3x_DIS.pdf)