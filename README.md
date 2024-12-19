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

https://files.waveshare.com/wiki/ESP32-C6-LCD-1.47/ESP32-C6-LCD-1.47_schemetics.pdf
