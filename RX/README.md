## Installation

```sh
# Install Toolchain
rustup toolchain install stable --component rust-src

# Set the target
rustup target add riscv32imac-unknown-none-elf # For ESP32-C6 and ESP32-H2

# Install the flasher if not already done
sudo apt-get install libudev-dev
cargo install esp-flash
```