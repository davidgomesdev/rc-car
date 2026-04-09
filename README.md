# rc-car ESP32-S3 Motor Driver

This project drives a DC motor through a Keyestudio KS0066 (TB6612FNG) motor driver from an ESP32-S3.

## Wiring

Default pin mapping in `src/main.rs`:

- `GPIO4` -> `PWMA` (PWM)
- `GPIO5` -> `AIN1`
- `GPIO6` -> `AIN2`
- Set `STBY` high (or wire it to a GPIO and drive it high in software)
- ESP32-S3 `GND` -> motor driver `GND`
- External motor power to the driver (`VM` / `VCC_MOTOR`), not directly from the ESP32 pin

## Host Simulation (quick sanity check)

Run this on your Mac to verify speed-to-command mapping:

```bash
cargo run
cargo test
```

## Build/Flash for ESP32-S3 (esp-idf target)

1. Install Espressif Rust toolchain (`espup`) and export env in your shell.
2. Build for `xtensa-esp32s3-espidf`.
3. Flash with your preferred tool (`espflash` or `cargo espflash`).

Example commands:

```bash
rustup target add xtensa-esp32s3-espidf
cargo build --target xtensa-esp32s3-espidf --release
cargo run --target xtensa-esp32s3-espidf
```

## Motor Behavior

- On boot, motor output is forced to stopped state (`AIN1=0`, `AIN2=0`, duty `0`).
- Demo loop sends a sequence of speeds: `-40`, `0`, `40`, `80`, `0`, `-40`, `-80`, `0`.
- Adjust sequence, PWM frequency, and pins in `src/main.rs` to fit your hardware.

