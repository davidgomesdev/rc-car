# AGENTS.md — Project knowledge base for AI agents

This file documents the full architecture, conventions, and workflow of the
`rc-car` project so that any AI agent can orient itself quickly.

---

## 1. What this project is

A Rust firmware for an **ESP32-S3** that drives **four DC motors** through two
**Keyestudio KS0066 (TB6612FNG)** dual-channel motor driver chips.  
The same codebase compiles for the **native host** (macOS / Linux) for quick
logic verification without hardware.

---

## 2. Repository layout

```
rc-car/
├── Cargo.toml          # workspace manifest; ESP deps are target-gated
├── Cargo.lock
├── build.rs            # calls embuild::espidf::sysenv::output() for ESP target
├── Makefile            # convenience targets: build-host, build-esp, flash
├── README.md           # hardware wiring reference
├── AGENTS.md           # ← this file
└── src/
    ├── lib.rs          # platform-agnostic motor API (no std, no ESP types)
    └── main.rs         # ESP-IDF hardware driver + host simulation entry point
```

---

## 3. Dual-target compilation model

The project uses **`cfg(target_os = "espidf")`** to switch between the two
compilation paths. There are no feature flags for this.

| Target                                     | `target_os`   | What compiles                              |
|--------------------------------------------|---------------|--------------------------------------------|
| `xtensa-esp32s3-espidf`                    | `"espidf"`    | Full firmware: ESP-IDF HAL, LEDC PWM, GPIO |
| Native host (`aarch64-apple-darwin`, etc.) | anything else | Simulation `main` only                     |

### `.cargo/config.toml` (important)

The default Cargo target is locked to `xtensa-esp32s3-espidf`.  
**Always pass `--target <host-triple>` when running tests or the host binary.**

```toml
[build]
target = "xtensa-esp32s3-espidf"

[target.'cfg(target_os = "espidf")']
linker = "ldproxy"
rustflags = ["--cfg", "espidf_time64"]

[env]
ESP_IDF_SDKCONFIG_DEFAULTS = ".github/configs/sdkconfig.defaults"
ESP_IDF_VERSION = "v5.5.3"

[unstable]
build-std = ["std", "panic_abort"]
```

---

## 4. Dependencies

### Runtime (ESP-IDF only — `[target.'cfg(target_os = "espidf")'.dependencies]`)

| Crate         | Version | Purpose                                                           |
|---------------|---------|-------------------------------------------------------------------|
| `anyhow`      | 1       | Error propagation with `?`                                        |
| `esp-idf-hal` | 0.46.2  | GPIO (`PinDriver`) and LEDC PWM (`LedcDriver`, `LedcTimerDriver`) |
| `esp-idf-svc` | 0.52.1  | Logging (`EspLogger`) and system patches (`link_patches`)         |

The native host has **no runtime dependencies** — `lib.rs` is pure Rust.

### Build

| Crate     | Version                   | Purpose                                      |
|-----------|---------------------------|----------------------------------------------|
| `embuild` | 0.33 (feature `"espidf"`) | Emits ESP-IDF linker env vars via `build.rs` |

> **Why `features = ["espidf"]`?**  
> `build.rs` references `embuild::espidf::sysenv` at compile time, so the
> feature must be unconditionally enabled even when building for the host.
> The runtime guard (`if CARGO_CFG_TARGET_OS == "espidf"`) prevents it from
> actually running on non-ESP builds.

---

## 5. Public API (`src/lib.rs`)

Everything in `lib.rs` is platform-agnostic (no `std` requirement, no ESP
imports). It lives in the `rc_car` crate.

### Low-level — single motor

```rust
pub enum Direction { Forward, Reverse, Stop }

pub struct MotorPins {
    pub in1_high: bool,
    pub in2_high: bool
}

pub struct MotorCommand {
    pub direction: Direction,
    pub pins: MotorPins,
    pub duty: u32,          // absolute PWM duty value, 0..=max_duty
}

impl MotorCommand {
    /// speed_percent: -100..=100 (clamped). max_duty from LedcDriver::get_max_duty().
    pub fn from_percent(speed_percent: i8, max_duty: u32) -> Self;
}
```

**Direction encoding for TB6612FNG:**

| State        | IN1  | IN2  |
|--------------|------|------|
| Forward      | HIGH | LOW  |
| Reverse      | LOW  | HIGH |
| Stop / brake | LOW  | LOW  |

### High-level — four motors

```rust
pub enum MotorId { FrontLeft, FrontRight, RearLeft, RearRight }

pub struct CarCommand {
    pub front_left: MotorCommand,
    pub front_right: MotorCommand,
    pub rear_left: MotorCommand,
    pub rear_right: MotorCommand,
}

impl CarCommand {
    // Drive all four motors at the same speed (-100..=100).
    pub fn drive(speed: i8, max_duty: u32) -> Self;

    // Tank-style: left side and right side independently (-100..=100).
    pub fn steer(left_speed: i8, right_speed: i8, max_duty: u32) -> Self;

    // Right motors drive, left motors stopped.
    pub fn turn_left(speed: i8, max_duty: u32) -> Self;

    // Left motors drive, right motors stopped.
    pub fn turn_right(speed: i8, max_duty: u32) -> Self;

    // Left reverse, right forward — spin in place.
    pub fn spin_left(speed: i8, max_duty: u32) -> Self;

    // Left forward, right reverse — spin in place.
    pub fn spin_right(speed: i8, max_duty: u32) -> Self;

    // Zero duty, all direction pins LOW.
    pub fn stop(max_duty: u32) -> Self;

    // Builder: override one motor, leave the other three unchanged.
    pub fn with_motor(self, id: MotorId, speed: i8, max_duty: u32) -> Self;
}
```

`spin_left` / `spin_right` clamp their `speed` argument to `0..=100` before
negating; calling them with a negative value is safe but treated as positive.

---

## 6. Hardware driver (`src/main.rs`)

### `EspMotorController<'d>` (ESP-IDF only)

Owns all twelve HAL drivers (4 motors × IN1 + IN2 + PWM).  
The lifetime `'d` is tied to the ESP peripheral ownership (`Peripherals::take()`).

```
EspMotorController<'d>
 ├── fr_in1 / fr_in2 : PinDriver<'d, Output>   — Front-Right direction
 ├── fr_pwm          : LedcDriver<'d>           — Front-Right PWM
 ├── rr_in1 / rr_in2 : PinDriver<'d, Output>   — Rear-Right direction
 ├── rr_pwm          : LedcDriver<'d>           — Rear-Right PWM
 ├── fl_in1 / fl_in2 : PinDriver<'d, Output>   — Front-Left direction
 ├── fl_pwm          : LedcDriver<'d>           — Front-Left PWM
 ├── rl_in1 / rl_in2 : PinDriver<'d, Output>   — Rear-Left direction
 ├── rl_pwm          : LedcDriver<'d>           — Rear-Left PWM
 └── max_duty        : u32
```

**Methods:**

| Method                               | Description                                      |
|--------------------------------------|--------------------------------------------------|
| `drive_pins(in1, in2, pwm, cmd)`     | Static helper — sets one motor's GPIO and duty   |
| `apply(&mut self, cmd: &CarCommand)` | Drives all four motors from one `CarCommand`     |
| `stop(&mut self)`                    | Convenience: `apply(CarCommand::stop(max_duty))` |

### Initialization sequence in `main()`

1. `Peripherals::take()` — takes ownership of all peripherals.
2. `LedcTimerDriver::new(timer0, 25 kHz)` — one shared PWM timer.
3. `LedcDriver::new(channel0, &timer, gpio4)` — first channel; read `max_duty` from it.
4. Construct `EspMotorController` with all remaining channels and GPIO pins.
5. `controller.stop()` — safe state before any movement.
6. Infinite demo loop using `CarCommand` variants with 1500 ms delays.

### Default pin mapping

| Motor       | IN1    | IN2    | PWM    | LEDC channel |
|-------------|--------|--------|--------|--------------|
| Front-Right | GPIO6  | GPIO5  | GPIO4  | ch0          |
| Rear-Right  | GPIO7  | GPIO8  | GPIO16 | ch1          |
| Front-Left  | GPIO38 | GPIO39 | GPIO40 | ch2          |
| Rear-Left   | GPIO37 | GPIO36 | GPIO35 | ch3          |

**Reserved / avoid:** GPIO 0, 19, 20, 45, 46.  
Both chips share a common `GND` with the ESP32-S3. Motor power (`VM`) comes
from an external supply — not the ESP's 3.3 V or 5 V pin. `STBY` must be
pulled HIGH (wire to 3V3 or drive with a GPIO).

### Host simulation `main()`

When the target OS is not `espidf`, a simple `main()` prints the
`MotorCommand` fields for each `CarCommand` variant. No hardware required.

---

## 7. Build & flash workflow

### Run tests / host simulation

```bash
# Replace aarch64-apple-darwin with your host triple if different
cargo test --target aarch64-apple-darwin
cargo run  --target aarch64-apple-darwin
```

### Build firmware

Requires the Espressif Rust toolchain (`espup`) and the env exported from
`~/export-esp.sh`.

```bash
# Via Makefile (recommended)
make build-esp          # release by default
make build-esp PROFILE=debug

# Manually
. ~/export-esp.sh
cargo +esp build -Zbuild-std=std,panic_abort \
    --target xtensa-esp32s3-espidf --release
```

### Flash

```bash
make flash PORT=/dev/cu.usbserial-XXXX   # PORT optional if only one device
# or
espflash flash --baud 460800 target/xtensa-esp32s3-espidf/release/rc-car
```

### Makefile targets

| Target       | Action                                 |
|--------------|----------------------------------------|
| `build-host` | Native debug binary                    |
| `build-esp`  | ESP32-S3 firmware (release by default) |
| `flash`      | build-esp then flash with espflash     |
| `clean`      | `cargo clean`                          |

---

## 8. Testing

All tests live in `src/lib.rs` (unit tests, `#[cfg(test)]`).  
There are **11 tests** covering:

- `MotorCommand::from_percent`: stop at 0, forward/reverse polarity, duty
  calculation, out-of-range clamping.
- `CarCommand`: drive all motors, reverse all, stop all, independent steering,
  spin left/right, `with_motor` override.

Tests always run on the **host target** — they import nothing from the
ESP-IDF HAL.

---

## 9. Key conventions & gotchas

- **`max_duty` is always threaded explicitly.** The LEDC resolution (and thus
  `max_duty`) depends on the timer config. The value is read from
  `LedcDriver::get_max_duty()` once after the first channel is initialised
  and then stored in `EspMotorController::max_duty`.

- **`CarCommand` is `Copy`.** All structs in the API derive `Copy`/`Clone`.
  Prefer passing `&CarCommand` in hot paths to make intent clear.

- **No shared timer ownership issue.** Multiple `LedcDriver` instances can be
  created from `&LedcTimerDriver` (the HAL accepts `impl Borrow<LedcTimerDriver<'d>>`).
  The timer lives on the stack in `main()` and outlives all channel drivers.

- **`embuild` `espidf` feature must be enabled unconditionally.** Without it,
  `build.rs` fails to compile even for host targets because the module path
  `embuild::espidf` doesn't exist.

- **Default Cargo target is `xtensa-esp32s3-espidf`.** Always add
  `--target <host-triple>` for host builds; omitting it will attempt an
  ESP-IDF cross-compilation.

- **`log::info!` vs `println!`.** The ESP-IDF path uses `log::info!` (routed
  through `EspLogger`). The host path uses `println!`. Do not use
  `log::info!` in `#[cfg(not(target_os = "espidf"))]` code without adding a
  logger dependency for the host.

