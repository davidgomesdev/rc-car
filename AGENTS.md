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
    ├── lib.rs              # module declarations only
    ├── main.rs             # ESP orchestrator + host simulation
    ├── controller.html     # web UI (served by api::web)
    ├── app/                # app models — platform-agnostic, host-testable
    │   ├── direction.rs    # Direction, MotorPins
    │   ├── command.rs      # MotorCommand, CarCommand, MotorId (+ 11 tests)
    │   └── controller.rs   # RemoteCmd, parse_cmd, to_car_command
    ├── internal/           # internal models — ESP-only hardware abstraction
    │   ├── gpio.rs         # build_controller: pin/channel assignment
    │   └── motor_driver.rs # EspMotorDriver, EspMotorController
    └── api/                # API layer — ESP-only services
        ├── wifi.rs         # WifiService
        ├── motors.rs       # MotorService (command bus + control loop)
        └── web.rs          # HTTP/WebSocket server
```

### Layered architecture

The crate is split into three layers, declared in `lib.rs` (which contains
nothing but `mod` declarations):

- **`app`** — always compiled, platform-agnostic, host-testable. No `std`
  requirement, no ESP imports. Contains `direction`, `command`, `controller`.
- **`internal`** — ESP-only (`#[cfg(target_os = "espidf")]`). Hardware
  abstraction over GPIO and the motor drivers.
- **`api`** — ESP-only. Service modules organised by concern: Wi-Fi
  connectivity, the motor command bus + control loop, and the HTTP/WS server.

`main.rs` is a thin orchestrator: it wires the layers together and runs the
control loop (ESP), or prints a `CarCommand` demo (host).

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
| `anyhow`       | 1       | Error propagation with `?`                                        |
| `log`          | 0.4     | Logging facade (`log::info!`), routed through `EspLogger`         |
| `esp-idf-hal`  | 0.46.2  | GPIO (`PinDriver`) and LEDC PWM (`LedcDriver`, `LedcTimerDriver`) |
| `esp-idf-svc`  | 0.52.1  | Wi-Fi, HTTP/WS server, logging (`EspLogger`), `link_patches`      |
| `embedded-svc` | 0.29    | Wi-Fi/HTTP traits used by the `api` layer                         |

The native host has **no runtime dependencies** — the `app` layer is pure Rust.

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

## 5. App layer (`src/app/`)

Everything in `app` is platform-agnostic (no ESP imports) and host-testable. It
is declared in `lib.rs` as `pub mod app;` and lives in the `rc_car` crate. (It
uses `std` — e.g. `String` in `CarCommand` — so it is not `no_std`.)

The layer has three modules:

- **`app::direction`** — `Direction`, `MotorPins`.
- **`app::command`** — `MotorCommand`, `CarCommand`, `MotorId` (the motor API
  below). The 11 unit tests live here.
- **`app::controller`** — the remote-control protocol: `RemoteCmd`,
  `parse_cmd`, `to_car_command` (see §6a).

### Low-level — single motor (`app::command`, `app::direction`)

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
    pub message: String,    // human-readable label for the command, e.g. "drive 50%"
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

## 6. Internal layer (`src/internal/`)

### `EspMotorController<'d>` (ESP-IDF only, `internal::motor_driver`)

Composed of four `EspMotorDriver<'d>` — one per wheel — plus the shared
`max_duty`. Each `EspMotorDriver` owns a single motor's two direction pins and
its PWM channel. The lifetime `'d` ties the HAL drivers to the ESP peripheral
ownership and the shared `LedcTimerDriver` borrow.

```
EspMotorController<'d>
 ├── front_right : EspMotorDriver<'d>   ┐  each driver owns:
 ├── rear_right  : EspMotorDriver<'d>   │    in1 : PinDriver<'d, Output>
 ├── front_left  : EspMotorDriver<'d>   │    in2 : PinDriver<'d, Output>
 ├── rear_left   : EspMotorDriver<'d>   ┘    pwm : LedcDriver<'d>
 └── max_duty    : u32
```

**Methods:**

| Type / method                                  | Description                                       |
|------------------------------------------------|---------------------------------------------------|
| `EspMotorDriver::new(in1, in2, pwm)`           | Build one motor's driver                          |
| `EspMotorDriver::max_duty(&self)`              | Max PWM duty for this channel                      |
| `EspMotorController::new(fr, rr, fl, rl, max)` | Build the four-motor controller                   |
| `EspMotorController::max_duty(&self)`          | Shared max PWM duty                                |
| `apply(&mut self, cmd: &CarCommand)`           | Drives all four motors from one `CarCommand`      |
| `stop(&mut self)`                              | Convenience: `apply(CarCommand::stop(max_duty))`  |

`EspMotorDriver::apply` (private) sets one motor's two direction pins and duty;
`EspMotorController::apply` fans a `CarCommand` out to all four drivers.

### Controller construction (`internal::gpio::build_controller`)

`build_controller` owns the default pin/channel map and returns a **stopped**
controller. `main()` calls it after taking peripherals:

1. `Peripherals::take()` — takes ownership of all peripherals.
2. `LedcTimerDriver::new(timer0, 25 kHz)` — one shared PWM timer, kept on
   `main`'s stack (it must outlive the controller).
3. `gpio::build_controller(&timer, channel0..3, pins)` — creates all four
   `LedcDriver`s + direction pins, reads `max_duty`, and calls `stop()` before
   returning.
4. `MotorService::new(controller)` wraps it with the shared command bus.

### Default pin mapping

| Motor       | IN1    | IN2    | PWM    | LEDC channel |
|-------------|--------|--------|--------|--------------|
| Front-Right | GPIO6  | GPIO5  | GPIO4  | ch0          |
| Rear-Right  | GPIO15 | GPIO7  | GPIO16 | ch1          |
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

## 6a. API layer & networking (`src/api/`)

The `api` layer (ESP-only) holds the service modules, organised by concern:

- **`api::wifi::WifiService`** — `connect(modem, sys_loop, nvs)` tries the
  client SSID (`CLIENT_WIFI_SSID` / `CLIENT_WIFI_PASSWORD`) and falls back to an
  access point (`AP_WIFI_SSID` / `AP_WIFI_PASSWORD`). `ip()` returns the current
  address on demand. The owned `WifiService` is kept alive by a binding in
  `main` for the whole program (the control loop never returns).
- **`api::motors::MotorService`** — owns the `EspMotorController` and exposes a
  shared `CommandBus = Arc<Mutex<RemoteCmd>>`. `command_bus()` hands a clone to
  the web server; `run()` polls the bus every **50 ms** and applies the mapped
  `CarCommand`. `run()` diverges (`-> !`).
- **`api::web::start(command_bus)`** — starts the HTTP/WS server and returns the
  server handle, which **must stay alive** (dropping it tears down the handlers).

### Runtime control flow

```
boot
 └─ WifiService::connect         (client SSID, AP fallback)
     └─ gpio::build_controller   (stopped controller)
         └─ MotorService::new
             └─ web::start(command_bus)   ── WS writes RemoteCmd ──┐
                 └─ MotorService::run  ── polls bus every 50 ms ───┘
```

Only `RemoteCmd` (a `Copy` enum) crosses the WebSocket thread boundary into the
shared `Arc<Mutex<>>` bus — no hardware types are shared across threads.

### HTTP / WebSocket endpoints

| Route             | Method | Purpose                                              |
|-------------------|--------|------------------------------------------------------|
| `/`               | GET    | Serves `controller.html` (the web UI)                |
| `/camerastatus?ip=` | GET  | Proxies the camera's `http://<ip>:8080/videostatus`  |
| `/ws`             | WS     | Receives control commands (see wire protocol below)  |

### Wire protocol (`app::controller::parse_cmd`)

WebSocket text frames:

- `"S"` → **Stop**.
- `"<VERB>:<SPEED>"` where `SPEED` is `0..=100` (defaults to `75` if missing/
  invalid) and `VERB` is one of:

  | VERB | `RemoteCmd`            | Motion             |
  |------|-----------------------|--------------------|
  | `F`  | `Drive(speed)`        | forward            |
  | `B`  | `Drive(-speed)`       | reverse            |
  | `L`  | `TurnLeft(speed)`     | turn left          |
  | `R`  | `TurnRight(speed)`    | turn right         |
  | `SL` | `SpinLeft(speed)`     | spin in place left |
  | `SR` | `SpinRight(speed)`    | spin in place right|

  Any unknown verb maps to `Stop`. `to_car_command` then converts the
  `RemoteCmd` into a `CarCommand` using `max_duty`.

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

All tests live in `src/app/command.rs` (unit tests, `#[cfg(test)]`).  
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

- **`CarCommand` is `Clone`, not `Copy`.** It carries a `String` `message`
  field, so it only derives `Clone`. (`MotorCommand`, `Direction`, `MotorPins`
  and `MotorId` are `Copy`.) Prefer passing `&CarCommand` in hot paths.

- **No shared timer ownership issue.** Multiple `LedcDriver` instances can be
  created from `&LedcTimerDriver` (the HAL accepts `impl Borrow<LedcTimerDriver<'d>>`).
  The timer lives on the stack in `main()` and outlives all channel drivers.

- **Long-lived bindings instead of `forget`.** `wifi`, `timer`, and the web
  server handle are owned by `main` and never dropped because `MotorService::run`
  diverges (`-> !`) and `main`'s scope never exits. Keep these bindings — do not
  let them be dropped.

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

