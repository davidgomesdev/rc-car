# Layered Module Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split the RC-car firmware into three clearly-bounded layers — `app` (platform-agnostic domain + control logic), `internal` (ESP-only hardware abstraction), and `api` (ESP-only WiFi / motors / web services) — so `lib.rs` contains only module declarations and `main.rs` is a thin orchestrator.

**Architecture:** Pure relocation refactor with no behavioural changes. The `app` layer is always compiled and host-testable; `internal` and `api` are `cfg(target_os = "espidf")`-gated because they depend on the ESP-IDF HAL. Data flow: `main` → `WifiService::connect` → `internal::gpio::build_controller` → `MotorService` (command bus + 50 ms control loop) ← `api::web` (HTTP/WS server writes commands into the bus).

**Tech Stack:** Rust (edition 2024), `esp-idf-hal` 0.46.2, `esp-idf-svc` 0.52.1, `embedded-svc` 0.29, `anyhow`. Host target `aarch64-apple-darwin`; firmware target `xtensa-esp32s3-espidf`.

---

## File Structure

| File | Responsibility |
|------|----------------|
| `src/lib.rs` | Module declarations ONLY |
| `src/main.rs` | ESP orchestration `main` + host-simulation `main` |
| `src/app/mod.rs` | `pub mod` for app submodules |
| `src/app/direction.rs` | `Direction`, `MotorPins` |
| `src/app/command.rs` | `MotorCommand`, `CarCommand`, `MotorId` + unit tests |
| `src/app/controller.rs` | `RemoteCmd`, `parse_cmd`, `to_car_command` |
| `src/internal/mod.rs` | `pub mod` for internal submodules |
| `src/internal/motor_driver.rs` | `EspMotorDriver`, `EspMotorController` |
| `src/internal/gpio.rs` | `build_controller` — pin/channel assignment |
| `src/api/mod.rs` | `pub mod` for api submodules |
| `src/api/wifi.rs` | `WifiService` (connect-or-AP, IP) |
| `src/api/motors.rs` | `MotorService` (command bus + control loop) |
| `src/api/web.rs` | HTTP/WS server (`/`, `/camerastatus`, `/ws`) |
| `AGENTS.md` | Updated architecture docs |

**Verification commands used throughout:**
- Host: `cargo test --target aarch64-apple-darwin` and `cargo build --target aarch64-apple-darwin`
- ESP: `make build-esp` (sources `~/export-esp.sh`; first run compiles ESP-IDF and may take 5–15+ minutes)

---

## Task 1: Create the `app` layer

**Files:**
- Create: `src/app/mod.rs`
- Create: `src/app/direction.rs`
- Create: `src/app/command.rs`
- Create: `src/app/controller.rs`
- Modify: `src/lib.rs` (replace entire contents)
- Modify: `src/main.rs` (top-of-file imports; remove inline `RemoteCmd`/`parse_cmd`/`to_car_command`; host `main` import)

- [ ] **Step 1: Create `src/app/direction.rs`**

```rust
//! App models: motor direction and the two direction pins it drives.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Reverse,
    Stop,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorPins {
    pub in1_high: bool,
    pub in2_high: bool,
}
```

- [ ] **Step 2: Create `src/app/command.rs`** (domain types + the existing unit tests, with the `Direction`/`MotorPins` import adjusted)

```rust
//! App models: per-motor and whole-car movement commands.

use crate::app::direction::{Direction, MotorPins};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorCommand {
    pub direction: Direction,
    pub pins: MotorPins,
    pub duty: u32,
}

impl MotorCommand {
    /// Converts a signed speed percentage (-100..=100) into direction + PWM duty.
    pub fn from_percent(speed_percent: i8, max_duty: u32) -> Self {
        let bounded = speed_percent.clamp(-100, 100);
        let magnitude = bounded.unsigned_abs() as u32;
        let duty = (magnitude * max_duty) / 100;

        if bounded > 0 {
            Self {
                direction: Direction::Forward,
                pins: MotorPins {
                    in1_high: true,
                    in2_high: false,
                },
                duty,
            }
        } else if bounded < 0 {
            Self {
                direction: Direction::Reverse,
                pins: MotorPins {
                    in1_high: false,
                    in2_high: true,
                },
                duty,
            }
        } else {
            Self {
                direction: Direction::Stop,
                pins: MotorPins {
                    in1_high: false,
                    in2_high: false,
                },
                duty: 0,
            }
        }
    }
}

/// Identifies one of the four motors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorId {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

/// Combined command for all four motors simultaneously.
#[derive(Debug, Clone)]
pub struct CarCommand {
    pub front_left: MotorCommand,
    pub front_right: MotorCommand,
    pub rear_left: MotorCommand,
    pub rear_right: MotorCommand,
    pub message: String,
}

impl CarCommand {
    /// Drive all four motors at the same speed (-100..=100).
    pub fn drive(speed: i8, max_duty: u32) -> Self {
        let cmd = MotorCommand::from_percent(speed, max_duty);
        let message = if speed > 0 {
            format!("forward {speed}%")
        } else if speed < 0 {
            format!("reverse {speed}%")
        } else {
            "stop".to_string()
        };
        Self {
            front_left: cmd,
            front_right: cmd,
            rear_left: cmd,
            rear_right: cmd,
            message,
        }
    }

    /// Tank-style differential steering: each side can have a different speed (-100..=100).
    pub fn steer(left_speed: i8, right_speed: i8, max_duty: u32) -> Self {
        Self {
            front_left: MotorCommand::from_percent(left_speed, max_duty),
            rear_left: MotorCommand::from_percent(left_speed, max_duty),
            front_right: MotorCommand::from_percent(right_speed, max_duty),
            rear_right: MotorCommand::from_percent(right_speed, max_duty),
            message: "steer".to_string(),
        }
    }

    /// Turn left: right motors at `speed`, left motors stopped.
    pub fn turn_left(speed: i8, max_duty: u32) -> Self {
        let mut cmd = Self::steer(0, speed, max_duty);
        cmd.message = format!("turn left {speed}%");
        cmd
    }

    /// Turn right: left motors at `speed`, right motors stopped.
    pub fn turn_right(speed: i8, max_duty: u32) -> Self {
        let mut cmd = Self::steer(speed, 0, max_duty);
        cmd.message = format!("turn right {speed}%");
        cmd
    }

    /// Spin left in place: left motors reverse, right motors forward.
    pub fn spin_left(speed: i8, max_duty: u32) -> Self {
        let s = speed.clamp(0, 100);
        let mut cmd = Self::steer(-s, s, max_duty);
        cmd.message = format!("spin left {speed}%");
        cmd
    }

    /// Spin right in place: left motors forward, right motors reverse.
    pub fn spin_right(speed: i8, max_duty: u32) -> Self {
        let s = speed.clamp(0, 100);
        let mut cmd = Self::steer(s, -s, max_duty);
        cmd.message = format!("spin right {speed}%");
        cmd
    }

    /// Stop all motors.
    pub fn stop(max_duty: u32) -> Self {
        Self::drive(0, max_duty)
    }

    /// Set a single motor by ID, leaving the others unchanged.
    pub fn with_motor(mut self, id: MotorId, speed: i8, max_duty: u32) -> Self {
        let cmd = MotorCommand::from_percent(speed, max_duty);
        match id {
            MotorId::FrontLeft => self.front_left = cmd,
            MotorId::FrontRight => self.front_right = cmd,
            MotorId::RearLeft => self.rear_left = cmd,
            MotorId::RearRight => self.rear_right = cmd,
        }
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::direction::Direction;

    #[test]
    fn zero_speed_stops_motor() {
        let cmd = MotorCommand::from_percent(0, 1023);
        assert_eq!(cmd.direction, Direction::Stop);
        assert_eq!(cmd.duty, 0);
        assert!(!cmd.pins.in1_high);
        assert!(!cmd.pins.in2_high);
    }

    #[test]
    fn positive_speed_maps_to_forward_pwm() {
        let cmd = MotorCommand::from_percent(50, 1000);
        assert_eq!(cmd.direction, Direction::Forward);
        assert_eq!(cmd.duty, 500);
        assert!(cmd.pins.in1_high);
        assert!(!cmd.pins.in2_high);
    }

    #[test]
    fn negative_speed_maps_to_reverse_pwm() {
        let cmd = MotorCommand::from_percent(-25, 800);
        assert_eq!(cmd.direction, Direction::Reverse);
        assert_eq!(cmd.duty, 200);
        assert!(!cmd.pins.in1_high);
        assert!(cmd.pins.in2_high);
    }

    #[test]
    fn values_are_clamped_to_safe_range() {
        let high = MotorCommand::from_percent(120, 2000);
        let low = MotorCommand::from_percent(-120, 2000);

        assert_eq!(high.duty, 2000);
        assert_eq!(low.duty, 2000);
    }

    #[test]
    fn drive_sets_all_motors_same_speed() {
        let cmd = CarCommand::drive(50, 1000);
        assert_eq!(cmd.front_left.direction, Direction::Forward);
        assert_eq!(cmd.front_right.direction, Direction::Forward);
        assert_eq!(cmd.rear_left.direction, Direction::Forward);
        assert_eq!(cmd.rear_right.direction, Direction::Forward);
        assert_eq!(cmd.front_left.duty, 500);
        assert_eq!(cmd.rear_right.duty, 500);
    }

    #[test]
    fn drive_reverse_sets_all_motors_reverse() {
        let cmd = CarCommand::drive(-75, 1000);
        for motor in [
            cmd.front_left,
            cmd.front_right,
            cmd.rear_left,
            cmd.rear_right,
        ] {
            assert_eq!(motor.direction, Direction::Reverse);
            assert_eq!(motor.duty, 750);
        }
    }

    #[test]
    fn stop_all_halts_every_motor() {
        let cmd = CarCommand::stop(1000);
        for motor in [
            cmd.front_left,
            cmd.front_right,
            cmd.rear_left,
            cmd.rear_right,
        ] {
            assert_eq!(motor.direction, Direction::Stop);
            assert_eq!(motor.duty, 0);
        }
    }

    #[test]
    fn steer_applies_independent_speeds() {
        let cmd = CarCommand::steer(-30, 70, 1000);
        assert_eq!(cmd.front_left.direction, Direction::Reverse);
        assert_eq!(cmd.rear_left.direction, Direction::Reverse);
        assert_eq!(cmd.front_right.direction, Direction::Forward);
        assert_eq!(cmd.rear_right.direction, Direction::Forward);
        assert_eq!(cmd.front_left.duty, 300);
        assert_eq!(cmd.front_right.duty, 700);
    }

    #[test]
    fn spin_left_reverses_left_side() {
        let cmd = CarCommand::spin_left(50, 1000);
        assert_eq!(cmd.front_left.direction, Direction::Reverse);
        assert_eq!(cmd.rear_left.direction, Direction::Reverse);
        assert_eq!(cmd.front_right.direction, Direction::Forward);
        assert_eq!(cmd.rear_right.direction, Direction::Forward);
    }

    #[test]
    fn spin_right_reverses_right_side() {
        let cmd = CarCommand::spin_right(50, 1000);
        assert_eq!(cmd.front_left.direction, Direction::Forward);
        assert_eq!(cmd.rear_left.direction, Direction::Forward);
        assert_eq!(cmd.front_right.direction, Direction::Reverse);
        assert_eq!(cmd.rear_right.direction, Direction::Reverse);
    }

    #[test]
    fn with_motor_overrides_single_motor() {
        let base = CarCommand::stop(1000);
        let cmd = base.with_motor(MotorId::FrontLeft, 80, 1000);
        assert_eq!(cmd.front_left.direction, Direction::Forward);
        assert_eq!(cmd.front_right.direction, Direction::Stop);
        assert_eq!(cmd.rear_left.direction, Direction::Stop);
    }
}
```

- [ ] **Step 3: Create `src/app/controller.rs`** (moved verbatim from `main.rs`, with the `cfg` gate removed and `CarCommand` imported)

```rust
//! App control logic: the remote-control verb protocol and its mapping to car
//! movement commands. Platform-agnostic so it can be unit-tested on the host.

use crate::app::command::CarCommand;

/// Compact command enum that is `Copy + Send` — safe to share across threads.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum RemoteCmd {
    Drive(i8),
    TurnLeft(i8),
    TurnRight(i8),
    SpinLeft(i8),
    SpinRight(i8),
    Stop,
}

/// Parse a text frame from the browser.
///
/// Protocol: `"S"` → stop; `"<VERB>:<SPEED>"` otherwise.
/// VERBs: `F` forward, `B` backward, `L` turn-left, `R` turn-right,
///        `SL` spin-left, `SR` spin-right.
/// SPEED is an unsigned integer 0–100 (the sign is encoded in the verb).
pub fn parse_cmd(s: &str) -> RemoteCmd {
    let s = s.trim_matches(|c: char| c.is_ascii_control() || c.is_whitespace());
    if s == "S" {
        return RemoteCmd::Stop;
    }
    let mut it = s.splitn(2, ':');
    let verb = it.next().unwrap_or("S");
    let spd: i8 = it.next().and_then(|v| v.trim().parse().ok()).unwrap_or(75);
    match verb {
        "F" => RemoteCmd::Drive(spd),
        "B" => RemoteCmd::Drive(-spd),
        "L" => RemoteCmd::TurnLeft(spd),
        "R" => RemoteCmd::TurnRight(spd),
        "SL" => RemoteCmd::SpinLeft(spd),
        "SR" => RemoteCmd::SpinRight(spd),
        _ => RemoteCmd::Stop,
    }
}

/// Map a `RemoteCmd` to the corresponding `CarCommand`.
pub fn to_car_command(cmd: RemoteCmd, max_duty: u32) -> CarCommand {
    match cmd {
        RemoteCmd::Drive(s) => CarCommand::drive(s, max_duty),
        RemoteCmd::TurnLeft(s) => CarCommand::turn_left(s, max_duty),
        RemoteCmd::TurnRight(s) => CarCommand::turn_right(s, max_duty),
        RemoteCmd::SpinLeft(s) => CarCommand::spin_left(s, max_duty),
        RemoteCmd::SpinRight(s) => CarCommand::spin_right(s, max_duty),
        RemoteCmd::Stop => CarCommand::stop(max_duty),
    }
}
```

- [ ] **Step 4: Create `src/app/mod.rs`**

```rust
pub mod command;
pub mod controller;
pub mod direction;
```

- [ ] **Step 5: Replace the entire contents of `src/lib.rs`** with module declarations only

```rust
pub mod app;

#[cfg(target_os = "espidf")]
pub mod internal;

#[cfg(target_os = "espidf")]
pub mod api;
```

> Note: `internal` and `api` modules are created in Tasks 2 and 3. On the host target these `#[cfg]` lines compile to nothing, so the host build stays green now.

- [ ] **Step 6: Update `src/main.rs` top-of-file imports.** Delete these two lines at the very top of the file:

```rust
use rc_car::CarCommand;
#[cfg(target_os = "espidf")]
use rc_car::MotorCommand;
```

Replace them with nothing (each `main` and the ESP helpers will import what they need). Then, in the ESP import block (the run of `#[cfg(target_os = "espidf")] use ...;` lines), add these two lines:

```rust
#[cfg(target_os = "espidf")]
use rc_car::app::command::{CarCommand, MotorCommand};
#[cfg(target_os = "espidf")]
use rc_car::app::controller::{parse_cmd, to_car_command, RemoteCmd};
```

- [ ] **Step 7: Remove the now-duplicated definitions from `src/main.rs`.** Delete the inline `RemoteCmd` enum (the `#[cfg(target_os = "espidf")] #[derive(Copy, Clone, PartialEq, Eq)] enum RemoteCmd { ... }` block), the inline `parse_cmd` function, and the inline `to_car_command` function. These now live in `app::controller`. Leave everything else in `main.rs` (the `EspMotorDriver`/`EspMotorController` structs, `INDEX_HTML`, WiFi helpers, `run_web_server`, and both `main` functions) untouched for now.

- [ ] **Step 8: Update the host `main` in `src/main.rs`.** Inside `#[cfg(not(target_os = "espidf"))] fn main()`, add as the first line:

```rust
    use rc_car::app::command::CarCommand;
```

(The body already uses `CarCommand` unqualified; this import replaces the deleted top-level `use rc_car::CarCommand;`.)

- [ ] **Step 9: Run host tests to verify they pass**

Run: `cargo test --target aarch64-apple-darwin`
Expected: PASS — `test result: ok. 11 passed` (the relocated tests in `app::command`).

- [ ] **Step 10: Run host build to verify it compiles**

Run: `cargo build --target aarch64-apple-darwin`
Expected: build succeeds (warnings about unused ESP-only items are acceptable).

- [ ] **Step 11: Commit**

```bash
git add src/lib.rs src/app src/main.rs
git commit -m "refactor: extract platform-agnostic app layer (direction, command, controller)

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 2: Create the `internal` layer (hardware abstraction)

**Files:**
- Create: `src/internal/mod.rs`
- Create: `src/internal/motor_driver.rs`
- Create: `src/internal/gpio.rs`
- Modify: `src/main.rs` (remove inline `EspMotorDriver`/`EspMotorController`; build controller via `internal::gpio`)

- [ ] **Step 1: Create `src/internal/motor_driver.rs`** (the ESP hardware driver; `drive_pins` becomes `EspMotorDriver::apply`, and constructors/accessors are added so the sibling `gpio` and `api` modules can build and use it)

```rust
//! Internal model: ESP-IDF hardware driver for four DC motors wired through two
//! TB6612FNG chips. One [`EspMotorDriver`] owns a single motor's two direction
//! pins and PWM channel.

use anyhow::Result;
use esp_idf_hal::gpio::{Output, PinDriver};
use esp_idf_hal::ledc::LedcDriver;

use crate::app::command::{CarCommand, MotorCommand};

/// Direction pins and PWM channel for a single DC motor.
pub struct EspMotorDriver<'d> {
    in1: PinDriver<'d, Output>,
    in2: PinDriver<'d, Output>,
    pwm: LedcDriver<'d>,
}

impl<'d> EspMotorDriver<'d> {
    pub fn new(
        in1: PinDriver<'d, Output>,
        in2: PinDriver<'d, Output>,
        pwm: LedcDriver<'d>,
    ) -> Self {
        Self { in1, in2, pwm }
    }

    /// Maximum PWM duty for this channel (depends on the LEDC timer resolution).
    pub fn max_duty(&self) -> u32 {
        self.pwm.get_max_duty()
    }

    /// Apply a [`MotorCommand`] to this motor's direction pins and PWM channel.
    fn apply(&mut self, cmd: MotorCommand) -> Result<()> {
        if cmd.pins.in1_high {
            self.in1.set_high()?
        } else {
            self.in1.set_low()?
        }
        if cmd.pins.in2_high {
            self.in2.set_high()?
        } else {
            self.in2.set_low()?
        }
        self.pwm.set_duty(cmd.duty)?;
        Ok(())
    }
}

/// Hardware driver for four DC motors.
pub struct EspMotorController<'d> {
    front_right: EspMotorDriver<'d>,
    rear_right: EspMotorDriver<'d>,
    front_left: EspMotorDriver<'d>,
    rear_left: EspMotorDriver<'d>,
    max_duty: u32,
}

impl<'d> EspMotorController<'d> {
    pub fn new(
        front_right: EspMotorDriver<'d>,
        rear_right: EspMotorDriver<'d>,
        front_left: EspMotorDriver<'d>,
        rear_left: EspMotorDriver<'d>,
        max_duty: u32,
    ) -> Self {
        Self {
            front_right,
            rear_right,
            front_left,
            rear_left,
            max_duty,
        }
    }

    /// Maximum PWM duty (shared across all four channels).
    pub fn max_duty(&self) -> u32 {
        self.max_duty
    }

    /// Apply a [`CarCommand`] to all four motors simultaneously.
    pub fn apply(&mut self, cmd: &CarCommand) -> Result<()> {
        self.front_right.apply(cmd.front_right)?;
        self.rear_right.apply(cmd.rear_right)?;
        self.front_left.apply(cmd.front_left)?;
        self.rear_left.apply(cmd.rear_left)?;
        Ok(())
    }

    /// Convenience: stop every motor immediately.
    pub fn stop(&mut self) -> Result<()> {
        self.apply(&CarCommand::stop(self.max_duty))
    }
}
```

- [ ] **Step 2: Create `src/internal/gpio.rs`** (the pin/channel assignment that previously lived inline in `main`)

```rust
//! Internal model: GPIO and PWM-channel assignment for the four motors.

use anyhow::Result;
use esp_idf_hal::gpio::{Pins, PinDriver};
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver, CHANNEL0, CHANNEL1, CHANNEL2, CHANNEL3};

use crate::internal::motor_driver::{EspMotorController, EspMotorDriver};

/// Build the four-motor controller from the LEDC channels and GPIO pins.
///
/// Default pin map (two TB6612FNG chips):
///
/// | Motor       | IN1    | IN2    | PWM    | LEDC channel |
/// |-------------|--------|--------|--------|--------------|
/// | Front-Right | GPIO6  | GPIO5  | GPIO4  | ch0          |
/// | Rear-Right  | GPIO15 | GPIO7  | GPIO16 | ch1          |
/// | Front-Left  | GPIO38 | GPIO39 | GPIO40 | ch2          |
/// | Rear-Left   | GPIO37 | GPIO36 | GPIO35 | ch3          |
///
/// `timer` must outlive the returned controller — the PWM channels borrow it at
/// construction. The controller is returned in a stopped state.
pub fn build_controller<'d>(
    timer: &'d LedcTimerDriver<'d>,
    channel0: CHANNEL0<'static>,
    channel1: CHANNEL1<'static>,
    channel2: CHANNEL2<'static>,
    channel3: CHANNEL3<'static>,
    pins: Pins,
) -> Result<EspMotorController<'d>> {
    let front_right = EspMotorDriver::new(
        PinDriver::output(pins.gpio6)?,
        PinDriver::output(pins.gpio5)?,
        LedcDriver::new(channel0, timer, pins.gpio4)?,
    );
    let max_duty = front_right.max_duty();

    let rear_right = EspMotorDriver::new(
        PinDriver::output(pins.gpio15)?,
        PinDriver::output(pins.gpio7)?,
        LedcDriver::new(channel1, timer, pins.gpio16)?,
    );
    let front_left = EspMotorDriver::new(
        PinDriver::output(pins.gpio38)?,
        PinDriver::output(pins.gpio39)?,
        LedcDriver::new(channel2, timer, pins.gpio40)?,
    );
    let rear_left = EspMotorDriver::new(
        PinDriver::output(pins.gpio37)?,
        PinDriver::output(pins.gpio36)?,
        LedcDriver::new(channel3, timer, pins.gpio35)?,
    );

    let mut controller =
        EspMotorController::new(front_right, rear_right, front_left, rear_left, max_duty);
    controller.stop()?;
    Ok(controller)
}
```

- [ ] **Step 3: Create `src/internal/mod.rs`**

```rust
pub mod gpio;
pub mod motor_driver;
```

- [ ] **Step 4: Add the `internal` module declaration to `src/lib.rs`.** It is already present from Task 1 Step 5:

```rust
#[cfg(target_os = "espidf")]
pub mod internal;
```

Confirm this line exists; no change needed if it does.

- [ ] **Step 5: Remove the inline hardware structs from `src/main.rs`.** Delete the `EspMotorDriver` struct, the `EspMotorController` struct, and the entire `impl<'d> EspMotorController<'d> { ... }` block (the `drive_pins`, `apply`, `stop` methods). Also delete the now-unused ESP import lines for `Output`, `PinDriver`, and `LedcDriver` at the top IF they are no longer referenced elsewhere in `main.rs` — `main.rs` still needs `LedcTimerDriver` (see next step) but no longer needs `PinDriver`/`Output`/`LedcDriver` directly. Remove:

```rust
#[cfg(target_os = "espidf")]
use esp_idf_hal::gpio::{Output, PinDriver};
#[cfg(target_os = "espidf")]
use esp_idf_hal::ledc::LedcDriver;
```

- [ ] **Step 6: Replace the controller-construction block in the ESP `main`.** Find the block that creates `timer`, `fr_pwm`, `max_duty`, and the `let mut controller = EspMotorController { ... };` literal, plus the following `controller.stop()?;`. Replace that whole block with:

```rust
    let timer_cfg = TimerConfig::default().frequency(25_u32.kHz().into());
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_cfg)?;

    let mut controller = rc_car::internal::gpio::build_controller(
        &timer,
        peripherals.ledc.channel0,
        peripherals.ledc.channel1,
        peripherals.ledc.channel2,
        peripherals.ledc.channel3,
        peripherals.pins,
    )?;
    let max_duty = controller.max_duty();
```

`build_controller` already leaves the controller stopped, so the standalone `controller.stop()?;` line is removed. The existing `log::info!("Motors ready. Open http://{ip} to control.");` and the `loop { ... }` that calls `to_car_command(cmd, max_duty)` and `controller.apply(...)` stay as-is.

- [ ] **Step 7: Run host build (sanity — `internal` is cfg-gated out on host, but the workspace must still compile)**

Run: `cargo build --target aarch64-apple-darwin`
Expected: succeeds.

- [ ] **Step 8: Run the ESP build to verify the firmware compiles**

Run: `make build-esp`
Expected: `Firmware binary: target/xtensa-esp32s3-espidf/release/rc-car`. (First run is slow; allow 5–15+ minutes.)

- [ ] **Step 9: Commit**

```bash
git add src/internal src/main.rs
git commit -m "refactor: extract internal hardware layer (gpio, motor_driver)

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 3: Create the `api` layer and slim `main.rs`

**Files:**
- Create: `src/api/mod.rs`
- Create: `src/api/wifi.rs`
- Create: `src/api/motors.rs`
- Create: `src/api/web.rs`
- Modify: `src/main.rs` (replace the entire file with the slim orchestrator + host sim)

- [ ] **Step 1: Create `src/api/wifi.rs`** (WiFi service: STA+AP netif config, connect-or-AP fallback, on-demand IP; replaces the inline WiFi setup and the `core::mem::forget` hack)

```rust
//! API: Wi-Fi service. Brings up the station + access-point netifs, tries to
//! join the configured network, and falls back to access-point mode. Owns the
//! Wi-Fi driver for the program's lifetime.

use anyhow::Result;
use std::net::Ipv4Addr;

use embedded_svc::ipv4::{Mask, RouterConfiguration, Subnet};
use embedded_svc::wifi::Wifi;
use esp_idf_hal::modem::Modem;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::ipv4;
use esp_idf_svc::ipv4::Configuration;
use esp_idf_svc::netif::{EspNetif, NetifConfiguration};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{
    AccessPointConfiguration, AuthMethod, BlockingWifi, ClientConfiguration,
    Configuration as WifiConfig, EspWifi, WifiDriver,
};

/// Owns the Wi-Fi driver and stays alive as long as the firmware runs.
pub struct WifiService {
    wifi: BlockingWifi<EspWifi<'static>>,
}

impl WifiService {
    /// Bring up Wi-Fi: try the configured client network, falling back to AP.
    pub fn connect(
        modem: Modem<'static>,
        sys_loop: EspSystemEventLoop,
        nvs: EspDefaultNvsPartition,
    ) -> Result<Self> {
        let sta_netif = EspNetif::new_with_conf(&NetifConfiguration {
            ip_configuration: Some(ipv4::Configuration::Client(
                ipv4::ClientConfiguration::DHCP(ipv4::DHCPClientSettings {
                    hostname: Some("rc-car".try_into().unwrap()),
                }),
            )),
            ..NetifConfiguration::wifi_default_client()
        })?;
        let ap_netif = EspNetif::new_with_conf(&NetifConfiguration {
            ip_configuration: Some(Configuration::Router(RouterConfiguration {
                subnet: Subnet {
                    gateway: Ipv4Addr::from_octets([192u8, 168u8, 1u8, 1u8]),
                    mask: Mask(24),
                },
                dhcp_enabled: true,
                dns: None,
                secondary_dns: None,
            })),
            ..NetifConfiguration::wifi_default_router()
        })?;

        let mut wifi = BlockingWifi::wrap(
            EspWifi::wrap_all(
                WifiDriver::new(modem, sys_loop.clone(), Some(nvs))?,
                sta_netif,
                #[cfg(esp_idf_esp_wifi_softap_support)]
                ap_netif,
            )?,
            sys_loop,
        )?;

        if let Err(e) = Self::connect_to_client(&mut wifi) {
            log::error!("Failed to connect to client WiFi: {e}");
            Self::start_access_point(&mut wifi)?;
        }

        Ok(Self { wifi })
    }

    /// Resolve the assigned IP: station address if up, otherwise the AP address.
    pub fn ip(&self) -> Result<Ipv4Addr> {
        let ip = if self.wifi.wifi().sta_netif().is_up()? {
            self.wifi.wifi().sta_netif().get_ip_info()?.ip
        } else {
            self.wifi.wifi().ap_netif().get_ip_info()?.ip
        };
        Ok(ip)
    }

    fn connect_to_client(wifi: &mut BlockingWifi<EspWifi<'static>>) -> Result<()> {
        let ssid = env!("CLIENT_WIFI_SSID");
        let password = env!("CLIENT_WIFI_PASSWORD");

        log::info!("Attempting to connect to WiFi SSID: {}", ssid);
        wifi.set_configuration(&WifiConfig::Client(ClientConfiguration {
            ssid: ssid.try_into().unwrap(),
            password: password.try_into().unwrap(),
            auth_method: AuthMethod::WPA2Personal,
            ..Default::default()
        }))?;
        wifi.start()?;
        wifi.connect()?;
        Ok(())
    }

    fn start_access_point(wifi: &mut BlockingWifi<EspWifi<'static>>) -> Result<()> {
        let ssid = env!("AP_WIFI_SSID");
        let password = env!("AP_WIFI_PASSWORD");

        log::info!("Switching to access point mode");
        wifi.stop()?;
        wifi.set_configuration(&WifiConfig::AccessPoint(AccessPointConfiguration {
            ssid: ssid.try_into()?,
            auth_method: AuthMethod::WPA2Personal,
            password: password.try_into()?,
            channel: 1,
            ..Default::default()
        }))?;
        wifi.start()?;
        wifi.wait_netif_up()?;

        let ip = wifi.wifi().ap_netif().get_ip_info()?.ip;
        log::info!("Wi-Fi AP up — SSID: {ssid}  IP: {ip}");
        Ok(())
    }
}
```

- [ ] **Step 2: Create `src/api/motors.rs`** (motor service: owns the controller, exposes the shared command bus, and runs the 50 ms control loop)

```rust
//! API: motor service. Owns the hardware controller, exposes a shared command
//! bus for the web layer to write into, and runs the control loop that applies
//! the latest command to the motors.

use std::sync::{Arc, Mutex};

use esp_idf_hal::delay::FreeRtos;

use crate::app::controller::{to_car_command, RemoteCmd};
use crate::internal::motor_driver::EspMotorController;

/// Shared command bus: the web handler writes, the control loop reads.
pub type CommandBus = Arc<Mutex<RemoteCmd>>;

pub struct MotorService<'d> {
    controller: EspMotorController<'d>,
    command: CommandBus,
}

impl<'d> MotorService<'d> {
    pub fn new(controller: EspMotorController<'d>) -> Self {
        Self {
            controller,
            command: Arc::new(Mutex::new(RemoteCmd::Stop)),
        }
    }

    /// A clone of the command bus to hand to the web layer.
    pub fn command_bus(&self) -> CommandBus {
        Arc::clone(&self.command)
    }

    /// Run the control loop forever: poll the bus every 50 ms and drive motors.
    pub fn run(mut self) -> ! {
        let max_duty = self.controller.max_duty();
        loop {
            let cmd = *self.command.lock().unwrap();
            if let Err(e) = self.controller.apply(&to_car_command(cmd, max_duty)) {
                log::error!("Motor apply error: {e}");
            }
            FreeRtos::delay_ms(50);
        }
    }
}
```

- [ ] **Step 3: Create `src/api/web.rs`** (HTTP/WS server, moved from `main.rs::run_web_server`; `parse_cmd` now comes from `app::controller` and the HTML include path is `../controller.html`)

```rust
//! API: HTTP + WebSocket server. Serves the controller UI, proxies camera
//! status, and turns inbound WebSocket text frames into [`RemoteCmd`]s on the
//! shared command bus.

use anyhow::Result;
use std::sync::Arc;

use embedded_svc::io::Write as _;
use esp_idf_svc::http::client::{Configuration as HttpClientConfig, EspHttpConnection};
use esp_idf_svc::http::server::{Configuration as ServerConfig, EspHttpServer};
use esp_idf_svc::http::Method;
use esp_idf_svc::sys::EspError;

use crate::api::motors::CommandBus;
use crate::app::controller::{parse_cmd, RemoteCmd};

static INDEX_HTML: &str = include_str!("../controller.html");

/// Register all HTTP/WebSocket handlers and return the running server handle.
///
/// The handle **must be kept alive** for the duration of the program; dropping
/// it tears down the server. `shared` is the command bus that the WS handler
/// writes into and the motor control loop reads from.
pub fn start(shared: CommandBus) -> Result<EspHttpServer<'static>> {
    let server_cfg = ServerConfig {
        stack_size: 10240,
        ..Default::default()
    };
    let mut server = EspHttpServer::new(&server_cfg)?;

    // Serve the controller UI at /.
    server.fn_handler("/", Method::Get, |req| {
        log::info!("User visited page");
        req.into_ok_response()?
            .write_all(INDEX_HTML.as_bytes())
            .map(|_| ())
    })?;

    // Proxy: fetch camera status server-side to avoid CORS issues in the browser.
    // GET /camerastatus?ip=<camera-ip> → 200 if camera is up, 502 otherwise.
    server.fn_handler("/camerastatus", Method::Get, |req| {
        let uri = req.uri();
        let ip = uri
            .split_once("ip=")
            .map(|(_, v)| v.split('&').next().unwrap_or(v).trim())
            .unwrap_or("");

        let ok = if ip.is_empty() {
            false
        } else {
            let url = format!("http://{}:8080/videostatus", ip);
            let cfg = HttpClientConfig::default();
            EspHttpConnection::new(&cfg)
                .and_then(|mut conn| {
                    conn.initiate_request(Method::Get, &url, &[])?;
                    conn.initiate_response()?;
                    Ok(conn.status() == 200)
                })
                .unwrap_or(false)
        };

        let mut resp = req.into_response(
            if ok { 200 } else { 502 },
            None,
            &[("Content-Type", "text/plain")],
        )?;
        resp.write_all(if ok { b"ok" } else { b"error" }).map(|_| ())
    })?;

    // WebSocket handler: parse incoming text frames and push to shared state.
    // Only RemoteCmd (Copy) crosses the thread boundary — motor types stay on main.
    let shared_ws = Arc::clone(&shared);
    server.ws_handler("/ws", None, move |ws| {
        if ws.is_new() {
            log::info!("WS: client connected (session {})", ws.session());
            return Ok(());
        }
        if ws.is_closed() {
            log::info!("WS: client disconnected — stopping motors");
            *shared_ws.lock().unwrap() = RemoteCmd::Stop;
            return Ok(());
        }

        // ESP-IDF WS requires two recv calls: first with empty buf to get length,
        // then with a sized buf to read the payload.
        let (_frame_type, len) = ws.recv(&mut [])?;
        if len == 0 || len > 32 {
            return Ok(());
        }
        let mut buf = [0u8; 32];
        ws.recv(&mut buf[..len])?;

        let s = std::str::from_utf8(&buf[..len])
            .unwrap_or("")
            .trim_matches(|c: char| c.is_ascii_control() || c.is_whitespace());

        log::info!("WS rx: '{s}'");
        *shared_ws.lock().unwrap() = parse_cmd(s);

        Ok::<(), EspError>(())
    })?;

    Ok(server)
}
```

- [ ] **Step 4: Create `src/api/mod.rs`**

```rust
pub mod motors;
pub mod web;
pub mod wifi;
```

- [ ] **Step 5: Confirm `src/lib.rs` declares `api`.** From Task 1 Step 5 it already contains:

```rust
#[cfg(target_os = "espidf")]
pub mod api;
```

No change needed if present.

- [ ] **Step 6: Replace the entire contents of `src/main.rs`** with the slim orchestrator plus the host simulation:

```rust
// ── ESP-IDF target ────────────────────────────────────────────────────────────

#[cfg(target_os = "espidf")]
fn main() -> anyhow::Result<()> {
    use esp_idf_hal::ledc::config::TimerConfig;
    use esp_idf_hal::ledc::LedcTimerDriver;
    use esp_idf_hal::peripherals::Peripherals;
    use esp_idf_hal::units::*;
    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;

    use rc_car::api::motors::MotorService;
    use rc_car::api::web;
    use rc_car::api::wifi::WifiService;
    use rc_car::internal::gpio;

    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("RC Car booting…");

    let peripherals =
        Peripherals::take().map_err(|_| anyhow::anyhow!("Failed to take peripherals"))?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    // ── Wi-Fi: connect, or fall back to access point ─────────────────────────
    // Kept alive for the whole program (replaces the old core::mem::forget hack).
    let wifi = WifiService::connect(peripherals.modem, sys_loop, nvs)?;

    // ── Motors ───────────────────────────────────────────────────────────────
    // `timer` must outlive the controller; it lives here on main's stack.
    let timer_cfg = TimerConfig::default().frequency(25_u32.kHz().into());
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_cfg)?;
    let controller = gpio::build_controller(
        &timer,
        peripherals.ledc.channel0,
        peripherals.ledc.channel1,
        peripherals.ledc.channel2,
        peripherals.ledc.channel3,
        peripherals.pins,
    )?;
    let motors = MotorService::new(controller);

    // ── Web server (must stay alive; dropping it tears down all handlers) ─────
    let _server = web::start(motors.command_bus())?;

    let ip = wifi.ip()?;
    log::info!("Motors ready. Open http://{ip} to control.");

    // Run the control loop forever. `wifi` and `_server` stay alive because this
    // never returns.
    motors.run()
}

// ── Host / simulation target ──────────────────────────────────────────────────

#[cfg(not(target_os = "espidf"))]
fn main() {
    use rc_car::app::command::CarCommand;

    let max_duty = 1023_u32;
    println!("Host simulation mode – build for xtensa-esp32s3-espidf to run on the board.\n");

    let demos: &[(&str, CarCommand)] = &[
        ("drive(50%)", CarCommand::drive(50, max_duty)),
        ("drive(-50%)", CarCommand::drive(-50, max_duty)),
        ("stop", CarCommand::stop(max_duty)),
        ("turn_left(60%)", CarCommand::turn_left(60, max_duty)),
        ("turn_right(60%)", CarCommand::turn_right(60, max_duty)),
        ("spin_left(40%)", CarCommand::spin_left(40, max_duty)),
        ("spin_right(40%)", CarCommand::spin_right(40, max_duty)),
    ];

    for (name, cmd) in demos {
        println!("{name}:");
        println!("  FL: {:?}", cmd.front_left);
        println!("  FR: {:?}", cmd.front_right);
        println!("  RL: {:?}", cmd.rear_left);
        println!("  RR: {:?}", cmd.rear_right);
        println!();
    }
}
```

- [ ] **Step 7: Run host tests and build**

Run: `cargo test --target aarch64-apple-darwin && cargo build --target aarch64-apple-darwin`
Expected: `11 passed`; build succeeds.

- [ ] **Step 8: Run the ESP build to verify the firmware still compiles**

Run: `make build-esp`
Expected: `Firmware binary: target/xtensa-esp32s3-espidf/release/rc-car`.

- [ ] **Step 9: Commit**

```bash
git add src/api src/main.rs
git commit -m "refactor: extract api layer (wifi, motors, web); slim main to orchestrator

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Task 4: Update `AGENTS.md`

**Files:**
- Modify: `AGENTS.md`

- [ ] **Step 1: Rewrite the repository-layout section** (section 2) to show the new `src/` tree:

```
rc-car/
├── Cargo.toml
├── Cargo.lock
├── build.rs
├── Makefile
├── README.md
├── AGENTS.md
└── src/
    ├── lib.rs              # module declarations only
    ├── main.rs             # ESP orchestrator + host simulation
    ├── controller.html     # web UI (served by api::web)
    ├── app/                # app models — platform-agnostic, host-testable
    │   ├── direction.rs    # Direction, MotorPins
    │   ├── command.rs      # MotorCommand, CarCommand, MotorId (+ tests)
    │   └── controller.rs   # RemoteCmd, parse_cmd, to_car_command
    ├── internal/           # internal models — ESP-only hardware abstraction
    │   ├── gpio.rs         # build_controller: pin/channel assignment
    │   └── motor_driver.rs # EspMotorDriver, EspMotorController
    └── api/                # API layer — ESP-only services
        ├── wifi.rs         # WifiService
        ├── motors.rs       # MotorService (command bus + control loop)
        └── web.rs          # HTTP/WebSocket server
```

- [ ] **Step 2: Update the architecture description.** Replace the parts of sections 5–6 that describe the public API and hardware driver so they reflect the three layers:
  - `app` (always compiled): `direction`, `command` (`MotorCommand`/`CarCommand`/`MotorId`), and `controller` (`RemoteCmd`, `parse_cmd`, `to_car_command`). All host-testable; the 11 unit tests live in `app::command`.
  - `internal` (ESP-only): `motor_driver` (`EspMotorDriver`, `EspMotorController` with `new`/`apply`/`stop`/`max_duty`) and `gpio` (`build_controller`, which owns the default pin map and returns a stopped controller).
  - `api` (ESP-only): `wifi::WifiService` (`connect`, `ip`), `motors::MotorService` (`new`, `command_bus`, `run`), `web::start` (HTTP/WS handlers).

- [ ] **Step 3: Add a "Networking & control flow" subsection** documenting the runtime data flow and the wire protocol (previously undocumented):
  - Boot → `WifiService::connect` (client SSID from `CLIENT_WIFI_*`, AP fallback `AP_WIFI_*`) → `gpio::build_controller` → `MotorService` → `web::start(command_bus)` → `MotorService::run` polls every 50 ms.
  - WebSocket `/ws` protocol: `"S"` = stop; `"<VERB>:<SPEED>"` where VERB ∈ {`F`,`B`,`L`,`R`,`SL`,`SR`} and SPEED is 0–100 (default 75).
  - `/` serves `controller.html`; `/camerastatus?ip=` proxies the camera's `:8080/videostatus`.

- [ ] **Step 4: Fix the default pin-mapping table** (section 6) so Rear-Right IN1 reads **GPIO15** (the code uses `gpio15`, not `gpio7`/`gpio8` as the old doc stated).

- [ ] **Step 5: Update the dependencies section** to add `log` 0.4 and `embedded-svc` 0.29 to the ESP-IDF runtime dependency table (they are in `Cargo.toml` but missing from the doc).

- [ ] **Step 6: Commit**

```bash
git add AGENTS.md
git commit -m "docs: update AGENTS.md for layered architecture and networking

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Self-Review Notes

- **Spec coverage:** app/internal/api layers (Tasks 1–3), `lib.rs` mods-only (Task 1 Step 5), thin `main.rs` (Task 3 Step 6), `mem::forget` removed (Task 3 Step 1/6), tests relocated (Task 1 Step 2), docs (Task 4) — all covered.
- **Type consistency:** `build_controller(&timer, channel0..3, pins)` signature matches its call site in `main`; `MotorService::new(controller)` / `command_bus()` / `run()` match `web::start(CommandBus)`; `CommandBus = Arc<Mutex<RemoteCmd>>` used consistently in `motors.rs` and `web.rs`; `EspMotorController::{new, apply, stop, max_duty}` and `EspMotorDriver::{new, max_duty, apply}` are defined in `motor_driver.rs` and used by `gpio.rs`/`motors.rs`.
- **No behavioural change:** WiFi sequence, web handlers, protocol parsing, 50 ms loop, and 25 kHz PWM are byte-for-byte preserved; only their location and the `forget`→owned-binding lifetime management changed.
