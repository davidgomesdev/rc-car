# RC-Car: Layered Module Refactor — Design

**Date:** 2026-06-06
**Status:** Approved

## Problem

All firmware logic currently lives in two files:

- `src/lib.rs` — domain types (`Direction`, `MotorPins`, `MotorCommand`,
  `CarCommand`, `MotorId`) plus their tests.
- `src/main.rs` (~420 lines) — mixes the ESP-IDF hardware driver, WiFi/netif
  setup, the HTTP/WebSocket server, the wire-protocol parser, the
  `RemoteCmd` → `CarCommand` mapping, and the host simulation entry point.

`main.rs` has grown into a grab-bag with no separation of concerns. Protocol
parsing and command mapping (`parse_cmd`, `to_car_command`) are
platform-agnostic logic that is currently untestable because it is buried in
the `cfg(target_os = "espidf")` binary. GPIO pin numbers are hard-coded inline
in `main`.

## Goals

1. Separate concerns into three layers, each a Rust module:
   - **app models** — platform-agnostic domain + control logic (host-testable).
   - **internal models** — ESP-only hardware abstraction (GPIO, motor driver).
   - **api** — ESP-only service + network layer, split by concern (WiFi,
     motors, web).
2. `lib.rs` contains **nothing but module declarations**.
3. `main.rs` becomes a thin orchestrator (plus the host simulation `main`).
4. Update `AGENTS.md` to match the new architecture.

Non-goals: no behavioural changes, no new features, no new unit tests beyond
relocating the existing ones, no changes to `controller.html` or the wire
protocol.

## Target structure

```
src/
├── lib.rs              # ONLY `pub mod` declarations
├── main.rs             # thin: ESP orchestration + host simulation
├── controller.html
├── app/                # app models — platform-agnostic, host-testable
│   ├── mod.rs
│   ├── direction.rs    # Direction, MotorPins
│   ├── command.rs      # MotorCommand, CarCommand, MotorId
│   └── controller.rs   # RemoteCmd + parse_cmd + RemoteCmd→CarCommand mapping
├── internal/           # internal models — ESP-only hardware abstraction
│   ├── mod.rs
│   ├── gpio.rs         # pin/channel assignment for the four motors
│   └── motor_driver.rs # EspMotorDriver, EspMotorController
└── api/                # API layer — ESP-only
    ├── mod.rs
    ├── wifi.rs         # WifiService: connect-or-AP, exposes assigned IP
    ├── motors.rs       # MotorService: shared command bus + control loop
    └── web.rs          # HTTP/WS server (serves HTML, /camerastatus, /ws)
```

### `lib.rs`

```rust
pub mod app;

#[cfg(target_os = "espidf")]
pub mod internal;

#[cfg(target_os = "espidf")]
pub mod api;
```

The `app` layer is always compiled (it is what the host simulation and the
existing tests use). `internal` and `api` are ESP-only because they depend on
the ESP-IDF HAL.

## Layer responsibilities

### `app` (platform-agnostic)

- `app::direction` — `Direction`, `MotorPins`.
- `app::command` — `MotorCommand` (with `from_percent`), `CarCommand`
  (`drive`, `steer`, `turn_*`, `spin_*`, `stop`, `with_motor`), `MotorId`.
- `app::controller` — `RemoteCmd` enum, `parse_cmd(&str) -> RemoteCmd`, and the
  `RemoteCmd` → `CarCommand` mapping (currently `to_car_command`). Moving these
  out of the ESP binary makes them compile and test on the host. `RemoteCmd`
  loses its `cfg` gate and stays `Copy` so it can still cross the WS thread
  boundary.

Existing unit tests move alongside the types they cover (into
`app::command` / `app::direction`).

### `internal` (ESP-only hardware abstraction)

- `internal::gpio` — the GPIO/PWM-channel assignment for the four motors. Pin
  numbers move here out of `main`, e.g. a helper that builds the four motor
  drivers from `Peripherals` pins + LEDC channels, keeping the default pin map
  (FR: 6/5/4·ch0, RR: 15/7/16·ch1, FL: 38/39/40·ch2, RL: 37/36/35·ch3) in one
  place.
- `internal::motor_driver` — `EspMotorDriver` (in1/in2/pwm for one motor),
  `EspMotorController` (four drivers + `max_duty`), and its `drive_pins`,
  `apply`, `stop` methods. Unchanged logic, relocated.

### `api` (ESP-only service + network layer)

- `api::wifi` — `WifiService`: encapsulates the STA + AP netif configuration,
  `connect_to_wifi`, `start_access_point` fallback, and exposes the assigned
  IP. Owns the `BlockingWifi`/`EspWifi` and stays alive for the program's
  lifetime, replacing the current `core::mem::forget(wifi)` hack with a binding
  kept alive in `main`.
- `api::motors` — `MotorService`: owns the `EspMotorController`, holds the
  shared command bus (`Arc<Mutex<RemoteCmd>>`), hands a clone of the bus to the
  web layer, and provides the control loop (`run`) that polls the bus every
  50 ms and applies it to the hardware via `app::controller` mapping.
- `api::web` — the HTTP/WebSocket server. Registers the `/` (UI),
  `/camerastatus` (camera proxy), and `/ws` (command stream) handlers and
  returns the live `EspHttpServer`. Takes the shared command bus and writes
  parsed `RemoteCmd`s into it. `controller.html` is included via
  `include_str!("../controller.html")`.

## Data flow (ESP target)

```
main()
  ├─ link_patches / logger
  ├─ Peripherals::take()
  ├─ WifiService::connect(modem, sys_loop, nvs)   → keeps WiFi alive, returns IP
  ├─ MotorService::new(ledc, pins)                → builds EspMotorController
  │     └─ command_bus(): Arc<Mutex<RemoteCmd>>   (clone shared with web)
  ├─ web::start(bus)                              → EspHttpServer (kept alive)
  └─ motor_service.run()                          → loop { read bus; apply; 50ms }
```

The WS handler (in `api::web`) writes `RemoteCmd`s into the bus; the control
loop (in `api::motors`) reads them, maps to `CarCommand` via `app::controller`,
and drives `internal` hardware. Only `RemoteCmd` (Copy) crosses the thread
boundary, exactly as today.

### Host target

`main.rs`'s `cfg(not(target_os = "espidf"))` entry point stays a thin demo that
prints `CarCommand` variants using only the `app` layer. No hardware needed.

## Error handling

Unchanged: `anyhow::Result` propagation in the ESP paths; WiFi client failure
falls back to AP mode; motor `apply` errors are logged and the loop continues.

## Testing

Relocate the existing 11 unit tests into the `app` submodules; they continue to
run on the host target (`cargo test --target <host-triple>`). No new tests are
added in this refactor. Verify both `cargo test` and (where the toolchain is
available) the ESP build still compile.

## Documentation

Rewrite `AGENTS.md` sections 2 (layout), 5–6 (API / hardware driver), and add
coverage of the WiFi service, the web/WS server, and the wire protocol to
reflect the new module boundaries.
