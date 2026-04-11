use embedded_svc::ipv4::{Mask, RouterConfiguration, Subnet};
use embedded_svc::wifi::Wifi;
use rc_car::CarCommand;
#[cfg(target_os = "espidf")]
use rc_car::MotorCommand;
use std::net::Ipv4Addr;

// ── ESP-IDF target ────────────────────────────────────────────────────────────

#[cfg(target_os = "espidf")]
use esp_idf_hal::gpio::{Output, PinDriver};
#[cfg(target_os = "espidf")]
use esp_idf_hal::ledc::LedcDriver;
use esp_idf_svc::ipv4;
use esp_idf_svc::netif::{EspNetif, NetifConfiguration, NetifStack};
use esp_idf_svc::wifi::WifiDriver;
use ipv4::Configuration;

/// Direction pins and PWM channel for a single DC motor.
#[cfg(target_os = "espidf")]
struct EspMotorDriver<'d> {
    in1: PinDriver<'d, Output>,
    in2: PinDriver<'d, Output>,
    pwm: LedcDriver<'d>,
}

/// Hardware driver for four DC motors wired through two TB6612FNG chips.
#[cfg(target_os = "espidf")]
struct EspMotorController<'d> {
    front_right: EspMotorDriver<'d>,
    rear_right: EspMotorDriver<'d>,
    front_left: EspMotorDriver<'d>,
    rear_left: EspMotorDriver<'d>,
    max_duty: u32,
}

#[cfg(target_os = "espidf")]
impl<'d> EspMotorController<'d> {
    /// Apply a [`MotorCommand`] to a single motor's direction pins and PWM channel.
    fn drive_pins(motor: &mut EspMotorDriver<'_>, cmd: MotorCommand) -> anyhow::Result<()> {
        if cmd.pins.in1_high {
            motor.in1.set_high()?
        } else {
            motor.in1.set_low()?
        }
        if cmd.pins.in2_high {
            motor.in2.set_high()?
        } else {
            motor.in2.set_low()?
        }
        motor.pwm.set_duty(cmd.duty)?;
        Ok(())
    }

    /// Apply a [`CarCommand`] to all four motors simultaneously.
    fn apply(&mut self, cmd: &CarCommand) -> anyhow::Result<()> {
        Self::drive_pins(&mut self.front_right, cmd.front_right)?;
        Self::drive_pins(&mut self.rear_right, cmd.rear_right)?;
        Self::drive_pins(&mut self.front_left, cmd.front_left)?;
        Self::drive_pins(&mut self.rear_left, cmd.rear_left)?;
        Ok(())
    }

    /// Convenience: stop every motor immediately.
    fn stop(&mut self) -> anyhow::Result<()> {
        self.apply(&CarCommand::stop(self.max_duty))
    }
}

// ── Remote command (received over WebSocket) ──────────────────────────────────

/// Compact command enum that is `Copy + Send` — safe to share across threads.
#[cfg(target_os = "espidf")]
#[derive(Copy, Clone, PartialEq, Eq)]
enum RemoteCmd {
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
#[cfg(target_os = "espidf")]
fn parse_cmd(s: &str) -> RemoteCmd {
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
#[cfg(target_os = "espidf")]
fn to_car_command(cmd: RemoteCmd, max_duty: u32) -> CarCommand {
    match cmd {
        RemoteCmd::Drive(s) => CarCommand::drive(s, max_duty),
        RemoteCmd::TurnLeft(s) => CarCommand::turn_left(s, max_duty),
        RemoteCmd::TurnRight(s) => CarCommand::turn_right(s, max_duty),
        RemoteCmd::SpinLeft(s) => CarCommand::spin_left(s, max_duty),
        RemoteCmd::SpinRight(s) => CarCommand::spin_right(s, max_duty),
        RemoteCmd::Stop => CarCommand::stop(max_duty),
    }
}

#[cfg(target_os = "espidf")]
static INDEX_HTML: &str = include_str!("controller.html");

// ── ESP-IDF main ──────────────────────────────────────────────────────────────

#[cfg(target_os = "espidf")]
fn main() -> anyhow::Result<()> {
    use embedded_svc::io::Write as _;
    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::ledc::LedcTimerDriver;
    use esp_idf_hal::ledc::config::TimerConfig;
    use esp_idf_hal::peripherals::Peripherals;
    use esp_idf_hal::units::*;
    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::http::Method;
    use esp_idf_svc::http::server::Configuration as ServerConfig;
    use esp_idf_svc::http::server::EspHttpServer;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;
    use esp_idf_svc::sys::EspError;
    use esp_idf_svc::wifi::{
        AccessPointConfiguration, AuthMethod, BlockingWifi, Configuration as WifiConfig, EspWifi,
    };
    use std::sync::{Arc, Mutex};

    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("RC Car booting…");

    let peripherals =
        Peripherals::take().map_err(|_| anyhow::anyhow!("Failed to take peripherals"))?;

    // ── Wi-Fi access point ────────────────────────────────────────────────────
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let netif = EspNetif::new_with_conf(&NetifConfiguration {
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
            WifiDriver::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
            EspNetif::new(NetifStack::Sta)?,
            #[cfg(esp_idf_esp_wifi_softap_support)]
            netif,
        )?,
        sys_loop,
    )?;

    wifi.set_configuration(&WifiConfig::AccessPoint(AccessPointConfiguration {
        ssid: "RC-CAR".try_into().unwrap(),
        auth_method: AuthMethod::WPA2Personal,
        password: "verysecure!".try_into().unwrap(),
        channel: 1,
        ..Default::default()
    }))?;
    wifi.start()?;

    wifi.wait_netif_up()?;

    let ip = wifi.wifi().ap_netif().get_ip_info()?.ip;
    log::info!("Wi-Fi AP up — SSID: RC-CAR  IP: {}", ip);

    // ── HTTP + WebSocket server ───────────────────────────────────────────────
    let server_cfg = ServerConfig {
        stack_size: 10240,
        ..Default::default()
    };
    let mut server = EspHttpServer::new(&server_cfg)?;

    // Serve the controller UI at /
    server.fn_handler("/", Method::Get, |req| {
        log::info!("User visited page");
        req.into_ok_response()?
            .write_all(INDEX_HTML.as_bytes())
            .map(|_| ())
    })?;

    // Shared state: WebSocket handler writes, main loop reads.
    // Only RemoteCmd (Copy) crosses the thread boundary — motor types stay on main.
    let shared: Arc<Mutex<RemoteCmd>> = Arc::new(Mutex::new(RemoteCmd::Stop));
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

    // Forget wifi so it is never dropped (lives as long as the firmware runs).
    core::mem::forget(wifi);

    // ── Motor controller ──────────────────────────────────────────────────────
    let timer_cfg = TimerConfig::default().frequency(25_u32.kHz().into());
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_cfg)?;

    let fr_pwm = LedcDriver::new(peripherals.ledc.channel0, &timer, peripherals.pins.gpio4)?;
    let max_duty = fr_pwm.get_max_duty();

    let mut controller = EspMotorController {
        front_right: EspMotorDriver {
            pwm: fr_pwm,
            in1: PinDriver::output(peripherals.pins.gpio6)?,
            in2: PinDriver::output(peripherals.pins.gpio5)?,
        },
        rear_right: EspMotorDriver {
            pwm: LedcDriver::new(peripherals.ledc.channel1, &timer, peripherals.pins.gpio16)?,
            in1: PinDriver::output(peripherals.pins.gpio15)?,
            in2: PinDriver::output(peripherals.pins.gpio7)?,
        },
        front_left: EspMotorDriver {
            pwm: LedcDriver::new(peripherals.ledc.channel2, &timer, peripherals.pins.gpio40)?,
            in1: PinDriver::output(peripherals.pins.gpio38)?,
            in2: PinDriver::output(peripherals.pins.gpio39)?,
        },
        rear_left: EspMotorDriver {
            pwm: LedcDriver::new(peripherals.ledc.channel3, &timer, peripherals.pins.gpio35)?,
            in1: PinDriver::output(peripherals.pins.gpio37)?,
            in2: PinDriver::output(peripherals.pins.gpio36)?,
        },
        max_duty,
    };

    controller.stop()?;
    log::info!("Motors ready. Open http://{} to control.", ip);

    // ── Main control loop (polls shared command every 50 ms) ─────────────────
    loop {
        let cmd = *shared.lock().unwrap();
        controller.apply(&to_car_command(cmd, max_duty))?;
        FreeRtos::delay_ms(50);
    }
}

// ── Host / simulation target ──────────────────────────────────────────────────

#[cfg(not(target_os = "espidf"))]
fn main() {
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
