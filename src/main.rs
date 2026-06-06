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
    // `wifi` is kept alive for the whole program by this binding (the control
    // loop below never returns), so the Wi-Fi driver is never dropped.
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
