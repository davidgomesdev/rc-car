use rc_car::CarCommand;
#[cfg(target_os = "espidf")]
use rc_car::MotorCommand;
#[cfg(target_os = "espidf")]
use log;

// ── ESP-IDF target ────────────────────────────────────────────────────────────

#[cfg(target_os = "espidf")]
use esp_idf_hal::gpio::{Output, PinDriver};
#[cfg(target_os = "espidf")]
use esp_idf_hal::ledc::LedcDriver;

/// Direction pins and PWM channel for a single DC motor.
#[cfg(target_os = "espidf")]
struct EspMotorDriver<'d> {
    in1: PinDriver<'d, Output>,
    in2: PinDriver<'d, Output>,
    pwm: LedcDriver<'d>,
}

/// Hardware driver for four DC motors wired through two TB6612FNG chips.
///
/// Pin assignment (adjust in `main` to match your wiring):
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
        if cmd.pins.in1_high { motor.in1.set_high()? } else { motor.in1.set_low()? }
        if cmd.pins.in2_high { motor.in2.set_high()? } else { motor.in2.set_low()? }
        motor.pwm.set_duty(cmd.duty)?;
        Ok(())
    }

    /// Apply a [`CarCommand`] to all four motors simultaneously.
    fn apply(&mut self, cmd: &CarCommand) -> anyhow::Result<()> {
        log::info!("→ {}", cmd.message);
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

#[cfg(target_os = "espidf")]
fn main() -> anyhow::Result<()> {
    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::ledc::config::TimerConfig;
    use esp_idf_hal::ledc::LedcTimerDriver;
    use esp_idf_hal::peripherals::Peripherals;
    use esp_idf_hal::units::*;

    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("ESP32-S3 Motor Controller booting...");

    let peripherals = Peripherals::take()
        .map_err(|_| anyhow::anyhow!("Failed to take peripherals"))?;

    let timer_cfg = TimerConfig::default().frequency(25_u32.kHz().into());
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_cfg)?;

    // Initialize the first PWM channel first so we can read max_duty.
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

    // Safe state on boot.
    controller.stop()?;

    loop {
        let d = controller.max_duty;
        let sequence: &[CarCommand] = &[
            CarCommand::drive(100, d),
            CarCommand::stop(d),
            CarCommand::turn_left(50, d),
            CarCommand::stop(d),
            CarCommand::turn_right(50, d),
            CarCommand::stop(d),
        ];

        for cmd in sequence {
            controller.apply(cmd)?;
            FreeRtos::delay_ms(1000);
        }
    }
}

// ── Host / simulation target ──────────────────────────────────────────────────

#[cfg(not(target_os = "espidf"))]
fn main() {
    let max_duty = 1023_u32;
    println!("Host simulation mode – build for xtensa-esp32s3-espidf to run on the board.\n");

    let demos: &[(&str, CarCommand)] = &[
        ("drive(50%)",       CarCommand::drive(50, max_duty)),
        ("drive(-50%)",      CarCommand::drive(-50, max_duty)),
        ("stop",             CarCommand::stop(max_duty)),
        ("turn_left(60%)",   CarCommand::turn_left(60, max_duty)),
        ("turn_right(60%)",  CarCommand::turn_right(60, max_duty)),
        ("spin_left(40%)",   CarCommand::spin_left(40, max_duty)),
        ("spin_right(40%)",  CarCommand::spin_right(40, max_duty)),
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
