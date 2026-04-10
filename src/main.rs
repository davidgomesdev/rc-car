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

/// Hardware driver for four DC motors wired through two TB6612FNG chips.
///
/// Pin assignment (adjust in `main` to match your wiring):
///
/// | Motor       | IN1   | IN2   | PWM    | LEDC ch |
/// |-------------|-------|-------|--------|---------|
/// | Front-Right | GPIO6 | GPIO5 | GPIO4  | ch0     |
/// | Rear-Right  | GPIO7 | GPIO8 | GPIO16 | ch1     |
/// | Front-Left  | GPIO38| GPIO39| GPIO40 | ch2     |
/// | Rear-Left   | GPIO37| GPIO36| GPIO35 | ch3     |
#[cfg(target_os = "espidf")]
struct EspMotorController<'d> {
    // Front-right motor (chip 1 – channel A)
    fr_in1: PinDriver<'d, Output>,
    fr_in2: PinDriver<'d, Output>,
    fr_pwm: LedcDriver<'d>,
    // Rear-right motor (chip 1 – channel B)
    rr_in1: PinDriver<'d, Output>,
    rr_in2: PinDriver<'d, Output>,
    rr_pwm: LedcDriver<'d>,
    // Front-left motor (chip 2 – channel A)
    fl_in1: PinDriver<'d, Output>,
    fl_in2: PinDriver<'d, Output>,
    fl_pwm: LedcDriver<'d>,
    // Rear-left motor (chip 2 – channel B)
    rl_in1: PinDriver<'d, Output>,
    rl_in2: PinDriver<'d, Output>,
    rl_pwm: LedcDriver<'d>,
    max_duty: u32,
}

#[cfg(target_os = "espidf")]
impl<'d> EspMotorController<'d> {
    /// Apply a [`MotorCommand`] to a single motor's direction pins and PWM channel.
    fn drive_pins(
        in1: &mut PinDriver<'_, Output>,
        in2: &mut PinDriver<'_, Output>,
        pwm: &mut LedcDriver<'_>,
        cmd: MotorCommand,
    ) -> anyhow::Result<()> {
        if cmd.pins.in1_high { in1.set_high()? } else { in1.set_low()? }
        if cmd.pins.in2_high { in2.set_high()? } else { in2.set_low()? }
        pwm.set_duty(cmd.duty)?;
        Ok(())
    }

    /// Apply a [`CarCommand`] to all four motors simultaneously.
    fn apply(&mut self, cmd: &CarCommand) -> anyhow::Result<()> {
        Self::drive_pins(&mut self.fr_in1, &mut self.fr_in2, &mut self.fr_pwm, cmd.front_right)?;
        Self::drive_pins(&mut self.rr_in1, &mut self.rr_in2, &mut self.rr_pwm, cmd.rear_right)?;
        Self::drive_pins(&mut self.fl_in1, &mut self.fl_in2, &mut self.fl_pwm, cmd.front_left)?;
        Self::drive_pins(&mut self.rl_in1, &mut self.rl_in2, &mut self.rl_pwm, cmd.rear_left)?;
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

    // All four PWM channels share one timer at 25 kHz.
    let timer_cfg = TimerConfig::default().frequency(25_u32.kHz().into());
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_cfg)?;

    // Initialise the first PWM channel first so we can read max_duty.
    let fr_pwm = LedcDriver::new(peripherals.ledc.channel0, &timer, peripherals.pins.gpio4)?;
    let max_duty = fr_pwm.get_max_duty();

    let mut controller = EspMotorController {
        // Front-right: GPIO4 PWM, GPIO6 IN1, GPIO5 IN2
        fr_pwm,
        fr_in1: PinDriver::output(peripherals.pins.gpio6)?,
        fr_in2: PinDriver::output(peripherals.pins.gpio5)?,
        // Rear-right: GPIO16 PWM, GPIO7 IN1, GPIO8 IN2
        rr_pwm: LedcDriver::new(peripherals.ledc.channel1, &timer, peripherals.pins.gpio16)?,
        rr_in1: PinDriver::output(peripherals.pins.gpio7)?,
        rr_in2: PinDriver::output(peripherals.pins.gpio8)?,
        // Front-left: GPIO40 PWM, GPIO38 IN1, GPIO39 IN2
        fl_pwm: LedcDriver::new(peripherals.ledc.channel2, &timer, peripherals.pins.gpio40)?,
        fl_in1: PinDriver::output(peripherals.pins.gpio38)?,
        fl_in2: PinDriver::output(peripherals.pins.gpio39)?,
        // Rear-left: GPIO35 PWM, GPIO37 IN1, GPIO36 IN2
        rl_pwm: LedcDriver::new(peripherals.ledc.channel3, &timer, peripherals.pins.gpio35)?,
        rl_in1: PinDriver::output(peripherals.pins.gpio37)?,
        rl_in2: PinDriver::output(peripherals.pins.gpio36)?,
        max_duty,
    };

    // Safe state on boot.
    controller.stop()?;

    // Demo sequence – replace with your own control logic.
    loop {
        let d = controller.max_duty;
        let sequence: &[(&str, CarCommand)] = &[
            ("forward  50%",  CarCommand::drive(50, d)),
            ("stop",          CarCommand::stop(d)),
            ("reverse  50%",  CarCommand::drive(-50, d)),
            ("stop",          CarCommand::stop(d)),
            ("turn left  60%",  CarCommand::turn_left(60, d)),
            ("turn right 60%",  CarCommand::turn_right(60, d)),
            ("spin left  40%",  CarCommand::spin_left(40, d)),
            ("spin right 40%",  CarCommand::spin_right(40, d)),
            ("stop",          CarCommand::stop(d)),
        ];

        for (label, cmd) in sequence {
            log::info!("→ {label}");
            controller.apply(cmd)?;
            FreeRtos::delay_ms(1500);
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
