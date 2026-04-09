use rc_car::MotorCommand;

#[cfg(target_os = "espidf")]
fn main() -> anyhow::Result<()> {
    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::gpio::PinDriver;
    use esp_idf_hal::ledc::config::TimerConfig;
    use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver};
    use esp_idf_hal::peripherals::Peripherals;
    use esp_idf_hal::prelude::*;

    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().ok_or_else(|| anyhow::anyhow!("Failed to take peripherals"))?;

    // Keyestudio KS0066 (TB6612FNG) wiring for channel A:
    // PWMA (PWM) -> GPIO4, AIN1 -> GPIO5, AIN2 -> GPIO6, STBY -> HIGH
    let timer_cfg = TimerConfig::default().frequency(25.kHz().into());
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_cfg)?;
    let mut pwm = LedcDriver::new(peripherals.ledc.channel0, &timer, peripherals.pins.gpio4)?;
    let mut in1 = PinDriver::output(peripherals.pins.gpio5)?;
    let mut in2 = PinDriver::output(peripherals.pins.gpio6)?;

    // Keep the motor stopped on boot.
    in1.set_low()?;
    in2.set_low()?;
    pwm.set_duty(0)?;

    let max_duty = pwm.get_max_duty();

    loop {
        for speed in [-40, 0, 40, 80, 0, -40, -80, 0] {
            apply_motor_command(&mut in1, &mut in2, &mut pwm, MotorCommand::from_percent(speed, max_duty))?;
            FreeRtos::delay_ms(1200);
        }
    }
}

#[cfg(target_os = "espidf")]
fn apply_motor_command(
    in1: &mut esp_idf_hal::gpio::PinDriver<'_, esp_idf_hal::gpio::Gpio5, esp_idf_hal::gpio::Output>,
    in2: &mut esp_idf_hal::gpio::PinDriver<'_, esp_idf_hal::gpio::Gpio6, esp_idf_hal::gpio::Output>,
    pwm: &mut esp_idf_hal::ledc::LedcDriver<'_>,
    command: MotorCommand,
) -> anyhow::Result<()> {
    if command.pins.in1_high {
        in1.set_high()?;
    } else {
        in1.set_low()?;
    }

    if command.pins.in2_high {
        in2.set_high()?;
    } else {
        in2.set_low()?;
    }

    pwm.set_duty(command.duty)?;
    Ok(())
}

#[cfg(not(target_os = "espidf"))]
fn main() {
    let max_duty = 1023;
    println!("Host simulation mode. Build for xtensa-esp32s3-espidf to run on board.");
    for speed in [-80, -30, 0, 30, 80] {
        let cmd = MotorCommand::from_percent(speed, max_duty);
        println!("speed={speed:>4}% -> {:?}", cmd);
    }
}
