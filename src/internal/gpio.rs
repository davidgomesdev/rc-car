//! Internal model: GPIO and PWM-channel assignment for the four motors.

use anyhow::Result;
use esp_idf_hal::gpio::{PinDriver, Pins};
use esp_idf_hal::ledc::{
    CHANNEL0, CHANNEL1, CHANNEL2, CHANNEL3, LedcDriver, LedcTimerDriver, LowSpeed,
};

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
    timer: &'d LedcTimerDriver<'d, LowSpeed>,
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
