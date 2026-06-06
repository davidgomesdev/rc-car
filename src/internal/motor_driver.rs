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
