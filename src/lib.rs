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
