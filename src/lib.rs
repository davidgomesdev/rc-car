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
}

