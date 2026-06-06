//! App models: motor direction and the two direction pins it drives.

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
