//! App control logic: the remote-control verb protocol and its mapping to car
//! movement commands. Platform-agnostic so it can be unit-tested on the host.

use crate::app::command::CarCommand;

/// Compact command enum that is `Copy + Send` — safe to share across threads.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum RemoteCmd {
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
/// SPEED is an integer −100..=100. For `L`/`R` the sign selects the arc
/// direction (positive = forward arc, negative = backward arc); the other
/// verbs use an unsigned 0–100 speed with the direction encoded in the verb.
pub fn parse_cmd(s: &str) -> RemoteCmd {
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
pub fn to_car_command(cmd: RemoteCmd, max_duty: u32) -> CarCommand {
    match cmd {
        RemoteCmd::Drive(s) => CarCommand::drive(s, max_duty),
        RemoteCmd::TurnLeft(s) => CarCommand::turn_left(s, max_duty),
        RemoteCmd::TurnRight(s) => CarCommand::turn_right(s, max_duty),
        RemoteCmd::SpinLeft(s) => CarCommand::spin_left(s, max_duty),
        RemoteCmd::SpinRight(s) => CarCommand::spin_right(s, max_duty),
        RemoteCmd::Stop => CarCommand::stop(max_duty),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_signed_arc_speed() {
        // Back-left is sent as "L:-75" by the UI.
        assert_eq!(parse_cmd("L:-75"), RemoteCmd::TurnLeft(-75));
        assert_eq!(parse_cmd("R:-75"), RemoteCmd::TurnRight(-75));
        // Forward arc and spin keep working.
        assert_eq!(parse_cmd("L:75"), RemoteCmd::TurnLeft(75));
        assert_eq!(parse_cmd("SL:60"), RemoteCmd::SpinLeft(60));
        assert_eq!(parse_cmd("S"), RemoteCmd::Stop);
    }
}
