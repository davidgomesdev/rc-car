//! API: motor service. Owns the hardware controller, exposes a shared command
//! bus for the web layer to write into, and runs the control loop that applies
//! the latest command to the motors.

use std::sync::{Arc, Mutex};

use esp_idf_hal::delay::FreeRtos;

use crate::app::controller::{to_car_command, RemoteCmd};
use crate::internal::motor_driver::EspMotorController;

/// Shared command bus: the web handler writes, the control loop reads.
pub type CommandBus = Arc<Mutex<RemoteCmd>>;

pub struct MotorService<'d> {
    controller: EspMotorController<'d>,
    command: CommandBus,
}

impl<'d> MotorService<'d> {
    pub fn new(controller: EspMotorController<'d>) -> Self {
        Self {
            controller,
            command: Arc::new(Mutex::new(RemoteCmd::Stop)),
        }
    }

    /// A clone of the command bus to hand to the web layer.
    pub fn command_bus(&self) -> CommandBus {
        Arc::clone(&self.command)
    }

    /// Run the control loop forever: poll the bus every 50 ms and drive motors.
    pub fn run(mut self) -> ! {
        let max_duty = self.controller.max_duty();
        loop {
            let cmd = *self.command.lock().unwrap();
            if let Err(e) = self.controller.apply(&to_car_command(cmd, max_duty)) {
                log::error!("Motor apply error: {e}");
            }
            FreeRtos::delay_ms(50);
        }
    }
}
