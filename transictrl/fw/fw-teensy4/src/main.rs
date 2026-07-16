// SPDX-License-Identifier: GPL-3.0-only

//! Teensy 4.x firmware skeleton for OpenShift.
//!
//! This crate is a placeholder. The final implementation will wire the
//! transictrl-722 controller to the Teensy 4.1 pins using `embassy` and the
//! `teensy4-bsp` HAL.

#![no_std]
#![no_main]

#[cfg(target_arch = "arm")]
mod hal_impl;

#[cfg(target_arch = "arm")]
use panic_halt as _;

#[cfg(target_arch = "arm")]
#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    // TODO(port): initialize Teensy 4.1 clocks, pins, ADC, PWM, CAN.
    // TODO(port): instantiate Controller722 and run the control loop.
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
    }
}

#[cfg(not(target_arch = "arm"))]
fn main() {
    // Host placeholder so the crate parses on x86_64 during workspace checks.
}
