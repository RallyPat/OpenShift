//! Shared fixtures for unit tests across the crate.

use crate::config::{ElementState, TransConfig};

/// Build a representative GM 4L60E configuration used throughout the tests.
///
/// Ratios are nominal 4L60E values and the SS-A/SS-B pattern follows the
/// transmission's documented shift-solenoid truth table.
pub(crate) fn cfg_4l60e() -> TransConfig {
    use ElementState::{Applied as A, Released as R};
    let mut cfg = TransConfig::empty();
    cfg.gear_count = 4;
    cfg.element_count = 2; // SS-A, SS-B
    cfg.final_drive = 3.42;
    let ratios = [3.06, 1.63, 1.00, 0.70];
    let pattern = [[A, A], [R, A], [R, R], [A, R]];
    for i in 0..4 {
        cfg.gears[i].ratio = ratios[i];
        cfg.gears[i].elements[0] = pattern[i][0];
        cfg.gears[i].elements[1] = pattern[i][1];
        cfg.gears[i].lockup_allowed = i >= 2; // lockup in 3rd/4th
    }
    cfg
}
