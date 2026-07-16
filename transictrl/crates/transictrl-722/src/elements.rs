// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! 722.6 shift element / solenoid mapping.
//!
//! The 722.6 conductor plate uses three shift solenoids (Y3, Y4, Y5) plus the
//! MPC/SPC pressure regulators and the TCC lockup clutch. This module maps the
//! abstract `transictrl_core::ElementState` to the physical solenoids for each
//! upshift and downshift, matching the original `core.cpp` logic.

use transictrl_core::{ElementState, GearProfile};

/// Logical shift solenoids on the 722.6 conductor plate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Solenoid {
    Y3,
    Y4,
    Y5,
}

/// Static element states for each 722.6 gear.
///
/// Derived from the original conductor plate truth table. Each gear engages a
/// specific combination of shift elements.
pub const GEAR_PROFILES: [GearProfile; 5] = [
    GearProfile {
        elements: [
            ElementState::Applied,
            ElementState::Released,
            ElementState::Released,
            ElementState::Released,
            ElementState::Released,
            ElementState::Released,
        ],
        ratio: 3.59,
        lockup_allowed: false,
    },
    GearProfile {
        elements: [
            ElementState::Released,
            ElementState::Applied,
            ElementState::Released,
            ElementState::Released,
            ElementState::Released,
            ElementState::Released,
        ],
        ratio: 2.19,
        lockup_allowed: false,
    },
    GearProfile {
        elements: [
            ElementState::Released,
            ElementState::Released,
            ElementState::Applied,
            ElementState::Released,
            ElementState::Released,
            ElementState::Released,
        ],
        ratio: 1.41,
        lockup_allowed: true,
    },
    GearProfile {
        elements: [
            ElementState::Released,
            ElementState::Released,
            ElementState::Released,
            ElementState::Applied,
            ElementState::Released,
            ElementState::Released,
        ],
        ratio: 1.00,
        lockup_allowed: true,
    },
    GearProfile {
        elements: [
            ElementState::Released,
            ElementState::Released,
            ElementState::Released,
            ElementState::Released,
            ElementState::Applied,
            ElementState::Released,
        ],
        ratio: 0.83,
        lockup_allowed: true,
    },
];

/// Which solenoid must be pulsed to perform a given upshift/downshift.
/// Returns `None` if no solenoid change is required (e.g. 1->1).
pub fn solenoid_for_shift(from: u8, to: u8) -> Option<Solenoid> {
    match (from, to) {
        // Upshifts
        (1, 2) => Some(Solenoid::Y3),
        (2, 3) => Some(Solenoid::Y4),
        (3, 4) => Some(Solenoid::Y5),
        (4, 5) => Some(Solenoid::Y3),
        // Downshifts
        (2, 1) => Some(Solenoid::Y3),
        (3, 2) => Some(Solenoid::Y4),
        (4, 3) => Some(Solenoid::Y5),
        (5, 4) => Some(Solenoid::Y3),
        _ => None,
    }
}

/// Gear ratio for a confirmed 722.6 gear.
pub fn ratio_from_gear(gear: u8) -> f32 {
    GEAR_PROFILES
        .get(gear.saturating_sub(1) as usize)
        .map(|p| p.ratio)
        .unwrap_or(0.0)
}

/// Try to identify a gear from a measured shaft ratio.
pub fn gear_from_ratio(input: f32) -> Option<u8> {
    if input < 3.82 && input > 3.46 {
        Some(1)
    } else if input < 2.62 && input > 2.26 {
        Some(2)
    } else if input < 1.60 && input > 1.35 {
        Some(3)
    } else if input < 1.20 && input > 0.92 {
        Some(4)
    } else if input < 0.90 && input > 0.60 {
        Some(5)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solenoid_selection_matches_original() {
        assert_eq!(solenoid_for_shift(1, 2), Some(Solenoid::Y3));
        assert_eq!(solenoid_for_shift(2, 3), Some(Solenoid::Y4));
        assert_eq!(solenoid_for_shift(3, 4), Some(Solenoid::Y5));
        assert_eq!(solenoid_for_shift(4, 5), Some(Solenoid::Y3));
        assert_eq!(solenoid_for_shift(2, 1), Some(Solenoid::Y3));
        assert_eq!(solenoid_for_shift(5, 4), Some(Solenoid::Y3));
    }

    #[test]
    fn gear_from_ratio_windows() {
        assert_eq!(gear_from_ratio(3.59), Some(1));
        assert_eq!(gear_from_ratio(2.44), Some(2));
        assert_eq!(gear_from_ratio(1.41), Some(3));
        assert_eq!(gear_from_ratio(1.00), Some(4));
        assert_eq!(gear_from_ratio(0.83), Some(5));
        assert_eq!(gear_from_ratio(5.0), None);
    }
}
