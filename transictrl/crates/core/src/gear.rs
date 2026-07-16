//! Gear identification from shaft speeds.
//!
//! Ported from the `evaluateGear` / `gearFromRatio` logic in the original
//! `main/core.cpp`, but driven entirely by [`TransConfig`] so it works for any
//! gear count. Given the measured input/output speed ratio we find the gear
//! whose theoretical ratio is closest, provided it falls inside the
//! configured tolerance band. This is used both to confirm a shift completed
//! and to detect a slipping / mis-engaged transmission.

use crate::config::TransConfig;
use crate::math::absf;

/// Result of evaluating which gear the transmission is physically in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GearEval {
    /// Best-matching 1-based gear, if the ratio is within tolerance of one.
    pub gear: Option<u8>,
    /// Fractional slip relative to the matched gear's theoretical ratio
    /// (`measured/theoretical - 1`). `None` when no gear matched.
    pub slip: Option<f32>,
}

/// Identify the engaged gear from a measured input/output ratio.
///
/// Returns the gear whose theoretical ratio is nearest `measured_ratio`, but
/// only if the relative error is within `cfg.limits.ratio_tolerance`.
pub fn gear_from_ratio(cfg: &TransConfig, measured_ratio: f32) -> GearEval {
    if measured_ratio <= 0.0 || cfg.gear_count == 0 {
        return GearEval {
            gear: None,
            slip: None,
        };
    }

    let mut best_gear = 0u8;
    let mut best_err = f32::INFINITY;
    let mut best_slip = 0.0f32;

    for g in 1..=cfg.gear_count {
        let theo = cfg.gears[(g - 1) as usize].ratio;
        if theo <= 0.0 {
            continue;
        }
        let rel = measured_ratio / theo - 1.0;
        let err = absf(rel);
        if err < best_err {
            best_err = err;
            best_gear = g;
            best_slip = rel;
        }
    }

    if best_gear != 0 && best_err <= cfg.limits.ratio_tolerance {
        GearEval {
            gear: Some(best_gear),
            slip: Some(best_slip),
        }
    } else {
        GearEval {
            gear: None,
            slip: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::cfg_4l60e;

    #[test]
    fn exact_ratios_match_their_gear() {
        let cfg = cfg_4l60e();
        assert_eq!(gear_from_ratio(&cfg, 3.06).gear, Some(1));
        assert_eq!(gear_from_ratio(&cfg, 1.63).gear, Some(2));
        assert_eq!(gear_from_ratio(&cfg, 1.00).gear, Some(3));
        assert_eq!(gear_from_ratio(&cfg, 0.70).gear, Some(4));
    }

    #[test]
    fn within_tolerance_still_matches() {
        let cfg = cfg_4l60e();
        // 3rd gear is 1.00, tolerance 8% -> 1.05 should still be 3rd.
        let e = gear_from_ratio(&cfg, 1.05);
        assert_eq!(e.gear, Some(3));
        let slip = e.slip.unwrap();
        assert!((slip - 0.05).abs() < 1e-4);
    }

    #[test]
    fn far_from_any_gear_is_none() {
        let cfg = cfg_4l60e();
        // Between 1st (3.06) and 2nd (1.63): heavily slipping, no match.
        assert_eq!(gear_from_ratio(&cfg, 2.4).gear, None);
    }

    #[test]
    fn invalid_inputs() {
        let cfg = cfg_4l60e();
        assert_eq!(gear_from_ratio(&cfg, 0.0).gear, None);
        assert_eq!(gear_from_ratio(&cfg, -1.0).gear, None);
    }
}
