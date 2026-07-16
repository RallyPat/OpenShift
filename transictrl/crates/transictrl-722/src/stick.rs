// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! W202 gearstick decoding, ported from `input.cpp`.
//!
//! The W202 electronic selector provides either four digital lines (white,
//! blue, green, yellow) or a single resistive analog output. Both modes are
//! supported here.

/// High-level selector position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelectorPos {
    Park,
    Reverse,
    Neutral,
    Drive,
    Three,
    Two,
    One,
    Unknown,
}

/// Digital W202 line states (active-high / active-low depends on wiring).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DigitalStick {
    pub white: bool,
    pub blue: bool,
    pub green: bool,
    pub yellow: bool,
    pub auto_switch: bool,
}

impl DigitalStick {
    /// Decode the digital W202 selector lines.
    ///
    /// Truth table taken directly from `pollstick()` in `input.cpp`.
    pub fn decode(&self) -> SelectorPos {
        match (self.white, self.blue, self.green, self.yellow) {
            (true, true, true, false) => SelectorPos::Park,
            (false, true, true, true) => SelectorPos::Reverse,
            (true, false, true, true) => SelectorPos::Neutral,
            (false, false, true, false) => SelectorPos::Drive,
            (false, false, false, true) => SelectorPos::Three,
            (false, true, false, false) => SelectorPos::Two,
            (true, false, false, false) => SelectorPos::One,
            _ => SelectorPos::Unknown,
        }
    }

    /// Whether the auto/manual toggle switch is on.
    pub fn manual_mode(&self) -> bool {
        self.auto_switch
    }
}

/// Resistive stick ADC count.
pub struct ResistiveStick(pub u16);

impl ResistiveStick {
    /// Decode the resistive selector output.
    pub fn decode(&self) -> SelectorPos {
        match self.0 {
            450..=750 => SelectorPos::Park,
            300..=399 => SelectorPos::Reverse,
            200..=299 => SelectorPos::Neutral,
            100..=199 => SelectorPos::Drive,
            _ => SelectorPos::Unknown,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_park() {
        let stick = DigitalStick {
            white: true,
            blue: true,
            green: true,
            yellow: false,
            auto_switch: false,
        };
        assert_eq!(stick.decode(), SelectorPos::Park);
    }

    #[test]
    fn decode_drive() {
        let stick = DigitalStick {
            white: false,
            blue: false,
            green: true,
            yellow: false,
            auto_switch: false,
        };
        assert_eq!(stick.decode(), SelectorPos::Drive);
    }
}
