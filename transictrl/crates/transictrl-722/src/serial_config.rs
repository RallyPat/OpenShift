// SPDX-License-Identifier: GPL-3.0-only
// Portions derived from 7226ctrl (c) Markus Kovero et al., GPL-3.0

//! Serial command-line configuration, ported from `main/serial_config.cpp`.
//!
//! The original firmware accepts space-separated commands over USB serial to
//! read and write feature flags, calibration values, and gear-speed tables.
//! This module parses those commands into a structured enum so the firmware
//! can apply them without string handling in the hot path.

use heapless::String;

/// Parsed serial command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SerialCommand {
    /// Read or write a feature flag: `f <n> [0|1]`.
    Feature { index: u8, value: Option<bool> },
    /// Read or write an integer config: `c <id> [value]`.
    ConfigInt { id: u8, value: Option<i32> },
    /// Read or write a float config: `cf <id> [value]`.
    ConfigFloat { id: u8, value: Option<f32> },
    /// Read or write an upshift speed: `u <gear> [km/h]`.
    UpShiftSpeed { gear: u8, value: Option<u8> },
    /// Read or write a downshift speed: `d <gear> [km/h]`.
    DownShiftSpeed { gear: u8, value: Option<u8> },
    /// Request a dump of all configuration.
    Dump,
    /// Unknown / malformed command.
    Unknown,
}

/// Parse one line of serial input.
pub fn parse(line: &str) -> SerialCommand {
    let trimmed = line.trim();
    if trimmed.is_empty() {
        return SerialCommand::Unknown;
    }

    let mut parts = trimmed.split_whitespace();
    let cmd = match parts.next() {
        Some(c) => c,
        None => return SerialCommand::Unknown,
    };

    match cmd {
        "dump" => SerialCommand::Dump,
        "f" | "F" => {
            let index = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
            let value = parts.next().and_then(|s| parse_bool(s));
            SerialCommand::Feature { index, value }
        }
        "c" | "C" => {
            let id = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
            let value = parts.next().and_then(|s| s.parse().ok());
            SerialCommand::ConfigInt { id, value }
        }
        "cf" | "CF" => {
            let id = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
            let value = parts.next().and_then(|s| s.parse().ok());
            SerialCommand::ConfigFloat { id, value }
        }
        "u" | "U" => {
            let gear = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
            let value = parts.next().and_then(|s| s.parse().ok());
            SerialCommand::UpShiftSpeed { gear, value }
        }
        "d" | "D" => {
            let gear = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0);
            let value = parts.next().and_then(|s| s.parse().ok());
            SerialCommand::DownShiftSpeed { gear, value }
        }
        _ => SerialCommand::Unknown,
    }
}

fn parse_bool(s: &str) -> Option<bool> {
    match s {
        "1" | "true" | "on" | "yes" => Some(true),
        "0" | "false" | "off" | "no" => Some(false),
        _ => None,
    }
}

/// Format a simple acknowledgement string.
pub fn ack(cmd: &SerialCommand) -> String<128> {
    let mut s = String::<128>::new();
    use core::fmt::Write;
    let _ = match cmd {
        SerialCommand::Feature { index, value } => {
            write!(s, "feature {} {:?}", index, value)
        }
        SerialCommand::ConfigInt { id, value } => write!(s, "config {} {:?}", id, value),
        SerialCommand::ConfigFloat { id, value } => write!(s, "configf {} {:?}", id, value),
        SerialCommand::UpShiftSpeed { gear, value } => write!(s, "upshift {} {:?}", gear, value),
        SerialCommand::DownShiftSpeed { gear, value } => {
            write!(s, "downshift {} {:?}", gear, value)
        }
        SerialCommand::Dump => write!(s, "dump"),
        SerialCommand::Unknown => write!(s, "unknown"),
    };
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_feature_set() {
        let c = parse("f 3 1");
        assert_eq!(
            c,
            SerialCommand::Feature {
                index: 3,
                value: Some(true)
            }
        );
    }

    #[test]
    fn parse_config_int() {
        let c = parse("c 55 195");
        assert_eq!(
            c,
            SerialCommand::ConfigInt {
                id: 55,
                value: Some(195)
            }
        );
    }

    #[test]
    fn parse_dump() {
        assert_eq!(parse("dump"), SerialCommand::Dump);
    }
}
