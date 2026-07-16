//! Small floating-point helpers so the crate stays `no_std` and
//! dependency-free (no `libm`). Only the operations actually needed by the
//! control logic are provided.

/// Absolute value of an `f32` without pulling in `libm`/`std`.
#[inline]
pub fn absf(x: f32) -> f32 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}

/// Clamp `x` into the inclusive range `[lo, hi]`.
#[inline]
pub fn clampf(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

/// Linear interpolation between `a` and `b` by fraction `t` (`t` is clamped to
/// `[0, 1]`).
#[inline]
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    let t = clampf(t, 0.0, 1.0);
    a + (b - a) * t
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn absf_works() {
        assert_eq!(absf(-3.5), 3.5);
        assert_eq!(absf(2.0), 2.0);
        assert_eq!(absf(0.0), 0.0);
    }

    #[test]
    fn clampf_works() {
        assert_eq!(clampf(5.0, 0.0, 10.0), 5.0);
        assert_eq!(clampf(-1.0, 0.0, 10.0), 0.0);
        assert_eq!(clampf(11.0, 0.0, 10.0), 10.0);
    }

    #[test]
    fn lerp_clamps_fraction() {
        assert_eq!(lerp(0.0, 10.0, 0.5), 5.0);
        assert_eq!(lerp(0.0, 10.0, -1.0), 0.0);
        assert_eq!(lerp(0.0, 10.0, 2.0), 10.0);
    }
}
