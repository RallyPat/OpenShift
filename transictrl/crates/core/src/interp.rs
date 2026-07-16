//! Lookup-table interpolation.
//!
//! Ported in spirit from the original `main/calc.cpp` percentual-map reads of
//! the 7226ctrl firmware, but generalised to borrow arbitrary axis/value
//! slices so the same code serves pressure maps, shift-time maps, temperature
//! curves and the automatic shift-point map.
//!
//! Tables store their break-points explicitly (rather than assuming a fixed
//! grid), which lets a calibration use whatever resolution it needs. Axis
//! points must be sorted ascending.

use crate::math::lerp;

/// A 1-D lookup table: `value = f(x)` with linear interpolation and clamping
/// at the ends.
#[derive(Debug, Clone, Copy)]
pub struct Map1d<'a> {
    /// Break-points along the input axis (ascending).
    pub x: &'a [f32],
    /// Output value at each break-point (`values.len() == x.len()`).
    pub values: &'a [f32],
}

impl<'a> Map1d<'a> {
    /// Create a 1-D map, returning `None` if the lengths disagree or it is empty.
    pub fn new(x: &'a [f32], values: &'a [f32]) -> Option<Self> {
        if x.is_empty() || x.len() != values.len() {
            None
        } else {
            Some(Self { x, values })
        }
    }

    /// Evaluate the table at `x`, clamping to the table's range.
    pub fn get(&self, x: f32) -> f32 {
        let (i0, i1, t) = bracket(self.x, x);
        lerp(self.values[i0], self.values[i1], t)
    }
}

/// A 2-D lookup table: `value = f(x, y)` with bilinear interpolation and
/// clamping at the edges. Values are stored row-major: index `r * x.len() + c`
/// for row `r` (a `y` break-point) and column `c` (an `x` break-point).
#[derive(Debug, Clone, Copy)]
pub struct Map2d<'a> {
    /// Break-points along the X (column) axis, ascending.
    pub x: &'a [f32],
    /// Break-points along the Y (row) axis, ascending.
    pub y: &'a [f32],
    /// Row-major values, length `x.len() * y.len()`.
    pub values: &'a [f32],
}

impl<'a> Map2d<'a> {
    /// Create a 2-D map, validating the dimensions.
    pub fn new(x: &'a [f32], y: &'a [f32], values: &'a [f32]) -> Option<Self> {
        if x.is_empty() || y.is_empty() || values.len() != x.len() * y.len() {
            None
        } else {
            Some(Self { x, y, values })
        }
    }

    #[inline]
    fn at(&self, col: usize, row: usize) -> f32 {
        self.values[row * self.x.len() + col]
    }

    /// Evaluate the table at `(x, y)`, clamping to the table's range.
    pub fn get(&self, x: f32, y: f32) -> f32 {
        let (cx0, cx1, tx) = bracket(self.x, x);
        let (ry0, ry1, ty) = bracket(self.y, y);
        // Interpolate along x on both rows, then along y.
        let top = lerp(self.at(cx0, ry0), self.at(cx1, ry0), tx);
        let bot = lerp(self.at(cx0, ry1), self.at(cx1, ry1), tx);
        lerp(top, bot, ty)
    }
}

/// Find the two break-points bracketing `v` and the interpolation fraction
/// between them. Clamps to the ends of the axis.
///
/// Returns `(lower_index, upper_index, fraction)` where `fraction` is in
/// `[0, 1]` and `value ≈ lerp(axis[lower], axis[upper], fraction)`.
fn bracket(axis: &[f32], v: f32) -> (usize, usize, f32) {
    let n = axis.len();
    if n == 1 {
        return (0, 0, 0.0);
    }
    if v <= axis[0] {
        return (0, 1, 0.0);
    }
    if v >= axis[n - 1] {
        return (n - 2, n - 1, 1.0);
    }
    // Linear scan is fine: calibration axes are short (typically < 16 points)
    // and this keeps the code branch-predictable on an MCU.
    let mut i = 0;
    while i + 1 < n && axis[i + 1] < v {
        i += 1;
    }
    let lo = axis[i];
    let hi = axis[i + 1];
    let span = hi - lo;
    let t = if span > 0.0 { (v - lo) / span } else { 0.0 };
    (i, i + 1, t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn map1d_interpolates_and_clamps() {
        let x = [0.0, 10.0, 20.0];
        let v = [100.0, 200.0, 400.0];
        let m = Map1d::new(&x, &v).unwrap();
        assert_eq!(m.get(0.0), 100.0);
        assert_eq!(m.get(5.0), 150.0); // halfway 100->200
        assert_eq!(m.get(15.0), 300.0); // halfway 200->400
        assert_eq!(m.get(-5.0), 100.0); // clamp low
        assert_eq!(m.get(99.0), 400.0); // clamp high
    }

    #[test]
    fn map2d_bilinear_corners() {
        // 2x2 grid.
        let x = [0.0, 10.0];
        let y = [0.0, 100.0];
        // row 0 (y=0):   [10, 20]
        // row 1 (y=100): [30, 40]
        let v = [10.0, 20.0, 30.0, 40.0];
        let m = Map2d::new(&x, &y, &v).unwrap();
        assert_eq!(m.get(0.0, 0.0), 10.0);
        assert_eq!(m.get(10.0, 0.0), 20.0);
        assert_eq!(m.get(0.0, 100.0), 30.0);
        assert_eq!(m.get(10.0, 100.0), 40.0);
        // center of the grid = average of corners.
        assert_eq!(m.get(5.0, 50.0), 25.0);
    }

    #[test]
    fn map2d_clamps_outside() {
        let x = [0.0, 10.0];
        let y = [0.0, 100.0];
        let v = [10.0, 20.0, 30.0, 40.0];
        let m = Map2d::new(&x, &y, &v).unwrap();
        assert_eq!(m.get(-50.0, -50.0), 10.0);
        assert_eq!(m.get(999.0, 999.0), 40.0);
    }

    #[test]
    fn rejects_bad_dims() {
        assert!(Map1d::new(&[0.0, 1.0], &[1.0]).is_none());
        assert!(Map2d::new(&[0.0, 1.0], &[0.0], &[1.0, 2.0, 3.0]).is_none());
    }
}
