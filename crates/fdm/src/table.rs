//! Lookup tables for aerodynamic data, mirroring JSBSim's `<table>` semantics:
//! linear interpolation between breakpoints, clamped (held constant) outside
//! the table's range. Both 1-D (`<independentVar>`) and 2-D row/column tables
//! appear throughout the c172p aero file.
//!
//! The data is stored flat so an `Aircraft` definition can be a `const`:

use std::fmt;

/// A 1-D lookup table: `(x, y)` breakpoints sorted by ascending `x`.
#[derive(Clone, Copy)]
pub struct Table1 {
    pub xs: &'static [f64],
    pub ys: &'static [f64],
}

impl Table1 {
    pub const fn new(xs: &'static [f64], ys: &'static [f64]) -> Self {
        Table1 { xs, ys }
    }

    /// Linear interpolation, clamped to the end values outside the range.
    pub fn lookup(&self, x: f64) -> f64 {
        let xs = self.xs;
        let n = xs.len();
        debug_assert_eq!(n, self.ys.len(), "table x/y length mismatch");
        if x <= xs[0] {
            return self.ys[0];
        }
        if x >= xs[n - 1] {
            return self.ys[n - 1];
        }
        // xs is sorted ascending; find the bracketing segment.
        let mut i = 0;
        while i + 1 < n && x > xs[i + 1] {
            i += 1;
        }
        let (x0, x1) = (xs[i], xs[i + 1]);
        let t = if x1 > x0 { (x - x0) / (x1 - x0) } else { 0.0 };
        self.ys[i] + t * (self.ys[i + 1] - self.ys[i])
    }
}

impl fmt::Debug for Table1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Table1({} pts)", self.xs.len())
    }
}

/// A 2-D lookup table for `<independentVar lookup="row">` /
/// `lookup="column"`. `ys` is stored row-major: `ys[row * ncol + col]`, one
/// value per (row, col) breakpoint pair. Interpolation is bilinear; both axes
/// clamp outside their ranges (matching JSBSim, whose 2-D tables require the
/// query to sit inside the grid and otherwise clamp).
#[derive(Clone, Copy)]
pub struct Table2 {
    pub rows: &'static [f64],
    pub cols: &'static [f64],
    /// Row-major: `ys[row * cols.len() + col]`.
    pub ys: &'static [f64],
}

impl Table2 {
    pub const fn new(rows: &'static [f64], cols: &'static [f64], ys: &'static [f64]) -> Self {
        Table2 { rows, cols, ys }
    }

    pub fn lookup(&self, row: f64, col: f64) -> f64 {
        let (r0, r1, tr) = bracket(self.rows, row);
        let (c0, c1, tc) = bracket(self.cols, col);
        let ncol = self.cols.len();
        let get = |r: usize, c: usize| self.ys[r * ncol + c];
        let v00 = get(r0, c0);
        let v01 = get(r0, c1);
        let v10 = get(r1, c0);
        let v11 = get(r1, c1);
        let top = v00 + tc * (v01 - v00);
        let bot = v10 + tc * (v11 - v10);
        top + tr * (bot - top)
    }
}

impl fmt::Debug for Table2 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Table2({}x{})", self.rows.len(), self.cols.len())
    }
}

/// Return the bracketing indices and interpolation fraction for `x` within a
/// sorted-ascending breakpoint list, clamping to the ends.
fn bracket(xs: &[f64], x: f64) -> (usize, usize, f64) {
    let n = xs.len();
    if x <= xs[0] {
        return (0, 0, 0.0);
    }
    if x >= xs[n - 1] {
        return (n - 1, n - 1, 0.0);
    }
    let mut i = 0;
    while i + 1 < n && x > xs[i + 1] {
        i += 1;
    }
    let (x0, x1) = (xs[i], xs[i + 1]);
    let t = if x1 > x0 { (x - x0) / (x1 - x0) } else { 0.0 };
    (i, i + 1, t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table1_interpolates_between_points() {
        let t = Table1::new(&[0.0, 10.0, 20.0], &[0.0, 100.0, 300.0]);
        assert_eq!(t.lookup(0.0), 0.0);
        assert_eq!(t.lookup(5.0), 50.0);
        assert_eq!(t.lookup(10.0), 100.0);
        assert_eq!(t.lookup(15.0), 200.0);
        assert_eq!(t.lookup(20.0), 300.0);
    }

    #[test]
    fn table1_clamps_outside_range() {
        let t = Table1::new(&[0.0, 10.0], &[1.0, 2.0]);
        assert_eq!(t.lookup(-5.0), 1.0);
        assert_eq!(t.lookup(99.0), 2.0);
    }

    #[test]
    fn table2_bilinear() {
        // rows: alpha 0.0, 1.0 ; cols: flap 0.0, 1.0
        // values: [0, 1; 2, 4]
        let t = Table2::new(&[0.0, 1.0], &[0.0, 1.0], &[0.0, 1.0, 2.0, 4.0]);
        assert_eq!(t.lookup(0.0, 0.0), 0.0);
        assert_eq!(t.lookup(1.0, 0.0), 2.0);
        assert_eq!(t.lookup(0.0, 1.0), 1.0);
        assert_eq!(t.lookup(1.0, 1.0), 4.0);
        // centre: average of the four corners = 1.75
        assert_eq!(t.lookup(0.5, 0.5), 1.75);
    }

    #[test]
    fn table2_clamps_both_axes() {
        let t = Table2::new(&[0.0, 1.0], &[0.0, 10.0], &[0.0, 1.0, 2.0, 4.0]);
        assert_eq!(t.lookup(-1.0, -5.0), 0.0);
        assert_eq!(t.lookup(9.0, 99.0), 4.0);
    }
}
