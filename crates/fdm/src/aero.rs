//! Generic aerodynamic coefficient evaluator — the piece shared by every
//! aircraft. It mirrors how JSBSim turns an `<axis>` block into body forces and
//! moments:
//!
//! * Each `<function>` is a **product** of *factors*. A factor is a named
//!   property, a literal `<value>`, or a `<table>` lookup.
//! * Each `<axis>` is the **sum** of its functions. The c172p file orders force
//!   axes as DRAG, SIDE, LIFT (wind frame) and moment axes as ROLL, PITCH, YAW
//!   (body frame); forces are rotated wind -> body and moments are shifted from
//!   the aerodynamic reference point to the CG.
//!
//! All data is `&'static`, so an aircraft definition can be a Rust `const` and
//! the whole thing runs with no allocation beyond the property scratch map.

use crate::state::{add, cross, Vec3};
use crate::table::{Table1, Table2};
use std::collections::HashMap;

/// A named-property scratch map: the evaluator reads inputs from here and
/// writes intermediate function results back in (e.g. `aero/function/kCDge`).
pub struct PropMap {
    map: HashMap<&'static str, f64>,
}

impl PropMap {
    pub fn new() -> Self {
        PropMap {
            map: HashMap::with_capacity(64),
        }
    }

    pub fn set(&mut self, name: &'static str, value: f64) {
        self.map.insert(name, value);
    }

    /// Missing properties read as 0.0, matching JSBSim (an unbound property is
    /// zero rather than an error).
    pub fn get(&self, name: &str) -> f64 {
        *self.map.get(name).unwrap_or(&0.0)
    }
}

impl Default for PropMap {
    fn default() -> Self {
        Self::new()
    }
}

/// A multiplicative factor inside a coefficient function.
pub enum Factor {
    /// Read a named property.
    Prop(&'static str),
    /// A literal constant.
    Value(f64),
    /// A 1-D lookup: read `var`, interpolate.
    Table1 {
        var: &'static str,
        table: Table1,
    },
    /// A 2-D lookup over row/column variables.
    Table2 {
        rvar: &'static str,
        cvar: &'static str,
        table: Table2,
    },
}

impl Factor {
    fn eval(&self, p: &PropMap) -> f64 {
        match self {
            Factor::Prop(name) => p.get(name),
            Factor::Value(v) => *v,
            Factor::Table1 { var, table } => table.lookup(p.get(var)),
            Factor::Table2 { rvar, cvar, table } => table.lookup(p.get(rvar), p.get(cvar)),
        }
    }
}

/// How a function's factors combine.
#[derive(Clone, Copy, PartialEq)]
pub enum Op {
    /// `<product>` — multiply the factors (the common case).
    Product,
    /// `<sum>` — add the factors.
    Sum,
}

/// A coefficient function: its factors combined with `op`.
pub struct Function {
    pub op: Op,
    pub factors: &'static [Factor],
}

impl Function {
    pub const fn product(factors: &'static [Factor]) -> Self {
        Function {
            op: Op::Product,
            factors,
        }
    }

    pub const fn sum(factors: &'static [Factor]) -> Self {
        Function {
            op: Op::Sum,
            factors,
        }
    }

    pub fn eval(&self, p: &PropMap) -> f64 {
        match self.op {
            Op::Product => self.factors.iter().map(|f| f.eval(p)).product(),
            Op::Sum => self.factors.iter().map(|f| f.eval(p)).sum(),
        }
    }
}

/// An axis: the sum of its functions (each already carries qbar·S and the
/// non-dimensional coefficient, exactly as JSBSim stores them).
pub struct Axis {
    pub funcs: &'static [Function],
}

impl Axis {
    pub const fn new(funcs: &'static [Function]) -> Self {
        Axis { funcs }
    }

    pub fn eval(&self, p: &PropMap) -> f64 {
        self.funcs.iter().map(|f| f.eval(p)).sum()
    }
}

/// An intermediate function published under a property name, referenced later by
/// other factors (JSBSim's file-level `<function name="aero/function/...">`).
pub struct NamedFn {
    pub name: &'static str,
    pub func: Function,
}

/// The aerodynamic model: pre-functions plus the six axes, in the c172p/JSBSim
/// wind-force + body-moment arrangement.
pub struct Aerodynamics {
    pub pre: &'static [NamedFn],
    pub drag: Axis,
    pub side: Axis,
    pub lift: Axis,
    pub roll: Axis,
    pub pitch: Axis,
    pub yaw: Axis,
    /// Aerodynamic reference point in structural-frame inches, as the model file
    /// gives it (e.g. c172p `AERORP`). The FDM turns this into a CG-relative
    /// offset to shift moments with `M += r x F`.
    pub aerorp_in: Vec3,
}

/// Body-axis force and moment produced by a single aero evaluation.
pub struct AeroResult {
    pub force_body: Vec3,
    pub moment_body: Vec3,
}

impl Aerodynamics {
    /// Evaluate every function and return the resulting body force and moment.
    /// `props` is mutated to hold the intermediate `aero/function/*` values.
    /// `rp_offset` is the AeroRP relative to the CG (body axes, ft); moments
    /// computed about the AeroRP are shifted to the CG with `M += r x F`.
    pub fn evaluate(&self, props: &mut PropMap, rp_offset: Vec3) -> AeroResult {
        for nf in self.pre {
            let v = nf.func.eval(props);
            props.set(nf.name, v);
        }

        let drag = self.drag.eval(props);
        let side = self.side.eval(props);
        let lift = self.lift.eval(props);

        let alpha = props.get("aero/alpha-rad");
        let beta = props.get("aero/beta-rad");
        let (sa, ca) = alpha.sin_cos();
        let (sb, cb) = beta.sin_cos();

        // Wind native frame is (drag, side, lift); JSBSim flips drag and lift
        // before rotating into body axes with Tw2b.
        let nd = -drag;
        let ns = side;
        let nl = -lift;
        let force_body = [
            ca * cb * nd - ca * sb * ns - sa * nl,
            sb * nd + cb * ns,
            sa * cb * nd - sa * sb * ns + ca * nl,
        ];

        let roll = self.roll.eval(props);
        let pitch = self.pitch.eval(props);
        let yaw = self.yaw.eval(props);
        // Moments are about the AeroRP (body axes); transfer to the CG.
        let moment_body = add([roll, pitch, yaw], cross(rp_offset, force_body));

        AeroResult {
            force_body,
            moment_body,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn product_and_sum_compose() {
        const FA: Function = Function::product(&[Factor::Value(2.0), Factor::Value(3.0)]);
        const FB: Function = Function::product(&[Factor::Value(4.0)]);
        const AXIS: Axis = Axis::new(&[FA, FB]);
        let p = PropMap::new();
        // 2*3 + 4 = 10
        assert_eq!(AXIS.eval(&p), 10.0);
    }

    #[test]
    fn factors_read_named_properties_and_tables() {
        const F: Function = Function::product(&[
            Factor::Prop("aero/qbar-psf"),
            Factor::Value(10.0),
            Factor::Table1 {
                var: "aero/alpha-rad",
                table: Table1::new(&[0.0, 0.1], &[0.0, 1.0]),
            },
        ]);
        let mut p = PropMap::new();
        p.set("aero/alpha-rad", 0.05);
        p.set("aero/qbar-psf", 100.0);
        // 100 * 10 * interp(0.05 -> 0.5) = 500
        assert_eq!(F.eval(&p), 500.0);
    }

    #[test]
    fn pre_functions_are_published_for_reuse() {
        const PRE: &[NamedFn] = &[NamedFn {
            name: "aero/function/kCLge",
            func: Function::product(&[Factor::Table1 {
                var: "aero/h_b-mac-ft",
                table: Table1::new(&[0.0, 1.0], &[1.2, 1.0]),
            }]),
        }];
        const EMPTY: Axis = Axis::new(&[]);
        let aerody = Aerodynamics {
            pre: PRE,
            drag: EMPTY,
            side: EMPTY,
            lift: EMPTY,
            roll: EMPTY,
            pitch: EMPTY,
            yaw: EMPTY,
            aerorp_in: [0.0; 3],
        };
        let mut p = PropMap::new();
        p.set("aero/h_b-mac-ft", 0.5);
        aerody.evaluate(&mut p, [0.0; 3]);
        assert!((p.get("aero/function/kCLge") - 1.1).abs() < 1e-12);
    }

    #[test]
    fn lift_maps_to_negative_body_z_and_drag_to_negative_body_x() {
        // alpha = beta = 0 -> ca=cb=1, sa=sb=0.
        // force_body = [-drag, side, -lift].
        const DRAG: Axis = Axis::new(&[Function::product(&[Factor::Value(2.0)])]);
        const SIDE: Axis = Axis::new(&[Function::product(&[Factor::Value(0.0)])]);
        const LIFT: Axis = Axis::new(&[Function::product(&[Factor::Value(5.0)])]);
        const EMPTY: Axis = Axis::new(&[]);
        let aerody = Aerodynamics {
            pre: &[],
            drag: DRAG,
            side: SIDE,
            lift: LIFT,
            roll: EMPTY,
            pitch: EMPTY,
            yaw: EMPTY,
            aerorp_in: [0.0; 3],
        };
        let mut p = PropMap::new();
        p.set("aero/alpha-rad", 0.0);
        p.set("aero/beta-rad", 0.0);
        let r = aerody.evaluate(&mut p, [0.0; 3]);
        assert!((r.force_body[0] + 2.0).abs() < 1e-12); // -drag
        assert!(r.force_body[1].abs() < 1e-12);
        assert!((r.force_body[2] + 5.0).abs() < 1e-12); // -lift
    }

    #[test]
    fn aero_reference_offset_produces_transfer_moment() {
        // Force of -10 in body x (drag) applied 1 ft below the CG (r_z = 1):
        // M = r x F = (0, r_z*F_x, 0) = (0, -10, 0).
        const DRAG: Axis = Axis::new(&[Function::product(&[Factor::Value(10.0)])]);
        const EMPTY: Axis = Axis::new(&[]);
        let aerody = Aerodynamics {
            pre: &[],
            drag: DRAG,
            side: EMPTY,
            lift: EMPTY,
            roll: EMPTY,
            pitch: EMPTY,
            yaw: EMPTY,
            aerorp_in: [0.0; 3],
        };
        let mut p = PropMap::new();
        p.set("aero/alpha-rad", 0.0);
        p.set("aero/beta-rad", 0.0);
        let r = aerody.evaluate(&mut p, [0.0, 0.0, 1.0]);
        assert!((r.moment_body[1] + 10.0).abs() < 1e-12);
    }
}
