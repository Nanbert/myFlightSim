//! Core 6-DoF rigid-body state and the equations of motion, in a flat-earth
//! NED frame. Units are imperial: feet, slugs, seconds, radians.
//!
//! The state is the classic flight-dynamics set — NED position, body-axis
//! velocity, a body->NED attitude quaternion, and body-axis angular rate. A
//! quaternion (not Euler angles) holds attitude so the model stays valid through
//! vertical flight, which the F-22 that lands later will need.
//!
//! The EOM follow JSBSim's `FGAccelerations` with earth rotation dropped (a
//! flat, non-rotating world):
//!
//! ```text
//!   uvw_dot = F_body/m - omega x uvw + R^T * (0, 0, g0)
//!   omega_dot = J^-1 * (M_body - omega x (J * omega))
//!   q_dot     = 0.5 * q (x) (0, omega)
//!   pos_dot   = R * uvw
//! ```
//!
//! where `R` maps body axes to NED, and gravity is `(0, 0, +g0)` — down is
//! positive in NED.

use crate::atmos::G0;

pub type Vec3 = [f64; 3];

pub fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn scale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}

/// Inertia tensor about the CG (slug*ft^2), symmetric. `inv` is its inverse.
#[derive(Clone, Copy, Debug)]
pub struct Inertia {
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyz: f64,
    inv: [[f64; 3]; 3],
}

impl Inertia {
    pub fn new(ixx: f64, iyy: f64, izz: f64, ixy: f64, ixz: f64, iyz: f64) -> Self {
        let j = [[ixx, -ixy, -ixz], [-ixy, iyy, -iyz], [-ixz, -iyz, izz]];
        Inertia {
            ixx,
            iyy,
            izz,
            ixy,
            ixz,
            iyz,
            inv: invert3(j).expect("inertia tensor is singular"),
        }
    }

    /// `J * v`.
    pub fn mul(&self, v: Vec3) -> Vec3 {
        [
            self.ixx * v[0] - self.ixy * v[1] - self.ixz * v[2],
            -self.ixy * v[0] + self.iyy * v[1] - self.iyz * v[2],
            -self.ixz * v[0] - self.iyz * v[1] + self.izz * v[2],
        ]
    }

    /// `J^-1 * v`.
    pub fn mul_inv(&self, v: Vec3) -> Vec3 {
        let m = &self.inv;
        [
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
        ]
    }
}

fn invert3(m: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1e-12 {
        return None;
    }
    let id = 1.0 / det;
    Some([
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * id,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * id,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * id,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * id,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * id,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * id,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id,
        ],
    ])
}

/// Full 6-DoF state.
#[derive(Clone, Copy, Debug)]
pub struct State {
    /// Position in NED (ft): `[north, east, down]`.
    pub pos: Vec3,
    /// Velocity in body axes (ft/s): `[u, v, w]`.
    pub vel: Vec3,
    /// Attitude, body -> NED, `[w, x, y, z]`.
    pub quat: [f64; 4],
    /// Angular rate in body axes (rad/s): `[p, q, r]`.
    pub omega: Vec3,
}

impl Default for State {
    fn default() -> Self {
        State {
            pos: [0.0; 3],
            vel: [0.0; 3],
            quat: [1.0, 0.0, 0.0, 0.0],
            omega: [0.0; 3],
        }
    }
}

/// Time derivative of every state component.
#[derive(Clone, Copy, Debug)]
pub struct Deriv {
    pub pos: Vec3,
    pub vel: Vec3,
    pub quat: [f64; 4],
    pub omega: Vec3,
}

impl State {
    pub fn advance(&self, d: &Deriv, dt: f64) -> State {
        State {
            pos: add(self.pos, scale(d.pos, dt)),
            vel: add(self.vel, scale(d.vel, dt)),
            quat: [
                self.quat[0] + d.quat[0] * dt,
                self.quat[1] + d.quat[1] * dt,
                self.quat[2] + d.quat[2] * dt,
                self.quat[3] + d.quat[3] * dt,
            ],
            omega: add(self.omega, scale(d.omega, dt)),
        }
    }

    pub fn renormalize(&mut self) {
        let n = (self.quat[0] * self.quat[0]
            + self.quat[1] * self.quat[1]
            + self.quat[2] * self.quat[2]
            + self.quat[3] * self.quat[3])
            .sqrt();
        for c in self.quat.iter_mut() {
            *c /= n;
        }
    }

    /// Body vector -> NED.
    pub fn body_to_ned(&self, v: Vec3) -> Vec3 {
        quat_rotate(self.quat, v)
    }

    /// NED vector -> body.
    pub fn ned_to_body(&self, v: Vec3) -> Vec3 {
        quat_rotate_inv(self.quat, v)
    }

    /// Euler angles (rad) as `[phi, theta, psi]` from the body->NED quaternion.
    pub fn euler(&self) -> (f64, f64, f64) {
        let r = quat_to_mat(self.quat);
        let theta = (-r[2][0]).asin();
        let phi = r[2][1].atan2(r[2][2]);
        let psi = r[1][0].atan2(r[0][0]);
        (phi, theta, psi)
    }

    /// True airspeed (ft/s).
    pub fn airspeed(&self) -> f64 {
        norm(self.vel)
    }
}

/// Equations of motion: given body-frame force/moment and mass properties,
/// produce the state derivative. Gravity is added here (NED `(0,0,g0)`).
pub fn eom(s: &State, force_body: Vec3, moment_body: Vec3, mass: f64, j: &Inertia) -> Deriv {
    let grav_ned = [0.0, 0.0, G0];
    let grav_body = s.ned_to_body(grav_ned);

    // uvw_dot = F/m - omega x uvw + gravity_body
    let accel = scale(force_body, 1.0 / mass);
    let vel_dot = add(sub(accel, cross(s.omega, s.vel)), grav_body);

    // omega_dot = J^-1 (M - omega x (J omega))
    let jw = j.mul(s.omega);
    let omega_dot = j.mul_inv(sub(moment_body, cross(s.omega, jw)));

    // pos_dot = R * uvw  (NED)
    let pos_dot = s.body_to_ned(s.vel);

    // q_dot = 0.5 * q (x) (0, omega)
    let q = s.quat;
    let w = s.omega;
    let quat_dot = [
        0.5 * (-q[1] * w[0] - q[2] * w[1] - q[3] * w[2]),
        0.5 * (q[0] * w[0] + q[2] * w[2] - q[3] * w[1]),
        0.5 * (q[0] * w[1] - q[1] * w[2] + q[3] * w[0]),
        0.5 * (q[0] * w[2] + q[1] * w[1] - q[2] * w[0]),
    ];

    Deriv {
        pos: pos_dot,
        vel: vel_dot,
        quat: quat_dot,
        omega: omega_dot,
    }
}

/// Classical RK4 step. `forces` is evaluated at each stage from the trial
/// state, so aero/engine nonlinearities are honoured.
pub fn rk4<F>(s: &State, dt: f64, mass: f64, j: &Inertia, mut forces: F) -> State
where
    F: FnMut(&State) -> (Vec3, Vec3),
{
    let k1 = {
        let (f, m) = forces(s);
        eom(s, f, m, mass, j)
    };
    let k2 = {
        let (f, m) = forces(&s.advance(&k1, dt * 0.5));
        eom(&s.advance(&k1, dt * 0.5), f, m, mass, j)
    };
    let k3 = {
        let (f, m) = forces(&s.advance(&k2, dt * 0.5));
        eom(&s.advance(&k2, dt * 0.5), f, m, mass, j)
    };
    let k4 = {
        let (f, m) = forces(&s.advance(&k3, dt));
        eom(&s.advance(&k3, dt), f, m, mass, j)
    };

    let combine = |a: f64, b: f64, c: f64, d: f64| (a + 2.0 * b + 2.0 * c + d) / 6.0;
    let mut out = State {
        pos: [0.0; 3],
        vel: [0.0; 3],
        quat: [0.0; 4],
        omega: [0.0; 3],
    };
    for i in 0..3 {
        out.pos[i] = s.pos[i] + dt * combine(k1.pos[i], k2.pos[i], k3.pos[i], k4.pos[i]);
        out.vel[i] = s.vel[i] + dt * combine(k1.vel[i], k2.vel[i], k3.vel[i], k4.vel[i]);
        out.omega[i] =
            s.omega[i] + dt * combine(k1.omega[i], k2.omega[i], k3.omega[i], k4.omega[i]);
    }
    for i in 0..4 {
        out.quat[i] = s.quat[i] + dt * combine(k1.quat[i], k2.quat[i], k3.quat[i], k4.quat[i]);
    }
    out.renormalize();
    out
}

// --- quaternion helpers (Hamilton convention, [w, x, y, z]) ---

/// Rotate a vector by a quaternion (body -> NED).
pub fn quat_rotate(q: [f64; 4], v: Vec3) -> Vec3 {
    // t = 2 * (qv x v); v' = v + qw*t + qv x t
    let qv = [q[1], q[2], q[3]];
    let t = scale(cross(qv, v), 2.0);
    add(add(v, scale(t, q[0])), cross(qv, t))
}

/// Inverse rotation (NED -> body).
pub fn quat_rotate_inv(q: [f64; 4], v: Vec3) -> Vec3 {
    quat_rotate([q[0], -q[1], -q[2], -q[3]], v)
}

/// Rotation matrix (row-major) corresponding to a body->NED quaternion.
pub fn quat_to_mat(q: [f64; 4]) -> [[f64; 3]; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    const R2D: f64 = 180.0 / std::f64::consts::PI;

    fn quat_from_euler(phi: f64, theta: f64, psi: f64) -> [f64; 4] {
        let (cp, sp) = ((phi / 2.0).cos(), (phi / 2.0).sin());
        let (ct, st) = ((theta / 2.0).cos(), (theta / 2.0).sin());
        let (cy, sy) = ((psi / 2.0).cos(), (psi / 2.0).sin());
        // ZYX: q = qz * qy * qx
        [
            cy * ct * cp + sy * st * sp,
            cy * ct * sp - sy * st * cp,
            cy * st * cp + sy * ct * sp,
            sy * ct * cp - cy * st * sp,
        ]
    }

    #[test]
    fn euler_roundtrip_matches_quaternion() {
        let (phi, theta, psi) = (0.2, 0.15, 1.1);
        let s = State {
            quat: quat_from_euler(phi, theta, psi),
            ..Default::default()
        };
        let (p, t, y) = s.euler();
        assert!((p - phi).abs() < 1e-9);
        assert!((t - theta).abs() < 1e-9);
        assert!((y - psi).abs() < 1e-9);
    }

    #[test]
    fn body_to_ned_rotates_by_heading() {
        // Heading 90 deg: body +x (forward) should point east (NED +y).
        let s = State {
            quat: quat_from_euler(0.0, 0.0, 90.0 / R2D),
            ..Default::default()
        };
        let ned = s.body_to_ned([1.0, 0.0, 0.0]);
        assert!(ned[0].abs() < 1e-9);
        assert!((ned[1] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn free_fall_accelerates_downward() {
        // No force, no rotation: gravity pulls +z (down) in NED.
        let s = State::default();
        let j = Inertia::new(1000.0, 1000.0, 1000.0, 0.0, 0.0, 0.0);
        let d = eom(&s, [0.0; 3], [0.0; 3], 50.0, &j);
        assert!((d.vel[2] - G0).abs() < 1e-9); // w_dot = +g0 (body z down)
        assert!(d.pos[2].abs() < 1e-9); // not moving yet
                                        // Integrate one second and check distance ~ 0.5 g t^2.
        let s1 = rk4(&s, 1.0, 50.0, &j, |_| ([0.0; 3], [0.0; 3]));
        assert!((s1.pos[2] - 0.5 * G0).abs() < 1e-6);
        assert!((s1.vel[2] - G0).abs() < 1e-6);
    }

    #[test]
    fn pitch_rate_integrates_to_pitch_angle() {
        // A constant pitch rate q should tilt theta by q*t.
        let j = Inertia::new(1000.0, 1000.0, 1000.0, 0.0, 0.0, 0.0);
        let mut s = State {
            omega: [0.0, 0.1, 0.0],
            ..Default::default()
        };
        // Moment to hold q constant (no gyroscopic coupling since J diagonal
        // and p=r=0): zero moment works.
        s = rk4(&s, 1.0, 50.0, &j, |st| {
            let _ = st;
            ([0.0; 3], [0.0; 3])
        });
        let (_, theta, _) = s.euler();
        assert!((theta - 0.1).abs() < 1e-6);
    }

    #[test]
    fn inertia_inverse_is_correct() {
        let j = Inertia::new(948.0, 1346.0, 1967.0, 0.0, 0.0, 0.0);
        let v = [3.0, -2.0, 5.0];
        let back = j.mul_inv(j.mul(v));
        for i in 0..3 {
            assert!((back[i] - v[i]).abs() < 1e-9);
        }
    }
}
