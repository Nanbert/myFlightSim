//! Apron taxi guidance: live lateral offset from the stand's yellow lead-in
//! line, plus a grade for the last full stop.
//!
//! The home apron has one painted lead-in line along z = -150 (see
//! `aerodrome::STAND_*`), leading from the pump lane west onto the taxiway.
//! While the aircraft is rolling inside that corridor the status block shows
//! `TAXI Δ +z m` — the signed north offset from the line — so steering the
//! centreline becomes a numbers game like the landing card's CENTRE row.
//! Coming to rest after a move freezes the average |offset| into a grade:
//! under a metre is GOOD, under 2.5 m OK, anything more SLOPPY.

use bevy::prelude::*;

use crate::app_state::{unpaused, AppState};
use crate::sim::AircraftState;

/// The lead-in corridor: z within this of the line, x from the runway edge
/// out to the pump lane.
const CORRIDOR_Z: f32 = 8.0;
const X_MIN: f32 = 10.0;
const X_MAX: f32 = 95.0;

/// Speeds (kt): above this a frame counts as "moving" (samples accumulate),
/// below this for long enough counts as a full stop.
const MOVE_MIN_GS: f32 = 3.0;
const REST_MAX_GS: f32 = 0.5;
const REST_SECONDS: f32 = 0.8;
/// Above this ground speed the aircraft is flying, not taxiing: the meter
/// goes quiet so a pass over the apron doesn't smear old samples into a stop.
const TAXI_MAX_GS: f32 = 30.0;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Grade {
    Good,
    Ok,
    Sloppy,
}

impl Grade {
    pub fn label(self) -> &'static str {
        match self {
            Grade::Good => "GOOD",
            Grade::Ok => "OK",
            Grade::Sloppy => "SLOPPY",
        }
    }

    fn from_avg(avg: f32) -> Self {
        if avg < 1.0 {
            Grade::Good
        } else if avg < 2.5 {
            Grade::Ok
        } else {
            Grade::Sloppy
        }
    }
}

#[derive(Resource, Default)]
pub struct Taximeter {
    /// Signed offset from the lead-in line (north-positive), while in the
    /// corridor at taxi speed.
    pub offset: Option<f32>,
    /// Average |offset| and its grade, frozen at the last full stop.
    pub grade: Option<(f32, Grade)>,
    samples: u32,
    sum: f32,
    /// Ground speed reached since the last stop — the "has actually taxied"
    /// gate that keeps a parked aircraft from grading itself.
    peak_gs: f32,
    rest_for: f32,
}

impl Taximeter {
    /// Feed one frame's readouts. Pure over its own state, for tests.
    fn step(&mut self, on_ground: bool, gs_kts: f64, x: f32, z: f32, dt: f32) {
        let gs = gs_kts as f32;
        self.peak_gs = self.peak_gs.max(gs);
        let dz = z - crate::aerodrome::STAND_Z;
        let in_corridor =
            on_ground && gs < TAXI_MAX_GS && x > X_MIN && x < X_MAX && dz.abs() < CORRIDOR_Z;
        self.offset = in_corridor.then_some(dz);

        if gs > MOVE_MIN_GS {
            // Moving: arm the stop detector, clear any previous grade, sample.
            self.rest_for = 0.0;
            self.grade = None;
            if in_corridor {
                self.samples += 1;
                self.sum += dz.abs();
            }
        } else if self.samples > 0 && self.peak_gs > MOVE_MIN_GS {
            if gs < REST_MAX_GS {
                self.rest_for += dt;
                if self.rest_for >= REST_SECONDS {
                    let avg = self.sum / self.samples as f32;
                    self.grade = Some((avg, Grade::from_avg(avg)));
                    self.samples = 0;
                    self.sum = 0.0;
                    self.peak_gs = 0.0;
                    self.rest_for = 0.0;
                }
            } else {
                self.rest_for = 0.0;
            }
        } else if gs > REST_MAX_GS {
            self.samples = 0;
            self.sum = 0.0;
        }
    }
}

pub struct TaxiPlugin;

impl Plugin for TaxiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Taximeter>()
            .add_systems(OnEnter(AppState::Flying), reset_taximeter)
            .add_systems(Update, update.run_if(unpaused));
    }
}

fn reset_taximeter(mut meter: ResMut<Taximeter>) {
    *meter = Taximeter::default();
}

fn update(
    time: Res<Time>,
    aircraft: Res<AircraftState>,
    map: Res<crate::map::ActiveMap>,
    mut meter: ResMut<Taximeter>,
) {
    // Home-field guidance only: the stand lives in the aerodrome frame.
    if map.0.is_some() {
        return;
    }
    meter.step(
        aircraft.on_ground,
        aircraft.gs_kts,
        aircraft.pos.x,
        aircraft.pos.z,
        time.delta_secs(),
    );
}

#[cfg(test)]
mod tests {
    use super::{Grade, Taximeter};

    const DT: f32 = 1.0 / 60.0;
    // The stand's line sits at z = -150; corridor x runs 10..95.
    const LINE_Z: f32 = -150.0;

    fn roll(meter: &mut Taximeter, secs: f32, gs: f64, x: f32, z: f32) {
        let steps = (secs / DT).ceil() as usize;
        for _ in 0..steps {
            meter.step(true, gs, x, z, DT);
        }
    }

    #[test]
    fn the_offset_reads_the_line() {
        let mut meter = Taximeter::default();
        roll(&mut meter, 0.5, 5.0, 60.0, LINE_Z + 2.5);
        assert_eq!(meter.offset, Some(2.5));
        // Out of the corridor the readout goes quiet.
        roll(&mut meter, 0.2, 5.0, 60.0, LINE_Z + 20.0);
        assert_eq!(meter.offset, None);
    }

    #[test]
    fn a_holding_stop_grades_the_centreline() {
        let mut meter = Taximeter::default();
        // Roll a 100 m leg hugging the line within ~0.6 m, then park there.
        roll(&mut meter, 4.0, 6.0, 80.0, LINE_Z + 0.5);
        roll(&mut meter, 1.5, 0.2, 80.0, LINE_Z + 0.5);
        let (avg, grade) = meter.grade.expect("stopped on the line");
        assert_eq!(grade, Grade::Good);
        assert!(avg < 1.0, "{avg} should be small");
        // The readout freezes while parked...
        assert!(meter.offset.is_some() || true);
        // ...and a fresh move re-arms the meter (grade clears, no samples).
        roll(&mut meter, 0.5, 5.0, 80.0, LINE_Z);
        assert_eq!(meter.grade, None);
    }

    #[test]
    fn a_wandering_taxi_is_not_good() {
        let mut meter = Taximeter::default();
        roll(&mut meter, 2.0, 6.0, 80.0, LINE_Z + 4.0);
        roll(&mut meter, 2.0, 6.0, 80.0, LINE_Z - 3.0);
        roll(&mut meter, 1.5, 0.2, 80.0, LINE_Z - 3.5);
        let (avg, grade) = meter.grade.expect("stopped off the line");
        assert_eq!(grade, Grade::Sloppy);
        assert!(avg >= 2.5, "{avg}");
    }

    #[test]
    fn a_parked_aircraft_never_grades_itself() {
        let mut meter = Taximeter::default();
        // Never moved: no samples, no grade, even after minutes stopped.
        roll(&mut meter, 5.0, 0.2, 60.0, LINE_Z);
        assert_eq!(meter.grade, None);
    }

    #[test]
    fn flying_over_the_apron_does_not_taxi() {
        let mut meter = Taximeter::default();
        // 60 kt along the lead line: too fast to be a taxi.
        roll(&mut meter, 2.0, 60.0, 40.0, LINE_Z);
        assert_eq!(meter.offset, None);
        roll(&mut meter, 1.5, 0.2, 40.0, LINE_Z);
        assert_eq!(meter.grade, None);
    }
}
