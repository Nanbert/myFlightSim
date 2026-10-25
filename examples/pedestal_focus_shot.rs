//! Screenshot of the 737 control stand raised as a 2D hero in the cockpit.
//!
//! This is an end-to-end check of the cockpit path, not a forced state: it
//! looks down at the pedestal (the calibrated pose) and clicks the left mouse
//! button. The click has to travel through `cockpit::board_probe`'s pedestal
//! hit test and set `Ui::panel_focus`; only then does the hero appear.
//!
//! Run: `cargo run --features shots --example pedestal_focus_shot` and open
//! `/tmp/pedestal_focus.png`.

mod common;

use bevy::input::mouse::MouseButton;
use bevy::prelude::*;
use my_flight_sim::view::Ui;
use my_flight_sim::{cockpit, pedestal_ui, pfd};

fn main() {
    std::env::set_var("AIRCRAFT", "737");
    std::env::set_var("FDM_BACKEND", "rust");
    common::app("pedestal_focus_shot", Some((1200, 800)))
        .insert_resource(ClearColor(Color::srgb(0.02, 0.03, 0.05)))
        .add_plugins((
            pedestal_ui::PedestalUiPlugin,
            pfd::PfdPlugin,
            cockpit::CockpitPlugin,
            my_flight_sim::lights::LightsPlugin,
        ))
        .insert_resource(common::Shots::new(&[(90, "/tmp/pedestal_focus.png")], 110))
        .init_resource::<my_flight_sim::map::ActiveMap>()
        .init_resource::<my_flight_sim::map::SunPose>()
        .init_resource::<my_flight_sim::airfield::Airfield>()
        .init_resource::<my_flight_sim::mission::ActiveLesson>()
        .init_resource::<my_flight_sim::taxi::Taximeter>()
        .add_systems(Update, (force_look, click_pedestal, common::drive_shots))
        .run();
}

fn force_look(mut ui: ResMut<Ui>) {
    ui.cockpit = true;
    // The pose from `pedestal_calib`: the pedestal is under the crosshair.
    ui.look_yaw = (-52.0_f32).to_radians();
    ui.look_pitch = (-60.0_f32).to_radians();
}

fn click_pedestal(
    time: Res<Time>,
    mut mouse: ResMut<ButtonInput<MouseButton>>,
    mut clicked: Local<bool>,
) {
    if !*clicked && time.elapsed_secs() > 0.6 {
        *clicked = true;
        mouse.press(MouseButton::Left);
    }
}
