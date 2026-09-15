//! Interactive gimbal-lock visualization (self-contained SVG/HTML export)
//!
//! # Design
//!
//! The project goal (see README.md) is an interactive, browser-based
//! visualization that is good enough to put on a lecture projector. This module
//! takes the pragmatic route to that goal: **Rust owns every number** - the
//! Euler angles, the quaternions, the rotation matrices and the projected
//! geometry are all computed here with `nalgebra` - and the result is exported
//! as one dependency-free HTML file (`visualizations/gimbal_lock_demo.html`).
//!
//! That gives three things:
//!
//! * **No duplicated math.** The browser only draws data that Rust produced, so
//!   the picture can never disagree with the library it teaches.
//! * **No toolchain required.** The file opens offline in any browser and can be
//!   shared or dropped onto GitHub Pages as-is.
//! * **A reusable scene layer.** `build_gimbal_lock_demo()`, `Camera` and the
//!   frame types are renderer-agnostic, so the browser/WebAssembly front end
//!   described in the README (`--features wasm`) can consume exactly the same
//!   data later.
//!
//! # What the demonstration shows
//!
//! A physical three-ring gimbal rig is drawn in 3D: an outer **yaw** ring that
//! turns about the fixed world Z axis, a **pitch** ring mounted inside it, and an
//! inner **roll** ring that carries a vehicle marker. Each ring also draws its
//! own rotation axis, and that is the point of the whole picture:
//!
//! Rolling the inner ring turns its axis *within* the pitch plane, so as pitch
//! approaches ±90° the roll axis swings towards the yaw axis. At exactly ±90°
//! the two axes are collinear - both rings now spin about the same line - and
//! twisting one of them produces exactly what twisting the other one produces.
//! One of the three controls has stopped doing anything: that is gimbal lock.
//! `FrameMetrics::axis_alignment_degrees` reports how far apart the two axes
//! are, and it is exactly `90° - |pitch|`.

//! # Module layout
//!
//! - `camera`: orthographic camera configuration and projection.
//! - `geometry`: world and projected points, plus geometric helpers.
//! - `model`: serializable scene and frame data.
//! - `rig`: nested gimbal geometry and projected shapes.
//! - `demo`: teaching sequences, metrics, and terminal summary.
//! - `export`: self-contained HTML assembly and file/browser operations.
//! - `assets/`: page markup, styles, and browser renderer.

mod camera;
mod demo;
mod export;
mod geometry;
mod model;
mod rig;

#[allow(unused_imports)]
pub use camera::Camera;
pub use demo::{build_gimbal_lock_demo, print_demo_summary};
#[allow(unused_imports)]
pub use export::{
    default_output_path, open_in_browser, output_directory, render_html, write_html_file,
};
#[allow(unused_imports)]
pub use geometry::{Point3D, ProjectedPoint};
#[allow(unused_imports)]
pub use model::{
    Demo, DemoFrame, DemoLabel, DemoStatus, EquivalenceRow, Experiment, FrameMetrics, PartKind,
    QuaternionFrame, Shape,
};
