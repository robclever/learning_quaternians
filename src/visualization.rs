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

use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;

use nalgebra::{UnitQuaternion, Vector3};
use serde::Serialize;

// Pull constants required from other internal module
use crate::constants::{
    GIMBAL_APPROACH_WARNING_DEGREES, GIMBAL_AXIS_ALIGNMENT_TOLERANCE_DEGREES,
    GIMBAL_AXIS_ALIGNMENT_WARNING_DEGREES, GIMBAL_AXIS_EXTENT, GIMBAL_BODY_SCALE,
    GIMBAL_DEMO_MAX_PITCH_DEGREES, GIMBAL_DEMO_ROLL_DEGREES, GIMBAL_DEMO_STEP_DEGREES,
    GIMBAL_DEMO_YAW_DEGREES, GIMBAL_PITCH_RING_RADIUS, GIMBAL_RING_SEGMENTS,
    GIMBAL_ROLL_RING_RADIUS, GIMBAL_YAW_RING_RADIUS, TWO_PI, VISUALIZATION_CAMERA_AZIMUTH_DEGREES,
    VISUALIZATION_CAMERA_ELEVATION_DEGREES, VISUALIZATION_METRIC_DECIMAL_PLACES,
    VISUALIZATION_OUTPUT_FILE_NAME, VISUALIZATION_PROJECTION_DECIMALS,
    VISUALIZATION_VIEWPORT_PADDING_PIXELS, VISUALIZATION_VIEWPORT_PIXELS,
    VISUALIZATION_WORLD_EXTENT,
};
use crate::gimbal_lock::{GimbalLockAnalysis, GimbalLockDetector, SingularityType};
use crate::quaternion::{EulerAngles, QuaternionMath};

/// A point (or direction) in the rig's world coordinate system.
///
/// The world frame is the frame used by `QuaternionMath`: `+Z` is the fixed yaw
/// axis, `+Y` is the pitch axis of the neutral rig and `+X` is the roll axis of
/// the neutral rig. Keeping this frame identical to the mathematical one is what
/// keeps the picture and the numbers in agreement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3D {
    /// Construct a point from its three world coordinates.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Point3D { x, y, z }
    }

    /// Convert an `nalgebra` vector into a visualization point.
    pub fn from_vector(vector: &Vector3<f64>) -> Self {
        Point3D::new(vector.x, vector.y, vector.z)
    }

    /// Convert this point back into an `nalgebra` vector.
    pub fn to_vector(self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }

    /// Scale this point relative to the world origin.
    pub fn scaled(self, factor: f64) -> Self {
        Point3D::new(self.x * factor, self.y * factor, self.z * factor)
    }

    /// Euclidean length of the point's position vector.
    pub fn length(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Unit vector in the same direction, or the zero vector if this is the origin.
    pub fn normalized(self) -> Self {
        let length = self.length();
        if length <= f64::EPSILON {
            self
        } else {
            self.scaled(1.0 / length)
        }
    }

    /// Rotate this point about the world origin using a unit quaternion.
    ///
    /// This is the only place the visualization applies a rotation, so every
    /// drawn ring is guaranteed to use the same convention as `QuaternionMath`.
    pub fn rotated_by(self, rotation: &UnitQuaternion<f64>) -> Self {
        Point3D::from_vector(&rotation.transform_vector(&self.to_vector()))
    }
}

/// Undirected angle, in degrees, between two lines through the world origin.
///
/// An angle of 0° means the two axes are collinear (the gimbal has collapsed)
/// and 90° means they are fully independent.
fn angle_between_lines_degrees(first: Point3D, second: Point3D) -> f64 {
    let (a, b) = (first.normalized(), second.normalized());
    let cosine = (a.x * b.x + a.y * b.y + a.z * b.z).abs().clamp(0.0, 1.0);
    cosine.acos().to_degrees()
}

/// Round a value to a fixed number of decimal places.
///
/// Applied before exporting data so the generated page stays small. The
/// precision is far finer than one screen pixel, so nothing visible is lost.
fn round_to(value: f64, decimals: u32) -> f64 {
    let factor = 10f64.powi(decimals as i32);
    (value * factor).round() / factor
}

/// A world point after the camera has projected it onto the SVG viewport.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct ProjectedPoint {
    /// Horizontal position in SVG user units (0 = left edge of the viewport).
    pub x: f64,
    /// Vertical position in SVG user units (0 = top edge of the viewport).
    pub y: f64,
    /// Distance towards the camera; larger means closer to the viewer. The
    /// renderer uses the sign to dim the half of a ring that passes behind the
    /// rig, which is what makes the drawing read as three nested rings.
    pub depth: f64,
}

impl ProjectedPoint {
    /// Compact `[x, y, depth]` form used in the exported data.
    fn to_export_array(self, decimals: u32) -> [f64; 3] {
        [
            round_to(self.x, decimals),
            round_to(self.y, decimals),
            round_to(self.depth, decimals),
        ]
    }
}

/// Orthographic orbit camera that maps the rig onto the square SVG viewport.
///
/// Orthographic projection is deliberate: with no perspective there is nothing
/// to hide, so two collinear axis lines stay collinear on screen and the
/// gimbal-lock collapse cannot be faked by the camera. The view is a fixed
/// isometric-style three-quarter view (angles come from `constants.rs`).
#[derive(Debug, Clone, Copy)]
pub struct Camera {
    /// Screen +x direction, expressed in world coordinates.
    right: Point3D,
    /// Screen +y (up) direction, expressed in world coordinates.
    up: Point3D,
    /// Direction from the world origin towards the viewer.
    toward_viewer: Point3D,
    /// Viewport size in SVG user units (the page scales it responsively).
    viewport_pixels: f64,
    /// Screen pixels per world unit.
    pixels_per_world_unit: f64,
    /// Decimal places kept when exporting projected geometry.
    projection_decimals: u32,
}

impl Default for Camera {
    fn default() -> Self {
        Camera::standard()
    }
}

impl Camera {
    /// The camera configuration used by the generated demonstration.
    pub fn standard() -> Self {
        Camera::new(
            VISUALIZATION_CAMERA_AZIMUTH_DEGREES,
            VISUALIZATION_CAMERA_ELEVATION_DEGREES,
            VISUALIZATION_VIEWPORT_PIXELS,
            VISUALIZATION_VIEWPORT_PADDING_PIXELS,
            VISUALIZATION_WORLD_EXTENT,
        )
    }

    /// Build a camera from its orbit angles and viewport geometry.
    ///
    /// # Arguments
    /// * `azimuth_degrees` - Rotation about the world Z (yaw) axis
    /// * `elevation_degrees` - Height above the world XY plane
    /// * `viewport_pixels` - Square viewport size in SVG user units
    /// * `padding_pixels` - Margin kept free around the projected rig
    /// * `world_extent` - Half-height of the visible world region, in world units
    pub fn new(
        azimuth_degrees: f64,
        elevation_degrees: f64,
        viewport_pixels: f64,
        padding_pixels: f64,
        world_extent: f64,
    ) -> Self {
        let azimuth = azimuth_degrees.to_radians();
        let elevation = elevation_degrees.to_radians();
        let (sin_azimuth, cos_azimuth) = azimuth.sin_cos();
        let (sin_elevation, cos_elevation) = elevation.sin_cos();

        // Orthonormal basis: `right` and `up` span the screen plane, and
        // `toward_viewer` completes the right-handed triple.
        let right = Point3D::new(-cos_azimuth, 0.0, sin_azimuth);
        let up = Point3D::new(
            -sin_azimuth * sin_elevation,
            cos_elevation,
            -cos_azimuth * sin_elevation,
        );
        let toward_viewer = Point3D::new(
            sin_azimuth * cos_elevation,
            sin_elevation,
            cos_azimuth * cos_elevation,
        );

        let usable_pixels = (viewport_pixels - 2.0 * padding_pixels).max(1.0);

        Camera {
            right,
            up,
            toward_viewer,
            viewport_pixels,
            pixels_per_world_unit: usable_pixels / (2.0 * world_extent),
            projection_decimals: VISUALIZATION_PROJECTION_DECIMALS,
        }
    }

    /// Project a world point onto the SVG viewport.
    ///
    /// Screen `y` grows downwards (SVG convention) while the camera's `up` is
    /// world-up, hence the sign flip.
    pub fn project(&self, point: &Point3D) -> ProjectedPoint {
        let center = self.viewport_pixels / 2.0;
        ProjectedPoint {
            x: center + self.dot(point, self.right) * self.pixels_per_world_unit,
            y: center - self.dot(point, self.up) * self.pixels_per_world_unit,
            depth: self.dot(point, self.toward_viewer),
        }
    }

    /// Dot product of a world point with one of the camera's basis vectors.
    fn dot(&self, point: &Point3D, axis: Point3D) -> f64 {
        point.x * axis.x + point.y * axis.y + point.z * axis.z
    }
}

/// Which part of the rig a shape belongs to; also selects its colour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "camelCase")]
pub enum PartKind {
    /// Outer ring, turning about the fixed world Z axis.
    YawRing,
    /// Middle ring, mounted inside the yaw ring.
    PitchRing,
    /// Inner ring, mounted inside the pitch ring.
    RollRing,
    /// Indicator line for the yaw rotation axis.
    YawAxis,
    /// Indicator line for the pitch rotation axis.
    PitchAxis,
    /// Indicator line for the roll rotation axis.
    RollAxis,
    /// Body of the vehicle marker, carried by the inner ring.
    BodyNose,
    /// Wing cross-bar of the vehicle marker.
    BodyWing,
    /// Tail stub of the vehicle marker.
    BodyTail,
}

/// One projected polyline, ready to be drawn by the renderer.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Shape {
    /// Which part of the rig this is (drives colour and line style).
    pub kind: PartKind,
    /// True when the renderer should join the last point back to the first.
    pub closed: bool,
    /// Projected points as `[x, y, depth]` in SVG user units.
    pub points: Vec<[f64; 3]>,
}

/// A text label anchored at a projected position.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DemoLabel {
    /// Label text, e.g. `roll axis`.
    pub text: String,
    /// Horizontal anchor, in SVG user units.
    pub x: f64,
    /// Vertical anchor, in SVG user units.
    pub y: f64,
}

/// How close a frame is to the gimbal-lock singularity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "camelCase")]
pub enum DemoStatus {
    /// Comfortably far from ±90° pitch.
    Safe,
    /// Within the warning band - the two axes are starting to merge.
    Approaching,
    /// At the singularity: the roll and yaw axes are collinear.
    Locked,
}

/// Every number the read-out panel shows for a single frame.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct FrameMetrics {
    /// Roll, pitch and yaw in degrees, in that order.
    pub euler_degrees: [f64; 3],
    /// Quaternion components in `nalgebra`'s storage order `[x, y, z, w]`.
    pub quaternion: [f64; 4],
    /// Rotation matrix in row-major order.
    pub rotation_matrix: [f64; 9],
    /// Roll axis in world coordinates (the axis the inner ring spins about).
    pub roll_axis: [f64; 3],
    /// Pitch axis in world coordinates.
    pub pitch_axis: [f64; 3],
    /// Yaw axis in world coordinates (always world Z for this rig).
    pub yaw_axis: [f64; 3],
    /// Undirected angle between the roll axis and the yaw axis, in degrees.
    /// 90° means fully independent, 0° means collapsed onto each other.
    pub axis_alignment_degrees: f64,
    /// `GimbalLockDetector` safety factor: 0.0 at the singularity, 1.0 safest.
    pub safety_factor: f64,
    /// True when `GimbalLockDetector` reports gimbal lock for this pose.
    pub gimbal_lock: bool,
    /// True when the roll and yaw axes are collinear to within the configured
    /// tolerance - the geometric statement of the lost degree of freedom.
    pub axes_collinear: bool,
    /// Degrees of freedom lost, as reported by `GimbalLockDetector`.
    pub degrees_of_freedom_lost: usize,
    /// Human-readable name of the singularity.
    pub singularity: String,
}

/// One step of the demonstration animation.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct DemoFrame {
    /// Short caption such as `pitch = 42°`.
    pub label: String,
    /// How close this frame is to the singularity.
    pub status: DemoStatus,
    /// Sentence explaining what happens in this frame.
    pub explanation: String,
    /// The numbers shown in the read-out panel.
    pub metrics: FrameMetrics,
    /// Geometry to draw, in painter's order (rings, then axes, then marker).
    pub shapes: Vec<Shape>,
    /// Axis labels anchored to the projected axis tips.
    pub labels: Vec<DemoLabel>,
}

/// One row of the "many Euler triples, one orientation" table.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct EquivalenceRow {
    /// Roll angle in degrees.
    pub roll_degrees: f64,
    /// Pitch angle in degrees (always the singularity for these rows).
    pub pitch_degrees: f64,
    /// Yaw angle in degrees.
    pub yaw_degrees: f64,
    /// `yaw - roll`: the only Euler quantity that still matters at pitch = +90°.
    pub invariant_degrees: f64,
    /// Quaternion components `[x, y, z, w]`.
    pub quaternion: [f64; 4],
    /// True when this triple produces the same orientation as the reference row.
    pub matches_reference: bool,
}

/// A selectable experiment; every pose is computed in Rust.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Experiment {
    pub title: String,
    pub explanation: String,
    pub control_label: String,
    pub frames: Vec<DemoFrame>,
}

/// A quaternion-driven pose, without an Euler-angle control chain.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct QuaternionFrame {
    pub angle_degrees: f64,
    pub quaternion: [f64; 4],
    pub shapes: Vec<Shape>,
}

/// A complete exported demonstration: animation frames plus teaching text.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Demo {
    /// Page heading.
    pub title: String,
    /// One-paragraph explanation under the heading.
    pub subtitle: String,
    /// Square SVG viewport size the frames were projected into.
    pub viewport_pixels: f64,
    /// Decimal places the renderer should print numbers with.
    pub metrics_decimals: u32,
    /// Alignment angle at or below which the axes are drawn as collapsed.
    pub axis_tolerance_degrees: f64,
    /// Alignment angle at or below which the axes are drawn as converging.
    pub axis_warning_degrees: f64,
    /// Animation frames, ordered by increasing pitch.
    pub frames: Vec<DemoFrame>,
    /// Euler triples at pitch = 90° that map to identical orientations.
    pub equivalence: Vec<EquivalenceRow>,
    /// Teaching notes shown beside the animation.
    pub notes: Vec<String>,
    pub experiments: Vec<Experiment>,
    pub quaternion_frames: Vec<QuaternionFrame>,
}

/// The plane a gimbal ring is drawn in, in its own unrotated frame.
///
/// The three rings are physically nested, so each one is drawn in the frame of
/// its parent: the yaw ring lives directly in the world frame, the pitch ring is
/// carried by the yaw ring, and the roll ring is carried by the pitch ring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RingPlane {
    Yaw,
    Pitch,
    Roll,
}

impl RingPlane {
    /// All three rings, ordered outermost first so it is also the painter's order.
    const ALL: [RingPlane; 3] = [RingPlane::Yaw, RingPlane::Pitch, RingPlane::Roll];

    /// Radius of this ring in world units.
    fn radius(self) -> f64 {
        match self {
            RingPlane::Yaw => GIMBAL_YAW_RING_RADIUS,
            RingPlane::Pitch => GIMBAL_PITCH_RING_RADIUS,
            RingPlane::Roll => GIMBAL_ROLL_RING_RADIUS,
        }
    }

    /// Which part kind the ring itself is drawn as.
    fn ring_kind(self) -> PartKind {
        match self {
            RingPlane::Yaw => PartKind::YawRing,
            RingPlane::Pitch => PartKind::PitchRing,
            RingPlane::Roll => PartKind::RollRing,
        }
    }

    /// Which part kind this ring's axis indicator is drawn as.
    fn axis_kind(self) -> PartKind {
        match self {
            RingPlane::Yaw => PartKind::YawAxis,
            RingPlane::Pitch => PartKind::PitchAxis,
            RingPlane::Roll => PartKind::RollAxis,
        }
    }

    /// Unit vector of the rotation axis in this ring's local frame.
    fn local_axis(self) -> Point3D {
        match self {
            RingPlane::Yaw => Point3D::new(0.0, 0.0, 1.0),
            RingPlane::Pitch => Point3D::new(0.0, 1.0, 0.0),
            RingPlane::Roll => Point3D::new(1.0, 0.0, 0.0),
        }
    }

    /// A point on the ring at parameter `angle` (radians), in the local frame.
    ///
    /// Each ring lies in the plane perpendicular to its own rotation axis, which
    /// is why the parameterisation differs per ring.
    fn point_at(self, angle: f64, radius: f64) -> Point3D {
        let (cosine, sine) = angle.sin_cos();
        match self {
            // Spins about Z, so it lies in the world XY plane.
            RingPlane::Yaw => Point3D::new(radius * cosine, radius * sine, 0.0),
            // Spins about Y, so it lies in the XZ plane.
            RingPlane::Pitch => Point3D::new(radius * cosine, 0.0, radius * sine),
            // Spins about X, so it lies in the YZ plane.
            RingPlane::Roll => Point3D::new(0.0, radius * cosine, radius * sine),
        }
    }

    /// Text used for this ring's axis label.
    fn axis_label(self) -> &'static str {
        match self {
            RingPlane::Yaw => "yaw axis (fixed)",
            RingPlane::Pitch => "pitch axis",
            RingPlane::Roll => "roll axis",
        }
    }

    /// Vertical nudge applied to this ring's label so overlapping labels at the
    /// singularity stay readable.
    fn label_offset_y(self) -> f64 {
        match self {
            RingPlane::Yaw => 4.0,
            RingPlane::Pitch => 18.0,
            RingPlane::Roll => 32.0,
        }
    }
}

/// The three stacked rotations of one pose, in the ZYX order used by
/// `QuaternionMath::from_euler_angles`.
///
/// Keeping the per-axis quaternions separate (instead of composing them once)
/// is what lets the renderer show *mounting order*: the pitch ring inherits the
/// yaw rotation, and the roll ring inherits both.
#[derive(Debug, Clone, Copy)]
struct GimbalRig {
    /// Rotation of the outer ring about the world Z axis.
    yaw: UnitQuaternion<f64>,
    /// Rotation of the middle ring about its own Y axis.
    pitch: UnitQuaternion<f64>,
    /// Rotation of the inner ring about its own X axis.
    roll: UnitQuaternion<f64>,
}

impl GimbalRig {
    /// Split an Euler pose into the three individual axis rotations.
    fn from_euler(euler: &EulerAngles) -> Self {
        GimbalRig {
            yaw: QuaternionMath::create_unit_quaternion(Vector3::new(0.0, 0.0, 1.0), euler.yaw),
            pitch: QuaternionMath::create_unit_quaternion(Vector3::new(0.0, 1.0, 0.0), euler.pitch),
            roll: QuaternionMath::create_unit_quaternion(Vector3::new(1.0, 0.0, 0.0), euler.roll),
        }
    }

    /// Frame of the outer ring: yaw only.
    fn yaw_frame(&self) -> UnitQuaternion<f64> {
        self.yaw
    }

    /// Frame of the middle ring: it is mounted inside the yaw ring.
    fn pitch_frame(&self) -> UnitQuaternion<f64> {
        self.yaw * self.pitch
    }

    /// Frame of the inner ring, which is also the frame of the vehicle marker.
    fn body_frame(&self) -> UnitQuaternion<f64> {
        self.yaw * self.pitch * self.roll
    }

    /// Frame this ring is carried in.
    fn frame_of(&self, plane: RingPlane) -> UnitQuaternion<f64> {
        match plane {
            RingPlane::Yaw => self.yaw_frame(),
            RingPlane::Pitch => self.pitch_frame(),
            RingPlane::Roll => self.body_frame(),
        }
    }
}

/// Points of one full ring, in the ring's own local frame.
fn ring_local_points(plane: RingPlane) -> Vec<Point3D> {
    let radius = plane.radius();
    (0..GIMBAL_RING_SEGMENTS)
        .map(|segment| {
            let angle = TWO_PI * segment as f64 / GIMBAL_RING_SEGMENTS as f64;
            plane.point_at(angle, radius)
        })
        .collect()
}

/// The two endpoints of a ring's rotation-axis indicator, in the local frame.
fn axis_local_points(plane: RingPlane) -> Vec<Point3D> {
    let axis = plane.local_axis();
    vec![
        axis.scaled(-GIMBAL_AXIS_EXTENT),
        axis.scaled(GIMBAL_AXIS_EXTENT),
    ]
}

/// The vehicle marker as `(kind, closed, points)`, in the body frame.
fn body_marker_local_shapes() -> Vec<(PartKind, bool, Vec<Point3D>)> {
    let scale = GIMBAL_BODY_SCALE;
    vec![
        // Dart-shaped body in the XY plane, pointing along +X.
        (
            PartKind::BodyNose,
            true,
            vec![
                Point3D::new(0.95 * scale, 0.0, 0.0),
                Point3D::new(-0.55 * scale, 0.50 * scale, 0.0),
                Point3D::new(-0.55 * scale, -0.50 * scale, 0.0),
            ],
        ),
        // Cross-bar through the body, which makes the marker read as 3D.
        (
            PartKind::BodyWing,
            false,
            vec![
                Point3D::new(0.0, 0.0, -0.45 * scale),
                Point3D::new(0.0, 0.0, 0.45 * scale),
            ],
        ),
        // Tail stub, so the marker has an obvious nose direction.
        (
            PartKind::BodyTail,
            false,
            vec![
                Point3D::new(-0.55 * scale, 0.0, 0.0),
                Point3D::new(-1.05 * scale, 0.0, 0.0),
            ],
        ),
    ]
}

/// Rotate a local polyline into the world, project it, and package it for export.
fn project_shape(
    camera: &Camera,
    kind: PartKind,
    closed: bool,
    local_points: &[Point3D],
    frame: &UnitQuaternion<f64>,
) -> Shape {
    let points = local_points
        .iter()
        .map(|point| {
            camera
                .project(&point.rotated_by(frame))
                .to_export_array(camera.projection_decimals)
        })
        .collect();
    Shape {
        kind,
        closed,
        points,
    }
}

/// Build every shape of one pose, in painter's order.
fn build_shapes(camera: &Camera, rig: &GimbalRig) -> Vec<Shape> {
    let mut shapes = Vec::with_capacity(RingPlane::ALL.len() * 2 + 3);

    // The three rings, outermost first. Each is carried by its own frame, which
    // is what makes the nesting visible.
    for plane in RingPlane::ALL {
        shapes.push(project_shape(
            camera,
            plane.ring_kind(),
            true,
            &ring_local_points(plane),
            &rig.frame_of(plane),
        ));
    }

    // The axis indicators: these are the lines that collapse at gimbal lock.
    for plane in RingPlane::ALL {
        shapes.push(project_shape(
            camera,
            plane.axis_kind(),
            false,
            &axis_local_points(plane),
            &rig.frame_of(plane),
        ));
    }

    // The vehicle marker, carried by the innermost ring.
    let body_frame = rig.body_frame();
    for (kind, closed, points) in body_marker_local_shapes() {
        shapes.push(project_shape(camera, kind, closed, &points, &body_frame));
    }

    shapes
}

/// Labels for the three axis indicators, anchored at their positive tips.
fn build_labels(camera: &Camera, rig: &GimbalRig) -> Vec<DemoLabel> {
    let decimals = camera.projection_decimals;
    RingPlane::ALL
        .iter()
        .map(|plane| {
            let frame = rig.frame_of(*plane);
            let tip = plane
                .local_axis()
                .scaled(GIMBAL_AXIS_EXTENT)
                .rotated_by(&frame);
            let projected = camera.project(&tip);
            DemoLabel {
                text: plane.axis_label().to_string(),
                x: round_to(projected.x + 9.0, decimals),
                y: round_to(projected.y + plane.label_offset_y(), decimals),
            }
        })
        .collect()
}

/// The `(roll, pitch, yaw)` triples used for the equivalence table, in degrees.
///
/// Every row sits at the pitch = +90° singularity. At that pitch the orientation
/// depends only on `yaw - roll`, so rows sharing that difference must describe an
/// identical orientation even though their individual angles differ wildly. The
/// last two rows break the rule on purpose and describe a different attitude.
const EQUIVALENCE_TRIPLES: [(f64, f64, f64); 5] = [
    (
        GIMBAL_DEMO_ROLL_DEGREES,
        GIMBAL_DEMO_MAX_PITCH_DEGREES,
        GIMBAL_DEMO_YAW_DEGREES,
    ),
    (-10.0, GIMBAL_DEMO_MAX_PITCH_DEGREES, -20.0),
    (75.0, GIMBAL_DEMO_MAX_PITCH_DEGREES, 65.0),
    (
        GIMBAL_DEMO_ROLL_DEGREES,
        GIMBAL_DEMO_MAX_PITCH_DEGREES,
        30.0,
    ),
    (10.0, GIMBAL_DEMO_MAX_PITCH_DEGREES, GIMBAL_DEMO_YAW_DEGREES),
];

/// Build `EulerAngles` from a `(roll, pitch, yaw)` triple given in degrees.
fn euler_from_degrees((roll, pitch, yaw): (f64, f64, f64)) -> EulerAngles {
    EulerAngles::new(roll.to_radians(), pitch.to_radians(), yaw.to_radians())
}

/// Build the equivalence table, proving the degeneracy with the real API.
fn build_equivalence_rows() -> Vec<EquivalenceRow> {
    let decimals = VISUALIZATION_METRIC_DECIMAL_PLACES;
    let reference = euler_from_degrees(EQUIVALENCE_TRIPLES[0]);

    EQUIVALENCE_TRIPLES
        .iter()
        .map(|&triple| {
            let euler = euler_from_degrees(triple);
            let (roll, pitch, yaw) = triple;
            EquivalenceRow {
                roll_degrees: round_to(roll, decimals),
                pitch_degrees: round_to(pitch, decimals),
                yaw_degrees: round_to(yaw, decimals),
                invariant_degrees: round_to(yaw - roll, decimals),
                quaternion: quaternion_export(&QuaternionMath::from_euler_angles(&euler)),
                matches_reference: GimbalLockDetector::demonstrate_equivalence(&reference, &euler),
            }
        })
        .collect()
}

/// Teaching notes shown next to the animation.
fn demo_notes() -> Vec<String> {
    vec![
        "The blue ring yaws about the fixed world Z axis, the green ring pitches inside it and the \
         orange ring rolls inside that - a real gimbal stack."
            .to_string(),
        "Every ring also draws its own rotation axis. As pitch grows, the orange (roll) axis swings \
         towards the blue (yaw) axis."
            .to_string(),
        "At pitch = ±90° those two axes become the same line, so the orange and blue rings spin \
         about the same axis. Roll and yaw then act about the same line: one independent control direction \
         is lost. That is gimbal lock."
            .to_string(),
        format!(
            "At pitch = +90° the orientation depends only on (yaw - roll) = {:.0}°, so completely \
             different Euler triples describe the very same attitude - see the table below. \
             Unit quaternions avoid this Euler singularity. They encode orientation directly; q and −q still represent the same rotation.",
            GIMBAL_DEMO_YAW_DEGREES - GIMBAL_DEMO_ROLL_DEGREES
        ),
        "Every number on this page - quaternion, rotation matrix, axis directions, safety factor - \
         is computed in Rust with nalgebra and exported as data, so the picture cannot disagree \
         with the library it teaches."
            .to_string(),
    ]
}

/// Build the complete gimbal-lock demonstration.
///
/// The animation sweeps pitch from 0° to 90° - the exact singularity - in
/// `GIMBAL_DEMO_STEP_DEGREES` increments while roll and yaw stay fixed. Sweeping
/// *into* the singularity instead of starting at it is what makes the collapse of
/// the roll axis onto the yaw axis visible.
pub fn build_gimbal_lock_demo() -> Demo {
    let camera = Camera::standard();
    let steps = (GIMBAL_DEMO_MAX_PITCH_DEGREES / GIMBAL_DEMO_STEP_DEGREES).round() as usize;
    let mut frames = Vec::with_capacity(steps + 1);

    for step in 0..=steps {
        let pitch_degrees = step as f64 * GIMBAL_DEMO_STEP_DEGREES;
        let euler = EulerAngles::new(
            GIMBAL_DEMO_ROLL_DEGREES.to_radians(),
            pitch_degrees.to_radians(),
            GIMBAL_DEMO_YAW_DEGREES.to_radians(),
        );
        frames.push(build_frame(&camera, &euler));
    }

    Demo {
        title: "From gimbal lock to quaternions".to_string(),
        subtitle: "Explore why rotation controls can lose a direction, then see how a quaternion represents orientation through the same pose.".into(),
        viewport_pixels: camera.viewport_pixels,
        metrics_decimals: VISUALIZATION_METRIC_DECIMAL_PLACES,
        axis_tolerance_degrees: GIMBAL_AXIS_ALIGNMENT_TOLERANCE_DEGREES,
        axis_warning_degrees: GIMBAL_AXIS_ALIGNMENT_WARNING_DEGREES,
        frames,
        equivalence: build_equivalence_rows(),
        notes: demo_notes(),
        experiments: build_experiments(&camera),
        quaternion_frames: build_quaternion_frames(&camera),
    }
}

/// Compare approaching either singularity with cancelling controls at and near it.
fn build_experiments(camera: &Camera) -> Vec<Experiment> {
    let cases = [
        ("1. Pitch up to +90°", "Sweep upward: the roll and yaw axes become the same line. At +90°, orientation depends on yaw − roll.", "Pitch sweep", 0),
        ("2. Pitch down to −90°", "The mirror case also locks. At −90°, orientation depends on yaw + roll; opposite changes in roll and yaw cancel.", "Pitch sweep", 1),
        ("3. Two moving controls, one still body", "Pitch stays at +90°. Increase roll and yaw together: both numbers change, but their difference stays −10° and the vehicle stays still. Neither control is individually broken; their effects cancel.", "Coupled roll and yaw", 2),
        ("4. Almost locked at 85°", "Repeat the same coupled changes at 85°. The vehicle moves a little because the axes are nearly aligned. This loss of sensitivity explains why Euler controls become awkward before exact lock.", "Coupled roll and yaw", 3),
    ];
    cases.into_iter().map(|(title, explanation, control_label, case)| {
        let frames = (0..=45).map(|step| {
            let amount = step as f64 * 2.0;
            let (roll, pitch, yaw) = match case {
                0 => (30.0, amount, 20.0),
                1 => (30.0, -amount, 20.0),
                2 => (30.0 + amount, 90.0, 20.0 + amount),
                _ => (30.0 + amount, 85.0, 20.0 + amount),
            };
            let mut frame = build_frame(camera, &euler_from_degrees((roll, pitch, yaw)));
            if case >= 2 {
                frame.label = format!("controls +{amount:.0}°");
                frame.explanation = if case == 2 {
                    "Roll and yaw change together, but the vehicle orientation is unchanged. The quaternion and rotation matrix stay constant.".into()
                } else {
                    "The same control changes now produce a small motion. Near alignment makes these two controls nearly redundant.".into()
                };
            }
            frame
        }).collect();
        Experiment { title: title.into(), explanation: explanation.into(), control_label: control_label.into(), frames }
    }).collect()
}

/// SLERP crosses the Euler singularity without converting back to Euler controls.
fn build_quaternion_frames(camera: &Camera) -> Vec<QuaternionFrame> {
    let axis = Vector3::new(0.0, 1.0, 0.0);
    let start = QuaternionMath::create_unit_quaternion(axis, 60_f64.to_radians());
    let end = QuaternionMath::create_unit_quaternion(axis, 120_f64.to_radians());
    (0..=60)
        .map(|step| {
            let q = QuaternionMath::slerp(&start, &end, step as f64 / 60.0);
            let mut shapes: Vec<Shape> = body_marker_local_shapes()
                .into_iter()
                .map(|(kind, closed, points)| project_shape(camera, kind, closed, &points, &q))
                .collect();
            for plane in RingPlane::ALL {
                shapes.push(project_shape(
                    camera,
                    plane.axis_kind(),
                    false,
                    &[Point3D::new(0.0, 0.0, 0.0), plane.local_axis().scaled(1.15)],
                    &q,
                ));
            }
            QuaternionFrame {
                angle_degrees: 60.0 + step as f64,
                quaternion: quaternion_export(&q),
                shapes,
            }
        })
        .collect()
}

/// Print the key frames of the demonstration to the terminal.
///
/// Handy when the page is generated on a machine with no browser, and it doubles
/// as a sanity check: these are the same numbers the page shows.
pub fn print_demo_summary(demo: &Demo) {
    println!(
        "=== GIMBAL LOCK VISUALIZATION DATA ({} frames) ===\n",
        demo.frames.len()
    );
    println!(
        "{:>7}  {:>18}  {:>8}  {:>12}  quaternion [x, y, z, w]",
        "pitch", "roll/yaw axis gap", "safety", "status"
    );

    let key_frames = demo
        .frames
        .iter()
        .filter(|frame| (frame.metrics.euler_degrees[1] as i64) % 15 == 0);

    for frame in key_frames {
        let metrics = &frame.metrics;
        println!(
            "{:>6.0}°  {:>17.1}°  {:>8.3}  {:>12}  [{:.3}, {:.3}, {:.3}, {:.3}]",
            metrics.euler_degrees[1],
            metrics.axis_alignment_degrees,
            metrics.safety_factor,
            format!("{:?}", frame.status).to_lowercase(),
            metrics.quaternion[0],
            metrics.quaternion[1],
            metrics.quaternion[2],
            metrics.quaternion[3],
        );
    }
    println!();
}

/// Convert a quaternion into rounded `[x, y, z, w]` components for export.
///
/// `nalgebra` stores quaternions as `[x, y, z, w]`, which is also the order the
/// page prints them in.
fn quaternion_export(quaternion: &UnitQuaternion<f64>) -> [f64; 4] {
    [
        round_to(quaternion.coords[0], VISUALIZATION_METRIC_DECIMAL_PLACES),
        round_to(quaternion.coords[1], VISUALIZATION_METRIC_DECIMAL_PLACES),
        round_to(quaternion.coords[2], VISUALIZATION_METRIC_DECIMAL_PLACES),
        round_to(quaternion.coords[3], VISUALIZATION_METRIC_DECIMAL_PLACES),
    ]
}

/// Convert a world-space axis vector into the rounded export form.
fn axis_export(axis: &Vector3<f64>) -> [f64; 3] {
    [
        round_to(axis.x, VISUALIZATION_METRIC_DECIMAL_PLACES),
        round_to(axis.y, VISUALIZATION_METRIC_DECIMAL_PLACES),
        round_to(axis.z, VISUALIZATION_METRIC_DECIMAL_PLACES),
    ]
}

/// Collect every number the read-out panel needs for one pose.
///
/// All of it comes from `QuaternionMath` and `GimbalLockDetector`, so the panel
/// and the geometry are guaranteed to describe the same rotation.
fn build_metrics(
    euler: &EulerAngles,
    rig: &GimbalRig,
    analysis: &GimbalLockAnalysis,
) -> FrameMetrics {
    let decimals = VISUALIZATION_METRIC_DECIMAL_PLACES;
    let quaternion = rig.body_frame();
    let rotation_matrix = QuaternionMath::to_rotation_matrix(&quaternion);

    // Axis directions as vectors, rotated with the very same quaternions that
    // produced the geometry.
    let roll_axis = rig
        .body_frame()
        .transform_vector(&Vector3::new(1.0, 0.0, 0.0));
    let pitch_axis = rig
        .pitch_frame()
        .transform_vector(&Vector3::new(0.0, 1.0, 0.0));
    let yaw_axis = rig
        .yaw_frame()
        .transform_vector(&Vector3::new(0.0, 0.0, 1.0));

    let axis_alignment_degrees = angle_between_lines_degrees(
        Point3D::from_vector(&roll_axis),
        Point3D::from_vector(&yaw_axis),
    );

    let mut matrix = [0.0_f64; 9];
    for (index, value) in matrix.iter_mut().enumerate() {
        *value = round_to(rotation_matrix[(index / 3, index % 3)], decimals);
    }

    FrameMetrics {
        euler_degrees: [
            round_to(euler.roll.to_degrees(), decimals),
            round_to(euler.pitch.to_degrees(), decimals),
            round_to(euler.yaw.to_degrees(), decimals),
        ],
        quaternion: [
            round_to(quaternion.coords[0], decimals),
            round_to(quaternion.coords[1], decimals),
            round_to(quaternion.coords[2], decimals),
            round_to(quaternion.coords[3], decimals),
        ],
        rotation_matrix: matrix,
        roll_axis: axis_export(&roll_axis),
        pitch_axis: axis_export(&pitch_axis),
        yaw_axis: axis_export(&yaw_axis),
        axis_alignment_degrees: round_to(axis_alignment_degrees, decimals),
        safety_factor: round_to(
            GimbalLockDetector::gimbal_lock_safety_factor(euler),
            decimals,
        ),
        gimbal_lock: analysis.is_gimbal_lock,
        axes_collinear: axis_alignment_degrees <= GIMBAL_AXIS_ALIGNMENT_TOLERANCE_DEGREES,
        degrees_of_freedom_lost: analysis.loss_of_degree_of_freedom,
        singularity: singularity_label(analysis),
    }
}

/// Human-readable name of the singularity reported by the detector.
fn singularity_label(analysis: &GimbalLockAnalysis) -> String {
    match &analysis.singularity_type {
        SingularityType::None => "no singularity".to_string(),
        SingularityType::PitchUp => "pitch = +90° (looking straight up)".to_string(),
        SingularityType::PitchDown => "pitch = -90° (looking straight down)".to_string(),
        SingularityType::General => "general singularity".to_string(),
    }
}

/// Classify a pose for the renderer (status pill colour and axis highlight).
fn status_for(
    safety_factor: f64,
    analysis: &GimbalLockAnalysis,
    axis_alignment_degrees: f64,
) -> DemoStatus {
    if analysis.is_gimbal_lock || axis_alignment_degrees <= GIMBAL_AXIS_ALIGNMENT_TOLERANCE_DEGREES
    {
        DemoStatus::Locked
    } else if safety_factor * 90.0 <= GIMBAL_APPROACH_WARNING_DEGREES {
        DemoStatus::Approaching
    } else {
        DemoStatus::Safe
    }
}

/// One-sentence narration for a frame, written for a lecture audience.
fn frame_explanation(status: DemoStatus, metrics: &FrameMetrics) -> String {
    let pitch = metrics.euler_degrees[1];
    match status {
        DemoStatus::Safe => format!(
            "Pitch {:.0}°: the roll axis (orange) is {:.1}° away from the yaw axis (blue). \
             Three rings turning about three different lines - three independent controls.",
            pitch, metrics.axis_alignment_degrees
        ),
        DemoStatus::Approaching => format!(
            "Pitch {:.0}°: only {:.1}° of separation left between the roll axis (orange) and the \
             yaw axis (blue). Keep going and they merge.",
            pitch, metrics.axis_alignment_degrees
        ),
        DemoStatus::Locked => format!(
            "Gimbal lock at pitch {:.0}°: the roll axis now lies exactly on the yaw axis, so \
             roll and yaw are redundant. {} independent rotation direction is lost.",
            pitch, metrics.degrees_of_freedom_lost
        ),
    }
}

/// Build one animation frame from an Euler pose.
fn build_frame(camera: &Camera, euler: &EulerAngles) -> DemoFrame {
    let rig = GimbalRig::from_euler(euler);
    let analysis = GimbalLockDetector::analyze_gimbal_lock(euler);
    let metrics = build_metrics(euler, &rig, &analysis);
    let status = status_for(
        metrics.safety_factor,
        &analysis,
        metrics.axis_alignment_degrees,
    );

    DemoFrame {
        label: format!("pitch = {:.0}°", euler.pitch.to_degrees()),
        status,
        explanation: frame_explanation(status, &metrics),
        metrics,
        shapes: build_shapes(camera, &rig),
        labels: build_labels(camera, &rig),
    }
}

/// Directory holding generated visualizations (created on demand).
///
/// It lives at the project root so the page is easy to find and share, and it is
/// listed in `.gitignore` so generated output never lands in a commit.
pub fn output_directory() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("visualizations")
}

/// Where the generated demonstration page is written by default.
pub fn default_output_path() -> PathBuf {
    output_directory().join(VISUALIZATION_OUTPUT_FILE_NAME)
}

/// Render the demonstration and write it to `path`.
///
/// Missing parent directories are created, so the caller can pass any path.
/// Returns the path that was written to.
pub fn write_html_file(demo: &Demo, path: &Path) -> io::Result<PathBuf> {
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            fs::create_dir_all(parent)?;
        }
    }
    fs::write(path, render_html(demo))?;
    Ok(path.to_path_buf())
}

/// Hand a generated page to the operating system's default browser.
///
/// The browser is spawned, not waited on, so the demo never blocks the CLI.
pub fn open_in_browser(path: &Path) -> io::Result<()> {
    browser_command().arg(path).spawn().map(|_child| ())
}

/// Platform browser launcher used by `open_in_browser`.
#[cfg(target_os = "macos")]
fn browser_command() -> Command {
    Command::new("open")
}

/// Platform browser launcher used by `open_in_browser`.
#[cfg(target_os = "windows")]
fn browser_command() -> Command {
    let mut command = Command::new("cmd");
    // The empty argument after `start` stops Windows from treating the file path
    // as a window title.
    command.args(["/C", "start", ""]);
    command
}

/// Platform browser launcher used by `open_in_browser`.
#[cfg(not(any(target_os = "macos", target_os = "windows")))]
fn browser_command() -> Command {
    Command::new("xdg-open")
}

/// Render the complete, self-contained HTML page for a demonstration.
///
/// The page embeds the frame data as JSON and does no mathematics of its own: it
/// only moves, colours and prints the numbers Rust already computed.
pub fn render_html(demo: &Demo) -> String {
    let mut html = String::with_capacity(64 * 1024);
    html.push_str(HTML_DOCUMENT_HEAD);
    html.push_str(HTML_PAGE_BODY);
    html.push_str(DATA_SCRIPT_OPEN);
    html.push_str(&json_payload(demo));
    html.push_str(HTML_BETWEEN);
    html.push_str(RENDERER_SCRIPT);
    html.push_str(HTML_TAIL);
    html
}

/// Serialize the demonstration as JSON that is safe to inline in a page.
///
/// `<` is escaped so a string in the data can never close the surrounding
/// `<script>` element early.
fn json_payload(demo: &Demo) -> String {
    let json = serde_json::to_string(demo).expect("the demo scene is always serializable");
    json.replace('<', "\\u003c")
}

/// Document head, styling and page shell.
///
/// The generated page is intentionally self-contained: no fonts, scripts or
/// styles are loaded from the network, so the file works offline, from a USB
/// stick, or on GitHub Pages without any build step.
const HTML_DOCUMENT_HEAD: &str = r##"<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>From gimbal lock to quaternions</title>
<style>
:root {
  --bg:#0d1117; --panel:#161b22; --line:#30363d; --text:#e6edf3; --muted:#8b949e;
  --yaw:#4d9dff; --pitch:#43d17a; --roll:#ffa94d; --body:#e6edf3;
  --lock:#ff4d4d; --warn:#ffd166; --ok:#43d17a;
}
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--text); font:15px/1.55 ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif; }
header { padding:26px 28px 6px; }
h1 { font-size:23px; margin:0 0 6px; }
h2 { font-size:14px; margin:0 0 8px; color:var(--muted); text-transform:uppercase; letter-spacing:.03em; }
.sub { color:var(--muted); margin:0; max-width:88ch; }
.layout { display:grid; grid-template-columns:minmax(0,1fr) minmax(0,520px); gap:18px; padding:16px 28px 32px; align-items:start; }
@media (max-width:980px) { .layout { grid-template-columns:1fr; } }
.stack { display:grid; gap:18px; }
.card { background:var(--panel); border:1px solid var(--line); border-radius:12px; padding:16px; }
svg#stage, svg#quaternion-stage { display:block; width:100%; height:auto; border-radius:10px; background:radial-gradient(circle at 50% 45%,#111823,#0a0e14 72%); }
.caption { min-height:3.4em; margin:12px 0 0; }
.controls { display:flex; align-items:center; gap:12px; margin-top:12px; }
.btn { background:#21262d; color:var(--text); border:1px solid var(--line); border-radius:8px; padding:6px 14px; font:inherit; cursor:pointer; }
.btn:hover { border-color:var(--muted); }
input[type=range] { flex:1; accent-color:var(--yaw); }
.frame-label { color:var(--muted); min-width:7.5em; text-align:right; font-variant-numeric:tabular-nums; }
.hint { color:var(--muted); font-size:12.5px; margin:8px 0 0; }
.status-row { display:flex; align-items:center; gap:10px; margin-bottom:12px; }
.pill { display:inline-block; padding:2px 10px; border-radius:999px; font-size:12px; font-weight:700; text-transform:uppercase; letter-spacing:.05em; }
.pill.safe { background:rgba(67,209,122,.16); color:var(--ok); }
.pill.approaching { background:rgba(255,209,102,.16); color:var(--warn); }
.pill.locked { background:rgba(255,77,77,.2); color:var(--lock); }
.muted { color:var(--muted); }
.stats { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:8px 14px; }
.stat { display:flex; flex-direction:column; }
.stat.wide { grid-column:1 / -1; }
.stat .k { color:var(--muted); font-size:12px; }
.stat .v { font-variant-numeric:tabular-nums; }
.mono { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:12.5px; }
.notes { margin:0; padding-left:18px; }
.notes li { margin-bottom:7px; }
table.equiv { width:100%; border-collapse:collapse; font-size:13px; font-variant-numeric:tabular-nums; }
table.equiv th, table.equiv td { text-align:left; padding:5px 6px; border-bottom:1px solid var(--line); }
table.equiv th { color:var(--muted); font-size:11.5px; text-transform:uppercase; letter-spacing:.03em; }
td.yes { color:var(--ok); }
td.no { color:var(--lock); }
.k-yawRing { stroke:var(--yaw); }
.k-pitchRing { stroke:var(--pitch); }
.k-rollRing { stroke:var(--roll); }
.k-yawAxis { stroke:var(--yaw); stroke-dasharray:7 5; }
.k-pitchAxis { stroke:var(--pitch); stroke-dasharray:7 5; }
.k-rollAxis { stroke:var(--roll); stroke-dasharray:7 5; }
.k-bodyNose, .k-bodyWing, .k-bodyTail { stroke:var(--body); }
.ring { stroke-width:3; }
.line { stroke-width:2.4; }
.dim { opacity:.3; }
.near { stroke-width:3.4; }
.converging.k-yawAxis, .converging.k-rollAxis { stroke:var(--warn); }
.aligned.k-yawAxis, .aligned.k-rollAxis { stroke:var(--lock); stroke-width:3.6; animation:pulse 1.1s ease-in-out infinite; }
@keyframes pulse { 0%,100% { opacity:1; } 50% { opacity:.4; } }
.axis-label { fill:var(--muted); font-size:13px; font-family:inherit; }
.lesson { margin:18px 28px; max-width:1400px; }
.lesson h2, .experiment-heading { color:var(--text); font-size:21px; text-transform:none; letter-spacing:0; }
.lesson-grid { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:20px; }
.lesson p { max-width:85ch; }
.step { color:var(--pitch); font-size:12px; font-weight:700; letter-spacing:.12em; text-transform:uppercase; }
.concept-picture { margin:24px 0; padding:18px; background:var(--bg); border:1px solid var(--line); border-radius:12px; }
.concept-picture svg { display:block; width:100%; height:auto; }
.concept-picture figcaption { color:var(--muted); margin-top:12px; max-width:85ch; }
.concept-panels { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:16px; }
@media (max-width:700px) { .concept-panels { grid-template-columns:1fr; } }
.formula { white-space:pre-wrap; padding:14px; background:var(--bg); border-radius:8px; font-family:ui-monospace,monospace; overflow-wrap:anywhere; }
select { width:100%; padding:10px; border:1px solid var(--line); border-radius:8px; background:var(--bg); color:var(--text); font:inherit; }
.table-scroll { overflow-x:auto; }
.quaternion-layout { display:grid; grid-template-columns:minmax(0,1fr) minmax(0,1fr); gap:24px; align-items:center; }
a { color:var(--yaw); }
@media (max-width:700px) { .lesson-grid, .quaternion-layout { grid-template-columns:1fr; } .lesson { margin:16px; } .layout { padding:16px; } .controls { flex-wrap:wrap; } }
</style>
</head>
<body>
"##;

/// Static page markup.
///
/// The renderer fills these elements with data; nothing here is generated per
/// frame, which keeps the exported file small and the JavaScript simple.
const HTML_PAGE_BODY: &str = r##"<header>
  <h1 id="title"></h1>
  <p class="sub" id="subtitle"></p>
</header>
<section class="lesson card" aria-labelledby="intro-heading">
  <span class="step">01 / The problem</span>
  <h2 id="intro-heading">What is gimbal lock?</h2>
  <p>Imagine a camera carried by three nested rings. Each ring gives you a rotation control: <strong>yaw</strong> about Z, <strong>pitch</strong> about Y, and <strong>roll</strong> about X. Normally, their combined motions let you adjust orientation in three independent directions.</p>
  <div class="lesson-grid">
    <div><h3>Three controls</h3><p>Euler angles describe a sequence of turns. Here the outer yaw ring carries the pitch ring, which carries the roll ring. Turning an outer ring also moves the axes inside it.</p></div>
    <div><h3>Two axes align</h3><p>At pitch +90° or −90°, the roll axis lies on the yaw axis. Those controls now turn about the same line. They cannot provide three independent small rotation adjustments.</p></div>
    <div><h3>One direction is lost</h3><p>The object still has a valid orientation. The trouble is the controls: different roll/yaw settings can describe exactly the same pose. A camera pointing vertically or a three-ring platform can encounter this configuration.</p></div>
  </div>
  <p><strong>Why quaternions help:</strong> software can store and update orientation directly with a unit quaternion, avoiding this singularity in Euler coordinates. A quaternion does not mechanically unlock physical rings, and converting back to Euler controls brings the singularity back.</p>
</section>
<section class="lesson" aria-labelledby="examples-heading"><span class="step">02 / Try the controls</span><h2 id="examples-heading">Four ways to see the problem</h2><p>Choose an experiment, then drag its slider or press Play. Blue is yaw, green is pitch, orange is roll; the white marker is the vehicle.</p></section>
<main class="layout">
  <section class="card">
    <label for="experiment"><strong>Experiment</strong></label>
    <select id="experiment"></select>
    <p id="experiment-explanation"></p>
    <svg id="stage" viewBox="0 0 720 720" role="img" aria-label="Three nested gimbal rings with their rotation axes and a vehicle marker"></svg>
    <p class="caption" id="caption"></p>
    <label id="control-label" for="slider">Pitch sweep</label>
    <div class="controls">
      <button class="btn" id="play" type="button">Play</button>
      <input id="slider" type="range" min="0" max="1" step="1" value="0" aria-label="Pitch angle">
      <span class="frame-label" id="frame-label"></span>
    </div>
    <p class="hint">Space plays or pauses, and the left/right arrow keys step one frame at a time - handy while presenting.</p>
  </section>
  <section class="stack">
    <div class="card">
      <div class="status-row">
        <span class="pill safe" id="status-pill">safe</span>
        <span class="muted" id="status-text"></span>
      </div>
      <div class="stats">
        <div class="stat"><span class="k">roll</span><span class="v" id="rd-roll"></span></div>
        <div class="stat"><span class="k">pitch</span><span class="v" id="rd-pitch"></span></div>
        <div class="stat"><span class="k">yaw</span><span class="v" id="rd-yaw"></span></div>
        <div class="stat"><span class="k">safety factor</span><span class="v" id="rd-safety"></span></div>
        <div class="stat wide"><span class="k">quaternion (x, y, z, w)</span><span class="v mono" id="rd-quaternion"></span></div>
        <div class="stat wide"><span class="k">angle between the roll axis and the yaw axis</span><span class="v" id="rd-axis"></span></div>
        <div class="stat"><span class="k">gimbal lock</span><span class="v" id="rd-lock"></span></div>
        <div class="stat"><span class="k">degrees of freedom lost</span><span class="v" id="rd-dof"></span></div>
        <div class="stat wide"><span class="k">singularity</span><span class="v" id="rd-singularity"></span></div>
        <div class="stat wide"><span class="k">rotation matrix (rows)</span><span class="v mono" id="rd-matrix"></span></div>
      </div>
    </div>
    <div class="card">
      <h2>A fixed example at +90°</h2>
      <p class="muted" id="equiv-intro"></p>
      <div class="table-scroll"><table class="equiv">
        <thead>
          <tr><th>roll</th><th>pitch</th><th>yaw</th><th>yaw - roll</th><th>quaternion (x, y, z, w)</th><th>same?</th></tr>
        </thead>
        <tbody id="equiv-body"></tbody>
      </table></div>
    </div>
    <div class="card">
      <h2>What to watch</h2>
      <ul class="notes" id="notes"></ul>
    </div>
  </section>
</main>
<section class="lesson card" aria-labelledby="quaternion-heading">
  <span class="step">03 / A different representation</span>
  <h2 id="quaternion-heading">Quaternions, without the math refresher</h2>
  <p><strong>Imagine holding a toy airplane.</strong> You want to describe which way it is pointing and how much it is tilted. That is its <em>orientation</em>. Its position in the room does not matter here.</p>
  <div class="lesson-grid">
    <div><h3>Euler angles: three turning instructions</h3><p>You could say, “Turn this far left, tip this far up, then roll this far.” Those are the yaw, pitch, and roll controls we used above.</p><p>The catch is that these turns affect the directions of the later controls. When two control axes line up, you effectively have two knobs doing the same job.</p></div>
    <div><h3>Another way: one imaginary skewer</h3><p>Push an imaginary skewer through the airplane’s center. Point that skewer in the right direction, then turn the airplane around it.</p><p>Starting from a reference pose, you can reach any final orientation with one carefully chosen skewer direction and one amount of turn.</p></div>
    <div><h3>A quaternion: four numbers storing that turn</h3><p>A rotation quaternion is a package of <strong>four numbers</strong> that encodes that same skewer-and-turn idea.</p><p>The numbers are arranged to make it easy for a computer to combine turns and smoothly move between orientations. You generally let a math library calculate them.</p></div>
  </div>
  <h3>So what do the four numbers mean?</h3>
  <p>We label them <strong>(x, y, z, w)</strong>. The first three point along the imaginary skewer, but their size also depends on the amount of turn. The last number, w, helps encode that amount. <strong>They work together: x, y, and z are not three separate turn angles, and w is not a fourth direction.</strong></p>
  <p>You do not have to read a quaternion and immediately picture the airplane. Think of these numbers as a useful storage format. The animation is what makes the stored orientation understandable.</p>
  <h3>A concrete example</h3>
  <p>Start with the airplane level. Put the skewer along its left-to-right axis—the Y axis in this demo—and turn it a quarter-turn, or <strong>90°</strong>. The quaternion for that turn is approximately:</p>
  <figure class="concept-picture">
    <div class="concept-panels">
      <svg viewBox="0 0 320 290" role="img" aria-labelledby="skewer-start-title skewer-start-desc">
        <title id="skewer-start-title">1. Choose an axis through the airplane</title>
        <desc id="skewer-start-desc">Side view of a level airplane pointing right. A green dot at its center marks the Y axis pointing out of the page toward you, like looking straight down the skewer.</desc>
        <text x="16" y="28" fill="#e6edf3" font-size="18" font-weight="bold">1. Put the skewer through</text>
        <path d="M 50 155 L 115 155 L 145 139 L 215 145 L 269 166 L 213 178 L 104 178 L 70 171 L 50 126 L 66 126 L 86 155 Z" fill="#243a50" stroke="#e6edf3" stroke-width="2.5"/>
        <circle cx="160" cy="165" r="14" fill="#0d1117" stroke="#43d17a" stroke-width="3"/>
        <circle cx="160" cy="165" r="5" fill="#43d17a"/>
        <path d="M 160 148 L 160 82 L 215 82" fill="none" stroke="#43d17a" stroke-width="2"/>
        <text x="178" y="66" fill="#43d17a" font-size="16">Y axis / skewer</text>
        <text x="18" y="240" fill="#e6edf3" font-size="16">You are looking along the skewer.</text>
        <text x="18" y="264" fill="#8b949e" font-size="15">The green dot points toward you.</text>
      </svg>
      <svg viewBox="0 0 320 290" role="img" aria-labelledby="skewer-turn-title skewer-turn-desc">
        <title id="skewer-turn-title">2. Turn 90 degrees around that axis</title>
        <desc id="skewer-turn-desc">An orange arrow sweeps counterclockwise from right to up around a fixed green center. The nose direction changes by a quarter-turn while the axis stays fixed.</desc>
        <defs><marker id="concept-turn-arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 Z" fill="#ffa94d"/></marker></defs>
        <text x="16" y="28" fill="#e6edf3" font-size="18" font-weight="bold">2. Make a quarter-turn</text>
        <path d="M 150 173 L 260 173" stroke="#8b949e" stroke-width="3" stroke-dasharray="6 5"/>
        <path d="M 150 173 L 150 63" stroke="#e6edf3" stroke-width="3"/>
        <path d="M 240 165 A 90 90 0 0 0 158 83" fill="none" stroke="#ffa94d" stroke-width="4" marker-end="url(#concept-turn-arrow)"/>
        <circle cx="150" cy="173" r="14" fill="#0d1117" stroke="#43d17a" stroke-width="3"/>
        <circle cx="150" cy="173" r="5" fill="#43d17a"/>
        <text x="195" y="122" fill="#ffa94d" font-size="22" font-weight="bold">90°</text>
        <text x="227" y="199" fill="#8b949e" font-size="15">start</text>
        <text x="91" y="71" fill="#e6edf3" font-size="15">finish</text>
        <text x="18" y="240" fill="#e6edf3" font-size="16">The nose moves around the axis.</text>
        <text x="18" y="264" fill="#8b949e" font-size="15">The skewer stays in the same place.</text>
      </svg>
      <svg viewBox="0 0 320 290" role="img" aria-labelledby="skewer-end-title skewer-end-desc">
        <title id="skewer-end-title">3. Store the resulting turn as a quaternion</title>
        <desc id="skewer-end-desc">The same airplane is now pointing up, rotated 90 degrees around the green axis. Its center has not moved. The quaternion stores this turn relative to the level starting pose.</desc>
        <text x="16" y="28" fill="#e6edf3" font-size="18" font-weight="bold">3. Save the new orientation</text>
        <path d="M 50 155 L 115 155 L 145 139 L 215 145 L 269 166 L 213 178 L 104 178 L 70 171 L 50 126 L 66 126 L 86 155 Z" transform="translate(160 165) rotate(-90) scale(0.85) translate(-160 -165)" fill="#243a50" stroke="#e6edf3" stroke-width="2.5"/>
        <circle cx="160" cy="165" r="14" fill="#0d1117" stroke="#43d17a" stroke-width="3"/>
        <circle cx="160" cy="165" r="5" fill="#43d17a"/>
        <text x="18" y="280" fill="#8b949e" font-size="15">Same center. Different orientation.</text>
      </svg>
    </div>
    <figcaption><strong>One axis + one turn → one quaternion.</strong> The green dot is the end of the imaginary skewer, not another moving part. This side-view sketch shows the positive Y axis pointing toward you, so the positive 90° turn goes counterclockwise. The quaternion below stores that turn relative to the level starting pose.</figcaption>
  </figure>
  <p class="formula">q = (0, 0.707, 0, 0.707)</p>
  <p>Do not worry about memorizing 0.707. The useful point is that this is a perfectly ordinary set of numbers, even at the pose where our Euler controls lined up. A slightly larger turn gives a slightly different quaternion.</p>
  <h3>How do the 3D axes map into quaternion numbers?</h3>
  <p><strong>The airplane still lives in ordinary 3D space.</strong> A quaternion is not a new spatial frame with a fourth direction. It describes the rotation between two frames: here, the airplane’s local axes and the fixed world axes.</p>
  <p>Use capital <strong>X, Y, Z</strong> for spatial axes, and lowercase <strong>x, y, z, w</strong> for the four stored quaternion components. If the rotation axis is the unit vector u = (u<sub>X</sub>, u<sub>Y</sub>, u<sub>Z</sub>), the mapping is:</p>
  <p class="formula">x = u<sub>X</sub> sin(θ/2)<br>y = u<sub>Y</sub> sin(θ/2)<br>z = u<sub>Z</sub> sin(θ/2)<br>w = cos(θ/2)</p>
  <p>So a rotation axis pointing along X contributes to x; an axis along Y contributes to y; an axis along Z contributes to z. A tilted axis contributes to several components. All three are scaled by the <em>same</em> half-angle factor.</p>
  <div class="table-scroll"><table class="equiv">
    <caption>Four examples of a positive 90° turn: sin(45°) = cos(45°) ≈ 0.707</caption>
    <thead><tr><th>Spatial rotation axis</th><th>Unit direction u</th><th>Quaternion (x, y, z, w)</th></tr></thead>
    <tbody>
      <tr><td>X</td><td>(1, 0, 0)</td><td>(0.707, 0, 0, 0.707)</td></tr>
      <tr><td>Y</td><td>(0, 1, 0)</td><td>(0, 0.707, 0, 0.707)</td></tr>
      <tr><td>Z</td><td>(0, 0, 1)</td><td>(0, 0, 0.707, 0.707)</td></tr>
      <tr><td>Halfway between X and Y</td><td>(1/√2, 1/√2, 0)</td><td>(0.5, 0.5, 0, 0.707)</td></tr>
    </tbody>
  </table></div>
  <p>The last row is <strong>one turn around a diagonal axis</strong>. It is not a 90° X turn followed by a 90° Y turn. Those sequential turns would need quaternion multiplication.</p>
  <h3>Where do the airplane’s axes point after the turn?</h3>
  <p>This is a different question from choosing the rotation axis. The airplane has three local directions: <strong>X = forward</strong>, <strong>Y = sideways</strong>, and <strong>Z = the remaining perpendicular direction</strong>. To find each direction in world coordinates, rotate it with the same quaternion.</p>
  <p>For this demo, q maps a local vector into the world frame. Put a vector v = (v<sub>X</sub>, v<sub>Y</sub>, v<sub>Z</sub>) into a quaternion with w = 0, then apply:</p>
  <p class="formula">p = (v<sub>X</sub>, v<sub>Y</sub>, v<sub>Z</sub>, 0)<br>p′ = q p q⁻¹<br>world vector = the first three components of p′</p>
  <p>Here q⁻¹ means the inverse of q. For a unit quaternion in our (x, y, z, w) order, q⁻¹ = (−x, −y, −z, w). Multiplication uses quaternion rules, not component-by-component multiplication. To map a world vector back into the local frame, reverse the operation: q⁻¹ p q.</p>
  <div class="table-scroll"><table class="equiv">
    <caption>Our +90° Y-axis example: q = (0, 1/√2, 0, 1/√2)</caption>
    <thead><tr><th>Local direction before the turn</th><th>World direction after q p q⁻¹</th><th>What happened?</th></tr></thead>
    <tbody>
      <tr><td>X = (1, 0, 0)</td><td>(0, 0, −1) = −Z</td><td>The airplane’s forward direction now points along world −Z.</td></tr>
      <tr><td>Y = (0, 1, 0)</td><td>(0, 1, 0) = +Y</td><td>The rotation axis itself stays fixed.</td></tr>
      <tr><td>Z = (0, 0, 1)</td><td>(1, 0, 0) = +X</td><td>The third local axis turns along with the airplane.</td></tr>
    </tbody>
  </table></div>
  <p>These three output vectors are the <strong>columns</strong> of the rotation matrix shown in the readout. For this example:</p>
  <p class="formula">R = [ 0  0  1 ]<br>    [ 0  1  0 ]<br>    [−1  0  0 ]<br><br>R × (a, b, c) = (c, b, −a)</p>
  <p>For example, a point at local (2, 1, 0) rotates to world (0, 1, −2), assuming the origins coincide. Rotation changes its direction without changing its distance from the center. Translation—moving the airplane’s center—is a separate operation.</p>
  <p><strong>Connect this to the pictures:</strong> in the side-view sketch, +Y points toward you, +X points right, and −Z points up. That is why the nose moves from right to up. In section 4, the orange, green, and blue lines show the rotated local X, Y, and Z axes respectively. They stay perpendicular in 3D, even when projection makes their screen angles look different.</p>
  <h3>Why does that help with gimbal lock?</h3>
  <p>Go back to experiment 3: two angle readings changed, but the airplane stayed still. The problem was how those controls described the motion.</p>
  <p>With quaternions, the computer keeps the airplane’s orientation and combines it with whatever turn you ask for next. It does not have to untangle a separate roll and yaw at the vertical pose first. You can still request a small turn around any spatial direction.</p>
  <p><strong>The important distinction:</strong> quaternions avoid the problem in how software represents and updates orientation. They do not repair a physical set of rings that has locked, and converting back to Euler angles still gives ambiguous roll/yaw readings at that pose.</p>
  <h3>And why are they useful for animation?</h3>
  <p>Suppose you save two airplane poses and want to move smoothly between them. A method called <strong>SLERP</strong> fills in the poses along a shortest rotation path. With steady playback, the airplane turns at a steady speed instead of separately adjusting three angle controls.</p>
  <p>In section 4, drag the slider from 60° to 120°. Watch the airplane pass through 90° and keep turning. Passing through 90° alone is possible with Euler angles too; the quaternion advantage is that further rotation updates do not rely on the Euler controls staying independent.</p>
  <details>
    <summary><strong>Optional: where do those numbers come from?</strong></summary>
    <p>Use a direction vector u for the skewer, scaled so its length is 1. Let θ (theta) be the angle you want to turn. The recipe is:</p>
    <p class="formula">(x, y, z) = u × sin(θ/2)<br>w = cos(θ/2)</p>
    <p>For our Y-axis example, u = (0, 1, 0) and θ = 90°. Half the angle is 45°, and both sin(45°) and cos(45°) are about 0.707. That gives (0, 0.707, 0, 0.707).</p>
    <p>The four numbers obey x² + y² + z² + w² = 1. This makes it a <strong>unit quaternion</strong>. The constraint is why four stored numbers do not mean four independent rotation controls.</p>
    <p>The half-angle comes from the way a quaternion is applied: the vector being rotated is multiplied on both sides, as q p q⁻¹. This operation turns it by twice the angle encoded inside q.</p>
    <p>You may see q written as w + xi + yj + zk. That is another notation for the same four components. Also, reversing all four signs gives the same orientation: q and −q describe the same turn.</p>
    <p>To combine turns, multiply their quaternions. Order matters, just as tipping an airplane and then rolling it can give a different result from rolling it and then tipping it.</p>
  </details>
</section>
<section class="lesson card" aria-labelledby="crossing-heading">
  <span class="step">04 / Through the singularity</span>
  <h2 id="crossing-heading">A quaternion keeps turning through 90°</h2>
  <div class="quaternion-layout">
    <div><svg id="quaternion-stage" viewBox="0 0 720 720" role="img" aria-label="Quaternion-driven vehicle with three perpendicular body axes"></svg>
      <label for="quaternion-slider">Rotation about Y: 60° → 120°</label>
      <input id="quaternion-slider" type="range" min="0" max="60" value="30" step="1" style="width:100%">
      <div class="controls"><button class="btn" id="quaternion-play" type="button">Play quaternion motion</button><output id="quaternion-angle" for="quaternion-slider"></output></div>
    </div>
    <div><h3>Same vertical pose, continuous motion</h3><p>Drag across 90°. The white vehicle moves smoothly. These colored lines are the vehicle’s <strong>body axes</strong>, which stay perpendicular in 3D; they are not the nested gimbal control axes above.</p>
      <p id="quaternion-caption" aria-live="polite"></p>
      <p class="formula" id="quaternion-value"></p>
      <p>At θ = 90°, q ≈ (0, 0.707, 0, 0.707) in (x, y, z, w) order. No component becomes undefined, and the next rotation is still well-defined.</p>
      <p>This example blends the 60° and 120° endpoint quaternions with SLERP. A simple Euler pitch sweep can also pass through 90°; the benefit is that quaternion updates do not depend on three Euler controls remaining independent.</p>
      <p><strong>In practice:</strong> keep orientation as a unit quaternion, compose rotation updates with quaternion multiplication, and normalize when numerical drift requires it. Use Euler angles for display when useful, knowing they are ambiguous at the singularity.</p>
    </div>
  </div>
</section>
<footer class="lesson"><p class="hint">Convention: R = Rz(yaw) Ry(pitch) Rx(roll). Applied to a vector, the rightmost rotation acts first. Geometry and numerical readouts are calculated in Rust; both demonstrations work offline.</p><p class="hint">Further reading: <a href="https://cseweb.ucsd.edu/~alchern/teaching/cse167_wi25/3-1Rotation3D2.pdf">UC San Diego lecture: 3D rotations and quaternions</a>.</p></footer>
"##;

/// Opening tag of the script element that carries the frame data.
///
/// Kept separate so the JSON can be written into the page untouched, apart from
/// the `<` escaping in `json_payload`.
const DATA_SCRIPT_OPEN: &str = r##"<script id="demo-data" type="application/json">
"##;

/// Closes the data script.
const HTML_BETWEEN: &str = r##"</script>
"##;

/// Renderer: draws the frames Rust exported and wires up the controls.
///
/// Deliberately free of rotation mathematics - it only reads the pre-projected
/// geometry and the pre-computed numbers, so the page cannot drift away from the
/// library's mathematics.
const RENDERER_SCRIPT: &str = r##"<script>
(function () {
  "use strict";
  const data = JSON.parse(document.getElementById("demo-data").textContent);
  const setText = (id, text) => {
    const el = document.getElementById(id);
    if (el) { el.textContent = text; }
  };
  if (!data || !data.frames || !data.frames.length) {
    setText("caption", "No demonstration data was embedded in this page.");
    return;
  }

  let frames = data.experiments[0].frames;
  const NS = "http://www.w3.org/2000/svg";
  const stage = document.getElementById("stage");
  stage.setAttribute("viewBox", "0 0 " + data.viewportPixels + " " + data.viewportPixels);
  const backLayer = document.createElementNS(NS, "g");
  const frontLayer = document.createElementNS(NS, "g");
  stage.appendChild(backLayer);
  stage.appendChild(frontLayer);

  const decimals = data.metricsDecimals;
  const fixed = (value) => Number(value).toFixed(decimals);
  const polyline = (classes, layer) => {
    const element = document.createElementNS(NS, "polyline");
    element.setAttribute("class", classes);
    element.setAttribute("fill", "none");
    element.setAttribute("stroke-linejoin", "round");
    element.setAttribute("stroke-linecap", "round");
    layer.appendChild(element);
    return element;
  };

  // One entry per shape of the first frame. Every frame has the same shapes in
  // the same order (guaranteed by the exporter), so the elements are built once.
  // Rings are split in two: a dim full ring at low opacity plus a bright arc for
  // the half that passes in front, which gives the flat SVG a sense of depth.
  const shapeElements = frames[0].shapes.map((shape) => ({
    kind: shape.kind,
    closed: shape.closed,
    main: polyline(
      "k-" + shape.kind + (shape.closed ? " ring dim" : " line"),
      shape.closed ? backLayer : frontLayer
    ),
    near: shape.closed ? polyline("k-" + shape.kind + " ring near", frontLayer) : null
  }));
  const axisElements = {};
  shapeElements.forEach((entry) => {
    if (entry.kind === "yawAxis" || entry.kind === "pitchAxis" || entry.kind === "rollAxis") {
      axisElements[entry.kind] = entry.main;
    }
  });
  const labelElements = frames[0].labels.map(() => {
    const element = document.createElementNS(NS, "text");
    element.setAttribute("class", "axis-label");
    frontLayer.appendChild(element);
    return element;
  });

  // Closed shapes repeat their first point so the polyline joins up.
  const toPoints = (points, closed) => {
    const list = closed && points.length ? points.concat([points[0]]) : points;
    return list.map((point) => point[0] + "," + point[1]).join(" ");
  };

  // The contiguous run of points facing the camera. It is drawn brighter on top
  // of the dim full ring, so each ring reads as a solid object rather than a flat
  // outline.
  const frontArc = (points) => {
    const count = points.length;
    let start = -1;
    for (let index = 0; index < count; index++) {
      if (points[index][2] >= 0 && points[(index - 1 + count) % count][2] < 0) {
        start = index;
        break;
      }
    }
    if (start < 0) {
      return points[0][2] >= 0 ? points : [];
    }
    const arc = [];
    for (let step = 0; step < count; step++) {
      const point = points[(start + step) % count];
      if (point[2] < 0) { break; }
      arc.push(point);
    }
    return arc;
  };

  const axisClass = (kind, state) => "k-" + kind + " line" + state;

  function drawFrame(index) {
    const frame = frames[index];
    frame.shapes.forEach((shape, position) => {
      const element = shapeElements[position];
      if (!element) { return; }
      element.main.setAttribute("points", toPoints(shape.points, shape.closed));
      if (element.near) {
        element.near.setAttribute("points", toPoints(frontArc(shape.points), false));
      }
    });

    // Highlight the two axes that are merging into each other.
    const gap = frame.metrics.axisAlignmentDegrees;
    const state = gap <= data.axisToleranceDegrees
      ? " aligned"
      : (gap <= data.axisWarningDegrees ? " converging" : "");
    if (axisElements.yawAxis) {
      axisElements.yawAxis.setAttribute("class", axisClass("yawAxis", state));
    }
    if (axisElements.rollAxis) {
      axisElements.rollAxis.setAttribute("class", axisClass("rollAxis", state));
    }

    frame.labels.forEach((label, position) => {
      const element = labelElements[position];
      if (!element) { return; }
      element.setAttribute("x", label.x);
      element.setAttribute("y", label.y);
      element.textContent = label.text;
    });
  }

  function updatePanel(frame) {
    const metrics = frame.metrics;
    setText("rd-roll", fixed(metrics.eulerDegrees[0]) + "°");
    setText("rd-pitch", fixed(metrics.eulerDegrees[1]) + "°");
    setText("rd-yaw", fixed(metrics.eulerDegrees[2]) + "°");
    setText("rd-quaternion", metrics.quaternion.map(fixed).join(", "));
    setText("rd-axis", fixed(metrics.axisAlignmentDegrees) + "°");
    setText("rd-safety", fixed(metrics.safetyFactor));
    setText("rd-lock", metrics.gimbalLock ? "yes" : "no");
    setText("rd-dof", String(metrics.degreesOfFreedomLost));
    setText("rd-singularity", metrics.singularity);

    const rows = [0, 1, 2].map((row) =>
      metrics.rotationMatrix.slice(row * 3, row * 3 + 3).map(fixed).join("  ")
    );
    setText("rd-matrix", rows.join("   |   "));

    const pill = document.getElementById("status-pill");
    if (pill) {
      pill.textContent = frame.status;
      pill.className = "pill " + frame.status;
    }
    setText("status-text", frame.label);
    setText("frame-label", frame.label);
    setText("caption", frame.explanation);
  }

  // Static content that never changes from frame to frame.
  setText("title", data.title);
  setText("subtitle", data.subtitle);

  const singularityPitch = data.equivalence.length
    ? fixed(data.equivalence[0].pitchDegrees) + "°"
    : "the singularity";
  setText(
    "equiv-intro",
    "Every row sits at pitch = " + singularityPitch + ". At that pitch the attitude depends " +
    "only on (yaw - roll), so rows sharing that difference describe exactly the same orientation " +
    "even though their angles differ wildly."
  );

  const equivalenceBody = document.getElementById("equiv-body");
  data.equivalence.forEach((row) => {
    const tableRow = document.createElement("tr");
    const cells = [
      fixed(row.rollDegrees) + "°",
      fixed(row.pitchDegrees) + "°",
      fixed(row.yawDegrees) + "°",
      fixed(row.invariantDegrees) + "°",
      row.quaternion.map(fixed).join(", "),
      row.matchesReference ? "same" : "different"
    ];
    cells.forEach((text, position) => {
      const cell = document.createElement("td");
      cell.textContent = text;
      if (position === 5) {
        cell.className = row.matchesReference ? "yes" : "no";
      }
      tableRow.appendChild(cell);
    });
    equivalenceBody.appendChild(tableRow);
  });

  const notes = document.getElementById("notes");
  data.notes.forEach((note) => {
    const item = document.createElement("li");
    item.textContent = note;
    notes.appendChild(item);
  });

  const slider = document.getElementById("slider");
  const playButton = document.getElementById("play");
  slider.max = String(frames.length - 1);

  let current = 0;
  let direction = 1;
  let timer = null;

  function show(index) {
    current = Math.max(0, Math.min(frames.length - 1, index));
    slider.value = String(current);
    drawFrame(current);
    updatePanel(frames[current]);
  }

  function stop() {
    if (timer !== null) {
      clearInterval(timer);
      timer = null;
    }
    playButton.textContent = "Play";
  }

  function start() {
    stop();
    // Ping-pong playback: sweep into the singularity, then back out again.
    timer = setInterval(() => {
      let next = current + direction;
      if (next >= frames.length || next < 0) {
        direction = -direction;
        next = current + direction;
      }
      show(next);
    }, 110);
    playButton.textContent = "Pause";
  }

  playButton.addEventListener("click", () => {
    if (timer === null) { start(); } else { stop(); }
  });
  slider.addEventListener("input", () => {
    stop();
    show(Number(slider.value));
  });
  document.addEventListener("keydown", (event) => {
    if (event.target.matches("input, select, button, a")) { return; }
    if (event.key === " ") {
      event.preventDefault();
      if (timer === null) { start(); } else { stop(); }
    } else if (event.key === "ArrowRight") {
      stop();
      show(current + 1);
    } else if (event.key === "ArrowLeft") {
      stop();
      show(current - 1);
    }
  });

  const experiment = document.getElementById("experiment");
  data.experiments.forEach((item, index) => {
    const option = document.createElement("option");
    option.value = String(index);
    option.textContent = item.title;
    experiment.appendChild(option);
  });
  function chooseExperiment() {
    stop();
    direction = 1;
    const selected = data.experiments[Number(experiment.value)];
    frames = selected.frames;
    slider.max = String(frames.length - 1);
    slider.setAttribute("aria-label", selected.controlLabel);
    setText("control-label", selected.controlLabel);
    setText("experiment-explanation", selected.explanation);
    show(0);
  }
  experiment.addEventListener("change", chooseExperiment);
  chooseExperiment();

  const qStage = document.getElementById("quaternion-stage");
  qStage.setAttribute("viewBox", "0 0 " + data.viewportPixels + " " + data.viewportPixels);
  const qShapes = data.quaternionFrames[0].shapes.map((shape) =>
    polyline("k-" + shape.kind + " line", qStage));
  const qSlider = document.getElementById("quaternion-slider");
  const qPlay = document.getElementById("quaternion-play");
  let qTimer = null;
  let qDirection = 1;
  function showQuaternion() {
    const frame = data.quaternionFrames[Number(qSlider.value)];
    frame.shapes.forEach((shape, index) => qShapes[index].setAttribute("points", toPoints(shape.points, shape.closed)));
    setText("quaternion-angle", frame.angleDegrees + "° about Y");
    setText("quaternion-value", "q (x, y, z, w) = (" + frame.quaternion.map(fixed).join(", ") + ")");
    setText("quaternion-caption", frame.angleDegrees === 90
      ? "At 90°: Euler roll/yaw controls would align here. The quaternion remains a valid orientation."
      : (frame.angleDegrees < 90 ? "Approaching the vertical pose." : "Past the vertical pose: rotation continues smoothly."));
  }
  function stopQuaternion() {
    clearInterval(qTimer);
    qTimer = null;
    qPlay.textContent = "Play quaternion motion";
  }
  qSlider.addEventListener("input", () => { stopQuaternion(); showQuaternion(); });
  qPlay.addEventListener("click", () => {
    if (qTimer !== null) { stopQuaternion(); return; }
    stop();
    qPlay.textContent = "Pause quaternion motion";
    qTimer = setInterval(() => {
      let next = Number(qSlider.value) + qDirection;
      if (next > 60 || next < 0) { qDirection *= -1; next = Number(qSlider.value) + qDirection; }
      qSlider.value = String(next);
      showQuaternion();
    }, 80);
  });
  showQuaternion();
})();
</script>
"##;

/// Closing tags of the document.
const HTML_TAIL: &str = r##"</body>
</html>
"##;

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn experiments_show_both_singularities_and_control_redundancy() {
        let experiments = build_experiments(&Camera::standard());
        assert_eq!(
            experiments[1].frames.last().unwrap().metrics.euler_degrees[1],
            -90.0
        );
        assert!(experiments[1].frames.last().unwrap().metrics.gimbal_lock);
        let orientation = |frame: &DemoFrame| {
            let [roll, pitch, yaw] = frame.metrics.euler_degrees;
            QuaternionMath::from_euler_angles(&euler_from_degrees((roll, pitch, yaw)))
        };
        let locked = &experiments[2].frames;
        let reference = orientation(&locked[0]);
        for frame in locked {
            assert!(QuaternionMath::same_orientation(
                &reference,
                &orientation(frame)
            ));
        }
        let near = &experiments[3].frames;
        let movement = QuaternionMath::angular_distance(
            &orientation(&near[0]),
            &orientation(near.last().unwrap()),
        );
        assert!(movement > 0.01 && movement < 0.2);
        assert!(near.iter().all(|frame| !frame.metrics.gimbal_lock));
    }

    #[test]
    fn quaternion_motion_crosses_vertical_with_equal_angular_steps() {
        let frames = build_quaternion_frames(&Camera::standard());
        let axis = Vector3::new(0.0, 1.0, 0.0);
        for (index, frame) in frames.iter().enumerate() {
            let q =
                QuaternionMath::create_unit_quaternion(axis, (60.0 + index as f64).to_radians());
            assert_eq!(frame.quaternion, quaternion_export(&q));
            let expected = Point3D::new(0.95 * GIMBAL_BODY_SCALE, 0.0, 0.0).rotated_by(&q);
            let projected = Camera::standard()
                .project(&expected)
                .to_export_array(VISUALIZATION_PROJECTION_DECIMALS);
            assert_eq!(frame.shapes[0].points[0], projected);
        }
        assert_eq!(frames[30].angle_degrees, 90.0);
        assert_eq!(frames[30].quaternion, [0.0, 0.707, 0.0, 0.707]);
    }

    /// The camera must be orthonormal: it may rotate and scale the scene, but it
    /// must not shear it, or the gimbal-lock collapse on screen would be a lie.
    #[test]
    fn camera_projection_is_orthographic() {
        let camera = Camera::standard();
        let first = Point3D::new(0.31, -0.72, 0.44);
        let second = Point3D::new(-0.15, 0.28, 0.91);
        let (a, b) = (camera.project(&first), camera.project(&second));
        let scale = camera.pixels_per_world_unit;
        let center = camera.viewport_pixels / 2.0;

        let screen_dot = ((a.x - center) / scale) * ((b.x - center) / scale)
            + ((center - a.y) / scale) * ((center - b.y) / scale)
            + a.depth * b.depth;
        let world_dot = first.x * second.x + first.y * second.y + first.z * second.z;

        assert!((screen_dot - world_dot).abs() < 1e-9);
    }

    /// Everything the demo draws has to stay inside the viewport, including the
    /// axis indicators that reach past the outermost ring.
    #[test]
    fn projected_rig_stays_inside_viewport() {
        let demo = build_gimbal_lock_demo();
        for frame in &demo.frames {
            for shape in &frame.shapes {
                for point in &shape.points {
                    assert!(
                        point[0] >= 0.0 && point[0] <= demo.viewport_pixels,
                        "x outside the viewport: {point:?}"
                    );
                    assert!(
                        point[1] >= 0.0 && point[1] <= demo.viewport_pixels,
                        "y outside the viewport: {point:?}"
                    );
                }
            }
        }
    }

    /// Rings are generated in their own plane, so an unrotated ring has to sit
    /// exactly on a circle of the configured radius, in the right plane.
    #[test]
    fn rings_are_circles_in_their_own_plane() {
        for plane in RingPlane::ALL {
            let points = ring_local_points(plane);
            assert_eq!(points.len(), GIMBAL_RING_SEGMENTS);
            for point in points {
                assert!((point.length() - plane.radius()).abs() < 1e-9);
            }
        }

        assert!(ring_local_points(RingPlane::Yaw).iter().all(|p| p.z == 0.0));
        assert!(ring_local_points(RingPlane::Pitch)
            .iter()
            .all(|p| p.y == 0.0));
        assert!(ring_local_points(RingPlane::Roll)
            .iter()
            .all(|p| p.x == 0.0));
    }

    /// The rig's stacked per-axis rotations must reproduce the library's own ZYX
    /// Euler conversion - otherwise the picture would show a different rotation
    /// from the one the numbers describe.
    #[test]
    fn gimbal_rig_matches_euler_conversion() {
        for euler in [
            EulerAngles::new(0.0, 0.0, 0.0),
            EulerAngles::new(0.4, 0.7, -0.35),
            EulerAngles::new(-PI / 3.0, PI / 2.0, PI / 4.0),
        ] {
            let rig = GimbalRig::from_euler(&euler);
            assert!(QuaternionMath::same_orientation(
                &rig.body_frame(),
                &QuaternionMath::from_euler_angles(&euler)
            ));
        }
    }

    /// The headline claim of the whole visualization: as pitch approaches ±90°
    /// the roll axis swings onto the yaw axis, leaving a gap of exactly
    /// `90° - |pitch|`, and the status goes from safe to locked.
    #[test]
    fn roll_axis_collapses_onto_yaw_axis() {
        let camera = Camera::standard();
        let cases: [(f64, f64, DemoStatus); 4] = [
            (0.0, 90.0, DemoStatus::Safe),
            (45.0, 45.0, DemoStatus::Safe),
            (88.0, 2.0, DemoStatus::Approaching),
            (90.0, 0.0, DemoStatus::Locked),
        ];

        for (pitch_degrees, expected_gap, expected_status) in cases {
            // Roll and yaw are deliberately non-zero: the gap must depend on
            // pitch alone, never on the other two angles.
            let euler = EulerAngles::new(0.3, pitch_degrees.to_radians(), 0.7);
            let frame = build_frame(&camera, &euler);
            assert!(
                (frame.metrics.axis_alignment_degrees - expected_gap).abs() < 1e-6,
                "pitch {pitch_degrees}° left a gap of {}°",
                frame.metrics.axis_alignment_degrees
            );
            assert_eq!(frame.status, expected_status, "at pitch {pitch_degrees}°");
        }
    }

    /// The exported animation must sweep pitch monotonically into the
    /// singularity and finish in the locked state.
    #[test]
    fn demo_sweep_reaches_the_singularity() {
        let demo = build_gimbal_lock_demo();
        assert_eq!(demo.frames.len(), 46);
        assert_eq!(demo.frames[0].metrics.euler_degrees[1], 0.0);
        assert_eq!(demo.frames[0].status, DemoStatus::Safe);

        let last = demo.frames.last().expect("a final frame");
        assert_eq!(last.metrics.euler_degrees[1], GIMBAL_DEMO_MAX_PITCH_DEGREES);
        assert_eq!(last.status, DemoStatus::Locked);
        assert_eq!(last.metrics.degrees_of_freedom_lost, 1);

        let pitches: Vec<f64> = demo
            .frames
            .iter()
            .map(|frame| frame.metrics.euler_degrees[1])
            .collect();
        assert!(pitches.windows(2).all(|pair| pair[1] > pair[0]));
    }

    /// The renderer builds its SVG elements once, from the first frame, so every
    /// frame has to expose the same shapes and labels in the same order.
    #[test]
    fn all_frames_share_one_shape_layout() {
        let demo = build_gimbal_lock_demo();
        let first = &demo.frames[0];

        // The renderer relies on this exact painter's order.
        let kinds: Vec<PartKind> = first.shapes.iter().map(|shape| shape.kind).collect();
        assert_eq!(
            kinds,
            vec![
                PartKind::YawRing,
                PartKind::PitchRing,
                PartKind::RollRing,
                PartKind::YawAxis,
                PartKind::PitchAxis,
                PartKind::RollAxis,
                PartKind::BodyNose,
                PartKind::BodyWing,
                PartKind::BodyTail,
            ]
        );

        for frame in &demo.frames {
            assert_eq!(frame.shapes.len(), first.shapes.len());
            assert_eq!(frame.labels.len(), first.labels.len());
            for (shape, reference) in frame.shapes.iter().zip(&first.shapes) {
                assert_eq!(shape.kind, reference.kind);
                assert_eq!(shape.closed, reference.closed);
                assert_eq!(shape.points.len(), reference.points.len());
            }
        }
    }

    /// The equivalence table must demonstrate the real invariant: at pitch = +90°
    /// the attitude depends only on (yaw - roll).
    #[test]
    fn equivalence_table_proves_the_yaw_minus_roll_invariant() {
        let demo = build_gimbal_lock_demo();
        let reference = demo.equivalence[0].invariant_degrees;
        let mut matching = 0;
        let mut differing = 0;

        for row in &demo.equivalence {
            assert_eq!(row.pitch_degrees, GIMBAL_DEMO_MAX_PITCH_DEGREES);
            let shares_invariant = (row.invariant_degrees - reference).abs() < 1e-12;
            assert_eq!(
                row.matches_reference, shares_invariant,
                "row with invariant {} must match the reference if and only if it shares it",
                row.invariant_degrees
            );
            if row.matches_reference {
                matching += 1;
            } else {
                differing += 1;
            }
        }

        assert!(matching >= 3, "expected several equivalent rows");
        assert!(differing >= 2, "expected contrast rows that differ");
    }

    /// The page must embed the frames as valid JSON and must not be able to break
    /// out of the surrounding script element.
    #[test]
    fn html_embeds_parseable_frame_data() {
        let demo = build_gimbal_lock_demo();
        let html = render_html(&demo);
        assert!(html.starts_with("<!DOCTYPE html>"));
        assert!(html.contains(HTML_PAGE_BODY));

        let start = html.find(DATA_SCRIPT_OPEN).expect("data script tag") + DATA_SCRIPT_OPEN.len();
        let end = start + html[start..].find("</script>").expect("data script end");
        let payload = &html[start..end];
        assert!(
            !payload.contains('<'),
            "the data must not be able to close the script element early"
        );

        let parsed: serde_json::Value = serde_json::from_str(payload).expect("valid demo JSON");
        assert_eq!(
            parsed["frames"].as_array().expect("frames array").len(),
            demo.frames.len()
        );
        assert_eq!(
            parsed["equivalence"]
                .as_array()
                .expect("equivalence array")
                .len(),
            demo.equivalence.len()
        );
        assert_eq!(parsed["frames"][0]["metrics"]["eulerDegrees"][1], 0.0);
    }

    /// Writing the page must create missing directories and produce a real file.
    #[test]
    fn writing_the_page_creates_directories() {
        let demo = build_gimbal_lock_demo();
        let directory = std::env::temp_dir().join("learning_quaternians_visualization_test");
        let path = directory
            .join("nested")
            .join(VISUALIZATION_OUTPUT_FILE_NAME);

        let written = write_html_file(&demo, &path).expect("write the demo page");
        let length = fs::metadata(&written).expect("page metadata").len();
        assert!(length > 20_000, "unexpectedly small page: {length} bytes");

        let _ = fs::remove_dir_all(&directory);
    }

    /// Exported numbers are rounded before they are embedded, which is what keeps
    /// the generated file small.
    #[test]
    fn exported_metrics_are_rounded() {
        let demo = build_gimbal_lock_demo();
        let frame = demo.frames.last().expect("a final frame");
        let factor = 10f64.powi(VISUALIZATION_METRIC_DECIMAL_PLACES as i32);

        for value in frame
            .metrics
            .quaternion
            .iter()
            .chain(frame.metrics.rotation_matrix.iter())
        {
            let scaled = value * factor;
            assert!(
                (scaled - scaled.round()).abs() < 1e-9,
                "{value} is not rounded"
            );
        }
    }
}
