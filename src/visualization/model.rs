use serde::Serialize;

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
