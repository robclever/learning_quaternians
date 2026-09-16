use nalgebra::{UnitQuaternion, Vector3};
use serde::Serialize;

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
pub(super) fn angle_between_lines_degrees(first: Point3D, second: Point3D) -> f64 {
    let (a, b) = (first.normalized(), second.normalized());
    let cosine = (a.x * b.x + a.y * b.y + a.z * b.z).abs().clamp(0.0, 1.0);
    cosine.acos().to_degrees()
}

/// Round a value to a fixed number of decimal places.
///
/// Applied before exporting data so the generated page stays small. The
/// precision is far finer than one screen pixel, so nothing visible is lost.
pub(super) fn round_to(value: f64, decimals: u32) -> f64 {
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
    pub(super) fn to_export_array(self, decimals: u32) -> [f64; 3] {
        [
            round_to(self.x, decimals),
            round_to(self.y, decimals),
            round_to(self.depth, decimals),
        ]
    }
}
