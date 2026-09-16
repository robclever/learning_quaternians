use super::camera::Camera;
use super::geometry::{round_to, Point3D};
use super::model::{DemoLabel, PartKind, Shape};
use crate::constants::{
    GIMBAL_AXIS_EXTENT, GIMBAL_BODY_SCALE, GIMBAL_PITCH_RING_RADIUS, GIMBAL_RING_SEGMENTS,
    GIMBAL_ROLL_RING_RADIUS, GIMBAL_YAW_RING_RADIUS, TWO_PI,
};
use crate::quaternion::{EulerAngles, QuaternionMath};
use nalgebra::{UnitQuaternion, Vector3};

/// The plane a gimbal ring is drawn in, in its own unrotated frame.
///
/// The three rings are physically nested, so each one is drawn in the frame of
/// its parent: the yaw ring lives directly in the world frame, the pitch ring is
/// carried by the yaw ring, and the roll ring is carried by the pitch ring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum RingPlane {
    Yaw,
    Pitch,
    Roll,
}

impl RingPlane {
    /// All three rings, ordered outermost first so it is also the painter's order.
    pub(super) const ALL: [RingPlane; 3] = [RingPlane::Yaw, RingPlane::Pitch, RingPlane::Roll];

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
    pub(super) fn axis_kind(self) -> PartKind {
        match self {
            RingPlane::Yaw => PartKind::YawAxis,
            RingPlane::Pitch => PartKind::PitchAxis,
            RingPlane::Roll => PartKind::RollAxis,
        }
    }

    /// Unit vector of the rotation axis in this ring's local frame.
    pub(super) fn local_axis(self) -> Point3D {
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
pub(super) struct GimbalRig {
    /// Rotation of the outer ring about the world Z axis.
    yaw: UnitQuaternion<f64>,
    /// Rotation of the middle ring about its own Y axis.
    pitch: UnitQuaternion<f64>,
    /// Rotation of the inner ring about its own X axis.
    roll: UnitQuaternion<f64>,
}

impl GimbalRig {
    /// Split an Euler pose into the three individual axis rotations.
    pub(super) fn from_euler(euler: &EulerAngles) -> Self {
        GimbalRig {
            yaw: QuaternionMath::create_unit_quaternion(Vector3::new(0.0, 0.0, 1.0), euler.yaw),
            pitch: QuaternionMath::create_unit_quaternion(Vector3::new(0.0, 1.0, 0.0), euler.pitch),
            roll: QuaternionMath::create_unit_quaternion(Vector3::new(1.0, 0.0, 0.0), euler.roll),
        }
    }

    /// Frame of the outer ring: yaw only.
    pub(super) fn yaw_frame(&self) -> UnitQuaternion<f64> {
        self.yaw
    }

    /// Frame of the middle ring: it is mounted inside the yaw ring.
    pub(super) fn pitch_frame(&self) -> UnitQuaternion<f64> {
        self.yaw * self.pitch
    }

    /// Frame of the inner ring, which is also the frame of the vehicle marker.
    pub(super) fn body_frame(&self) -> UnitQuaternion<f64> {
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
pub(super) fn body_marker_local_shapes() -> Vec<(PartKind, bool, Vec<Point3D>)> {
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
pub(super) fn project_shape(
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
pub(super) fn build_shapes(camera: &Camera, rig: &GimbalRig) -> Vec<Shape> {
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
pub(super) fn build_labels(camera: &Camera, rig: &GimbalRig) -> Vec<DemoLabel> {
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;
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
}
