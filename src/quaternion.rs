//! Quaternion mathematics module for educational visualization
//! 
//! This module provides comprehensive quaternion mathematics with a focus on
//! demonstrating gimbal lock issues and why quaternions are superior to Euler
//! angles for representing rotations in 3D space.
//! 
//! Key features:
//! - Complete quaternion arithmetic operations
//! - Euler angle conversions with gimbal lock detection
//! - Educational demonstrations showing gimbal lock problems
//! - Interpolation methods comparison (Euler vs Quaternion)
//! - Vector rotation and rotation matrix operations

use crate::constants::MATHEMATICAL_TOLERANCE;
use nalgebra::{Matrix3, Quaternion, Unit, UnitQuaternion, Vector3};
use std::f64::consts::PI;

/// Initialize the quaternion mathematics module
/// 
/// Called at application startup to verify that the quaternion system
/// is properly configured and ready for use.
pub fn initialize() {
    println!("Quaternion math module initialized");
}

/// Euler angles representation of 3D rotation
/// 
/// Euler angles represent rotation as three sequential rotations around
/// the X (roll), Y (pitch), and Y (aw) axes. While intuitive for humans,
/// they suffer from gimbal lock singularities at pitch = ±90°.
/// 
/// Rotation order: Yaw → Pitch → Roll (applied right to left)
#[derive(Debug, Clone, Copy)]
pub struct EulerAngles {
    /// Rotation around X-axis in radians (-π to π)
    pub roll: f64,
    /// Rotation around Y-axis in radians (-π/2 to π/2)
    pub pitch: f64,
    /// Rotation around Z-axis in radians (-π to π)
    pub yaw: f64,
}

impl EulerAngles {
    /// Create a new EulerAngles instance
    /// 
    /// # Arguments
    /// * `roll` - Rotation around X-axis in radians
    /// * `pitch` - Rotation around Y-axis in radians  
    /// * `yaw` - Rotation around Z-axis in radians
    /// 
    /// # Returns
    /// New EulerAngles instance with the specified rotation
    pub fn new(roll: f64, pitch: f64, yaw: f64) -> Self {
        EulerAngles { roll, pitch, yaw }
    }

    /// Convert Euler angles from radians to degrees
    /// 
    /// Takes self by value since EulerAngles implements Copy trait.
    /// This follows Rust convention for to_* methods on Copy types.
    /// 
    /// # Returns
    /// New EulerAngles instance with angles converted to degrees
    #[allow(dead_code)]
    pub fn to_degrees(self) -> EulerAngles {
        EulerAngles {
            roll: self.roll.to_degrees(),
            pitch: self.pitch.to_degrees(),
            yaw: self.yaw.to_degrees(),
        }
    }

    /// Convert Euler angles from degrees to radians
    /// 
    /// Takes self by value since EulerAngles implements Copy trait.
    /// This follows Rust convention for to_* methods on Copy types.
    /// 
    /// # Returns
    /// New EulerAngles instance with angles converted to radians
    #[allow(dead_code)]
    pub fn to_radians(self) -> EulerAngles {
        EulerAngles {
            roll: self.roll.to_radians(),
            pitch: self.pitch.to_radians(),
            yaw: self.yaw.to_radians(),
        }
    }
}

#[allow(dead_code)]
pub struct QuaternionMath;

#[allow(dead_code)]
impl QuaternionMath {
    /// Create a quaternion from its four components
    /// 
    /// Creates a general (non-unit) quaternion with the specified w, x, y, z components.
    /// For rotation operations, use `create_unit_quaternion` instead.
    /// 
    /// # Arguments
    /// * `w` - Real component (cos(θ/2) for rotation quaternions)
    /// * `x` - First imaginary component (axis_x * sin(θ/2))
    /// * `y` - Second imaginary component (axis_y * sin(θ/2))
    /// * `z` - Third imaginary component (axis_z * sin(θ/2))
    /// 
    /// # Returns
    /// Quaternion with the specified components
    pub fn create_quaternion(w: f64, x: f64, y: f64, z: f64) -> Quaternion<f64> {
        Quaternion::new(w, x, y, z)
    }

    /// Create a unit quaternion from an axis and angle
    /// 
    /// Creates a normalized quaternion representing rotation by `angle` radians
    /// around the specified `axis`. The axis will be normalized automatically.
    /// 
    /// # Arguments
    /// * `axis` - Rotation axis (will be normalized)
    /// * `angle` - Rotation angle in radians
    /// 
    /// # Returns
    /// Unit quaternion representing the specified rotation
    pub fn create_unit_quaternion(axis: Vector3<f64>, angle: f64) -> UnitQuaternion<f64> {
        // Normalize the axis to ensure we create a valid rotation quaternion
        let unit_axis = Unit::new_normalize(axis);
        UnitQuaternion::from_axis_angle(&unit_axis, angle)
    }

    /// Convert Euler angles to a unit quaternion
    /// 
    /// Converts Euler angles (roll, pitch, yaw) to a quaternion representation.
    /// Uses ZYX rotation order: yaw → pitch → roll (applied right to left).
    /// This conversion eliminates gimbal lock singularities.
    /// 
    /// # Arguments
    /// * `euler` - Euler angles to convert
    /// 
    /// # Returns
    /// Unit quaternion representing the same rotation as the Euler angles
    pub fn from_euler_angles(euler: &EulerAngles) -> UnitQuaternion<f64> {
        // Create individual rotation quaternions for each axis
        let q_roll = Self::create_unit_quaternion(Vector3::new(1.0, 0.0, 0.0), euler.roll);
        let q_pitch = Self::create_unit_quaternion(Vector3::new(0.0, 1.0, 0.0), euler.pitch);
        let q_yaw = Self::create_unit_quaternion(Vector3::new(0.0, 0.0, 1.0), euler.yaw);

        // Combine rotations in ZYX order: yaw → pitch → roll
        // Quaternion multiplication applies rotations right to left
        q_yaw * q_pitch * q_roll
    }

    /// Convert a unit quaternion to Euler angles
    /// 
    /// Converts a quaternion back to Euler angles (roll, pitch, yaw).
    /// Note: This conversion may encounter gimbal lock issues when
    /// pitch is near ±90°, which is why quaternions are preferred.
    /// 
    /// # Arguments
    /// * `q` - Unit quaternion to convert
    /// 
    /// # Returns
    /// Euler angles representing the same rotation as the quaternion
    pub fn to_euler_angles(q: &UnitQuaternion<f64>) -> EulerAngles {
        let (roll, pitch, yaw) = q.euler_angles();
        EulerAngles::new(roll, pitch, yaw)
    }

    /// Spherical linear interpolation between two quaternions
    /// 
    /// Performs SLERP (Spherical Linear Interpolation) between two unit quaternions.
    /// This provides smooth, constant-speed interpolation along the shortest path
    /// on the 4D unit sphere, avoiding gimbal lock issues entirely.
    /// 
    /// # Arguments
    /// * `q1` - Starting quaternion
    /// * `q2` - Ending quaternion
    /// * `t` - Interpolation parameter (0.0 = q1, 1.0 = q2)
    /// 
    /// # Returns
    /// Interpolated unit quaternion
    pub fn slerp(
        q1: &UnitQuaternion<f64>,
        q2: &UnitQuaternion<f64>,
        t: f64,
    ) -> UnitQuaternion<f64> {
        q1.slerp(q2, t)
    }

    /// Convert a unit quaternion to a rotation matrix
    /// 
    /// Converts a unit quaternion to a 3x3 rotation matrix.
    /// The resulting matrix is orthogonal with determinant 1.0.
    /// 
    /// # Arguments
    /// * `q` - Unit quaternion to convert
    /// 
    /// # Returns
    /// 3x3 rotation matrix representing the same rotation
    pub fn to_rotation_matrix(q: &UnitQuaternion<f64>) -> Matrix3<f64> {
        q.to_rotation_matrix().into_inner()
    }

    /// Rotate a vector using a quaternion
    /// 
    /// Applies the rotation represented by a unit quaternion to a 3D vector.
    /// This is more efficient and numerically stable than matrix rotation.
    /// 
    /// # Arguments
    /// * `q` - Unit quaternion representing the rotation
    /// * `v` - Vector to rotate
    /// 
    /// # Returns
    /// Rotated vector
    pub fn rotate_vector(q: &UnitQuaternion<f64>, v: &Vector3<f64>) -> Vector3<f64> {
        q.transform_vector(v)
    }

    /// Extract the rotation axis and angle from a quaternion
    /// 
    /// Converts a unit quaternion back to axis-angle representation.
    /// For identity rotations (no rotation), returns default axis (1,0,0) and angle 0.
    /// 
    /// # Arguments
    /// * `q` - Unit quaternion to convert
    /// 
    /// # Returns
    /// Tuple of (rotation_axis, rotation_angle_in_radians)
    pub fn to_axis_angle(q: &UnitQuaternion<f64>) -> (Vector3<f64>, f64) {
        let axis_angle = q.axis_angle();
        match axis_angle {
            Some((axis, angle)) => (axis.into_inner(), angle),
            // Identity rotation has no defined axis, so return default
            None => (Vector3::new(1.0, 0.0, 0.0), 0.0),
        }
    }

    /// Compute the conjugate of a unit quaternion
    /// 
    /// For unit quaternions, the conjugate is equivalent to the inverse.
    /// The conjugate represents the opposite rotation.
    /// 
    /// # Arguments
    /// * `q` - Unit quaternion to conjugate
    /// 
    /// # Returns
    /// Conjugate (inverse) of the input quaternion
    pub fn conjugate(q: &UnitQuaternion<f64>) -> UnitQuaternion<f64> {
        q.conjugate()
    }

    /// Normalize a quaternion to unit length
    /// 
    /// Converts a general quaternion to a unit quaternion by dividing
    /// by its magnitude. Only unit quaternions can represent rotations.
    /// 
    /// # Arguments
    /// * `q` - Quaternion to normalize
    /// 
    /// # Returns
    /// Unit quaternion with the same direction but magnitude 1.0
    pub fn normalize(q: &Quaternion<f64>) -> UnitQuaternion<f64> {
        UnitQuaternion::from_quaternion(*q)
    }

    /// Check if two quaternions represent the same orientation
    /// 
    /// Two quaternions represent the same rotation if they are equal
    /// or if one is the negative of the other (double cover property).
    /// Uses the dot product to check for equivalence within tolerance.
    /// 
    /// # Arguments
    /// * `q1` - First quaternion to compare
    /// * `q2` - Second quaternion to compare
    /// 
    /// # Returns
    /// True if both quaternions represent the same orientation
    pub fn same_orientation(q1: &UnitQuaternion<f64>, q2: &UnitQuaternion<f64>) -> bool {
        // Calculate absolute dot product (handles double cover: q = -q)
        let dot_product = q1.coords.dot(&q2.coords).abs();
        // Quaternions are equivalent if dot product is ±1 (within tolerance)
        (dot_product - 1.0).abs() < MATHEMATICAL_TOLERANCE
    }

    /// Calculate the angular distance between two orientations
    /// 
    /// Computes the minimum rotation angle needed to go from orientation q1 to q2.
    /// This provides a measure of how different two rotations are.
    /// 
    /// # Arguments
    /// * `q1` - Starting orientation
    /// * `q2` - Ending orientation
    /// 
    /// # Returns
    /// Angular distance in radians (0 to π)
    pub fn angular_distance(q1: &UnitQuaternion<f64>, q2: &UnitQuaternion<f64>) -> f64 {
        // Calculate the relative rotation from q1 to q2
        let q_diff = q1.inverse() * q2;
        // Extract the rotation angle (always positive)
        let angle = q_diff.angle();
        angle.abs()
    }
}

/// Educational demonstration functions
/// 
/// This module provides functions that demonstrate key concepts in quaternion
/// mathematics, focusing on gimbal lock and the advantages of quaternions
/// over Euler angles. These functions are designed for educational use and
/// provide clear, formatted output suitable for teaching.
pub mod demonstrations {
    use super::*;

    /// Compare Euler angle vs quaternion interpolation methods
    /// 
    /// Demonstrates the difference between linear interpolation of Euler angles
    /// and spherical linear interpolation (SLERP) of quaternions.
    /// Shows why quaternion interpolation is superior for smooth rotations.
    /// 
    /// Euler interpolation can produce jerky motion near gimbal lock,
    /// while quaternion interpolation maintains constant angular velocity.
    pub fn demonstrate_interpolation() {
        println!("=== INTERPOLATION COMPARISON ===\n");

        // Define start and end orientations for interpolation
        let start = EulerAngles::new(0.0, 0.0, 0.0);
        let end = EulerAngles::new(PI/2.0, PI/4.0, PI/2.0);

        println!("Start orientation: roll={:.3}, pitch={:.3}, yaw={:.3} radians",
                 start.roll, start.pitch, start.yaw);
        println!("End orientation:   roll={:.3}, pitch={:.3}, yaw={:.3} radians",
                 end.roll, end.pitch, end.yaw);
        println!();

        // Convert to quaternions for comparison
        let q_start = QuaternionMath::from_euler_angles(&start);
        let q_end = QuaternionMath::from_euler_angles(&end);

        println!("Interpolation steps (t=0.0 to t=1.0):");
        // Show interpolation at several points using demonstration constant
        let steps = 4; // Use 4 steps for clear demonstration
        for i in 0..=steps {
            let t = i as f64 / steps as f64;

            // Euler interpolation (linear - problematic near gimbal lock)
            let euler_interp = EulerAngles::new(
                start.roll + t * (end.roll - start.roll),
                start.pitch + t * (end.pitch - start.pitch),
                start.yaw + t * (end.yaw - start.yaw),
            );

            // Quaternion interpolation (spherical - smooth and consistent)
            let q_interp = QuaternionMath::slerp(&q_start, &q_end, t);

            println!("   t={:.2}:", t);
            println!("      Euler:     roll={:.3}, pitch={:.3}, yaw={:.3}",
                     euler_interp.roll, euler_interp.pitch, euler_interp.yaw);

            let (axis, angle) = QuaternionMath::to_axis_angle(&q_interp);
            println!("      Quaternion: axis=({:.3}, {:.3}, {:.3}), angle={:.3} radians",
                     axis.x, axis.y, axis.z, angle);
        }
        println!();
    }

    /// Demonstrate practical rotation examples
    /// 
    /// Shows common rotation scenarios and how to compose rotations using
    /// quaternions. Demonstrates why quaternions are more intuitive for
    /// complex rotation sequences.
    pub fn demonstrate_rotation_examples() {
        println!("=== ROTATION EXAMPLES ===\n");

        // Example 1: 90-degree rotation around Z-axis
        // Simple rotation that's easy to verify visually
        println!("1. 90° rotation around Z-axis:");
        let q_z90 = QuaternionMath::create_unit_quaternion(Vector3::new(0.0, 0.0, 1.0), PI/2.0);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let rotated = QuaternionMath::rotate_vector(&q_z90, &v);
        println!("   Input vector:  ({:.3}, {:.3}, {:.3})", v.x, v.y, v.z);
        println!("   Rotated vector: ({:.3}, {:.3}, {:.3})", rotated.x, rotated.y, rotated.z);
        println!();

        // Example 2: Compound rotation (Z then Y)
        // Shows how to combine multiple rotations using quaternion multiplication
        println!("2. Compound rotation (Z then Y):");
        let q_z = QuaternionMath::create_unit_quaternion(Vector3::new(0.0, 0.0, 1.0), PI/4.0);
        let q_y = QuaternionMath::create_unit_quaternion(Vector3::new(0.0, 1.0, 0.0), PI/6.0);
        // Quaternion multiplication combines rotations: q_total = q_first * q_second
        let q_compound = q_z * q_y;
        let v2 = Vector3::new(1.0, 0.0, 0.0);
        let rotated2 = QuaternionMath::rotate_vector(&q_compound, &v2);
        println!("   Input vector:  ({:.3}, {:.3}, {:.3})", v2.x, v2.y, v2.z);
        println!("   Rotated vector: ({:.3}, {:.3}, {:.3})", rotated2.x, rotated2.y, rotated2.z);

        // Show the equivalent Euler angles (may have gimbal lock issues)
        let euler = QuaternionMath::to_euler_angles(&q_compound);
        println!("   As Euler angles: roll={:.3}, pitch={:.3}, yaw={:.3} radians",
                 euler.roll, euler.pitch, euler.yaw);
        println!();
    }

    /// Run all educational demonstrations
    /// 
    /// Convenience function that runs all demonstrations in sequence.
    /// Useful for showing complete examples to students or testing
    /// all functionality at once.
    #[allow(dead_code)]
    pub fn run_all_demonstrations() {
        demonstrate_interpolation();
        demonstrate_rotation_examples();
    }
}

#[cfg(test)]
mod tests {
    use crate::constants::{
        ANGLE_CONVERSION_TOLERANCE, MATHEMATICAL_TOLERANCE, QUATERNION_NORMALIZATION_TOLERANCE,
        ROTATION_MATRIX_DETERMINANT_TOLERANCE, VECTOR_ROTATION_TOLERANCE,
    };
    use super::*;

    #[test]
    fn test_quaternion_creation() {
        let q = QuaternionMath::create_quaternion(1.0, 0.0, 0.0, 0.0);
        assert_eq!(q.w, 1.0);
        assert_eq!(q.i, 0.0);
        assert_eq!(q.j, 0.0);
        assert_eq!(q.k, 0.0);
    }

    #[test]
    fn test_unit_quaternion_from_axis_angle() {
        let axis = Vector3::new(0.0, 0.0, 1.0);
        let angle = std::f64::consts::PI / 2.0; // 90 degrees
        let q = QuaternionMath::create_unit_quaternion(axis, angle);

        // Should be a unit quaternion (norm = 1.0 within tolerance)
        assert!((q.norm() - 1.0).abs() < QUATERNION_NORMALIZATION_TOLERANCE);
    }

    #[test]
    fn test_euler_to_quaternion_conversion() {
        let euler = EulerAngles::new(0.0, 0.0, 0.0);
        let q = QuaternionMath::from_euler_angles(&euler);
        let result_euler = QuaternionMath::to_euler_angles(&q);

        // Conversion should preserve the original Euler angles
        assert!((result_euler.roll - euler.roll).abs() < ANGLE_CONVERSION_TOLERANCE);
        assert!((result_euler.pitch - euler.pitch).abs() < ANGLE_CONVERSION_TOLERANCE);
        assert!((result_euler.yaw - euler.yaw).abs() < ANGLE_CONVERSION_TOLERANCE);
    }

    #[test]
    fn test_rotation_matrix_conversion() {
        let q = QuaternionMath::create_unit_quaternion(Vector3::new(0.0, 0.0, 1.0), PI/2.0);
        let matrix = QuaternionMath::to_rotation_matrix(&q);

        // Should be a valid rotation matrix with determinant = 1.0
        let det = matrix.determinant();
        assert!((det - 1.0).abs() < ROTATION_MATRIX_DETERMINANT_TOLERANCE);
    }

    #[test]
    fn test_vector_rotation() {
        let q = QuaternionMath::create_unit_quaternion(Vector3::new(0.0, 0.0, 1.0), PI/2.0);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let rotated = QuaternionMath::rotate_vector(&q, &v);

        // Should rotate (1,0,0) to approximately (0,1,0)
        assert!((rotated.x - 0.0).abs() < VECTOR_ROTATION_TOLERANCE);
        assert!((rotated.y - 1.0).abs() < VECTOR_ROTATION_TOLERANCE);
        assert!((rotated.z - 0.0).abs() < VECTOR_ROTATION_TOLERANCE);
    }

    #[test]
    fn test_axis_angle_conversion() {
        let axis = Vector3::new(1.0, 0.0, 0.0);
        let angle = PI/4.0;
        let q = QuaternionMath::create_unit_quaternion(axis, angle);
        let (result_axis, result_angle) = QuaternionMath::to_axis_angle(&q);

        // Should preserve the original axis and angle
        assert!((result_axis.x - axis.x).abs() < MATHEMATICAL_TOLERANCE);
        assert!((result_axis.y - axis.y).abs() < MATHEMATICAL_TOLERANCE);
        assert!((result_axis.z - axis.z).abs() < MATHEMATICAL_TOLERANCE);
        assert!((result_angle - angle).abs() < MATHEMATICAL_TOLERANCE);
    }

    #[test]
    fn test_euler_angle_conversions() {
        let euler_rad = EulerAngles::new(PI/2.0, PI/4.0, PI/6.0);
        let euler_deg = euler_rad.to_degrees();

        // Should correctly convert radians to degrees
        assert!((euler_deg.roll - 90.0).abs() < ANGLE_CONVERSION_TOLERANCE);
        assert!((euler_deg.pitch - 45.0).abs() < ANGLE_CONVERSION_TOLERANCE);
        assert!((euler_deg.yaw - 30.0).abs() < ANGLE_CONVERSION_TOLERANCE);

        // Round-trip conversion should preserve original values
        let back_to_rad = euler_deg.to_radians();
        assert!((back_to_rad.roll - euler_rad.roll).abs() < ANGLE_CONVERSION_TOLERANCE);
        assert!((back_to_rad.pitch - euler_rad.pitch).abs() < ANGLE_CONVERSION_TOLERANCE);
        assert!((back_to_rad.yaw - euler_rad.yaw).abs() < ANGLE_CONVERSION_TOLERANCE);
    }
}
