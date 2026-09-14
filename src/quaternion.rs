use nalgebra::{Quaternion, Unit, UnitQuaternion, Vector3};

pub fn initialize() {
    println!("Quaternion math module initialized");
}

#[allow(dead_code)]
pub struct QuaternionMath;

#[allow(dead_code)]
impl QuaternionMath {
    pub fn create_quaternion(w: f64, x: f64, y: f64, z: f64) -> Quaternion<f64> {
        Quaternion::new(w, x, y, z)
    }

    pub fn create_unit_quaternion(axis: Vector3<f64>, angle: f64) -> UnitQuaternion<f64> {
        let unit_axis = Unit::new_normalize(axis);
        UnitQuaternion::from_axis_angle(&unit_axis, angle)
    }

    pub fn slerp(
        q1: &UnitQuaternion<f64>,
        q2: &UnitQuaternion<f64>,
        t: f64,
    ) -> UnitQuaternion<f64> {
        q1.slerp(q2, t)
    }

    pub fn to_euler_angles(q: &UnitQuaternion<f64>) -> (f64, f64, f64) {
        q.euler_angles()
    }
}

#[cfg(test)]
mod tests {
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

        // Should be a unit quaternion
        assert!((q.norm() - 1.0).abs() < 1e-10);
    }
}
