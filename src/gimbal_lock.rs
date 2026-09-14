//! Gimbal lock detection and analysis module
//! 
//! This module provides comprehensive gimbal lock detection, analysis,
//! and educational demonstrations. Gimbal lock is a fundamental problem
//! with Euler angle representations that occurs when two rotation axes
//! become aligned, causing a loss of one degree of freedom.
//! 
//! Key features:
//! - Gimbal lock detection with configurable tolerance
//! - Detailed singularity analysis and classification
//! - Educational demonstrations showing gimbal lock problems
//! - Comparison between problematic Euler angles and robust quaternions

use crate::constants::GIMBAL_LOCK_TOLERANCE;
use crate::quaternion::{EulerAngles, QuaternionMath};
use std::f64::consts::PI;

/// Analysis result for gimbal lock detection
/// 
/// Contains detailed information about whether a given Euler angle
/// configuration results in gimbal lock, including the type of singularity
/// and how many degrees of freedom are lost.
#[derive(Debug)]
pub struct GimbalLockAnalysis {
    /// True if this configuration results in gimbal lock
    #[allow(dead_code)]
    pub is_gimbal_lock: bool,
    /// Type of singularity detected
    #[allow(dead_code)]
    pub singularity_type: SingularityType,
    /// Human-readable description of the gimbal lock situation
    pub description: String,
    /// Number of degrees of freedom lost (0, 1, or 2)
    pub loss_of_degree_of_freedom: usize,
}

/// Types of gimbal lock singularities
/// 
/// Represents the different types of singularities that can occur
/// in Euler angle representations. The most common are pitch = ±90°.
#[derive(Debug)]
pub enum SingularityType {
    /// No singularity - all three rotation axes are independent
    None,
    /// Pitch angle is at +90° (looking straight up)
    PitchUp,
    /// Pitch angle is at -90° (looking straight down)
    PitchDown,
    /// Other types of singularities (rare in practice)
    #[allow(dead_code)]
    General,
}

/// Gimbal lock detection and analysis functions
/// 
/// This struct contains static methods for detecting and analyzing
/// gimbal lock conditions in Euler angle representations.
#[allow(dead_code)]
pub struct GimbalLockDetector;

#[allow(dead_code)]
impl GimbalLockDetector {
    /// Analyze Euler angles for gimbal lock conditions
    /// 
    /// Detects whether the given Euler angles result in gimbal lock
    /// and provides detailed analysis of the singularity.
    /// Gimbal lock occurs when pitch = ±90°, causing roll and yaw
    /// axes to align and losing one degree of freedom.
    /// 
    /// # Arguments
    /// * `euler` - Euler angles to analyze
    /// 
    /// # Returns
    /// Detailed analysis of gimbal lock conditions
    /// 
    /// # Examples
    /// ```
    /// use learning_quaternians::gimbal_lock::GimbalLockDetector;
    /// use learning_quaternians::quaternion::EulerAngles;
    /// 
    /// // Normal case - no gimbal lock
    /// let normal = EulerAngles::new(0.1, 0.2, 0.3);
    /// let analysis = GimbalLockDetector::analyze_gimbal_lock(&normal);
    /// assert!(!analysis.is_gimbal_lock);
    /// 
    /// // Gimbal lock case
    /// let gimbal = EulerAngles::new(0.0, std::f64::consts::PI/2.0, 0.0);
    /// let analysis = GimbalLockDetector::analyze_gimbal_lock(&gimbal);
    /// assert!(analysis.is_gimbal_lock);
    /// ```
    pub fn analyze_gimbal_lock(euler: &EulerAngles) -> GimbalLockAnalysis {
        // Convert pitch to degrees for easier comparison with ±90°
        let pitch_deg = euler.pitch.to_degrees();

        // Check for gimbal lock at pitch = +90° (looking straight up)
        if (pitch_deg - 90.0).abs() < GIMBAL_LOCK_TOLERANCE {
            GimbalLockAnalysis {
                is_gimbal_lock: true,
                singularity_type: SingularityType::PitchUp,
                description: "Gimbal lock at pitch = +90°. Roll and yaw axes become aligned, losing one degree of freedom.".to_string(),
                loss_of_degree_of_freedom: 1,
            }
        }
        // Check for gimbal lock at pitch = -90° (looking straight down)
        else if (pitch_deg + 90.0).abs() < GIMBAL_LOCK_TOLERANCE {
            GimbalLockAnalysis {
                is_gimbal_lock: true,
                singularity_type: SingularityType::PitchDown,
                description: "Gimbal lock at pitch = -90°. Roll and yaw axes become aligned, losing one degree of freedom.".to_string(),
                loss_of_degree_of_freedom: 1,
            }
        }
        // No gimbal lock detected
        else {
            GimbalLockAnalysis {
                is_gimbal_lock: false,
                singularity_type: SingularityType::None,
                description: "No gimbal lock detected. All three axes are independent.".to_string(),
                loss_of_degree_of_freedom: 0,
            }
        }
    }

    /// Check if two different Euler angle sets produce the same rotation
    /// 
    /// At gimbal lock, different combinations of roll and yaw can produce
    /// the same final orientation. This function demonstrates that problem
    /// by converting both to quaternions and comparing them.
    /// 
    /// # Arguments
    /// * `euler1` - First Euler angle configuration
    /// * `euler2` - Second Euler angle configuration
    /// 
    /// # Returns
    /// True if both Euler angle sets produce the same rotation
    pub fn demonstrate_equivalence(euler1: &EulerAngles, euler2: &EulerAngles) -> bool {
        let q1 = QuaternionMath::from_euler_angles(euler1);
        let q2 = QuaternionMath::from_euler_angles(euler2);
        QuaternionMath::same_orientation(&q1, &q2)
    }

    /// Calculate how close Euler angles are to gimbal lock
    /// 
    /// Returns a value from 0.0 (at gimbal lock) to 1.0 (maximum distance
    /// from gimbal lock). This can be used to warn users when they're
    /// approaching problematic configurations.
    /// 
    /// # Arguments
    /// * `euler` - Euler angles to check
    /// 
    /// # Returns
    /// Safety factor from 0.0 (gimbal lock) to 1.0 (safe)
    pub fn gimbal_lock_safety_factor(euler: &EulerAngles) -> f64 {
        let pitch_deg = euler.pitch.to_degrees();
        // Distance from nearest singularity (±90°)
        let distance_from_singularity = ((pitch_deg - 90.0).abs()).min((pitch_deg + 90.0).abs());
        // Normalize to 0.0-1.0 range (90° = maximum safety)
        distance_from_singularity / 90.0
    }
}

/// Educational demonstrations for gimbal lock
/// 
/// This module provides functions that demonstrate gimbal lock concepts
/// for educational purposes. These functions produce formatted output
/// suitable for teaching and presentations.
pub mod demonstrations {
    use super::*;

    /// Demonstrate gimbal lock with different Euler angle configurations
    /// 
    /// Shows three key cases:
    /// 1. Normal rotation without gimbal lock
    /// 2. Gimbal lock at pitch = +90° (looking straight up)
    /// 3. Different Euler angles that produce the same rotation at gimbal lock
    /// 
    /// This demonstration clearly shows why Euler angles can be problematic
    /// and why quaternions are preferred for smooth rotations.
    pub fn demonstrate_gimbal_lock() {
        println!("=== GIMBAL LOCK DEMONSTRATION ===\n");

        // Case 1: Normal rotation without gimbal lock
        // All three axes (roll, pitch, yaw) are independent and provide full 3 DOF
        println!("1. Normal Case (No Gimbal Lock):");
        let normal_euler = EulerAngles::new(0.1, 0.2, 0.3);
        let analysis = GimbalLockDetector::analyze_gimbal_lock(&normal_euler);
        println!("   Euler angles: roll={:.3}, pitch={:.3}, yaw={:.3} radians",
                 normal_euler.roll, normal_euler.pitch, normal_euler.yaw);
        println!("   Analysis: {}", analysis.description);
        println!("   Safety factor: {:.3}", GimbalLockDetector::gimbal_lock_safety_factor(&normal_euler));
        println!();

        // Case 2: Gimbal lock at pitch = +90°
        // When pitch is +90°, roll and yaw axes become aligned, losing 1 DOF
        println!("2. Gimbal Lock Case (Pitch = +90°):");
        let gimbal_euler = EulerAngles::new(0.5, PI/2.0, 0.3);
        let analysis = GimbalLockDetector::analyze_gimbal_lock(&gimbal_euler);
        println!("   Euler angles: roll={:.3}, pitch={:.3}, yaw={:.3} radians",
                 gimbal_euler.roll, gimbal_euler.pitch, gimbal_euler.yaw);
        println!("   Analysis: {}", analysis.description);
        println!("   DOF Lost: {}", analysis.loss_of_degree_of_freedom);
        println!("   Safety factor: {:.3}", GimbalLockDetector::gimbal_lock_safety_factor(&gimbal_euler));
        println!();

        // Case 3: Show equivalence at gimbal lock
        // At gimbal lock, different roll/yaw combinations produce the same rotation
        println!("3. Gimbal Lock Equivalence:");
        let euler1 = EulerAngles::new(0.5, PI/2.0, 0.3);
        let euler2 = EulerAngles::new(0.8, PI/2.0, 0.0);

        println!("   Euler angles 1: roll={:.3}, pitch={:.3}, yaw={:.3}", euler1.roll, euler1.pitch, euler1.yaw);
        println!("   Euler angles 2: roll={:.3}, pitch={:.3}, yaw={:.3}", euler2.roll, euler2.pitch, euler2.yaw);
        println!("   Quaternions represent same orientation: {}", 
                 GimbalLockDetector::demonstrate_equivalence(&euler1, &euler2));
        println!();
    }

    /// Demonstrate approaching gimbal lock
    /// 
    /// Shows how the safety factor changes as Euler angles approach
    /// gimbal lock configurations. This helps users understand when
    /// they're getting into dangerous territory.
    pub fn demonstrate_gimbal_lock_approach() {
        println!("=== APPROACHING GIMBAL LOCK ===\n");

        let test_angles = [80.0_f64, 85.0, 89.0, 89.9, 90.0];
        
        for &pitch_deg in &test_angles {
            let euler = EulerAngles::new(0.5, pitch_deg.to_radians(), 0.3);
            let safety = GimbalLockDetector::gimbal_lock_safety_factor(&euler);
            let analysis = GimbalLockDetector::analyze_gimbal_lock(&euler);
            
            println!("Pitch = {:5.1}°: Safety = {:.6}, Gimbal Lock = {}", 
                     pitch_deg, safety, analysis.is_gimbal_lock);
        }
        println!();
    }

    /// Run all gimbal lock demonstrations
    /// 
    /// Convenience function that runs all gimbal lock demonstrations
    /// in sequence. Useful for educational presentations or testing.
    pub fn run_all_demonstrations() {
        demonstrate_gimbal_lock();
        demonstrate_gimbal_lock_approach();
    }
}

#[cfg(test)]
mod tests {
    use crate::constants::MATHEMATICAL_TOLERANCE;
    use super::*;

    #[test]
    fn test_gimbal_lock_detection() {
        // Test normal case - no gimbal lock
        let normal_euler = EulerAngles::new(0.1, 0.2, 0.3);
        let analysis = GimbalLockDetector::analyze_gimbal_lock(&normal_euler);
        assert!(!analysis.is_gimbal_lock);
        assert!(matches!(analysis.singularity_type, SingularityType::None));
        assert_eq!(analysis.loss_of_degree_of_freedom, 0);

        // Test gimbal lock at pitch = 90° (looking straight up)
        let gimbal_euler = EulerAngles::new(0.0, PI/2.0, 0.0);
        let analysis = GimbalLockDetector::analyze_gimbal_lock(&gimbal_euler);
        assert!(analysis.is_gimbal_lock);
        assert!(matches!(analysis.singularity_type, SingularityType::PitchUp));
        assert_eq!(analysis.loss_of_degree_of_freedom, 1);

        // Test gimbal lock at pitch = -90° (looking straight down)
        let gimbal_euler = EulerAngles::new(0.0, -PI/2.0, 0.0);
        let analysis = GimbalLockDetector::analyze_gimbal_lock(&gimbal_euler);
        assert!(analysis.is_gimbal_lock);
        assert!(matches!(analysis.singularity_type, SingularityType::PitchDown));
        assert_eq!(analysis.loss_of_degree_of_freedom, 1);
    }

    #[test]
    fn test_gimbal_lock_safety_factor() {
        // Test maximum safety (no gimbal lock danger)
        let safe_euler = EulerAngles::new(0.0, 0.0, 0.0);
        let safety = GimbalLockDetector::gimbal_lock_safety_factor(&safe_euler);
        assert!((safety - 1.0).abs() < MATHEMATICAL_TOLERANCE);

        // Test zero safety (at gimbal lock)
        let gimbal_euler = EulerAngles::new(0.0, PI/2.0, 0.0);
        let safety = GimbalLockDetector::gimbal_lock_safety_factor(&gimbal_euler);
        assert!(safety.abs() < MATHEMATICAL_TOLERANCE);

        // Test intermediate value
        let mid_euler = EulerAngles::new(0.0, PI/4.0, 0.0); // 45° pitch
        let safety = GimbalLockDetector::gimbal_lock_safety_factor(&mid_euler);
        assert!((safety - 0.5).abs() < MATHEMATICAL_TOLERANCE);
    }

    #[test]
    fn test_gimbal_lock_equivalence() {
        // Test that the equivalence function works correctly
        // Same angles should be equivalent
        let euler1 = EulerAngles::new(0.5, PI/2.0, 0.3);
        let euler2 = EulerAngles::new(0.5, PI/2.0, 0.3);
        
        assert!(GimbalLockDetector::demonstrate_equivalence(&euler1, &euler2));

        // Different configurations should not be equivalent
        let euler3 = EulerAngles::new(0.1, 0.2, 0.3);
        let euler4 = EulerAngles::new(0.4, 0.5, 0.6);
        
        assert!(!GimbalLockDetector::demonstrate_equivalence(&euler3, &euler4));
    }
}