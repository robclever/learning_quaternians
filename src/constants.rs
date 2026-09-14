//! Global constants and tolerances for quaternion mathematics
//! 
//! This file contains all mathematical tolerances and configuration constants
//! used throughout the quaternion visualization project. Centralizing these
//! values ensures consistency and makes tuning easier.

/// Mathematical tolerance for floating-point comparisons
/// 
/// Used when comparing floating-point numbers for equality.
/// Two numbers are considered equal if their difference is less than this value.
/// This accounts for floating-point arithmetic imprecision.
pub const MATHEMATICAL_TOLERANCE: f64 = 1e-10;

/// Tolerance for gimbal lock detection
/// 
/// Used to determine if a pitch angle is close enough to ±90 degrees
/// to be considered gimbal lock. Smaller values mean stricter detection.
pub const GIMBAL_LOCK_TOLERANCE: f64 = 1e-6;

/// Tolerance for quaternion normalization validation
/// 
/// Used to verify that quaternions are properly normalized (unit length).
/// A quaternion is considered normalized if its norm is within this tolerance of 1.0.
#[allow(dead_code)]
pub const QUATERNION_NORMALIZATION_TOLERANCE: f64 = 1e-10;

/// Tolerance for rotation matrix validation
/// 
/// Used to verify that a matrix is a proper rotation matrix.
/// The determinant should be within this tolerance of 1.0.
#[allow(dead_code)]
pub const ROTATION_MATRIX_DETERMINANT_TOLERANCE: f64 = 1e-10;

/// Tolerance for vector rotation validation
/// 
/// Used to verify the accuracy of vector rotation operations.
/// Results are considered accurate if within this tolerance of expected values.
#[allow(dead_code)]
pub const VECTOR_ROTATION_TOLERANCE: f64 = 1e-10;

/// Tolerance for angular distance calculations
/// 
/// Used when comparing angular distances between orientations.
/// Smaller values mean more precise angular comparisons.
#[allow(dead_code)]
pub const ANGULAR_DISTANCE_TOLERANCE: f64 = 1e-10;

/// Angle conversion tolerance
/// 
/// Used when converting between radians and degrees to ensure
/// round-trip conversions maintain precision.
#[allow(dead_code)]
pub const ANGLE_CONVERSION_TOLERANCE: f64 = 1e-10;

/// Mathematical constants
/// 
/// Commonly used mathematical constants to avoid magic numbers
/// and improve code readability.
/// Half of PI (π/2) - 90 degrees in radians
/// 
/// Frequently used in gimbal lock detection since gimbal lock
/// occurs at pitch angles of ±90 degrees.
#[allow(dead_code)]
pub const HALF_PI: f64 = std::f64::consts::FRAC_PI_2;

/// Full PI (π) - 180 degrees in radians
/// 
/// Used for angle normalization and rotation calculations.
#[allow(dead_code)]
pub const FULL_PI: f64 = std::f64::consts::PI;

/// Two PI (2π) - 360 degrees in radians
/// 
/// Used for angle wrapping and normalization operations.
#[allow(dead_code)]
pub const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

/// Demonstration configuration
/// 
/// Constants used in educational demonstrations and examples.
/// Number of interpolation steps in demonstrations
/// 
/// Controls the smoothness of interpolation examples.
/// Higher values provide smoother demonstrations but more output.
#[allow(dead_code)]
pub const DEMONSTRATION_INTERPOLATION_STEPS: usize = 4;

/// Decimal places for demonstration output
/// 
/// Controls the precision of numerical output in demonstrations.
/// Balance between readability and precision.
#[allow(dead_code)]
pub const DEMONSTRATION_DECIMAL_PLACES: u32 = 3;