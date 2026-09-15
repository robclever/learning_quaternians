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

// ---------------------------------------------------------------------------
// Gimbal rig visualization geometry
// ---------------------------------------------------------------------------
// These constants describe the physical gimbal rig that the visualization
// draws. Changing them re-shapes the rendered rig without touching any of the
// rendering code.

/// Number of line segments used to draw each gimbal ring
///
/// Higher values produce smoother circles at the cost of a larger generated
/// visualization file (each segment contributes vertex data).
pub const GIMBAL_RING_SEGMENTS: usize = 48;

/// Radius of the outer (yaw) gimbal ring in world units
///
/// The yaw ring is the largest ring because the other two are mounted inside of
/// it, exactly like a physical gyroscope gimbal.
pub const GIMBAL_YAW_RING_RADIUS: f64 = 1.0;

/// Radius of the middle (pitch) gimbal ring in world units
///
/// Smaller than the yaw ring so the nesting of the three rings stays visible
/// when they are projected into 2D.
pub const GIMBAL_PITCH_RING_RADIUS: f64 = 0.78;

/// Radius of the inner (roll) gimbal ring in world units
///
/// The innermost ring carries the vehicle/body marker.
pub const GIMBAL_ROLL_RING_RADIUS: f64 = 0.56;

/// Half-length of the rotation axis indicator lines in world units
///
/// Each ring gets an axis line through its center showing which axis it spins
/// around. The lines are drawn slightly longer than the largest ring so they
/// stay visible once two axis lines become collinear at gimbal lock.
pub const GIMBAL_AXIS_EXTENT: f64 = 1.35;

/// Scale factor applied to the body (vehicle) marker geometry
///
/// The marker is drawn inside the innermost ring; this controls how large it
/// appears relative to that ring.
pub const GIMBAL_BODY_SCALE: f64 = 0.5;

// ---------------------------------------------------------------------------
// Gimbal lock demonstration configuration
// ---------------------------------------------------------------------------

/// Pitch angle increment (degrees) between rendered demonstration frames
///
/// Controls the smoothness of the "approaching gimbal lock" animation. Smaller
/// values give smoother motion and a larger generated file.
pub const GIMBAL_DEMO_STEP_DEGREES: f64 = 2.0;

/// Final pitch angle (degrees) of the demonstration sweep
///
/// The sweep runs from 0° to this value. 90° is the exact gimbal lock
/// singularity for the ZYX Euler convention used by `QuaternionMath`.
pub const GIMBAL_DEMO_MAX_PITCH_DEGREES: f64 = 90.0;

/// Roll angle (degrees) held constant during the demonstration sweep
///
/// A non-zero roll (together with `GIMBAL_DEMO_YAW_DEGREES`) makes the
/// roll/yaw degeneracy at pitch = 90° easy to see.
pub const GIMBAL_DEMO_ROLL_DEGREES: f64 = 30.0;

/// Yaw angle (degrees) held constant during the demonstration sweep
///
/// See `GIMBAL_DEMO_ROLL_DEGREES`.
pub const GIMBAL_DEMO_YAW_DEGREES: f64 = 20.0;

/// Pitch distance (degrees) from the singularity at which a warning is raised
///
/// Within this many degrees of pitch = ±90° the visualization switches to its
/// "approaching gimbal lock" state so students get a warning before the
/// control axis fully collapses.
pub const GIMBAL_APPROACH_WARNING_DEGREES: f64 = 15.0;

/// Angle (degrees) below which two rotation axes are reported as collinear
///
/// When the roll axis and the yaw axis are within this angle of each other the
/// gimbal rig has lost a degree of freedom and the visualization highlights the
/// two collapsed axes.
pub const GIMBAL_AXIS_ALIGNMENT_TOLERANCE_DEGREES: f64 = 1.0;

/// Angle (degrees) below which the visualization warns that two axes are
/// becoming collinear
///
/// Slightly larger than `GIMBAL_AXIS_ALIGNMENT_TOLERANCE_DEGREES` so the
/// visualization can highlight axes that are *about* to collapse.
pub const GIMBAL_AXIS_ALIGNMENT_WARNING_DEGREES: f64 = 5.0;

// ---------------------------------------------------------------------------
// Visualization / camera configuration
// ---------------------------------------------------------------------------

/// Size of the square visualization viewport in pixels
///
/// Values are in SVG user units. The generated page scales the viewport
/// responsively, so this only controls the internal drawing resolution.
pub const VISUALIZATION_VIEWPORT_PIXELS: f64 = 720.0;

/// Margin (pixels) kept free around the projected rig
///
/// Prevents the outermost ring and its axis line from touching the edge of the
/// viewport.
pub const VISUALIZATION_VIEWPORT_PADDING_PIXELS: f64 = 48.0;

/// Half-height of the visible world region in world units
///
/// The camera maps this many world units onto the padded half-height of the
/// viewport. Must be larger than `GIMBAL_AXIS_EXTENT` so nothing is clipped.
pub const VISUALIZATION_WORLD_EXTENT: f64 = 1.6;

/// Camera azimuth (degrees) used for the default isometric-style view
///
/// Rotates the camera around the world Z axis (the yaw axis).
pub const VISUALIZATION_CAMERA_AZIMUTH_DEGREES: f64 = -38.0;

/// Camera elevation (degrees) used for the default isometric-style view
///
/// Tilts the camera above the world XY plane so the pitch ring is seen as an
/// ellipse rather than a flat line.
pub const VISUALIZATION_CAMERA_ELEVATION_DEGREES: f64 = 20.0;

/// Decimal places kept when exporting projected geometry
///
/// Projected coordinates are rounded before being embedded in the generated
/// page. Two decimals are well below one screen pixel, so this only shrinks the
/// file size.
pub const VISUALIZATION_PROJECTION_DECIMALS: u32 = 2;

/// Decimal places kept when exporting angles and quaternion components
///
/// Three decimals match `DEMONSTRATION_DECIMAL_PLACES` so the numbers shown in
/// the browser agree with the terminal output.
pub const VISUALIZATION_METRIC_DECIMAL_PLACES: u32 = 3;

/// File name of the generated visualization
///
/// Written into the `visualizations/` directory at the project root.
pub const VISUALIZATION_OUTPUT_FILE_NAME: &str = "gimbal_lock_demo.html";
