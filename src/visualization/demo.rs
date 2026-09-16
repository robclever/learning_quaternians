use super::camera::Camera;
use super::geometry::{angle_between_lines_degrees, round_to, Point3D};
use super::model::{
    Demo, DemoFrame, DemoStatus, EquivalenceRow, Experiment, FrameMetrics, QuaternionFrame, Shape,
};
use super::rig::{
    body_marker_local_shapes, build_labels, build_shapes, project_shape, GimbalRig, RingPlane,
};
use crate::constants::{
    GIMBAL_APPROACH_WARNING_DEGREES, GIMBAL_AXIS_ALIGNMENT_TOLERANCE_DEGREES,
    GIMBAL_AXIS_ALIGNMENT_WARNING_DEGREES, GIMBAL_DEMO_MAX_PITCH_DEGREES, GIMBAL_DEMO_ROLL_DEGREES,
    GIMBAL_DEMO_STEP_DEGREES, GIMBAL_DEMO_YAW_DEGREES, VISUALIZATION_METRIC_DECIMAL_PLACES,
};
use crate::gimbal_lock::{GimbalLockAnalysis, GimbalLockDetector, SingularityType};
use crate::quaternion::{EulerAngles, QuaternionMath};
use nalgebra::{UnitQuaternion, Vector3};

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

#[cfg(test)]
mod tests {
    use super::super::model::PartKind;
    use super::*;
    use crate::constants::{GIMBAL_BODY_SCALE, VISUALIZATION_PROJECTION_DECIMALS};
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
