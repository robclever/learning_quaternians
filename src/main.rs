mod constants;
mod gimbal_lock;
mod quaternion;
mod visualization;

use gimbal_lock::demonstrations as gimbal_lock_demonstrations;
use quaternion::demonstrations as quaternion_demonstrations;

fn main() {
    println!("Learning Quaternions - Interactive Visualization Tool");
    println!("=====================================================\n");

    // Initialize the quaternion system
    quaternion::initialize();

    // Run educational demonstrations
    gimbal_lock_demonstrations::run_all_demonstrations();
    quaternion_demonstrations::demonstrate_interpolation();
    quaternion_demonstrations::demonstrate_rotation_examples();

    println!("Quaternion mathematics module ready for visualization development.");
    println!("Next steps: Implement 3D visualization tools to show these concepts interactively.");
}
