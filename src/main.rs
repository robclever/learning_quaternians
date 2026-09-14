mod quaternion;
mod visualization;

fn main() {
    println!("Learning Quaternions - Interactive Visualization Tool");
    println!("=====================================================");

    // Initialize the quaternion system
    quaternion::initialize();

    // Start the visualization (placeholder for now)
    visualization::start();

    println!("Quaternion visualization system initialized successfully!");
}
