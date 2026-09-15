//! Learning Quaternions - interactive educational visualization tool
//!
//! Running the binary does three things:
//!
//! 1. prints the quaternion and gimbal-lock demonstrations to the terminal,
//! 2. builds the gimbal-lock visualization scene with the Rust math library,
//! 3. writes that scene out as a self-contained HTML page and reports where.
//!
//! Command line flags:
//!
//! * `--no-visualize` - skip generating the HTML page
//! * `--open` - open the generated page in the default browser

mod constants;
mod gimbal_lock;
mod quaternion;
mod visualization;

use gimbal_lock::demonstrations as gimbal_lock_demonstrations;
use quaternion::demonstrations as quaternion_demonstrations;

fn main() {
    let arguments: Vec<String> = std::env::args().skip(1).collect();
    let skip_visualization = arguments
        .iter()
        .any(|argument| argument == "--no-visualize");
    let open_page = arguments.iter().any(|argument| argument == "--open");

    println!("Learning Quaternions - Interactive Visualization Tool");
    println!("=====================================================\n");

    // Initialize the quaternion system
    quaternion::initialize();

    // Run educational demonstrations
    gimbal_lock_demonstrations::run_all_demonstrations();
    quaternion_demonstrations::demonstrate_interpolation();
    quaternion_demonstrations::demonstrate_rotation_examples();

    if skip_visualization {
        println!("Skipping visualization generation (--no-visualize).");
        return;
    }

    generate_visualization(open_page);
}

/// Build the gimbal-lock scene, write the page, and report what happened.
fn generate_visualization(open_page: bool) {
    let demo = visualization::build_gimbal_lock_demo();
    let path = visualization::default_output_path();

    match visualization::write_html_file(&demo, &path) {
        Ok(written) => {
            println!(
                "Wrote an interactive visualization of {} frames to:",
                demo.frames.len()
            );
            println!("  {}", written.display());
            println!("Open that file in any browser: it is fully self-contained.\n");

            if open_page {
                if let Err(error) = visualization::open_in_browser(&written) {
                    eprintln!("Could not launch a browser automatically: {error}");
                }
            }
        }
        Err(error) => {
            eprintln!(
                "Could not write the visualization to {}: {error}",
                path.display()
            );
        }
    }

    visualization::print_demo_summary(&demo);
}
