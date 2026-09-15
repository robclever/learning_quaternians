# learning_quaternians

[Open the interactive quaternion lesson](https://robclever.github.io/learning_quaternians/)

The demo link becomes available after the first successful GitHub Pages deployment.

# This is why google tells me to use Quaternions:
- No gimbal lock
- Faster performance / compact
- Smooth animation for blending smoothly between two different camera or object angles
- Smooth interpolation

# Why should I use them in sensor fusion / robotics?

# Getting Started

## Quick Setup

### Linux/macOS
```bash
git clone https://github.com/robclever/learning_quaternians.git
cd learning_quaternians
make setup
```

### Windows
```cmd
git clone https://github.com/robclever/learning_quaternians.git
cd learning_quaternians
make setup-win
```

## Development Commands

```bash
make dev          # Build and run
make test         # Run tests
make fmt          # Format code
make lint         # Run linter
make release      # Build release version
```

See [DEVELOPMENT.md](DEVELOPMENT.md) for detailed development instructions.

## Interactive Visualization: Gimbal Lock Demo

`cargo run` prints the maths demonstrations **and** generates an interactive 3D
visualization of gimbal lock:

```bash
cargo run                    # print the demos, generate the page, print a summary
cargo run -- --open          # ...and open the page in your default browser
make visualize               # same as `cargo run -- --open`
cargo run -- --no-visualize  # maths demos only
```

Output: `visualizations/gimbal_lock_demo.html` - one self-contained file, no
server, no CDN, no WebAssembly toolchain, works offline and can be shared or
dropped onto GitHub Pages as-is.

### What it shows

The page is a guided lesson with four sections:

1. **What gimbal lock is:** nested Euler controls, aligned axes, and the loss of
   an independent rotation direction.
2. **Four selectable experiments:** pitch up to +90°, pitch down to −90°,
   coupled roll/yaw changes that cancel at +90°, and the same changes at 85°.
   Each experiment supports a slider and Play/Pause. Space and arrow keys work
   when focus is outside an interactive control.
3. **What a quaternion is:** four components, the unit-length constraint,
   axis-angle encoding, composition, SLERP, and the q/−q equivalence.
4. **Quaternion motion through 90°:** an independent slider and animation blend
   rotations from 60° to 120° about Y. The vehicle's body axes stay perpendicular
   in 3D, unlike the nested Euler control axes at gimbal lock.

The readout shows Euler angles, quaternion `[x, y, z, w]`, a rotation matrix,
axis separation, and singularity information. A fixed +90° equivalence table
shows why different Euler triples can describe the same orientation.

The lesson distinguishes an Euler-coordinate singularity from a physical gimbal
mechanism: storing orientation as a quaternion avoids the former but does not
mechanically unlock the latter. A pitch sweep alone can cross 90° with either
representation; the problem is the loss of independent Euler controls there.

### Why it is built this way

All mathematics lives in Rust; the page only draws numbers that were computed
by `quaternion.rs` and `gimbal_lock.rs`, so the picture can never disagree with
the library it teaches:

* `visualization::build_gimbal_lock_demo()` builds a renderer-agnostic scene
  (Euler angles, quaternions, rotation matrices, projected geometry).
* `visualization::Camera` projects that scene orthographically.
* `visualization::render_html()` serialises it into a single HTML file.

`visualizations/` is git-ignored because it is generated output. The browser
dependencies (`yew`, `wasm-bindgen`, `web-sys`, …) are optional behind the
`wasm` feature, so native builds stay dependency-light while this scene layer
stays ready for a WebAssembly front end.

## Project Structure

```
learning_quaternians/
├── Cargo.toml          # Project configuration
├── Makefile            # Development commands
├── setup.sh            # Unix/Mac setup script
├── setup.bat           # Windows setup script
├── visualizations/     # Generated demo pages (git-ignored)
└── src/
    ├── main.rs         # Application entry point
    ├── constants.rs    # Shared tolerances and visualization configuration
    ├── quaternion.rs   # Quaternion mathematics
    ├── gimbal_lock.rs  # Gimbal lock detection and analysis
    ├── visualization.rs # Visualization module entry point and public API
    └── visualization/
        ├── camera.rs    # Orthographic camera configuration and projection
        ├── geometry.rs  # World/projected points and geometric helpers
        ├── model.rs     # Serializable scene, frame, and metric data types
        ├── rig.rs       # Nested gimbal geometry and projected shapes
        ├── demo.rs      # Teaching sequences, metrics, and terminal summary
        ├── export.rs    # HTML assembly, file writing, and browser launching
        └── assets/      # Embedded HTML, CSS, and JavaScript for the page
```

### What each Rust module does

- `main.rs` parses command-line flags, runs the terminal demonstrations, and
  coordinates visualization generation.
- `constants.rs` keeps the shared geometry dimensions, tolerances, camera
  settings, and demo defaults in one place.
- `quaternion.rs` implements quaternion operations, Euler conversions,
  interpolation, and rotation helpers.
- `gimbal_lock.rs` analyzes Euler poses and identifies when the control axes
  become redundant.
- `visualization.rs` is the small public facade for the visualization package;
  it exposes the demo builder, camera, scene types, and HTML export functions.
- `visualization/camera.rs` defines the fixed orthographic camera and maps world
  points into viewport coordinates.
- `visualization/geometry.rs` contains reusable 3D points, projected points,
  angle calculations, and numeric rounding helpers.
- `visualization/model.rs` defines the serializable scene graph: shapes, labels,
  frames, metrics, experiments, and demo metadata.
- `visualization/rig.rs` builds the nested yaw, pitch, and roll rings, axes, and
  vehicle marker for each pose.
- `visualization/demo.rs` assembles the lesson's animation frames and
  experiments, computes their metrics, and prints the terminal summary.
- `visualization/export.rs` renders the scene data into a self-contained HTML
  document, writes it to disk, and can open it in the default browser.

The files under `visualization/assets/` are embedded at compile time. They hold
the page shell (`head.html` and `body.html`), styles (`style.css`), and browser
renderer (`renderer.js`). The browser only draws the numbers exported by Rust;
it does not redo the quaternion or gimbal-lock mathematics.

## Compiling the Project

### Quick Start
```bash
# Development build (fast compilation with debug info)
make build

# Release build (optimized for performance)
make release
```

### Using Cargo Directly
```bash
# Development build
cargo build

# Release build (optimized)
cargo build --release
```

### Running the Project
```bash
# Build and run in one step
cargo run
# or
make dev

# Run compiled binary directly
./target/debug/learning_quaternians    # Development
./target/release/learning_quaternians  # Release
```

### Additional Commands
```bash
# Fast compilation check without producing binary
cargo check

# Clean build artifacts
make clean

# Build and run all tests
cargo test
```

### Web Compilation
When web dependencies are enabled in `Cargo.toml`:
```bash
# Install WASM target
rustup target add wasm32-unknown-unknown

# Build for web deployment
cargo build --target wasm32-unknown-unknown
```

### Build Outputs
- **Development**: `target/debug/learning_quaternians`
- **Release**: `target/release/learning_quaternians`

The development build is recommended for everyday coding and testing, while the release build provides optimized performance for demonstrations and deployment.

## Publishing the visualization with GitHub Pages

The workflow in `.github/workflows/pages.yml` tests the project, runs the Rust
exporter, and publishes the generated HTML as the site's `index.html`.
Generated files can stay git-ignored; GitHub builds them from source.

One-time setup:

1. In this repository on GitHub, open **Settings → Pages**.
2. Under **Build and deployment**, choose **GitHub Actions** as the source.
3. Merge the visualization changes and publishing workflow into `main`.
4. Open **Actions → Publish visualization** and wait for the deployment to finish.
   If the changes were already on `main` when Pages was enabled, use **Run workflow**
   and select `main`.
5. Visit <https://robclever.github.io/learning_quaternians/>.

Subsequent pushes to `main` rebuild and publish the lesson automatically.
The README links to that site; the interactive HTML does not run inside the README.
If deployment is blocked by environment rules, check that the `github-pages`
environment permits deployments from `main`.
