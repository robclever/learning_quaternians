# learning_quaternians

# TODO - 
1. Determine what lanaguage is best for this activity
2. Setup Shell of project
3. Create mathematical representations
4. Create visualization tools
5. Create a visualization of quaternians with explanations - I'd like for this to be something I could show students one day as a professor.

# This is why google tells me to use Quaternions:
- No gimbal lock
- Faster performance / compact
- Smooth animation for blending smoothly between two different camera or object angles
- Smooth interpolation

# Why should I use them in sensor fusion / robotics?

# Recommended Development Language: Rust (with WebAssembly)

## Why Rust is the best choice for this project:

### 1. Visualization Excellence
- Rust compiles to WebAssembly, allowing you to create **interactive, browser-based 3D visualizations** that students can access instantly without installing anything
- Libraries like `nalgebra` provide excellent quaternion math support
- Can integrate with web graphics libraries (Three.js, wgpu) for stunning 3D renders

### 2. Educational Accessibility
- Web-based visualizations mean students just need a browser - no complex setup
- You can host it on GitHub Pages for free, making it easily shareable
- Interactive elements (sliders, rotation controls) work naturally in browsers

### 3. Performance + Safety
- Rust's performance ensures smooth real-time 3D rendering even on complex quaternion operations
- Memory safety prevents crashes during demonstrations
- Satisfies the preference for Rust/C++ while providing modern tooling

### 4. Professor-Friendly Features
- Create step-by-step animations showing quaternion interpolation (slerp)
- Interactive gimbal lock demonstrations
- Real-time sensor fusion visualizations
- All accessible via a simple URL you can share in class

## Alternative Options:

| Language | Pros | Cons |
|----------|------|------|
| **Python + Manim** | Beautiful math animations (3Blue1Brown style), rapid prototyping | Lower performance, less interactive |
| **C++ + OpenGL** | Maximum performance, mature graphics ecosystem | Complex setup, harder to share with students |
| **JavaScript/Three.js** | Easiest web deployment, great 3D libraries | No static typing, less mathematical rigor |

## Suggested Rust Stack:
- **Math**: `nalgebra` crate for quaternion operations
- **Graphics**: `wgpu` or compile to WASM and use Three.js
- **Web Framework**: `yew` or `seed` for interactive UI elements
- **Deployment**: GitHub Pages (free hosting)

This approach gives you the **Rust performance and safety** you prefer while creating **accessible, interactive visualizations** that will be perfect for teaching students about quaternions.

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

A physical three-ring gimbal rig (outer **yaw**, middle **pitch**, inner
**roll**) carrying a vehicle marker, drawn with an *orthographic* camera so that
two collinear axes really do look collinear on screen.

* Sweep the pitch slider, or press **Play**, or step with ←/→, from 0° to 90°.
* Every ring draws its own rotation axis. As pitch grows, the orange **roll**
  axis swings onto the blue **yaw** axis. The angle between them is exactly
  `90° − |pitch|`, so at pitch = ±90° both rings spin about the same line.
* At the singularity the two axes turn red and pulse, and the page explains that
  rolling and yawing now produce the *same* twist: one degree of freedom is gone.
* The read-out panel shows the Euler angles, the quaternion `[x, y, z, w]`, the
  rotation matrix, the safety factor, the singularity type and the DOF lost.
* The equivalence table lists *completely different* Euler triples at
  pitch = +90° that describe the *same* orientation, because at that pitch the
  attitude depends only on `(yaw − roll)`. Quaternions have no such degeneracy.

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
    └── visualization.rs # Scene building, projection and HTML export
```

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

### Web Compilation (Future)
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