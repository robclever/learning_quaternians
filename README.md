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

## Project Structure

```
learning_quaternians/
├── Cargo.toml          # Project configuration
├── Makefile            # Development commands
├── setup.sh            # Unix/Mac setup script
├── setup.bat           # Windows setup script
└── src/
    ├── main.rs         # Application entry point
    ├── quaternion.rs   # Quaternion mathematics
    └── visualization.rs # Visualization module
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