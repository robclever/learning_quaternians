# Development Guide

This guide will help you get started with developing the Learning Quaternians project.

## Quick Start

### Option 1: Automated Setup (Recommended)

**Linux/macOS:**
```bash
git clone https://github.com/robclever/learning_quaternians.git
cd learning_quaternians
make setup
```

**Windows:**
```cmd
git clone https://github.com/robclever/learning_quaternians.git
cd learning_quaternians
make setup-win
```

### Option 2: Manual Setup

1. **Install Rust** (if not already installed):
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   source $HOME/.cargo/env
   ```

2. **Install required components**:
   ```bash
   rustup component add rustfmt clippy
   rustup update stable
   ```

3. **Install development tools**:
   ```bash
   cargo install cargo-watch cargo-audit
   ```

4. **Build and test**:
   ```bash
   cargo build
   cargo test
   ```

## Project Structure

```
learning_quaternians/
├── Cargo.toml          # Project configuration and dependencies
├── Makefile            # Development commands
├── README.md           # Project overview
├── DEVELOPMENT.md      # This file
├── setup.sh            # Unix/Mac setup script
├── setup.bat           # Windows setup script
├── .gitignore          # Git ignore rules
├── visualizations/     # Generated demo pages (git-ignored)
└── src/
    ├── main.rs         # Application entry point
    ├── constants.rs    # Tolerances, rig geometry and demo configuration
    ├── quaternion.rs   # Quaternion mathematics
    ├── gimbal_lock.rs  # Gimbal lock detection and analysis
    ├── visualization.rs # Visualization module entry point
    └── visualization/
        ├── camera.rs    # Orthographic camera and projection
        ├── geometry.rs  # World/projected points and geometric helpers
        ├── model.rs     # Serializable scene and frame types
        ├── rig.rs       # Nested gimbal geometry
        ├── demo.rs      # Teaching sequences, metrics and summary
        ├── export.rs    # HTML assembly and file/browser operations
        └── assets/      # Embedded HTML, CSS and JavaScript
```

## Development Commands

Use the Makefile for common development tasks:

```bash
# Development
make dev          # Build and run
make run          # Run the project
make watch        # Auto-reload on changes

# Testing and Quality
make test         # Run tests
make check        # Fast compilation check
make fmt          # Format code
make lint         # Run linter
make audit        # Security audit

# Release
make release      # Build optimized release

# Maintenance
make clean        # Clean build artifacts
make update       # Update dependencies
```

## Interactive Visualization

`cargo run` (or `make dev`) prints the maths demonstrations **and** generates
`visualizations/gimbal_lock_demo.html` - a single self-contained page that needs
no server, no CDN and no WebAssembly toolchain.

```bash
cargo run                    # generate the page and print a summary table
cargo run -- --open          # generate and open it in your default browser
make visualize               # same as `cargo run -- --open`
cargo run -- --no-visualize  # skip page generation
```

The page is data plus SVG rendering: every number and every projected vertex is
computed in Rust (`src/visualization.rs`), so a change to `quaternion.rs` or
`gimbal_lock.rs` shows up in the picture on the next run. `visualizations/` is
git-ignored because it is generated output.

Where to change things:

| Location | What it controls |
|----------|------------------|
| `src/constants.rs` (`GIMBAL_*`, `VISUALIZATION_*`) | ring radii, camera angle, sweep step, warning thresholds |
| `GimbalRig` in `src/visualization.rs` | the three stacked ring rotations |
| `build_shapes` / `build_labels` | what is drawn, and in what painter's order |
| `HTML_DOCUMENT_HEAD` / `RENDERER_SCRIPT` | page styling and the math-free browser renderer |

Run `cargo test` after a change: the visualization tests assert the geometric
claims the demo makes (the axis gap equals `90 - |pitch|`, the rig matches the
library's Euler conversion, the exported JSON parses, every point stays inside
the viewport).

## WebAssembly (optional feature)

The browser/WebAssembly path described in the README is wired up as an optional
feature so that native builds, `cargo test` and CI stay dependency-light:

```bash
rustup target add wasm32-unknown-unknown
cargo build --features wasm --target wasm32-unknown-unknown
```

Nothing in the default build needs these dependencies. The scene built by
`build_gimbal_lock_demo()` is renderer-agnostic, so a `yew` or `wgpu` front end
can consume it without touching the mathematics.

## Code Quality

This project enforces code quality through:

- **rustfmt**: Automatic code formatting
- **clippy**: Linting for common mistakes and improvements
- **cargo-audit**: Security vulnerability checking
- **Unit tests**: Comprehensive test coverage

Before committing code, ensure:
```bash
cargo fmt -- --check    # Check formatting
cargo clippy -- -D warnings  # Check linting
cargo test              # Run all tests
```

## Adding Dependencies

To add new dependencies:

1. Edit `Cargo.toml` and add the dependency
2. Run `cargo build` to download and compile
3. Update this guide if the dependency requires special setup

## Web Development (Future)

When ready to develop the web-based visualization:

1. Uncomment web dependencies in `Cargo.toml`
2. Install WASM target: `rustup target add wasm32-unknown-unknown`
3. Install trunk: `cargo install trunk`
4. Use `trunk serve` for development

## Troubleshooting

### Common Issues

**Build fails with linker errors:**
- Ensure you have the standard library: `rustup component add rust-src`

**Permission denied on setup script:**
- Make it executable: `chmod +x setup.sh`

**Tests fail:**
- Ensure all dependencies are up to date: `cargo update`
- Check for breaking changes in recent commits

### Getting Help

- Check the [Rust Book](https://doc.rust-lang.org/book/)
- Visit [Rust by Example](https://doc.rust-lang.org/rust-by-example/)
- Review [nalgebra documentation](https://docs.rs/nalgebra/) for quaternion operations