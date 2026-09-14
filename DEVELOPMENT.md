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
└── src/
    ├── main.rs         # Application entry point
    ├── quaternion.rs   # Quaternion mathematics
    └── visualization.rs # Visualization module
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