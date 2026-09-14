#!/bin/bash

# Learning Quaternians - Project Setup Script
# This script sets up the development environment for the quaternion visualization project

set -e

echo "Setting up Learning Quaternians development environment..."
echo "=========================================================="

# Check if Rust is installed
if ! command -v rustc &> /dev/null; then
    echo "Rust is not installed. Installing Rust..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source $HOME/.cargo/env
    echo "Rust installed successfully!"
else
    echo "Rust is already installed"
    rustc --version
    cargo --version
fi

# Update Rust to latest stable
echo "Updating Rust to latest stable version..."
rustup update stable

# Install required targets and components
echo "Installing required Rust components..."
rustup component add rustfmt
rustup component add clippy

# Install useful development tools
echo "Installing development tools..."
cargo install cargo-watch 2>/dev/null || echo "cargo-watch already installed"
cargo install cargo-audit 2>/dev/null || echo "cargo-audit already installed"

# Check if we're in the right directory
if [ ! -f "Cargo.toml" ]; then
    echo "Error: Cargo.toml not found. Please run this script from the project root."
    exit 1
fi

# Build the project
echo "Building project..."
cargo build

# Run tests to verify setup
echo "Running tests..."
cargo test

echo ""
echo "Setup complete! You can now:"
echo "  - Run the project: cargo run"
echo "  - Run tests: cargo test"
echo "  - Check code formatting: cargo fmt -- --check"
echo "  - Run linter: cargo clippy"
echo "  - Build for release: cargo build --release"
echo ""
echo "For development with auto-reload:"
echo "  cargo watch -x run"