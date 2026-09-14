@echo off
REM Learning Quaternians - Project Setup Script for Windows
REM This script sets up the development environment for the quaternion visualization project

echo Setting up Learning Quaternians development environment...
echo ==========================================================

REM Check if Rust is installed
where rustc >nul 2>nul
if %errorlevel% neq 0 (
    echo Rust is not installed. Please install Rust from https://rustup.rs/
    echo After installing Rust, run this script again.
    pause
    exit /b 1
) else (
    echo Rust is already installed
    rustc --version
    cargo --version
)

REM Update Rust to latest stable
echo Updating Rust to latest stable version...
rustup update stable

REM Install required targets and components
echo Installing required Rust components...
rustup component add rustfmt
rustup component add clippy

REM Install useful development tools
echo Installing development tools...
cargo install cargo-watch 2>nul
cargo install cargo-audit 2>nul

REM Check if we're in the right directory
if not exist "Cargo.toml" (
    echo Error: Cargo.toml not found. Please run this script from the project root.
    pause
    exit /b 1
)

REM Build the project
echo Building project...
cargo build

REM Run tests to verify setup
echo Running tests...
cargo test

echo.
echo Setup complete! You can now:
echo   - Run the project: cargo run
echo   - Run tests: cargo test
echo   - Check code formatting: cargo fmt -- --check
echo   - Run linter: cargo clippy
echo   - Build for release: cargo build --release
echo.
echo For development with auto-reload:
echo   cargo watch -x run
echo.
pause