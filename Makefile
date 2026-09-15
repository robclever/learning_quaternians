.PHONY: build run test check fmt clean help setup dev release visualize

# Default target
help:
	@echo "Learning Quaternians - Available commands:"
	@echo ""
	@echo "Setup:"
	@echo "  make setup      - Set up development environment (Unix/Mac)"
	@echo "  make setup-win  - Set up development environment (Windows)"
	@echo ""
	@echo "Development:"
	@echo "  make dev        - Build and run in development mode"
	@echo "  make run        - Run the project"
	@echo "  make build      - Build the project"
	@echo "  make watch      - Run with auto-reload on file changes"
	@echo "  make visualize  - Generate the gimbal lock demo page and open it"
	@echo ""
	@echo "Testing & Quality:"
	@echo "  make test       - Run all tests"
	@echo "  make check      - Run cargo check (fast compilation check)"
	@echo "  make fmt        - Format code"
	@echo "  make lint       - Run clippy linter"
	@echo "  make audit      - Check for security vulnerabilities"
	@echo ""
	@echo "Release:"
	@echo "  make release    - Build optimized release version"
	@echo ""
	@echo "Maintenance:"
	@echo "  make clean      - Clean build artifacts"
	@echo "  make update     - Update dependencies"

# Setup commands
setup:
	./setup.sh

setup-win:
	setup.bat

# Development commands
dev: build run

run:
	cargo run

build:
	cargo build

watch:
	cargo watch -x run

visualize:
	cargo run -- --open

# Testing and quality
test:
	cargo test

check:
	cargo check

fmt:
	cargo fmt

lint:
	cargo clippy -- -D warnings

audit:
	cargo audit

# Release
release:
	cargo build --release

# Maintenance
clean:
	cargo clean

update:
	cargo update