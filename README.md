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