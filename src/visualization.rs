//! Web-based 3D visualization module for quaternion education
//! 
//! This module provides interactive 3D visualizations for demonstrating
//! quaternion concepts, gimbal lock problems, and rotation interpolation.
//! It compiles to WebAssembly for browser-based education.

use wasm_bindgen::prelude::*;
use web_sys::{CanvasRenderingContext2d, HtmlCanvasElement, window};

use crate::quaternion::{EulerAngles, QuaternionMath};
use crate::gimbal_lock::GimbalLockDetector;

/// 3D point for visualization
#[derive(Clone, Copy)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// 3D visualization canvas and rendering context
pub struct Visualization3D {
    canvas: HtmlCanvasElement,
    context: CanvasRenderingContext2d,
    width: f64,
    height: f64,
}
