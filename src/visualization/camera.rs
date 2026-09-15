use super::geometry::{Point3D, ProjectedPoint};
use crate::constants::{
    VISUALIZATION_CAMERA_AZIMUTH_DEGREES, VISUALIZATION_CAMERA_ELEVATION_DEGREES,
    VISUALIZATION_PROJECTION_DECIMALS, VISUALIZATION_VIEWPORT_PADDING_PIXELS,
    VISUALIZATION_VIEWPORT_PIXELS, VISUALIZATION_WORLD_EXTENT,
};

/// Orthographic orbit camera that maps the rig onto the square SVG viewport.
///
/// Orthographic projection is deliberate: with no perspective there is nothing
/// to hide, so two collinear axis lines stay collinear on screen and the
/// gimbal-lock collapse cannot be faked by the camera. The view is a fixed
/// isometric-style three-quarter view (angles come from `constants.rs`).
#[derive(Debug, Clone, Copy)]
pub struct Camera {
    /// Screen +x direction, expressed in world coordinates.
    right: Point3D,
    /// Screen +y (up) direction, expressed in world coordinates.
    up: Point3D,
    /// Direction from the world origin towards the viewer.
    toward_viewer: Point3D,
    /// Viewport size in SVG user units (the page scales it responsively).
    pub(super) viewport_pixels: f64,
    /// Screen pixels per world unit.
    pixels_per_world_unit: f64,
    /// Decimal places kept when exporting projected geometry.
    pub(super) projection_decimals: u32,
}

impl Default for Camera {
    fn default() -> Self {
        Camera::standard()
    }
}

impl Camera {
    /// The camera configuration used by the generated demonstration.
    pub fn standard() -> Self {
        Camera::new(
            VISUALIZATION_CAMERA_AZIMUTH_DEGREES,
            VISUALIZATION_CAMERA_ELEVATION_DEGREES,
            VISUALIZATION_VIEWPORT_PIXELS,
            VISUALIZATION_VIEWPORT_PADDING_PIXELS,
            VISUALIZATION_WORLD_EXTENT,
        )
    }

    /// Build a camera from its orbit angles and viewport geometry.
    ///
    /// # Arguments
    /// * `azimuth_degrees` - Rotation about the world Z (yaw) axis
    /// * `elevation_degrees` - Height above the world XY plane
    /// * `viewport_pixels` - Square viewport size in SVG user units
    /// * `padding_pixels` - Margin kept free around the projected rig
    /// * `world_extent` - Half-height of the visible world region, in world units
    pub fn new(
        azimuth_degrees: f64,
        elevation_degrees: f64,
        viewport_pixels: f64,
        padding_pixels: f64,
        world_extent: f64,
    ) -> Self {
        let azimuth = azimuth_degrees.to_radians();
        let elevation = elevation_degrees.to_radians();
        let (sin_azimuth, cos_azimuth) = azimuth.sin_cos();
        let (sin_elevation, cos_elevation) = elevation.sin_cos();

        // Orthonormal basis: `right` and `up` span the screen plane, and
        // `toward_viewer` completes the right-handed triple.
        let right = Point3D::new(-cos_azimuth, 0.0, sin_azimuth);
        let up = Point3D::new(
            -sin_azimuth * sin_elevation,
            cos_elevation,
            -cos_azimuth * sin_elevation,
        );
        let toward_viewer = Point3D::new(
            sin_azimuth * cos_elevation,
            sin_elevation,
            cos_azimuth * cos_elevation,
        );

        let usable_pixels = (viewport_pixels - 2.0 * padding_pixels).max(1.0);

        Camera {
            right,
            up,
            toward_viewer,
            viewport_pixels,
            pixels_per_world_unit: usable_pixels / (2.0 * world_extent),
            projection_decimals: VISUALIZATION_PROJECTION_DECIMALS,
        }
    }

    /// Project a world point onto the SVG viewport.
    ///
    /// Screen `y` grows downwards (SVG convention) while the camera's `up` is
    /// world-up, hence the sign flip.
    pub fn project(&self, point: &Point3D) -> ProjectedPoint {
        let center = self.viewport_pixels / 2.0;
        ProjectedPoint {
            x: center + self.dot(point, self.right) * self.pixels_per_world_unit,
            y: center - self.dot(point, self.up) * self.pixels_per_world_unit,
            depth: self.dot(point, self.toward_viewer),
        }
    }

    /// Dot product of a world point with one of the camera's basis vectors.
    fn dot(&self, point: &Point3D, axis: Point3D) -> f64 {
        point.x * axis.x + point.y * axis.y + point.z * axis.z
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    /// The camera must be orthonormal: it may rotate and scale the scene, but it
    /// must not shear it, or the gimbal-lock collapse on screen would be a lie.
    #[test]
    fn camera_projection_is_orthographic() {
        let camera = Camera::standard();
        let first = Point3D::new(0.31, -0.72, 0.44);
        let second = Point3D::new(-0.15, 0.28, 0.91);
        let (a, b) = (camera.project(&first), camera.project(&second));
        let scale = camera.pixels_per_world_unit;
        let center = camera.viewport_pixels / 2.0;

        let screen_dot = ((a.x - center) / scale) * ((b.x - center) / scale)
            + ((center - a.y) / scale) * ((center - b.y) / scale)
            + a.depth * b.depth;
        let world_dot = first.x * second.x + first.y * second.y + first.z * second.z;

        assert!((screen_dot - world_dot).abs() < 1e-9);
    }
}
