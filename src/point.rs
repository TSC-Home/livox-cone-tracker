/// A 3D point with reflectivity.
#[derive(Debug, Clone, Copy)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub reflectivity: f32,
}

impl Point3D {
    #[inline]
    pub fn from_raw(raw: &[f32; 4]) -> Self {
        Self {
            x: raw[0],
            y: raw[1],
            z: raw[2],
            reflectivity: raw[3],
        }
    }

    /// Returns true if all coordinates are finite (not NaN, not inf).
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}
