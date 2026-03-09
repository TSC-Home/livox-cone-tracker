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

    #[inline]
    pub fn distance_2d(&self, other: &Point3D) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    #[inline]
    pub fn distance_3d(&self, other: &Point3D) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// Estimated ground plane: z = ax + by + c
#[derive(Debug, Clone, Copy)]
pub struct GroundPlane {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

impl GroundPlane {
    /// Height of a point above the ground plane.
    #[inline]
    pub fn height_above(&self, p: &Point3D) -> f32 {
        p.z - (self.a * p.x + self.b * p.y + self.c)
    }
}

/// Simple ground plane estimation using RANSAC on low-z points.
pub fn estimate_ground(points: &[Point3D], iterations: usize) -> GroundPlane {
    if points.len() < 3 {
        return GroundPlane { a: 0.0, b: 0.0, c: 0.0 };
    }

    let mut best_plane = GroundPlane { a: 0.0, b: 0.0, c: 0.0 };
    let mut best_inliers = 0usize;
    let threshold = 0.05; // 5cm tolerance

    // Use a simple deterministic sampling for speed
    let step = (points.len() / iterations.max(1)).max(1);

    for i in 0..iterations.min(points.len() / 3) {
        let i0 = (i * step * 3) % points.len();
        let i1 = (i * step * 3 + step) % points.len();
        let i2 = (i * step * 3 + step * 2) % points.len();

        let p0 = &points[i0];
        let p1 = &points[i1];
        let p2 = &points[i2];

        // Normal vector via cross product
        let v1 = (p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
        let v2 = (p2.x - p0.x, p2.y - p0.y, p2.z - p0.z);
        let nx = v1.1 * v2.2 - v1.2 * v2.1;
        let ny = v1.2 * v2.0 - v1.0 * v2.2;
        let nz = v1.0 * v2.1 - v1.1 * v2.0;

        if nz.abs() < 0.01 {
            continue; // Skip near-vertical planes
        }

        let a = -nx / nz;
        let b = -ny / nz;
        let c = p0.z - a * p0.x - b * p0.y;
        let plane = GroundPlane { a, b, c };

        let inliers = points
            .iter()
            .filter(|p| plane.height_above(p).abs() < threshold)
            .count();

        if inliers > best_inliers {
            best_inliers = inliers;
            best_plane = plane;
        }
    }

    best_plane
}
