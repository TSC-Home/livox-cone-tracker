use crate::detection::Cone;

/// Coordinate system centered at the sensor (origin).
/// X = forward, Y = left, Z = up (right-hand rule).
#[derive(Debug, Clone)]
pub struct CoordinateSystem {
    pub cones: Vec<ConePosition>,
}

/// A cone's position in the local coordinate system.
#[derive(Debug, Clone, Copy)]
pub struct ConePosition {
    pub x: f32,
    pub y: f32,
    pub distance: f32,
    pub angle_deg: f32,
    pub height: f32,
}

impl CoordinateSystem {
    pub fn new() -> Self {
        Self { cones: Vec::new() }
    }

    /// Update cone positions from detection results.
    /// The sensor is at origin (0,0,0), looking along +X.
    pub fn update(&mut self, detected: &[Cone]) {
        self.cones.clear();
        self.cones.reserve(detected.len());

        for cone in detected {
            let distance = (cone.x * cone.x + cone.y * cone.y).sqrt();
            let angle_deg = cone.y.atan2(cone.x).to_degrees();

            self.cones.push(ConePosition {
                x: cone.x,
                y: cone.y,
                distance,
                angle_deg,
                height: cone.height,
            });
        }

        // Sort by distance for consistent ordering
        self.cones.sort_by(|a, b| {
            a.distance
                .partial_cmp(&b.distance)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    /// Print cone positions to stdout.
    pub fn print_positions(&self) {
        if self.cones.is_empty() {
            return;
        }
        println!("--- Detected Cones ({}) ---", self.cones.len());
        for (i, c) in self.cones.iter().enumerate() {
            println!(
                "  Cone {}: pos=({:.2}m, {:.2}m) dist={:.2}m angle={:.1}° h={:.2}m",
                i + 1,
                c.x,
                c.y,
                c.distance,
                c.angle_deg,
                c.height
            );
        }
    }
}
