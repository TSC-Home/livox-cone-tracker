use crate::detection::Cone;

/// How many detection cycles a cone survives without being re-detected.
const MAX_MISSED: u32 = 8;
/// Max distance (meters) to match a new detection to an existing tracked cone.
const MATCH_RADIUS: f32 = 0.40;
/// Smoothing factor: 0.0 = no smoothing, 1.0 = ignore new data.
const SMOOTH: f32 = 0.6;

#[derive(Debug, Clone)]
pub struct CoordinateSystem {
    tracked: Vec<TrackedCone>,
    pub cones: Vec<ConePosition>,
}

#[derive(Debug, Clone, Copy)]
struct TrackedCone {
    x: f32,
    y: f32,
    height: f32,
    missed: u32,
    hit_count: u32,
}

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
        Self {
            tracked: Vec::new(),
            cones: Vec::new(),
        }
    }

    pub fn update(&mut self, detected: &[Cone]) {
        // Mark all tracked cones as missed this cycle
        for t in &mut self.tracked {
            t.missed += 1;
        }

        let tracked_count = self.tracked.len();
        let mut used = vec![false; tracked_count];

        // First pass: match detections to existing tracked cones
        // Collect new cones separately to avoid modifying tracked during iteration
        let mut new_cones: Vec<TrackedCone> = Vec::new();

        for det in detected {
            let mut best_idx = None;
            let mut best_dist = MATCH_RADIUS;

            for (i, t) in self.tracked[..tracked_count].iter().enumerate() {
                if used[i] { continue; }
                let d = ((det.x - t.x).powi(2) + (det.y - t.y).powi(2)).sqrt();
                if d < best_dist {
                    best_dist = d;
                    best_idx = Some(i);
                }
            }

            if let Some(i) = best_idx {
                let t = &mut self.tracked[i];
                t.x = t.x * SMOOTH + det.x * (1.0 - SMOOTH);
                t.y = t.y * SMOOTH + det.y * (1.0 - SMOOTH);
                t.height = t.height * SMOOTH + det.height * (1.0 - SMOOTH);
                t.missed = 0;
                t.hit_count += 1;
                used[i] = true;
            } else {
                new_cones.push(TrackedCone {
                    x: det.x,
                    y: det.y,
                    height: det.height,
                    missed: 0,
                    hit_count: 1,
                });
            }
        }

        // Add new cones after iteration is done
        self.tracked.extend(new_cones);

        // Remove cones that have been missing too long
        self.tracked.retain(|t| t.missed < MAX_MISSED);

        // Build output: only cones seen at least twice (filters single-frame noise)
        self.cones.clear();
        for t in &self.tracked {
            if t.hit_count < 2 {
                continue;
            }
            let distance = (t.x * t.x + t.y * t.y).sqrt();
            let angle_deg = t.y.atan2(t.x).to_degrees();
            self.cones.push(ConePosition {
                x: t.x,
                y: t.y,
                distance,
                angle_deg,
                height: t.height,
            });
        }

        self.cones.sort_by(|a, b| {
            a.distance.partial_cmp(&b.distance).unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    pub fn print_positions(&self) {
        if self.cones.is_empty() {
            return;
        }
        println!("--- Tracked Cones ({}) ---", self.cones.len());
        for (i, c) in self.cones.iter().enumerate() {
            println!(
                "  Cone {}: ({:.2}m, {:.2}m) dist={:.2}m angle={:.1}° h={:.2}m",
                i + 1, c.x, c.y, c.distance, c.angle_deg, c.height
            );
        }
    }
}
