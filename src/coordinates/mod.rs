mod conflicts;
mod cost;
mod extensions;
mod fitting;
mod graph;
mod pathfind;
mod persistence;
mod types;

pub use types::ConePosition;

use crate::detection::Cone;
use types::*;

#[derive(Debug, Clone)]
pub struct CoordinateSystem {
    tracked: Vec<TrackedCone>,
    pub cones: Vec<ConePosition>,
    pub track_left: Vec<usize>,
    pub track_right: Vec<usize>,
    pub track_center: Vec<[f32; 2]>,
    pub show_cones: bool,
    pub show_pathfinding: bool,
    pub use_virtual: bool,
    prev_left: Vec<[f32; 2]>,
    prev_right: Vec<[f32; 2]>,
    prev_left_cost: f32,
    prev_right_cost: f32,
    left_stable_frames: u32,
    right_stable_frames: u32,
    left_age: u32,
    right_age: u32,
}

impl CoordinateSystem {
    pub fn new() -> Self {
        Self {
            tracked: Vec::new(),
            cones: Vec::new(),
            track_left: Vec::new(),
            track_right: Vec::new(),
            track_center: Vec::new(),
            show_cones: true,
            show_pathfinding: true,
            use_virtual: true,
            prev_left: Vec::new(),
            prev_right: Vec::new(),
            prev_left_cost: f32::MAX,
            prev_right_cost: f32::MAX,
            left_stable_frames: 0,
            right_stable_frames: 0,
            left_age: 0,
            right_age: 0,
        }
    }

    pub fn toggle_cones(&mut self) {
        self.show_cones = !self.show_cones;
        if !self.show_cones {
            // cones off → pathfinding and virtual off too
            self.show_pathfinding = false;
            self.use_virtual = false;
            self.track_left.clear();
            self.track_right.clear();
            self.track_center.clear();
        }
    }

    pub fn toggle_pathfinding(&mut self) {
        self.show_pathfinding = !self.show_pathfinding;
        if self.show_pathfinding {
            // pathfinding on → cones must be on
            self.show_cones = true;
        } else {
            // pathfinding off → virtual off too
            self.use_virtual = false;
            self.track_left.clear();
            self.track_right.clear();
            self.track_center.clear();
        }
    }

    pub fn toggle_virtual(&mut self) {
        self.use_virtual = !self.use_virtual;
        if self.use_virtual {
            // virtual on → pathfinding and cones must be on
            self.show_pathfinding = true;
            self.show_cones = true;
        }
    }

    pub fn update(&mut self, detected: &[Cone]) {
        // --- Tracking ---
        for t in &mut self.tracked {
            t.missed += 1;
        }
        let tc = self.tracked.len();
        let mut used = vec![false; tc];
        let mut new_cones: Vec<TrackedCone> = Vec::new();

        for det in detected {
            let mut best_idx = None;
            let mut best_d = MATCH_RADIUS;
            for (i, t) in self.tracked[..tc].iter().enumerate() {
                if used[i] {
                    continue;
                }
                let d = ((det.x - t.x).powi(2) + (det.y - t.y).powi(2)).sqrt();
                if d < best_d {
                    best_d = d;
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
                new_cones.push(TrackedCone::from_cone(det));
            }
        }
        self.tracked.extend(new_cones);
        self.tracked.retain(|t| t.missed < MAX_MISSED);

        // --- Build cone list ---
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
                on_track: false,
                side: 0,
                is_virtual: false,
            });
        }

        if self.show_pathfinding {
            self.fit_track();
        }
    }

    pub fn print_positions(&self) {
        if self.cones.is_empty() {
            return;
        }
        let on = self.cones.iter().filter(|c| c.on_track).count();
        let virt = self
            .cones
            .iter()
            .filter(|c| c.is_virtual && c.on_track)
            .count();
        println!(
            "--- Cones: {} total, {} on track (L:{} R:{}) virtual:{} ---",
            self.cones.len(),
            on,
            self.track_left.len(),
            self.track_right.len(),
            virt
        );
        for (i, c) in self.cones.iter().enumerate() {
            if !c.on_track {
                continue;
            }
            let s = match c.side {
                1 => "L",
                -1 => "R",
                _ => "?",
            };
            let v = if c.is_virtual { " [V]" } else { "" };
            println!(
                "  #{}: ({:.2},{:.2}) dist={:.2}m {}{}",
                i + 1,
                c.x,
                c.y,
                c.distance,
                s,
                v
            );
        }
    }
}
