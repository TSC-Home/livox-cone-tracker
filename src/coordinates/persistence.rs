use super::cost::track_overlap;
use super::types::*;
use super::CoordinateSystem;

impl CoordinateSystem {
    /// Returns (chosen_path, did_change)
    pub(crate) fn pick_stable_track_v2(
        &self,
        new_path: &[usize],
        new_cost: f32,
        prev_positions: &[[f32; 2]],
        prev_cost: f32,
        stable_frames: u32,
        age: u32,
    ) -> (Vec<usize>, bool) {
        if prev_positions.is_empty() {
            return (new_path.to_vec(), true);
        }

        if new_path.is_empty() {
            return (self.reconstruct_from_previous(prev_positions), false);
        }

        let new_positions: Vec<[f32; 2]> = new_path
            .iter()
            .map(|&i| [self.cones[i].x, self.cones[i].y])
            .collect();
        let overlap = track_overlap(&new_positions, prev_positions);

        if overlap > 0.6 {
            return (new_path.to_vec(), false);
        }

        let stability_resistance = (stable_frames as f32 * 0.1).min(0.8);
        let cost_threshold = 1.0 - stability_resistance;
        if new_cost < prev_cost * cost_threshold && new_path.len() >= prev_positions.len() {
            return (new_path.to_vec(), true);
        }

        if new_path.len() >= prev_positions.len() + 3 && new_cost < prev_cost * 2.0 {
            return (new_path.to_vec(), true);
        }

        if age >= TRACK_KEEP_FRAMES {
            return (new_path.to_vec(), true);
        }

        (self.reconstruct_from_previous(prev_positions), false)
    }

    pub(crate) fn reconstruct_from_previous(&self, prev_positions: &[[f32; 2]]) -> Vec<usize> {
        let mut result = Vec::new();
        let mut used = vec![false; self.cones.len()];

        for pos in prev_positions {
            let mut best_idx = None;
            let mut best_d = TRACK_MATCH_RADIUS;
            for (i, c) in self.cones.iter().enumerate() {
                if used[i] || c.is_virtual {
                    continue;
                }
                let d = ((c.x - pos[0]).powi(2) + (c.y - pos[1]).powi(2)).sqrt();
                if d < best_d {
                    best_d = d;
                    best_idx = Some(i);
                }
            }
            if let Some(i) = best_idx {
                result.push(i);
                used[i] = true;
            }
        }
        result
    }

    pub(crate) fn insert_virtual_cones(&mut self, path: &[usize], side: i8) -> Vec<usize> {
        if path.len() < 2 {
            return path.to_vec();
        }

        let mut virtuals: Vec<(usize, Vec<ConePosition>)> = Vec::new();

        for i in 1..path.len() {
            let prev_x = self.cones[path[i - 1]].x;
            let prev_y = self.cones[path[i - 1]].y;
            let prev_h = self.cones[path[i - 1]].height;
            let curr_x = self.cones[path[i]].x;
            let curr_y = self.cones[path[i]].y;
            let curr_h = self.cones[path[i]].height;

            let dx = curr_x - prev_x;
            let dy = curr_y - prev_y;
            let gap = (dx * dx + dy * dy).sqrt();

            if gap > VIRTUAL_GAP_THRESHOLD {
                let n_virtual = ((gap / MAX_VIRTUAL_SPACING).ceil() as usize).max(1) - 1;
                if n_virtual > 0 {
                    let mut batch = Vec::new();
                    for k in 1..=n_virtual {
                        let frac = k as f32 / (n_virtual + 1) as f32;
                        let vx = prev_x + dx * frac;
                        let vy = prev_y + dy * frac;
                        let vh = (prev_h + curr_h) / 2.0;
                        let vdist = (vx * vx + vy * vy).sqrt();
                        let vangle = vy.atan2(vx).to_degrees();
                        batch.push(ConePosition {
                            x: vx,
                            y: vy,
                            distance: vdist,
                            angle_deg: vangle,
                            height: vh,
                            on_track: false,
                            side,
                            is_virtual: true,
                            tracked_idx: usize::MAX,
                        });
                    }
                    virtuals.push((i, batch));
                }
            }
        }

        let mut result: Vec<usize> = Vec::new();
        result.push(path[0]);
        let mut virtual_iter = virtuals.into_iter().peekable();

        for i in 1..path.len() {
            if let Some((at_idx, _)) = virtual_iter.peek() {
                if *at_idx == i {
                    let (_, batch) = virtual_iter.next().unwrap();
                    for vc in batch {
                        let vidx = self.cones.len();
                        self.cones.push(vc);
                        result.push(vidx);
                    }
                }
            }
            result.push(path[i]);
        }

        result
    }
}
