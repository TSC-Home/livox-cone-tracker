use super::conflicts::*;
use super::cost::path_cost;
use super::extensions::*;
use super::graph::build_knn_graph;
use super::pathfind::*;
use super::types::*;
use super::CoordinateSystem;

impl CoordinateSystem {
    pub(crate) fn fit_track(&mut self) {
        let n = self.cones.len();
        if n < 2 {
            self.track_left.clear();
            self.track_right.clear();
            self.track_center.clear();
            return;
        }

        for c in &mut self.cones {
            c.on_track = false;
            c.side = 0;
        }
        self.cones.retain(|c| !c.is_virtual);

        // Step 1: KNN graph
        let adj = build_knn_graph(&self.cones);

        let outlier: Vec<bool> = (0..self.cones.len())
            .map(|i| adj[i].len() < 2 && self.cones[i].distance > 3.0)
            .collect();

        // Step 2: Start cones
        let left_start = find_start_cone(&self.cones, &outlier, true);
        let right_start = find_start_cone(&self.cones, &outlier, false);

        // Step 3: DFS path search
        let mut new_left = if let Some(start) = left_start {
            best_path(&self.cones, &adj, &outlier, start, true)
        } else {
            Vec::new()
        };

        let mut new_right = if let Some(start) = right_start {
            best_path(&self.cones, &adj, &outlier, start, false)
        } else {
            Vec::new()
        };

        // Step 3b: Greedy forward extension
        if new_left.len() >= 2 {
            extend_forward(&self.cones, &mut new_left, true);
        }
        if new_right.len() >= 2 {
            extend_forward(&self.cones, &mut new_right, false);
        }

        // Step 4: Resolve conflicts
        let (mut new_left_clean, mut new_right_clean) =
            resolve_conflicts(&self.cones, &new_left, &new_right);

        // Step 4b: Cross-track extension
        if new_left_clean.len() >= 3 && new_right_clean.len() + 2 < new_left_clean.len() {
            extend_from_guide(&self.cones, &mut new_right_clean, &new_left_clean, false);
        }
        if new_right_clean.len() >= 3 && new_left_clean.len() + 2 < new_right_clean.len() {
            extend_from_guide(&self.cones, &mut new_left_clean, &new_right_clean, true);
        }

        // Step 4c: Validate paths
        validate_path(&self.cones, &mut new_left_clean, &new_right_clean, true);
        validate_path(&self.cones, &mut new_right_clean, &new_left_clean, false);

        // Step 5: Track persistence
        let new_left_cost = if new_left_clean.len() >= 2 {
            path_cost(&self.cones, &new_left_clean, true)
        } else {
            f32::MAX
        };
        let new_right_cost = if new_right_clean.len() >= 2 {
            path_cost(&self.cones, &new_right_clean, false)
        } else {
            f32::MAX
        };

        self.left_age += 1;
        self.right_age += 1;

        let prev_l = self.prev_left.clone();
        let prev_l_cost = self.prev_left_cost;
        let l_stable = self.left_stable_frames;
        let l_age = self.left_age;

        let prev_r = self.prev_right.clone();
        let prev_r_cost = self.prev_right_cost;
        let r_stable = self.right_stable_frames;
        let r_age = self.right_age;

        let (final_left, left_changed) = self.pick_stable_track_v2(
            &new_left_clean,
            new_left_cost,
            &prev_l,
            prev_l_cost,
            l_stable,
            l_age,
        );
        if left_changed {
            self.left_age = 0;
            self.left_stable_frames = 0;
        } else {
            self.left_stable_frames += 1;
        }

        let (final_right, right_changed) = self.pick_stable_track_v2(
            &new_right_clean,
            new_right_cost,
            &prev_r,
            prev_r_cost,
            r_stable,
            r_age,
        );
        if right_changed {
            self.right_age = 0;
            self.right_stable_frames = 0;
        } else {
            self.right_stable_frames += 1;
        }

        // Step 6: Virtual cones
        let left_with_virtual = if self.use_virtual {
            self.insert_virtual_cones(&final_left, 1)
        } else {
            final_left.clone()
        };
        let right_with_virtual = if self.use_virtual {
            self.insert_virtual_cones(&final_right, -1)
        } else {
            final_right.clone()
        };

        // Step 7: Mark
        for &i in &left_with_virtual {
            self.cones[i].on_track = true;
            self.cones[i].side = 1;
        }
        for &i in &right_with_virtual {
            self.cones[i].on_track = true;
            self.cones[i].side = -1;
        }

        // Step 8: Center line
        self.track_center.clear();
        for &li in &left_with_virtual {
            let lc = &self.cones[li];
            let mut best_ri = None;
            let mut best_score = f32::MAX;
            for &ri in &right_with_virtual {
                let rc = &self.cones[ri];
                let dx = (lc.x - rc.x).abs();
                if dx > MAX_MATCH_X_DIFF {
                    continue;
                }
                let width = (lc.y - rc.y).abs();
                if width < MIN_TRACK_WIDTH || width > MAX_TRACK_WIDTH {
                    continue;
                }
                let score = dx + (width - 3.0).abs();
                if score < best_score {
                    best_score = score;
                    best_ri = Some(ri);
                }
            }
            if let Some(ri) = best_ri {
                let rc = &self.cones[ri];
                self.track_center
                    .push([(lc.x + rc.x) / 2.0, (lc.y + rc.y) / 2.0]);
            }
        }

        // Save
        self.prev_left = final_left
            .iter()
            .map(|&i| [self.cones[i].x, self.cones[i].y])
            .collect();
        self.prev_right = final_right
            .iter()
            .map(|&i| [self.cones[i].x, self.cones[i].y])
            .collect();
        self.prev_left_cost = if final_left.len() >= 2 {
            path_cost(&self.cones, &final_left, true)
        } else {
            f32::MAX
        };
        self.prev_right_cost = if final_right.len() >= 2 {
            path_cost(&self.cones, &final_right, false)
        } else {
            f32::MAX
        };

        self.track_left = left_with_virtual;
        self.track_right = right_with_virtual;

        self.track_center
            .sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap_or(std::cmp::Ordering::Equal));
    }
}
