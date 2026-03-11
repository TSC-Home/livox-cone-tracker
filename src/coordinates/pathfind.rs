use super::cost::path_cost;
use super::types::*;

pub(crate) fn find_start_cone(
    cones: &[ConePosition],
    outlier: &[bool],
    left: bool,
) -> Option<usize> {
    let mut best = None;
    let mut best_dist = f32::MAX;

    for (i, c) in cones.iter().enumerate() {
        if outlier[i] {
            continue;
        }
        let correct_side = if left { c.y > 0.0 } else { c.y < 0.0 };
        if !correct_side || c.x < -1.0 || c.distance > 8.0 {
            continue;
        }
        if c.angle_deg.abs() > 144.0 {
            continue;
        }
        if c.distance < best_dist {
            best_dist = c.distance;
            best = Some(i);
        }
    }
    best
}

pub(crate) fn best_path(
    cones: &[ConePosition],
    adj: &[Vec<usize>],
    outlier: &[bool],
    start: usize,
    is_left: bool,
) -> Vec<usize> {
    let mut best_path: Vec<usize> = Vec::new();
    let mut best_score = f32::MAX;

    let init_dir = (cones[start].x, cones[start].y);
    let mut stack: Vec<(Vec<usize>, (f32, f32))> = vec![(vec![start], init_dir)];

    while let Some((path, last_dir)) = stack.pop() {
        if path.len() >= 2 {
            let sc = path_cost(cones, &path, is_left);
            if sc < best_score || (path.len() > best_path.len() && sc < best_score * 1.5) {
                best_score = sc;
                best_path = path.clone();
            }
        }

        if path.len() >= MAX_PATH_LEN {
            continue;
        }
        let last = *path.last().unwrap();

        for &next in &adj[last] {
            if path.contains(&next) || outlier[next] {
                continue;
            }

            let dx = cones[next].x - cones[last].x;
            let dy = cones[next].y - cones[last].y;
            let seg_len = (dx * dx + dy * dy).sqrt();
            if seg_len < 0.01 || seg_len > MAX_SEGMENT_DIST {
                continue;
            }

            let dot = last_dir.0 * dx + last_dir.1 * dy;
            let cross_z = last_dir.0 * dy - last_dir.1 * dx;
            let last_len = (last_dir.0.powi(2) + last_dir.1.powi(2)).sqrt();
            let cos_a = (dot / (last_len * seg_len)).clamp(-1.0, 1.0);
            let angle_deg = cos_a.acos().to_degrees();

            if angle_deg > ANGLE_THRESHOLD_ABS {
                continue;
            }

            let signed_angle = cross_z.atan2(dot).to_degrees();
            if seg_len >= CLOSE_DIST_BYPASS {
                if is_left && signed_angle < -ANGLE_THRESHOLD_DIR {
                    continue;
                }
                if !is_left && signed_angle > ANGLE_THRESHOLD_DIR {
                    continue;
                }
            }

            if path.len() >= 2 {
                let prev2 = path[path.len() - 2];
                let prev_dx = cones[last].x - cones[prev2].x;
                let prev_dy = cones[last].y - cones[prev2].y;
                let prev_signed =
                    (prev_dx * dy - prev_dy * dx).atan2(prev_dx * dx + prev_dy * dy);
                let curr_signed = cross_z.atan2(dot);
                if prev_signed * curr_signed < 0.0
                    && prev_signed.abs() > REVERSAL_THRESHOLD
                    && curr_signed.abs() > REVERSAL_THRESHOLD
                {
                    continue;
                }
            }

            if is_blocked(cones, &path, last, next) {
                continue;
            }

            let mut new_path = path.clone();
            new_path.push(next);
            stack.push((new_path, (dx, dy)));
        }
    }

    best_path
}

fn is_blocked(cones: &[ConePosition], path: &[usize], from: usize, to: usize) -> bool {
    let fx = cones[from].x;
    let fy = cones[from].y;
    let seg_dx = cones[to].x - fx;
    let seg_dy = cones[to].y - fy;
    let seg_len = (seg_dx * seg_dx + seg_dy * seg_dy).sqrt();
    if seg_len < 0.01 {
        return false;
    }

    for (k, c) in cones.iter().enumerate() {
        if k == from || k == to || path.contains(&k) {
            continue;
        }
        let cx = c.x - fx;
        let cy = c.y - fy;
        let d = (cx * cx + cy * cy).sqrt();
        if d >= seg_len || d > MAX_NEIGHBOR_DIST {
            continue;
        }
        let dot = cx * seg_dx + cy * seg_dy;
        if dot < 0.0 {
            continue;
        }
        let perp = (cx * seg_dy - cy * seg_dx).abs() / seg_len;
        if perp < 0.8 {
            return true;
        }
    }
    false
}
