use super::cost::estimate_track_width;
use super::types::*;

pub(crate) fn extend_forward(cones: &[ConePosition], path: &mut Vec<usize>, is_left: bool) {
    if path.len() < 2 {
        return;
    }

    for _ in 0..EXTEND_MAX_CONES {
        let n = path.len();
        let last = path[n - 1];
        let prev = path[n - 2];

        let dir_x = cones[last].x - cones[prev].x;
        let dir_y = cones[last].y - cones[prev].y;
        let dir_len = (dir_x * dir_x + dir_y * dir_y).sqrt();
        if dir_len < 0.01 {
            break;
        }

        let mut best_idx = None;
        let mut best_score = f32::MAX;

        for (i, c) in cones.iter().enumerate() {
            if path.contains(&i) {
                continue;
            }

            let dx = c.x - cones[last].x;
            let dy = c.y - cones[last].y;
            let dist = (dx * dx + dy * dy).sqrt();

            if dist < 0.5 || dist > EXTEND_MAX_DIST || dist > MAX_SEGMENT_DIST {
                continue;
            }

            let dot = dir_x * dx + dir_y * dy;
            let cos_a = (dot / (dir_len * dist)).clamp(-1.0, 1.0);
            let angle = cos_a.acos().to_degrees();
            if angle > EXTEND_MAX_ANGLE {
                continue;
            }

            if dot < 0.0 {
                continue;
            }

            let cross_z = dir_x * dy - dir_y * dx;
            let signed = cross_z.atan2(dot).to_degrees();
            if is_left && signed < -60.0 {
                continue;
            }
            if !is_left && signed > 60.0 {
                continue;
            }

            let score = dist * 0.5 + angle * 2.0;
            if score < best_score {
                best_score = score;
                best_idx = Some(i);
            }
        }

        if let Some(idx) = best_idx {
            path.push(idx);
        } else {
            break;
        }
    }
}

pub(crate) fn extend_from_guide(
    cones: &[ConePosition],
    short_path: &mut Vec<usize>,
    guide_path: &[usize],
    is_left: bool,
) {
    if short_path.is_empty() || guide_path.len() < 2 {
        return;
    }

    let width = estimate_track_width(cones, short_path, guide_path);
    if width < MIN_TRACK_WIDTH || width > MAX_TRACK_WIDTH {
        return;
    }

    let last_short = *short_path.last().unwrap();
    let last_x = cones[last_short].x;
    let last_y = cones[last_short].y;

    let mut guide_start = 0;
    let mut best_d = f32::MAX;
    for (gi, &gc) in guide_path.iter().enumerate() {
        let d = ((cones[gc].x - last_x).powi(2) + (cones[gc].y - last_y).powi(2)).sqrt();
        if d < best_d {
            best_d = d;
            guide_start = gi;
        }
    }

    for gi in (guide_start + 1)..guide_path.len() {
        let gc = &cones[guide_path[gi]];

        let (dir_x, dir_y) = if gi > 0 {
            let prev = &cones[guide_path[gi - 1]];
            let dx = gc.x - prev.x;
            let dy = gc.y - prev.y;
            let len = (dx * dx + dy * dy).sqrt();
            if len > 0.01 {
                (dx / len, dy / len)
            } else {
                (1.0, 0.0)
            }
        } else {
            (1.0, 0.0)
        };

        let perp_x = -dir_y;
        let perp_y = dir_x;
        let sign = if is_left { 1.0 } else { -1.0 };
        let expected_x = gc.x + perp_x * width * sign;
        let expected_y = gc.y + perp_y * width * sign;

        let mut best_idx = None;
        let mut best_d = 3.5;
        for (i, c) in cones.iter().enumerate() {
            if short_path.contains(&i) || guide_path.contains(&i) {
                continue;
            }
            let dx = c.x - expected_x;
            let dy = c.y - expected_y;
            let d = (dx * dx + dy * dy).sqrt();
            if d < best_d {
                let side_dot = (c.x - gc.x) * perp_x + (c.y - gc.y) * perp_y;
                if (is_left && side_dot > 0.0) || (!is_left && side_dot < 0.0) {
                    best_d = d;
                    best_idx = Some(i);
                }
            }
        }

        if let Some(idx) = best_idx {
            if short_path.len() >= 2 {
                let prev = *short_path.last().unwrap();
                let prev2 = short_path[short_path.len() - 2];
                let ax = cones[prev].x - cones[prev2].x;
                let ay = cones[prev].y - cones[prev2].y;
                let bx = cones[idx].x - cones[prev].x;
                let by = cones[idx].y - cones[prev].y;
                let al = (ax * ax + ay * ay).sqrt();
                let bl = (bx * bx + by * by).sqrt();
                if al > 0.01 && bl > 0.01 {
                    let cos_a = ((ax * bx + ay * by) / (al * bl)).clamp(-1.0, 1.0);
                    if cos_a.acos().to_degrees() > 90.0 {
                        continue;
                    }
                }
            }
            short_path.push(idx);
        }
    }
}
