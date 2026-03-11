use super::types::*;

pub(crate) fn track_overlap(a: &[[f32; 2]], b: &[[f32; 2]]) -> f32 {
    if a.is_empty() || b.is_empty() {
        return 0.0;
    }
    let mut matches = 0;
    for pa in a {
        for pb in b {
            let d = ((pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2)).sqrt();
            if d < TRACK_MATCH_RADIUS {
                matches += 1;
                break;
            }
        }
    }
    matches as f32 / a.len().max(b.len()) as f32
}

pub(crate) fn path_cost(cones: &[ConePosition], path: &[usize], is_left: bool) -> f32 {
    if path.len() < 2 {
        return f32::MAX;
    }
    let mut cost = 0.0f32;

    cost += 5000.0 / path.len() as f32;

    for i in 1..path.len() - 1 {
        let (ax, ay) = (
            cones[path[i]].x - cones[path[i - 1]].x,
            cones[path[i]].y - cones[path[i - 1]].y,
        );
        let (bx, by) = (
            cones[path[i + 1]].x - cones[path[i]].x,
            cones[path[i + 1]].y - cones[path[i]].y,
        );
        let al = (ax * ax + ay * ay).sqrt();
        let bl = (bx * bx + by * by).sqrt();
        if al > 0.01 && bl > 0.01 {
            let cos_a = ((ax * bx + ay * by) / (al * bl)).clamp(-1.0, 1.0);
            let angle = cos_a.acos().to_degrees();
            let cross_z = ax * by - ay * bx;
            let curving_correct = if is_left {
                cross_z > 0.0
            } else {
                cross_z < 0.0
            };

            if curving_correct {
                if angle > 50.0 {
                    cost += 200.0 * (angle / 30.0);
                } else {
                    cost += 5.0 * angle;
                    cost -= 15.0 * angle.min(35.0);
                }
            } else {
                if angle > 40.0 {
                    cost += 1000.0 * (angle / 30.0);
                } else if angle > 15.0 {
                    cost += 80.0 * angle;
                } else {
                    cost += 15.0 * angle;
                }
            }
        }
    }

    for i in 1..path.len() {
        let d = ((cones[path[i]].x - cones[path[i - 1]].x).powi(2)
            + (cones[path[i]].y - cones[path[i - 1]].y).powi(2))
        .sqrt();
        if d > 6.0 {
            cost += 150.0 * (d - 5.0);
        } else if d < 1.0 {
            cost += 300.0 * (1.0 - d);
        }
    }

    let init_angle = cones[path[0]].y.atan2(cones[path[0]].x).abs().to_degrees();
    cost += 500.0 * (init_angle / 90.0).min(1.0);

    for &idx in path {
        let on_wrong_side = if is_left {
            cones[idx].y < -1.0
        } else {
            cones[idx].y > 1.0
        };
        if on_wrong_side {
            cost += 1500.0;
        }
    }

    cost
}

pub(crate) fn turn_angle(cones: &[ConePosition], a: usize, b: usize, c: usize) -> f32 {
    let ax = cones[b].x - cones[a].x;
    let ay = cones[b].y - cones[a].y;
    let bx = cones[c].x - cones[b].x;
    let by = cones[c].y - cones[b].y;
    (ax * by - ay * bx).atan2(ax * bx + ay * by)
}

pub(crate) fn estimate_track_width(
    cones: &[ConePosition],
    path_a: &[usize],
    path_b: &[usize],
) -> f32 {
    let mut widths = Vec::new();
    for &ai in path_a {
        let ac = &cones[ai];
        for &bi in path_b {
            let bc = &cones[bi];
            if (ac.x - bc.x).abs() > MAX_MATCH_X_DIFF {
                continue;
            }
            let dist = ((ac.x - bc.x).powi(2) + (ac.y - bc.y).powi(2)).sqrt();
            if dist > MIN_TRACK_WIDTH && dist < MAX_TRACK_WIDTH {
                widths.push(dist);
                break;
            }
        }
    }
    if widths.is_empty() {
        return 3.0;
    }
    widths.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    widths[widths.len() / 2]
}
