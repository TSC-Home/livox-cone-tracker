use super::cost::*;
use super::types::*;

pub(crate) fn resolve_conflicts(
    cones: &[ConePosition],
    left: &[usize],
    right: &[usize],
) -> (Vec<usize>, Vec<usize>) {
    let mut lc = left.to_vec();
    let mut rc = right.to_vec();
    let conflicts: Vec<usize> = lc.iter().filter(|i| rc.contains(i)).copied().collect();

    for &c in &conflicts {
        let li_pos = lc.iter().position(|&x| x == c);
        let ri_pos = rc.iter().position(|&x| x == c);
        let (lp, rp) = match (li_pos, ri_pos) {
            (Some(l), Some(r)) => (l, r),
            _ => continue,
        };

        let l_prev_dist = if lp > 0 {
            let prev = lc[lp - 1];
            ((cones[prev].x - cones[c].x).powi(2) + (cones[prev].y - cones[c].y).powi(2)).sqrt()
        } else {
            f32::MAX
        };
        let r_prev_dist = if rp > 0 {
            let prev = rc[rp - 1];
            ((cones[prev].x - cones[c].x).powi(2) + (cones[prev].y - cones[c].y).powi(2)).sqrt()
        } else {
            f32::MAX
        };

        if l_prev_dist < 3.0 && r_prev_dist >= 3.0 {
            rc.remove(rp);
            continue;
        }
        if r_prev_dist < 3.0 && l_prev_dist >= 3.0 {
            lc.remove(lp);
            continue;
        }

        if lp > 0 && lp + 1 < lc.len() && rp > 0 && rp + 1 < rc.len() {
            let l_angle = turn_angle(cones, lc[lp - 1], c, lc[lp + 1]);
            let r_angle = turn_angle(cones, rc[rp - 1], c, rc[rp + 1]);
            if l_angle > 0.0 && r_angle >= 0.0 {
                rc.remove(rp);
                continue;
            }
            if r_angle < 0.0 && l_angle <= 0.0 {
                lc.remove(lp);
                continue;
            }
        }

        let l_rem = lc.len() - lp;
        let r_rem = rc.len() - rp;
        if l_rem.abs_diff(r_rem) > 2 {
            if l_rem > r_rem {
                rc.remove(rp);
            } else {
                lc.remove(lp);
            }
            continue;
        }

        if cones[c].y > 0.5 {
            rc.remove(rp);
        } else if cones[c].y < -0.5 {
            lc.remove(lp);
        } else if lc.len() <= rc.len() {
            rc.remove(rp);
        } else {
            lc.remove(lp);
        }
    }

    (lc, rc)
}

pub(crate) fn validate_path(
    cones: &[ConePosition],
    path: &mut Vec<usize>,
    other_path: &[usize],
    is_left: bool,
) {
    if path.len() < 3 || other_path.is_empty() {
        return;
    }

    let width = estimate_track_width(cones, path, other_path);
    if width <= 0.0 {
        return;
    }

    while path.len() > 2 {
        let last = *path.last().unwrap();
        let lc = &cones[last];

        let mut min_dist = f32::MAX;
        for &oi in other_path.iter() {
            let oc = &cones[oi];
            let d = ((lc.x - oc.x).powi(2) + (lc.y - oc.y).powi(2)).sqrt();
            if d < min_dist {
                min_dist = d;
            }
        }

        if min_dist > width * 3.0 && min_dist > MAX_TRACK_WIDTH * 2.0 {
            path.pop();
            continue;
        }

        let wrong_side = if is_left { lc.y < -2.0 } else { lc.y > 2.0 };
        if wrong_side && path.len() > 3 {
            path.pop();
            continue;
        }

        break;
    }
}
