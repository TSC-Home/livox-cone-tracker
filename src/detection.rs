use crate::point::Point3D;

#[derive(Debug, Clone, Copy)]
pub struct Cone {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub height: f32,
    pub point_count: usize,
}

pub struct DetectionConfig {
    pub cluster_radius: f32,
    pub min_points: usize,
    pub max_points: usize,
    pub min_spread: f32,
    pub max_spread: f32,
    pub max_aspect_ratio: f32,
    pub min_z_range: f32,
    pub debug: bool,
}

impl Default for DetectionConfig {
    fn default() -> Self {
        Self {
            cluster_radius: 0.15,
            min_points: 3,
            max_points: 1000,
            min_spread: 0.02,
            max_spread: 0.40,
            max_aspect_ratio: 4.0,
            min_z_range: 0.03,
            debug: true, // Print cluster stats to find cone signature
        }
    }
}

/// Detect cones from point cloud.
/// First pass: cluster and print ALL cluster stats (debug mode).
/// This helps us understand what cones vs walls look like in the data.
pub fn detect_cones(points: &[Point3D], config: &DetectionConfig) -> Vec<Cone> {
    if points.is_empty() {
        return Vec::new();
    }

    let clusters = grid_cluster(points, config.cluster_radius);
    let mut cones = Vec::new();

    if config.debug {
        // Print stats for ALL clusters so we can see what cones look like
        let mut stats_list: Vec<_> = clusters.iter()
            .filter(|c| c.len() >= config.min_points)
            .map(|c| (c.len(), cluster_stats(c)))
            .collect();
        stats_list.sort_by(|a, b| b.0.cmp(&a.0));

        println!("  --- Cluster Analysis ({} clusters with >= {} pts) ---",
            stats_list.len(), config.min_points);
        for (i, (count, stats)) in stats_list.iter().enumerate().take(10) {
            let label = if is_cone(stats, *count, config) { "CONE" } else { "    " };
            println!(
                "  [{label}] #{i}: pts={count:4}, pos=({:.2},{:.2}), \
                spread={:.3}m, aspect={:.1}, z_range={:.3}m, radial_dev={:.0}°",
                stats.cx, stats.cy,
                stats.spread, stats.aspect_ratio,
                stats.z_range, stats.radial_deviation,
            );
        }
    }

    for cluster in &clusters {
        let count = cluster.len();
        if count < config.min_points || count > config.max_points {
            continue;
        }

        let stats = cluster_stats(cluster);
        if !is_cone(&stats, count, config) {
            continue;
        }

        cones.push(Cone {
            x: stats.cx,
            y: stats.cy,
            z: stats.min_z,
            height: stats.z_range,
            point_count: count,
        });
    }

    cones
}

fn is_cone(stats: &ClusterStats, _count: usize, config: &DetectionConfig) -> bool {
    // Size check: cone is small (10-40cm spread)
    if stats.spread < config.min_spread || stats.spread > config.max_spread {
        return false;
    }

    // Shape check: not too elongated (wall would be very elongated)
    if stats.aspect_ratio > config.max_aspect_ratio {
        return false;
    }

    // Vertical extent: cone sticks up, ground is flat
    if stats.z_range < config.min_z_range {
        return false;
    }

    true
}

struct ClusterStats {
    cx: f32,
    cy: f32,
    min_z: f32,
    z_range: f32,
    spread: f32,
    aspect_ratio: f32,
    radial_deviation: f32,
}

fn cluster_stats(cluster: &[&Point3D]) -> ClusterStats {
    let n = cluster.len() as f32;
    let mut sx = 0.0f32;
    let mut sy = 0.0f32;
    let mut min_z = f32::MAX;
    let mut max_z = f32::MIN;

    for p in cluster {
        sx += p.x;
        sy += p.y;
        min_z = min_z.min(p.z);
        max_z = max_z.max(p.z);
    }

    let cx = sx / n;
    let cy = sy / n;

    let mut cxx = 0.0f32;
    let mut cyy = 0.0f32;
    let mut cxy = 0.0f32;
    let mut max_r = 0.0f32;

    for p in cluster {
        let dx = p.x - cx;
        let dy = p.y - cy;
        cxx += dx * dx;
        cyy += dy * dy;
        cxy += dx * dy;
        max_r = max_r.max((dx * dx + dy * dy).sqrt());
    }
    cxx /= n;
    cyy /= n;
    cxy /= n;

    let trace = cxx + cyy;
    let det = cxx * cyy - cxy * cxy;
    let disc = (trace * trace / 4.0 - det).max(0.0).sqrt();
    let e1 = (trace / 2.0 + disc).max(1e-6);
    let e2 = (trace / 2.0 - disc).max(1e-6);

    let major = e1.sqrt();
    let minor = e2.sqrt();
    let aspect_ratio = major / minor.max(1e-6);

    let major_angle = if cxy.abs() > 1e-8 {
        (e1 - cxx).atan2(cxy)
    } else if cxx >= cyy {
        0.0
    } else {
        std::f32::consts::FRAC_PI_2
    };

    let radial_angle = cy.atan2(cx);
    let mut angle_diff = (major_angle - radial_angle).abs();
    if angle_diff > std::f32::consts::FRAC_PI_2 {
        angle_diff = std::f32::consts::PI - angle_diff;
    }

    ClusterStats {
        cx, cy, min_z,
        z_range: max_z - min_z,
        spread: max_r,
        aspect_ratio,
        radial_deviation: angle_diff.to_degrees(),
    }
}

fn grid_cluster<'a>(points: &'a [Point3D], cell_size: f32) -> Vec<Vec<&'a Point3D>> {
    use std::collections::HashMap;

    let inv = 1.0 / cell_size;
    let mut grid: HashMap<(i32, i32), Vec<&'a Point3D>> = HashMap::new();

    for p in points {
        let gx = (p.x * inv).floor() as i32;
        let gy = (p.y * inv).floor() as i32;
        grid.entry((gx, gy)).or_default().push(p);
    }

    let mut visited = std::collections::HashSet::new();
    let mut clusters = Vec::new();

    for key in grid.keys().copied().collect::<Vec<_>>() {
        if visited.contains(&key) {
            continue;
        }
        let mut cluster = Vec::new();
        let mut stack = vec![key];

        while let Some(k) = stack.pop() {
            if !visited.insert(k) {
                continue;
            }
            if let Some(pts) = grid.get(&k) {
                cluster.extend_from_slice(pts);
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        let nb = (k.0 + dx, k.1 + dy);
                        if !visited.contains(&nb) && grid.contains_key(&nb) {
                            stack.push(nb);
                        }
                    }
                }
            }
        }

        if !cluster.is_empty() {
            clusters.push(cluster);
        }
    }

    clusters
}
