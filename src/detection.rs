use crate::point::Point3D;

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub struct Cone {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub height: f32,
    pub radius: f32,
    pub point_count: usize,
    pub confidence: f32,
}

pub struct DetectionConfig {
    // Ground removal
    pub ground_percentile: f32,
    pub ground_margin: f32,

    // Height band: only look between ground+margin and ground+max_above_ground
    // Cones are on the ground, sensor is on car roof (~1.5m up).
    // Anything more than ~0.6m above ground can't be a 30cm cone.
    pub max_above_ground: f32,

    // Voxel downsample
    pub voxel_size: f32,

    // DBSCAN
    pub dbscan_eps: f32,
    pub dbscan_min_pts: usize,

    // Cone geometry (30cm cones)
    pub cone_height_min: f32,
    pub cone_height_max: f32,
    pub cone_width_min: f32,
    pub cone_width_max: f32,
    pub max_aspect_ratio: f32,
    pub min_cluster_points: usize,
    pub max_cluster_points: usize,

    pub min_confidence: f32,
    pub debug: bool,
}

impl Default for DetectionConfig {
    fn default() -> Self {
        Self {
            ground_percentile: 0.15,
            ground_margin: 0.03,
            max_above_ground: 0.55, // only look up to 55cm above ground

            voxel_size: 0.03,

            dbscan_eps: 0.12,
            dbscan_min_pts: 3,

            cone_height_min: 0.12,  // at least 12cm visible
            cone_height_max: 0.45,  // at most 45cm
            cone_width_min: 0.04,
            cone_width_max: 0.30,   // tighter: 30cm max width
            max_aspect_ratio: 2.2,  // stricter roundness
            min_cluster_points: 4,
            max_cluster_points: 200, // tighter: big clusters aren't cones

            min_confidence: 0.50,   // higher threshold
            debug: false,
        }
    }
}

pub fn detect_cones(points: &[Point3D], config: &DetectionConfig) -> Vec<Cone> {
    if points.len() < 20 {
        return Vec::new();
    }

    // Step 1: Find ground level
    let ground_z = z_percentile(points, config.ground_percentile);
    let z_min = ground_z + config.ground_margin;
    let z_max = ground_z + config.max_above_ground;

    // Step 2: Voxel downsample only the height band where cones can be
    let band = voxel_downsample(points, z_min, z_max, config.voxel_size);

    if config.debug {
        println!(
            "  Ground={:.3}m | band=[{:.2}..{:.2}] | {} voxels",
            ground_z, z_min, z_max, band.len()
        );
    }

    if band.len() < config.dbscan_min_pts {
        return Vec::new();
    }

    // Step 3: DBSCAN
    let clusters = dbscan(&band, config.dbscan_eps, config.dbscan_min_pts);

    // Step 4: Score each cluster
    let mut cones = Vec::new();

    for cluster in &clusters {
        let n = cluster.len();
        if n < config.min_cluster_points || n > config.max_cluster_points {
            continue;
        }

        let stats = cluster_stats(cluster);
        let conf = score(stats, config);

        if config.debug && n >= config.min_cluster_points {
            let l = if conf >= config.min_confidence { "CONE" } else { "    " };
            println!(
                "  [{l}] pts={n:3} ({:.2},{:.2}) w={:.2} h={:.2} a={:.1} c={:.2}",
                stats.cx, stats.cy, stats.width, stats.height, stats.aspect, conf,
            );
        }

        if conf >= config.min_confidence {
            cones.push(Cone {
                x: stats.cx,
                y: stats.cy,
                z: stats.min_z,
                height: stats.height,
                radius: stats.width / 2.0,
                point_count: n,
                confidence: conf,
            });
        }
    }

    cones
}

// --- Stats ---

#[derive(Clone, Copy)]
struct Stats {
    cx: f32,
    cy: f32,
    min_z: f32,
    height: f32,
    width: f32,
    aspect: f32,
}

fn cluster_stats(cluster: &[Point3D]) -> Stats {
    let n = cluster.len() as f32;
    let (mut sx, mut sy, mut minz, mut maxz) = (0.0f32, 0.0, f32::MAX, f32::MIN);
    for p in cluster {
        sx += p.x; sy += p.y;
        minz = minz.min(p.z); maxz = maxz.max(p.z);
    }
    let cx = sx / n;
    let cy = sy / n;

    let mut max_r = 0.0f32;
    let (mut cxx, mut cyy, mut cxy) = (0.0f32, 0.0, 0.0);
    for p in cluster {
        let dx = p.x - cx;
        let dy = p.y - cy;
        cxx += dx * dx; cyy += dy * dy; cxy += dx * dy;
        max_r = max_r.max((dx * dx + dy * dy).sqrt());
    }
    cxx /= n; cyy /= n; cxy /= n;

    let tr = cxx + cyy;
    let det = cxx * cyy - cxy * cxy;
    let disc = (tr * tr / 4.0 - det).max(0.0).sqrt();
    let e1 = (tr / 2.0 + disc).max(1e-8).sqrt();
    let e2 = (tr / 2.0 - disc).max(1e-8).sqrt();

    Stats {
        cx, cy, min_z: minz,
        height: maxz - minz,
        width: max_r * 2.0,
        aspect: e1 / e2.max(1e-8),
    }
}

fn score(s: Stats, c: &DetectionConfig) -> f32 {
    // Hard reject: outside bounds = 0
    if s.height < c.cone_height_min || s.height > c.cone_height_max { return 0.0; }
    if s.width < c.cone_width_min || s.width > c.cone_width_max { return 0.0; }
    if s.aspect > c.max_aspect_ratio { return 0.0; }

    let mut sc = 0.0f32;
    let mut mx = 0.0f32;

    // Height closeness to 30cm (weight 3)
    mx += 3.0;
    let hdev = (s.height - 0.30).abs() / 0.30;
    sc += 3.0 * (1.0 - hdev.min(1.0));

    // Width closeness to ~15cm (weight 2)
    mx += 2.0;
    let wdev = (s.width - 0.15).abs() / 0.15;
    sc += 2.0 * (1.0 - wdev.min(1.0));

    // Roundness (weight 2)
    mx += 2.0;
    sc += 2.0 * (1.0 - ((s.aspect - 1.0) / (c.max_aspect_ratio - 1.0)).min(1.0));

    // H/W ratio ~2.0 (weight 2)
    mx += 2.0;
    if s.width > 0.01 {
        let hw = s.height / s.width;
        if hw > 0.8 && hw < 4.0 {
            let dev = (hw - 2.0).abs() / 2.0;
            sc += 2.0 * (1.0 - dev.min(1.0));
        }
    }

    if mx > 0.0 { sc / mx } else { 0.0 }
}

// --- Voxel downsample with height band ---

fn voxel_downsample(points: &[Point3D], z_min: f32, z_max: f32, voxel: f32) -> Vec<Point3D> {
    use std::collections::HashMap;
    let inv = 1.0 / voxel;
    let mut grid: HashMap<(i32, i32, i32), (f32, f32, f32, f32, u32)> = HashMap::new();

    for p in points {
        if p.z <= z_min || p.z > z_max { continue; }
        let k = (
            (p.x * inv).floor() as i32,
            (p.y * inv).floor() as i32,
            (p.z * inv).floor() as i32,
        );
        let e = grid.entry(k).or_insert((0.0, 0.0, 0.0, 0.0, 0));
        e.0 += p.x; e.1 += p.y; e.2 += p.z; e.3 += p.reflectivity; e.4 += 1;
    }

    grid.values()
        .map(|(sx, sy, sz, sr, n)| {
            let n = *n as f32;
            Point3D { x: sx / n, y: sy / n, z: sz / n, reflectivity: sr / n }
        })
        .collect()
}

// --- Z percentile ---

fn z_percentile(points: &[Point3D], pct: f32) -> f32 {
    if points.is_empty() { return 0.0; }
    let step = (points.len() / 2000).max(1);
    let mut zs: Vec<f32> = points.iter().step_by(step)
        .filter(|p| p.z.is_finite())
        .map(|p| p.z)
        .collect();
    if zs.is_empty() { return 0.0; }
    zs.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    zs[((zs.len() as f32 * pct) as usize).min(zs.len() - 1)]
}

// --- DBSCAN ---

fn dbscan(points: &[Point3D], eps: f32, min_pts: usize) -> Vec<Vec<Point3D>> {
    let n = points.len();
    if n == 0 { return Vec::new(); }

    let inv = 1.0 / eps;
    let mut grid: std::collections::HashMap<(i32, i32, i32), Vec<usize>> =
        std::collections::HashMap::new();
    for (i, p) in points.iter().enumerate() {
        let k = ((p.x * inv).floor() as i32, (p.y * inv).floor() as i32, (p.z * inv).floor() as i32);
        grid.entry(k).or_default().push(i);
    }

    let eps2 = eps * eps;
    let mut labels = vec![-1i32; n];
    let mut cid = 0i32;

    for i in 0..n {
        if labels[i] != -1 { continue; }
        let nb = neighbors(points, i, eps2, &grid, inv);
        if nb.len() < min_pts { labels[i] = -2; continue; }

        labels[i] = cid;
        let mut seeds = nb;
        let mut j = 0;
        while j < seeds.len() {
            let q = seeds[j]; j += 1;
            if labels[q] == -2 { labels[q] = cid; }
            if labels[q] != -1 { continue; }
            labels[q] = cid;
            let qn = neighbors(points, q, eps2, &grid, inv);
            if qn.len() >= min_pts {
                for x in qn { if labels[x] <= -1 { seeds.push(x); } }
            }
        }
        cid += 1;
    }

    let nc = cid as usize;
    let mut clusters = vec![Vec::new(); nc];
    for (i, &l) in labels.iter().enumerate() {
        if l >= 0 { clusters[l as usize].push(points[i]); }
    }
    clusters.retain(|c| !c.is_empty());
    clusters
}

fn neighbors(
    pts: &[Point3D], idx: usize, eps2: f32,
    grid: &std::collections::HashMap<(i32, i32, i32), Vec<usize>>, inv: f32,
) -> Vec<usize> {
    let p = &pts[idx];
    let (gx, gy, gz) = ((p.x * inv).floor() as i32, (p.y * inv).floor() as i32, (p.z * inv).floor() as i32);
    let mut out = Vec::new();
    for dx in -1..=1 { for dy in -1..=1 { for dz in -1..=1 {
        if let Some(cell) = grid.get(&(gx+dx, gy+dy, gz+dz)) {
            for &j in cell {
                let q = &pts[j];
                let d = (p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y) + (p.z-q.z)*(p.z-q.z);
                if d <= eps2 { out.push(j); }
            }
        }
    }}}
    out
}
