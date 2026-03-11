use super::types::*;

pub(crate) fn build_knn_graph(cones: &[ConePosition]) -> Vec<Vec<usize>> {
    let n = cones.len();
    let mut adj = vec![Vec::new(); n];

    for i in 0..n {
        let mut dists: Vec<(usize, f32)> = (0..n)
            .filter(|&j| j != i)
            .map(|j| {
                let dx = cones[j].x - cones[i].x;
                let dy = cones[j].y - cones[i].y;
                (j, (dx * dx + dy * dy).sqrt())
            })
            .filter(|(_, d)| *d < MAX_NEIGHBOR_DIST)
            .collect();
        dists.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        dists.truncate(K_NEIGHBORS);
        for (j, _) in dists {
            if !adj[i].contains(&j) {
                adj[i].push(j);
            }
        }
    }

    // Keep mutual edges + close directed edges
    let mut result = vec![Vec::new(); n];
    for i in 0..n {
        for &j in &adj[i] {
            let d = {
                let dx = cones[i].x - cones[j].x;
                let dy = cones[i].y - cones[j].y;
                (dx * dx + dy * dy).sqrt()
            };
            if adj[j].contains(&i) || d < 4.0 {
                if !result[i].contains(&j) {
                    result[i].push(j);
                }
            }
        }
    }
    result
}
