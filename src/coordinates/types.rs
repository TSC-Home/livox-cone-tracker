use crate::detection::Cone;

// --- Tracking config ---
pub(crate) const MAX_MISSED: u32 = 8;
pub(crate) const MATCH_RADIUS: f32 = 0.40;
pub(crate) const SMOOTH: f32 = 0.6;

// --- Track fitting config ---
pub(crate) const K_NEIGHBORS: usize = 6;
pub(crate) const MAX_NEIGHBOR_DIST: f32 = 6.0; // KNN graph radius - keep tight
pub(crate) const MAX_PATH_LEN: usize = 16;
pub(crate) const MAX_SEGMENT_DIST: f32 = 3.0; // max distance between consecutive cones in a path
pub(crate) const ANGLE_THRESHOLD_ABS: f32 = 75.0;
pub(crate) const ANGLE_THRESHOLD_DIR: f32 = 50.0;
pub(crate) const CLOSE_DIST_BYPASS: f32 = 4.0;
pub(crate) const REVERSAL_THRESHOLD: f32 = 1.3;
pub(crate) const MIN_TRACK_WIDTH: f32 = 1.5;
pub(crate) const MAX_TRACK_WIDTH: f32 = 5.0;
pub(crate) const MAX_MATCH_X_DIFF: f32 = 4.0;
pub(crate) const VIRTUAL_GAP_THRESHOLD: f32 = 5.0;
pub(crate) const MAX_VIRTUAL_SPACING: f32 = 3.5;

// --- Forward extension (beyond KNN graph) ---
pub(crate) const EXTEND_MAX_DIST: f32 = 6.0; // search radius for extension
pub(crate) const EXTEND_MAX_ANGLE: f32 = 70.0;
pub(crate) const EXTEND_MAX_CONES: usize = 6;

// --- Track persistence config ---
pub(crate) const TRACK_MATCH_RADIUS: f32 = 0.8;
pub(crate) const TRACK_KEEP_FRAMES: u32 = 12; // keep previous track longer before forcing update

#[derive(Debug, Clone, Copy)]
pub(crate) struct TrackedCone {
    pub x: f32,
    pub y: f32,
    pub height: f32,
    pub missed: u32,
    pub hit_count: u32,
}

impl TrackedCone {
    pub fn from_cone(det: &Cone) -> Self {
        Self {
            x: det.x,
            y: det.y,
            height: det.height,
            missed: 0,
            hit_count: 1,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ConePosition {
    pub x: f32,
    pub y: f32,
    pub distance: f32,
    pub angle_deg: f32,
    pub height: f32,
    pub on_track: bool,
    pub side: i8, // -1 right, 0 unknown, 1 left
    pub is_virtual: bool,
}
