use minifb::{Key, MouseButton, MouseMode, Window, WindowOptions};

use crate::coordinates::{ConePosition, CoordinateSystem};
use crate::point::Point3D;

const WIDTH: usize = 1024;
const HEIGHT: usize = 700;
const BG_COLOR: u32 = 0x0D0D1A;
const POINT_DIM: u32 = 0x2A2A40;
const ORIGIN_COLOR: u32 = 0x00FF88;
const CONE_COLOR: u32 = 0xFF6B35;
const CONE_OUTLIER: u32 = 0x554433; // dimmed outlier cones
const CONE_VIRTUAL_L: u32 = 0x00CCCC; // cyan = virtual left cone
const CONE_VIRTUAL_R: u32 = 0xCC44CC; // magenta = virtual right cone
const TRACK_LEFT: u32 = 0x4488FF;   // blue = left boundary
const TRACK_RIGHT: u32 = 0xFFDD00;  // yellow = right boundary
const TRACK_CENTER: u32 = 0x00FF88; // green = center line
const GRID_COLOR: u32 = 0x181828;
const AXIS_X: u32 = 0xCC3333;
const AXIS_Y: u32 = 0x33CC33;
const AXIS_Z: u32 = 0x3366CC;
const HUD_BG: u32 = 0x151530;

const MAX_RENDER_POINTS: usize = 8_000;

pub struct Viewer {
    window: Window,
    buffer: Vec<u32>,
    cam_yaw: f32,
    cam_pitch: f32,
    cam_dist: f32,
    cam_target: [f32; 3],
    last_mouse: Option<(f32, f32)>,
    right_last: Option<(f32, f32)>,
    show_all_points: bool,
    show_outliers: bool,
    auto_center: bool,
    cone_toggle_pressed: bool,
    path_toggle_pressed: bool,
    virtual_toggle_pressed: bool,
}

impl Viewer {
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let mut opts = WindowOptions::default();
        opts.resize = true;
        let window = Window::new("Livox 3D Track Detection", WIDTH, HEIGHT, opts)?;
        Ok(Self {
            window,
            buffer: vec![BG_COLOR; WIDTH * HEIGHT],
            cam_yaw: -0.5,
            cam_pitch: 0.6,
            cam_dist: 8.0,
            cam_target: [2.0, 0.0, 0.0],
            last_mouse: None,
            right_last: None,
            show_all_points: false,
            show_outliers: true,
            auto_center: true,
            cone_toggle_pressed: false,
            path_toggle_pressed: false,
            virtual_toggle_pressed: false,
        })
    }

    pub fn is_open(&self) -> bool {
        self.window.is_open() && !self.window.is_key_down(Key::Escape)
    }

    fn handle_input(&mut self) {
        if let Some(scroll) = self.window.get_scroll_wheel() {
            if scroll.1.abs() > 0.01 {
                self.cam_dist *= 1.0 - scroll.1 * 0.08;
                self.cam_dist = self.cam_dist.clamp(0.5, 100.0);
            }
        }
        if self.window.is_key_down(Key::Equal) || self.window.is_key_down(Key::NumPadPlus) { self.cam_dist *= 0.97; }
        if self.window.is_key_down(Key::Minus) || self.window.is_key_down(Key::NumPadMinus) { self.cam_dist *= 1.03; }
        self.cam_dist = self.cam_dist.clamp(0.5, 100.0);

        let left_down = self.window.get_mouse_down(MouseButton::Left);
        if let Some(pos) = self.window.get_mouse_pos(MouseMode::Discard) {
            if left_down {
                if let Some((lx, ly)) = self.last_mouse {
                    self.cam_yaw -= (pos.0 - lx) * 0.005;
                    self.cam_pitch = (self.cam_pitch + (pos.1 - ly) * 0.005).clamp(-1.5, 1.5);
                    self.auto_center = false;
                }
                self.last_mouse = Some((pos.0, pos.1));
            } else { self.last_mouse = None; }

            let right_down = self.window.get_mouse_down(MouseButton::Right);
            if right_down {
                if let Some((rx, ry)) = self.right_last {
                    let sp = self.cam_dist * 0.002;
                    let (sy, cy) = self.cam_yaw.sin_cos();
                    self.cam_target[0] += sy * (pos.0 - rx) * sp;
                    self.cam_target[1] -= cy * (pos.0 - rx) * sp;
                    self.cam_target[2] += (pos.1 - ry) * sp;
                    self.auto_center = false;
                }
                self.right_last = Some((pos.0, pos.1));
            } else { self.right_last = None; }
        }

        let sp = self.cam_dist * 0.02;
        let (sy, cy) = self.cam_yaw.sin_cos();
        if self.window.is_key_down(Key::W) { self.cam_target[0] += cy * sp; self.cam_target[1] += sy * sp; self.auto_center = false; }
        if self.window.is_key_down(Key::S) { self.cam_target[0] -= cy * sp; self.cam_target[1] -= sy * sp; self.auto_center = false; }
        if self.window.is_key_down(Key::A) { self.cam_target[0] += sy * sp; self.cam_target[1] -= cy * sp; self.auto_center = false; }
        if self.window.is_key_down(Key::D) { self.cam_target[0] -= sy * sp; self.cam_target[1] += cy * sp; self.auto_center = false; }
        if self.window.is_key_down(Key::Q) { self.cam_target[2] += sp; }
        if self.window.is_key_down(Key::E) { self.cam_target[2] -= sp; }

        if self.window.is_key_down(Key::R) {
            self.cam_yaw = -0.5; self.cam_pitch = 0.6; self.cam_dist = 8.0;
            self.cam_target = [2.0, 0.0, 0.0]; self.auto_center = true;
        }
        if self.window.is_key_down(Key::Key1) { self.cam_pitch = 1.49; self.cam_yaw = 0.0; }
        if self.window.is_key_down(Key::Key2) { self.cam_pitch = 0.0; self.cam_yaw = 0.0; }
        if self.window.is_key_down(Key::Key3) { self.cam_pitch = 0.0; self.cam_yaw = std::f32::consts::FRAC_PI_2; }

        if self.window.is_key_pressed(Key::P, minifb::KeyRepeat::No) { self.show_all_points = !self.show_all_points; }
        if self.window.is_key_pressed(Key::T, minifb::KeyRepeat::No) { self.show_outliers = !self.show_outliers; }
        if self.window.is_key_pressed(Key::F, minifb::KeyRepeat::No) { self.auto_center = !self.auto_center; }
        self.cone_toggle_pressed = self.window.is_key_pressed(Key::C, minifb::KeyRepeat::No);
        self.path_toggle_pressed = self.window.is_key_pressed(Key::B, minifb::KeyRepeat::No);
        self.virtual_toggle_pressed = self.window.is_key_pressed(Key::V, minifb::KeyRepeat::No);
    }

    fn cam_vectors(&self) -> ([f32; 3], [f32; 3], [f32; 3], [f32; 3]) {
        let (cp, sp) = (self.cam_pitch.cos(), self.cam_pitch.sin());
        let (cy, sy) = (self.cam_yaw.cos(), self.cam_yaw.sin());
        let eye = [
            self.cam_target[0] + self.cam_dist * cp * cy,
            self.cam_target[1] + self.cam_dist * cp * sy,
            self.cam_target[2] + self.cam_dist * sp,
        ];
        let fwd = [self.cam_target[0] - eye[0], self.cam_target[1] - eye[1], self.cam_target[2] - eye[2]];
        let fl = (fwd[0]*fwd[0] + fwd[1]*fwd[1] + fwd[2]*fwd[2]).sqrt();
        let fwd = [fwd[0]/fl, fwd[1]/fl, fwd[2]/fl];
        let r = cross(fwd, [0.0, 0.0, 1.0]);
        let rl = (r[0]*r[0] + r[1]*r[1] + r[2]*r[2]).sqrt();
        let right = if rl > 1e-6 { [r[0]/rl, r[1]/rl, r[2]/rl] } else { [1.0, 0.0, 0.0] };
        let up = cross(right, fwd);
        (eye, fwd, right, up)
    }

    #[inline]
    fn proj(pos: [f32; 3], eye: &[f32; 3], fwd: &[f32; 3], right: &[f32; 3], up: &[f32; 3], hw: f32, hh: f32, f: f32) -> Option<(i32, i32)> {
        let dx = pos[0]-eye[0]; let dy = pos[1]-eye[1]; let dz = pos[2]-eye[2];
        let cz = dx*fwd[0] + dy*fwd[1] + dz*fwd[2];
        if cz < 0.1 { return None; }
        let inv = f / cz;
        let cx = dx*right[0] + dy*right[1] + dz*right[2];
        let cy = dx*up[0] + dy*up[1] + dz*up[2];
        Some(((hw + cx * inv * hh) as i32, (hh - cy * inv * hh) as i32))
    }

    pub fn render(&mut self, points: &[Point3D], coord: &mut CoordinateSystem) {
        self.handle_input();

        // Toggle layers (with dependency cascading)
        if self.cone_toggle_pressed {
            coord.toggle_cones();
        }
        if self.path_toggle_pressed {
            coord.toggle_pathfinding();
        }
        if self.virtual_toggle_pressed {
            coord.toggle_virtual();
        }

        let (w, h) = self.window.get_size();
        if w == 0 || h == 0 { return; }
        let needed = w * h;
        if self.buffer.len() != needed { self.buffer.resize(needed, BG_COLOR); }
        self.buffer.fill(BG_COLOR);

        // Auto-center
        if self.auto_center && points.len() >= 10 {
            let step = (points.len() / 500).max(1);
            let (mut sx, mut sy, mut sz, mut c) = (0.0f32, 0.0, 0.0, 0.0);
            for p in points.iter().step_by(step) { sx += p.x; sy += p.y; sz += p.z; c += 1.0; }
            if c > 0.0 {
                self.cam_target[0] += (sx/c - self.cam_target[0]) * 0.05;
                self.cam_target[1] += (sy/c - self.cam_target[1]) * 0.05;
                self.cam_target[2] += (sz/c - self.cam_target[2]) * 0.05;
            }
        }

        let (eye, fwd, right, up) = self.cam_vectors();
        let hw = w as f32 / 2.0;
        let hh = h as f32 / 2.0;
        let f = 1.0 / (0.6f32).tan();
        let cones = &coord.cones;

        // Grid
        for i in (-4..=4).map(|x| x * 2) {
            let fi = i as f32;
            for t in 0..40 {
                let v = -8.0 + t as f32 * 0.4;
                if let Some((sx, sy)) = Self::proj([v, fi, 0.0], &eye, &fwd, &right, &up, hw, hh, f) { self.px(w, h, sx, sy, GRID_COLOR); }
                if let Some((sx, sy)) = Self::proj([fi, v, 0.0], &eye, &fwd, &right, &up, hw, hh, f) { self.px(w, h, sx, sy, GRID_COLOR); }
            }
        }

        // Axes
        for t in 0..50 {
            let v = t as f32 * 0.02;
            if let Some((sx, sy)) = Self::proj([v, 0.0, 0.0], &eye, &fwd, &right, &up, hw, hh, f) { self.px(w, h, sx, sy, AXIS_X); self.px(w, h, sx+1, sy, AXIS_X); }
            if let Some((sx, sy)) = Self::proj([0.0, v, 0.0], &eye, &fwd, &right, &up, hw, hh, f) { self.px(w, h, sx, sy, AXIS_Y); self.px(w, h, sx, sy+1, AXIS_Y); }
            if let Some((sx, sy)) = Self::proj([0.0, 0.0, v], &eye, &fwd, &right, &up, hw, hh, f) { self.px(w, h, sx, sy, AXIS_Z); self.px(w, h, sx+1, sy, AXIS_Z); }
        }

        // Origin
        if let Some((sx, sy)) = Self::proj([0.0, 0.0, 0.0], &eye, &fwd, &right, &up, hw, hh, f) {
            for dx in -2..=2i32 { for dy in -2..=2i32 { if dx*dx+dy*dy <= 4 { self.px(w, h, sx+dx, sy+dy, ORIGIN_COLOR); } } }
        }

        // Points
        if self.show_all_points {
            let step = (points.len() / MAX_RENDER_POINTS).max(1);
            for p in points.iter().step_by(step) {
                if let Some((sx, sy)) = Self::proj([p.x, p.y, p.z], &eye, &fwd, &right, &up, hw, hh, f) {
                    self.px(w, h, sx, sy, height_color(p.z));
                }
            }
        } else {
            let step = (points.len() / (MAX_RENDER_POINTS / 2)).max(1);
            for p in points.iter().step_by(step) {
                if let Some((sx, sy)) = Self::proj([p.x, p.y, p.z], &eye, &fwd, &right, &up, hw, hh, f) {
                    let near = cones.iter().any(|c| (p.x-c.x)*(p.x-c.x)+(p.y-c.y)*(p.y-c.y) < 0.25);
                    self.px(w, h, sx, sy, if near { height_color(p.z) } else { POINT_DIM });
                }
            }
        }

        // --- Track lines (only if pathfinding is on) ---
        if coord.show_pathfinding {
            // Left boundary (blue)
            self.draw_track_line(w, h, &coord.track_left, cones, TRACK_LEFT, &eye, &fwd, &right, &up, hw, hh, f);
            // Right boundary (yellow)
            self.draw_track_line(w, h, &coord.track_right, cones, TRACK_RIGHT, &eye, &fwd, &right, &up, hw, hh, f);
            // Center line (green, dashed)
            for i in 1..coord.track_center.len() {
                let a = coord.track_center[i - 1];
                let b = coord.track_center[i];
                self.draw_line_3d(w, h, [a[0], a[1], 0.05], [b[0], b[1], 0.05], TRACK_CENTER, &eye, &fwd, &right, &up, hw, hh, f);
            }
        }

        // --- Cone markers (only if cones are on) ---
        for (i, cone) in cones.iter().enumerate() {
            if !coord.show_cones { break; }
            if !cone.on_track && !self.show_outliers { continue; }

            let color = if cone.is_virtual {
                match cone.side { 1 => CONE_VIRTUAL_L, -1 => CONE_VIRTUAL_R, _ => CONE_VIRTUAL_L }
            } else if cone.on_track {
                match cone.side { 1 => TRACK_LEFT, -1 => TRACK_RIGHT, _ => CONE_COLOR }
            } else {
                CONE_OUTLIER
            };

            // Vertical line
            for t in 0..10 {
                let z = cone.height * t as f32 / 9.0;
                if let Some((sx, sy)) = Self::proj([cone.x, cone.y, z], &eye, &fwd, &right, &up, hw, hh, f) {
                    self.px(w, h, sx, sy, color); self.px(w, h, sx+1, sy, color);
                }
            }

            // Diamond
            if let Some((sx, sy)) = Self::proj([cone.x, cone.y, cone.height + 0.12], &eye, &fwd, &right, &up, hw, hh, f) {
                let r = if cone.on_track { 6i32 } else { 3 };
                for dx in -r..=r { for dy in -r..=r {
                    if dx.abs() + dy.abs() <= r { self.px(w, h, sx+dx, sy+dy, color); }
                }}
                if cone.on_track {
                    self.draw_number(w, h, (sx-1).max(0) as usize, (sy-2).max(0) as usize, (i+1) as u8, 0xFFFFFF);
                }
            }
        }

        // HUD
        self.draw_hud(w, h, points.len(), coord);

        let _ = self.window.update_with_buffer(&self.buffer, w, h);
    }

    fn draw_track_line(
        &mut self, w: usize, h: usize,
        chain: &[usize], cones: &[ConePosition], color: u32,
        eye: &[f32; 3], fwd: &[f32; 3], right: &[f32; 3], up: &[f32; 3],
        hw: f32, hh: f32, f: f32,
    ) {
        for i in 1..chain.len() {
            let a = &cones[chain[i - 1]];
            let b = &cones[chain[i]];
            self.draw_line_3d(w, h, [a.x, a.y, 0.02], [b.x, b.y, 0.02], color, eye, fwd, right, up, hw, hh, f);
        }
    }

    fn draw_line_3d(
        &mut self, w: usize, h: usize,
        a: [f32; 3], b: [f32; 3], color: u32,
        eye: &[f32; 3], fwd: &[f32; 3], right: &[f32; 3], up: &[f32; 3],
        hw: f32, hh: f32, f: f32,
    ) {
        let dist = ((b[0]-a[0]).powi(2) + (b[1]-a[1]).powi(2) + (b[2]-a[2]).powi(2)).sqrt();
        let steps = (dist * 30.0).max(5.0).min(80.0) as usize;
        let mut prev: Option<(i32, i32)> = None;
        for t in 0..=steps {
            let frac = t as f32 / steps as f32;
            let pos = [
                a[0] + (b[0]-a[0]) * frac,
                a[1] + (b[1]-a[1]) * frac,
                a[2] + (b[2]-a[2]) * frac,
            ];
            if let Some((sx, sy)) = Self::proj(pos, eye, fwd, right, up, hw, hh, f) {
                // Draw thick line by also filling between prev and current
                self.px(w, h, sx, sy, color);
                self.px(w, h, sx+1, sy, color);
                self.px(w, h, sx, sy+1, color);
                if let Some((px, py)) = prev {
                    // Bresenham-ish fill for gaps
                    let ddx = (sx - px).abs();
                    let ddy = (sy - py).abs();
                    if ddx > 1 || ddy > 1 {
                        let ms = ddx.max(ddy);
                        for s in 1..ms {
                            let ix = px + (sx - px) * s / ms;
                            let iy = py + (sy - py) * s / ms;
                            self.px(w, h, ix, iy, color);
                            self.px(w, h, ix+1, iy, color);
                        }
                    }
                }
                prev = Some((sx, sy));
            }
        }
    }

    #[inline]
    fn px(&mut self, w: usize, h: usize, x: i32, y: i32, color: u32) {
        if x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h {
            self.buffer[y as usize * w + x as usize] = color;
        }
    }

    fn draw_hud(&mut self, w: usize, h: usize, npts: usize, coord: &CoordinateSystem) {
        let cones = &coord.cones;
        let on_track = cones.iter().filter(|c| c.on_track).count();
        let pw = 210usize;
        let ph = 50 + on_track * 12 + 20;
        for x in 4..(4+pw).min(w) { for y in 4..(4+ph).min(h) { self.buffer[y*w+x] = HUD_BG; } }

        self.draw_text(w, h, 10, 10, "PTS", 0x888888);
        self.draw_dec(w, h, 30, 10, npts as u32, 0xAAAAAA);

        self.draw_text(w, h, 10, 20, "TRACK", 0x888888);
        self.draw_dec(w, h, 42, 20, on_track as u32, TRACK_CENTER);

        self.draw_text(w, h, 80, 20, "ALL", 0x888888);
        self.draw_dec(w, h, 100, 20, cones.len() as u32, CONE_COLOR);

        let mut row = 0;
        for (i, c) in cones.iter().enumerate() {
            if !c.on_track { continue; }
            let y = 34 + row * 12;
            if y + 5 >= h { break; }
            let side_color = match c.side { 1 => TRACK_LEFT, -1 => TRACK_RIGHT, _ => CONE_COLOR };
            self.draw_number(w, h, 10, y, (i+1) as u8, side_color);
            let side_str = match c.side { 1 => "L", -1 => "R", _ => "?" };
            self.draw_text(w, h, 18, y, side_str, side_color);
            let cm = (c.distance * 100.0) as u32;
            self.draw_dec(w, h, 28, y, cm, 0xCCCCCC);
            self.draw_text(w, h, 28 + dw(cm) + 2, y, "CM", 0x666666);
            row += 1;
        }

        let by = h.saturating_sub(12);
        let virt = coord.cones.iter().filter(|c| c.is_virtual && c.on_track).count();
        if virt > 0 {
            self.draw_text(w, h, 140, 20, "VIRT", 0x888888);
            self.draw_dec(w, h, 162, 20, virt as u32, CONE_VIRTUAL_L);
        }

        let c_mode = if coord.show_cones { "C ON" } else { "C OFF" };
        let b_mode = if coord.show_pathfinding { "B ON" } else { "B OFF" };
        let v_mode = if coord.use_virtual { "V ON" } else { "V OFF" };
        let p_mode = if self.show_all_points { "P ALL" } else { "P CONES" };
        let t_mode = if self.show_outliers { "T SHOW OUT" } else { "T HIDE OUT" };
        self.draw_text(w, h, 4, by, p_mode, 0x444466);
        self.draw_text(w, h, 48, by, t_mode, 0x444466);
        self.draw_text(w, h, 110, by, c_mode, if coord.show_cones { 0xFF6B35 } else { 0x444466 });
        self.draw_text(w, h, 130, by, b_mode, if coord.show_pathfinding { 0x4488FF } else { 0x444466 });
        self.draw_text(w, h, 150, by, v_mode, if coord.use_virtual { 0x00CCCC } else { 0x444466 });
        self.draw_text(w, h, 174, by, "LMB ROT  SCROLL ZOOM  R RESET", 0x333355);
    }

    fn draw_number(&mut self, w: usize, h: usize, x: usize, y: usize, n: u8, color: u32) {
        let bm = DIGITS[(n % 10) as usize];
        for row in 0..5 { for col in 0..3 {
            if bm[row] & (4 >> col) != 0 {
                let px = x+col; let py = y+row;
                if px < w && py < h { self.buffer[py*w+px] = color; }
            }
        }}
    }

    fn draw_dec(&mut self, w: usize, h: usize, x: usize, y: usize, mut n: u32, color: u32) {
        if n == 0 { self.draw_number(w, h, x, y, 0, color); return; }
        let mut buf = [0u8; 10]; let mut cnt = 0;
        while n > 0 { buf[cnt] = (n%10) as u8; n /= 10; cnt += 1; }
        let mut cx = x;
        for i in (0..cnt).rev() { self.draw_number(w, h, cx, y, buf[i], color); cx += 4; }
    }

    fn draw_text(&mut self, w: usize, h: usize, x: usize, y: usize, text: &str, color: u32) {
        let mut cx = x;
        for ch in text.bytes() {
            if ch == b' ' { cx += 4; continue; }
            if let Some(bm) = char_bm(ch) {
                for row in 0..5 { for col in 0..3 {
                    if bm[row] & (4 >> col) != 0 {
                        let px = cx+col; let py = y+row;
                        if px < w && py < h { self.buffer[py*w+px] = color; }
                    }
                }}
                cx += 4;
            } else { cx += 4; }
        }
    }

    pub fn sync_size(&mut self) {
        let (w, h) = self.window.get_size();
        let needed = w * h;
        if self.buffer.len() != needed { self.buffer.resize(needed, BG_COLOR); }
    }
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
}

fn height_color(z: f32) -> u32 {
    let t = ((z + 0.5) / 2.0).clamp(0.0, 1.0);
    let (r, g, b) = if t < 0.33 {
        let f = t / 0.33;
        (40, (100.0 + 120.0 * f) as u8, (200.0 + 55.0 * f) as u8)
    } else if t < 0.66 {
        let f = (t - 0.33) / 0.33;
        ((40.0 + 180.0 * f) as u8, (220.0 + 35.0 * f) as u8, (255.0 - 130.0 * f) as u8)
    } else {
        let f = (t - 0.66) / 0.34;
        ((220.0 + 35.0 * f) as u8, 255, (125.0 + 130.0 * f) as u8)
    };
    (r as u32) << 16 | (g as u32) << 8 | b as u32
}

fn dw(mut n: u32) -> usize {
    if n == 0 { return 3; }
    let mut w: usize = 0;
    while n > 0 { w += 4; n /= 10; }
    w.saturating_sub(1)
}

#[rustfmt::skip]
const DIGITS: [[u8; 5]; 10] = [
    [0b111, 0b101, 0b101, 0b101, 0b111], [0b010, 0b110, 0b010, 0b010, 0b111],
    [0b111, 0b001, 0b111, 0b100, 0b111], [0b111, 0b001, 0b111, 0b001, 0b111],
    [0b101, 0b101, 0b111, 0b001, 0b001], [0b111, 0b100, 0b111, 0b001, 0b111],
    [0b111, 0b100, 0b111, 0b101, 0b111], [0b111, 0b001, 0b010, 0b010, 0b010],
    [0b111, 0b101, 0b111, 0b101, 0b111], [0b111, 0b101, 0b111, 0b001, 0b111],
];

fn char_bm(ch: u8) -> Option<[u8; 5]> {
    if ch >= b'0' && ch <= b'9' { return Some(DIGITS[(ch - b'0') as usize]); }
    #[rustfmt::skip]
    let map: &[(u8, [u8; 5])] = &[
        (b'A', [0b010,0b101,0b111,0b101,0b101]), (b'B', [0b110,0b101,0b110,0b101,0b110]),
        (b'C', [0b111,0b100,0b100,0b100,0b111]), (b'D', [0b110,0b101,0b101,0b101,0b110]),
        (b'E', [0b111,0b100,0b110,0b100,0b111]), (b'F', [0b111,0b100,0b110,0b100,0b100]),
        (b'G', [0b111,0b100,0b101,0b101,0b111]), (b'H', [0b101,0b101,0b111,0b101,0b101]),
        (b'I', [0b111,0b010,0b010,0b010,0b111]), (b'L', [0b100,0b100,0b100,0b100,0b111]),
        (b'M', [0b101,0b111,0b111,0b101,0b101]), (b'N', [0b101,0b111,0b111,0b111,0b101]),
        (b'O', [0b111,0b101,0b101,0b101,0b111]), (b'P', [0b111,0b101,0b111,0b100,0b100]),
        (b'R', [0b111,0b101,0b111,0b110,0b101]), (b'S', [0b111,0b100,0b111,0b001,0b111]),
        (b'T', [0b111,0b010,0b010,0b010,0b010]), (b'U', [0b101,0b101,0b101,0b101,0b111]),
        (b'V', [0b101,0b101,0b101,0b101,0b010]), (b'W', [0b101,0b101,0b111,0b111,0b101]),
        (b'X', [0b101,0b101,0b010,0b101,0b101]), (b'Z', [0b111,0b001,0b010,0b100,0b111]),
    ];
    for (c, bm) in map { if *c == ch { return Some(*bm); } }
    None
}
