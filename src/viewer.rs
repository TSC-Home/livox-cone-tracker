use minifb::{Key, MouseButton, MouseMode, Window, WindowOptions};

use crate::coordinates::ConePosition;
use crate::point::Point3D;

const WIDTH: usize = 800;
const HEIGHT: usize = 800;
const BG_COLOR: u32 = 0x1A1A2E;
const POINT_COLOR: u32 = 0x6A6A9A;
const CONE_COLOR: u32 = 0xFF6B35;
const ORIGIN_COLOR: u32 = 0x00FF88;
const GRID_COLOR: u32 = 0x252540;
const TEXT_COLOR: u32 = 0xFFFFFF;

pub struct Viewer {
    window: Window,
    buffer: Vec<u32>,
    // Camera state
    offset_x: f32,
    offset_y: f32,
    scale: f32,
    // Mouse drag state
    drag_start: Option<(f32, f32)>,
    drag_offset_start: (f32, f32),
    last_scroll: f32,
}

impl Viewer {
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let mut opts = WindowOptions::default();
        opts.resize = true;
        let window = Window::new("Livox Cone Detection", WIDTH, HEIGHT, opts)?;
        let buffer = vec![BG_COLOR; WIDTH * HEIGHT];
        Ok(Self {
            window, buffer,
            offset_x: 0.0, offset_y: 0.0,
            scale: 40.0,
            drag_start: None,
            drag_offset_start: (0.0, 0.0),
            last_scroll: 0.0,
        })
    }

    pub fn is_open(&self) -> bool {
        self.window.is_open() && !self.window.is_key_down(Key::Escape)
    }

    fn handle_input(&mut self) {
        // Zoom with scroll or +/-
        if self.window.is_key_down(Key::Equal) || self.window.is_key_down(Key::NumPadPlus) {
            self.scale *= 1.05;
        }
        if self.window.is_key_down(Key::Minus) || self.window.is_key_down(Key::NumPadMinus) {
            self.scale *= 0.95;
        }
        self.scale = self.scale.clamp(2.0, 500.0);

        // Scroll wheel zoom
        if let Some(scroll) = self.window.get_scroll_wheel() {
            let factor = 1.0 + scroll.1 * 0.1;
            self.scale *= factor;
            self.scale = self.scale.clamp(2.0, 500.0);
        }

        // Pan with WASD or arrow keys
        let pan_speed = 5.0 / self.scale;
        if self.window.is_key_down(Key::W) || self.window.is_key_down(Key::Up) {
            self.offset_y -= pan_speed;
        }
        if self.window.is_key_down(Key::S) || self.window.is_key_down(Key::Down) {
            self.offset_y += pan_speed;
        }
        if self.window.is_key_down(Key::A) || self.window.is_key_down(Key::Left) {
            self.offset_x -= pan_speed;
        }
        if self.window.is_key_down(Key::D) || self.window.is_key_down(Key::Right) {
            self.offset_x += pan_speed;
        }

        // Reset view with R
        if self.window.is_key_down(Key::R) {
            self.offset_x = 0.0;
            self.offset_y = 0.0;
            self.scale = 40.0;
        }

        // Mouse drag to pan
        let mouse_down = self.window.get_mouse_down(MouseButton::Left);
        if let Some(pos) = self.window.get_mouse_pos(MouseMode::Discard) {
            if mouse_down {
                if let Some((sx, sy)) = self.drag_start {
                    let dx = (pos.0 - sx) / self.scale;
                    let dy = (pos.1 - sy) / self.scale;
                    self.offset_x = self.drag_offset_start.0 - dx;
                    self.offset_y = self.drag_offset_start.1 + dy;
                } else {
                    self.drag_start = Some((pos.0, pos.1));
                    self.drag_offset_start = (self.offset_x, self.offset_y);
                }
            } else {
                self.drag_start = None;
            }
        }
    }

    pub fn render(&mut self, points: &[Point3D], cones: &[ConePosition]) {
        self.handle_input();
        self.buffer.fill(BG_COLOR);
        let (w, h) = self.window.get_size();
        let cx = (w / 2) as f32;
        let cy = (h / 2) as f32;
        let s = self.scale;
        let ox = self.offset_x;
        let oy = self.offset_y;

        self.draw_grid(w, h, cx, cy, s, ox, oy);
        self.draw_origin(w, cx, cy, s, ox, oy);
        self.draw_points(points, w, h, cx, cy, s, ox, oy);
        self.draw_cones(cones, w, h, cx, cy, s, ox, oy);
        self.draw_hud(w, h, points.len(), cones.len());

        let _ = self.window.update_with_buffer(&self.buffer, w, h);
    }

    fn world_to_screen(&self, wx: f32, wy: f32, cx: f32, cy: f32, s: f32, ox: f32, oy: f32) -> (i32, i32) {
        let px = (cx + (wx - ox) * s) as i32;
        let py = (cy - (wy - oy) * s) as i32;
        (px, py)
    }

    fn draw_grid(&mut self, w: usize, h: usize, cx: f32, cy: f32, s: f32, ox: f32, oy: f32) {
        let max_r = (w.max(h) as f32 / s / 2.0).ceil() as i32 + 1;
        let base_x = ox.floor() as i32;
        let base_y = oy.floor() as i32;
        for i in (base_x - max_r)..=(base_x + max_r) {
            let (px, _) = self.world_to_screen(i as f32, 0.0, cx, cy, s, ox, oy);
            if px >= 0 && (px as usize) < w {
                for py in 0..h { self.set_pixel(w, px as usize, py, GRID_COLOR); }
            }
        }
        for i in (base_y - max_r)..=(base_y + max_r) {
            let (_, py) = self.world_to_screen(0.0, i as f32, cx, cy, s, ox, oy);
            if py >= 0 && (py as usize) < h {
                for px in 0..w { self.set_pixel(w, px, py as usize, GRID_COLOR); }
            }
        }
    }

    fn draw_origin(&mut self, w: usize, cx: f32, cy: f32, s: f32, ox: f32, oy: f32) {
        let (opx, opy) = self.world_to_screen(0.0, 0.0, cx, cy, s, ox, oy);
        for dx in -3..=3i32 {
            for dy in -3..=3i32 {
                let x = (opx + dx) as usize;
                let y = (opy + dy) as usize;
                if x < w && y < w { self.set_pixel(w, x, y, ORIGIN_COLOR); }
            }
        }
    }

    fn draw_points(&mut self, points: &[Point3D], w: usize, h: usize, cx: f32, cy: f32, s: f32, ox: f32, oy: f32) {
        for p in points {
            let (px, py) = self.world_to_screen(p.x, p.y, cx, cy, s, ox, oy);
            if px >= 0 && py >= 0 && (px as usize) < w && (py as usize) < h {
                self.set_pixel(w, px as usize, py as usize, POINT_COLOR);
            }
        }
    }

    fn draw_cones(&mut self, cones: &[ConePosition], w: usize, h: usize, cx: f32, cy: f32, s: f32, ox: f32, oy: f32) {
        for (i, cone) in cones.iter().enumerate() {
            let (px, py) = self.world_to_screen(cone.x, cone.y, cx, cy, s, ox, oy);
            // Filled circle
            for dx in -6..=6i32 {
                for dy in -6..=6i32 {
                    if dx * dx + dy * dy <= 36 {
                        let x = (px + dx) as usize;
                        let y = (py + dy) as usize;
                        if x < w && y < h { self.set_pixel(w, x, y, CONE_COLOR); }
                    }
                }
            }
            // Crosshair
            for d in 7..12i32 {
                for &(dx, dy) in &[(d,0),(-d,0),(0,d),(0,-d)] {
                    let x = (px + dx) as usize;
                    let y = (py + dy) as usize;
                    if x < w && y < h { self.set_pixel(w, x, y, CONE_COLOR); }
                }
            }
            // Number
            self.draw_number(w, h, (px - 1) as usize, (py - 12) as usize,
                (i + 1) as u8, TEXT_COLOR);
        }
    }

    fn draw_hud(&mut self, w: usize, h: usize, point_count: usize, cone_count: usize) {
        // Simple HUD: show zoom level and counts at top-left
        // Just draw colored bars as indicators
        let zoom_bar = ((self.scale / 500.0) * 100.0) as usize;
        for x in 4..(4 + zoom_bar.min(w - 8)) {
            for y in 4..8 { self.set_pixel(w, x, y, ORIGIN_COLOR); }
        }
        if cone_count > 0 {
            let cone_bar = cone_count * 15;
            for x in 4..(4 + cone_bar.min(w - 8)) {
                for y in 10..14 { self.set_pixel(w, x, y, CONE_COLOR); }
            }
        }
    }

    fn draw_number(&mut self, w: usize, h: usize, x: usize, y: usize, n: u8, color: u32) {
        #[rustfmt::skip]
        let digits: [[u8; 5]; 10] = [
            [0b111, 0b101, 0b101, 0b101, 0b111],
            [0b010, 0b110, 0b010, 0b010, 0b111],
            [0b111, 0b001, 0b111, 0b100, 0b111],
            [0b111, 0b001, 0b111, 0b001, 0b111],
            [0b101, 0b101, 0b111, 0b001, 0b001],
            [0b111, 0b100, 0b111, 0b001, 0b111],
            [0b111, 0b100, 0b111, 0b101, 0b111],
            [0b111, 0b001, 0b010, 0b010, 0b010],
            [0b111, 0b101, 0b111, 0b101, 0b111],
            [0b111, 0b101, 0b111, 0b001, 0b111],
        ];
        let d = (n % 10) as usize;
        for row in 0..5 {
            for col in 0..3 {
                if digits[d][row] & (0b100 >> col) != 0 {
                    let px = x + col;
                    let py = y + row;
                    if px < w && py < h { self.set_pixel(w, px, py, color); }
                }
            }
        }
    }

    #[inline]
    fn set_pixel(&mut self, w: usize, x: usize, y: usize, color: u32) {
        let idx = y * w + x;
        if idx < self.buffer.len() { self.buffer[idx] = color; }
    }

    pub fn sync_size(&mut self) {
        let (w, h) = self.window.get_size();
        let needed = w * h;
        if self.buffer.len() != needed { self.buffer.resize(needed, BG_COLOR); }
    }
}
