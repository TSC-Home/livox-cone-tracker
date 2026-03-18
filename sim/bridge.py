#!/usr/bin/env python3
"""
Livox HAP TX LiDAR Simulator – FastTube

Generates synthetic point-cloud data that matches the Livox HAP binary
UDP protocol expected by the Rust detection app (src/hap/protocol.rs).

Key design decisions
────────────────────
• Frustum (truncated-cone) geometry instead of cylinder, so the cluster
  shape seen by DBSCAN matches what real cone detections look like.
• Per-frame random seed → different rays every frame → after 500 ms the
  voxel-downsampler accumulates many unique surface points per cone,
  satisfying min_cluster_points=4 and giving good height/width estimates.
• 8 000 rays/frame @ 20 Hz ≈ 80 000 pts per 500 ms detection window,
  roughly matching real Livox HAP TX density (240 kpts/s × 0.5 s).

Coordinate frame sent in the packets (sensor / Livox frame):
  x+: forward   y+: left   z+: up   (ROS / Livox HAP convention)
"""

import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import socket
import struct
import math
import time
import argparse

import numpy as np

from track import (
    CONES, OVAL_CONES, LIDAR_HEIGHT,
    CONE_RADIUS_BASE, CONE_RADIUS_TOP, CONE_HEIGHT,
)

# ── Livox HAP TX specs ───────────────────────────────────────────────────────
H_FOV_HALF = math.radians(35.2)   # ±35.2° horizontal
V_FOV_HALF = math.radians(38.6)   # ±38.6° vertical

FRAME_RATE     = 20      # Hz
RAYS_PER_FRAME = 8_000   # rays per frame  (~real HAP TX density at 20 Hz)

GROUND_Z  = -LIDAR_HEIGHT  # cone base z in sensor frame
NOISE_STD = 0.004          # 4 mm σ  (HAP TX range noise spec)
MAX_RANGE = 60.0           # m


# ── Livox HAP packet builder ─────────────────────────────────────────────────
def make_livox_packet(points_mm: list) -> bytes:
    """
    Header (36 bytes, layout from protocol.rs):
      version(1) + length(2) + time_interval(2) + dot_num(2) +
      udp_cnt(2) + frame_cnt(1) + data_type(1) + time_type(1) +
      rsvd(12) + crc32(4) + timestamp(8)
    Each point (14 bytes): x(i32 mm) y(i32 mm) z(i32 mm) refl(u8) tag(u8)
    """
    n  = len(points_mm)
    ts = int(time.time() * 1e9) & 0xFFFF_FFFF_FFFF_FFFF
    hdr  = struct.pack('<B',  0x01)
    hdr += struct.pack('<H',  36 + n * 14)
    hdr += struct.pack('<H',  0)
    hdr += struct.pack('<H',  n)
    hdr += struct.pack('<H',  0)
    hdr += struct.pack('<B',  0)
    hdr += struct.pack('<B',  0x01)      # Cartesian-high data type
    hdr += struct.pack('<B',  0)
    hdr += b'\x00' * 12
    hdr += struct.pack('<I',  0)
    hdr += struct.pack('<Q',  ts)
    assert len(hdr) == 36
    body = b''.join(
        struct.pack('<iiiBB', x, y, z, min(255, max(0, r)), 0)
        for x, y, z, r in points_mm
    )
    return hdr + body


# ── Ray-direction generator ──────────────────────────────────────────────────
def build_ray_directions(n: int, seed: int) -> np.ndarray:
    """
    Stratified-uniform sample within the HAP TX FOV.
    A different seed every frame guarantees unique voxel positions
    after accumulation, so DBSCAN can form clusters from the 500 ms window.
    """
    rng = np.random.default_rng(seed)
    k   = int(math.sqrt(n))
    rem = n - k * k

    cell_az = 2.0 * H_FOV_HALF / k
    cell_el = 2.0 * V_FOV_HALF / k

    # One jittered sample per cell
    ci = np.arange(k, dtype=np.float64)
    cj = np.arange(k, dtype=np.float64)
    ii, jj = np.meshgrid(ci, cj, indexing='ij')
    ii = ii.ravel(); jj = jj.ravel()
    jitter_az = rng.uniform(0, 1, k * k)
    jitter_el = rng.uniform(0, 1, k * k)
    az = -H_FOV_HALF + (ii + jitter_az) * cell_az
    el = -V_FOV_HALF + (jj + jitter_el) * cell_el

    if rem > 0:
        az = np.concatenate([az, rng.uniform(-H_FOV_HALF, H_FOV_HALF, rem)])
        el = np.concatenate([el, rng.uniform(-V_FOV_HALF, V_FOV_HALF, rem)])

    dx = np.sin(az) * np.cos(el)
    dy = np.cos(az) * np.cos(el)
    dz = np.sin(el)
    lengths = np.sqrt(dx**2 + dy**2 + dz**2)
    return np.column_stack([dx / lengths, dy / lengths, dz / lengths])


# ── Frustum (truncated-cone) intersection ────────────────────────────────────
def intersect_frustum_batch(
    dirs: np.ndarray,
    cx: float, cy: float,
    cz_base: float, r_base: float, r_top: float, height: float,
) -> np.ndarray:
    """
    Vectorised ray–frustum intersection.

    Frustum: vertical axis at (cx, cy), from z=cz_base (radius r_base)
             to z=cz_base+height (radius r_top).

    Ray: P(t) = t * d  (origin at sensor = world origin in sensor frame).

    At height h above base: r(h) = r_base + dr*h,  dr = (r_top - r_base)/height
    Substituting h = t*dz - cz_base:
      r(t) = (r_base - dr*cz_base) + dr*t*dz  =:  R0 + dr*dz*t

    Intersection with cone surface:
      (t*dx - cx)^2 + (t*dy - cy)^2 = (R0 + dr*dz*t)^2

    → quadratic:  a·t² + b·t + c = 0
    """
    dx, dy, dz = dirs[:, 0], dirs[:, 1], dirs[:, 2]
    ox = -cx
    oy = -cy
    dr = (r_top - r_base) / height
    R0 = r_base - dr * cz_base      # effective radius at z = 0

    a = dx*dx + dy*dy - (dr * dz)**2
    b = 2.0 * (ox*dx + oy*dy - R0 * dr * dz)
    c = ox*ox + oy*oy - R0*R0

    disc = b*b - 4.0*a*c
    t_out = np.full(len(dirs), np.inf)

    valid_disc = disc >= 0
    if not np.any(valid_disc):
        return t_out

    sqrt_d  = np.sqrt(np.where(valid_disc, disc, 0.0))
    safe_a  = np.where(np.abs(a) > 1e-12, a, 1.0)

    for sign in (-1.0, 1.0):
        t = (-b + sign * sqrt_d) / (2.0 * safe_a)
        # z-bounds check
        z_hit = t * dz
        in_z  = (z_hit >= cz_base) & (z_hit <= cz_base + height)
        keep  = valid_disc & in_z & (t > 0.05) & (t < t_out)
        t_out = np.where(keep, t, t_out)

    return t_out


# ── Ground-plane intersection ────────────────────────────────────────────────
def intersect_ground_batch(dirs: np.ndarray, ground_z: float) -> np.ndarray:
    dz     = dirs[:, 2]
    safe   = np.where(np.abs(dz) > 1e-10, dz, np.nan)
    t      = ground_z / safe
    return np.where((t > 0.05) & np.isfinite(t), t, np.inf)


# ── Simulator ────────────────────────────────────────────────────────────────
class LidarSimulator:
    def __init__(self, host: str = "127.0.0.1", port: int = 57000,
                 circle: bool = False):
        self.sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (host, port)
        self.frame  = 0

        cone_list = OVAL_CONES if circle else CONES

        # Cone table in ray-casting frame (x=right+, y=forward+, z=up+):
        # bridge internal frame has x_sim=right, y_sim=forward.
        # CONES / OVAL_CONES store (x_sim, y_sim, side).
        self.cones = [
            (
                float(x), float(y),
                GROUND_Z,
                CONE_RADIUS_BASE, CONE_RADIUS_TOP, CONE_HEIGHT,
                240 if side == 'left' else 220,
            )
            for (x, y, side) in cone_list
        ]

        track_name = "oval" if circle else "straight"
        print(f"[bridge] track={track_name} | {len(self.cones)} cones | "
              f"{RAYS_PER_FRAME} rays/frame @ {FRAME_RATE} Hz")
        print(f"[bridge] Sending to {host}:{port}")

    # ── one frame ─────────────────────────────────────────────────────────────
    def simulate_frame(self) -> list:
        # Fresh directions every frame → different voxels → accumulation works
        rng  = np.random.default_rng(self.frame)
        dirs = build_ray_directions(RAYS_PER_FRAME, seed=self.frame)
        n    = len(dirs)

        best_t    = np.full(n, np.inf)
        best_refl = np.zeros(n, dtype=np.int32)

        # Frustum intersections
        for (cx, cy, cz_base, r_base, r_top, height, refl) in self.cones:
            t = intersect_frustum_batch(
                dirs, cx, cy, cz_base, r_base, r_top, height)
            mask = (t < best_t) & (t < MAX_RANGE)
            best_t    = np.where(mask, t, best_t)
            best_refl = np.where(mask, refl, best_refl)

        # Ground (sample ~20 % of downward rays to keep ground sparse but present)
        t_g        = intersect_ground_batch(dirs, GROUND_Z)
        ground_sel = rng.random(n) < 0.20
        mask_g     = (t_g < best_t) & (t_g < 30.0) & ground_sel
        best_t     = np.where(mask_g, t_g, best_t)
        best_refl  = np.where(mask_g, 25, best_refl)

        hit = np.isfinite(best_t)
        if not np.any(hit):
            return []

        t_hit = best_t[hit, np.newaxis]
        pts   = dirs[hit] * t_hit
        pts  += rng.normal(0, NOISE_STD, pts.shape)

        # ── Coordinate-frame transform ────────────────────────────────────────
        # Internal ray-casting frame:  x = lateral (right+),  y = forward,  z = up
        # Rust / Livox HAP convention: x = forward,           y = lateral (left+), z = up
        #
        #   x_out =  pts_sim[:,1]   (forward distance)
        #   y_out = -pts_sim[:,0]   (flip: right+ → left+)
        #   z_out =  pts_sim[:,2]   (unchanged)
        x_out = pts[:, 1]
        y_out = -pts[:, 0]
        z_out = pts[:, 2]
        pts_out = np.column_stack([x_out, y_out, z_out])

        pts_mm = (pts_out * 1000).astype(np.int32)
        refls  = best_refl[hit]

        return list(zip(
            pts_mm[:, 0].tolist(),
            pts_mm[:, 1].tolist(),
            pts_mm[:, 2].tolist(),
            refls.tolist(),
        ))

    # ── main loop ─────────────────────────────────────────────────────────────
    def run(self):
        interval = 1.0 / FRAME_RATE
        print("[bridge] Running – Ctrl+C to stop")
        try:
            while True:
                t0 = time.monotonic()

                points = self.simulate_frame()
                if points:
                    self.sock.sendto(make_livox_packet(points), self.target)

                self.frame += 1
                if self.frame % FRAME_RATE == 0:
                    print(f"[bridge] frame {self.frame:5d} | "
                          f"pts: {len(points):4d}")

                wait = interval - (time.monotonic() - t0)
                if wait > 0:
                    time.sleep(wait)

        except KeyboardInterrupt:
            print("[bridge] Stopped.")


# ── entry point ───────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host",   default="127.0.0.1")
    ap.add_argument("--port",   type=int, default=57000)
    ap.add_argument("--circle", action="store_true",
                    help="Use oval/circular track instead of straight track")
    args = ap.parse_args()
    LidarSimulator(host=args.host, port=args.port, circle=args.circle).run()


if __name__ == "__main__":
    main()
