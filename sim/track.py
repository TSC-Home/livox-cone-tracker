"""
Track definition for FastTube simulation.

Coordinate frame (internal sim frame used by bridge.py):
  x+: right   y+: forward   z+: up

Gazebo frame (used in world files):
  X+: forward  Y+: left  Z+: up
  → Gazebo X = y_sim,  Gazebo Y = -x_sim
"""

import math

LIDAR_HEIGHT     = 0.5     # m above ground
CONE_RADIUS_BASE = 0.125   # m  (keeps cluster width within detection limit)
CONE_RADIUS_TOP  = 0.018   # m
CONE_HEIGHT      = 0.325   # m

# ── Straight track ──────────────────────────────────────────────────────────
LANE_HALF    = 1.5    # m lateral offset (3 m between left and right cone)
CONE_SPACING = 1.5    # m longitudinal spacing between consecutive same-side cones
START_DIST   = 3.0    # m ahead for first cone pair
N_PAIRS      = 30     # cone pairs → 45 m total

# CONES: list of (x_sim, y_sim, side)
CONES = []
for _i in range(N_PAIRS):
    _y = START_DIST + _i * CONE_SPACING
    CONES.append((-LANE_HALF, _y, 'left'))
    CONES.append((+LANE_HALF, _y, 'right'))


# ── Oval (circular) track ────────────────────────────────────────────────────
# Elliptical oval, centre 20 m ahead of sensor.
# Outer boundary (left/right depending on side):  semi_lat=11.5, semi_fwd=18.5
# Inner boundary:                                  semi_lat= 8.5, semi_fwd=15.5
# Lane width ≈ 3 m

OVAL_CENTER_FWD = 17.0   # y_sim distance to oval centre
# With OVAL_OUTER_FWD=18.5 the near outer boundary sits at y_sim=-1.5 (behind
# sensor) and with OVAL_INNER_FWD=15.5 the near inner boundary at y_sim=+1.5
# (ahead), so the sensor origin (0,0) is already inside the track corridor.
OVAL_OUTER_LAT  = 11.5   # semi-minor (lateral) of outer ellipse
OVAL_OUTER_FWD  = 18.5   # semi-major (forward) of outer ellipse
OVAL_INNER_LAT  =  8.5
OVAL_INNER_FWD  = 15.5
N_OVAL          = 40     # cones per boundary


def generate_oval_cones():
    """Return list of (x_sim, y_sim, side) for the oval track."""
    cones = []
    for i in range(N_OVAL):
        theta = 2.0 * math.pi * i / N_OVAL
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        # Outer boundary
        x_out = OVAL_OUTER_LAT * cos_t
        y_out = OVAL_CENTER_FWD + OVAL_OUTER_FWD * sin_t
        cones.append((x_out, y_out, 'left' if x_out <= 0 else 'right'))

        # Inner boundary
        x_in = OVAL_INNER_LAT * cos_t
        y_in = OVAL_CENTER_FWD + OVAL_INNER_FWD * sin_t
        cones.append((x_in, y_in, 'left' if x_in <= 0 else 'right'))

    return cones


OVAL_CONES = generate_oval_cones()
