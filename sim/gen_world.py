#!/usr/bin/env python3
"""
Generate Gazebo Classic world files for FastTube simulation.
Usage:
    python3 sim/gen_world.py           # generates both worlds
    python3 sim/gen_world.py straight
    python3 sim/gen_world.py oval
"""

import math
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
from track import (
    CONES, OVAL_CONES,
    LANE_HALF, CONE_SPACING, START_DIST, N_PAIRS,
    OVAL_CENTER_FWD, OVAL_OUTER_LAT, OVAL_OUTER_FWD,
    OVAL_INNER_LAT, OVAL_INNER_FWD, N_OVAL,
)

# Sensor frame → Gazebo frame
def s2g(x_sim, y_sim):
    return y_sim, -x_sim   # Gazebo X=forward=y_sim, Y=left=-x_sim


# ── SDF building blocks ──────────────────────────────────────────────────────

def world_header(vehicle_x: float = 0.0, vehicle_y: float = 0.0) -> str:
    """Return the common world header SDF with the vehicle at (vehicle_x, vehicle_y)."""
    return f"""\
<?xml version="1.0"?>
<sdf version="1.6">
<world name="fasttube_track">

  <physics type="ode">
    <real_time_update_rate>1000</real_time_update_rate>
    <max_step_size>0.001</max_step_size>
  </physics>

  <light name="sun" type="directional">
    <cast_shadows>true</cast_shadows>
    <pose>0 0 20 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>-0.5 0.1 -0.9</direction>
  </light>

  <model name="ground">
    <static>true</static>
    <link name="link">
      <collision name="col">
        <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
      </collision>
      <visual name="vis">
        <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.4 0.4 0.4 1</diffuse>
        </material>
      </visual>
    </link>
  </model>

  <!-- Vehicle / LiDAR origin marker -->
  <model name="vehicle">
    <static>true</static>
    <pose>{vehicle_x:.3f} {vehicle_y:.3f} 0 0 0 1.5708</pose>
    <link name="body">
      <!-- Car body: 1.8 m long, 1.0 m wide, 0.4 m tall → top at z=0.4 -->
      <visual name="chassis">
        <pose>0 0 0.2 0 0 0</pose>
        <geometry><box><size>1.8 1.0 0.4</size></box></geometry>
        <material><ambient>0.1 0.1 0.8 1</ambient><diffuse>0.1 0.1 0.8 1</diffuse></material>
      </visual>
      <!-- LiDAR mount bracket on roof -->
      <visual name="lidar_mount">
        <pose>0 0 0.43 0 0 0</pose>
        <geometry><cylinder><radius>0.04</radius><length>0.06</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
      <!-- LiDAR sensor: sphere resting on mount, centre at z=0.5 = LIDAR_HEIGHT -->
      <visual name="lidar_sensor">
        <pose>0 0 0.5 0 0 0</pose>
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
        <material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material>
      </visual>
    </link>
  </model>

"""

WORLD_FOOTER = """\

</world>
</sdf>
"""


def cone_include(name, gz_x, gz_y):
    return (
        f"  <include>\n"
        f"    <name>{name}</name>\n"
        f"    <uri>model://cone</uri>\n"
        f"    <pose>{gz_x:.3f} {gz_y:.3f} 0 0 0 0</pose>\n"
        f"  </include>\n"
    )


# ── Straight world ────────────────────────────────────────────────────────────

def gen_straight():
    lines = [
        world_header(vehicle_x=0.0, vehicle_y=0.0),
        f"  <!-- {N_PAIRS} cone pairs | lateral ±{LANE_HALF} m | spacing {CONE_SPACING} m -->\n",
    ]
    for i in range(N_PAIRS):
        y_sim = START_DIST + i * CONE_SPACING
        gz_x, gz_y = s2g(-LANE_HALF, y_sim)
        lines.append(cone_include(f"cone_left_{i:02d}", gz_x, gz_y))
        gz_x, gz_y = s2g(+LANE_HALF, y_sim)
        lines.append(cone_include(f"cone_right_{i:02d}", gz_x, gz_y))
    lines.append(WORLD_FOOTER)
    return "".join(lines)


# ── Oval world ────────────────────────────────────────────────────────────────

def gen_oval():
    # Vehicle stays at world origin (0,0) – this is where bridge.py casts rays
    # from. OVAL_CENTER_FWD=17.0 places the near outer boundary at y_sim=-1.5
    # (behind the sensor) and near inner boundary at y_sim=+1.5 (ahead), so
    # the sensor origin is inside the track corridor.
    lines = [
        world_header(vehicle_x=0.0, vehicle_y=0.0),
        f"  <!-- Oval track | centre {OVAL_CENTER_FWD} m ahead "
        f"| outer {OVAL_OUTER_LAT}×{OVAL_OUTER_FWD} "
        f"| inner {OVAL_INNER_LAT}×{OVAL_INNER_FWD} -->\n",
    ]
    for i in range(N_OVAL):
        theta = 2.0 * math.pi * i / N_OVAL
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        x_out = OVAL_OUTER_LAT * cos_t
        y_out = OVAL_CENTER_FWD + OVAL_OUTER_FWD * sin_t
        gz_x, gz_y = s2g(x_out, y_out)
        lines.append(cone_include(f"cone_outer_{i:02d}", gz_x, gz_y))

        x_in = OVAL_INNER_LAT * cos_t
        y_in = OVAL_CENTER_FWD + OVAL_INNER_FWD * sin_t
        gz_x, gz_y = s2g(x_in, y_in)
        lines.append(cone_include(f"cone_inner_{i:02d}", gz_x, gz_y))

    lines.append(WORLD_FOOTER)
    return "".join(lines)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "both"
    out_dir = os.path.join(os.path.dirname(__file__), "worlds")
    os.makedirs(out_dir, exist_ok=True)

    if mode in ("straight", "both"):
        path = os.path.join(out_dir, "track.world")
        with open(path, "w") as f:
            f.write(gen_straight())
        print(f"Generated {path}")

    if mode in ("oval", "both"):
        path = os.path.join(out_dir, "oval.world")
        with open(path, "w") as f:
            f.write(gen_oval())
        print(f"Generated {path}")


if __name__ == "__main__":
    main()
