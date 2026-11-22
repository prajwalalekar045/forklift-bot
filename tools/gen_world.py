#!/usr/bin/env python3
"""Generate the Ignition Gazebo (Fortress) warehouse world.

Usage: python3 tools/gen_world.py > src/forklift_gazebo/worlds/warehouse.world
       python3 tools/gen_world.py --render-engine ogre > ...   (if ogre2 fails on your GPU/VM)

Layout (metres, world frame, robot spawns at (-3.5, 0) facing +x):
  - 10 x 8 m room with 1 m high walls
  - one pallet (dynamic, 8 kg) in slot A at (3.0, +2.0), open towards -x; slot B at (3.0, -2.0) is empty
  - behind each slot a rack panel with an ArUco marker (ID 0 for A, ID 1 for B) on its -x face
  - three static crates as obstacles

Pallet geometry matters for the forks: the forks (top at 0.27 m) slide in under the
top plate (underside at 0.37 m) between the two side runners.
"""

import argparse

PALLETS = [
    # name, x, y  (only slot A holds a pallet at the start; slot B is empty, the pallet gets placed there)
    ("pallet_a", 3.0, 2.0),
]
RACKS = [
    # name, x, y, marker id  (x, y = pallet slot centre; the panel stands behind it)
    ("slot_a", 3.0, 2.0, 0),
    ("slot_b", 3.0, -2.0, 1),
]
CRATES = [
    ("crate_1", -0.5, 2.6, 0.6),
    ("crate_2", -1.0, -2.6, 0.6),
    ("crate_3", -2.2, -1.2, 0.5),
]

PALLET_L = 0.8      # along x
PALLET_W = 0.8      # along y
RUNNER_W = 0.08
RUNNER_H = 0.37
PLATE_T = 0.03
PANEL_X_OFFSET = PALLET_L / 2 + 0.03  # panel centre behind the pallet back face
MARKER_Z = 0.80   # same height as the camera (0.80 m) so it looks straight ahead and clears a carried pallet
MARKER_SIZE = 0.25


def box_visual_collision(name, size, pose, color, mu=None):
    sx, sy, sz = size
    px, py, pz = pose
    friction = ""
    if mu:
        friction = (f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>")
    return f"""
        <collision name="{name}_col">
          <pose>{px} {py} {pz} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          {friction}
        </collision>
        <visual name="{name}_vis">
          <pose>{px} {py} {pz} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>"""


def wall(name, x, y, sx, sy):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.5 0 0 0</pose>
      <link name="link">{box_visual_collision(name, (sx, sy, 1.0), (0, 0, 0), "0.62 0.64 0.66 1")}
      </link>
    </model>"""


def crate(name, x, y, s):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {s / 2} 0 0 0</pose>
      <link name="link">{box_visual_collision(name, (s, s, s), (0, 0, 0), "0.55 0.38 0.22 1")}
      </link>
    </model>"""


def pallet(name, x, y):
    runner_y = PALLET_W / 2 - RUNNER_W / 2
    body = ""
    body += box_visual_collision(f"{name}_runner_l", (PALLET_L, RUNNER_W, RUNNER_H), (0, runner_y, RUNNER_H / 2), "0.72 0.55 0.30 1", mu=1.5)
    body += box_visual_collision(f"{name}_runner_r", (PALLET_L, RUNNER_W, RUNNER_H), (0, -runner_y, RUNNER_H / 2), "0.72 0.55 0.30 1", mu=1.5)
    body += box_visual_collision(f"{name}_plate", (PALLET_L, PALLET_W, PLATE_T), (0, 0, RUNNER_H + PLATE_T / 2), "0.78 0.60 0.34 1", mu=1.5)
    return f"""
    <model name="{name}">
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
        <inertial>
          <pose>0 0 0.25 0 0 0</pose>
          <mass>8.0</mass>
          <inertia><ixx>0.55</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.55</iyy><iyz>0</iyz><izz>0.85</izz></inertia>
        </inertial>{body}
      </link>
    </model>"""


def rack_panel(name, px, py, marker_id):
    """Static back panel with a marker on its -x face."""
    panel_x = px + PANEL_X_OFFSET
    face_x = panel_x - 0.01
    return f"""
    <model name="{name}_rack">
      <static>true</static>
      <pose>{panel_x} {py} 0 0 0 0</pose>
      <link name="link">{box_visual_collision(name + "_panel", (0.02, 0.5, 1.0), (0, 0, 0.5), "0.35 0.38 0.42 1")}
        <visual name="marker">
          <pose>-0.0105 0 {MARKER_Z} 0 0 0</pose>
          <geometry><box><size>0.002 {MARKER_SIZE} {MARKER_SIZE}</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0 0 0 1</specular>
            <pbr>
              <metal>
                <albedo_map>model://aruco_markers/materials/textures/marker_{marker_id}.png</albedo_map>
                <roughness>1.0</roughness>
                <metalness>0.0</metalness>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--render-engine", default="ogre2", choices=["ogre2", "ogre"],
                        help="rendering engine for the sensors system (use ogre if ogre2 crashes)")
    args = parser.parse_args()

    parts = []
    parts.append(wall("wall_n", 0, 4.1, 10.4, 0.2))
    parts.append(wall("wall_s", 0, -4.1, 10.4, 0.2))
    parts.append(wall("wall_e", 5.1, 0, 0.2, 8.0))
    parts.append(wall("wall_w", -5.1, 0, 0.2, 8.0))
    for n, x, y, s in CRATES:
        parts.append(crate(n, x, y, s))
    for n, x, y in PALLETS:
        parts.append(pallet(n, x, y))
    for n, x, y, mid in RACKS:
        parts.append(rack_panel(n, x, y, mid))

    print(f"""<?xml version="1.0"?>
<!-- Generated by tools/gen_world.py. Edit the script, not this file. -->
<sdf version="1.8">
  <world name="warehouse">
    <physics name="default_physics" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>{args.render_engine}</render_engine>
    </plugin>

    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.85 0.9 1</background>
      <shadows>false</shadows>
    </scene>

    <!-- Light travels towards +x so the rack markers (on the -x face of the panels) are lit. -->
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse><specular>0 0 0 1</specular></material>
        </visual>
      </link>
    </model>
{''.join(parts)}
  </world>
</sdf>""")


if __name__ == "__main__":
    main()
