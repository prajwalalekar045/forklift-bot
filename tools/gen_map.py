#!/usr/bin/env python3
"""Write a ready-made occupancy map of the warehouse world for Nav2.

Usage: python3 tools/gen_map.py

Only static things the 0.34 m lidar plane can see are drawn: walls, crates and the rack
panels. The pallet is left out because it gets moved. To make a map from real scans
instead, use `ros2 launch forklift_nav slam.launch.py` and save it with nav2_map_server.
"""
import os

import numpy as np

RES = 0.05
X0, Y0 = -5.2, -4.2          # map origin (lower-left corner)
W_M, H_M = 10.4, 8.4
W, H = int(round(W_M / RES)), int(round(H_M / RES))

# (x_min, x_max, y_min, y_max) in world metres. Keep in sync with tools/gen_world.py.
BOXES = [
    (-5.2, 5.2, 4.0, 4.2),       # north wall
    (-5.2, 5.2, -4.2, -4.0),     # south wall
    (5.0, 5.2, -4.2, 4.2),       # east wall
    (-5.2, -5.0, -4.2, 4.2),     # west wall
    (-0.5 - 0.3, -0.5 + 0.3, 2.6 - 0.3, 2.6 + 0.3),       # crate_1
    (-1.0 - 0.3, -1.0 + 0.3, -2.6 - 0.3, -2.6 + 0.3),     # crate_2
    (-2.2 - 0.25, -2.2 + 0.25, -1.2 - 0.25, -1.2 + 0.25), # crate_3
    (3.41, 3.43, 2.0 - 0.25, 2.0 + 0.25),                 # rack panel A
    (3.41, 3.43, -2.0 - 0.25, -2.0 + 0.25),               # rack panel B
]

img = np.full((H, W), 254, dtype=np.uint8)
for x0, x1, y0, y1 in BOXES:
    c0 = max(0, int(np.floor((x0 - X0) / RES)))
    c1 = min(W, int(np.ceil((x1 - X0) / RES)))
    r1 = min(H, H - int(np.floor((y0 - Y0) / RES)))      # row 0 is the top (max y)
    r0 = max(0, H - int(np.ceil((y1 - Y0) / RES)))
    img[r0:r1, c0:c1] = 0

out_dir = os.path.join(os.path.dirname(__file__), "..", "src", "forklift_nav", "maps")
os.makedirs(out_dir, exist_ok=True)
pgm = os.path.join(out_dir, "warehouse.pgm")
with open(pgm, "wb") as f:
    f.write(f"P5\n{W} {H}\n255\n".encode())
    f.write(img.tobytes())

with open(os.path.join(out_dir, "warehouse.yaml"), "w") as f:
    f.write(f"""image: warehouse.pgm
mode: trinary
resolution: {RES}
origin: [{X0}, {Y0}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")
print(f"wrote {pgm} ({W}x{H}, {int((img == 0).sum())} occupied cells)")
