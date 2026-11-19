#!/usr/bin/env python3
"""Regenerate the ArUco (DICT_4X4_50) textures used on the rack panels.

Usage: python3 tools/gen_markers.py [--flip]

The detector also accepts mirrored markers, so --flip is normally not needed.
Use it if you want the texture itself to appear unmirrored in the Gazebo camera image
(box UV orientation differs between renderers).
Requires: pip install opencv-contrib-python-headless
"""
import argparse
import os

import cv2

OUT = os.path.join(os.path.dirname(__file__), "..", "src", "forklift_gazebo",
                   "models", "aruco_markers", "materials", "textures")

parser = argparse.ArgumentParser()
parser.add_argument("--flip", action="store_true", help="mirror the texture horizontally")
args = parser.parse_args()

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
for marker_id in (0, 1, 2):
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, 400)
    img = cv2.copyMakeBorder(img, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
    if args.flip:
        img = cv2.flip(img, 1)
    path = os.path.join(OUT, f"marker_{marker_id}.png")
    cv2.imwrite(path, img)
    print("wrote", os.path.normpath(path))
