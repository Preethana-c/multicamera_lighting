"""
compute_homographies.py — rebuild config/homographies.json from the click points.

Reads:
  config/cam_points.json    pixel coordinates of each labelled point per camera
  config/floor_points.json  tile coordinates [col, row] of the same points
Writes:
  config/homographies.json  pixel -> tile homography per camera

Run from the project root:  python tools/compute_homographies.py
"""

import json
import sys
from pathlib import Path

import numpy as np
import cv2

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
TILE_SIZE = 28   # for reprojection-error reporting only

with open(CONFIG_DIR / "cam_points.json") as f:
    cam_data = json.load(f)
with open(CONFIG_DIR / "floor_points.json") as f:
    floor_data = json.load(f)

homographies = {}

for cam in sorted(cam_data.keys()):
    if cam not in floor_data:
        print(f"{cam}: no floor points — skipped")
        continue

    cam_dict = cam_data[cam]
    floor_dict = floor_data[cam]
    common = sorted(set(cam_dict.keys()) & set(floor_dict.keys()))
    cam_only = set(cam_dict) - set(floor_dict)
    floor_only = set(floor_dict) - set(cam_dict)
    print(f"\n{cam}: {len(common)} matched points", end="")
    if cam_only:
        print(f"  cam-only: {sorted(cam_only)}", end="")
    if floor_only:
        print(f"  floor-only: {sorted(floor_only)}", end="")
    print()

    if len(common) < 4:
        print("  SKIP — need at least 4 points")
        continue

    src = np.array([cam_dict[p] for p in common], dtype=np.float32)
    dst = np.array([[float(floor_dict[p][0]), float(floor_dict[p][1])] for p in common],
                   dtype=np.float32)

    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 1.5)
    if H is None:
        print("  ERROR: findHomography returned None")
        continue

    projected = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    errs = np.sqrt(((projected - dst) ** 2).sum(axis=1))
    inliers = int(mask.sum()) if mask is not None else len(common)
    print(f"  RANSAC inliers: {inliers} / {len(common)}  (low count = bad point pairs)")
    print(f"  {'point':<6}  {'cam_px':>14}  {'floor_tile':>12}  {'err_px':>8}  status")
    for i, p in enumerate(common):
        status = "OK " if mask[i] else "BAD <-- check this point"
        print(f"  {p:<6}  ({cam_dict[p][0]:4d},{cam_dict[p][1]:4d})  "
              f"tile({floor_dict[p][0]:2d},{floor_dict[p][1]:2d})  {errs[i]:8.1f}  {status}")

    inlier_errs = errs[mask.ravel() == 1]
    print(f"  Inlier error: mean={inlier_errs.mean():.2f} tiles  max={inlier_errs.max():.2f} tiles")
    homographies[cam] = H.tolist()

with open(CONFIG_DIR / "homographies.json", "w") as f:
    json.dump(homographies, f, indent=2)

print("\nhomographies.json saved — cameras:", list(homographies.keys()))
