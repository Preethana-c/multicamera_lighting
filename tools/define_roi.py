"""
define_roi.py — draw the detection region(s) for a camera.

detection.py ONLY looks for people inside the regions drawn here. You may draw
more than one region per camera. Saved live to config/cam_roi.json, which
detection.py reloads automatically (no restart needed).

Usage (from project root):
  python tools/define_roi.py cam1            (live RTSP)
  python tools/define_roi.py cam1 shot.jpg   (from a saved image)

Keys:
  Left click = add a point      n = finish region, start a new one
  z = undo last point           c = clear this camera
  s = save and quit             q = quit without saving
"""

import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config as cfg   # noqa: E402

ROI_FILE = str(cfg.CONFIG_DIR / "cam_roi.json")
FRAME_W, FRAME_H = 640, 480

cam_name = sys.argv[1] if len(sys.argv) > 1 else "cam1"
image_path = sys.argv[2] if len(sys.argv) > 2 else None
if cam_name not in cfg.CAM_URLS:
    print("Unknown camera. Use: cam1 cam2 cam3 cam4")
    sys.exit(1)

regions = []
current = []

all_roi = json.load(open(ROI_FILE)) if os.path.exists(ROI_FILE) else {}
if cam_name in all_roi:
    regions = [list(map(list, poly)) for poly in all_roi[cam_name]]
    print(f"Loaded {len(regions)} existing region(s) for {cam_name}")

COLORS = [(0, 255, 0), (255, 180, 0), (0, 165, 255), (255, 0, 255)]


def on_mouse(event, x, y, flags, _):
    if event == cv2.EVENT_LBUTTONDOWN:
        current.append([x, y])
        print(f"  point ({x},{y}) — region {len(regions) + 1}, {len(current)} pts")


os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
static = cv2.imread(image_path) if (image_path and os.path.exists(image_path)) else None
cap = None
if static is None:
    cap = cv2.VideoCapture(cfg.CAM_URLS[cam_name], cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

cv2.namedWindow("roi")
cv2.setMouseCallback("roi", on_mouse)
print(f"\n=== Drawing detection region(s) for {cam_name} ===")
print("Click around each area to enclose it. 'n'=new region, 's'=save.\n")

frame = static
while True:
    if cap:
        ret, f = cap.read()
        if ret:
            frame = f
    if frame is None:
        continue
    disp = cv2.resize(frame, (FRAME_W, FRAME_H)).copy()

    for i, poly in enumerate(regions):
        if len(poly) >= 2:
            pts = np.array(poly, np.int32)
            cv2.polylines(disp, [pts], True, COLORS[i % len(COLORS)], 2)
            ov = disp.copy()
            cv2.fillPoly(ov, [pts], COLORS[i % len(COLORS)])
            cv2.addWeighted(ov, 0.18, disp, 0.82, 0, disp)

    col = COLORS[len(regions) % len(COLORS)]
    for p in current:
        cv2.circle(disp, tuple(p), 4, col, -1)
    if len(current) >= 2:
        cv2.polylines(disp, [np.array(current, np.int32)], False, col, 2)

    cv2.putText(disp, f"{cam_name}  regions:{len(regions)}  pts:{len(current)}"
                "   n=new z=undo c=clear s=save q=quit",
                (8, FRAME_H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 180), 1)
    cv2.imshow("roi", disp)

    key = cv2.waitKey(30 if cap else 50) & 0xFF
    if key == ord("q"):
        print("Quit — not saved.")
        break
    elif key == ord("n"):
        if len(current) >= 3:
            regions.append(current)
            current = []
            print(f"  Region {len(regions)} closed. Start the next one.")
        else:
            print("  Need at least 3 points before starting a new region.")
    elif key == ord("z"):
        if current:
            current.pop()
        elif regions:
            current = regions.pop()
    elif key == ord("c"):
        regions = []
        current = []
        print("  Cleared.")
    elif key == ord("s"):
        if len(current) >= 3:
            regions.append(current)
            current = []
        all_roi[cam_name] = regions
        with open(ROI_FILE, "w") as fp:
            json.dump(all_roi, fp, indent=2)
        print(f"\nSaved {len(regions)} region(s) for {cam_name} → {ROI_FILE}")
        break

if cap:
    cap.release()
cv2.destroyAllWindows()
