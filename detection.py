"""
detection.py — zone-aware multi-camera person detection + smart light control.

Reads four RTSP cameras, runs YOLOv8 + ByteTrack person detection, maps each
person's foot position to a floor tile via a per-camera homography, and turns
the nearest lights ON. Light state is published over MQTT for the web UI and,
when enabled, forwarded to the Bluetooth-mesh gateway that drives the
physical lights.

All connection settings come from the .env file via config.py. Detection
tuning, the floor grid, camera zones and the light map are defined below.

Run:    python detection.py
Keys:   1/2/3/4 = single camera view   0 = 2x2 grid   q = quit
        Tab = cycle tuned camera   c = cycle corner
        [ / ] = base col   - / = = base row   u/i = corner row   ,/. = corner col
        s = save offsets
"""

import json
import threading
import time
import os

import cv2
import numpy as np
import paho.mqtt.client as mqtt
from ultralytics import YOLO
from http.server import BaseHTTPRequestHandler, HTTPServer

import config as cfg

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

CAM_NAMES = cfg.CAM_NAMES
CAM_URLS = cfg.CAM_URLS

# ── Config file paths ─────────────────────────────────────────────────────────
OFFSET_FILE = str(cfg.CONFIG_DIR / "cam_offsets.json")
ROI_FILE = str(cfg.CONFIG_DIR / "cam_roi.json")
HOMOGRAPHY_FILE = str(cfg.CONFIG_DIR / "homographies.json")
CAM_POINTS_FILE = str(cfg.CONFIG_DIR / "cam_points.json")
FLOOR_POINTS_FILE = str(cfg.CONFIG_DIR / "floor_points.json")
LIGHTS_FILE = str(cfg.CONFIG_DIR / "lights.json")

# ── Floor / light grid ────────────────────────────────────────────────────────
LIGHT_ROWS = [3, 7, 11, 15, 19, 23, 27]
LIGHT_COLS = [2, 7, 11, 15, 19]
FLOOR_COLS = 22
FLOOR_ROWS = 28

FRAME_W = 640
FRAME_H = 480

# ── Detection tuning ──────────────────────────────────────────────────────────
YOLO_EVERY = 2           # run YOLO every Nth frame
YOLO_INPUT_W = 960       # higher res from native stream → detects far/occluded people
YOLO_INPUT_H = 544
YOLO_CONF = 0.12         # low — catches occluded/seated people; ByteTrack filters flickers
DETECTION_HOLD = 8.0     # s — keep a person's box alive after YOLO misses them
LIGHT_HOLD = 30.0        # s — keep a light ON after the last confirmed detection
LIGHT_RADIUS = 3         # tiles — lights within this radius of a person turn on
MATCH_THRESH = 450       # px — fallback distance match when ByteTrack id is None

# ── Per-camera floor tile zones ───────────────────────────────────────────────
# Each camera only activates lights for detections inside its zone (plus its ROI
# polygon). Overlapping zones are fine — cross-camera duplicates are merged.
# Format: (col_min, col_max, row_min, row_max)
CAM_ZONES = {
    "cam1": (0, 21, 0, 27),   # full floor — ROI polygon is the real gate
    "cam2": (0, 21, 0, 27),   # full floor
    "cam3": (0, 16, 0, 13),
    "cam4": (0, 18, 8, 24),
}

# ── Per-camera fine-tune offsets (base shift + 4-corner bilinear correction) ──
# tl/tr/bl/br = [row_delta, col_delta]; interpolated across the frame at runtime.
# Adjusted live with the tuning keys and persisted to cam_offsets.json.
def _corner():
    return {"col": 0, "row": 0, "tl": [0, 0], "tr": [0, 0], "bl": [0, 0], "br": [0, 0]}

CAM_OFFSET = {
    "cam1": {**_corner(), "row": 3},
    "cam2": _corner(),
    "cam3": _corner(),
    "cam4": _corner(),
}
if os.path.exists(OFFSET_FILE):
    try:
        with open(OFFSET_FILE) as _f:
            _saved = json.load(_f)
        for _c, _v in _saved.items():
            if _c in CAM_OFFSET:
                CAM_OFFSET[_c].update(_v)
        print(f"Loaded offsets: {OFFSET_FILE}")
    except Exception as _e:
        print(f"Could not load {OFFSET_FILE}: {_e}")


def save_offsets():
    with open(OFFSET_FILE, "w") as f:
        json.dump(CAM_OFFSET, f, indent=2)
    print("Offsets saved:")
    for c, v in CAM_OFFSET.items():
        print(f"  {c}: base row={v['row']:+d} col={v['col']:+d} | "
              f"TL{v['tl']} TR{v['tr']} BL{v['bl']} BR{v['br']}")

TUNE_CAM_IDX = 0
TUNE_CORNER = "tl"
CORNERS = ["tl", "tr", "bl", "br"]

SHOW_CAM = "cam1"   # None = 2x2 grid

# ── Person colours (BGR for OpenCV; hex derived so feed and map always match) ─
_COLORS_BGR = [
    (0, 165, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255),
    (180, 20, 255), (255, 180, 0), (50, 205, 50), (100, 50, 255),
    (0, 200, 160), (200, 80, 255), (0, 128, 255), (128, 255, 0),
]
PERSON_COLORS_BGR = _COLORS_BGR
PERSON_COLORS_HEX = [f"#{r:02X}{g:02X}{b:02X}" for (b, g, r) in _COLORS_BGR]

# ── Load homographies (pixel → tile index, direct — no scaling needed) ────────
with open(HOMOGRAPHY_FILE) as f:
    homographies = {k: np.array(v) for k, v in json.load(f).items()}

# ── Detection regions (ROI) — drawn with tools/define_roi.py ──────────────────
# Detection only keeps people whose foot point falls inside one of these polygons.
# No entry for a camera → whole frame allowed. Auto-reloads when the file changes.
def load_roi():
    roi = {}
    if not os.path.exists(ROI_FILE):
        return roi
    try:
        raw = json.load(open(ROI_FILE))
    except Exception:
        return None   # file mid-write — skip this reload, retry next loop
    for cam, polys in raw.items():
        roi[cam] = [np.array(p, np.int32) for p in polys if len(p) >= 3]
    return roi

CAM_ROI = load_roi() or {}
_roi_mtime = os.path.getmtime(ROI_FILE) if os.path.exists(ROI_FILE) else 0
print(f"Loaded ROI regions: { {c: len(v) for c, v in CAM_ROI.items()} }")


def in_roi(cam_name, px, py):
    polys = CAM_ROI.get(cam_name)
    if not polys:
        return True
    for poly in polys:
        if cv2.pointPolygonTest(poly, (float(px), float(py)), False) >= 0:
            return True
    return False


# ── Per-pixel offset correction (base shift + 4-corner bilinear) ──────────────
def offset_delta(cam_name, px, py):
    off = CAM_OFFSET[cam_name]
    xf = max(0.0, min(1.0, px / FRAME_W))
    yf = max(0.0, min(1.0, py / FRAME_H))
    tl, tr = off.get("tl", [0, 0]), off.get("tr", [0, 0])
    bl, br = off.get("bl", [0, 0]), off.get("br", [0, 0])
    drow = (tl[0] * (1 - xf) + tr[0] * xf) * (1 - yf) + (bl[0] * (1 - xf) + br[0] * xf) * yf
    dcol = (tl[1] * (1 - xf) + tr[1] * xf) * (1 - yf) + (bl[1] * (1 - xf) + br[1] * xf) * yf
    return off["col"] + dcol, off["row"] + drow


# ── Calibration points (browser drag-to-correct adds new ones live) ───────────
def _loadjson(p):
    return json.load(open(p)) if os.path.exists(p) else {}

cam_points_all = _loadjson(CAM_POINTS_FILE)
floor_points_all = _loadjson(FLOOR_POINTS_FILE)
calib_lock = threading.Lock()


def recompute_homography(cam_name):
    cp = cam_points_all.get(cam_name, {})
    fp = floor_points_all.get(cam_name, {})
    common = sorted(set(cp) & set(fp))
    # Only rebuild from browser "drag" points (same 640x480 space the feed uses),
    # and only once 4+ exist — so a few stray drags can't collapse a camera.
    drag_pts = [p for p in common if p.startswith("drag")]
    if len(drag_pts) < 4:
        print(f"[calib] {cam_name}: {len(drag_pts)} drag point(s) — need 4+ to rebuild "
              f"(existing calibration kept)")
        return False
    common = drag_pts
    src = np.array([cp[p] for p in common], dtype=np.float32)
    dst = np.array([[float(fp[p][0]), float(fp[p][1])] for p in common], dtype=np.float32)
    H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 1.5)
    if H is None:
        print(f"[calib] {cam_name}: findHomography failed")
        return False
    homographies[cam_name] = H
    json.dump(cam_points_all, open(CAM_POINTS_FILE, "w"), indent=2)
    json.dump(floor_points_all, open(FLOOR_POINTS_FILE, "w"), indent=2)
    homs = _loadjson(HOMOGRAPHY_FILE)
    homs[cam_name] = H.tolist()
    json.dump(homs, open(HOMOGRAPHY_FILE, "w"), indent=2)
    print(f"[calib] {cam_name}: recomputed with {len(common)} points → live")
    return True


# ── MQTT (local broker: UI + calibration + master override) ───────────────────
def on_mqtt_message(client, userdata, msg):
    global real_override
    topic = msg.topic
    if topic == "control/lights":
        cmd = msg.payload.decode(errors="ignore").strip().upper()
        if cmd == "ALL_ON":
            real_override = True
        elif cmd == "ALL_OFF":
            real_override = False
        else:
            real_override = None      # back to automatic
        real_light_on.clear()         # force the loop to re-send every light
        print(f"[real] master override -> {cmd or 'AUTO'}")
        return

    try:
        data = json.loads(msg.payload.decode())
    except Exception:
        return
    with calib_lock:
        if topic == "calibration/add":
            cam = data.get("cam")
            if cam not in CAM_NAMES:
                return
            fx, fy = float(data["foot_x"]), float(data["foot_y"])
            dcol, drow = offset_delta(cam, fx, fy)   # subtract offset runtime re-adds
            tgt_col = float(data["tile_col"]) - dcol
            tgt_row = float(data["tile_row"]) - drow
            cam_points_all.setdefault(cam, {})
            floor_points_all.setdefault(cam, {})
            n = 1
            while f"drag{n}" in cam_points_all[cam]:
                n += 1
            lbl = f"drag{n}"
            cam_points_all[cam][lbl] = [fx, fy]
            floor_points_all[cam][lbl] = [tgt_col, tgt_row]
            print(f"[calib] +{cam}.{lbl}: foot({fx:.0f},{fy:.0f}) -> "
                  f"tile({data['tile_col']},{data['tile_row']})")
            recompute_homography(cam)
        elif topic == "calibration/undo":
            cam = data.get("cam")
            if cam not in CAM_NAMES:
                return
            drags = [k for k in cam_points_all.get(cam, {}) if k.startswith("drag")]
            if not drags:
                print(f"[calib] {cam}: no drag points to undo")
                return
            last = sorted(drags, key=lambda k: int(k[4:]))[-1]
            cam_points_all[cam].pop(last, None)
            floor_points_all[cam].pop(last, None)
            print(f"[calib] undo {cam}.{last}")
            recompute_homography(cam)


mqttclient = mqtt.Client()
mqttclient.on_message = on_mqtt_message
mqttclient.connect(cfg.MQTT_HOST, cfg.MQTT_PORT)
mqttclient.subscribe("calibration/add")
mqttclient.subscribe("calibration/undo")
mqttclient.subscribe("control/lights")
mqttclient.loop_start()

# ── REAL LIGHTS (Bluetooth-mesh gateway) ──────────────────────────────────────
# REAL_LIGHTS_ENABLED=false → simulation only (physical lights untouched).
REAL_LIGHTS_ENABLED = cfg.REAL_LIGHTS_ENABLED
REAL_CAMS = set(CAM_NAMES)   # every camera drives real lights → real matches the UI map

# UI light number -> mesh element address (loaded from config/lights.json).
# Numbers sharing an address are the same physical fixture; lights with no
# address (e.g. 18) are omitted and left permanently ON manually.
_lights_raw = _loadjson(LIGHTS_FILE)
LIGHT_NUM_ELEM = {int(k): int(v) for k, v in _lights_raw.items() if k.isdigit()}

# UI number -> (row, col) grid key used by the light state machine
LIGHT_ELEM = {((n - 1) // len(LIGHT_COLS), (n - 1) % len(LIGHT_COLS)): e
              for n, e in LIGHT_NUM_ELEM.items()}

# Lights sharing one mesh element are the SAME fixture and switch together:
# LIGHT_SIBLINGS[key] = all grid keys on that element.
_elem_keys = {}
for _k, _e in LIGHT_ELEM.items():
    _elem_keys.setdefault(_e, []).append(_k)
LIGHT_SIBLINGS = {k: _elem_keys[e] for k, e in LIGHT_ELEM.items()}

realmqtt = None
if REAL_LIGHTS_ENABLED:
    realmqtt = mqtt.Client()
    realmqtt.username_pw_set(cfg.REAL_USER, cfg.REAL_PWD)
    try:
        realmqtt.connect(cfg.REAL_BROKER, cfg.REAL_PORT, 60)
        realmqtt.loop_start()
        print(f"Real lights ENABLED via {cfg.REAL_BROKER} — {len(LIGHT_ELEM)} lights mapped")
    except Exception as e:
        print(f"Real-light broker connect FAILED ({e}) — running sim-only")
        realmqtt = None

real_override = None   # None = automatic; True = force all ON; False = force all OFF

_real_seq = 18000


def send_real_light(elem, on):
    """Publish one on/off command to the mesh gateway (16-bit rolling seq)."""
    global _real_seq
    if realmqtt is None:
        return
    _real_seq = 18000 if _real_seq > 64000 else _real_seq + 1
    payload = {"MsgSeqNo": _real_seq, "PacketId": 4, "ElementAddr": elem,
               "ModelId": 4096, "State": [{"OnOff": 1 if on else 0}]}
    realmqtt.publish(cfg.REAL_CMD_TOPIC, json.dumps(payload), qos=1)


# ── YOLO — one model per camera so each has its own ByteTrack state ───────────
print("Loading YOLO models …")
models = {cam: YOLO("yolov8s.pt") for cam in CAM_NAMES}
print("YOLO ready.")

# CLAHE — improves detection in dim/uneven office lighting
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

# ── Camera reader threads ─────────────────────────────────────────────────────
latest_frames = {cam: None for cam in CAM_NAMES}
frame_locks = {cam: threading.Lock() for cam in CAM_NAMES}

# ── MJPEG server (serves annotated feeds to the web UI) ───────────────────────
latest_jpegs = {cam: None for cam in CAM_NAMES}
jpeg_locks = {cam: threading.Lock() for cam in CAM_NAMES}


class _MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        cam = self.path.strip("/").split("/")[-1]
        if cam not in CAM_NAMES:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        try:
            while True:
                with jpeg_locks[cam]:
                    data = latest_jpegs.get(cam)
                if data:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n")
                time.sleep(0.05)
        except Exception:
            pass

    def log_message(self, *a):
        pass


threading.Thread(
    target=HTTPServer(("0.0.0.0", cfg.MJPEG_PORT), _MJPEGHandler).serve_forever,
    daemon=True).start()


def camera_reader(cam_name, url):
    """Continuously read one RTSP camera, auto-reconnecting on drop."""
    while True:
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        fails = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                fails += 1
                if fails > 20:
                    with frame_locks[cam_name]:
                        latest_frames[cam_name] = None
                    break
                time.sleep(0.05)
                continue
            fails = 0
            with frame_locks[cam_name]:
                latest_frames[cam_name] = frame
        cap.release()
        time.sleep(2.0)


for cam in CAM_NAMES:
    threading.Thread(target=camera_reader, args=(cam, CAM_URLS[cam]), daemon=True).start()

# ── Runtime state ─────────────────────────────────────────────────────────────
held_detections = {cam: [] for cam in CAM_NAMES}
last_seen_light = {}          # (row,col) -> last time someone was near (UI lights)
light_on = {}                 # (row,col) -> current UI on/off, to publish on change
last_seen_light_real = {}     # (row,col) -> last time, for the real lights
real_light_on = {}            # element -> current real on/off, to send on change
person_color_map = {}
next_color_slot = 0
next_person_id = 0
frame_count = {cam: 0 for cam in CAM_NAMES}
last_resync_time = 0.0


def set_all_lights(on):
    """Force EVERY light (UI + real) ON/OFF at once — used at startup & shutdown."""
    state = "ON" if on else "OFF"
    for ri in range(len(LIGHT_ROWS)):
        for ci in range(len(LIGHT_COLS)):
            mqttclient.publish(f"lights/{ri}/{ci}", state)
            light_on[(ri, ci)] = on
    for elem in set(LIGHT_ELEM.values()):
        send_real_light(elem, on)
        real_light_on[elem] = on


# ── Helpers ───────────────────────────────────────────────────────────────────
def pixel_to_tile(cam_name, px, py):
    H = homographies.get(cam_name)
    if H is None:
        return -1, -1
    r = cv2.perspectiveTransform(np.array([[[float(px), float(py)]]], np.float32), H)
    return int(r[0][0][0]), int(r[0][0][1])


def in_zone(cam_name, col, row):
    c0, c1, r0, r1 = CAM_ZONES[cam_name]
    return c0 <= col <= c1 and r0 <= row <= r1


def active_lights_for(tile_col, tile_row):
    rsq = LIGHT_RADIUS ** 2
    hits = [(ri, ci)
            for ri, lr in enumerate(LIGHT_ROWS)
            for ci, lc in enumerate(LIGHT_COLS)
            if (tile_row - lr) ** 2 + (tile_col - lc) ** 2 <= rsq]
    if not hits:
        hits = [min(
            [(ri, ci) for ri, _ in enumerate(LIGHT_ROWS) for ci, _ in enumerate(LIGHT_COLS)],
            key=lambda rc: (tile_row - LIGHT_ROWS[rc[0]]) ** 2 + (tile_col - LIGHT_COLS[rc[1]]) ** 2
        )]
    return hits


def _zone_area(cam):
    z = CAM_ZONES.get(cam)
    if not z:
        return FLOOR_COLS * FLOOR_ROWS
    c0, c1, r0, r1 = z
    return (c1 - c0) * (r1 - r0)


def deduplicate(dots):
    """Merge the same person seen by two cameras (within 2 tiles). The camera
    whose zone most specifically owns the spot (smallest zone) keeps the dot."""
    dots = sorted(dots, key=lambda d: _zone_area(d["cam"]))
    merged, used = [], set()
    for i, a in enumerate(dots):
        if i in used:
            continue
        for j in range(i + 1, len(dots)):
            if j in used or dots[j]["cam"] == a["cam"]:
                continue
            if abs(a["tile_col"] - dots[j]["tile_col"]) <= 2 and \
               abs(a["tile_row"] - dots[j]["tile_row"]) <= 2:
                used.add(j)
        merged.append(a)
    return merged


def draw_box(frame, x1, y1, x2, y2, color):
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 3)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
    head_bot = y1 + (y2 - y1) // 3
    cv2.rectangle(frame, (x1, y1), (x2, head_bot), color, 2)


def put_label(frame, text, x, y, color):
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), bl = cv2.getTextSize(text, font, 0.48, 1)
    cv2.rectangle(frame, (x - 1, y - th - 6), (x + tw + 3, y + bl - 2), (0, 0, 0), -1)
    cv2.putText(frame, text, (x + 1, y - 2), font, 0.48, color, 1)


def draw_cam_label(frame, text):
    font = cv2.FONT_HERSHEY_DUPLEX
    (tw, th), _ = cv2.getTextSize(text, font, 1.1, 2)
    cv2.rectangle(frame, (6, 6), (tw + 22, th + 22), (0, 0, 0), -1)
    cv2.putText(frame, text, (14, th + 14), font, 1.1, (255, 255, 255), 2)


# ── Start with EVERY light ON, then let detection turn them off as it warms up ─
set_all_lights(True)
_now0 = time.time()
for _ri in range(len(LIGHT_ROWS)):
    for _ci in range(len(LIGHT_COLS)):
        last_seen_light[(_ri, _ci)] = _now0
        last_seen_light_real[(_ri, _ci)] = _now0
print(f"All {len(LIGHT_ROWS) * len(LIGHT_COLS)} lights ON — following detection after {LIGHT_HOLD:.0f}s")

# ── Main loop ─────────────────────────────────────────────────────────────────
print("detection.py running — keys: 1/2/3/4=cam  0=grid  q=quit  Tab/[/]/-/= tune")

while True:
    now = time.time()
    all_dots = []

    # live-reload ROI if define_roi.py just saved a new boundary
    if os.path.exists(ROI_FILE):
        _m = os.path.getmtime(ROI_FILE)
        if _m != _roi_mtime:
            _new = load_roi()
            if _new is not None:
                CAM_ROI = _new
                _roi_mtime = _m
                print(f"[roi] reloaded — { {c: len(v) for c, v in CAM_ROI.items()} }")

    for cam_name in CAM_NAMES:
        with frame_locks[cam_name]:
            raw = latest_frames[cam_name]

        # offline placeholder
        if raw is None:
            blank = np.zeros((FRAME_H, FRAME_W, 3), np.uint8)
            draw_cam_label(blank, cam_name.upper())
            cv2.putText(blank, "OFFLINE", (FRAME_W // 2 - 60, FRAME_H // 2),
                        cv2.FONT_HERSHEY_DUPLEX, 1.2, (60, 60, 60), 2)
            _, buf = cv2.imencode(".jpg", blank, [cv2.IMWRITE_JPEG_QUALITY, 60])
            with jpeg_locks[cam_name]:
                latest_jpegs[cam_name] = buf.tobytes()
            continue

        frame = cv2.resize(raw, (FRAME_W, FRAME_H))
        frame_count[cam_name] += 1
        run_yolo = (frame_count[cam_name] % YOLO_EVERY) == 0
        has_H = cam_name in homographies

        if run_yolo and has_H:
            # detect from the NATIVE frame (high-res) so far/occluded people survive
            small = cv2.resize(raw, (YOLO_INPUT_W, YOLO_INPUT_H))
            lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB)
            lab[:, :, 0] = clahe.apply(lab[:, :, 0])
            small = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
            results = models[cam_name].track(
                small, classes=[0], conf=YOLO_CONF, iou=0.45,
                persist=True, verbose=False,
            )
            sx = FRAME_W / YOLO_INPUT_W
            sy = FRAME_H / YOLO_INPUT_H

            cam_idx = CAM_NAMES.index(cam_name)
            held_by_id = {d["person_id"]: d for d in held_detections[cam_name]}
            seen_ids = set()
            updated = []

            for box in results[0].boxes:
                bx1, by1, bx2, by2 = (int(v) for v in box.xyxy[0])
                x1 = int(bx1 * sx); y1 = int(by1 * sy)
                x2 = int(bx2 * sx); y2 = int(by2 * sy)
                bw = x2 - x1
                bh = y2 - y1
                cx = (x1 + x2) // 2

                is_sitting = bh < bw * 1.8
                head_x = cx
                head_y = y1 + bh // 8
                foot_x = cx           # floor mapping uses the foot point
                foot_y = y2

                # ROI gate — test body centre so seated people (feet hidden) still count
                if not in_roi(cam_name, cx, (y1 + y2) // 2):
                    continue

                # resolve person_id
                if box.id is not None:
                    person_id = cam_idx * 1000 + int(box.id[0])
                else:
                    best_held, best_d = None, MATCH_THRESH
                    for h in held_detections[cam_name]:
                        d = abs(head_x - h["head_x"]) + abs(head_y - h["head_y"])
                        if d < best_d:
                            best_d = d
                            best_held = h
                    if best_held is not None:
                        person_id = best_held["person_id"]
                    else:
                        person_id = cam_idx * 1000 + next_person_id
                        next_person_id += 1

                seen_ids.add(person_id)

                # EMA on box coords for a smooth bounding box
                if person_id in held_by_id:
                    prev = held_by_id[person_id]
                    a = 0.5
                    x1 = int(a * x1 + (1 - a) * prev["x1"])
                    y1 = int(a * y1 + (1 - a) * prev["y1"])
                    x2 = int(a * x2 + (1 - a) * prev["x2"])
                    y2 = int(a * y2 + (1 - a) * prev["y2"])
                    head_x = int(a * head_x + (1 - a) * prev["head_x"])
                    head_y = int(a * head_y + (1 - a) * prev["head_y"])
                    foot_x = int(a * foot_x + (1 - a) * prev.get("foot_x", foot_x))
                    foot_y = int(a * foot_y + (1 - a) * prev.get("foot_y", foot_y))

                # sticky tile — foot position + offset correction, slow EMA + hysteresis
                raw_col, raw_row = pixel_to_tile(cam_name, foot_x, foot_y)
                dcol, drow = offset_delta(cam_name, foot_x, foot_y)
                raw_col += dcol
                raw_row += drow

                if person_id in held_by_id:
                    prev = held_by_id[person_id]
                    a_t = 0.12
                    tcf = a_t * raw_col + (1 - a_t) * prev.get("tile_col_f", float(raw_col))
                    trf = a_t * raw_row + (1 - a_t) * prev.get("tile_row_f", float(raw_row))
                    tile_col = round(tcf) if abs(tcf - prev["tile_col"]) > 1.0 else prev["tile_col"]
                    tile_row = round(trf) if abs(trf - prev["tile_row"]) > 1.0 else prev["tile_row"]
                else:
                    tcf = float(raw_col)
                    trf = float(raw_row)
                    tile_col = round(raw_col)
                    tile_row = round(raw_row)

                tile_col = max(0, min(tile_col, FLOOR_COLS - 1))
                tile_row = max(0, min(tile_row, FLOOR_ROWS - 1))

                zone_ok = in_zone(cam_name, tile_col, tile_row)
                lights = active_lights_for(tile_col, tile_row) if zone_ok else []

                updated.append({
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                    "head_x": head_x, "head_y": head_y,
                    "foot_x": foot_x, "foot_y": foot_y,
                    "tile_col": tile_col, "tile_row": tile_row,
                    "tile_col_f": tcf, "tile_row_f": trf,
                    "active_lights": lights,
                    "sitting": is_sitting,
                    "in_zone": zone_ok,
                    "last_seen": now,
                    "person_id": person_id,
                })

            # keep held detections not seen this frame until DETECTION_HOLD expires
            for d in held_detections[cam_name]:
                if d["person_id"] not in seen_ids and now - d["last_seen"] < DETECTION_HOLD:
                    updated.append(d)

            held_detections[cam_name] = updated
        else:
            held_detections[cam_name] = [
                d for d in held_detections[cam_name]
                if now - d["last_seen"] < DETECTION_HOLD
            ]

        # update light timestamps
        for d in held_detections[cam_name]:
            if in_zone(cam_name, d["tile_col"], d["tile_row"]):
                for (ri, ci) in d.get("active_lights", []):
                    last_seen_light[(ri, ci)] = now
                    if cam_name in REAL_CAMS:
                        last_seen_light_real[(ri, ci)] = now

        # draw ROI outline
        for poly in CAM_ROI.get(cam_name, []):
            cv2.polylines(frame, [poly], True, (0, 255, 180), 1)

        # draw detections
        for d in held_detections[cam_name]:
            pid = d.get("person_id", 0)
            if pid not in person_color_map:
                person_color_map[pid] = next_color_slot % len(PERSON_COLORS_BGR)
                next_color_slot += 1
            color = PERSON_COLORS_BGR[person_color_map[pid]]
            chex = PERSON_COLORS_HEX[person_color_map[pid]]

            x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]
            draw_box(frame, x1, y1, x2, y2, color)
            cv2.circle(frame, (d["head_x"], d["head_y"]), 5, (0, 0, 0), -1)
            cv2.circle(frame, (d["head_x"], d["head_y"]), 3, color, -1)

            posture = "sit" if d.get("sitting") else "std"
            cur_zone = in_zone(cam_name, d["tile_col"], d["tile_row"])
            zone_tag = "" if cur_zone else " OZ"
            local_id = d["person_id"] % 1000
            put_label(frame, f"P{local_id} {posture}{zone_tag}", x1, y1, color)

            if cur_zone:
                all_dots.append({
                    "tile_col": d["tile_col"],
                    "tile_row": d["tile_row"],
                    "cam": cam_name,
                    "color": chex,
                    "sitting": d.get("sitting", False),
                    "id": local_id,
                    "foot_x": d.get("foot_x", d["head_x"]),
                    "foot_y": d.get("foot_y", d["head_y"]),
                })

        draw_cam_label(frame, cam_name.upper())

        # offset HUD for the camera being tuned
        if CAM_NAMES[TUNE_CAM_IDX] == cam_name:
            o = CAM_OFFSET[cam_name]
            hud = (f"[{TUNE_CORNER.upper()}] row:{o.get(TUNE_CORNER, [0, 0])[0]:+d} "
                   f"col:{o.get(TUNE_CORNER, [0, 0])[1]:+d}  "
                   f"TL{o['tl']} TR{o['tr']} BL{o['bl']} BR{o['br']}  "
                   f"base row:{o['row']:+d} col:{o['col']:+d}  "
                   f"c=corner u/i=row ,/.=col -/==base s=save")
            cv2.putText(frame, hud, (10, FRAME_H - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 180), 1)

        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        with jpeg_locks[cam_name]:
            latest_jpegs[cam_name] = buf.tobytes()

    # publish person positions for the UI
    mqttclient.publish("persons/positions", json.dumps(deduplicate(all_dots)))

    # UI light states (force-resync every 5 s)
    force = (now - last_resync_time) >= 5.0
    if force:
        last_resync_time = now
    for ri in range(len(LIGHT_ROWS)):
        for ci in range(len(LIGHT_COLS)):
            key = (ri, ci)
            # ON if this light OR any sibling on the same element has someone nearby
            on = any((now - last_seen_light.get(sib, 0)) < LIGHT_HOLD
                     for sib in LIGHT_SIBLINGS.get(key, [key]))
            if light_on.get(key) != on or force:
                mqttclient.publish(f"lights/{ri}/{ci}", "ON" if on else "OFF")
                light_on[key] = on

    # REAL lights — send a gateway command on state change only.
    # Lights sharing an element are ON if ANY of them has someone (OR), OFF only
    # when ALL are clear — otherwise shared lights would fight each other.
    elem_on = {}
    for key, elem in LIGHT_ELEM.items():
        if real_override is True:
            o = True
        elif real_override is False:
            o = False
        else:
            o = (now - last_seen_light_real.get(key, 0)) < LIGHT_HOLD
        elem_on[elem] = elem_on.get(elem, False) or o
    for elem, on in elem_on.items():
        if real_light_on.get(elem) != on:
            send_real_light(elem, on)
            real_light_on[elem] = on
            print(f"[real] elem{elem} -> {'ON' if on else 'OFF'}")

    # display
    if SHOW_CAM is not None and SHOW_CAM in CAM_NAMES:
        with jpeg_locks[SHOW_CAM]:
            data = latest_jpegs.get(SHOW_CAM)
        if data:
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                cv2.imshow("detection", img)
    else:
        cells = []
        for cam in CAM_NAMES:
            with jpeg_locks[cam]:
                data = latest_jpegs.get(cam)
            img = (cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                   if data else None)
            if img is None:
                img = np.zeros((FRAME_H // 2, FRAME_W // 2, 3), np.uint8)
            cells.append(cv2.resize(img, (FRAME_W // 2, FRAME_H // 2)))
        cv2.imshow("detection", np.vstack([np.hstack(cells[:2]), np.hstack(cells[2:])]))

    # keys
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("1"):
        SHOW_CAM = "cam1"
    elif key == ord("2"):
        SHOW_CAM = "cam2"
    elif key == ord("3"):
        SHOW_CAM = "cam3"
    elif key == ord("4"):
        SHOW_CAM = "cam4"
    elif key == ord("0"):
        SHOW_CAM = None
    elif key == 9:   # Tab
        TUNE_CAM_IDX = (TUNE_CAM_IDX + 1) % len(CAM_NAMES)
        c = CAM_NAMES[TUNE_CAM_IDX]
        print(f"Tuning: {c}  offset={CAM_OFFSET[c]}")
    elif key == ord("["):
        c = CAM_NAMES[TUNE_CAM_IDX]
        CAM_OFFSET[c]["col"] -= 1
        print(f"{c} col={CAM_OFFSET[c]['col']}")
    elif key == ord("]"):
        c = CAM_NAMES[TUNE_CAM_IDX]
        CAM_OFFSET[c]["col"] += 1
        print(f"{c} col={CAM_OFFSET[c]['col']}")
    elif key == ord("-"):
        c = CAM_NAMES[TUNE_CAM_IDX]
        CAM_OFFSET[c]["row"] -= 1
        print(f"{c} row={CAM_OFFSET[c]['row']}")
    elif key == ord("="):
        c = CAM_NAMES[TUNE_CAM_IDX]
        CAM_OFFSET[c]["row"] += 1
        print(f"{c} row={CAM_OFFSET[c]['row']}")
    elif key == ord("c"):
        TUNE_CORNER = CORNERS[(CORNERS.index(TUNE_CORNER) + 1) % len(CORNERS)]
        c = CAM_NAMES[TUNE_CAM_IDX]
        print(f"Corner → {TUNE_CORNER.upper()}  current={CAM_OFFSET[c][TUNE_CORNER]}")
    elif key == ord("u"):
        c = CAM_NAMES[TUNE_CAM_IDX]
        CAM_OFFSET[c][TUNE_CORNER][0] -= 1
        print(f"{c} {TUNE_CORNER} row={CAM_OFFSET[c][TUNE_CORNER][0]}")
    elif key == ord("i"):
        c = CAM_NAMES[TUNE_CAM_IDX]
        CAM_OFFSET[c][TUNE_CORNER][0] += 1
        print(f"{c} {TUNE_CORNER} row={CAM_OFFSET[c][TUNE_CORNER][0]}")
    elif key == ord(","):
        c = CAM_NAMES[TUNE_CAM_IDX]
        CAM_OFFSET[c][TUNE_CORNER][1] -= 1
        print(f"{c} {TUNE_CORNER} col={CAM_OFFSET[c][TUNE_CORNER][1]}")
    elif key == ord("."):
        c = CAM_NAMES[TUNE_CAM_IDX]
        CAM_OFFSET[c][TUNE_CORNER][1] += 1
        print(f"{c} {TUNE_CORNER} col={CAM_OFFSET[c][TUNE_CORNER][1]}")
    elif key == ord("s"):
        save_offsets()

cv2.destroyAllWindows()
set_all_lights(True)      # on stop: leave every light ON
time.sleep(0.3)           # let the ON messages flush before disconnecting
print("Stopping — all lights left ON")
mqttclient.loop_stop()
