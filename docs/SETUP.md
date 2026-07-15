# Setup & Deployment

## 1. Prerequisites

| Software | Notes |
|----------|-------|
| Python 3.9+ | For `detection.py` and the tools. |
| Node.js 18+ | For the web UI. |
| Mosquitto MQTT broker | Local message bus between detection and the UI. |
| A CUDA GPU (optional) | YOLO runs on CPU but a GPU is strongly recommended for 4 cameras. |

### Install the MQTT broker

- **Windows:** install from https://mosquitto.org/download/ and run the
  `Mosquitto` service (listens on `localhost:1883`).
- **Linux:** `sudo apt install mosquitto && sudo systemctl enable --now mosquitto`.

The default config (anonymous, localhost:1883) is enough. `detection.py` and the
UI both connect to `MQTT_HOST:MQTT_PORT` from `.env`.

## 2. Get the code and configure

```bash
git clone <your-repo> smart-lighting
cd smart-lighting
cp .env.example .env
```

Edit `.env` and set:
- `CAM_USER` / `CAM_PASS` and each `CAMn_IP` for your cameras.
- `MQTT_HOST` / `MQTT_PORT` (usually `localhost` / `1883`).
- Gateway credentials (`REAL_*`) — leave `REAL_LIGHTS_ENABLED=false` until tested.

`.env` is git-ignored and must never be committed.

## 3. Python side (detection)

```bash
python -m venv venv
# Windows:  venv\Scripts\activate
# Linux:    source venv/bin/activate
pip install -r requirements.txt
```

The YOLO weights (`yolov8s.pt`) download automatically on first run if not
present. Then:

```bash
python detection.py
```

A window opens showing the annotated camera feeds. Keys:

| Key | Action |
|-----|--------|
| `1` `2` `3` `4` | Show a single camera |
| `0` | 2×2 grid of all cameras |
| `Tab` | Cycle which camera the offset keys tune |
| `c` | Cycle the corner being tuned (TL/TR/BL/BR) |
| `[` `]` | Base column offset − / + |
| `-` `=` | Base row offset − / + |
| `u` `i` | Selected corner row − / + |
| `,` `.` | Selected corner column − / + |
| `s` | Save offsets to `config/cam_offsets.json` |
| `q` | Quit (leaves all lights ON) |

## 4. Web UI

```bash
cd light-ui
npm install
npm start
```

Open `http://<detection-host>:3000`. The camera feed inside the UI is pulled
from port `8765` of the same host that serves the page, so it works from any
machine on the LAN.

## 5. Run order (production)

1. Mosquitto broker (service, always on).
2. `python detection.py`.
3. `cd light-ui && npm start`.

Consider running detection and the UI as services (systemd on Linux, NSSM or
Task Scheduler on Windows) so they restart on boot/crash.

## Ports

| Port | Used by |
|------|---------|
| 1883 | MQTT broker |
| 8765 | MJPEG camera feeds (detection.py) |
| 3000 | Web UI (light-ui) |

## Timing behaviour

- A light turns ON when a person is within `LIGHT_RADIUS` (3) tiles.
- It stays ON for `LIGHT_HOLD` (30 s) after the last detection; a person's track
  is held `DETECTION_HOLD` (8 s) after they leave view, so total lingering is up
  to ~38 s. Adjust both at the top of `detection.py`.
- On start and on quit, **all lights are switched ON** deliberately.
