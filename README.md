# Multi-Camera Smart Lighting

Person-presence lighting for an open-plan floor. Four IP cameras detect people,
map each person onto a 22 × 28 floor-tile grid, and switch the nearest of 35
ceiling lights ON — shown live on a web floor map and, when enabled, driven on
the physical Bluetooth-mesh lighting gateway.

```
                 RTSP                 MQTT (localhost)              Socket.IO
 4 IP cameras ──────────►  detection.py  ──────────►  light-ui (Node)  ────────►  Browser
 (RTSP/ONVIF)              YOLOv8 + ByteTrack         Express + MQTT              floor map
                           homography → tiles                                     + camera feed
                                │  MJPEG :8765 ─────────────────────────────────────┘
                                │
                                ▼  MQTT (cloud, optional)
                        mesh gateway  ──►  physical lights
```

## Components

| Path | What it is |
|------|------------|
| `detection.py` | Reads the 4 cameras, runs detection/tracking, maps to tiles, publishes light + person state over MQTT, serves annotated MJPEG feeds. |
| `config.py` | Loads all connection settings from `.env`. |
| `config/` | Per-site calibration data (`homographies.json`, `cam_points.json`, `floor_points.json`, `cam_roi.json`, `cam_offsets.json`) and the light element-address map (`lights.json`). |
| `light-ui/` | Node web UI: floor map, live camera feed, drag-to-calibrate, master ON/OFF/AUTO. |
| `tools/` | `define_roi.py`, `compute_homographies.py`, `light_test.py` — calibration & test utilities. |
| `docs/` | Setup, camera placement, and calibration guides. |

## Quick start

1. Install prerequisites and dependencies — see **[docs/SETUP.md](docs/SETUP.md)**.
2. `cp .env.example .env` and fill in camera + gateway credentials.
3. Start the Mosquitto MQTT broker.
4. Run the UI: `cd light-ui && npm install && npm start` → open `http://<host>:3000`.
5. Run detection: `python detection.py`.

## If a camera is moved

Detection accuracy depends entirely on each camera's fixed position and its
calibration. **If any camera is bumped, re-aimed, or replaced, its lights will
map to the wrong tiles until it is recalibrated.** Follow:

- **[docs/CAMERA_PLACEMENT.md](docs/CAMERA_PLACEMENT.md)** — where each camera must point.
- **[docs/CALIBRATION.md](docs/CALIBRATION.md)** — how to redraw the region and recalibrate.

## Simulation vs. real lights

`REAL_LIGHTS_ENABLED=false` (default) runs everything except the physical lights —
the UI map still lights up. Set it to `true` in `.env` only after verifying the
mesh element addresses with `python tools/light_test.py <addr> on`.
