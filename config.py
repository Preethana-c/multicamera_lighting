"""
config.py — central configuration loaded from environment variables.

All deployment-specific values (camera IPs/credentials, MQTT broker, light
gateway credentials, ports) live in the .env file, NOT in the source code.
Copy .env.example to .env and edit it for your site.

Algorithm tuning (detection thresholds, floor grid, camera zones, the light
element-address map) stays in detection.py because it is not secret and rarely
changes once a site is calibrated.
"""

import os
from pathlib import Path

from dotenv import load_dotenv

# Load .env sitting next to this file
BASE_DIR = Path(__file__).resolve().parent
load_dotenv(BASE_DIR / ".env")

CONFIG_DIR = BASE_DIR / "config"          # calibration JSONs live here


def _bool(name, default=False):
    return os.getenv(name, str(default)).strip().lower() in ("1", "true", "yes", "on")


# ── Cameras (RTSP) ────────────────────────────────────────────────────────────
CAM_USER = os.getenv("CAM_USER", "admin")
CAM_PASS = os.getenv("CAM_PASS", "")
RTSP_PORT = os.getenv("CAM_RTSP_PORT", "554")
RTSP_PATH = os.getenv("CAM_RTSP_PATH", "/cam/realmonitor?channel=1&subtype=0")

CAM_NAMES = ["cam1", "cam2", "cam3", "cam4"]
_CAM_IPS = {
    "cam1": os.getenv("CAM1_IP", ""),
    "cam2": os.getenv("CAM2_IP", ""),
    "cam3": os.getenv("CAM3_IP", ""),
    "cam4": os.getenv("CAM4_IP", ""),
}


def rtsp_url(cam):
    ip = _CAM_IPS[cam]
    return f"rtsp://{CAM_USER}:{CAM_PASS}@{ip}:{RTSP_PORT}{RTSP_PATH}"


CAM_URLS = {cam: rtsp_url(cam) for cam in CAM_NAMES}

# ── Local MQTT broker (Mosquitto) ─────────────────────────────────────────────
MQTT_HOST = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

# ── Real light gateway (Bluetooth-mesh) ───────────────────────────────────────
REAL_LIGHTS_ENABLED = _bool("REAL_LIGHTS_ENABLED", False)
REAL_BROKER = os.getenv("REAL_BROKER", "mqtt.example.com")
REAL_PORT = int(os.getenv("REAL_PORT", "1883"))
REAL_USER = os.getenv("REAL_USER", "")
REAL_PWD = os.getenv("REAL_PWD", "")
REAL_CMD_TOPIC = os.getenv("REAL_CMD_TOPIC", "")
REAL_MSG_TOPIC = os.getenv("REAL_MSG_TOPIC", "")

# ── Servers / ports ───────────────────────────────────────────────────────────
MJPEG_PORT = int(os.getenv("MJPEG_PORT", "8765"))
UI_PORT = int(os.getenv("UI_PORT", "3000"))
