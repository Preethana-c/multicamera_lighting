"""
light_test.py — manually turn ONE real light on/off to verify mesh control.

Use this to confirm a physical fixture's element address before trusting it in
detection.py. It publishes one command, then listens 5 s for the gateway to echo
the new state.

Usage (from project root):
  python tools/light_test.py <element_addr> [on|off]
Examples:
  python tools/light_test.py 89 on
  python tools/light_test.py 89 off
"""

import json
import os
import sys
import time
from pathlib import Path

import paho.mqtt.client as mqtt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config as cfg   # noqa: E402

if len(sys.argv) < 2:
    print("Usage: python tools/light_test.py <element_addr> [on|off]")
    sys.exit(1)

ELEMENT_ADDR = int(sys.argv[1])
TURN_ON = (sys.argv[2].lower() != "off") if len(sys.argv) > 2 else True

SEQ_FILE = str(Path(__file__).resolve().parent.parent / ".msgseq")

seq = 18000
if os.path.exists(SEQ_FILE):
    try:
        seq = int(open(SEQ_FILE).read().strip()) + 1
    except Exception:
        pass
if seq > 64000:
    seq = 18000
open(SEQ_FILE, "w").write(str(seq))

payload = {
    "MsgSeqNo": seq,
    "PacketId": 4,
    "ElementAddr": ELEMENT_ADDR,
    "ModelId": 4096,
    "State": [{"OnOff": 1 if TURN_ON else 0}],
}

state = {"connected": False}


def on_connect(c, u, f, rc):
    state["connected"] = (rc == 0)
    print(f"connect rc={rc}", "(0=OK)" if rc == 0 else "(FAILED)")
    if rc == 0 and cfg.REAL_MSG_TOPIC:
        c.subscribe(cfg.REAL_MSG_TOPIC)


def on_message(c, u, m):
    try:
        d = json.loads(m.payload.decode(errors="replace"))
    except Exception:
        return
    if d.get("ElementAddr") == ELEMENT_ADDR:
        st = d.get("States") or d.get("State")
        print(f"  ← gateway reports light {ELEMENT_ADDR}: {st}")


cli = mqtt.Client()
cli.username_pw_set(cfg.REAL_USER, cfg.REAL_PWD)
cli.on_connect = on_connect
cli.on_message = on_message
print(f"connecting to {cfg.REAL_BROKER}:{cfg.REAL_PORT} …")
cli.connect(cfg.REAL_BROKER, cfg.REAL_PORT, 60)
cli.loop_start()

for _ in range(50):
    if state["connected"]:
        break
    time.sleep(0.1)
if not state["connected"]:
    print("✗ never connected — check network/credentials.")
    cli.loop_stop()
    raise SystemExit

msg = json.dumps(payload)
info = cli.publish(cfg.REAL_CMD_TOPIC, msg, qos=1)
info.wait_for_publish()
print(f"SENT (seq {seq}) → {cfg.REAL_CMD_TOPIC}")
print(f"     {msg}")
print(f"Light {ELEMENT_ADDR} → {'ON' if TURN_ON else 'OFF'}. Listening 5s for confirmation…")

time.sleep(5)
cli.loop_stop()
cli.disconnect()
print("done.")
