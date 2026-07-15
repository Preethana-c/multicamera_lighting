# Calibration & Recalibration

Calibration teaches each camera how its pixels map to floor tiles. There are two
levels:

1. **Detection region (ROI)** — the area of the frame the camera watches.
2. **Homography** — the pixel → tile transform, built from matched points.

Plus two live fine-tuning layers: **offset correction** (keyboard) and
**drag-to-correct** (from the browser).

All calibration data lives in `config/` and is per-site:

| File | Meaning |
|------|---------|
| `cam_roi.json` | Detection polygon(s) per camera. |
| `cam_points.json` | Clicked pixel coordinates of reference points. |
| `floor_points.json` | The tile (col,row) each reference point sits on. |
| `homographies.json` | Computed pixel → tile transform per camera. |
| `cam_offsets.json` | Base + 4-corner fine-tune nudges per camera. |

> **Before you start, back up `config/`** so you can roll back:
> `cp -r config config_backup_<date>`

---

## When do I need to recalibrate?

- A camera was moved, re-aimed, or replaced → **full recalibrate that camera**.
- Detection is fine but lights are consistently a tile or two off → **offset
  fine-tune** or a few **drag corrections** are enough.

---

## A. Redraw the detection region (ROI)

Run for the affected camera and trace the area it should watch:

```bash
python tools/define_roi.py cam1
```

- Left-click around the floor area to enclose it.
- `n` = finish this region and start another (a camera can have several).
- `z` = undo point, `c` = clear, `s` = save, `q` = quit without saving.

Saved to `config/cam_roi.json`; `detection.py` reloads it live (no restart).

---

## B. Full homography recalibration (camera moved)

You need **4+ points** whose position you know both in the camera image and on
the floor grid. Pick easy-to-identify floor features (tile corners, floor marks,
furniture feet).

1. For each reference point, record:
   - its **pixel** location in that camera's 640×480 image → `cam_points.json`
   - its **tile** location `[col, row]` on the 22×28 grid → `floor_points.json`

   Use the same label (e.g. `"p1"`) in both files, under the camera's key:

   ```json
   // config/cam_points.json
   { "cam1": { "p1": [512, 430], "p2": [120, 300], "p3": [...], "p4": [...] } }

   // config/floor_points.json
   { "cam1": { "p1": [19, 7], "p2": [11, 15], "p3": [...], "p4": [...] } }
   ```

2. Rebuild the transform:

   ```bash
   python tools/compute_homographies.py
   ```

   It prints a per-point reprojection error and flags `BAD` points (bad pairs or
   typos). Aim for a low mean error (a couple of tiles or less). Fix or remove
   bad points and re-run.

3. Restart `detection.py`.

---

## C. Live drag-to-correct (easiest, from the browser)

With `detection.py` and the UI running:

1. Have a person stand at a known spot in the camera's view.
2. On the floor map, **drag their dot** from where it appears to where they
   actually are. The correction is sent to `detection.py`, which adds it as a new
   calibration point and rebuilds that camera's homography once 4+ drag points
   exist.
3. `Undo` removes the last drag point for a camera.

This is the quickest way to tighten a camera without editing files.

---

## D. Keyboard offset fine-tune

In the `detection.py` window, `Tab` to select a camera, then:

- `[` `]` shift all mappings left/right by a column; `-` `=` shift up/down a row.
- `c` selects a corner (TL/TR/BL/BR); `u`/`i` and `,`/`.` nudge just that corner
  (useful when one side of the frame is off but the other is fine).
- `s` saves to `config/cam_offsets.json`.

---

## Verifying calibration

1. Walk the floor and watch the map dot track your real position within ~1 tile.
2. Confirm the correct lights turn ON near you and OFF ~30 s after you leave.
3. For **physical** lights, first verify each fixture's element address with
   `python tools/light_test.py <addr> on` (and `off`), then map any changes in
   `config/lights.json`. Two UI numbers may share one address (same fixture).
