# Camera Placement

The system only works if each camera keeps the **exact position and aim** it had
when it was calibrated. The homography that converts camera pixels to floor tiles
is specific to one viewpoint — move the camera and every mapping shifts.

> **Golden rule:** if a camera is bumped, re-aimed, cleaned loose, or replaced,
> you must recalibrate it (see [CALIBRATION.md](CALIBRATION.md)). Nothing else in
> software will fix a moved camera.

## The floor grid

The floor is modelled as a **22-column × 28-row** tile grid. Lights sit at:

- Columns: **2, 7, 11, 15, 19**
- Rows: **3, 7, 11, 15, 19, 23, 27**

That is **35 lights** (7 rows × 5 columns), numbered left-to-right, top-to-bottom:
`light number = row_index * 5 + col_index + 1` (light 1 = top-left).

## Camera coverage

Each camera owns a section of the floor. Approximate assignment (see `CAM_ZONES`
in `detection.py` for the exact tile boxes):

| Camera | Physical view | Approx. floor area |
|--------|---------------|--------------------|
| cam1   | NE corner | full floor via ROI (right/top clusters) |
| cam2   | Upper section | full floor (background rows 0–9, foreground 12–27) |
| cam3   | NW section | cols 0–16, rows 0–13 |
| cam4   | Mid / open area | cols 0–18, rows 8–24 |

Overlap between cameras is fine — the same person seen twice is merged.

## Mounting guidelines (for reliable detection)

1. **Height & downtilt:** mount high (ceiling / high wall) angled down so people's
   **feet** are visible in most of the frame. Tile mapping uses the foot point;
   if feet are always hidden, standing people map less accurately.
2. **Cover the working area, not walls/ceiling:** aim so the floor fills the frame.
3. **Avoid strong backlight:** windows directly behind people cause silhouettes.
   CLAHE contrast boosting helps, but framing matters more.
4. **Keep it rigid:** use a fixed bracket. Any later nudge = recalibration.
5. **Overlap seams:** where two cameras meet, ensure a few tiles of overlap so a
   person is never lost between views.

## Record the installed position (do this once, keep it)

So the camera can be restored to the **same** view after maintenance, record for
each camera and store the images in `docs/screenshots/`:

- A **reference frame** grab from each camera at its final position
  (`python tools/define_roi.py cam1 shot.jpg` can display a saved grab, or take a
  snapshot from the camera's own web page). Name them `cam1_reference.jpg`, etc.
- A photo of the **physical mount** (bracket angle, position on wall/ceiling).
- The mounting height and tilt angle if known.

See [screenshots/README.md](screenshots/README.md) for the exact list. With these
on hand, restoring a camera to its old view is quick and often avoids a full
recalibration.
