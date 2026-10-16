# SHARKY print files

These nine binary STL files are direct fabrication exports of the SHARKY body,
four legs, and four feet. They are supplied in millimetres and named by their
physical position on the assembled robot—no URDF, Xacro, simulation frames, or
ROS package metadata is mixed into this directory.

Print one of each file.

| File | Approximate bounding box (mm) |
| --- | ---: |
| `body.stl` | 119.71 × 104.50 × 71.77 |
| `front-left-leg.stl` | 37.19 × 56.98 × 38.26 |
| `front-left-foot.stl` | 20.00 × 77.66 × 15.48 |
| `front-right-leg.stl` | 37.19 × 56.98 × 38.26 |
| `front-right-foot.stl` | 20.00 × 77.66 × 15.48 |
| `rear-left-leg.stl` | 43.80 × 52.70 × 38.26 |
| `rear-left-foot.stl` | 41.88 × 74.61 × 15.48 |
| `rear-right-leg.stl` | 41.47 × 59.76 × 38.26 |
| `rear-right-foot.stl` | 48.19 × 74.61 × 15.48 |

## Before printing

1. Confirm that the body is approximately 120 mm long after import. If it is
   unexpectedly tiny or huge, set the slicer's import units to millimetres.
2. Choose a stable face and orient each part for your printer rather than
   assuming the exported CAD orientation is the ideal print orientation.
3. Preview every sliced layer and inspect thin walls, holes, bridges, and servo
   pockets.
4. Print a small fit test where practical. Servo cases, horns, screws, and
   printer calibration vary enough to affect assembly.

No universal material, support, wall, or infill setting is claimed: those
choices depend on the printer, material, payload, and intended gait testing.
