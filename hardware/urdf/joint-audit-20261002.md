# SHARKY Fusion joint audit — 2026-10-02

The simplified Fusion assembly has the correct **nine components and eight
connections**. All eight connections were converted in place from Rigid joints
to Fusion **Revolute** joints (joint type `1`), given semantic names, and
limited to -90 to +90 degrees around the current CAD pose.

The selected joint origins are centred exactly on matching 8 mm-diameter
cylindrical hinge faces. The inferred axes below have zero measured line error,
so the placement, component pairing, and rotation axes are suitable for URDF
generation.

| Fusion joint | Role | Connected components | Inferred axis |
| --- | --- | --- | --- |
| `rear_right_knuckle_joint` | rear-right knuckle | LegR2 ↔ FootR2 | `(0.906308, -0.422618, 0)` |
| `rear_right_shoulder_joint` | rear-right shoulder | base ↔ LegR2 | `(0, 0, 1)` |
| `front_right_knuckle_joint` | front-right knuckle | LegR1 ↔ FootR1 | `(1, 0, 0)` |
| `front_right_shoulder_joint` | front-right shoulder | base ↔ LegR1 | `(0, 0, 1)` |
| `front_left_shoulder_joint` | front-left shoulder | LegL1 ↔ base | `(0, 0, 1)` |
| `front_left_knuckle_joint` | front-left knuckle | LegL1 ↔ FootL1 | `(-1, 0, 0)` |
| `rear_left_shoulder_joint` | rear-left shoulder | LegL2 ↔ base | `(0, 0, 1)` |
| `rear_left_knuckle_joint` | rear-left knuckle | LegL2 ↔ FootL2 | `(-0.906308, -0.422618, 0)` |

Axis signs are geometric conventions, not yet servo-positive directions. The
final URDF signs must be checked against the existing channel flip map and one
non-actuating/low-risk joint-state visualization before hardware use.

`fusion_tools/SHARKYCollisionAudit` performs a read-only temporary-BRep sweep.
For shoulder joints it rotates the leg and attached foot together; for knuckle
joints it rotates the foot. It compares each sampled pose with all remaining
solids, preserves the intentional zero-pose hinge overlap as a baseline, scans
at 5° intervals, and refines the first collision to 1°. Its result will be
written to `fusion_collision_sweep.json`.

## Collision sweep result

The sweep completed for all eight axes. No new solid intersection was found at
any 5-degree sample between -180 and +180 degrees, so every joint has a
**CAD-clear geometric interval of at least 360 degrees** around the current
pose. This does **not** mean that the installed servos can rotate 360 degrees.
It means the simplified CAD geometry does not provide a tighter collision stop
than the actuator and commissioned software limits.

The operational limits should therefore remain the narrower, already-tested
hardware-facing envelopes. With the current channel map these are:

| CAD joint | Robot joint | PCA9685 channel | Commissioned servo range |
| --- | --- | ---: | ---: |
| `front_left_shoulder_joint` | front-left shoulder | 0 | 15-110 deg |
| `front_left_knuckle_joint` | front-left knuckle | 1 | 75-178 deg |
| `front_right_shoulder_joint` | front-right shoulder | 2 | 70-165 deg |
| `front_right_knuckle_joint` | front-right knuckle | 3 | 2-105 deg |
| `rear_left_shoulder_joint` | rear-left shoulder | 4 | 30-110 deg |
| `rear_left_knuckle_joint` | rear-left knuckle | 5 | 75-175 deg |
| `rear_right_shoulder_joint` | rear-right shoulder | 6 | 70-150 deg |
| `rear_right_knuckle_joint` | rear-right knuckle | 7 | 5-105 deg |

These servo-degree ranges cannot yet be copied mechanically into URDF radians:
the right-side channels are mirrored in software, and the Fusion axis signs
have not yet been reconciled with servo-positive motion. The current CAD pose is
the provisional zero/neutral pose. Verify axis sign in RViz without actuating
the physical robot before publishing final operational URDF limits.

The reported geometric range is relative to the current CAD pose. It must not
be copied directly into servo degrees until the current pose's intended servo
angle (normally neutral/90°) is confirmed. In this model there is no collision
boundary to margin; the actuator rating and commissioned envelopes are the
meaningful limits.
