# SHARKY URDF status

The Fusion mesh export is under `fusion_export/Quadruped_description`. It
contains the base plus four upper-leg and four foot STL meshes. The earlier
generated `Quadruped.xacro` is **not yet a usable robot model** because it was
exported before the assembly joints existed and therefore contains only
`base_link`.

The read-only audit at `fusion_assembly_manifest.json` records component bounds,
mass properties, transforms, and any assembly joints. The audit script lives at
`fusion_tools/SHARKYAssemblyAudit` and also reports cylindrical faces to help
locate the eight physical servo axes without guessing from mesh bounding boxes.

Expected leg mapping, assuming Fusion +X is robot-forward and +Y is left:

| CAD component | URDF position |
| --- | --- |
| `LegL1_1_1` / `FootL1_1_1` | front left |
| `LegR1_1_1` / `FootR1_1_1` | front right |
| `LegL2_1_1` / `FootL2_1_1` | rear left |
| `LegR2_1_1` / `FootR2_1_1` | rear right |

Fusion now contains eight named, limited Revolute joints: four body-to-leg
shoulder joints and four leg-to-foot knuckle joints. Their origins are the real
servo shaft centres, and their axes and component pairings are recorded in
`fusion_revolute_setup.json` and `fusion_assembly_manifest.json`.

Regenerate the Xacro and link-local meshes/frames from this saved Fusion
revision. Reconcile each model-axis sign with the controller's channel flip map,
replace the provisional +/-90 degree model range with the tested operational
range where appropriate, and verify the complete tree in RViz at neutral before
adding Gazebo dynamics or learned-gait training.
