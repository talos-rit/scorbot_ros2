# scorbot_description

URDF/xacro descriptions of the Scorbot ER-4pc ("Bluey") and ER-V ("Bingo") for the
Talos-RIT ROS 2 stack.

```bash
ros2 launch scorbot_description view_robot.launch.py robot_type:=er_4pc   # or er_v
xacro urdf/scorbot.urdf.xacro robot_type:=er_v prefix:=bingo_ x:=1.0 > /tmp/bingo.urdf
```

## Layout

| Path | Contents |
| --- | --- |
| `urdf/scorbot.urdf.xacro` | top-level file; arguments listed in its header |
| `urdf/scorbot_macro.xacro` | the `scorbot_arm` macro: topology only, numbers come from YAML |
| `urdf/scorbot.ros2_control.xacro` | `<ros2_control>` block, mock or real hardware plugin |
| `config/<robot>_kinematics.yaml` | joint origins, axes, limits, visuals, inertials |
| `config/<robot>_calibration.yaml` | encoder and limit data pushed to the ESP32 controller |
| `meshes/er_4pc/` | STL meshes for the ER-4pc (no ER-V meshes yet; primitives are used) |
| `launch/view_robot.launch.py` | RViz plus joint sliders, no controllers |
| `test/test_xacro_expands.py` | expansion, topology and limit checks for both robots |

## Frames and sign conventions

```
world --(fixed)--> base_link --base_joint(z)--> turret_link --shoulder_joint(-y)--> upper_arm_link
  --elbow_joint(-y)--> forearm_link --wrist_pitch_joint(-y)--> wrist_link
  --wrist_roll_joint(+x)--> flange_link --(fixed)--> tool0
```

- `base_link` sits at the center of the base's mounting face, z up. At the zero pose
  the arm points straight out along +x and is horizontal.
- `base_joint` positive is counter-clockwise seen from above.
- `shoulder_joint`, `elbow_joint`, `wrist_pitch_joint` positive **lift** the distal
  link. Limits follow the manuals: shoulder +130°/−35°, elbow and pitch ±130°, base 310°
  total, roll ±570°.
- `wrist_roll_joint` positive is right-handed about +x (the tool direction).
- `tool0` has z pointing out of the flange, following ROS-Industrial. The camera mount
  offset is added by bringup, not here.
- Every link in the chain has the same orientation as `base_link` at the zero pose, so
  the joint origins in the YAML are plain translations along +x and +z.

## Provenance of the ER-4pc numbers

The meshes and joint placements come from the SolidWorks export the 2024-25 team left in
`commander/digital_twin/sboter4u_model/`. That export had a 3.6° tilt baked into the
base joint and used per-link frames rotated 90° from ours. Rather than trust a tilted
joint axis, the description keeps every axis exact and folds the export's transforms
into the mesh visual origins (`base_link` gets the inverse tilt, every other mesh gets a
90° yaw, the wrist and flange meshes an additional pitch/roll). The arm therefore
assembles exactly as the export did while the kinematics stay clean.

Derived link lengths: shoulder axis 0.346 m above the mounting face and 0.029 m ahead of
the base axis; upper arm 0.220 m; forearm 0.220 m; wrist pitch and roll axes intersect.

## Things to verify on the robots

The config files mark these `VERIFY`:

- Whether the base mesh appears tilted in RViz. If it does, the 3.6° in the export was
  an artifact and the base visual origin should be reset to identity.
- Joint velocity and effort limits (currently the export's placeholder values).
- Shoulder positive direction matches "lift" on the real robot once the ESP sign
  (`invert`) is set, and every encoder counts up on positive drive (`encoder_invert`;
  the two Scorbots have their encoder phases in opposite order).
- All ER-V joint placements; they are copied from the ER-4pc pending measurement.
- Encoder counts, gear ratios, home offsets and home directions in the calibration
  files. Bluey's ratios come from `hardware/motor_ratios.md`; Bingo's are unknown.
