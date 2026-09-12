# scorbot_description

URDF/xacro descriptions of the Scorbot ER-4pc ("Bluey") and ER-V ("Bingo") for the Talos-RIT ROS 2 stack.

## Layout

| Path | Contents |
| --- | --- |
| `urdf/scorbot.urdf.xacro` | top-level file, arguments listed in its header |
| `urdf/scorbot_macro.xacro` | the `scorbot_arm` macro, topology only, numbers come from YAML |
| `urdf/scorbot.ros2_control.xacro` | `<ros2_conrol>` block, mock or real hardware plugin |
| `config/<robot>_kinematics.yaml` | joint origins, axes, limits, visuals, intertials |
| `config/<robot>_calibration.yaml` | encoder and limit data pushed to the ESP32 controller |
| `meshes/er_4pc/` | STL meshes for the ER-4pc |
| `launch/view_robot.launch.py` | RViz plus joint sliders, no controllers |
| `test/test_xacro_expands.py` | expansion, topology and limit checks for both  robots |

## Frames and sign conventions

```
world --(fixed)--> base_link --base_joint(z) --> turret_link --shoulder_joint(-y)--> upper_arm_link --elbow_joint(-y)--> forarm_link --wrist_pitch_joint(-y)--> wrist_link --wrist_roll_joint(+x)--> flange_link --(fixed)--> tool0
```

- `base_link` sits at the center of the base's mounting face, z up. At the zero pose the arm points straight out along +x and is horizontal.
- `base_joint` positive is counter-clockwise seen from above.
- `shoulder_joint`, `elbow_joint`, `wrist_pitch joint` positive **lift** the distal link. Limits follwo the manuals: shoudler +130/-35 degrees, elbow and pitch +- 130 degrees, base 310 degrees total, roll +-570 degrees.
- `wrist_roll_joint` positive is right-handed about +x (the tool direction). 
- `tool0` has z pointing out of the flange, following ROS-Industrial. The camera mount offset is added by bringup.
- Every link in the chain has the same orientation as `base_link` at the zero pose, so the joint origins in the YAML are plain translations along +x and +z. 

## Provenance of the ER-4pc numbers

The meshes and joint placements come from the SolidWorks export the 2024-25 team left in `commander/digital_twin/sboter4u_model/`. That export had a 3.6 degree tilt baked into the base joint and used per-link frames rotated 90 degrees from ours. Rather than trust a tilted joint axis, the description keeps every axis exact and folds the export's transforms into the mesh visual origins (`base_link` gets the inverse tilt, every other mesh gets a 90 degree yaw, the wrist and flange meshes an additional pitch/roll). the arm therefore assembles exactly as the export did while the kinematics stay clean.

## Things to verify on the robots

The config files mark these `VERIFY`:

- Whether the base mesh appears tilted in RViz, if it does, the 3.6 degrees in the export was an artifact and the base visual origin should be reset to identity. 
- Joint velocity and effor limits.
- SHoulder positive direction matches "lift" on the real robot once the ESP sign (`invert`) is set, and every encoder phases in opposite order).
- All ER-V joint placements, they are copied form the ER-4pc pending measurement.
- Encoder counts, gear ratios, home offsets and home directions in the calibration files. Bluey's ratios come from `hardware/motor_ratios.md`, Bingo's are unknown.