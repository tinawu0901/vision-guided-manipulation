# Validation

The saved-image prototype and the Gazebo RGB-D experiment are separate tests.
The prototype demonstrated ROS message flow and three executed poses; it did not
establish a geometric relationship between the stock photo and robot scene.

## Checks

```bash
source /opt/ros/jazzy/setup.bash
source ~/vision_guided_ws/install/setup.bash
python3 -m pytest vision_guided_simulation/test -q
```

Ten functional tests cover principal-point projection and axis signs, rejection of invalid
depth and intrinsics, quaternion sign equivalence, image row padding, depth byte
order, BGR message conversion, bottle model units/texture bindings and a normalized
downward hand target that leaves the measured point unchanged.

The original manipulation package's checks pass: two passed, one skipped. The
skipped check is the generated copyright-header test retained from the ROS package
template.

The integration monitor records localization XY error against the independently
configured scene position, `panda_hand` position against the final requested
pose, and quaternion angular error against the requested orientation.

Pass thresholds: 50 mm localization XY, 15 mm endpoint position and 0.18 rad
orientation. These are demonstration thresholds, not precision-grasping claims.
Height from a bounding-box centre measures a visible surface and is not evaluated
as the object's true centre height.

## Environment issues addressed

1. New Gazebo binaries alongside old ROS hardware-interface/lifecycle libraries
   produced undefined symbols. Installed ROS Jazzy packages were updated together.
2. The vision environment uses NumPy 2; the installed `cv_bridge` binary was built
   against NumPy 1. Image conversion handles ROS encodings, row stride and byte
   order directly.
3. VirtualBox rendering required a compatible Ogre/Xwayland configuration.
4. COLLADA export retained UV coordinates but dropped the texture binding; a
   complete image/surface/sampler/material binding restored the rendered bottle.
5. ROS Jazzy's Quaternion defaults to `w=1`. Setting only `x=1` produced a 90-degree
   hand rotation after normalization. Endpoint verification caught the error;
   the target now explicitly specifies the unit quaternion `(1, 0, 0, 0)`.
6. The overview camera publishes at its configured topic directly, while the
   RGB-D sensor appends `/image`. Their bridge topics are configured accordingly.

## Integration results

Tests ran on 2026-10-04 in Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic 8.15,
inside VirtualBox with Ogre software rendering and CPU YOLOv8n.

| Bottle reference XY (m) | Localization XY error | Final hand position error | Orientation error | Result |
|---|---:|---:|---:|---|
| (0.45, 0.00) | 4.0 mm | 8.1 mm | 5.4 degrees | Pass |
| (0.45, +0.08) | 4.5 mm | 5.8 mm | 4.9 degrees | Pass |
| (0.45, -0.08) | 4.4 mm | 7.7 mm | 6.7 degrees | Pass |

Raw reports are in [results](results/). Each report includes the detected pixel,
camera intrinsics, sampled depth, measured target and independent endpoint
comparison. The detector never reads the reference XY used by the monitor.

Reproduce a displaced-bottle fixture:

```bash
cd ~/vision_guided_ws/src/vision_guided_manipulation
python3 scripts/make_test_scene.py --y 0.08 --output /tmp/vision-scene-left.yaml
bash scripts/start_demo.sh --gui false --scene /tmp/vision-scene-left.yaml
```

Repeat with `--y -0.08`; omit `--scene` for the centre fixture. Startup archives
previous logs, reports and overview frames under `work/gazebo_results/runs/`.

These results establish integration for the tested scene and bottle model. They
do not measure general object-detection accuracy, calibration robustness, moving
object tracking or physical grasp success.
