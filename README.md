# Vision-Guided Robot Manipulation

A robotics portfolio project integrating perception, coordinate
transformation, and motion planning using ROS 2 and Franka Panda.

## Target Pipeline

Test image → YOLO → 3D localization → TF2 → MoveIt 2 → Panda reach/lift

## Current Progress

Verified in the MoveIt Panda demo environment:

- ROS 2 Jazzy and MoveIt 2 are available.
- A fake PoseStamped is published on /detected_object_pose.
- A Python node sends the target to the /move_action action.
- MoveIt reports successful planning and execution.

## Current Prototype

Start the Panda demo:

```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

## Limitations

The current prototype accepts a manually supplied pose.
Vision, localization, TF2 integration, and reach/lift sequencing
are not yet implemented. No object gripping is demonstrated.
