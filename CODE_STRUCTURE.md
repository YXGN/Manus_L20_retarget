# Code Structure

| Path | Responsibility |
| --- | --- |
| `src/manus_ros2_msgs` | MANUS ROS messages. |
| `src/manus_ros2` | MANUS SDK publisher. |
| `src/manus_l20_retarget` | Active retarget node, calibration capture, simulation, diagnostics. |
| `src/l20_thumb_ik` | MuJoCo L20 model and thumb segment IK dependency. |
| `src/linker_hand_ros2_sdk` | G20/L20 CAN driver. |
| `src/manus_l20_haptics` | Optional tactile/haptic feedback. |
| `src/bringup` | Right, left, and bimanual launch files. |

Active calibration files:

```text
finger_flexion_ergonomics_<hand>_calibration.yaml
finger_yaw_ergonomics_<hand>_calibration.yaml
thumb_<hand>_flexion_ergonomics_mapping.yaml
thumb_segment_frame_<hand>.yaml
```

`manus_l20_retarget_node.py` combines these into one 20-slot L20 `JointState` command. `retarget_pipeline.py` contains the pure ergonomics mapping and command filtering math. `manus_landmarks.py` exposes named MANUS finger joints and retains the 21-point adapter only because the current thumb segment IK consumes it.
