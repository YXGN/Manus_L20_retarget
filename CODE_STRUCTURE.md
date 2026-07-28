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
flexion_<hand>_calibration.yaml
finger_yaw_ergonomics_<hand>_calibration.yaml
thumb_<hand>_flexion_mapping.yaml
thumb_segment_frame_<hand>.yaml
```

`manus_l20_retarget_node.py` combines these into one 20-slot L20 `JointState` command. `retarget_pipeline.py` contains pure flexion/mapping math, and `manus_landmarks.py` only adapts MANUS raw nodes into landmarks.
