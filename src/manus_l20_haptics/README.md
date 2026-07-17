# MANUS L20 Haptics

ROS2 package that adapts LinkerHand L20 fingertip force readings to MANUS
Metagloves Pro Haptic vibration commands.

```text
L20 tactile CAN
  -> linkerhand_l20_tactile_source
  -> /manus_l20_haptics/force
  -> manus_l20_haptic_feedback
  -> /manus_glove_0/vibration_cmd
```

The force topic uses `std_msgs/Float32MultiArray` with 20 values:
`normal[0:5] + tangential[0:5] + tangential_dir[0:5] + approach[0:5]`.
`read_mode:=auto` detects matrix tactile sensors and reduces each fingertip
matrix with `matrix_reduce` (`max`, `mean`, or `sum`) before haptic mapping.

Standalone launch:

```bash
ros2 launch manus_l20_haptics l20_haptics.launch.py \
  hand_type:=left can_channel:=can0 glove_id:=0
```

Without CAN hardware:

```bash
ros2 launch manus_l20_haptics l20_haptics.launch.py \
  mock_tactile:=true glove_id:=0
```

Key parameters live in `config/haptic_feedback.yaml`. Tune
`normal_force_full_scale`, `contact_threshold`, `attack_alpha`, and
`release_alpha` first.
