# MANUS L20 Revo-Style Retarget

This is an independent MANUS -> L20 retarget path inspired by BrainCo's
Revo3 retarget structure:

```text
MANUS ergonomics
  -> four-finger flexion targets in radians
  -> relative spread targets in radians
  -> simple thumb ergonomics targets
  -> per-joint scale/offset calibration
  -> LinkerHand L20 20-slot command
```

It does not modify the existing `manus_l20_retarget` node.

## Offline Check

```bash
PYTHONPATH=src/manus_l20_revo_style \
python3 -m manus_l20_revo_style.offline_demo --close 0.75
```

## ROS Run

Debug only, publishes inside the node namespace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select manus_l20_revo_style
source install/setup.bash
ros2 launch manus_l20_revo_style revo_style_l20_right.launch.py input_topic:=/manus_glove_0
```

## MuJoCo Simulation

Start retargeting and the L20 MuJoCo simulation together:

```bash
ros2 launch manus_l20_revo_style revo_style_l20_mujoco_right.launch.py \
  input_topic:=/manus_glove_0 \
  enable_viewer:=true
```

For headless validation:

```bash
ros2 launch manus_l20_revo_style revo_style_l20_mujoco_right.launch.py \
  input_topic:=/manus_glove_0 \
  enable_viewer:=false
```

The simulation subscribes to `/manus_l20_revo_style_right/l20_command` and
publishes:

- `/manus_l20_revo_style_mujoco_right/joint_states`
- `/manus_l20_revo_style_mujoco_right/target_joint_states`

## Four-Finger Direct 255-0

For a minimal four-finger-only mapping, use
`config/four_finger_direct_l20_right.yaml`. It ignores the Revo-style YAML and
maps only:

- `Index/Middle/Ring/PinkyMCPStretch` -> L20 root slots `1..4`
- `Index/Middle/Ring/PinkyPIPStretch` -> L20 tip slots `16..19`

Each configured open ergonomics value maps to `255`; each closed ergonomics
value maps to `0`.

```bash
ros2 launch manus_l20_revo_style four_finger_direct_l20_mujoco_right.launch.py \
  input_topic:=/manus_glove_0 \
  enable_viewer:=true
```

Debug without MuJoCo:

```bash
ros2 launch manus_l20_revo_style four_finger_direct_l20_right.launch.py \
  input_topic:=/manus_glove_0
```

Hardware test, opt-in and conservative:

```bash
ros2 launch manus_l20_revo_style four_finger_direct_l20_hardware_right.launch.py \
  input_topic:=/manus_glove_0 \
  can:=can0
```

Do not run any other node that publishes `/cb_right_hand_control_cmd` at the
same time.

To send the generated command to the L20 driver, opt in explicitly:

```bash
ros2 launch manus_l20_revo_style revo_style_l20_right.launch.py \
  input_topic:=/manus_glove_0 \
  command_topic:=/cb_right_hand_control_cmd
```

## Tuning

Edit `config/revo_style_l20_right.yaml`.

- `four_finger`: Revo-style MANUS Stretch scale factors.
- `spread`: Revo-style relative spread calculation.
- `joint_calibration`: per-intermediate-joint scale/offset.
- `l20_command`: conversion from intermediate joint angles to L20 slots.
- `smoothing`: low-pass and per-cycle command delta limits.

For four-finger root/tip, `l20_command.direct_ergonomics_mapping` maps MANUS
ergonomics ranges directly to L20 bytes:

```yaml
index:
  root_key: IndexMCPStretch
  root_range: [open_value, closed_value]  # maps to slot 1: 255 -> 0
  tip_key: IndexPIPStretch
  tip_range: [open_value, closed_value]   # maps to slot 16: 255 -> 0
```

The thumb path here is intentionally simple. For higher-quality thumb behavior,
use this four-finger/spread path together with the existing L20 thumb segment IK
work from `manus_l20_retarget`.
