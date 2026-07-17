# Manus_L20_retarget

ROS2 workspace for MANUS glove -> LinkerHand L20 retargeting.

## Layout

```text
Manus_L20_retarget/
  src/
    ManusSDK/                 # MANUS SDK headers/libs
    manus_ros2_msgs/          # MANUS ROS2 messages
    manus_ros2/               # C++ MANUS publisher
    manus_l20_retarget/       # MANUS -> L20 command retarget
    linker_hand_ros2_sdk/     # LinkerHand G20 executable used for L20 hardware
    bringup/                  # main launch
    l20_thumb_ik/             # L20 MuJoCo model + thumb segment IK dependency
  rosbag/                     # bag output directory
  build/ install/ log/        # colcon generated
```

## Main chain

```text
MANUS Metagloves
  -> manus_ros2 / manus_data_publisher
  -> /manus_glove_0 and optionally /manus_glove_1
  -> manus_l20_retarget / manus_l20_retarget_node
  -> /cb_right_hand_control_cmd and/or /cb_left_hand_control_cmd
  -> linker_hand_ros2_sdk / linker_hand_advanced_g20
  -> can0
  -> LinkerHand L20
```

`linker_hand_advanced_g20` is the vendor executable name; this workspace documents and uses it as the L20 hand execution backend.

## Build

```bash
cd <Manus_L20_retarget>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

New user calibration and teleoperation SOP:

```text
MANUS_L20_TELEOP_SOP.md
```

Right hand only:

```bash
ros2 launch bringup manus_l20_linkerhand_g20_right.launch.py start_manus:=true
```

Left hand only:

```bash
ros2 launch bringup manus_l20_linkerhand_g20_left.launch.py start_manus:=true
```

Bimanual launch:

```bash
ros2 launch bringup manus_l20_linkerhand_g20.launch.py start_manus:=true
```

The bimanual launch defaults to `right_input_topic:=/manus_glove_0` and
`left_input_topic:=/manus_glove_1`. Override them if your MANUS publisher
assigns the gloves differently.

Or start MANUS separately:

```bash
ros2 run manus_ros2 manus_data_publisher
ros2 launch bringup manus_l20_linkerhand_g20_right.launch.py
```

If moved elsewhere, optionally set:

```bash
export MANUS_L20_ROOT=<Manus_L20_retarget>
```

## Diagnostics / simulation helpers

```bash
ros2 run manus_l20_retarget mock_manus_publisher
ros2 run manus_l20_retarget g20_joint_probe
ros2 run manus_l20_retarget inspect_manus_landmarks
ros2 run manus_l20_retarget visualize_thumb_ik
```

## Bag directory

Use `rosbag/` for recordings, for example:

```bash
ros2 bag record -o rosbag/manus_l20_session /manus_glove_0 /cb_right_hand_control_cmd /cb_right_hand_state
```

For bimanual runs, also record `/manus_glove_1` and `/cb_left_hand_control_cmd`.
The legacy
`manus_l20_linkerhand_l20_left.launch.py` entrypoint is kept as a
compatibility alias for the left-hand G20/L20 pipeline.
