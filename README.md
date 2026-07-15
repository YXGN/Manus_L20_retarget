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
    somehand-feature/         # L20-right MuJoCo model + thumb segment IK dependency
  rosbag/                     # bag output directory
  build/ install/ log/        # colcon generated
```

## Main chain

```text
MANUS Metagloves
  -> manus_ros2 / manus_data_publisher
  -> /manus_glove_0
  -> manus_l20_retarget / manus_somehand_retarget_node
  -> /cb_right_hand_control_cmd
  -> linker_hand_ros2_sdk / linker_hand_advanced_g20
  -> can0
  -> LinkerHand L20
```

`linker_hand_advanced_g20` is the vendor executable name; this workspace documents and uses it as the L20 hand execution backend.

## Build

```bash
cd <Manus_L20_retarget>
./scripts/fetch_manus_sdk.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The repository intentionally does not download MANUS SDK binaries during a
normal clone. `fetch_manus_sdk.sh` downloads only the Integrated SDK required
by `manus_ros2`.

## Run

One launch:

```bash
ros2 launch bringup manus_somehand_linkerhand_g20.launch.py start_manus:=true
```

Or two terminals:

```bash
ros2 run manus_ros2 manus_data_publisher
ros2 launch bringup manus_somehand_linkerhand_g20.launch.py
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
