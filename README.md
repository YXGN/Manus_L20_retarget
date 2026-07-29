# Manus_L20_retarget

ROS 2 Humble workspace for MANUS glove teleoperation of LinkerHand L20/G20.

## Active Control Path

```text
MANUS glove -> manus_l20_retarget -> /cb_<hand>_hand_control_cmd
            -> linker_hand_advanced_g20 -> CAN -> L20
```

- Four-finger root/tip: calibrated MANUS ergonomics flexion.
- Four-finger yaw: calibrated MANUS ergonomics spread values.
- Thumb root/tip: calibrated MANUS ergonomics flexion.
- Thumb roll/yaw: mandatory two-pose segment IK frame.

Revo retargeting, geometric flexion calibration, raw/orientation yaw calibration, full-hand IK, direct thumb roll/yaw, and startup open-vector alignment are removed from the active path.

## Build

```bash
source /opt/ros/humble/setup.bash
cd /home/huangzizhe/Manus_L20_retarget
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch bringup manus_l20_linkerhand_g20_right.launch.py start_manus:=true can:=can0
ros2 launch bringup manus_l20_linkerhand_g20_left.launch.py start_manus:=true can:=can1
ros2 launch bringup manus_l20_linkerhand_g20.launch.py start_manus:=true right_can:=can0 left_can:=can1
```

See [CODE_STRUCTURE.md](CODE_STRUCTURE.md) for package ownership and `MANUS_L20_TELEOP_SOP.md` for the detailed calibration procedure.
