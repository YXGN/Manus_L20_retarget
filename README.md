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
- Optional fingertip contact semantics: calibrated raw-skeleton thumb-to-fingertip distance selects one contact pair and continuously blends only the thumb and selected finger's calibrated Root/Tip plus thumb Roll/Yaw. Other fingers remain on normal teleoperation. It is disabled by default.

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

## Qt calibration UI

The Qt application contains two calibration tabs: MANUS glove calibration
produces `.mcal` files through the Integrated SDK, while MANUS-L20 calibration
captures the ROS2 mapping YAML files.

```bash
./tools/run_calibration_ui.sh
```

The launcher incrementally builds the MANUS publisher when needed, starts it
if it is not already running, and saves glove calibration files in
`src/manus_ros2/calibration/`. The UI does not send L20 motion commands during
either calibration workflow.
