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

## Calibration GUI directory package

The calibration GUI source and packaging script are located at:

```text
src/manus_l20_retarget/third_party/sharpa-manus-sdk/client/CalibrationGUI/
```

Build or refresh the standalone package in `deploy/` with:

```bash
cd src/manus_l20_retarget/third_party/sharpa-manus-sdk/client/CalibrationGUI
./package.sh ../../../../../../deploy/manus-calibration
```

Start it without sourcing the ROS workspace:

```bash
./deploy/manus-calibration/run.sh
```

The package contains the GUI executable, Integrated MANUS SDK, official GIF
assets, and a writable `calibration/` directory. It can be copied as a whole
to another Linux machine. When the package is inside this workspace, `run.sh`
automatically finds `src/manus_ros2/calibration/`; when deployed independently,
the `.mcal` files are saved inside the package's `calibration/` directory.
Override the destination explicitly when needed:

```bash
MANUS_CALIBRATION_DIR=/path/to/src/manus_ros2/calibration ./deploy/manus-calibration/run.sh
```

The target machine still needs GLFW, OpenGL, gdk-pixbuf/GLib runtime support
and a working MANUS Core or supported integrated SDK/hardware environment.
