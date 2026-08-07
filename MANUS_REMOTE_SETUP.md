# MANUS L20 remote setup notes

Use this when deploying `Manus_L20_retarget` to another machine.

## 1. Sync source only

Do not copy `build/`, `install/`, or `log/` between machines. They contain
absolute paths and stale symlinks.

```bash
cd /data/codeBase/src/agx_arm_ws

rsync -avz --delete \
  --exclude build/ --exclude install/ --exclude log/ --exclude .git/ \
  src/Manus_L20_retarget/ \
  luodongxu@192.168.100.24:~/agx_arm_ws/src/Manus_L20_retarget/
```

## 2. Build on target

```bash
cd ~/agx_arm_ws/src/Manus_L20_retarget
rm -rf build install log

./scripts/fetch_manus_sdk.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If only rebuilding MANUS publisher:

```bash
colcon build --symlink-install --packages-select manus_ros2_msgs manus_ros2
source install/setup.bash
```

## 3. MANUS Sensor Dongle permissions

If ROS starts but logs:

```text
MANUS landscape: dongles=0, gloves=0, users=0, license.sdk=false, license.integrated=false
```

then Linux may see the USB dongle, but MANUS Core Integrated cannot read it.

Check:

```bash
lsusb -d 3325:0049
```

Expected:

```text
ID 3325:0049 Manus VR ... Sensor Dongle
```

Add persistent udev rules:

```bash
sudo tee /etc/udev/rules.d/99-manus-usb.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="3325", ATTR{idProduct}=="0049", MODE="0666", GROUP="plugdev"
KERNEL=="hidraw*", ATTRS{idVendor}=="3325", ATTRS{idProduct}=="0049", MODE="0666", GROUP="plugdev"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then unplug/replug the MANUS Sensor Dongle.

Verify permissions:

```bash
lsusb -d 3325:0049
ls -l /dev/bus/usb/001/$(lsusb -d 3325:0049 | awk '{print $4}' | tr -d :)
```

Expected permission:

```text
crw-rw-rw- ... /dev/bus/usb/001/XXX
```

Note: `Device XXX` changes after every unplug/replug. That is normal.

## 4. Start MANUS publisher

```bash
cd ~/agx_arm_ws/src/Manus_L20_retarget
source /opt/ros/humble/setup.bash
source install/setup.bash

pkill -f Manus || true
pkill -f Core || true

ros2 run manus_ros2 manus_data_publisher
```

Good first sign:

```text
MANUS landscape: dongles=1, ...
```

If `dongles=1` but `license.integrated=false`, the dongle is visible but the
current MANUS authorization does not include SDK Integrated.

## 5. Start full L20 chain

```bash
cd ~/agx_arm_ws/src/Manus_L20_retarget
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bringup manus_l20_linkerhand_g20_right.launch.py start_manus:=true can:=can0
```

Verify topics:

```bash
ros2 topic hz /manus_glove_0
ros2 topic hz /cb_right_hand_control_cmd
ros2 topic hz /cb_right_hand_state
```

Interpretation:

- `/manus_glove_0` missing: MANUS dongle/license/glove side.
- `/manus_glove_0` exists but `/cb_right_hand_control_cmd` missing: retarget side.
- command exists but hand does not move: CAN/L20 driver side.
