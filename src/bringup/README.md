# Bringup

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

MANUS glove `.mcal` calibration is loaded automatically by `manus_data_publisher`
when `start_manus:=true`. The default paths are:

```text
src/manus_ros2/calibration/Calibration_left.mcal
src/manus_ros2/calibration/Calibration_right.mcal
```

To create or refresh these files, build and run the Sharpa calibration GUI:

```bash
cd src/sharpa-manus-sdk-main/client/CalibrationGUI
./build.sh
./CalibrationGUI.out
```

After completing left/right calibration, copy or point the launch arguments at
the generated files:

```bash
ros2 launch bringup manus_l20_linkerhand_g20.launch.py \
  start_manus:=true \
  left_manus_calibration_path:=/path/to/Calibration_left.mcal \
  right_manus_calibration_path:=/path/to/Calibration_right.mcal
```

Use `load_manus_calibration:=false` to run without applying `.mcal` files.

Bimanual defaults:

```text
right_input_topic := /manus_glove_0
left_input_topic  := /manus_glove_1
right_can         := can0
left_can          := can0
```

Single-hand chain:

```text
/manus_glove_0
  -> manus_l20_retarget
  -> /cb_right_hand_control_cmd
  -> linker_hand_advanced_g20
  -> LinkerHand L20
```

Optional L20 haptic feedback:

```bash
ros2 launch bringup manus_l20_linkerhand_l20_left.launch.py \
  start_manus:=true enable_haptics:=true haptic_glove_id:=0
```

For a no-hardware vibration-path smoke test, add `mock_tactile:=true`. The
haptic chain publishes:

```text
LinkerHand L20 tactile CAN
  -> /manus_l20_haptics/left/force
  -> /manus_glove_0/vibration_cmd
  -> MANUS glove vibration motors
```

The right-hand launch publishes to `/cb_right_hand_control_cmd`, sets
`hand_side=right`, and uses `src/l20_thumb_ik/configs/retargeting/right/linkerhand_l20_right.yaml`.
The left-hand launch publishes to `/cb_left_hand_control_cmd`, but intentionally
uses the verified right-hand retargeting path (`hand_side=right`, right-hand
calibration files, and `linkerhand_l20_right.yaml`). This lets a left MANUS glove
drive the left LinkerHand through the same command semantics that were validated
on the right hand.
The legacy `manus_l20_linkerhand_l20_left.launch.py` entrypoint remains
available as a compatibility alias.

Path defaults are derived from the `Manus_L20_retarget` workspace root. Override with:

```bash
export MANUS_L20_ROOT=<Manus_L20_retarget>
```
