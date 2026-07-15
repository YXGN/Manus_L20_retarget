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
