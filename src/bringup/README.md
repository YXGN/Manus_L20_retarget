# Bringup

Main launch:

```bash
ros2 launch bringup manus_somehand_linkerhand_g20.launch.py start_manus:=true
```

Chain:

```text
/manus_glove_0
  -> manus_somehand_retarget
  -> /cb_right_hand_control_cmd
  -> linker_hand_advanced_g20
  -> LinkerHand L20
```

Path defaults are derived from the `Manus_L20_retarget` workspace root. Override with:

```bash
export MANUS_L20_ROOT=<Manus_L20_retarget>
```
