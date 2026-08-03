# L20 Thumb IK Core

This directory contains the local MuJoCo model and Python IK utilities used by
`manus_l20_retarget` for LinkerHand L20 thumb roll/yaw solving.

The outer directory is a project dependency, not a ROS package. Runtime code adds
`src/l20_thumb_ik/src` to `sys.path` and imports the internal Python package:

```python
import l20_ik_core
```

## What This Module Does

- Loads L20 MuJoCo hand models from `assets/mjcf`.
- Loads retargeting YAML configs from `configs/retargeting`.
- Converts LinkerHand SDK command ranges to MuJoCo qpos and back.
- Provides vector-based IK primitives used by the MANUS -> L20 thumb segment IK.
- Provides optional visualization/runtime helpers for debugging the model.

## Important Paths

```text
src/l20_thumb_ik/
  assets/mjcf/                     MuJoCo hand assets.
  configs/retargeting/base/        Shared L20 model config.
  configs/retargeting/right/       Right-hand L20 config.
  configs/retargeting/left/        Left-hand L20 config.
  src/l20_ik_core/                 Internal Python library.
  third_party/linkerhand-python-sdk/
                                   Minimal LinkerHand SDK mapping dependency.
```

## Runtime Use

The ROS node normally receives these launch parameters:

```text
l20_thumb_ik_root=<workspace>/src/l20_thumb_ik
l20_thumb_ik_config_path=<workspace>/src/l20_thumb_ik/configs/retargeting/<side>/linkerhand_l20_<side>.yaml
linkerhand_sdk_root=<workspace>/src/l20_thumb_ik/third_party/linkerhand-python-sdk
```

Do not rename the internal package or config paths without updating
`src/manus_l20_retarget/manus_l20_retarget/manus_l20_retarget_node.py` and
`src/bringup/launch/manus_l20_common/pipeline.py`.
