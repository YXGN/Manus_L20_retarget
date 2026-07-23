Place MANUS glove calibration files here:

- `Calibration_left.mcal`
- `Calibration_right.mcal`

These files are produced by:

```bash
cd src/manus_l20_retarget/third_party/sharpa-manus-sdk/client/CalibrationGUI
./build.sh
./CalibrationGUI.out
```

`bringup` launch files pass these paths to `manus_ros2/manus_data_publisher` by
default when `start_manus:=true`.
