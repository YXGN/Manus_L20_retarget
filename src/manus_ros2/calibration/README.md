Place MANUS glove calibration files here:

- `Calibration_left.mcal`
- `Calibration_right.mcal`

These files are produced from the `MANUS 手套标定` tab in the Qt application:

```bash
./tools/run_calibration_ui.sh
```

`bringup` launch files pass these paths to `manus_ros2/manus_data_publisher` by
default when `start_manus:=true`.

Legacy ImGui calibration outputs, if retained, belong under `archive/`. They
are not loaded by the publisher.
