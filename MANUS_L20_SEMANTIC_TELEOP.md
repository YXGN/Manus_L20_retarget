# MANUS L20 语义对指遥操作说明

本文档记录语义对指版本从 Qt 标定到双手启动的完整流程。普通四指弯曲、四指 yaw、拇指弯曲、拇指 Roll/Yaw IK 的基础说明仍参考 `MANUS_L20_TELEOP_SOP.md`。

本文统一使用 ASCII 路径 `/home/huangzizhe/Manus_L20_retarget-main`。它指向当前工程
`/home/huangzizhe/下载/Manus_L20_retarget-main`，用于避免 ROS 2 接口生成器处理中文路径时失败。

## 1. 功能范围

语义对指当前只识别一类人手动作：

- 拇指指尖接触食指指尖。

数据来源是 `ManusGlove.raw_nodes`。程序每帧取 `Thumb.TIP` 和 `Index.TIP`，计算：

```text
distance_ratio = ||Thumb.TIP - Finger.TIP|| / ||Index.MCP - Pinky.MCP||
```

运行时顺序：

```text
MANUS ergonomics 四指/拇指弯曲
-> MANUS ergonomics 四指 yaw
-> raw skeleton 拇指 segment IK
-> 语义对指局部混合
-> 发布 L20 20-slot 命令
```

语义对指只接管局部 slot：

```text
拇指 Root/Tip: slot 0, 15
当前对指手指 Root/Tip:
  食指: slot 1, 16
拇指 Roll/Yaw: slot 5, 10
```

中指、无名指、小指以及四指 yaw 继续走正常遥操作，不触发语义接管。

语义对指现在带有“分相位接管”：

```text
张开 -> 对指夹住:
  先接管 slot 5/10，让 Thumb Roll/Yaw 提前到当前对指目标位。
  后接管 slot 0/当前手指root/15/当前手指tip，让弯曲再夹住。

对指夹住 -> 张开:
  达到稳定接触后，root/tip 按当前距离连续回到 open，避免松开时反夹一下。
  slot 5/10 先慢后快退回正常拇指 IK，减少横向扫到物体。
```

这个提前量是相对于当前语义动作的；当前只保留食指对指，所以 slot 5/10 只会提前走食指对指 YAML 里的 Roll/Yaw。

接触状态会先确认稳定接触，再进入释放模式。释放过程中距离噪声不会让接管比例反向增加，避免 L20 手指在松开时抖动或重新夹紧。

## 2. Qt 标定入口

先完成通用遥操作 SOP 中的环境准备和 CAN 检查。使用当前工作区构建：

```bash
conda deactivate
cd /home/huangzizhe/Manus_L20_retarget-main
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source /home/huangzizhe/Manus_L20_retarget-main/install/setup.bash
```

如果之前构建失败过，第一次恢复构建时增加 `--cmake-clean-cache`：

```bash
colcon build --symlink-install --packages-up-to bringup --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

确认 ROS 没有落到旧工作区：

```bash
ros2 pkg prefix bringup
ros2 pkg prefix manus_l20_retarget
```

两个输出都应以 `/home/huangzizhe/Manus_L20_retarget-main/install/` 开头。

启动 Qt 标定工作台：

```bash
conda deactivate
cd /home/huangzizhe/Manus_L20_retarget-main
./tools/run_calibration_ui.sh
```

在 Qt 的 `MANUS 手套标定` 标签页完成左右手套 `.mcal` 标定；再在 `MANUS-L20 标定` 标签页完成基础 YAML 标定。该界面只订阅 MANUS 数据，不会向 L20 发送运动命令。

语义对指还要在 `MANUS-L20 标定` 标签页完成指尖接触标定，依次采集：自然张开、拇指-食指、拇指-中指、拇指-无名指、拇指-小指。标定结果写入：

```text
/home/huangzizhe/Manus_L20_retarget-main/src/manus_l20_retarget/config/fingertip_contact_semantics_left.yaml
/home/huangzizhe/Manus_L20_retarget-main/src/manus_l20_retarget/config/fingertip_contact_semantics_right.yaml
```

Qt 接触标定完成后会将 `runtime.enabled` 保持为 `false`。人工确认每个 L20 接触姿态后，再将左右 YAML 的 `runtime.enabled` 改为 `true`。

## 3. 启动 MANUS 数据发布器

新开一个终端，只启动 MANUS，不启动 L20 真机：

```bash
source /opt/ros/humble/setup.bash
cd /home/huangzizhe/Manus_L20_retarget-main
source install/setup.bash

ros2 run manus_ros2 manus_data_publisher --ros-args \
  -p load_calibration:=true \
  -p left_calibration_path:=/home/huangzizhe/Manus_L20_retarget-main/src/manus_ros2/calibration/Calibration_left.mcal \
  -p right_calibration_path:=/home/huangzizhe/Manus_L20_retarget-main/src/manus_ros2/calibration/Calibration_right.mcal
```

确认话题有数据：

```bash
ros2 topic hz /manus_glove_0
ros2 topic hz /manus_glove_1
```

如果只有一只手套，可能只有 `/manus_glove_0`。

## 4. 对指标定命令

右手：

```bash
source /opt/ros/humble/setup.bash
cd /home/huangzizhe/Manus_L20_retarget-main
source install/setup.bash

ros2 run manus_l20_retarget calibration_capture fingertip-contact \
  --hand right \
  --glove-topic /manus_glove_0 \
  --duration 2.0
```

左手：

```bash
source /opt/ros/humble/setup.bash
cd /home/huangzizhe/Manus_L20_retarget-main
source install/setup.bash

ros2 run manus_l20_retarget calibration_capture fingertip-contact \
  --hand left \
  --glove-topic /manus_glove_1 \
  --duration 2.0
```

如果左手手套实际发布在 `/manus_glove_0`，把左手命令里的 `/manus_glove_1` 改成 `/manus_glove_0`。

工具会依次采 2 个姿势：

- `natural_open`: 自然张开，拇指远离四指。
- `thumb_index_tip_touch`: 拇指指尖触碰食指指尖。

生成文件：

```text
src/manus_l20_retarget/config/fingertip_contact_semantics_right.yaml
src/manus_l20_retarget/config/fingertip_contact_semantics_left.yaml
```

## 5. YAML 检查和人工参数

每个 YAML 里有两类信息：

- `human`: MANUS 指尖距离阈值，由标定命令采集。
- `robot.contact_command`: L20 对指姿态，需要人工填入已经在真机验证过的 20-slot 命令。

当前对指 Root/Tip 参数按下面统一填写：

```text
拇指 Root slot 0 = 133
食指 Root slot 1 = 79
拇指 Tip slot 15 = 160
食指 Tip slot 16 = 152
```

当前唯一语义对指对应 slot：

```text
thumb-index: slot 0/1/15/16 = 133/79/160/152
```

拇指 Roll/Yaw 使用食指对指 `contact_command` 里的值：

```text
slot 5  Thumb Roll
slot 10 Thumb Yaw
```

启用语义对指前，确认左右 YAML 的 `runtime.enabled` 都为 `true`。Qt 重新采集指尖接触后会写回 `false`，这是为了让新姿态先经过真机确认。

新算法参数可以显式写入左右 YAML：

```yaml
runtime:
  enabled: true
  takeover_start_progress: 0.35
  full_takeover_progress: 0.75
  firm_contact_enter_activation: 0.90
  release_start_progress_delta: 0.06
```

`full_takeover_progress` 让闭合接管在接触前稳定饱和；`firm_contact_enter_activation` 用于确认已经形成稳定接触；`release_start_progress_delta` 抑制距离噪声造成的误释放。旧 YAML 省略这三项时，程序分别采用相同的默认值 `0.75`、`0.90`、`0.06`。`enabled` 必须是 `true`，不是 `ture`；如果 YAML 写错，launch 参数打开了也不会生效。

## 6. 双手语义遥操作启动命令

真机测试由用户手动执行。确认 CAN 已配置好后运行：

```bash
source /opt/ros/humble/setup.bash
cd /home/huangzizhe/Manus_L20_retarget-main
source install/setup.bash

ros2 launch bringup manus_l20_linkerhand_g20.launch.py \
  start_manus:=true \
  right_can:=can0 \
  left_can:=can1 \
  is_touch:=false \
  right_input_topic:=/manus_glove_0 \
  left_input_topic:=/manus_glove_1 \
  enable_fingertip_contact_semantics:=true \
  fingertip_contact_debug:=true \
  thumb_ik_debug:=true
```

当前 launch 默认速度是：

```text
driver_speed:=80,80,80,80,80
```

常用语义相位参数默认已经写进 launch：

```text
fingertip_contact_close_orientation_completion:=0.25
fingertip_contact_close_flexion_start:=0.40
fingertip_contact_release_flexion_open_completion:=0.65
fingertip_contact_release_orientation_gamma:=2.5
```

含义：

- `close_orientation_completion`: 从张开到夹住的前多少接近行程内，Roll/Yaw 就完成接管。数值越小，Roll/Yaw 越早到位。
- `close_flexion_start`: 接近行程超过多少以后，root/tip 才开始按语义夹住。数值越大，越晚闭合。
- `release_flexion_open_completion`: 从夹住到松开的前多少释放行程内，root/tip 完成回 open。当前默认值为 `0.65`。
- `release_orientation_gamma`: 松开时 Roll/Yaw 的回退曲线。数值越大，前段越慢、后段越快。

如果后续要临时覆盖默认值，也可以在启动命令后加新的数值，例如：

```bash
fingertip_contact_close_orientation_completion:=0.25 \
fingertip_contact_close_flexion_start:=0.40 \
fingertip_contact_release_flexion_open_completion:=0.65 \
fingertip_contact_release_orientation_gamma:=2.5
```

如果要显式写出来：

```bash
driver_speed:=80,80,80,80,80
```

如需在语义遥操期间同时开启真实触觉，在上述 launch 命令加入 `enable_haptics:=true`、`mock_tactile:=false`、左右手 `haptic_glove_id` 等参数。完整双手触觉命令、触觉话题和 mock 测试说明见 `MANUS_L20_TELEOP_SOP.md` 的“7. 双手触觉遥操作”。

## 7. 调试观察

打开 `fingertip_contact_debug:=true` 后，日志会显示语义接触事件：

```text
fingertip_contact activated:index activation=...
fingertip_contact released:index activation=...
```

如果完全没有 `activated:*`：

- 检查 `enable_fingertip_contact_semantics` 是否被命令行临时覆盖成 `false`。
- 检查对应 YAML 的 `runtime.enabled` 是否为 `true`。
- 检查 MANUS raw skeleton 是否完整输出。
- 检查左右手话题是否接反。

如果释放时手指重新夹紧或出现抖动：

- 确认代码和安装空间来自 `/home/huangzizhe/Manus_L20_retarget-main`。
- 确认 `full_takeover_progress`、`firm_contact_enter_activation` 和 `release_start_progress_delta` 使用左右 YAML 中的当前值。
- 不要把 `release_flexion_open_completion` 改回旧值 `0.18`。

如果识别到错误手指：

- 重新采 `fingertip-contact`。
- 自然张开姿势要足够张开，拇指不要靠近四指。
- 每个对指姿势只让拇指接触目标手指，其他手指尽量远离。

如果接触姿态不对：

- 优先检查 `contacts.<finger>.robot.contact_command`。
- `human` 只决定什么时候触发；真正 L20 对指姿态来自 `robot.contact_command`。
- 当前动作只会接管本动作声明的 `override_slots`。
