# MANUS 手套到 LinkerHand L20 遥操作流程

本文档面向第一次拿到 MANUS 手套和 LinkerHand L20 的操作者，流程从 Qt 标定开始，到启动双手遥操作程序结束。

本文统一使用 ASCII 路径 `/home/huangzizhe/Manus_L20_retarget-main`。它指向当前工程
`/home/huangzizhe/下载/Manus_L20_retarget-main`，用于避免 ROS 2 接口生成器处理中文路径时失败。

## 1. 准备环境

打开一个终端，退出 Conda 环境，进入工作区并加载 ROS 2 环境：

```bash
conda deactivate
cd /home/huangzizhe/Manus_L20_retarget-main
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source /home/huangzizhe/Manus_L20_retarget-main/install/setup.bash
```

如果之前构建失败过，第一次恢复构建时使用：

```bash
cd /home/huangzizhe/Manus_L20_retarget-main
colcon build --symlink-install --packages-up-to bringup --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

如果只改过 Python 代码，后续一般只需要：

```bash
cd /home/huangzizhe/Manus_L20_retarget-main
source /home/huangzizhe/Manus_L20_retarget-main/install/setup.bash
```

确认 ROS 使用的是当前工作区：

```bash
ros2 pkg prefix bringup
ros2 pkg prefix manus_l20_retarget
```

输出都应以 `/home/huangzizhe/Manus_L20_retarget-main/install/` 开头。

## 2. 连接硬件

1. 将 MANUS 手套接收器插到电脑上。
2. 给左右手套上电，确认手套已配对。
3. 连接 LinkerHand L20 右手到 `can0`，左手到 `can1`。
4. 确认机械手周围没有障碍物，手指初始姿态安全。

检查 CAN 口是否存在：

```bash
ip -details link show can0
ip -details link show can1
```

如果 CAN 口未启动，手动启动：

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can1 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```

再次检查：

```bash
ip -details link show can0
ip -details link show can1
```

## 3. 手套标定

第一次使用、换操作者、重新佩戴手套、或者感觉手指弯曲角度明显不准时，都建议重新标定 MANUS 手套。

启动 Qt 标定工作台。脚本会自动启动 MANUS 数据发布器；不要在另一个终端重复运行 `manus_data_publisher`：

```bash
conda deactivate
cd /home/huangzizhe/Manus_L20_retarget-main
./tools/run_calibration_ui.sh
```

在 Qt 界面依次使用两个标签页。

### 3.1 MANUS 手套标定

在 `MANUS 手套标定` 标签页按以下流程操作：

1. 选择左手或右手手套。
2. 点击 `开始标定`。
3. 按界面提示做手势。
4. 每一步完成后点击 `下一步`。
5. 左右手都标定一遍。

标定完成后，程序会生成并自动同步以下文件：

```text
/home/huangzizhe/Manus_L20_retarget-main/src/manus_ros2/calibration/Calibration_left.mcal
/home/huangzizhe/Manus_L20_retarget-main/src/manus_ros2/calibration/Calibration_right.mcal
```

确认文件已生成：

```bash
ls -lh /home/huangzizhe/Manus_L20_retarget-main/src/manus_ros2/calibration/Calibration_*.mcal
```

这些 `.mcal` 文件会在启动 `manus_data_publisher` 时自动加载，不需要手动复制。

### 3.2 MANUS-L20 标定

在 `MANUS-L20 标定` 标签页选择左右手，按界面提示完成基础标定。该标签页只订阅 MANUS 数据，不会向 L20 发送运动命令，会生成四指弯曲、四指 yaw、拇指弯曲和拇指 segment IK 所需的 YAML。

如需使用语义对指，在同一标签页继续完成指尖接触标定，生成：

```text
src/manus_l20_retarget/config/fingertip_contact_semantics_left.yaml
src/manus_l20_retarget/config/fingertip_contact_semantics_right.yaml
```

完成后关闭 Qt，再继续数据检查和真机启动。

## 4. 标定后数据检查

新开一个终端：

```bash
cd /home/huangzizhe/Manus_L20_retarget-main
source /home/huangzizhe/Manus_L20_retarget-main/install/setup.bash
ros2 run manus_ros2 manus_data_publisher
```

再开一个终端查看话题：

```bash
cd /home/huangzizhe/Manus_L20_retarget-main
source /home/huangzizhe/Manus_L20_retarget-main/install/setup.bash
ros2 topic list | grep manus_glove
```

检查左右手套是否有数据：

```bash
ros2 topic hz /manus_glove_0
ros2 topic hz /manus_glove_1
```

如果只有一个手套，可能只会出现 `/manus_glove_0`。如果左右手对应反了，遥操作启动时交换 `right_input_topic` 和 `left_input_topic`。

检查完成后，在运行 `manus_data_publisher` 的终端按 `Ctrl+C` 退出。

## 5. 采集人手到 L20 映射数据

这一节采的是我们自己的 retarget 标定 YAML，不是 MANUS 官方 `.mcal`。

两者区别：

- 第 3 节 `.mcal`：MANUS 手套自己的标定文件，让手套输出更准。
- 本节 YAML：把 MANUS 人手动作映射到 L20 的 20 路控制命令。

如果只是日常启动，不需要每次都采。以下情况建议重采：

- 换操作者。
- 重新佩戴手套后动作明显变差。
- 修改了 `landmark_transform`、拇指 IK 策略或左右手逻辑。
- 想重新校准四指弯曲、四指 yaw、拇指弯曲或拇指 IK 平面。

先启动 MANUS 数据发布器。新开一个终端：

```bash
source /opt/ros/humble/setup.bash
cd /home/huangzizhe/Manus_L20_retarget-main
source /home/huangzizhe/Manus_L20_retarget-main/install/setup.bash

ros2 run manus_ros2 manus_data_publisher --ros-args \
  -p load_calibration:=true \
  -p left_calibration_path:=/home/huangzizhe/Manus_L20_retarget-main/src/manus_ros2/calibration/Calibration_left.mcal \
  -p right_calibration_path:=/home/huangzizhe/Manus_L20_retarget-main/src/manus_ros2/calibration/Calibration_right.mcal
```

保持这个终端运行，再开另一个终端采集 retarget YAML。

### 推荐：一键合并采集

现在推荐使用 `calibration_capture all`。它把重复姿态融合后，只需要采 5 次姿势，同时生成四类当前运行时使用的 YAML：

- `finger_flexion_ergonomics_right/left_calibration.yaml`: 四指 root/tip 的 ergonomics 弯曲标定。
- `finger_yaw_ergonomics_right/left_calibration.yaml`: 四指 yaw 的 ergonomics 标定。
- `thumb_right/left_flexion_ergonomics_mapping.yaml`: 拇指 root/tip 的 ergonomics 弯曲标定。
- `thumb_segment_frame_right/left.yaml`: 拇指 segment IK 双姿态 frame。

5 个姿势含义：

- `natural_open`: 自然张开，同时作为四指 open、四指 ergonomics yaw open、拇指弯曲 open、拇指 IK open。
- `four_finger_fist`: 四指完全弯曲握拳，只用于四指弯曲 closed。
- `finger_close`: 四指并拢，只用于四指 ergonomics yaw close。
- `finger_spread`: 四指外展，只用于四指 ergonomics yaw spread。
- `thumb_pinky_root_touch`: 大拇指触碰小拇指指根，同时用于拇指弯曲 touch、拇指 IK touch。

右手一键采集：

```bash
source /opt/ros/humble/setup.bash
cd /home/huangzizhe/Manus_L20_retarget-main
source /home/huangzizhe/Manus_L20_retarget-main/install/setup.bash

ros2 run manus_l20_retarget calibration_capture all \
  --hand right \
  --glove-topic /manus_glove_0 \
  --duration 2.0
```

左手一键采集。如果左手手套发布在 `/manus_glove_1`，使用下面命令；如果当前只有一只左手手套并发布在 `/manus_glove_0`，把 `/manus_glove_1` 改成 `/manus_glove_0`：

```bash
ros2 run manus_l20_retarget calibration_capture all \
  --hand left \
  --glove-topic /manus_glove_1 \
  --duration 2.0
```

注意：

- 采集命令会覆盖对应 YAML 里的 MANUS 标定数据。
- `thumb_segment_frame_*.yaml` 里的 L20 `robot_open_command` / `robot_touch_command` 会优先保留已有文件里的值，不会因为重新采 MANUS 手势就覆盖手动调好的 L20 参考姿态。
- `--hand left` 默认会使用 `left_glove_to_right_retarget`；右手默认使用 `right_glove_to_right_retarget`。
- 采集时不需要启动 L20 真机，只需要 MANUS 数据话题正常发布。
- 四指 yaw 固定由 MANUS ergonomics 的 `IndexSpread`、`MiddleSpread`、`RingSpread`、`PinkySpread` 映射到 L20 槽位 6-9；其三组姿势已经包含在一键采集流程中。
- 四指和拇指 root/tip 弯曲固定使用 ergonomics 标定，不再回退到旧的骨骼几何弯曲路径。

采集完成后确认 YAML 文件存在：

```bash
ls -lh /home/huangzizhe/Manus_L20_retarget-main/src/manus_l20_retarget/config/finger_*_ergonomics_*_calibration.yaml
ls -lh /home/huangzizhe/Manus_L20_retarget-main/src/manus_l20_retarget/config/thumb_*_flexion_ergonomics_mapping.yaml
ls -lh /home/huangzizhe/Manus_L20_retarget-main/src/manus_l20_retarget/config/thumb_segment_frame_*.yaml
```

### 可选：指尖接触语义标定

该功能识别四类人手接触：拇指指尖分别接触食指、中指、无名指和小指指尖。它不改变四指 ergonomics yaw，也不替代拇指两姿势 segment IK。

每帧从 `ManusGlove.raw_nodes` 取 `Thumb.TIP` 和四个 `Finger.TIP`，计算：

```text
distance_ratio = ||Thumb.TIP - Finger.TIP|| / ||Index.MCP - Pinky.MCP||
```

分母是掌宽，因此不同佩戴距离和左右手镜像不会改变阈值尺度。采集命令只订阅 MANUS 话题，不会启动或发送 L20 真机命令：

```bash
ros2 run manus_l20_retarget calibration_capture fingertip-contact \
  --hand right \
  --glove-topic /manus_glove_0 \
  --duration 2.0
```

左手把 `--hand` 改成 `left`，并使用对应 MANUS 话题。工具依次采集自然张开、拇指-食指指尖、拇指-中指指尖、拇指-无名指指尖、拇指-小指指尖五个姿势，生成：

```text
fingertip_contact_semantics_right.yaml
fingertip_contact_semantics_left.yaml
```

该 YAML 内每对接触有两种独立信息：

- `human`：自然张开距离 `natural_open_distance_p05_ratio` 与接触距离 `contact_distance_p95_ratio`。
- `robot`：对应 L20 的 20-slot `contact_command`、本动作实际覆盖的 slot 和最大修正量。

`runtime.takeover_start_progress` 是从自然张开向接触移动的比例。默认 `0.35` 表示距离完成该段行程的 35% 后开始接管；继续接近时，接管比例线性增加；达到接触距离时，完全接管当前接触动作。每个动作只覆盖拇指 Root/Tip（slot `0/15`）、当前接触手指 Root/Tip（slot `1/16`、`2/17`、`3/18` 或 `4/19`）以及拇指 Roll/Yaw（slot `5/10`）；其他手指的弯曲和所有四指 yaw 继续使用实时遥操值。

采集会保留已有的 `contact_command`，但不会自动得到真机接触姿态。先在原有连续遥操模式下手动验证每个接触姿态，将确认后的 20-slot 命令填入对应 `contact_command`；每一个数字都保留 slot 注释。随后把 YAML 的 `runtime.enabled` 改为 `true`。

Qt 指尖接触标定会保留已有接触姿态，但会将 `runtime.enabled` 设为 `false`。确认真机接触姿态后，将左右 YAML 中的 `runtime.enabled` 改为 `true`。省略 `full_takeover_progress`、`firm_contact_enter_activation`、`release_start_progress_delta` 的旧 YAML 仍可运行，程序会使用 `0.75`、`0.90`、`0.06` 的默认值；需要明确固化参数时可按语义文档中的示例补上。随后在启动命令中显式打开语义接触：

```bash
enable_fingertip_contact_semantics:=true \
fingertip_contact_debug:=true
```

接触在起始区间内稳定后，上述局部 slot 会随指尖接近程度平滑增强；松开并离开起始区间后恢复当前连续控制链路。

## 6. 启动双手遥操作

确认已完成：

- 手套已标定。
- `Calibration_left.mcal` 和 `Calibration_right.mcal` 已存在。
- 人手到 L20 映射 YAML 已存在。
- `can0` 和 `can1` 已连接并 up。
- 机械手周围安全。

启动双手遥操作：

```bash
cd /home/huangzizhe/Manus_L20_retarget-main
source /home/huangzizhe/Manus_L20_retarget-main/install/setup.bash

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

语义接触的默认相位参数为：

```text
fingertip_contact_close_orientation_completion:=0.25
fingertip_contact_close_flexion_start:=0.40
fingertip_contact_release_flexion_open_completion:=0.65
fingertip_contact_release_orientation_gamma:=2.5
```

其中 `0.65` 是当前 pick 算法迁移后的释放参数；不要继续使用旧文档中的 `0.18`。

启动后观察终端日志，确认能看到：

- MANUS publisher started / connected。
- Loaded glove calibration。
- `/manus_glove_0`、`/manus_glove_1` 有数据。
- LinkerHand driver 正常连接 `can0`、`can1`。

## 7. 双手触觉遥操作

在左右 L20 都支持触觉读取、CAN 接口均正常且 MANUS 手套编号为右 `0`、左 `1` 时，在上面的双手命令中增加：

```bash
  enable_haptics:=true \
  mock_tactile:=false \
  right_haptic_glove_id:=0 \
  left_haptic_glove_id:=1 \
  haptic_poll_rate_hz:=30.0 \
  haptic_read_mode:=auto \
  haptic_normal_force_full_scale:=100.0
```

完整触觉启动命令：

```bash
cd /home/huangzizhe/Manus_L20_retarget-main
source /opt/ros/humble/setup.bash
source /home/huangzizhe/Manus_L20_retarget-main/install/setup.bash

ros2 launch bringup manus_l20_linkerhand_g20.launch.py \
  start_manus:=true \
  right_can:=can0 \
  left_can:=can1 \
  is_touch:=false \
  right_input_topic:=/manus_glove_0 \
  left_input_topic:=/manus_glove_1 \
  enable_haptics:=true \
  mock_tactile:=false \
  right_haptic_glove_id:=0 \
  left_haptic_glove_id:=1 \
  haptic_poll_rate_hz:=30.0 \
  haptic_read_mode:=auto \
  haptic_normal_force_full_scale:=100.0 \
  enable_fingertip_contact_semantics:=false \
  fingertip_contact_debug:=true \
  thumb_ik_debug:=true
```

触觉链路为：

```text
右 L20 can0 -> /manus_l20_haptics/right/force -> /manus_glove_0/vibration_cmd
左 L20 can1 -> /manus_l20_haptics/left/force  -> /manus_glove_1/vibration_cmd
```

没有触觉硬件时可临时使用 `mock_tactile:=true` 做软件链路检查；它不会产生真实力反馈。

## 8. 常用检查命令

查看 MANUS 数据：

```bash
ros2 topic hz /manus_glove_0
ros2 topic hz /manus_glove_1
```

查看右手控制命令：

```bash
ros2 topic echo /cb_right_hand_control_cmd
```

查看左手控制命令：

```bash
ros2 topic echo /cb_left_hand_control_cmd
```

查看所有相关节点：

```bash
ros2 node list
```

查看所有相关话题：

```bash
ros2 topic list | grep -E "manus|cb_"
```

## 9. 常见问题

### 找不到 GLFW/glfw3.h

安装 GUI 编译依赖：

```bash
sudo apt-get update
sudo apt-get install -y build-essential libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

### 标定 GUI 可以打开，但显示没有手套

检查：

- 手套是否上电。
- MANUS 接收器是否插好。
- 手套是否已配对。
- 当前用户是否有 USB 设备权限。

### 启动遥操作后左右手反了

交换输入话题：

```bash
ros2 launch bringup manus_l20_linkerhand_g20.launch.py \
  start_manus:=true \
  right_can:=can0 \
  left_can:=can1 \
  right_input_topic:=/manus_glove_1 \
  left_input_topic:=/manus_glove_0
```

### 不想加载手套标定文件

临时禁用 `.mcal` 自动加载：

```bash
ros2 launch bringup manus_l20_linkerhand_g20.launch.py \
  start_manus:=true \
  load_manus_calibration:=false
```

### CAN 口不存在

先确认硬件是否识别：

```bash
ip link
lsusb
```

如果设备名不是 `can0` / `can1`，按实际名称修改 `right_can` 和 `left_can`。

如果 launch 日志出现 `Could not access SocketCAN device can1` 或 `OSError: [Errno 19] No such device`，表示系统中根本没有 `can1`，左手驱动和左手触觉节点都会退出。检查第二个 USB-CAN 转接器、线缆和驱动，确认 `ip -br link show type can` 同时列出 `can0`、`can1` 后，再启动双手命令。

## 10. 停止流程

1. 先松开手套动作，保持机械手在安全姿态。
2. 在遥操作 launch 终端按 `Ctrl+C`。
3. 如需关闭 CAN：

```bash
sudo ip link set can0 down
sudo ip link set can1 down
```
