# MANUS 手套到 LinkerHand L20 遥操作流程

本文档面向第一次拿到 MANUS 手套和 LinkerHand L20 的操作者，流程从手套标定开始，到启动双手遥操作程序结束。

## 1. 准备环境

打开一个终端，进入工作区并加载 ROS 2 环境：

```bash
cd /home/huangzizhe/Manus_L20_retarget
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /home/huangzizhe/Manus_L20_retarget/install/setup.bash
```

如果只改过代码，后续一般只需要：

```bash
cd /home/huangzizhe/Manus_L20_retarget
source /home/huangzizhe/Manus_L20_retarget/install/setup.bash
```

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

进入标定工具目录：

```bash
cd /home/huangzizhe/Manus_L20_retarget/src/sharpa-manus-sdk-main/client/CalibrationGUI
./build.sh
./CalibrationGUI.out
```

GUI 打开后按以下流程操作：

1. 选择 `Left Glove` 或 `Right Glove`。
2. 点击 `Start Calibration`，或按 `F5`。
3. 按界面提示做手势。
4. 每一步完成后点击 `Next Step`，或按 `F9`。
5. 左右手都标定一遍。

标定完成后，程序会生成并自动同步以下文件：

```text
/home/huangzizhe/Manus_L20_retarget/src/manus_ros2/calibration/Calibration_left.mcal
/home/huangzizhe/Manus_L20_retarget/src/manus_ros2/calibration/Calibration_right.mcal
```

确认文件已生成：

```bash
ls -lh /home/huangzizhe/Manus_L20_retarget/src/manus_ros2/calibration/Calibration_*.mcal
```

这些 `.mcal` 文件会在启动 `manus_data_publisher` 时自动加载，不需要手动复制。

## 4. 标定后数据检查

新开一个终端：

```bash
cd /home/huangzizhe/Manus_L20_retarget
source /home/huangzizhe/Manus_L20_retarget/install/setup.bash
ros2 run manus_ros2 manus_data_publisher
```

再开一个终端查看话题：

```bash
cd /home/huangzizhe/Manus_L20_retarget
source /home/huangzizhe/Manus_L20_retarget/install/setup.bash
ros2 topic list | grep manus_glove
```

检查左右手套是否有数据：

```bash
ros2 topic hz /manus_glove_0
ros2 topic hz /manus_glove_1
```

如果只有一个手套，可能只会出现 `/manus_glove_0`。如果左右手对应反了，遥操作启动时交换 `right_input_topic` 和 `left_input_topic`。

检查完成后，在运行 `manus_data_publisher` 的终端按 `Ctrl+C` 退出。

## 5. 启动双手遥操作

确认已完成：

- 手套已标定。
- `Calibration_left.mcal` 和 `Calibration_right.mcal` 已存在。
- `can0` 和 `can1` 已连接并 up。
- 机械手周围安全。

启动双手遥操作：

```bash
cd /home/huangzizhe/Manus_L20_retarget
source /home/huangzizhe/Manus_L20_retarget/install/setup.bash

ros2 launch bringup manus_l20_linkerhand_g20.launch.py \
  start_manus:=true \
  right_can:=can0 \
  left_can:=can1 \
  is_touch:=false \
  right_input_topic:=/manus_glove_0 \
  left_input_topic:=/manus_glove_1 \
  thumb_ik_debug:=true
```

启动后观察终端日志，确认能看到：

- MANUS publisher started / connected。
- Loaded glove calibration。
- `/manus_glove_0`、`/manus_glove_1` 有数据。
- LinkerHand driver 正常连接 `can0`、`can1`。

## 6. 常用检查命令

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

## 7. 常见问题

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

## 8. 停止流程

1. 先松开手套动作，保持机械手在安全姿态。
2. 在遥操作 launch 终端按 `Ctrl+C`。
3. 如需关闭 CAN：

```bash
sudo ip link set can0 down
sudo ip link set can1 down
```
