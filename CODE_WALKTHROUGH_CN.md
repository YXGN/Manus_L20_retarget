# MANUS L20 项目代码逐句导读

这份文档的目标不是替代源码，而是给你一个“拿着源码逐句读”的导航。这个工作区大约有 3.7 万行代码、配置和消息定义；真正做到每一句都懂，建议按本文的顺序分阶段读。第一阶段先读主运行链路，后面再读硬件驱动、拇指 IK 支撑库、触觉反馈和调试工具。

## 先记住这条数据流

```text
MANUS 手套硬件
  -> manus_ros2/manus_data_publisher
  -> /manus_glove_0, /manus_glove_1
  -> manus_l20_retarget/manus_l20_retarget_node
  -> /cb_right_hand_control_cmd, /cb_left_hand_control_cmd
  -> linker_hand_ros2_sdk/linker_hand_advanced_g20
  -> can0/can1
  -> LinkerHand L20 真机
```

每个箭头都对应一组代码：

- `src/manus_ros2`: C++ MANUS SDK 客户端，把手套骨架和 ergonomics 数据发布成 ROS2 topic。
- `src/manus_ros2_msgs`: 自定义 ROS2 消息，定义 MANUS 数据长什么样。
- `src/bringup`: launch 管线，负责把各节点启动起来，并把参数传进去。
- `src/manus_l20_retarget`: 核心重定向逻辑，把 MANUS skeleton 转成 L20 的 20 槽命令。
- `src/l20_thumb_ik`: 内部拇指 IK 支撑库，主要服务于 L20 拇指 roll/yaw 的 segment IK。
- `src/linker_hand_ros2_sdk`: 厂商 LinkerHand 驱动，订阅 20 槽命令并写 CAN。
- `src/manus_l20_haptics`: 可选触觉反馈，从 L20 触觉数据映射到 MANUS 手套震动。
- `src/manus_l20_revo_style`: 另一条 Revo 风格的实验性重定向路径，不是主链路。

## 第一轮阅读顺序

按这个顺序读，脑子最不容易乱：

1. `src/manus_ros2_msgs/msg/*.msg`: 先理解 ROS 消息字段。
2. `src/bringup/launch/*.launch.py`: 看命令行启动入口如何进入公共 pipeline。
3. `src/bringup/launch/manus_l20_common/pipeline.py`: 看启动参数、节点、topic、CAN 参数如何串起来。
4. `src/manus_l20_retarget/manus_l20_retarget/retarget_pipeline.py`: 看 feature 提取、四指 flexion target、L20 command adapter 和最终滤波。
5. `src/manus_l20_retarget/manus_l20_retarget/manus_l20_retarget_node.py`: 看 ROS 节点如何加载参数、组合 yaw/拇指 IK，并发布 20 个 L20 命令值。
6. `src/manus_l20_retarget/manus_l20_retarget/manus_landmarks.py`: 看 MANUS raw nodes 如何转成 MediaPipe 风格 21 个 landmark。
7. `src/manus_l20_retarget/manus_l20_retarget/mapping.py`: 看基础数值映射工具。
8. `src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/linker_hand_advanced_g20.py`: 看命令如何发给硬件。

## ROS 消息定义逐句读法

`src/manus_ros2_msgs/msg/ManusRawNode.msg`:

```text
int32 node_id
int32 parent_node_id
string joint_type
string chain_type
geometry_msgs/Pose pose
```

- `node_id`: MANUS SDK 给某个骨架节点的编号。
- `parent_node_id`: 父节点编号，用来表达骨架树的父子关系。
- `joint_type`: 关节类型，例如 `MCP`、`PIP`、`DIP`、`TIP`。
- `chain_type`: 属于哪根手指或手部链，例如 `Thumb`、`Index`、`Middle`、`Ring`、`Pinky`、`Hand`。
- `pose`: 这个节点的位置和姿态，包含 `position` 和 `orientation`。

`src/manus_ros2_msgs/msg/ManusGlove.msg`:

```text
int32 glove_id
string side
int32 raw_node_count
ManusRawNode[] raw_nodes
int32 ergonomics_count
ManusErgonomics[] ergonomics
geometry_msgs/Quaternion raw_sensor_orientation
int32 raw_sensor_count
geometry_msgs/Pose[] raw_sensor
```

- `glove_id`: 手套编号，通常决定发布到 `/manus_glove_0` 还是 `/manus_glove_1`。
- `side`: MANUS 识别出的左右手，常见值是 `left` 或 `right`。
- `raw_node_count`: `raw_nodes` 数量，便于检查数据完整性。
- `raw_nodes`: 原始骨架节点数组，是主重定向节点最关心的数据。
- `ergonomics_count`: `ergonomics` 数量。
- `ergonomics`: MANUS SDK 算好的工效学指标，这个项目主链路更多使用 raw skeleton。
- `raw_sensor_orientation`: 原始传感器姿态。
- `raw_sensor_count`: `raw_sensor` 数量。
- `raw_sensor`: 原始传感器 pose 数组。

`src/manus_ros2_msgs/msg/ManusErgonomics.msg`:

```text
string type
float32 value
```

- `type`: ergonomics 指标名。
- `value`: 该指标的浮点值。

`src/manus_ros2_msgs/msg/ManusVibrationCommand.msg`:

```text
float32[5] intensities
```

- 固定 5 个浮点强度，顺序是 Thumb、Index、Middle、Ring、Pinky。
- 值通常按 `0.0` 到 `1.0` 理解，发给 MANUS 手套震动马达。

## Launch 入口逐句读法

几个 `src/bringup/launch/manus_l20_linkerhand_*.launch.py` 入口长得几乎一样：

```python
from pathlib import Path
import importlib.util
```

- `Path` 用来拼文件路径。
- `importlib.util` 用来从一个明确路径动态加载 `pipeline.py`。

```python
def _load_generate_manus_l20_launch():
```

- 定义一个内部 helper 函数。
- 函数名以下划线开头，表示它不是给外部直接调用的公开 API。

```python
helper_path = Path(__file__).resolve().parent / "manus_l20_common" / "pipeline.py"
```

- `__file__` 是当前 launch 文件路径。
- `.resolve()` 转成绝对路径。
- `.parent` 取所在目录。
- `/ "manus_l20_common" / "pipeline.py"` 是 `pathlib` 的路径拼接写法。

```python
spec = importlib.util.spec_from_file_location("manus_l20_common_pipeline", helper_path)
```

- 通过文件路径创建一个 Python 模块加载规格。
- `"manus_l20_common_pipeline"` 是临时模块名。

```python
if spec is None or spec.loader is None:
    raise RuntimeError(f"cannot load launch helper: {helper_path}")
```

- 如果模块加载规格无效，就主动报错。
- `raise RuntimeError(...)` 会让 launch 失败，并显示清楚原因。

```python
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
return module.generate_manus_l20_launch
```

- `module_from_spec` 创建模块对象。
- `exec_module` 真正执行 `pipeline.py`。
- 返回公共 pipeline 里的 launch 工厂函数。

```python
def generate_launch_description():
```

- ROS2 launch 系统约定的入口函数。
- `ros2 launch` 会调用它拿到 `LaunchDescription`。

右手入口最后调用：

```python
return generate_manus_l20_launch(
    hand_type="right",
    retarget_node_name="manus_l20_retarget_right",
)
```

- `hand_type="right"` 表示物理驱动 topic 和节点按右手命名。
- `retarget_node_name` 是 ROS 节点名。

左手入口类似，只是 `hand_type="left"`、节点名变成 `manus_l20_retarget_left`。

双手入口调用的是：

```python
return generate_manus_l20_bimanual_launch()
```

- 这个函数会一次启动左右两套 retarget 节点和左右两套 LinkerHand 驱动。

## `pipeline.py` 核心结构

这个文件的职责是“把命令行参数变成 ROS2 节点启动描述”。

### 路径定位

```python
def _workspace_root() -> Path:
```

- 返回当前工作区根目录。
- 优先读环境变量 `MANUS_L20_ROOT`。
- 如果环境变量没有设置，就从当前文件路径的父目录一路往上找。

```python
env_root = os.environ.get("MANUS_L20_ROOT", "").strip()
```

- 从环境变量拿路径。
- 没有就用空字符串。
- `.strip()` 去掉首尾空白。

```python
if env_root:
    return Path(env_root).expanduser().resolve()
```

- 如果环境变量非空，就把它转成绝对路径返回。
- `expanduser()` 支持 `~`。
- `resolve()` 解析成标准绝对路径。

后面的循环检查两个关键目录是否存在：

```python
src/l20_thumb_ik
src/manus_l20_retarget
```

这两个目录都存在，基本就能确认找到了工作区根。

### 配置文件 helper

```python
def _config_file(name: str) -> str:
    return str(_SRC / "manus_l20_retarget" / "config" / name)
```

- 输入一个 YAML 文件名。
- 输出它在 `manus_l20_retarget/config` 里的绝对路径字符串。
- launch 参数最终需要字符串，所以这里 `str(...)`。

```python
def _flexion_config_name(hand_type: str) -> str:
    return "flexion_left_calibration.yaml" if hand_type == "left" else "flexion_right_calibration.yaml"
```

- 根据左右手选择四指弯曲标定文件。
- 这是 Python 的条件表达式。

`_finger_yaw_config_name` 和 `_thumb_flexion_config_name` 逻辑一样，只是选 yaw 和拇指弯曲配置。

### `_hand_actions`

`_hand_actions(...)` 是整个 launch 管线最重要的函数之一。它接收一大批参数，返回三个动作：

1. 启动 `manus_l20_retarget_node`。
2. 条件启动触觉采集节点 `tactile_source_node`。
3. 条件启动触觉反馈节点 `haptic_feedback_node`。
4. 启动 LinkerHand 硬件驱动进程 `linker_hand_advanced_g20`。

主重定向节点部分：

```python
Node(
    package="manus_l20_retarget",
    executable="manus_l20_retarget_node",
    name=retarget_node_name,
    output="screen",
    parameters=[{...}],
)
```

- `package`: ROS2 包名。
- `executable`: `setup.py` 里注册的 console script 名。
- `name`: ROS 节点名。
- `output="screen"`: 日志打印到终端。
- `parameters`: 传给节点的 ROS 参数字典。

关键参数含义：

- `input_topic`: 节点订阅的 MANUS topic，例如 `/manus_glove_0`。
- `command_topic`: 节点发布的 L20 命令 topic，例如 `/cb_right_hand_control_cmd`。
- `hand_side`: retarget 逻辑用哪只手的几何语义。
- `mapping_mode`: 映射模式，默认 `landmark_flexion`。
- `publish_rate_hz`: 定时发布频率。
- `max_delta_per_cycle`: 每次命令最大变化量，用于限速。
- `lowpass_alpha`: 低通滤波系数。
- `landmark_transform`: MANUS 坐标系转内部 canonical 坐标系的矩阵名。
- `flexion_calibration_path`: 四指弯曲标定 YAML。
- `finger_yaw_calibration_path`: 四指横摆标定 YAML。
- `thumb_flexion_mapping_path`: 拇指弯曲标定 YAML。
- `enable_thumb_ik`: 是否启用拇指 segment IK。
- `l20_thumb_ik_root`: 内部 IK 库根路径。
- `l20_thumb_ik_config_path`: L20 MuJoCo/IK 配置 YAML。
- `linkerhand_sdk_root`: LinkerHand SDK 映射工具路径。

硬件驱动部分：

```python
ExecuteProcess(
    cmd=[
        "ros2",
        "run",
        "linker_hand_ros2_sdk",
        "linker_hand_advanced_g20",
        "--hand_type",
        hand_type,
        "--can",
        can,
        ...
    ],
    output="screen",
)
```

- 这里没有用 `Node`，而是直接执行一条命令。
- 等价于终端里运行 `ros2 run linker_hand_ros2_sdk linker_hand_advanced_g20 ...`。
- `--hand_type` 决定驱动订阅 `/cb_right_hand_control_cmd` 还是 `/cb_left_hand_control_cmd`。
- `--can` 决定用 `can0`、`can1` 还是别的 CAN 口。

### 单手 launch

`generate_manus_l20_launch(...)` 用于右手或左手单独启动。

一个很关键的设计：

```python
retarget_type = "right"
landmark_transform_default = "left_glove_to_right_retarget" if logic_type == "left" else "right_glove_to_right_retarget"
```

- 当前左手运行时刻意复用右手重定向逻辑。
- 左手 MANUS 数据会通过 `left_glove_to_right_retarget` 做坐标轴翻转：让左手的手指外展、拇指方向等几何关系，在算法看来像右手 canonical 几何。
- 这样不是把左手硬件当成右手硬件，而是复用已验证的右手角度特征、标定和拇指 IK 公式。

`DeclareLaunchArgument(...)` 表示声明一个可从命令行覆盖的参数。例如：

```python
DeclareLaunchArgument("start_manus", default_value="false")
```

意思是 launch 默认不启动 MANUS publisher。你可以这样覆盖：

```bash
ros2 launch bringup manus_l20_linkerhand_g20_right.launch.py start_manus:=true
```

`LaunchConfiguration("start_manus")` 表示稍后读取这个 launch 参数。

### 双手 launch

`generate_manus_l20_bimanual_launch()` 会声明右手和左手两套参数：

- `right_can`, `left_can`
- `right_input_topic`, `left_input_topic`
- `right_landmark_transform`, `left_landmark_transform`
- `right_haptic_*`, `left_haptic_*`

内部小函数 `hand_actions(...)` 只是为了避免左右手代码重复。它最后仍然调用 `_hand_actions(...)`。

## 主重定向节点总览

核心文件是：

```text
src/manus_l20_retarget/manus_l20_retarget/manus_l20_retarget_node.py
```

可以把它分成 8 层：

1. import 和常量。
2. `ManusL20RetargetNode.__init__`: 声明参数、读取参数、加载配置、创建 publisher/subscriber/timer。
3. 标定 YAML 加载函数。
4. MANUS 消息回调和定时器。
5. `ManusGlove -> landmarks -> command` 的主映射。
6. 拇指 IK 和 debug。
7. 命令滤波与发布。
8. 文件末尾数学 helper 和 IK helper class。

### import 区

```python
from __future__ import annotations
```

- 让类型注解延迟求值。
- 好处是可以写 `ManusGlove | None` 这种注解，同时减少循环引用问题。

```python
import sys
import os
import threading
import math
from pathlib import Path
from time import monotonic
from typing import Any
```

- `sys`: 用来动态添加 `l20_thumb_ik/src` 到 Python import 路径。
- `os`: 读取环境变量。
- `threading`: 用锁保护最新 MANUS 消息。
- `math`: 角度、三角函数。
- `Path`: 路径处理。
- `monotonic`: 单调时钟，用于 watchdog 和 debug 节流。
- `Any`: 类型注解，表示任意类型。

```python
import numpy as np
import rclpy
import yaml
```

- `numpy`: 3D 点、向量、矩阵运算。
- `rclpy`: ROS2 Python 客户端库。
- `yaml`: 读取标定配置。

```python
from manus_ros2_msgs.msg import ManusGlove
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
```

- `ManusGlove`: 输入消息。
- `JointState`: 输出命令消息。这个项目把 20 个 L20 命令值放在 `position` 数组里。
- `Bool`: 急停 topic `/l20/estop` 的消息类型。

本地导入：

```python
from .mapping import clamp_u8
from .manus_landmarks import (...)
```

- `clamp_u8`: 把数值限制到 0 到 255 的整数范围。
- `manus_landmarks` 里的函数负责 MANUS raw skeleton 到内部 landmark 特征。

### 常量区

`STANDARD_OPEN_COMMAND` 是 L20 张开手势的 20 槽命令。`STANDARD_FIST_COMMAND` 是握拳命令。

槽位的含义由 `_publish()` 里的 `msg.name` 给出：

```text
0  Thumb Base
1  Index Finger Base
2  Middle Finger Base
3  Ring Finger Base
4  Pinky Finger Base
5  Thumb Roll
6  Index Finger Yaw
7  Middle Finger Yaw
8  Ring Finger Yaw
9  Pinky Finger Yaw
10 Thumb Yaw
11 Reserved
12 Reserved
13 Reserved
14 Reserved
15 Thumb Tip
16 Index Finger Tip
17 Middle Finger Tip
18 Ring Finger Tip
19 Pinky Finger Tip
```

记住这个槽位表非常重要。后面所有 `command[index] = ...` 都是在写这些槽位。

`DEFAULT_ROOT_OPEN_RAD`、`DEFAULT_ROOT_CLOSED_RAD`、`DEFAULT_TIP_OPEN_RAD`、`DEFAULT_TIP_CLOSED_RAD` 是默认角度标定：

- root 指根弯曲。
- tip 指尖弯曲。
- open 是张开时角度。
- closed 是闭合时角度。
- 单位是弧度。

`THUMB_COMMAND_SLOTS = (0, 5, 10, 15)` 表示拇指相关槽位。

`THUMB_IK_COMMAND_SLOTS = (5, 10)` 表示拇指 segment IK 只直接改 `Thumb Roll` 和 `Thumb Yaw`。

### `__init__` 做了什么

`__init__` 是节点启动时执行的初始化逻辑。

```python
super().__init__("manus_l20_retarget")
```

- 调用 ROS2 Node 父类初始化。
- 默认节点名是 `manus_l20_retarget`。
- launch 里可以通过 `name=...` 改实际节点名。

```python
workspace = _default_workspace_root()
```

- 找工作区根目录。
- 这保证节点从 `install/` 运行时也能找到 `src/l20_thumb_ik`。

大量 `declare_parameter(...)` 是在声明 ROS 参数和默认值。它们分几类：

- topic 参数：`input_topic`、`command_topic`。
- 路径参数：`l20_thumb_ik_root`、`l20_thumb_ik_config_path`、`linkerhand_sdk_root`。
- 安全滤波参数：`publish_rate_hz`、`max_delta_per_cycle`、`lowpass_alpha`、`watchdog_timeout_sec`。
- 基准命令参数：`neutral_command`、`closed_command`、`reserved_command`。
- 四指弯曲参数：`root_flexion_open_rad`、`root_flexion_closed_rad`、`tip_flexion_open_rad`、`tip_flexion_closed_rad`。
- yaw 参数：`enable_finger_yaw`、`enable_finger_yaw_mapping`、`finger_yaw_calibration_path`。
- 拇指参数：`enable_thumb_flexion_mapping`、`enable_thumb_ik`、`thumb_segment_*`。
- landmark 参数：`landmark_transform`、`wrist_mode`、`distal_mode`。

`self._lock`、`self._latest_msg`、`self._last_msg_time` 是实时数据缓存：

- subscription 回调只保存最新 MANUS 消息。
- timer 按固定频率取最新消息计算命令。
- 锁用于避免回调和 timer 同时读写变量。

加载配置时有三类 YAML：

- `_apply_flexion_calibration_path`: 四指弯曲标定。
- `_apply_finger_yaw_calibration_path`: 四指 yaw 标定。
- `_apply_thumb_flexion_mapping_path`: 拇指弯曲标定。

创建 ROS 通信：

```python
self._command_pub = self.create_publisher(JointState, self.get_parameter("command_topic").value, 10)
self.create_subscription(ManusGlove, self.get_parameter("input_topic").value, self._on_glove, 1)
self.create_subscription(Bool, "/l20/estop", self._on_estop, 1)
self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._on_timer)
```

- 发布 `JointState` 到命令 topic。
- 订阅 MANUS 手套 topic。
- 订阅急停 topic。
- 创建定时器，以固定频率执行 `_on_timer`。

### Timer 主循环

`_on_glove` 很短：

```python
with self._lock:
    self._latest_msg = msg
    self._last_msg_time = monotonic()
```

- 收到消息就缓存。
- 记录收到消息的时间。

`_on_timer` 是主循环：

1. 取最新 MANUS 消息。
2. 如果急停，发布张开安全命令。
3. 如果没有消息，什么都不做。
4. 如果消息超时，发布 neutral 命令。
5. 调 `_command_from_manus(msg)` 算原始命令。
6. 调 `_filter_command(command)` 做限速和低通。
7. 发布最终命令。

异常处理：

```python
except Exception as exc:
    self.get_logger().warning(f"failed to retarget MANUS frame: {exc}")
    return
```

- 某一帧转换失败时只打 warning。
- 节点不会因为一帧坏数据退出。

### MANUS 到命令

入口是 `_command_from_manus`：

```python
landmarks = manus_raw_nodes_to_mediapipe_landmarks(...)
```

- 把 MANUS raw nodes 转成 21 个 landmark。
- 输出是形状为 `(21, 3)` 的 numpy 数组。

```python
if self._hand_side_override == "auto":
    hand_side = "right" if str(msg.side).lower() != "left" else "left"
else:
    hand_side = self._hand_side_override
```

- 如果参数是 `auto`，就根据消息里的 `side` 判断左右手。
- 否则强制用 launch 传入的 `hand_side`。

```python
if self._mapping_mode != "l20_ik":
    return self._command_from_landmark_flexion(landmarks, hand_side, msg.raw_nodes)
```

- 默认不是全手 IK，而是 landmark flexion 映射。
- 这是当前主链路。

`_command_from_landmark_flexion` 的核心：

```python
command = list(self._neutral_command)
```

- 先从张开手势命令复制一份。
- 后面只覆盖需要控制的槽位。

四指循环：

```python
for finger_index, (mcp, pip, dip, tip) in enumerate(FINGER_LANDMARKS):
    if finger_index == 0:
        continue
```

- `FINGER_LANDMARKS` 包含拇指、食指、中指、无名指、小指的 landmark 索引。
- `finger_index == 0` 是拇指。
- 四指弯曲循环跳过拇指，因为拇指有单独逻辑。

角度计算：

```python
root_angle = _joint_flexion_rad(landmarks[mcp], landmarks[pip], landmarks[dip])
tip_angle = _joint_flexion_rad(landmarks[pip], landmarks[dip], landmarks[tip])
```

- root 角度用 MCP、PIP、DIP 三点算。
- tip 角度用 PIP、DIP、TIP 三点算。

归一化：

```python
root_amount = _normalized_angle(root_angle, open_rad, closed_rad, gamma)
```

- 把实际角度映射到 `0.0` 到 `1.0`。
- `0.0` 表示接近 open。
- `1.0` 表示接近 closed。
- `gamma` 用来改变响应曲线。

命令插值：

```python
command[finger_index] = _lerp_command(open_cmd, closed_cmd, root_amount)
command[15 + finger_index] = _lerp_command(open_cmd, closed_cmd, tip_amount)
```

- 指根写槽位 `1..4`。
- 指尖写槽位 `16..19`。
- `_lerp_command` 在 open 命令和 closed 命令之间线性插值。

四指后处理：

- `_apply_finger_yaw`: 写槽位 `6..9`。
- `_apply_thumb_flexion_mapping`: 写槽位 `0` 和 `15`。
- `_apply_thumb_pose`: 可选用简单 yaw/roll 特征写槽位 `5` 和 `10`。
- `_apply_thumb_ik`: 用 segment IK 写槽位 `5` 和 `10`。
- `for index in range(11, 15)`: 保留槽位统一写 reserved 值。
- `_apply_neutral_locks`: 把被锁定的槽位恢复 neutral。

### 命令滤波与发布

`_filter_command` 做两件事：

1. `clamp_u8`: 每个槽位限制在 `0..255`。
2. rate limit: 每次最多变 `max_delta_per_cycle`。
3. lowpass: 用 `lowpass_alpha` 在上次命令和当前命令之间平滑。

`_publish` 把 `list[int]` 变成 `sensor_msgs/JointState`：

```python
msg.position = [float(value) for value in command]
```

- L20 命令值本质是 0 到 255 的整数。
- 这里放进 `JointState.position`，所以转成 float。
- 下游驱动读取这些 position 值再发 CAN。

## `manus_landmarks.py` 怎么读

这个文件负责“MANUS skeleton 坐标清洗”。

核心常量：

```python
FINGER_CHAINS = ("Thumb", "Index", "Middle", "Ring", "Pinky")
JOINT_ORDER = ("MCP", "PIP", "IP", "DIP", "TIP")
```

- MANUS raw node 里每个点有 `chain_type` 和 `joint_type`。
- 代码按手指链和关节顺序把点排好。

`FINGER_SLOTS` 把每根手指映射到 MediaPipe 风格 21 点里的槽位：

- 拇指: `1,2,3,4`
- 食指: `5,6,7,8`
- 中指: `9,10,11,12`
- 无名指: `13,14,15,16`
- 小指: `17,18,19,20`
- `0` 留给 wrist，由代码估算。

`TRANSFORMS` 是坐标系转换矩阵：

- `identity`: 不变。
- `right_glove_to_right_retarget`: 当前右手默认 canonical 转换。
- `left_glove_to_right_retarget`: 左手坐标轴翻转到右手 retarget 算法使用的 canonical 几何空间。

`manus_raw_nodes_to_mediapipe_landmarks(...)` 主流程：

1. 检查 `distal_mode` 是否有效。
2. 按手指链收集 MANUS raw nodes。
3. 如果某些手指少于 3 个点，报错。
4. 创建 `(21, 3)` 零数组。
5. 每根手指选 4 个点填入对应槽位。
6. 估算 wrist 点填入 `landmarks[0]`。
7. 用 `TRANSFORMS[transform]` 做坐标转换。
8. 返回 landmarks。

`_joint_flexion_rad(a, b, c)` 的数学意思：

- 把 `b` 当关节中心。
- `a-b` 是上一段骨骼方向。
- `c-b` 是下一段骨骼方向。
- 两个向量夹角越接近 `pi`，手指越直。
- 函数返回 `pi - interior_angle`，所以手指越弯，返回值越大。

`_palm_frame(landmarks)` 构造手掌局部坐标系：

- `lateral`: 小指 MCP 到食指 MCP 的横向。
- `forward`: 四指 MCP 到 PIP 的平均前向，再去掉横向分量。
- `normal`: `lateral x forward` 得到掌面法向。

这个 palm frame 后面用于算四指 yaw 和拇指 yaw/roll。

## `mapping.py` 怎么读

`clamp_u8(value)`:

```python
return max(0, min(255, int(round(float(value)))))
```

- 先转 float。
- 四舍五入。
- 转 int。
- 小于 0 变 0，大于 255 变 255。

`ManusToL20Mapper` 是一个更通用的 ergonomics 映射器。当前主节点主要使用 `clamp_u8`，但这个类适合 Revo 风格或配置驱动映射。

当前主链路的最终命令滤波在 `retarget_pipeline.py` 的 `filter_l20_command(...)`：

- `max_delta_per_cycle` 限制单周期变化。
- `lowpass_alpha` 做低通。
- 返回值会统一裁剪到 `0..255`。

`mapping.py` 里还保留了更通用的 `ManusToL20Mapper` 和 `SafetyFilter`，主要适合配置驱动映射或实验链路，不是当前主节点最核心的阅读入口。

## 读代码时最容易混淆的词

- `landmark`: 手部关键点坐标。这里采用 MediaPipe 风格 21 点编号。
- `raw_node`: MANUS SDK 原始骨架节点，有 `chain_type`、`joint_type` 和 `pose`。
- `command`: L20 的 20 槽控制数组，值域是 `0..255`。
- `neutral_command`: 张开或自然打开时的命令。
- `closed_command`: 握拳或闭合时的命令。
- `root`: 指根弯曲，四指通常对应槽位 `1..4`。
- `tip`: 指尖弯曲，四指通常对应槽位 `16..19`。
- `yaw`: 手指左右摆动。
- `roll`: 拇指绕自身或局部轴滚转。
- `IK`: inverse kinematics，逆运动学。这里用目标向量求 L20 拇指关节命令。
- `watchdog`: 超时保护。如果一段时间没有新 MANUS 消息，就回 neutral。
- `lowpass`: 低通滤波，让命令变化更平滑。
- `rate limit`: 限制每次命令变化幅度，避免机械手突然大跳。

## 下一步逐行拆解建议

如果你想真正每一句都懂，建议下一轮从这两个文件开始：

```text
src/manus_l20_retarget/manus_l20_retarget/retarget_pipeline.py
src/manus_l20_retarget/manus_l20_retarget/manus_l20_retarget_node.py
```

分块顺序：

1. 先读 `retarget_pipeline.py` 第 1 到 100 行：20 槽命名、finger landmarks、feature/target 数据结构和 command adapter。
2. 再读 `retarget_pipeline.py` 第 101 到 203 行：MANUS feature 提取、四指 flexion target 计算、最终 command 滤波。
3. 再读 `retarget_pipeline.py` 第 206 到文件末尾：角度归一化、伸直保护、插值和命令缩放 helper。
4. 然后读 `manus_l20_retarget_node.py` 第 1 到 115 行：import、路径函数、默认命令和拇指相关常量。
5. 第 116 到 399 行：`__init__` 参数声明和节点初始化。
6. 第 400 到 835 行：YAML 标定加载、IK 初始化和拇指 segment frame 初始化。
7. 第 836 到 892 行：MANUS 输入如何进入主命令链路，四指 flexion 从 `retarget_pipeline.py` 接入。
8. 第 894 到 1230 行：四指 yaw、拇指弯曲、拇指 IK、debug 和拇指 smoothing。
9. 第 1232 到 1250 行：neutral lock、最终滤波和发布。
10. 第 1253 到文件末尾：参数解析、数学 helper、本地 IK helper class。

建议每次拆 150 到 250 行。这样既能逐句讲清楚，又不会把注意力打散。
