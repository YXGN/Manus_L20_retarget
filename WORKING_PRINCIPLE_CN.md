# MANUS 到 L20 工作原理代码导读

这份只讲工作原理，不讲通信层。也就是说，下面默认你已经拿到一帧 MANUS 手部骨架数据，目标是理解代码怎样把它变成 L20 的 20 个控制值。

核心文件：

```text
src/manus_l20_retarget/manus_l20_retarget/manus_l20_retarget_node.py
src/manus_l20_retarget/manus_l20_retarget/retarget_pipeline.py
src/manus_l20_retarget/manus_l20_retarget/manus_landmarks.py
src/manus_l20_retarget/config/*.yaml
src/l20_thumb_ik/src/l20_ik_core/infrastructure/hand_model.py
src/l20_thumb_ik/src/l20_ik_core/infrastructure/controllers/adapters.py
```

## 一句话版本

代码先把 MANUS 原始骨架点整理成 21 个手部 landmark，然后：

1. 四指弯曲：用三点夹角算手指弯曲程度，再按标定 YAML 映射到 L20 指根和指尖槽位。
2. 四指横摆：用手指方向或 MANUS 关节姿态算 yaw，再按 open/close/spread 三个标定姿态插值到 yaw 槽位。
3. 拇指弯曲：用拇指三点夹角映射到 thumb base 和 thumb tip 槽位。
4. 拇指 roll/yaw：取 MANUS 拇指某段向量，对齐到 L20 拇指段向量，用 MuJoCo 雅可比迭代求 L20 `thumb_cmc_yaw` 和 `thumb_cmc_roll`。
5. 最后对 20 个命令值做限幅、限速、低通滤波。

## L20 20 槽命令

主节点所有算法最后都在写一个长度为 20 的数组：

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

你读代码时看到：

```python
command[index] = value
```

就回到这个表查 `index` 是哪一个关节含义。

当前主链路里最重要的槽位是：

- `1..4`: 四指指根弯曲。
- `16..19`: 四指指尖弯曲。
- `6..9`: 四指左右横摆。
- `0`: 拇指根部弯曲。
- `15`: 拇指指尖弯曲。
- `5`: 拇指 roll。
- `10`: 拇指 yaw。

## MANUS 原始点到 21 个 landmark

文件：`manus_landmarks.py`

MANUS 输入不是直接按 0 到 20 编好的点，而是一堆 `raw_nodes`。每个点有：

- `chain_type`: 属于哪根手指，例如 `Index`。
- `joint_type`: 属于哪个关节，例如 `MCP`、`PIP`。
- `pose.position`: 3D 位置。
- `pose.orientation`: 3D 姿态四元数。

代码用这几个常量整理顺序：

```python
FINGER_CHAINS = ("Thumb", "Index", "Middle", "Ring", "Pinky")
JOINT_ORDER = ("MCP", "PIP", "IP", "DIP", "TIP")
FINGER_SLOTS = {
    "Thumb": (1, 2, 3, 4),
    "Index": (5, 6, 7, 8),
    "Middle": (9, 10, 11, 12),
    "Ring": (13, 14, 15, 16),
    "Pinky": (17, 18, 19, 20),
}
```

意思是：

- 拇指填 landmark `1..4`。
- 食指填 landmark `5..8`。
- 中指填 landmark `9..12`。
- 无名指填 landmark `13..16`。
- 小指填 landmark `17..20`。
- landmark `0` 是 wrist，由代码估算。

核心入口：

```python
manus_raw_nodes_to_mediapipe_landmarks(...)
```

它做四件事：

1. `_ordered_chain`: 按 `MCP, PIP, IP, DIP, TIP` 排序每根手指。
2. `_select_four_landmarks`: 从 MANUS 可能提供的 5 个点里选 4 个点，适配 MediaPipe 风格。
3. `_estimate_wrist`: 根据四指 MCP 点和四指根部方向估算腕点。
4. `landmarks @ matrix.T`: 做坐标系转换。

这里最重要的不是通信，而是“坐标统一”。后续所有角度、向量、IK 都假设 landmarks 已经在统一手部空间里。

右手默认使用 `right_glove_to_right_retarget`。左手默认使用 `left_glove_to_right_retarget`，它的意思不是把左手机械手改成右手机械手，而是把左手 MANUS 坐标做一次轴翻转，让算法看到的几何关系和右手标定时一致。这样四指 yaw 的正负方向、拇指向掌心运动的方向、IK 目标向量方向，都能复用同一套右手 retarget 逻辑。

## 手掌局部坐标系

文件：`manus_landmarks.py`

函数：

```python
_palm_frame(landmarks)
```

它返回三个单位向量：

```text
lateral, forward, normal
```

含义：

- `lateral`: 手掌横向，大致从食指根到小指根。
- `forward`: 手指伸出去的方向，由四指 MCP 到 PIP 的平均方向得到。
- `normal`: 掌面法向，由 `lateral x forward` 算出。

这个局部坐标系是后面算 yaw/roll 的基础。因为直接用全局 x/y/z 会受手腕转动影响，而用手掌局部坐标系，关注的是“手指相对手掌怎么动”。

## 四指弯曲原理

主代码：`_command_from_landmark_flexion`

数学函数：`_joint_flexion_rad`

```python
def _joint_flexion_rad(a, b, c):
    first = a - b
    second = c - b
    cosine = dot(first, second) / (norm(first) * norm(second))
    interior_angle = acos(cosine)
    return pi - interior_angle
```

它的意义：

- `b` 是中间关节。
- `a-b` 是上一段骨头方向。
- `c-b` 是下一段骨头方向。
- 手指伸直时，内角接近 `pi`，所以返回 `pi - pi = 0`。
- 手指弯曲时，内角变小，所以返回值变大。

四指循环里：

```python
root_angle = _joint_flexion_rad(landmarks[mcp], landmarks[pip], landmarks[dip])
tip_angle = _joint_flexion_rad(landmarks[pip], landmarks[dip], landmarks[tip])
```

这表示：

- root 弯曲用 `MCP-PIP-DIP` 三点。
- tip 弯曲用 `PIP-DIP-TIP` 三点。

然后调用：

```python
_normalized_angle(angle, open_angle, closed_angle, gamma)
```

把弯曲角度变成 `0..1`：

```text
amount = (angle - open_angle) / (closed_angle - open_angle)
amount = clamp(amount, 0, 1)
amount = amount ** gamma
```

含义：

- `0`: 当前像 open 姿态。
- `1`: 当前像 closed 姿态。
- `gamma`: 调响应曲线。大于 1 会让前半段更迟钝，小于 1 会让前半段更灵敏。

最后：

```python
_lerp_command(open_cmd, closed_cmd, amount)
```

线性插值：

```text
command = open_cmd + amount * (closed_cmd - open_cmd)
```

所以四指弯曲的本质是：

```text
MANUS 三点夹角
  -> 和 open/closed 标定角比较
  -> 得到 0..1 弯曲比例
  -> 在 L20 open/closed 命令之间插值
```

对应配置文件：

```text
src/manus_l20_retarget/config/flexion_right_calibration.yaml
src/manus_l20_retarget/config/flexion_left_calibration.yaml
```

里面的关键字段：

- `root_flexion_open_rad`
- `root_flexion_closed_rad`
- `tip_flexion_open_rad`
- `tip_flexion_closed_rad`
- `command.open_command`
- `command.four_finger_closed_command`

## 伸直保护

函数：

```python
_open_straightness_guard(...)
```

它解决一个实际问题：角度噪声可能让完全伸直的手指被误判成轻微弯曲。

它调用：

```python
_finger_straightness(mcp, pip, dip, tip)
```

计算：

```text
straightness = chord / chain_length
```

- `chain_length`: MCP 到 PIP 到 DIP 到 TIP 分段长度总和。
- `chord`: MCP 到 TIP 的直线距离。

手指越直，`chord` 越接近 `chain_length`，所以 straightness 越接近 1。

如果：

```python
straightness >= threshold
```

就强制 `amount = 0`，也就是认为手指完全张开。

## 四指 yaw 原理

主代码：`_apply_finger_yaw`

它有两种模式。

第一种是标定映射模式，也就是现在 launch 默认开启的：

```python
enable_finger_yaw_mapping = true
```

对应配置：

```text
src/manus_l20_retarget/config/finger_yaw_right_calibration.yaml
src/manus_l20_retarget/config/finger_yaw_left_calibration.yaml
```

配置里记录三种姿态：

- `natural_open`: 自然张开。
- `finger_close`: 四指收拢。
- `finger_spread`: 四指张开外展。

每种姿态都有：

- MANUS yaw 特征值 `yaw_rad`。
- 对应 L20 命令数组。

运行时对每根手指算当前 yaw，然后分别看它更像 close 还是 spread：

```python
close_amount = _normalized_signed_segment(angle, open_rad, close_rad)
spread_amount = _normalized_signed_segment(angle, open_rad, spread_rad)
```

如果 close 方向更强：

```python
command[slot] = lerp(open_cmd, close_cmd, close_amount)
```

否则：

```python
command[slot] = lerp(open_cmd, spread_cmd, spread_amount)
```

所以 yaw 映射不是简单一个线性区间，而是从 open 出发，分成两个方向：

```text
spread_cmd <- open_cmd -> close_cmd
```

第二种是非标定模式：

```python
delta = finger_yaw_command_gain * (angle - finger_yaw_open_rad)
command[slot] = neutral + delta
```

这只是用一个增益直接把角度差换成命令差，调起来不如标定映射稳定。

## yaw 特征从哪里来

配置里 `source` 可能是：

```text
pip
dip
tip
mcp_orientation
pip_orientation
ip_orientation
dip_orientation
```

如果是 `tip` 这类位置来源，代码调用：

```python
_finger_yaw_rad(landmarks, source="tip")
```

它在手掌局部坐标系里算：

```python
atan2(dot(direction, lateral), dot(direction, forward))
```

直觉是：

- 手指朝 forward 方向，yaw 接近 0。
- 手指往 lateral 方向偏，yaw 变大或变小。

如果是 `pip_orientation` 这类姿态来源，代码用 MANUS raw node 的四元数：

1. 找手掌或手部整体 orientation。
2. 找某个手指关节 orientation。
3. 算相对姿态。
4. 转旋转矩阵。
5. 从矩阵里取 yaw。

这类来源常常比位置点更稳定，因为它直接来自 MANUS 的关节姿态估计。

## 拇指弯曲原理

主代码：

```python
_apply_thumb_flexion_mapping
```

配置：

```text
src/manus_l20_retarget/config/thumb_right_flexion_mapping.yaml
src/manus_l20_retarget/config/thumb_left_flexion_mapping.yaml
```

它只管两个槽位：

- `command[0]`: Thumb Base。
- `command[15]`: Thumb Tip。

计算方式和四指弯曲一样：

```python
root_angle = _joint_flexion_rad(landmarks[1], landmarks[2], landmarks[3])
tip_angle = _joint_flexion_rad(landmarks[2], landmarks[3], landmarks[4])
```

然后用两个标定姿态：

- `thumb_natural_open`
- `thumb_pinky_root_touch`

把角度插值到命令：

```python
command[0] = lerp(root_open_cmd, root_touch_cmd, root_amount)
command[15] = lerp(tip_open_cmd, tip_touch_cmd, tip_amount)
```

所以拇指弯曲是标定驱动，不依赖 MuJoCo IK。

## 拇指 segment IK 原理

主代码：

```python
_apply_thumb_ik
_thumb_segment_ik_command
_ThumbSegmentIK
```

它只直接改两个槽位：

- `5`: Thumb Roll。
- `10`: Thumb Yaw。

为什么需要 IK？因为拇指 roll/yaw 不是单纯“某个三点夹角”能表达的。拇指在空间里的方向、外展、对掌动作更像“某一段骨头的 3D 向量应该指向哪里”。所以代码用向量目标反推 L20 的两个关节。

### MANUS 目标向量

函数：

```python
_thumb_segment_source_vector
```

默认参数：

```text
thumb_segment_start = 2
thumb_segment_end = 3
```

意思是取 MANUS 拇指 landmark 2 到 3 这一段。

代码先把点转到手掌局部坐标：

```python
_manus_thumb_local_point_at(landmarks, index)
```

它做的是：

```text
point_relative = landmarks[index] - landmarks[1]
local_x = dot(point_relative, lateral)
local_y = dot(point_relative, forward)
local_z = dot(point_relative, normal)
```

这样拇指向量不受整只手在世界里旋转的影响。

### open 对齐

MANUS 拇指 open 方向和 L20 拇指 open 方向不一定天然一致，所以代码需要对齐：

```python
_align_thumb_segment_open
```

有三种对齐来源：

1. `thumb_segment_frame_path`: 用 open 和 touch 两个姿态建立完整坐标框架。
2. `thumb_segment_manus_open_vector_path`: 只用 open 向量对齐。
3. 启动后前几秒采样当前 MANUS 拇指方向，当作 open。

最核心函数：

```python
_rotation_between(source_open, robot_open)
```

它求一个 3x3 旋转矩阵，把 MANUS open 向量转到 L20 open 向量。

如果有 open/touch 两个向量，则用：

```python
_frame_rotation_from_two_vectors
```

这比单向量对齐更强，因为两个向量可以确定一个局部坐标框架，不只是确定一个方向。

### IK 迭代

类：

```python
_ThumbSegmentIK
```

初始化时它只选择两个活动关节：

```python
("thumb_cmc_yaw", "thumb_cmc_roll")
```

这对应 L20 命令槽位 `10` 和 `5`。

每次求解：

```python
residual = current_segment_vector - target
```

也就是当前 L20 拇指段向量和目标 MANUS 拇指段向量的差。

然后算雅可比：

```python
jacobian = _segment_jacobian()
```

雅可比可以理解为：

```text
关节动一点点，segment 向量会怎么变
```

然后解线性方程：

```python
lhs = jacobian.T @ jacobian
rhs = -(jacobian.T @ residual)
lhs += damping * I
delta = solve(lhs, rhs)
```

这是阻尼最小二乘 IK。直觉是：

- `residual` 告诉你现在偏差在哪里。
- `jacobian` 告诉你每个关节对偏差的影响。
- `delta` 是这一步关节应该改多少。
- `damping` 防止数值不稳定。

每轮还会：

```python
delta 限制最大步长
active = clip(active + delta, lower, upper)
apply_mimic_constraints
```

意思是：

- 不允许一次跳太大。
- 不允许超过 MuJoCo 模型里的关节上下限。
- 应用机械联动约束。

最多迭代 36 次，残差足够小就提前停止。

### qpos 到 0..255 命令

IK 求出来的是 MuJoCo `qpos`，单位是弧度。L20 真正需要的是 0 到 255 命令槽位。

转换由：

```python
LinkerHandModelAdapter.qpos_to_sdk_range
```

完成。

它的内部链路是：

```text
MuJoCo qpos
  -> qpos_to_sdk_arc
  -> LinkerHand SDK arc_to_range_right/left
  -> 0..255 command
```

其中 `qpos_to_sdk_arc` 定义了 L20 的关节语义顺序。对 L20 来说：

```python
[
    thumb_cmc_pitch,
    index_mcp_pitch,
    middle_mcp_pitch,
    ring_mcp_pitch,
    pinky_mcp_pitch,
    thumb_cmc_roll,
    index_mcp_roll,
    middle_mcp_roll,
    ring_mcp_roll,
    pinky_mcp_roll,
    thumb_cmc_yaw,
    0, 0, 0, 0,
    thumb_dip,
    index_dip,
    middle_dip,
    ring_dip,
    pinky_dip,
]
```

这正好对应前面的 20 槽命令表。

## 拇指 progress gate

函数：

```python
_thumb_segment_motion_progress
_thumb_segment_progress_gate
```

用途：控制拇指 IK 的 roll/yaw 在某些阶段才逐渐生效。

比如左手默认：

```text
thumb_segment_yaw_progress_gate_start = 0.25
thumb_segment_yaw_progress_gate_end = 1.0
```

意思是：

- 拇指刚开始动时，yaw IK 不马上完全介入。
- motion progress 到 0.25 后开始逐步打开。
- 到 1.0 时完全打开。

这样可以减少拇指在 open 附近抖动或误触发。

## 命令整形

两个函数经常出现：

```python
_scale_command_delta
_scale_command_delta_ease_in_with_deadzone
```

它们不是重新算 IK，而是对 IK 出来的命令差做后处理。

`_scale_command_delta`：

```text
output = neutral + scale * (value - neutral)
```

用途：让某个关节动作幅度变大或变小。

`_scale_command_delta_ease_in_with_deadzone`：

- 小于 deadzone 的变化直接当成 0。
- 超过 deadzone 后按 gamma 曲线逐渐放大。

用途：让拇指 roll 在接近 open 时更稳，不被小噪声拉动。

## 安全滤波原理

函数：

```python
_filter_command
```

它做三步：

1. `clamp_u8`: 每个值限制在 `0..255`。
2. `rate limit`: 单周期变化不超过 `_max_delta`。
3. `lowpass`: 和上一帧命令做低通融合。

伪代码：

```text
current = clamp(raw_command)
delta = clamp(current - previous, -max_delta, max_delta)
rate_limited = previous + delta
filtered = previous + alpha * (rate_limited - previous)
```

如果：

- `alpha = 1.0`: 不额外低通，只保留限速。
- `alpha` 越小：动作越平滑，但延迟越大。
- `max_delta` 越小：单次动作变化越保守。

## 这套算法的主干图

```text
MANUS raw nodes
  -> 21 landmarks
  -> palm frame
  -> four-finger flexion angles
  -> four-finger root/tip command slots
  -> four-finger yaw feature
  -> yaw command slots
  -> thumb root/tip flexion angles
  -> thumb base/tip command slots
  -> thumb local segment vector
  -> open/frame alignment
  -> MuJoCo segment IK
  -> thumb roll/yaw command slots
  -> reserved slots locked
  -> command smoothing/filtering
  -> final 20-slot command
```

## 后续逐行讲解顺序

如果只看工作原理，建议按这个顺序逐行拆：

1. `manus_landmarks.py`: landmarks、手掌局部坐标系、flexion/yaw 特征。
2. `retarget_pipeline.py`: MANUS feature 提取、四指 flexion target、L20 command adapter 和最终限速/低通滤波。
3. `manus_l20_retarget_node.py` 第 836 到 892 行：主命令链路如何把 `retarget_pipeline.py`、四指 yaw、拇指弯曲和拇指 IK 串起来。
4. `manus_l20_retarget_node.py` 第 894 到 1230 行：四指 yaw、拇指 segment IK 调用、对齐、gate、平滑。
5. `manus_l20_retarget_node.py` 第 1253 到文件末尾：参数解析、数学 helper、本地 IK helper class。
6. `adapters.py` 里 L20 的 `qpos_to_sdk_range` 和 `sdk_range_to_qpos`：弧度和 0..255 命令怎么互转。

不建议优先读 ROS launch、publisher、CAN driver，它们对“为什么机械手这么动”的帮助不大。
