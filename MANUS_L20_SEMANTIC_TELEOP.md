# MANUS L20 语义对指遥操作说明

本文档只记录语义对指版本的标定和启动流程。普通四指弯曲、四指 yaw、拇指弯曲、拇指 Roll/Yaw IK 的基础标定仍参考 `MANUS_L20_TELEOP_SOP.md`。

## 1. 功能范围

语义对指识别四类人手动作：

- 拇指指尖接触食指指尖。
- 拇指指尖接触中指指尖。
- 拇指指尖接触无名指指尖。
- 拇指指尖接触小指指尖。

数据来源是 `ManusGlove.raw_nodes`。程序每帧取 `Thumb.TIP` 和对应 `Finger.TIP`，计算：

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
  中指: slot 2, 17
  无名指: slot 3, 18
  小指: slot 4, 19
拇指 Roll/Yaw: slot 5, 10
```

其他手指和四指 yaw 继续走正常遥操作。

语义对指现在带有“分相位接管”：

```text
张开 -> 对指夹住:
  先接管 slot 5/10，让 Thumb Roll/Yaw 提前到当前对指目标位。
  后接管 slot 0/当前手指root/15/当前手指tip，让弯曲再夹住。

对指夹住 -> 张开:
  先把 slot 0/当前手指root/15/当前手指tip 拉回 open，避免松开时反夹一下。
  slot 5/10 先慢后快退回正常拇指 IK，减少横向扫到物体。
```

这个提前量是相对于当前语义动作的，例如识别到食指对指时，slot 5/10 只提前走食指对指 YAML 里的 Roll/Yaw；不会提前转到小拇指对指姿态。

## 2. 代码结构

语义对指相关逻辑集中在 `contact_semantics.py`：

```text
thumb_fingertip_distance_ratios()
  从 raw skeleton 计算 Thumb.TIP 到四指 TIP 的掌宽归一化距离。

FingertipContactStateMachine
  根据距离比例选择 index/middle/ring/pinky 的当前对指 pair，并输出 activation。
  activation 带坚定接触保持，避免 raw skeleton 小抖动造成反复张开/闭合。

FingertipContactCommandBlender
  根据 activation 的增减方向做分相位接管：
  - 闭合时 slot 5/10 先走。
  - 松开时 root/tip 先回 open。

blend_fingertip_contact_command()
  只混合当前 pair 的 override_slots，不改其他手指。
```

`manus_l20_retarget_node.py` 只负责调度：

```text
基础遥操命令
-> thumb_fingertip_distance_ratios()
-> FingertipContactStateMachine.update()
-> FingertipContactCommandBlender.blend()
-> 发布 L20 20-slot
```

这样主节点保持一条主链路，语义模块自己负责“识别 + 状态机 + 接管权重”。

## 3. 启动 MANUS 数据发布器

新开一个终端，只启动 MANUS，不启动 L20 真机：

```bash
source /opt/ros/humble/setup.bash
cd /home/huangzizhe/Download/Manus_L20_retarget
source install/setup.bash

ros2 run manus_ros2 manus_data_publisher --ros-args \
  -p load_calibration:=true \
  -p left_calibration_path:=/home/huangzizhe/Download/Manus_L20_retarget/src/manus_ros2/calibration/Calibration_left.mcal \
  -p right_calibration_path:=/home/huangzizhe/Download/Manus_L20_retarget/src/manus_ros2/calibration/Calibration_right.mcal
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
cd /home/huangzizhe/Download/Manus_L20_retarget
source install/setup.bash

ros2 run manus_l20_retarget calibration_capture fingertip-contact \
  --hand right \
  --glove-topic /manus_glove_0 \
  --duration 2.0
```

左手：

```bash
source /opt/ros/humble/setup.bash
cd /home/huangzizhe/Download/Manus_L20_retarget
source install/setup.bash

ros2 run manus_l20_retarget calibration_capture fingertip-contact \
  --hand left \
  --glove-topic /manus_glove_1 \
  --duration 2.0
```

如果左手手套实际发布在 `/manus_glove_0`，把左手命令里的 `/manus_glove_1` 改成 `/manus_glove_0`。

工具会依次采 5 个姿势：

- `natural_open`: 自然张开，拇指远离四指。
- `thumb_index_tip_touch`: 拇指指尖触碰食指指尖。
- `thumb_middle_tip_touch`: 拇指指尖触碰中指指尖。
- `thumb_ring_tip_touch`: 拇指指尖触碰无名指指尖。
- `thumb_pinky_tip_touch`: 拇指指尖触碰小指指尖。

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
拇指 Root slot 0 = 174
四指 Root slot 1-4 = 78
拇指 Tip slot 15 = 160
四指 Tip slot 16-19 = 161
```

四组对应 slot：

```text
thumb-index:  slot 0/1/15/16  = 174/78/160/161
thumb-middle: slot 0/2/15/17  = 174/78/160/161
thumb-ring:   slot 0/3/15/18  = 174/78/160/161
thumb-pinky:  slot 0/4/15/19  = 174/78/160/161
```

拇指 Roll/Yaw 仍使用每组 `contact_command` 原有值：

```text
slot 5  Thumb Roll
slot 10 Thumb Yaw
```

启用语义对指前，确认：

```yaml
runtime:
  enabled: true
```

注意必须是 `true`，不是 `ture`。本语义调试分支里 YAML 和 launch 默认都已打开；如果 YAML 写错，launch 参数打开了也不会生效。

## 6. 双手语义遥操作启动命令

真机测试由用户手动执行。确认 CAN 已配置好后运行：

```bash
source /opt/ros/humble/setup.bash
cd /home/huangzizhe/Download/Manus_L20_retarget
source install/setup.bash

ros2 launch bringup manus_l20_linkerhand_g20.launch.py \
  start_manus:=true \
  right_can:=can0 \
  left_can:=can1 \
  is_touch:=false \
  right_input_topic:=/manus_glove_0 \
  left_input_topic:=/manus_glove_1 \
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
fingertip_contact_release_flexion_open_completion:=0.18
fingertip_contact_release_orientation_gamma:=2.5
```

含义：

- `close_orientation_completion`: 从张开到夹住的前多少接近行程内，Roll/Yaw 就完成接管。数值越小，Roll/Yaw 越早到位。
- `close_flexion_start`: 接近行程超过多少以后，root/tip 才开始按语义夹住。数值越大，越晚闭合。
- `release_flexion_open_completion`: 从夹住到松开的前多少释放行程内，root/tip 完成回 open。数值越小，弯曲越快松开。
- `release_orientation_gamma`: 松开时 Roll/Yaw 的回退曲线。数值越大，前段越慢、后段越快。

YAML `runtime` 里还有三个用于“夹住更坚定”的参数：

```text
full_takeover_progress: 0.75
firm_contact_enter_activation: 0.9
firm_contact_release_activation: 0.35
```

含义：

- `full_takeover_progress`: 从自然张开到接触标定值的 75% 行程时，语义接管就饱和到 1.0，不要求人手必须精确压到标定接触距离。
- `firm_contact_enter_activation`: activation 达到 0.9 后，认为已经进入坚定接触。
- `firm_contact_release_activation`: 进入坚定接触后，只有松到 0.35 以下才退出坚定接触，避免指尖距离小抖动导致 L20 反复张开闭合。

如果后续要临时覆盖默认值，也可以在启动命令后加新的数值，例如：

```bash
fingertip_contact_close_orientation_completion:=0.25 \
fingertip_contact_close_flexion_start:=0.40 \
fingertip_contact_release_flexion_open_completion:=0.18 \
fingertip_contact_release_orientation_gamma:=2.5
```

如果要显式写出来：

```bash
driver_speed:=80,80,80,80,80
```

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

如果识别到错误手指：

- 重新采 `fingertip-contact`。
- 自然张开姿势要足够张开，拇指不要靠近四指。
- 每个对指姿势只让拇指接触目标手指，其他手指尽量远离。

如果接触姿态不对：

- 优先检查 `contacts.<finger>.robot.contact_command`。
- `human` 只决定什么时候触发；真正 L20 对指姿态来自 `robot.contact_command`。
- 当前动作只会接管本动作声明的 `override_slots`。
