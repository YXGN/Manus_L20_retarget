# 代码结构说明

这个仓库现在是一个独立的 `Manus_L20_retarget` ROS2 工作区，核心目标是：

```text
MANUS 手套 -> MANUS ROS2 数据 -> MANUS 到 L20 重定向 -> LinkerHand L20 真机
```

当前代码里已经不再使用旧的上游工程命名。拇指 IK 支撑库被整理为：

```text
src/l20_thumb_ik
src/l20_thumb_ik/src/l20_ik_core
```

主 ROS 节点和主链路辅助模块被整理为：

```text
src/manus_l20_retarget/manus_l20_retarget/manus_l20_retarget_node.py
src/manus_l20_retarget/manus_l20_retarget/retarget_pipeline.py
```

## 顶层目录

| 路径 | 作用 |
| --- | --- |
| `.gitignore` | 排除 `build/`、`install/`、`log/`、Python 缓存、IDE 配置和 rosbag。 |
| `README.md` | 项目的快速构建和运行说明。 |
| `CODE_STRUCTURE.md` | 当前文件，说明代码组织和每个主要代码文件职责。 |
| `MANUS_L20_TELEOP_SOP.md` | 面向操作者的手套标定、硬件检查和遥操作启动流程。 |
| `CODE_WALKTHROUGH_CN.md` | 中文代码阅读导航，按 ROS 消息、launch、主节点等顺序带读。 |
| `WORKING_PRINCIPLE_CN.md` | 中文算法工作原理说明，重点讲 MANUS 数据如何变成 L20 20 槽命令。 |
| `license/license.txt` | 项目保留的许可证文本。 |
| `src/` | ROS2 包和本项目依赖库源码。 |
| `build/`、`install/`、`log/` | colcon 生成目录，不应该提交到 Git。 |

## 运行链路

```text
manus_ros2/manus_data_publisher
  -> /manus_glove_0
  -> manus_l20_retarget_node
  -> /cb_right_hand_control_cmd 或 /cb_left_hand_control_cmd
  -> linker_hand_advanced_g20
  -> can0
  -> LinkerHand L20
```

右手单独启动入口：

```bash
ros2 launch bringup manus_l20_linkerhand_g20_right.launch.py
```

左手单独启动入口：

```bash
ros2 launch bringup manus_l20_linkerhand_g20_left.launch.py
```

双手同时启动入口：

```bash
ros2 launch bringup manus_l20_linkerhand_g20.launch.py
```

## `src/bringup`

这个包负责把 MANUS 数据发布、重定向节点和 LinkerHand 驱动串成一条 launch 管线。

| 文件 | 作用 |
| --- | --- |
| `CMakeLists.txt` | ROS2 launch 文件安装规则。 |
| `package.xml` | `bringup` 包声明，主要依赖 `launch`、`launch_ros`、`manus_l20_retarget` 和 LinkerHand 驱动包。 |
| `README.md` | 简短的 launch 使用说明。 |
| `launch/manus_l20_common/__init__.py` | 让公共 launch helper 目录可以被 Python import。 |
| `launch/manus_l20_common/pipeline.py` | 右手、左手和双手共用的 launch 工厂。这里集中声明默认参数、标定文件路径、拇指 IK 路径、输出 topic、硬件驱动进程和可选 MANUS publisher。 |
| `launch/manus_l20_linkerhand_g20.launch.py` | 双手同时启动入口，默认右手读 `/manus_glove_0`，左手读 `/manus_glove_1`。 |
| `launch/manus_l20_linkerhand_g20_right.launch.py` | 右手整套真机管线入口，调用公共 pipeline 并设置 `hand_type=right`。 |
| `launch/manus_l20_linkerhand_g20_left.launch.py` | 左手整套真机管线入口，调用公共 pipeline 并设置 `hand_type=left`。 |
| `launch/manus_l20_linkerhand_l20_left.launch.py` | 左手兼容入口，最终仍调用同一套左手 pipeline。 |

## `src/manus_l20_retarget`

这是项目的核心 ROS2 Python 包，负责把 MANUS 手套数据转换成 L20 的 20 槽电机命令。

### 包和配置

| 文件 | 作用 |
| --- | --- |
| `package.xml` | `manus_l20_retarget` 包声明。 |
| `setup.py` | Python 包安装脚本，注册 `manus_l20_retarget_node`、标定采集、调试工具和仿真工具，并安装 `config/*.yaml`。 |
| `setup.cfg` | ament Python 安装路径配置。 |
| `resource/manus_l20_retarget` | ament 资源索引标记。 |
| `config/flexion_right_calibration.yaml` | 右手四指弯曲映射标定。 |
| `config/flexion_left_calibration.yaml` | 左手四指弯曲映射标定。 |
| `config/finger_yaw_right_calibration.yaml` | 右手四指 yaw 映射标定。 |
| `config/finger_yaw_left_calibration.yaml` | 左手四指 yaw 映射标定。 |
| `config/finger_yaw_ergonomics_right_calibration.yaml` | 右手四指 yaw 的 MANUS ergonomics 映射标定，存在时 launch 优先使用。 |
| `config/finger_yaw_ergonomics_left_calibration.yaml` | 左手四指 yaw 的 MANUS ergonomics 映射标定，存在时 launch 优先使用。 |
| `config/thumb_right_flexion_mapping.yaml` | 右手拇指 root/tip 弯曲映射。 |
| `config/thumb_left_flexion_mapping.yaml` | 左手拇指 root/tip 弯曲映射。 |
| `config/thumb_segment_frame_right.yaml` | 右手拇指 segment IK 的 open/touch 双姿态对齐 frame。 |
| `config/thumb_segment_frame_left.yaml` | 左手拇指 segment IK 的 open/touch 双姿态对齐 frame。 |
| `config/thumb_segment_open_vector_right.yaml` | 右手拇指 segment IK 的固定 MANUS 张开基准向量。 |

### Python 源码

| 文件 | 作用 |
| --- | --- |
| `manus_l20_retarget/__init__.py` | Python 包标记。 |
| `manus_l20_retarget/mapping.py` | 基础数值工具，目前主要提供 L20 命令范围裁剪等小函数。 |
| `manus_l20_retarget/manus_landmarks.py` | MANUS raw node 到手部 landmark 的转换层；同时提取四指弯曲、四指 yaw、拇指 segment 等算法输入特征。 |
| `manus_l20_retarget/retarget_pipeline.py` | 主链路纯函数和轻量类集合，包含 MANUS feature 提取、四指 target、L20 command adapter 和 safety filter。 |
| `manus_l20_retarget/manus_l20_retarget_node.py` | 主运行节点。订阅 MANUS 手套消息，组合四指弯曲映射、四指 yaw 映射、拇指弯曲映射、拇指 roll/yaw segment IK，最后发布 L20 20 槽命令。 |
| `manus_l20_retarget/calibration_capture.py` | 标定采集入口集合，推荐使用 `all` 一键合并采集；也保留四指 flexion、四指 yaw、四指 ergonomics yaw、拇指 flexion 和拇指 segment frame 的分项补采子命令。 |
| `manus_l20_retarget/l20_simulation.py` | MANUS 到 L20 的整手 MuJoCo 仿真入口，可选 `--thumb-debug` 查看拇指 segment IK 目标和残差。 |
| `manus_l20_retarget/debug_tools.py` | L20/G20 调试工具集合，目前保留 `g20-probe` 槽位探测子命令。 |

当前稳定控制策略：

| 部位 | 当前策略 |
| --- | --- |
| 四指 root/tip 弯曲 | 标定映射。 |
| 四指 yaw | 优先使用 MANUS ergonomics 的 `IndexSpread` / `MiddleSpread` / `RingSpread` / `PinkySpread` 标定映射；缺少 ergonomics yaw YAML 时回退到 raw skeleton / orientation yaw 标定映射。 |
| 拇指 root/tip 弯曲 | 独立拇指弯曲映射。 |
| 拇指 roll/yaw | MANUS 拇指 2->3 段映射到 L20 `thumb_metacarpals` 的 segment IK。 |

## `src/l20_thumb_ik`

这是本项目内部化后的 L20 拇指 IK 支撑库，主要给 `manus_l20_retarget_node.py` 和 `l20_simulation.py` 提供 MuJoCo 模型、配置加载、IK 求解和命令转换能力。

| 文件或目录 | 作用 |
| --- | --- |
| `README.md` | L20 拇指 IK 支撑库说明。 |
| `pyproject.toml` | `l20_ik_core` Python 包配置。 |
| `LICENSE` | 该支撑库保留的许可证文本。 |
| `assets/mjcf/linkerhand_l20_right/model.xml` | L20 右手 MuJoCo 模型。 |
| `assets/mjcf/linkerhand_l20_right/meshes/*.STL` | 右手 MuJoCo 模型引用的 L20 网格。 |
| `assets/mjcf/linkerhand_l20_left/model.xml` | L20 左手 MuJoCo 模型，由左手 URDF 转换生成。 |
| `assets/mjcf/linkerhand_l20_left/meshes/*.STL` | 左手 MuJoCo 模型引用的 L20 网格。 |
| `configs/retargeting/base/_universal_common.yaml` | 通用重定向配置默认值。 |
| `configs/retargeting/base/linkerhand_l20.yaml` | L20 基础模型配置。 |
| `configs/retargeting/right/linkerhand_l20_right.yaml` | 右手 L20 IK 配置，右手 launch 默认使用。 |
| `configs/retargeting/left/linkerhand_l20_left.yaml` | 左手 L20 IK 配置，左手 launch 默认使用。 |
| `third_party/linkerhand-python-sdk/LinkerHand/utils/mapping.py` | 本地保留的 LinkerHand Python SDK 映射辅助文件。 |

## `src/l20_thumb_ik/src/l20_ik_core`

这是内部 IK 库的 Python 包。当前主链路主要依赖配置加载、MuJoCo hand model、vector solver、L20 命令适配等部分；其余输入源和 CLI 代码作为调试或扩展能力保留。

### 顶层文件

| 文件 | 作用 |
| --- | --- |
| `__init__.py` | 包版本和基础导出。 |
| `api.py` | 对外稳定导入层，导出 `RetargetingEngine` 等核心对象。 |
| `acceptance.py` | 合成测试姿态、左右手镜像和验收辅助函数。 |
| `constants.py` | 配置目录名、模型名、默认参数等常量。 |
| `paths.py` | 解析本地资源、配置和 SDK 路径。 |
| `external_assets.py` | 解析可选外部模型资源路径。 |
| `hand_detector.py` | 可选相机手部检测封装。 |
| `pico_input.py` | 可选 PICO 输入适配。 |
| `hc_mocap_input.py` | 可选 HC mocap UDP 输入适配。 |
| `urdf_converter.py` | URDF 到 MuJoCo 资源转换工具。 |
| `visualization.py` | MuJoCo 可视化辅助函数。 |

### `domain`

| 文件 | 作用 |
| --- | --- |
| `domain/__init__.py` | domain 层导出。 |
| `domain/config.py` | 重定向配置 schema 的 dataclass 定义。 |
| `domain/control.py` | 手部命令和状态抽象。 |
| `domain/hand_detection.py` | 手部检测结果数据结构。 |
| `domain/hand_side.py` | 左右手枚举和归一化工具。 |
| `domain/models.py` | 输入帧、求解结果等核心数据模型。 |
| `domain/preprocessing.py` | landmark 归一化和预处理函数。 |

### `application`

| 文件 | 作用 |
| --- | --- |
| `application/__init__.py` | application 层导出。 |
| `application/engine.py` | 单手重定向引擎。 |
| `application/bihand_engine.py` | 双手重定向引擎。 |
| `application/session.py` | 单手输入到输出的运行循环。 |
| `application/bihand_session.py` | 双手输入到输出的运行循环。 |
| `application/controller_session.py` | 直接连接控制器或硬件时的运行循环。 |
| `application/mediapipe_angles.py` | MediaPipe landmark 角度诊断工具。 |
| `app/__init__.py` | app 层兼容导出。 |

### `infrastructure`

| 文件 | 作用 |
| --- | --- |
| `infrastructure/__init__.py` | infrastructure 层导出。 |
| `infrastructure/config_loader.py` | 读取 YAML 配置并转成 typed config。 |
| `infrastructure/hand_model.py` | MuJoCo 手模型封装，提供 body/site/joint 查询和 qpos 操作。 |
| `infrastructure/vector_solver.py` | 基于向量目标的数值 IK 求解器。 |
| `infrastructure/vector_solver_objective.py` | IK 目标函数和损失项。 |
| `infrastructure/vector_solver_primitives.py` | 向量、关节、约束等低层数学工具。 |
| `infrastructure/vector_solver_targets.py` | 把 landmark 约束转换成 solver target。 |
| `infrastructure/universal_config.py` | 合并通用配置和模型专用配置。 |
| `infrastructure/l20_tip_ik.py` | L20 指尖或接触点 IK 辅助。 |
| `infrastructure/model_name_resolver.py` | 模型名到配置和资源路径的解析。 |
| `infrastructure/preview.py` | 预览帧和可视化辅助。 |
| `infrastructure/artifacts.py` | 记录数据和调试产物的读写。 |
| `infrastructure/sources.py` | 输入源注册和构造。 |
| `infrastructure/sinks.py` | 输出目标注册和构造。 |
| `infrastructure/terminal_controls.py` | 终端非阻塞按键控制。 |
| `infrastructure/controllers/__init__.py` | 控制器适配层导出。 |
| `infrastructure/controllers/adapters.py` | MuJoCo `qpos` 与 LinkerHand 命令范围之间的转换。 |
| `infrastructure/controllers/l20_flexion_calibration.py` | L20 弯曲命令后校准工具。 |
| `infrastructure/controllers/l20_joint_range_mapping.py` | L20 关节范围映射工具。 |
| `infrastructure/controllers/linkerhand_sdk.py` | 可选 LinkerHand SDK 直接控制封装。 |
| `infrastructure/controllers/mujoco_sim.py` | MuJoCo 仿真控制器封装。 |

### `runtime`

| 文件 | 作用 |
| --- | --- |
| `runtime/__init__.py` | runtime 层导出。 |
| `runtime/config_validation.py` | 运行前配置检查。 |
| `runtime/source_adapters.py` | 相机、PICO、mocap、录制文件等输入源适配。 |
| `runtime/source_recording.py` | 录制数据回放输入源。 |
| `runtime/source_sampling.py` | 固定频率采样包装。 |
| `runtime/source_transforms.py` | 运行时坐标和帧变换。 |
| `runtime/sink_outputs.py` | 输出 sink 实现。 |
| `runtime/sink_rendering.py` | 渲染输出辅助。 |
| `runtime/viewer_camera.py` | MuJoCo viewer 相机设置。 |
| `runtime/viewer_hand.py` | 手模型 viewer 场景更新。 |
| `runtime/viewer_landmarks.py` | landmark viewer 场景更新。 |
| `runtime/viewer_passive.py` | 被动 MuJoCo viewer 管理。 |
| `runtime/viewer_async.py` | 异步 viewer 包装。 |

### `cli` 和 `interfaces`

| 文件 | 作用 |
| --- | --- |
| `cli/__init__.py` | CLI 包导出。 |
| `cli/__main__.py` | 支持 `python -m l20_ik_core.cli`。 |
| `cli/parser.py` | 命令行参数解析。 |
| `cli/commands.py` | CLI 子命令定义。 |
| `cli/runtime.py` | CLI 运行调度。 |
| `cli/main.py` | CLI 主入口。 |
| `interfaces/__init__.py` | 对外 interface 包导出。 |
| `interfaces/cli.py` | CLI 兼容导入层。 |
| `core/__init__.py` | core 兼容导出层。 |

## `src/linker_hand_ros2_sdk`

这是 LinkerHand 厂商驱动包。本项目调用其中的 `linker_hand_advanced_g20.py` 作为 L20 真机执行后端。

| 文件 | 作用 |
| --- | --- |
| `package.xml` | ROS2 包声明。 |
| `pyproject.toml` | Python 构建配置。 |
| `setup.py` | 注册 LinkerHand ROS2 驱动入口。 |
| `setup.cfg` | ament Python 安装配置。 |
| `linker_hand_ros2_sdk/__init__.py` | Python 包标记。 |
| `linker_hand_ros2_sdk/linker_hand_advanced_g20.py` | 真机驱动 ROS 可执行文件，订阅控制命令并通过 CAN 写入 G20/L20 硬件。 |
| `linker_hand_ros2_sdk/LinkerHand/__init__.py` | 厂商 LinkerHand Python 包标记。 |
| `linker_hand_ros2_sdk/LinkerHand/linker_hand_api.py` | 厂商 API 封装，负责设备通信、命令发送和状态读取。 |
| `linker_hand_ros2_sdk/LinkerHand/config/__init__.py` | 配置目录包标记。 |
| `linker_hand_ros2_sdk/LinkerHand/config/*.yaml` | 各型号手的默认位置、范围和通信配置。 |
| `linker_hand_ros2_sdk/LinkerHand/core/__init__.py` | 通信核心包标记。 |
| `linker_hand_ros2_sdk/LinkerHand/core/can/__init__.py` | CAN 通信包标记。 |
| `linker_hand_ros2_sdk/LinkerHand/core/can/linker_hand_g20_can.py` | G20/L20 使用的 CAN 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/can/linker_hand_l10_can.py` | L10 CAN 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/can/linker_hand_l20_can.py` | L20 CAN 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/can/linker_hand_l21_can.py` | L21 CAN 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/can/linker_hand_l24_can.py` | L24 CAN 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/can/linker_hand_l25_can.py` | L25 CAN 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/can/linker_hand_l6_can.py` | L6 CAN 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/can/linker_hand_l7_can.py` | L7 CAN 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/can/linker_hand_o6_can.py` | O6 CAN 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/rs485/linker_hand_l10_rs485.py` | L10 RS485 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/rs485/linker_hand_l6_rs485.py` | L6 RS485 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/rs485/linker_hand_l7_rs485.py` | L7 RS485 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/core/rs485/linker_hand_o6_rs485.py` | O6 RS485 协议实现。 |
| `linker_hand_ros2_sdk/LinkerHand/utils/__init__.py` | 工具包标记。 |
| `linker_hand_ros2_sdk/LinkerHand/utils/color_msg.py` | 终端彩色日志工具。 |
| `linker_hand_ros2_sdk/LinkerHand/utils/init_linker_hand.py` | 初始化 LinkerHand 设备的辅助函数。 |
| `linker_hand_ros2_sdk/LinkerHand/utils/load_write_yaml.py` | YAML 配置读写工具。 |
| `linker_hand_ros2_sdk/LinkerHand/utils/mapping.py` | 厂商命令范围映射工具。 |
| `linker_hand_ros2_sdk/LinkerHand/utils/open_can.py` | CAN 接口打开和检查工具。 |
| `test/test_copyright.py` | 厂商包默认测试。 |
| `test/test_flake8.py` | 厂商包默认 flake8 测试。 |
| `test/test_pep257.py` | 厂商包默认 docstring 测试。 |

## `src/manus_ros2`

这是 MANUS SDK 到 ROS2 的 C++ 发布器。

| 文件 | 作用 |
| --- | --- |
| `CMakeLists.txt` | C++ 节点编译和安装规则。 |
| `package.xml` | ROS2 包声明。 |
| `client_scripts/manus_data_viz.py` | MANUS 数据辅助可视化脚本。 |
| `src/manus_data_publisher.cpp` | C++ main 入口，启动 MANUS 数据发布器。 |
| `src/ManusDataPublisher.hpp` | MANUS 发布器类声明。 |
| `src/ManusDataPublisher.cpp` | MANUS SDK 连接、数据读取、ROS message 发布逻辑。 |
| `src/ClientLogging.hpp` | MANUS 客户端日志辅助。 |
| `src/ClientPlatformSpecific.hpp` | 平台相关工具声明。 |
| `src/ClientPlatformSpecific.cpp` | 平台相关工具实现。 |
| `src/ClientPlatformSpecificTypes.hpp` | 平台相关类型定义。 |

## `src/manus_ros2_msgs`

这是 MANUS ROS2 message 定义包。

| 文件 | 作用 |
| --- | --- |
| `CMakeLists.txt` | ROS message 生成规则。 |
| `package.xml` | message 包声明。 |
| `msg/ManusGlove.msg` | 主手套消息，包含 ergonomics 和 raw skeleton nodes。 |
| `msg/ManusRawNode.msg` | MANUS raw skeleton node 消息。 |
| `msg/ManusErgonomics.msg` | MANUS ergonomics 数据消息。 |
| `msg/ManusVibrationCommand.msg` | 可选振动命令消息。 |

## `src/ManusSDK`

这是 MANUS 官方 SDK 二进制和头文件。

| 文件 | 作用 |
| --- | --- |
| `include/ManusSDK.h` | MANUS SDK C API 头文件。 |
| `include/ManusSDKTypes.h` | MANUS SDK 类型定义。 |
| `include/ManusSDKTypeInitializers.h` | MANUS SDK 类型初始化辅助。 |
| `lib/libManusSDK.so` | MANUS SDK 动态库。 |
| `lib/libManusSDK_Integrated.so` | MANUS SDK 集成动态库。 |

## `src/manus_l20_retarget/third_party/sharpa-manus-sdk`

这是随 `manus_l20_retarget` 归档的 Sharpa MANUS 工具包，用于保留 MANUS 标定 GUI、Sharpa 参考客户端、Wave retargeting 参考代码和相关 vendor 资源。把它放到 `manus_l20_retarget/third_party` 后，上传 Git 时不会再依赖工作区根目录下单独散落的 `src/sharpa-manus-sdk-main`。

注意：在线 ROS2 发布器 `manus_ros2` 仍然使用 `src/ManusSDK` 下的 MANUS SDK 头文件和动态库；这里的 Sharpa vendor 包主要用于标定 GUI 和参考代码归档，不替代 `src/ManusSDK`。

| 文件或目录 | 作用 |
| --- | --- |
| `client/CalibrationGUI` | MANUS 手套 `.mcal` 标定 GUI。SOP 中的手套标定命令使用这个目录。 |
| `client/include` | Sharpa MANUS 客户端引用的 MANUS SDK 头文件副本。 |
| `client/*.mcal` | Sharpa 客户端保留的左右手标定文件示例或本地结果。 |
| `retargeting_alg_release_V4.0` | Sharpa Wave retargeting 参考实现和资源。当前 MANUS -> L20 主链路不直接依赖它。 |
| `README.md` / `NOTICE.txt` / `License` | vendor 包原始说明和许可证信息。 |

## 现在应该维护的主文件

日常算法和真机效果调整，优先看这些文件：

| 文件 | 为什么重要 |
| --- | --- |
| `src/manus_l20_retarget/manus_l20_retarget/manus_l20_retarget_node.py` | 在线 ROS 节点编排、参数、标定加载、yaw/拇指 IK 和发布逻辑在这里。 |
| `src/manus_l20_retarget/manus_l20_retarget/retarget_pipeline.py` | MANUS feature 提取、四指 flexion target、L20 command adapter 和最终 command filter 在这里。 |
| `src/manus_l20_retarget/manus_l20_retarget/manus_landmarks.py` | MANUS 点位、姿态和特征提取在这里。 |
| `src/bringup/launch/manus_l20_common/pipeline.py` | 启动参数和左右手默认配置在这里。 |
| `src/manus_l20_retarget/config/*.yaml` | 当前真机效果依赖的标定文件。 |
| `src/l20_thumb_ik/configs/retargeting/*/*.yaml` | 拇指 IK 使用的 L20 模型配置。 |
| `src/l20_thumb_ik/src/l20_ik_core/infrastructure/hand_model.py` | MuJoCo 模型封装。 |
| `src/l20_thumb_ik/src/l20_ik_core/infrastructure/vector_solver.py` | IK 数值求解器。 |
| `src/l20_thumb_ik/src/l20_ik_core/infrastructure/controllers/adapters.py` | IK 解和 LinkerHand 命令之间的转换。 |

## 不建议随便改的部分

| 路径 | 原因 |
| --- | --- |
| `src/ManusSDK` | 官方 MANUS 二进制 SDK。 |
| `src/manus_l20_retarget/third_party/sharpa-manus-sdk` | vendor 工具和参考实现，除路径整理和必要补丁外不要随意改算法内容。 |
| `src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/core` | 厂商底层通信协议。 |
| `src/manus_ros2_msgs/msg/*.msg` | 改 message 会触发接口兼容问题。 |
| `build/`、`install/`、`log/` | 生成目录，不提交、不人工维护。 |
