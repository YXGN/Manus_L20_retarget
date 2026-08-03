# Manus Glove Calibration GUI

基于Manus SDK的可视化标定应用程序。

## 功能特性

- 实时显示左右手套的连接状态
- 图形化标定流程界面
- 每个标定步骤显示手势示意图；连续动作步骤中的手指会动态摆动提示
- 支持键盘快捷键（F5开始标定，F9下一步）
- 自动步骤管理和提示

## 依赖要求

### 系统依赖

在Ubuntu/Debian系统上安装：

```bash
sudo apt-get update
sudo apt-get install build-essential libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libgdk-pixbuf-2.0-dev
```

在CentOS/RHEL系统上安装：

```bash
sudo yum install gcc-c++ glfw-devel mesa-libGL-devel mesa-libGLU-devel
```

### 项目依赖

- ManusSDK库（已包含在ManusSDK目录中）
- ImGui库（会自动下载）
- gdk-pixbuf 开发库（用于播放官方 GIF）

## 编译

### 方法1：使用构建脚本（推荐）

```bash
./build.sh
```

构建脚本会自动：
- 检查并下载ImGui库
- 检查系统依赖
- 编译项目

### 方法2：手动编译

1. 下载ImGui库（如果尚未下载）：
```bash
./download_imgui.sh
```

2. 编译项目：
```bash
make
```

3. 运行：
```bash
./CalibrationGUI.out
```

## 使用方法

1. **选择手套**：点击"Left Glove"或"Right Glove"按钮选择要标定的手套
2. **开始标定**：点击"Start Calibration"按钮或按F5键开始标定
3. **执行步骤**：按照提示做出手势，然后点击"Next Step"按钮或按F9键进入下一步
4. **完成标定**：最后一步会自动完成标定流程

## 界面说明

- **顶部状态栏**：显示左右手套连接状态，并突出当前标定手
- **左侧信息区**：显示当前标定步骤的详细信息
- **右侧按钮面板**：包含所有操作按钮
- **手势图区域**：显示 MANUS 官方四步 GIF；GIF 不可用时自动回退到内置示意图

## 注意事项

- 确保Manus Core正在运行
- 确保手套已正确连接并配对
- 标定过程中请按照提示做出准确的手势

## 关于 GIF 手势图

界面使用 gdk-pixbuf 解码官方 GIF，在主循环按帧更新 OpenGL 纹理，再通过
`ImGui::Image` 显示。官方素材来自 MANUS 文档：
<https://docs.manus-meta.com/3.1.0/Products/Metagloves%20Pro%20Haptic/Calibration/>。
四个 GIF 已放在 `assets/` 目录；如果运行环境缺少 gdk-pixbuf 或素材文件，界面会回退到
内置示意图，不影响文字标定流程。

## Linux 目录包部署

在本目录执行：

```bash
./package.sh
```

生成的 `dist/manus-calibration/` 可直接复制到其他 Ubuntu 设备，并通过
`./run.sh` 启动。包内包含程序、MANUS SDK、GIF 资源和 `calibration/` 目录。

标定结果路径规则：

- 从源码目录运行时，自动保存到工作区的 `src/manus_ros2/calibration/`。
- 目录包位于工作区内部时，`run.sh` 会向上查找该目录并自动保存到这里。
- 独立部署时，保存到包内 `calibration/`；也可显式指定：

```bash
MANUS_CALIBRATION_DIR=/path/to/src/manus_ros2/calibration ./run.sh
```
