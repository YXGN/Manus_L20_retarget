# MANUS 手套标定 GUI 目录包

这个目录用于替代旧的 MANUS 标定 GUI 入口：

```text
旧入口:
src/manus_l20_retarget/third_party/sharpa-manus-sdk/client/CalibrationGUI

新入口:
deploy/manus-calibration/run.sh
```

它只负责生成 MANUS 手套自己的 `.mcal` 标定文件，不负责生成 L20 retarget YAML，也不负责语义对指标定。

## 运行

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/huangzizhe/Download/Manus_L20_retarget
./deploy/manus-calibration/run.sh
```

`run.sh` 会向上查找当前工作区里的 `src/manus_ros2/calibration/`，并把 GUI 的标定结果保存到：

```text
src/manus_ros2/calibration/Calibration_left.mcal
src/manus_ros2/calibration/Calibration_right.mcal
```

如果你想把 `.mcal` 保存到其他目录，可以手动指定：

```bash
MANUS_CALIBRATION_DIR=/path/to/src/manus_ros2/calibration ./deploy/manus-calibration/run.sh
```

## GUI 操作

1. 选择 `Left Glove` 或 `Right Glove`。
2. 点击 `Start Calibration`，或按 `F5`。
3. 按界面提示完成每个手势。
4. 每一步完成后点击 `Next Step`，或按 `F9`。
5. 左右手分别标定一次。

## 数据流

```text
MANUS 官方标定 GUI
-> 生成 Calibration_left.mcal / Calibration_right.mcal
-> manus_data_publisher 启动时加载 .mcal
-> 输出更稳定的 ManusGlove ergonomics/raw_nodes 数据
-> 后续 L20 retarget / 语义对指继续使用自己的 YAML 标定
```

也就是说，这个 GUI 改善的是 MANUS 数据源本身；L20 的四指弯曲、四指 yaw、拇指弯曲、拇指 Roll/Yaw IK、语义对指接触参数，仍然由 `ros2 run manus_l20_retarget calibration_capture ...` 采集。

## SDK 动态库

工作区内的 `run.sh` 会优先使用目录包里的
`lib/libManusSDK_Integrated.so`。如果该文件还是 Git LFS 指针，脚本会自动回退到项目已有的：

```text
src/ManusSDK/lib/libManusSDK_Integrated.so
```

当前项目这份本地 SDK 约 117 MB，可以直接支持 GUI，不需要重复下载。

检查本地 SDK：

```bash
cd /home/huangzizhe/Download/Manus_L20_retarget
ls -lh deploy/manus-calibration/lib/libManusSDK_Integrated.so
ls -lh src/ManusSDK/lib/libManusSDK_Integrated.so
```

如果你把 `deploy/manus-calibration` 单独复制到另一台设备，没有项目已有 SDK，则需要拉取目录包中的 LFS 动态库：

```bash
git lfs install
git lfs pull -I deploy/manus-calibration/lib/libManusSDK_Integrated.so
```

如果机器没有 `git lfs`，先安装：

```bash
sudo apt-get update
sudo apt-get install -y git-lfs
```

## 注意事项

- 确保 MANUS Core 正在运行。
- 确保手套已上电、连接并配对。
- 不要在 GUI 运行时同时启动多个会占用 MANUS SDK 的程序。
- 这个工具不会启动 L20 真机控制。
