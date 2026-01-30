# ROS 2 Hikvision Camera Driver

适用于海康机器人（HikRobot）工业相机的 ROS 2 驱动包。

## 架构支持 (Architecture Support)

本驱动包含海康官方 SDK 动态库，支持以下架构：
- **x86_64 (amd64)**
- **aarch64 (arm64 / ARMv8)**

## 依赖安装 (Dependencies)

使用前请确保安装以下依赖库（以 Humble 为例）：

```bash
sudo apt update
sudo apt install -y ros-humble-image-transport-plugins
sudo apt install -y ros-humble-camera-info-manager
sudo apt install -y ros-humble-camera-calibration-parsers
```

## 编译与运行 (Build & Run)

```bash
# 编译
colcon build --symlink-install --packages-select hik_camera

# 运行
source install/setup.bash
ros2 launch hik_camera hik_camera.launch.py
```

## 参数配置 (Configuration)

配置文件位于 `config/camera_params.yaml`，支持动态配置以下参数：
- `exposure_time`: 曝光时间 (us)
- `gain`: 增益
- `acquisition_frame_rate`: 硬件采集帧率 (Hz)
- `image_width` / `image_height`: 分辨率宽/高
- `offset_x` / `offset_y`: ROI 偏移量
- `publish_rate`: 软件限流 (0.0 为禁用，推荐禁用)
