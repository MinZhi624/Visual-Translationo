# RoboMaster 装甲板视觉识别系统

基于 **ROS 2 Jazzy/Humble** + **OpenCV 4.x** + **OpenVINO Runtime** 的装甲板识别、跟踪与瞄准解算方案。当前开发环境为 **ROS 2 Jazzy**。

---

## 项目简介

完整的视觉处理链路：图像采集 → 预处理 → 灯条检测与配对 → 数字识别 → PnP 位姿解算 → 世界坐标系扩展卡尔曼滤波跟踪 → 串口通信。

支持**相机实时运行**与**离线视频调试**两种模式。

---

## 数据流

```mermaid
flowchart TD
    camera["相机/视频"] --> identification["armor_plate_identification<br/>识别 + PnP + 数字识别 + 云台数据打包"]
    ec_feedback["电控回传"] -->|"/gimbal_angle<br/>GimbalAngle"| identification
    identification -->|"/armor_plates<br/>ArmorPlates<br/>含 gimbal_yaw_abs / gimbal_pitch_abs"| tracker["armor_plate_tracker<br/>世界坐标系 11 维 EKF"]
    tracker -->|"/aim_command<br/>AimCommand"| serial["armor_plate_serial<br/>串口发送"]
    tracker -->|"/visualization_marker_array<br/>/tracker_debug<br/>/tracker_data"| visual["Foxglove / RViz"]
    serial --> ec["电控"]
```

**消息说明**

| Topic | 类型 | 说明 |
|-------|------|------|
| `/armor_plates` | `ArmorPlates` | 检测到的装甲板数组（含位姿、数字、图像中心距、云台绝对角） |
| `/gimbal_angle` | `GimbalAngle` | 电控回传的绝对 yaw/pitch（带时间戳） |
| `/aim_command` | `AimCommand` | 控制指令（delta_yaw, delta_pitch，单位 rad） |
| `/tracker_debug` | `TrackerDebug` | 相机系下测量点与滤波点（用于图像叠加绘制） |
| `/tracker_data` | `TrackerData` | measurement/filter 的 yaw/pitch |
| `/visualization_marker_array` | `MarkerArray` | 旋转中心、速度、观测装甲板、滤波装甲板和四块预测装甲板 |

---

## 功能包说明

| 功能包 | 职责 | 节点 | 订阅 | 发布 |
|--------|------|------|------|------|
| `armor_plate_identification` | 图像采集、预处理、灯条检测、PnP、数字识别、云台数据打包 | `ArmorPlateIdentification` (相机) / `Test` (视频) | `/gimbal_angle` | `/armor_plates`, TF |
| `armor_plate_tracker` | 目标选择、世界坐标系 11 维 EKF | `armor_plate_tracker_node` | `/armor_plates` | `/aim_command`, `/tracker_debug`, `/tracker_data`, `/visualization_marker_array` |
| `armor_plate_serial` | 串口双向通信 | `serial_node` | `/aim_command` | `/gimbal_angle`, (串口) |
| `armor_plate_interfaces` | 自定义消息定义 | — | — | — |
| `armor_plate_bringup` | 一键启动组合 | `run.launch.py` / `test.launch.py` / `auto_test.launch.py` | — | — |

---

## 技术特点

### 1. 预处理

- 灰度转换 → 固定阈值二值化 → 膨胀
- 不依赖颜色通道，过曝/杂光场景更鲁棒
- 支持 `target_color` 参数切换红/蓝方（用于数字识别与后续逻辑）

### 2. 灯条检测与匹配

- 轮廓筛选：面积 + 长宽比约束
- 直线提取：`fitEllipse` 方向 + `minAreaRect` 长度约束
- 几何约束配对：6 个参数（角度差、长度比、中心距比、Y/X 差比等）
- 打分 + NMS：按平行度、长度相似度排序，一个灯条只归属一个装甲板

### 3. 数字识别

- 装甲板中心 ROI 透视变换提取
- **OpenVINO Runtime** 推理，输出 0-9 + negative
- 模型文件：`model/number_cnn.onnx`（另有 `mlp.onnx` 备选）
- 训练工具链见 `DeepLearning/`

### 4. PnP 位姿解算

- `cv::solvePnP` 解算 `tvec` + 四元数
- 动态读取 `CameraInfo` 获取相机内参与畸变系数

### 5. 目标跟踪（世界坐标系 EKF）

- **云台数据**：Identification 订阅 `/gimbal_angle` 并打包进 `ArmorPlates` 消息（`gimbal_yaw_abs`、`gimbal_pitch_abs`），Tracker 直接读取，无需独立时间对齐。
- **坐标变换**：`CoordinateTransformer` 实现 camera → gimbal（固定旋转）→ world（动态 yaw/pitch）旋转链。
- **11 状态 EKF**：`[x_c, v_x, y_c, v_y, z_c, v_z, yaw, omega, r, l, h]`
  - `x_c, y_c, z_c`：目标旋转中心位置。
  - `yaw, omega`：目标自转角和自转角速度。
  - `r`：`id=0/2` 装甲板半径。
  - `r + l`：`id=1/3` 装甲板半径。
  - `z_c + h`：`id=1/3` 装甲板高度。
- **装甲板几何模型**：普通 `1-5` 目标统一按四装甲板模型处理。
  - `angle_i = yaw + i * PI / 2`
  - `id=0/2` 时 `radius = r`
  - `id=1/3` 时 `radius = r + l`
  - `id=0/2` 时 `armor_z = z_c`
  - `id=1/3` 时 `armor_z = z_c + h`
  - `armor_x = x_c - radius * cos(angle_i)`
  - `armor_y = y_c - radius * sin(angle_i)`
  - `armor_angle = angle_i`
- **观测量**：`[yaw_to_armor, pitch_to_armor, distance_to_armor, armor_yaw]`，由预测装甲板 `xyza` 转为 `ypda` 后与 PnP 观测比较。
- **过程噪声**：`x/v_x`、`y/v_y`、`z/v_z`、`yaw/omega` 使用分段白噪声加速度模型，`r/l/h` 当前视为静态几何参数。
- **约束**：`yaw` 归一化到 `[-PI, PI]`；`r` 与 `r + l` 限制在 `[0.05, 0.5] m`。
- **目标选择**：未初始化时选图像中心最近；已初始化时沿用当前世界系预测位置最近的简单匹配机制。
- **丢失处理**：无目标时只预测；`max_lost_time=0.5s` 超时重置。

### 6. 串口双向通信

**视觉 → 电控** (`0xA5 0x5A`)：
```c
typedef struct {
    uint8_t  sof1;              // 0xA5
    uint8_t  sof2;              // 0x5A
    uint8_t  seq;
    uint8_t  target_valid;
    int16_t  delta_yaw_1e4rad;
    int16_t  delta_pitch_1e4rad;
    uint16_t crc16;             // CRC16/MODBUS，前8字节
} VisionToEcFrame_t;
```

**电控 → 视觉** (`0x5A 0xA5`)：
```c
typedef struct {
    uint8_t  sof1;              // 0x5A
    uint8_t  sof2;              // 0xA5
    uint8_t  seq_echo;
    int32_t  yaw_actual_1e4rad;
    int32_t  pitch_actual_1e4rad;
    uint16_t crc16;             // CRC16/MODBUS，前11字节
} EcToVisionFrame_t;
```

- 发送：订阅 `/aim_command` 即时发送，`int16_t = rad * 10000.0f`
- 接收：独立线程 + 状态机解析，解析成功后填充 `stamp = now()` 发布 `/gimbal_angle`
- 无目标时自动停发

---

## 快速开始

### 环境依赖

当前开发环境：Ubuntu 24.04 + ROS 2 Jazzy。项目也保留 Ubuntu 22.04 + ROS 2 Humble 的兼容说明；下面命令用 `$ROS_DISTRO` 适配当前已 source 的 ROS 发行版。

基础工具：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  libopencv-dev libeigen3-dev
```

ROS 依赖包：

```bash
# 如果没有 source ROS 环境，Jazzy 用户可先执行：source /opt/ros/jazzy/setup.bash
# Humble 用户对应执行：source /opt/ros/humble/setup.bash
sudo apt install -y \
  ros-${ROS_DISTRO}-ament-cmake \
  ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-rclcpp-components \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-builtin-interfaces \
  ros-${ROS_DISTRO}-visualization-msgs \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-camera-info-manager \
  ros-${ROS_DISTRO}-serial-driver \
  ros-${ROS_DISTRO}-io-context \
  ros-${ROS_DISTRO}-rosidl-default-generators \
  ros-${ROS_DISTRO}-rosidl-default-runtime \
  ros-${ROS_DISTRO}-foxglove-bridge
```

推理与相机 SDK：

- OpenVINO Runtime：`armor_plate_identification` 使用 `find_package(OpenVINO REQUIRED)` 和 `openvino::runtime`，系统需要安装能提供 `OpenVINOConfig.cmake` 的 OpenVINO C++ 开发包。若 apt 源中提供，可安装 `openvino` 或 `libopenvino-dev`；否则按 Intel OpenVINO 官方方式安装并 source 对应 `setupvars.sh`。
- MindVision / Galaxy 相机 SDK：仓库已内置于 `src/armor_plate_identification/third_parties/`，安装后环境钩子会配置 `LD_LIBRARY_PATH` 和 `GENICAM_GENTL64_PATH`。

### 编译

```bash
cd /home/minzhi/Desktop/Visual-Translationo

colcon build --packages-select \
  armor_plate_interfaces \
  armor_plate_identification \
  armor_plate_tracker \
  armor_plate_serial \
  armor_plate_bringup

source install/setup.bash
```

### 运行

#### 一键启动（推荐）

```bash
# 相机实时全链路
ros2 launch armor_plate_bringup run.launch.py

# 视频回放测试
ros2 launch armor_plate_bringup test.launch.py video_path:=/path/to/video.mp4

# 自动化无头测试（150 帧自动退出）
ros2 launch armor_plate_bringup auto_test.launch.py
```

#### 单独启动

```bash
# 主程序（需连接相机）
ros2 launch armor_plate_identification run.launch.py

# 离线测试（视频文件）
ros2 launch armor_plate_identification test.launch.py video_path:=/path/to/video.mp4

# Tracker / 串口
ros2 launch armor_plate_tracker run.launch.py
ros2 run armor_plate_serial serial_node
```

---

## 参数配置

识别参数集中在 `armor_plate_identification/config/params.yaml`：

| 参数 | 类型 | 说明 |
|------|------|------|
| `target_color` | string | `"RED"` / `"BLUE"` |
| `camera_type` | string | `"mindvision"` / `"galaxy"` |
| `exposure_time` | float | 曝光时间（μs） |
| `gain` | float | 增益 |
| `debug_base` | bool | 基础调试（图像显示、键盘监听） |
| `debug_identification` | bool | 绘制检测框与参数 |
| `debug_preprocessing` | bool | 显示预处理中间结果 |
| `debug_number_classification` | bool | 显示数字识别结果 |

**灯条匹配参数**（支持运行时键盘实时调节 1-6 键 + T/G）：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `max_angle_diff` | 10.0° | 最大角度差 |
| `max_y_diff_ratio` | 1.0 | 最大 Y 方向高度差与灯条长度比 |
| `min_distance_ratio` | 0.1 | 最小中心距与灯条长度比 |
| `max_distance_ratio` | 0.8 | 最大中心距与灯条长度比 |
| `min_length_ratio` | 0.7 | 最小长度比（短/长） |
| `min_x_diff_ratio` | 0.75 | 最小 X 方向间距与灯条长度比 |

Tracker 参数在 `armor_plate_tracker/config/params.yaml`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_lost_time` | 0.5 | 丢失超时（秒） |
| `mutation_yaw_threshold` | 5° | 突变检测阈值（度） |

---

## 目录结构

```
Visual-Translationo/
├── src/
│   ├── armor_plate_bringup/           # 一键启动
│   ├── armor_plate_identification/    # 识别主包（相机/视频 + PnP + 数字识别）
│   │   ├── config/params.yaml
│   │   ├── launch/
│   │   ├── src/
│   │   ├── model/number_cnn.onnx      # ONNX 数字识别模型
│   │   ├── video/                     # 测试视频
│   │   └── third_parties/             # MindVision / Galaxy SDK
│   ├── armor_plate_tracker/           # 跟踪 + 世界坐标系 EKF
│   │   ├── config/params.yaml
│   │   ├── src/
│   │   │   ├── CoordinateTransformer.cpp  # 坐标变换（camera↔world）
│   │   │   ├── Tracker.cpp                # 目标选择 + 跟踪逻辑
│   │   │   └── MyExtendedKalmanFilter.cpp # 11 状态 EKF
│   │   └── launch/
│   ├── armor_plate_serial/            # 串口双向通信
│   └── armor_plate_interfaces/        # 自定义消息
├── DeepLearning/                      # 数字识别模型训练工具链
│   ├── src/
│   │   ├── dataset.py                 # 数据集加载
│   │   ├── model.py                   # Zenet + Lenet5 模型定义
│   │   └── train.py                   # 训练 + ONNX 导出
│   ├── example/rm_vision/             # rm_auto_aim 参考实现
│   ├── dataset/armors/                # 训练数据
│   └── output/                        # 已训练模型
├── Debug/                             # 调试脚本与分析数据
│   ├── analyze_tracker.py             # Tracker 数据分析 + 可视化
│   └── analyze_identification.py      # PnP 质量分析
└── 视觉电控协议/                       # 串口协议文档
```

---

## 数字识别模型训练

```bash
cd DeepLearning
pip install -r requirements.txt
python src/train.py    # 数据增强 → 训练 → 导出 ONNX
```

详见 `DeepLearning/README.md`。
