# RoboMaster 装甲板视觉识别系统

基于 **ROS 2 Humble** + **OpenCV 4.x** 的 RoboMaster 装甲板识别、跟踪与瞄准解算方案。

---

## 项目简介

本项目实现了一套完整的装甲板视觉处理链路，涵盖图像采集、预处理、灯条检测与匹配、PnP 位姿解算、卡尔曼滤波跟踪以及串口通信。系统采用模块化 ROS 2 功能包设计，支持离线视频调试与在线相机实时运行两种模式。

### 核心处理流程

```
相机采图 → 颜色分割/二值化 → 灯条检测与配对 → PnP 解算 → 目标跟踪(KF) → 串口发送(yaw/pitch)
```

---

## 系统架构

### 功能包说明

| 功能包 | 职责 | 节点/入口 |
|--------|------|-----------|
| `armor_plate_identification` | 图像采集、预处理、灯条检测、PnP 解算、结果发布 | `ArmorPlateIdentifcation` (相机) / `Test` (离线视频) |
| `armor_plate_tracker` | 订阅识别结果，进行目标选择与卡尔曼滤波 | `armor_plate_tracker_node` |
| `armor_plate_data_visualization` | 实时绘制 yaw/pitch 原始值与滤波值曲线 | `data_visualization_node` |
| `armor_plate_serial` | 订阅瞄准指令，按电控协议 CRC16 校验后 100Hz 串口发送 | `serial_node` |
| `armor_plate_interfaces` | 自定义 ROS 2 消息定义 (`ArmorPlates`, `DebugTracker` 等) | — |
| `armor_plate_bringup` | 一键启动多节点组合 | `bringup.launch.py` |

### 节点数据流

```
Camera/Video
      │
      ▼
┌─────────────────────────────┐
│ armor_plate_identification  │  → /armor_plates (ArmorPlates)
│    (识别 + PnP 解算)         │  → /tf (可选)
└─────────────────────────────┘
      │
      ▼
┌─────────────────────────────┐
│   armor_plate_tracker       │  → /debug_tracker (DebugTracker)
│    (目标跟踪 + 卡尔曼滤波)    │  → /aim_command
└─────────────────────────────┘
      │
      ├──────────────┐
      ▼              ▼
┌────────────┐  ┌─────────────────────────┐
│   serial   │  │  data_visualization     │
│   (串口)   │  │  (yaw/pitch 实时曲线)    │
└────────────┘  └─────────────────────────┘
```

---

## 技术特点

### 1. 预处理：灰度找亮点 + 通道差
- 先找到灰度图二值化找到亮点
- 放弃 HSV，采用 **R-B / B-R 通道差值** 增强目标颜色、抑制背景
- 过曝场景下光晕更小，边界更清晰
- 支持通过 `target_color` 参数动态切换红/蓝方

### 2. 灯条检测与匹配

- **轮廓筛选**：面积 + 长宽比约束剔除噪声
- **直线提取**：`fitEllipse` 提供精确方向，`minAreaRect` 提供长度约束，两者结合提取灯条中轴线
- **几何约束配对**：6 个经验参数控制（角度差、长度比、中心距比、y 差比、x 差比等）
- **打分 + NMS**：对候选装甲板按平行度、长度相似度、形状分排序，并做非极大值抑制，保证一个灯条只归属一个装甲板

### 3. PnP 位姿解算

- 基于 `cv::solvePnP`，采用 **双向量法** 直接计算瞄准角：
  ```cpp
  yaw   = -atan2(tx, tz) * 180 / PI;      // 左正右负，与电控协议对齐
  pitch = atan2(-ty, sqrt(tx^2 + tz^2)) * 180 / PI;
  distance = norm(tvec);
  ```
- 主程序通过 `image_transport::CameraSubscriber` **动态读取 `CameraInfo`**，自动获取相机内参与畸变系数

### 4. 目标跟踪与卡尔曼滤波

- 对 `yaw` 和 `pitch` 分别建立 **3 阶卡尔曼滤波模型**（位置-速度-加速度）
- 目标选择策略：
  - 未初始化时优先选择图像中心距离最近的装甲板
  - 已跟踪时选择与滤波预测值最接近的装甲板
- **突变检测**：测量值与预测差异过大时自动重置滤波器
- **丢失处理**：连续 0.5s 未检测到目标时重置跟踪状态

### 5. Debug 可视化

- **Tracker 图像叠加**：订阅 `debug_image`，绘制测量红点、滤波绿点，并在灯条旁标注 yaw/pitch 数值
- **DataVisualization**：上下分屏实时绘制 yaw/pitch 曲线（白=原始，紫=滤波）
- **去畸变 + 降采样**：`debug_image` 统一先 `undistort` 再 `resize(0.5x)`，Tracker 内参自动 `*0.5` 保证重投影像素严格对齐

### 6. 串口通信

- 基于 `serial_driver` 实现串口打开/关闭/自动重连
- **CRC16/MODBUS 查表法** 校验
- 独立发送线程，`rclcpp::Rate(100)` 严格 100Hz
- 无目标时自动停发，符合电控协议要求

---

## 快速开始

### 环境依赖

- Ubuntu 22.04
- ROS 2 Humble
- OpenCV 4.x
- Eigen3
- MindVision 相机 SDK（已包含在 `third_party/camera_sdk/`）

### 编译

```bash
cd /home/minzhi/ws05_fourth_assessment

# 全量编译
colcon build --packages-select \
  armor_plate_interfaces \
  armor_plate_identification \
  armor_plate_tracker \
  armor_plate_data_visualization \
  armor_plate_serial \
  armor_plate_bringup

source install/setup.bash
```


### 运行

#### 一键启动（推荐）

```bash
ros2 launch armor_plate_bringup bringup.launch.py
```

#### 单独启动

**主程序（需连接相机）**
```bash
ros2 launch armor_plate_identification run.launch.py
```

**离线测试（视频文件）**
```bash
ros2 launch armor_plate_identification test.launch.py
# 或指定视频路径
ros2 launch armor_plate_identification test.launch.py video_path:=/path/to/video.mp4
```

**Tracker / 可视化 / 串口**
```bash
ros2 launch armor_plate_tracker run.launch.py
ros2 run armor_plate_data_visualization data_visualization_node
ros2 run armor_plate_serial serial_node
```

---

## 参数配置

全局参数集中在 `armor_plate_identification/config/params.yaml`，主要包括：

| 参数 | 类型 | 说明 |
|------|------|------|
| `target_color` | string | `"RED"` / `"BLUE"`，切换识别颜色 |
| `debug_base` | bool | 是否开启基础调试功能（图像显示、键盘监听） |
| `debug_identification` | bool | 是否在图像上绘制检测框和参数信息 |
| `debug_preprocessing` | bool | 是否显示预处理中间结果（2x2 网格） |

**灯条匹配参数**（支持 Test/主程序运行时键盘实时调节）：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `MAX_ANGLE_DIFF` | 10.0° | 最大角度差 |
| `MAX_Y_DIFF_RATIO` | 1.0 | 最大 Y 方向高度差与灯条长度比 |
| `MIN_DISTANCE_RATIO` | 0.1 | 最小中心距倒数与灯条长度比 |
| `MAX_DISTANCE_RATIO` | 0.8 | 最大中心距倒数与灯条长度比 |
| `MIN_LENGTH_RATIO` | 0.7 | 最小长度比（短/长） |
| `MIN_X_DIFF_RATIO` | 0.75 | 最小 X 方向间距与灯条长度比 |

Tracker 参数在 `armor_plate_tracker/config/`（如有）或代码默认值中配置，包括滤波噪声协方差、突变阈值、丢失超时时间等。

---

## 目录结构

```
ws05_fourth_assessment/
├── src/
│   ├── armor_plate_bringup/           # 一键启动
│   ├── armor_plate_identification/    # 识别主包（相机/视频 + PnP）
│   │   ├── config/params.yaml
│   │   ├── launch/
│   │   ├── src/
│   │   └── third_party/camera_sdk/    # MindVision SDK
│   ├── armor_plate_tracker/           # 跟踪 + 卡尔曼滤波
│   ├── armor_plate_data_visualization/# 数据可视化
│   └── armor_plate_interfaces/        # 自定义消息
├── build/
├── install/
└── Camera/                            # 相机配置
```

---
