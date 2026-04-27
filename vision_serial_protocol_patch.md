 视觉侧串口协议适配修改说明

目标：适配电控当前 10 字节协议，并支持哨兵模式。

电控当前要求视觉发送：

```text
A5 5A | seq uint8 | target_valid uint8 | delta_yaw int16 | delta_pitch int16 | CRC16
```

其中：

- `target_valid = 1`：当前有目标，电控使用 `delta_yaw / delta_pitch` 跟踪
- `target_valid = 0`：当前无目标，视觉侧必须把 `delta_yaw / delta_pitch` 填 0，电控进入扫描
- `seq` 每发送一帧递增
- CRC16/MODBUS 计算前 8 字节

下面是建议修改 diff。

## 1. 修改 AimCommand 消息

文件：`src/armor_plate_interfaces/msg/AimCommand.msg`

```diff
 # 这里的单位是 Rad
+bool target_valid
 float32 delta_yaw
 float32 delta_pitch
```

原因：

- 只靠 `delta_yaw / delta_pitch` 无法判断有没有目标
- `delta_yaw = 0` 或 `delta_pitch = 0` 也可能是有效目标
- 所以必须显式传 `target_valid`

## 2. 修改 tracker 发布逻辑

文件：`src/armor_plate_tracker/src/ArmorPlateTracker.cpp`

当前逻辑大概是：

```cpp
if (tracker_.isLost()) {
    RCLCPP_WARN(this->get_logger(), "目标丢失");
    return;
} else {
    AimCommand aim_command;
    aim_command.delta_pitch = tracker_.getPitch();
    aim_command.delta_yaw = tracker_.getYaw();
    aim_command_pub_->publish(aim_command);
}
```

建议改成：

```diff
-        if (tracker_.isLost()) {
-            RCLCPP_WARN(this->get_logger(), "目标丢失");
-            return;
-        } else {
-            AimCommand aim_command;
-            aim_command.delta_pitch = tracker_.getPitch();
-            aim_command.delta_yaw = tracker_.getYaw();
-            aim_command_pub_->publish(aim_command);
-        }
+        AimCommand aim_command;
+        if (tracker_.isLost()) {
+            RCLCPP_WARN(this->get_logger(), "目标丢失");
+            aim_command.target_valid = false;
+            aim_command.delta_yaw = 0.0f;
+            aim_command.delta_pitch = 0.0f;
+        } else {
+            aim_command.target_valid = true;
+            aim_command.delta_yaw = tracker_.getYaw();
+            aim_command.delta_pitch = tracker_.getPitch();
+        }
+        aim_command_pub_->publish(aim_command);
```

原因：

- 目标丢失时也要通知电控 `target_valid = 0`
- 否则电控只能靠超时判断，状态切换不够明确

## 3. 修改 serial_node 的缓存变量

文件：`src/armor_plate_serial/src/serial_node.cpp`

当前成员变量大概是：

```cpp
float latest_yaw_ = 0.0f;
float latest_pitch_ = 0.0f;
uint8_t latest_seq_ = 0;
bool has_target_ = false;
std::mutex data_mutex_;
```

建议改成：

```diff
     float latest_yaw_ = 0.0f;
     float latest_pitch_ = 0.0f;
     uint8_t latest_seq_ = 0;
-    bool has_target_ = false;
+    bool latest_target_valid_ = false;
+    rclcpp::Time latest_cmd_time_;
     std::mutex data_mutex_;
```

原因：

- 不应该用 `has_target_` 表示“一次性事件”
- serial 线程应该持续发送最新视觉状态
- `latest_cmd_time_` 用于判断视觉数据是否过期

## 4. 修改 AimCommand 回调

当前逻辑大概是：

```cpp
[this](const AimCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_yaw_ = msg->delta_yaw;
    latest_pitch_ = msg->delta_pitch;
    has_target_ = true;
}
```

建议改成：

```diff
         [this](const AimCommand::SharedPtr msg) {
             std::lock_guard<std::mutex> lock(data_mutex_);
+            latest_target_valid_ = msg->target_valid;
             latest_yaw_ = msg->delta_yaw;
             latest_pitch_ = msg->delta_pitch;
-            has_target_ = true;
+            latest_cmd_time_ = this->now();
         }
```

原因：

- 是否有目标由 `msg->target_valid` 决定
- 不要再用 yaw/pitch 是否为 0 判断

## 5. 修改发送循环 target_valid 逻辑

当前逻辑里有这一段：

```cpp
if (has_target_ && latest_yaw_ != 0.0f && latest_pitch_!= 0.0f) {
    target_valid = 1;
    yaw = latest_yaw_;
    pitch = latest_pitch_;
    has_target_ = false;
}
```

这段需要删除，改成：

```diff
             float yaw = 0.0f;
             float pitch = 0.0f;
             uint8_t target_valid = 0;
             {
                 std::lock_guard<std::mutex> lock(data_mutex_);
-                if (has_target_ && latest_yaw_ != 0.0f && latest_pitch_!= 0.0f) {
-                    target_valid = 1;
-                    yaw = latest_yaw_;
-                    pitch = latest_pitch_;
-                    has_target_ = false;
-                }
+                double age_s = (this->now() - latest_cmd_time_).seconds();
+                bool valid = latest_target_valid_ && (age_s < 0.2);
+                if (valid) {
+                    target_valid = 1;
+                    yaw = latest_yaw_;
+                    pitch = latest_pitch_;
+                } else {
+                    target_valid = 0;
+                    yaw = 0.0f;
+                    pitch = 0.0f;
+                }
             }
```

原因：

- `yaw = 0` 或 `pitch = 0` 也可能是有效目标
- 有目标时应该连续发送最新 delta
- 超过 200ms 没有新的视觉命令，再认为无目标

## 6. 串口发送改成 sendAll

当前逻辑：

```cpp
size_t sent = serial_driver_->port()->send(data);
if (sent != data.size()) {
    RCLCPP_ERROR(...);
}
```

问题：

- `send(data)` 不保证一次写完所有字节
- 如果只发出部分字节，会导致电控收到半帧，出现 CRC 错误

建议在 `serial_node.cpp` 里新增函数：

```cpp
bool sendAll(const std::vector<uint8_t>& data)
{
    size_t total = 0;

    while (total < data.size()) {
        std::vector<uint8_t> remain(data.begin() + total, data.end());
        size_t sent = serial_driver_->port()->send(remain);

        if (sent == 0) {
            return false;
        }

        total += sent;
    }

    return true;
}
```

然后把发送部分改成：

```diff
-                size_t sent = serial_driver_->port()->send(data);
-                if (sent != data.size()) {
-                    RCLCPP_ERROR(
-                        this->get_logger(), "发送不完整: 期望 %zu, 实际 %zu", data.size(), sent);
-                } else {
+                if (!sendAll(data)) {
+                    RCLCPP_ERROR(this->get_logger(), "serial send failed");
+                } else {
                     RCLCPP_INFO(this->get_logger(), "发送数据: valid=%d, yaw=%f, pitch=%f",
                                 target_valid, yaw, pitch);
                 }
```





