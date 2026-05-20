# 视觉与电控最小通信协议

## 基本约定

- 角度单位：`0.0001 rad`
- 整数类型：有符号整数使用补码
- 字节序：小端序，低字节在前
- CRC：`CRC16/MODBUS`

换算关系：

```text
angle_rad = angle_1e4rad / 10000.0
angle_deg = angle_rad * 57.29577951
```

例如：

- `10` 表示 `0.001 rad`
- `-25` 表示 `-0.0025 rad`

## 正方向约定

视觉侧按下面方向发送：

- `delta_yaw > 0`：从上往下看，云台逆时针
- `delta_pitch > 0`：相机抬头

## 视觉发给电控

视觉发给电控的是增量角，不是绝对角。

固定 10 字节：

```text
Byte0   Byte1   Byte2   Byte3          Byte4~5      Byte6~7        Byte8~9
SOF1    SOF2    seq     target_valid   delta_yaw    delta_pitch    CRC16
```

字段定义：

- `SOF1 = 0xA5`
- `SOF2 = 0x5A`
- `seq`：`uint8_t`，帧序号
- `target_valid`：`uint8_t`，`1` 表示当前有目标，`0` 表示当前无目标
- `delta_yaw`：`int16_t`，单位 `0.0001 rad`
- `delta_pitch`：`int16_t`，单位 `0.0001 rad`
- `CRC16`：`uint16_t`，`CRC16/MODBUS`

对应 C 结构体：

```c
typedef struct __attribute__((packed))
{
    uint8_t sof1;
    uint8_t sof2;
    uint8_t seq;
    uint8_t target_valid;
    int16_t delta_yaw_1e4rad;
    int16_t delta_pitch_1e4rad;
    uint16_t crc16;
} VisionToEcFrame_t;
```

### seq 规则

- `seq` 从 `0` 到 `255` 循环递增
- 视觉侧每发送一帧，`seq` 加一
- 电控侧收到合法帧后，记录最近一次采用的 `seq`
- 电控回传时把最近一次采用的 `seq` 放到 `seq_echo`

`seq` 的作用：

- 视觉可以确认电控最后收到了哪一帧
- 联调时可以判断是否丢帧
- 联调时可以判断电控是否一直收到旧帧或重复帧

### target_valid 规则

- `target_valid = 1`：当前有有效目标，电控使用 `delta_yaw / delta_pitch` 生成新目标
- `target_valid = 0`：当前无有效目标，视觉侧应把 `delta_yaw / delta_pitch` 填 `0`
- `target_valid = 0` 的帧仍然是合法帧，`seq` 仍然递增
- 电控收到 `target_valid = 0` 后，不应继续用该帧生成新的瞄准目标
- 电控对无目标的具体动作由电控策略决定，例如保持当前位置、保持上一次目标或进入搜索模式

## 电控对命令的解释

电控收到合法帧且 `target_valid = 1` 后，按当前实际角度生成目标角：

```text
yaw_target   = current_yaw   + delta_yaw
pitch_target = current_pitch + delta_pitch
```

其中：

- 每个合法新帧只消费一次
- 若视觉暂时不发新帧，电控保持已有目标，不继续重复叠加旧增量
- 若 `target_valid = 0`，电控不使用该帧的 delta 生成新目标

## 电控回传给视觉

电控建议周期性回传当前实际角度。

固定 13 字节：

```text
Byte0   Byte1   Byte2      Byte3~6       Byte7~10        Byte11~12
SOF1    SOF2    seq_echo   yaw_actual    pitch_actual    CRC16
```

字段定义：

- `SOF1 = 0x5A`
- `SOF2 = 0xA5`
- `seq_echo`：`uint8_t`，最近一次被电控采用的视觉命令帧序号
- `yaw_actual`：`int32_t`，单位 `0.0001 rad`
- `pitch_actual`：`int32_t`，单位 `0.0001 rad`
- `CRC16`：`uint16_t`，`CRC16/MODBUS`

对应 C 结构体：

```c
typedef struct __attribute__((packed))
{
    uint8_t sof1;
    uint8_t sof2;
    uint8_t seq_echo;
    int32_t yaw_actual_1e4rad;
    int32_t pitch_actual_1e4rad;
    uint16_t crc16;
} EcToVisionFrame_t;
```

## CRC 规则

类型：

- `CRC16/MODBUS`

参数：

- 初值：`0xFFFF`
- 多项式：`0xA001`
- 输入按字节低位优先处理
- 结果按小端序附到帧尾，即 CRC 低字节在前，高字节在后

### 视觉到电控 CRC 范围

视觉到电控帧长度为 10 字节。

CRC 计算范围是前 8 字节：

```text
SOF1 SOF2 seq target_valid delta_yaw_low delta_yaw_high delta_pitch_low delta_pitch_high
```

不包含最后 2 字节 CRC。

### 电控到视觉 CRC 范围

电控到视觉帧长度为 13 字节。

CRC 计算范围是前 11 字节：

```text
SOF1 SOF2 seq_echo yaw_actual[4] pitch_actual[4]
```

不包含最后 2 字节 CRC。

## CRC 参考代码

### Python

```python
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF
```

### C/C++

```c
#include <stdint.h>
#include <stddef.h>

static uint16_t crc16_modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x0001u)
            {
                crc = (crc >> 1) ^ 0xA001u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}
```

## 视觉侧打包示例

示例输入：

- `seq = 1`
- `target_valid = 1`
- `delta_yaw_1e4rad = 10`
- `delta_pitch_1e4rad = 8`

打包代码：

```c
uint8_t frame[10];
uint8_t seq = 1;
uint8_t target_valid = 1;
int16_t yaw = 10;
int16_t pitch = 8;
uint16_t crc;

frame[0] = 0xA5;
frame[1] = 0x5A;
frame[2] = seq;
frame[3] = target_valid;
frame[4] = (uint8_t)(yaw & 0xFF);
frame[5] = (uint8_t)((yaw >> 8) & 0xFF);
frame[6] = (uint8_t)(pitch & 0xFF);
frame[7] = (uint8_t)((pitch >> 8) & 0xFF);

crc = crc16_modbus(frame, 8);
frame[8] = (uint8_t)(crc & 0xFF);
frame[9] = (uint8_t)((crc >> 8) & 0xFF);
```

最终 10 字节应为：

```text
A5 5A 01 01 0A 00 08 00 48 40
```

## 负数打包示例

示例输入：

- `seq = 2`
- `target_valid = 1`
- `delta_yaw_1e4rad = -10`
- `delta_pitch_1e4rad = -8`

补码表示：

```text
-10 -> 0xFFF6 -> F6 FF
-8  -> 0xFFF8 -> F8 FF
```

最终 10 字节应为：

```text
A5 5A 02 01 F6 FF F8 FF 4C 53
```

如果发出来的是：

- `FF F6`
- `FF F8`

说明 `int16_t` 字节序发反了。

## 无目标打包示例

示例输入：

- `seq = 3`
- `target_valid = 0`
- `delta_yaw_1e4rad = 0`
- `delta_pitch_1e4rad = 0`

最终 10 字节应为：

```text
A5 5A 03 00 00 00 00 00 70 7A
```

## 电控回传示例

示例输入：

- `seq_echo = 1`
- `yaw_actual_1e4rad = 10`
- `pitch_actual_1e4rad = 8`

最终 13 字节应为：

```text
5A A5 01 0A 00 00 00 08 00 00 00 BF 20
```

负数示例：

- `seq_echo = 2`
- `yaw_actual_1e4rad = -10`
- `pitch_actual_1e4rad = -8`

最终 13 字节应为：

```text
5A A5 02 F6 FF FF FF F8 FF FF FF A9 55
```