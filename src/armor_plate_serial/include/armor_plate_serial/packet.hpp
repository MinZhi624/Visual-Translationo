#pragma once
#include <cstdint>
// 数据结构
typedef struct __attribute__((packed))
{
    uint8_t sof1;
    uint8_t sof2;
    uint8_t seq;
    int16_t delta_yaw_1e4rad;
    int16_t delta_pitch_1e4rad;
    uint16_t crc16;
} VisionToEcFrame_t;
typedef struct __attribute__((packed))
{
    uint8_t sof1;
    uint8_t sof2;
    uint8_t seq_echo;
    int32_t yaw_actual_1e4rad;
    int32_t pitch_actual_1e4rad;
    uint16_t crc16;
} EcToVisionFrame_t;