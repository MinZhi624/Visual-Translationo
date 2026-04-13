#include <cstdint>
#include <cstdio>
#include <cmath>

#include "armor_plate_serial/crc.hpp"
#include "armor_plate_serial/packet.hpp"

void printHex(const uint8_t * data, size_t len)
{
  for (size_t i = 0; i < len; ++i) {
    printf("%02X ", data[i]);
  }
  printf("\n");
}

void packFrame(float yaw_rad, float pitch_rad, VisionToEcFrame_t * frame)
{
  frame->sof1 = 0xA5;
  frame->sof2 = 0x5A;
  frame->delta_yaw_1e4rad = static_cast<int16_t>(yaw_rad * 10000.0f);
  frame->delta_pitch_1e4rad = static_cast<int16_t>(pitch_rad * 10000.0f);
  frame->crc16 = crc16_modbus(reinterpret_cast<uint8_t *>(frame), 6);
}

int main()
{
  VisionToEcFrame_t frame;

  // ===== 样例 1：正数 =====
  // yaw = 0.001 rad, pitch = 0.0008 rad
  // 期望：A5 5A 0A 00 08 00 44 FB
  printf("=== Test 1: positive ===\n");
  packFrame(0.001f, 0.0008f, &frame);
  printf("Expected: A5 5A 0A 00 08 00 44 FB\n");
  printf("Actual:   ");
  printHex(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));

  // ===== 样例 2：负数 =====
  // yaw = -0.001 rad, pitch = -0.0008 rad
  // 期望：A5 5A F6 FF F8 FF 40 DB
  printf("\n=== Test 2: negative ===\n");
  packFrame(-0.001f, -0.0008f, &frame);
  printf("Expected: A5 5A F6 FF F8 FF 40 DB\n");
  printf("Actual:   ");
  printHex(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));

  // ===== 负数补码解释 =====
  printf("\n=== Negative explanation ===\n");
  int16_t neg_yaw = static_cast<int16_t>(-0.001f * 10000.0f);   // -10
  int16_t neg_pitch = static_cast<int16_t>(-0.0008f * 10000.0f); // -8
  printf("-10 as int16_t: 0x%04X -> little endian: %02X %02X\n",
         static_cast<uint16_t>(neg_yaw),
         static_cast<uint8_t>(neg_yaw & 0xFF),
         static_cast<uint8_t>((neg_yaw >> 8) & 0xFF));
  printf("-8  as int16_t: 0x%04X -> little endian: %02X %02X\n",
         static_cast<uint16_t>(neg_pitch),
         static_cast<uint8_t>(neg_pitch & 0xFF),
         static_cast<uint8_t>((neg_pitch >> 8) & 0xFF));

  return 0;
}
