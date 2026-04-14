#pragma once

#include <cstdint>
#include <cstddef>

uint16_t crc16_modbus(const uint8_t * data, size_t len);
