// CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, no reflection, no final xor.
// Check value for the ASCII string "123456789" is 0x29B1.
//
// Bitwise implementation on purpose: no table to keep in sync between firmware and
// host, and a 100-byte frame at 100 Hz is far below where a table would matter.

#pragma once

#include <cstddef>
#include <cstdint>

namespace scorbot_protocol
{

inline uint16_t crc16Update(uint16_t crc, uint8_t byte)
{
  crc ^= static_cast<uint16_t>(byte) << 8;
  for (int bit = 0; bit < 8; ++bit)
    crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  return crc;
}

inline uint16_t crc16(const uint8_t* data, std::size_t n, uint16_t crc = 0xFFFF)
{
  for (std::size_t i = 0; i < n; ++i)
    crc = crc16Update(crc, data[i]);
  return crc;
}

}  // namespace scorbot_protocol
