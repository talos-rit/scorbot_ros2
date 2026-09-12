// Consistent Overhead Byte Stuffing (COBS).
//
// Encodes a byte string so that it contains no 0x00 bytes, at a cost of one extra byte
// per 254 input bytes. A 0x00 can then be used as a frame delimiter on a byte stream
// and a receiver can always resynchronise on the next delimiter. Header-only, no
// allocation, usable from firmware and from the Pi.

#pragma once

#include <cstddef>
#include <cstdint>

namespace scorbot_protocol::cobs
{

constexpr std::size_t npos = static_cast<std::size_t>(-1);

/// Upper bound on the encoded size of `n` input bytes (without a delimiter).
constexpr std::size_t maxEncodedSize(std::size_t n) { return n + n / 254 + 1; }

/// Encode `n` bytes from `in` into `out` (capacity `cap`). No delimiter is appended.
/// Returns the encoded length, or 0 if `out` is too small.
inline std::size_t encode(const uint8_t* in, std::size_t n, uint8_t* out, std::size_t cap)
{
  if (cap == 0)
    return 0;
  std::size_t code_idx = 0;  // where the current block's length byte goes
  std::size_t o = 1;         // next output position
  uint8_t code = 1;          // current block length (data bytes + 1)
  bool open = true;          // a code slot is reserved at code_idx

  for (std::size_t i = 0; i < n; ++i)
  {
    if (!open)
    {
      // Previous block was a full 254 bytes and more input follows: start a new block.
      code_idx = o++;
      if (o > cap)
        return 0;
      code = 1;
      open = true;
    }
    const uint8_t b = in[i];
    if (b == 0)
    {
      out[code_idx] = code;
      code_idx = o++;
      if (o > cap)
        return 0;
      code = 1;
    }
    else
    {
      if (o >= cap)
        return 0;
      out[o++] = b;
      if (++code == 0xFF)
      {
        out[code_idx] = code;
        open = false;
      }
    }
  }
  if (open)
    out[code_idx] = code;
  return o;
}

/// Decode `n` COBS bytes (no delimiter inside) from `in` into `out` (capacity `cap`).
/// Returns the decoded length, or `npos` if the input is malformed or `out` too small.
inline std::size_t decode(const uint8_t* in, std::size_t n, uint8_t* out, std::size_t cap)
{
  std::size_t i = 0, o = 0;
  bool first = true;  // no implied zero before the first block or after a full block
  while (i < n)
  {
    const uint8_t code = in[i++];
    if (code == 0)
      return npos;
    if (!first)
    {
      if (o >= cap)
        return npos;
      out[o++] = 0;
    }
    const std::size_t len = static_cast<std::size_t>(code) - 1;
    if (i + len > n || o + len > cap)
      return npos;
    for (std::size_t k = 0; k < len; ++k)
    {
      const uint8_t b = in[i++];
      if (b == 0)
        return npos;
      out[o++] = b;
    }
    first = (code == 0xFF);
  }
  return o;
}

}  // namespace scorbot_protocol::cobs
