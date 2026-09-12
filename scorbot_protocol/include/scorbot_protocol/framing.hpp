// Serial framing: frame := COBS(payload || crc16_be) || 0x00
//
// The payload is one encoded scorbot.v1.Envelope. The CRC covers the payload bytes and
// is appended big-endian before COBS encoding, so the whole frame is zero-free until the
// delimiter. StreamDecoder turns an arbitrary byte stream (split anywhere, with garbage
// or partial frames in it) back into validated payloads without allocating.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "scorbot_protocol/cobs.hpp"
#include "scorbot_protocol/crc16.hpp"

namespace scorbot_protocol
{

/// Largest payload (encoded Envelope) the framing accepts. messages.hpp asserts that
/// the nanopb-computed maximum Envelope size fits.
constexpr std::size_t kMaxPayload = 1024;
/// Payload plus CRC after COBS plus the delimiter.
constexpr std::size_t kMaxFrame = cobs::maxEncodedSize(kMaxPayload + 2) + 1;
constexpr uint8_t kDelimiter = 0x00;

/// Build a wire frame for `payload` into `out` (capacity `cap`), delimiter included.
/// Returns the frame length, or 0 if the payload is too large or `out` too small.
inline std::size_t encodeFrame(const uint8_t* payload, std::size_t n, uint8_t* out, std::size_t cap)
{
  if (n > kMaxPayload || cap == 0)
    return 0;
  std::array<uint8_t, kMaxPayload + 2> with_crc{};
  for (std::size_t i = 0; i < n; ++i)
    with_crc[i] = payload[i];
  const uint16_t crc = crc16(payload, n);
  with_crc[n] = static_cast<uint8_t>(crc >> 8);
  with_crc[n + 1] = static_cast<uint8_t>(crc & 0xFF);

  const std::size_t encoded = cobs::encode(with_crc.data(), n + 2, out, cap - 1);
  if (encoded == 0)
    return 0;
  out[encoded] = kDelimiter;
  return encoded + 1;
}

struct DecoderStats
{
  uint64_t frames{0};        ///< valid payloads delivered
  uint64_t bytes{0};         ///< raw bytes consumed
  uint64_t crc_errors{0};
  uint64_t cobs_errors{0};
  uint64_t short_frames{0};  ///< decoded to fewer than 2 bytes (no room for a CRC)
  uint64_t overflows{0};     ///< frame longer than kMaxFrame, discarded up to the next delimiter
};

/// Incremental frame decoder. Feed it bytes in any chunking; it calls back once per
/// valid payload. Anything malformed is counted and skipped, and decoding resumes at
/// the next delimiter.
class StreamDecoder
{
public:
  template <typename OnPayload>
  void feed(const uint8_t* data, std::size_t n, OnPayload&& on_payload)
  {
    stats_.bytes += n;
    for (std::size_t i = 0; i < n; ++i)
    {
      const uint8_t b = data[i];
      if (b != kDelimiter)
      {
        if (len_ < buf_.size())
          buf_[len_++] = b;
        else
          overflow_ = true;
        continue;
      }

      if (overflow_)
        ++stats_.overflows;
      else if (len_ > 0)
        finishFrame(on_payload);
      // len_ == 0: back-to-back delimiters, idle line; nothing to do.
      len_ = 0;
      overflow_ = false;
    }
  }

  const DecoderStats& stats() const { return stats_; }
  void reset()
  {
    len_ = 0;
    overflow_ = false;
    stats_ = DecoderStats{};
  }

private:
  template <typename OnPayload>
  void finishFrame(OnPayload&& on_payload)
  {
    const std::size_t decoded = cobs::decode(buf_.data(), len_, payload_.data(), payload_.size());
    if (decoded == cobs::npos)
    {
      ++stats_.cobs_errors;
      return;
    }
    if (decoded < 2)
    {
      ++stats_.short_frames;
      return;
    }
    const std::size_t n = decoded - 2;
    const uint16_t expected = static_cast<uint16_t>((payload_[n] << 8) | payload_[n + 1]);
    if (crc16(payload_.data(), n) != expected)
    {
      ++stats_.crc_errors;
      return;
    }
    ++stats_.frames;
    on_payload(payload_.data(), n);
  }

  std::array<uint8_t, kMaxFrame> buf_{};
  std::array<uint8_t, kMaxPayload + 2> payload_{};
  std::size_t len_{0};
  bool overflow_{false};
  DecoderStats stats_{};
};

}  // namespace scorbot_protocol
