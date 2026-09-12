#include <gtest/gtest.h>

#include <cstring>
#include <random>
#include <vector>

#include "scorbot_protocol/crc16.hpp"
#include "scorbot_protocol/framing.hpp"

using namespace scorbot_protocol;

namespace
{
std::vector<uint8_t> frame(const std::vector<uint8_t>& payload)
{
  std::vector<uint8_t> out(kMaxFrame);
  const std::size_t n = encodeFrame(payload.data(), payload.size(), out.data(), out.size());
  out.resize(n);
  return out;
}

struct Collector
{
  std::vector<std::vector<uint8_t>> payloads;
  void operator()(const uint8_t* p, std::size_t n) { payloads.emplace_back(p, p + n); }
};
}  // namespace

TEST(Crc16, CheckValue)
{
  const char* s = "123456789";
  EXPECT_EQ(crc16(reinterpret_cast<const uint8_t*>(s), 9), 0x29B1);
  EXPECT_EQ(crc16(nullptr, 0), 0xFFFF);
}

TEST(Framing, FrameLayout)
{
  const std::vector<uint8_t> payload = {0x08, 0x01, 0x10, 0x02};
  const auto f = frame(payload);
  ASSERT_FALSE(f.empty());
  EXPECT_EQ(f.back(), kDelimiter);
  for (std::size_t i = 0; i + 1 < f.size(); ++i)
    EXPECT_NE(f[i], 0) << "no zeros before the delimiter";

  // COBS-decode by hand and check payload + big-endian CRC.
  std::vector<uint8_t> decoded(f.size());
  const std::size_t n = cobs::decode(f.data(), f.size() - 1, decoded.data(), decoded.size());
  ASSERT_EQ(n, payload.size() + 2);
  EXPECT_TRUE(std::equal(payload.begin(), payload.end(), decoded.begin()));
  const uint16_t crc = crc16(payload.data(), payload.size());
  EXPECT_EQ(decoded[n - 2], crc >> 8);
  EXPECT_EQ(decoded[n - 1], crc & 0xFF);
}

TEST(Framing, EmptyPayloadIsAFrame)
{
  const auto f = frame({});
  ASSERT_FALSE(f.empty());
  StreamDecoder d;
  Collector c;
  d.feed(f.data(), f.size(), c);
  ASSERT_EQ(c.payloads.size(), 1u);
  EXPECT_TRUE(c.payloads[0].empty());
}

TEST(Framing, ConcatenatedFramesAndArbitrarySplits)
{
  std::vector<std::vector<uint8_t>> payloads = {{1, 2, 3}, {0, 0, 0, 0}, {}, {0xFF, 0x00, 0x7F}};
  std::vector<uint8_t> stream;
  for (const auto& p : payloads)
  {
    const auto f = frame(p);
    stream.insert(stream.end(), f.begin(), f.end());
  }
  // Byte by byte
  {
    StreamDecoder d;
    Collector c;
    for (uint8_t b : stream)
      d.feed(&b, 1, c);
    EXPECT_EQ(c.payloads, payloads);
    EXPECT_EQ(d.stats().frames, payloads.size());
  }
  // All at once
  {
    StreamDecoder d;
    Collector c;
    d.feed(stream.data(), stream.size(), c);
    EXPECT_EQ(c.payloads, payloads);
  }
  // Random chunking
  std::mt19937 rng(3);
  for (int trial = 0; trial < 50; ++trial)
  {
    StreamDecoder d;
    Collector c;
    std::size_t i = 0;
    while (i < stream.size())
    {
      const std::size_t n = std::min<std::size_t>(1 + rng() % 7, stream.size() - i);
      d.feed(stream.data() + i, n, c);
      i += n;
    }
    EXPECT_EQ(c.payloads, payloads);
  }
}

TEST(Framing, ResyncAfterGarbageCorruptionAndPartialFrames)
{
  const std::vector<uint8_t> good = {10, 20, 30, 40, 50};
  const auto f = frame(good);

  std::vector<uint8_t> stream = {0x55, 0x66, 0x77};              // garbage, no delimiter
  stream.insert(stream.end(), f.begin(), f.end());               // -> garbage+frame fails CRC
  stream.insert(stream.end(), f.begin(), f.begin() + 3);         // partial frame
  stream.push_back(kDelimiter);                                  // cut short -> cobs/crc error
  auto corrupt = f;
  corrupt[2] ^= 0x10;                                            // flipped bit -> CRC error
  stream.insert(stream.end(), corrupt.begin(), corrupt.end());
  stream.push_back(kDelimiter);                                  // idle delimiters are fine
  stream.push_back(kDelimiter);
  stream.insert(stream.end(), f.begin(), f.end());               // finally a clean frame

  StreamDecoder d;
  Collector c;
  d.feed(stream.data(), stream.size(), c);
  ASSERT_EQ(c.payloads.size(), 1u);
  EXPECT_EQ(c.payloads[0], good);
  EXPECT_EQ(d.stats().frames, 1u);
  EXPECT_GE(d.stats().crc_errors + d.stats().cobs_errors + d.stats().short_frames, 3u);
}

TEST(Framing, OverflowIsDiscardedUpToNextDelimiter)
{
  std::vector<uint8_t> junk(kMaxFrame + 100, 0x42);
  junk.push_back(kDelimiter);
  const std::vector<uint8_t> good = {7, 8, 9};
  const auto f = frame(good);
  junk.insert(junk.end(), f.begin(), f.end());

  StreamDecoder d;
  Collector c;
  d.feed(junk.data(), junk.size(), c);
  EXPECT_EQ(d.stats().overflows, 1u);
  ASSERT_EQ(c.payloads.size(), 1u);
  EXPECT_EQ(c.payloads[0], good);
}

TEST(Framing, MaxPayloadFitsAndLargerIsRejected)
{
  std::vector<uint8_t> big(kMaxPayload, 0x00);
  EXPECT_FALSE(frame(big).empty());
  std::vector<uint8_t> too_big(kMaxPayload + 1, 0x01);
  EXPECT_TRUE(frame(too_big).empty());

  StreamDecoder d;
  Collector c;
  const auto f = frame(big);
  d.feed(f.data(), f.size(), c);
  ASSERT_EQ(c.payloads.size(), 1u);
  EXPECT_EQ(c.payloads[0], big);
}

TEST(Framing, FuzzNeverCrashes)
{
  std::mt19937 rng(11);
  StreamDecoder d;
  Collector c;
  std::vector<uint8_t> buf(512);
  for (int round = 0; round < 3000; ++round)
  {
    const std::size_t n = rng() % buf.size();
    for (std::size_t i = 0; i < n; ++i)
      buf[i] = (rng() % 5 == 0) ? 0 : static_cast<uint8_t>(rng());
    d.feed(buf.data(), n, c);
  }
  // Any accepted payload must have had a valid CRC, which is astronomically unlikely
  // for random data of this volume; mostly this test proves we survive.
  EXPECT_LE(c.payloads.size(), 2u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
