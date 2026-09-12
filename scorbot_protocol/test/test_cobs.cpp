#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "scorbot_protocol/cobs.hpp"

using namespace scorbot_protocol;

namespace
{
std::vector<uint8_t> enc(const std::vector<uint8_t>& in)
{
  std::vector<uint8_t> out(cobs::maxEncodedSize(in.size()));
  const std::size_t n = cobs::encode(in.data(), in.size(), out.data(), out.size());
  out.resize(n);
  return out;
}
std::vector<uint8_t> dec(const std::vector<uint8_t>& in, bool* ok = nullptr)
{
  std::vector<uint8_t> out(in.size() + 1);
  const std::size_t n = cobs::decode(in.data(), in.size(), out.data(), out.size());
  if (ok)
    *ok = (n != cobs::npos);
  if (n == cobs::npos)
    return {};
  out.resize(n);
  return out;
}
std::vector<uint8_t> range(int first, int last)
{
  std::vector<uint8_t> v;
  for (int i = first; i <= last; ++i)
    v.push_back(static_cast<uint8_t>(i));
  return v;
}
}  // namespace

// Vectors from the COBS description (Wikipedia), delimiter excluded.
TEST(Cobs, KnownVectors)
{
  EXPECT_EQ(enc({}), (std::vector<uint8_t>{0x01}));
  EXPECT_EQ(enc({0x00}), (std::vector<uint8_t>{0x01, 0x01}));
  EXPECT_EQ(enc({0x00, 0x00}), (std::vector<uint8_t>{0x01, 0x01, 0x01}));
  EXPECT_EQ(enc({0x00, 0x11, 0x00}), (std::vector<uint8_t>{0x01, 0x02, 0x11, 0x01}));
  EXPECT_EQ(enc({0x11, 0x22, 0x00, 0x33}), (std::vector<uint8_t>{0x03, 0x11, 0x22, 0x02, 0x33}));
  EXPECT_EQ(enc({0x11, 0x22, 0x33, 0x44}), (std::vector<uint8_t>{0x05, 0x11, 0x22, 0x33, 0x44}));
  EXPECT_EQ(enc({0x11, 0x00, 0x00, 0x00}), (std::vector<uint8_t>{0x02, 0x11, 0x01, 0x01, 0x01}));

  // 254 non-zero bytes: one full block, no trailing code.
  std::vector<uint8_t> e254 = {0xFF};
  auto r = range(1, 254);
  e254.insert(e254.end(), r.begin(), r.end());
  EXPECT_EQ(enc(range(1, 254)), e254);

  // 0x00 followed by 254 non-zero bytes.
  std::vector<uint8_t> in = {0x00};
  in.insert(in.end(), r.begin(), r.end());
  std::vector<uint8_t> e = {0x01, 0xFF};
  e.insert(e.end(), r.begin(), r.end());
  EXPECT_EQ(enc(in), e);

  // 255 non-zero bytes: full block then a block holding 0xFF.
  std::vector<uint8_t> e255 = {0xFF};
  e255.insert(e255.end(), r.begin(), r.end());
  e255.push_back(0x02);
  e255.push_back(0xFF);
  EXPECT_EQ(enc(range(1, 255)), e255);
}

TEST(Cobs, RoundTripRandom)
{
  std::mt19937 rng(7);
  for (int n = 0; n < 2000; ++n)
  {
    const std::size_t len = rng() % 700;
    std::vector<uint8_t> in(len);
    const int zero_bias = rng() % 4;  // some messages with many zeros, some with none
    for (auto& b : in)
      b = (zero_bias == 0 && rng() % 3 == 0) ? 0 : static_cast<uint8_t>(rng() % 256);
    const auto encoded = enc(in);
    for (uint8_t b : encoded)
      ASSERT_NE(b, 0) << "encoded output must contain no zero bytes";
    ASSERT_LE(encoded.size(), cobs::maxEncodedSize(in.size()));
    bool ok = false;
    EXPECT_EQ(dec(encoded, &ok), in);
    EXPECT_TRUE(ok);
  }
}

TEST(Cobs, DecodeRejectsMalformed)
{
  bool ok = true;
  dec({0x00}, &ok);
  EXPECT_FALSE(ok) << "zero inside a frame";
  dec({0x05, 0x11}, &ok);
  EXPECT_FALSE(ok) << "block length runs past the input";
  dec({0x02, 0x00}, &ok);
  EXPECT_FALSE(ok) << "zero data byte";
}

TEST(Cobs, EncodeReportsInsufficientCapacity)
{
  const std::vector<uint8_t> in = {0x11, 0x22, 0x00, 0x33};
  uint8_t out[4];
  EXPECT_EQ(cobs::encode(in.data(), in.size(), out, sizeof(out)), 0u);
  uint8_t out5[5];
  EXPECT_EQ(cobs::encode(in.data(), in.size(), out5, sizeof(out5)), 5u);
  uint8_t small[2];
  EXPECT_EQ(cobs::decode(out5, 5, small, sizeof(small)), cobs::npos);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
