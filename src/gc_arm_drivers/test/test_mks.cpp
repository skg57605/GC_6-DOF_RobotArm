#include <gtest/gtest.h>

#include "gc_arm_drivers/byte_utils.hpp"
#include "gc_arm_drivers/mks_protocol.hpp"

using namespace gc_arm;
using namespace gc_arm::mks;

TEST(Mks, CrcIncludesId)
{
  // 매뉴얼 예: ID=1, 0x31 읽기 → CRC = (0x01 + 0x31) & 0xFF = 0x32
  CanFrame f = makeReadEncoderAddition(1);
  EXPECT_FALSE(f.extended);
  EXPECT_EQ(f.id, 1u);
  ASSERT_EQ(f.dlc, 2);
  EXPECT_EQ(f.data[0], 0x31);
  EXPECT_EQ(f.data[1], 0x32);
  EXPECT_TRUE(checkCrc(f));
}

TEST(Mks, EnableFrame)
{
  // ID=2: F3 01 → CRC = 2 + 0xF3 + 1 = 0xF6
  CanFrame f = makeEnable(2, true);
  ASSERT_EQ(f.dlc, 3);
  EXPECT_EQ(f.data[0], 0xF3);
  EXPECT_EQ(f.data[1], 0x01);
  EXPECT_EQ(f.data[2], 0xF6);
}

TEST(Mks, SpeedFrame)
{
  // ID=1, CW(dir=1) 640 rpm (0x280), acc 2 → F6 82 80 02 CRC
  CanFrame f = makeSpeed(1, -640, 2);
  ASSERT_EQ(f.dlc, 5);
  EXPECT_EQ(f.data[0], 0xF6);
  EXPECT_EQ(f.data[1], 0x82);
  EXPECT_EQ(f.data[2], 0x80);
  EXPECT_EQ(f.data[3], 0x02);
  EXPECT_EQ(f.data[4], static_cast<uint8_t>(1 + 0xF6 + 0x82 + 0x80 + 0x02));

  CanFrame g = makeSpeed(1, 640, 2);
  EXPECT_EQ(g.data[1], 0x02);

  CanFrame sat = makeSpeed(1, 99999, 0);  // 3000 = 0x0BB8 로 포화
  EXPECT_EQ(sat.data[1], 0x0B);
  EXPECT_EQ(sat.data[2], 0xB8);
}

TEST(Mks, AbsAxisFrame)
{
  // ID=1, speed 600 rpm(0x0258), acc 2, axis 0x4000 → F5 02 58 02 00 40 00 CRC
  CanFrame f = makePosAbsAxis(1, 600, 2, 0x4000);
  ASSERT_EQ(f.dlc, 8);
  const uint8_t expected[7] = {0xF5, 0x02, 0x58, 0x02, 0x00, 0x40, 0x00};
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(f.data[i], expected[i]) << "byte " << i;
  }
  EXPECT_TRUE(checkCrc(f));
}

TEST(Mks, AbsAxisNegativeAndClamp)
{
  CanFrame f = makePosAbsAxis(3, 100, 0, -1);
  EXPECT_EQ(f.data[4], 0xFF);
  EXPECT_EQ(f.data[5], 0xFF);
  EXPECT_EQ(f.data[6], 0xFF);
  EXPECT_EQ(bytes::getBeInt24(&f.data[4]), -1);

  CanFrame c = makePosAbsAxis(3, 100, 0, 0x7FFFFFFF);
  EXPECT_EQ(bytes::getBeInt24(&c.data[4]), kAxisMax);
}

TEST(Mks, ParseEncoderAddition)
{
  CanFrame f;
  f.id = 2;
  f.dlc = 8;
  // -16384 (0xFFFFFFFFC000) = 모터축 -360 deg
  f.data = {0x31, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0};
  f.data[7] = crc(2, f.data.data(), 7);
  auto v = parseEncoderAddition(f);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(*v, -16384);
  EXPECT_NEAR(countsToDeg(*v), -360.0, 1e-9);

  f.data[7] ^= 0x01;  // CRC 손상
  EXPECT_FALSE(parseEncoderAddition(f).has_value());
}

TEST(Mks, ParseStatus)
{
  CanFrame f;
  f.id = 1;
  f.dlc = 3;
  f.data[0] = 0xF5;
  f.data[1] = 2;
  f.data[2] = crc(1, f.data.data(), 2);
  auto s = parseStatus(f, Cmd::PosAbsAxis);
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(*s, 2);
  EXPECT_FALSE(parseStatus(f, Cmd::Speed).has_value());
}

TEST(Mks, SetCanId)
{
  CanFrame f = makeSetCanId(1, 0x123);
  EXPECT_EQ(f.data[0], 0x8B);
  EXPECT_EQ(f.data[1], 0x01);
  EXPECT_EQ(f.data[2], 0x23);
  EXPECT_TRUE(checkCrc(f));
}

TEST(Mks, DegCountConversion)
{
  EXPECT_NEAR(degToCounts(360.0), 16384.0, 1e-9);
  EXPECT_NEAR(degToCounts(90.0), 4096.0, 1e-9);
}
