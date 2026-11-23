#include <gtest/gtest.h>

#include "gc_arm_drivers/ak_servo_protocol.hpp"

using namespace gc_arm;
using namespace gc_arm::ak::servo;

TEST(AkServo, ExtendedIdLayout)
{
  EXPECT_EQ(makeId(102, PacketId::SetPos), 0x0466u);
  EXPECT_EQ(makeId(101, PacketId::SetPosSpd), 0x0665u);
  EXPECT_EQ(makeId(103, PacketId::SetOrigin), 0x0567u);
}

TEST(AkServo, PositionCommand)
{
  // 90.0 deg → 900000 = 0x000DBBA0
  CanFrame f = makePosition(102, 90.0);
  EXPECT_TRUE(f.extended);
  EXPECT_EQ(f.id, 0x0466u);
  ASSERT_EQ(f.dlc, 4);
  EXPECT_EQ(f.data[0], 0x00);
  EXPECT_EQ(f.data[1], 0x0D);
  EXPECT_EQ(f.data[2], 0xBB);
  EXPECT_EQ(f.data[3], 0xA0);
}

TEST(AkServo, NegativePosition)
{
  // -1.0 deg → -10000 = 0xFFFFD8F0
  CanFrame f = makePosition(101, -1.0);
  EXPECT_EQ(f.data[0], 0xFF);
  EXPECT_EQ(f.data[1], 0xFF);
  EXPECT_EQ(f.data[2], 0xD8);
  EXPECT_EQ(f.data[3], 0xF0);
}

TEST(AkServo, PositionSpeedCommand)
{
  // pos 10 deg → 100000 = 0x000186A0, spd 5000 erpm → 500 = 0x01F4, acc 20000 → 2000 = 0x07D0
  CanFrame f = makePositionSpeed(103, 10.0, 5000.0, 20000.0);
  EXPECT_EQ(f.id, 0x0667u);
  ASSERT_EQ(f.dlc, 8);
  const uint8_t expected[8] = {0x00, 0x01, 0x86, 0xA0, 0x01, 0xF4, 0x07, 0xD0};
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(f.data[i], expected[i]) << "byte " << i;
  }
}

TEST(AkServo, SpeedSaturatesToInt16)
{
  CanFrame f = makePositionSpeed(1, 0.0, 1e9, 0.0);
  EXPECT_EQ(f.data[4], 0x7F);
  EXPECT_EQ(f.data[5], 0xFF);
}

TEST(AkServo, CurrentAndRpm)
{
  CanFrame c = makeCurrent(102, 1.5);  // 1500 = 0x05DC
  EXPECT_EQ(c.id, 0x0166u);
  EXPECT_EQ(c.data[2], 0x05);
  EXPECT_EQ(c.data[3], 0xDC);

  CanFrame r = makeRpm(102, -3000);  // 0xFFFFF448
  EXPECT_EQ(r.id, 0x0366u);
  EXPECT_EQ(r.data[0], 0xFF);
  EXPECT_EQ(r.data[3], 0x48);
}

TEST(AkServo, SetOrigin)
{
  CanFrame f = makeSetOrigin(102, OriginMode::Temporary);
  EXPECT_EQ(f.id, 0x0566u);
  EXPECT_EQ(f.dlc, 1);
  EXPECT_EQ(f.data[0], 0);
}

TEST(AkServo, ParseFeedback)
{
  CanFrame f;
  f.extended = true;
  f.id = 0x2966;  // id 102
  f.dlc = 8;
  // pos -12.3 deg → -123 = 0xFF85, spd 1500 erpm → 150 = 0x0096,
  // cur 2.5 A → 250 = 0x00FA, temp 35, err 0
  f.data = {0xFF, 0x85, 0x00, 0x96, 0x00, 0xFA, 35, 0};
  auto fb = parseFeedback(f);
  ASSERT_TRUE(fb.has_value());
  EXPECT_EQ(fb->driver_id, 102);
  EXPECT_NEAR(fb->position_deg, -12.3, 1e-9);
  EXPECT_NEAR(fb->speed_erpm, 1500.0, 1e-9);
  EXPECT_NEAR(fb->current_a, 2.5, 1e-9);
  EXPECT_EQ(fb->temperature_c, 35);
  EXPECT_EQ(fb->error, 0);
}

TEST(AkServo, RejectNonFeedback)
{
  CanFrame f;
  f.extended = false;
  f.id = 0x029;
  f.dlc = 8;
  EXPECT_FALSE(parseFeedback(f).has_value());

  f.extended = true;
  f.id = 0x0466;  // 명령 프레임
  EXPECT_FALSE(parseFeedback(f).has_value());

  f.id = 0x2966;
  f.dlc = 6;  // 길이 부족
  EXPECT_FALSE(parseFeedback(f).has_value());
}
