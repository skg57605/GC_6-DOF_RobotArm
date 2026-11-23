#include <gtest/gtest.h>

#include "gc_arm_drivers/ak_mit_protocol.hpp"

using namespace gc_arm;
using namespace gc_arm::ak::mit;

TEST(AkMit, SpecialFrames)
{
  CanFrame e = makeEnterMotorMode(0x66);
  EXPECT_FALSE(e.extended);
  EXPECT_EQ(e.id, 0x66u);
  EXPECT_EQ(e.dlc, 8);
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(e.data[i], 0xFF);
  }
  EXPECT_EQ(e.data[7], 0xFC);
  EXPECT_EQ(makeExitMotorMode(1).data[7], 0xFD);
  EXPECT_EQ(makeSetZero(1).data[7], 0xFE);
}

TEST(AkMit, QuantizationEndpoints)
{
  EXPECT_EQ(floatToUint(-12.5, -12.5, 12.5, 16), 0u);
  EXPECT_EQ(floatToUint(12.5, -12.5, 12.5, 16), 65535u);
  EXPECT_EQ(floatToUint(100.0, -12.5, 12.5, 16), 65535u);  // 포화
  EXPECT_EQ(floatToUint(0.0, 0.0, 500.0, 12), 0u);
  EXPECT_NEAR(uintToFloat(4095, -50, 50, 12), 50.0, 1e-9);
}

TEST(AkMit, CommandPacking)
{
  Limits l;  // p ±12.5, v ±50, t ±25, kp 0..500, kd 0..5
  Command c;
  c.position_rad = 0.0;
  c.velocity_rad_s = 0.0;
  c.kp = 0.0;
  c.kd = 0.0;
  c.torque_ff_nm = 0.0;
  CanFrame f = makeCommand(0x66, l, c);
  // p=0 → 32768(0x8000), v=0 → 2048(0x800), kp=0, kd=0, t=0 → 0x800
  EXPECT_EQ(f.data[0], 0x80);
  EXPECT_EQ(f.data[1], 0x00);
  EXPECT_EQ(f.data[2], 0x80);
  EXPECT_EQ(f.data[3], 0x00);
  EXPECT_EQ(f.data[4], 0x00);
  EXPECT_EQ(f.data[5], 0x00);
  EXPECT_EQ(f.data[6], 0x08);
  EXPECT_EQ(f.data[7], 0x00);
}

TEST(AkMit, FeedbackRoundTrip)
{
  Limits l;
  // 응답 프레임을 직접 구성: id, p(16), v(12), t(12), temp, err
  const uint32_t p = floatToUint(1.234, l.p_min, l.p_max, 16);
  const uint32_t v = floatToUint(-3.0, l.v_min, l.v_max, 12);
  const uint32_t t = floatToUint(2.0, l.t_min, l.t_max, 12);
  CanFrame f;
  f.id = 0;
  f.dlc = 8;
  f.data = {0x66, static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
    static_cast<uint8_t>(v >> 4), static_cast<uint8_t>(((v & 0xF) << 4) | (t >> 8)),
    static_cast<uint8_t>(t), 40, 0};
  auto fb = parseFeedback(f, l);
  ASSERT_TRUE(fb.has_value());
  EXPECT_EQ(fb->driver_id, 0x66);
  EXPECT_NEAR(fb->position_rad, 1.234, 25.0 / 65535);
  EXPECT_NEAR(fb->velocity_rad_s, -3.0, 100.0 / 4095);
  EXPECT_NEAR(fb->torque_nm, 2.0, 50.0 / 4095);
  EXPECT_EQ(fb->temperature_c, 40);
}
