#include <gtest/gtest.h>

#include <cmath>

#include "gc_arm_drivers/actuator.hpp"
#include "gc_arm_drivers/ak_servo_protocol.hpp"
#include "gc_arm_drivers/byte_utils.hpp"
#include "gc_arm_drivers/joint_config.hpp"
#include "gc_arm_drivers/joint_conversions.hpp"
#include "gc_arm_drivers/mks_protocol.hpp"

using namespace gc_arm;
using conv::kPi;

namespace
{

JointConfig j2Config()
{
  // J2: AK45-36 (id 101), 외부 4:1
  return parseJointConfig("joint2", {
    {"type", "ak_servo"}, {"can_id", "101"}, {"ratio", "4"}, {"direction", "-1"},
    {"offset", "0.1"}, {"ak_pole_pairs", "14"}, {"ak_internal_ratio", "36"},
    {"max_velocity", "1.0"}});
}

JointConfig j4Config()
{
  // J4: SERVO42D (id 2), 20:1
  return parseJointConfig("joint4", {
    {"type", "mks"}, {"can_id", "2"}, {"ratio", "20"}, {"max_velocity", "1.5"},
    {"mks_max_rpm", "1500"}});
}

}  // namespace

TEST(JointConfig, ParseAndDefaults)
{
  auto c = j2Config();
  EXPECT_EQ(c.type, ActuatorType::AkServo);
  EXPECT_EQ(c.can_id, 101);
  EXPECT_DOUBLE_EQ(c.ratio, 4.0);
  EXPECT_DOUBLE_EQ(c.direction, -1.0);
  EXPECT_DOUBLE_EQ(c.feedback_timeout, 0.1);  // 기본값

  auto m = parseJointConfig("j", {});
  EXPECT_EQ(m.type, ActuatorType::Mock);
}

TEST(JointConfig, RejectsBadValues)
{
  EXPECT_THROW(parseJointConfig("j", {{"type", "stepper"}}), std::invalid_argument);
  EXPECT_THROW(parseJointConfig("j", {{"type", "ak_servo"}, {"can_id", "0"}}), std::invalid_argument);
  EXPECT_THROW(parseJointConfig("j", {{"type", "mks"}, {"can_id", "2"}, {"ratio", "0"}}), std::invalid_argument);
  EXPECT_THROW(parseJointConfig("j", {{"direction", "2"}}), std::invalid_argument);
  EXPECT_THROW(parseJointConfig("j", {{"ratio", "4x"}}), std::invalid_argument);
  EXPECT_THROW(parseJointConfig("j", {{"typo_key", "1"}}), std::invalid_argument);
  EXPECT_THROW(parseJointConfig("j", {{"min_position", "1"}, {"max_position", "-1"}}), std::invalid_argument);
}

TEST(Conversions, JointActuatorRoundTrip)
{
  auto c = j2Config();
  for (double q : {-1.2, 0.0, 0.1, 0.75, 2.0}) {
    EXPECT_NEAR(conv::actuatorToJoint(c, conv::jointToActuator(c, q)), q, 1e-12);
  }
  // 관절 0.1(offset) → 액추에이터 0, 관절 +0.6 → 액추에이터 -(0.5×4) = -2.0 rad
  EXPECT_NEAR(conv::jointToActuator(c, 0.1), 0.0, 1e-12);
  EXPECT_NEAR(conv::jointToActuator(c, 0.6), -2.0, 1e-12);
}

TEST(Conversions, AkErpm)
{
  auto c = j2Config();
  // 출력축 1 rev/s = 60 rpm → 로터 2160 rpm → ERPM 30240
  EXPECT_NEAR(conv::akRadSToErpm(c, 2 * kPi), 30240.0, 1e-6);
  EXPECT_NEAR(conv::akErpmToRadS(c, 30240.0), 2 * kPi, 1e-9);
}

TEST(Conversions, SpeedLimit)
{
  auto c = j2Config();  // max 1.0, margin 1.5, floor 0.2
  EXPECT_DOUBLE_EQ(conv::jointSpeedLimit(c, std::nan("")), 1.0);
  EXPECT_DOUBLE_EQ(conv::jointSpeedLimit(c, 0.0), 0.2);
  EXPECT_DOUBLE_EQ(conv::jointSpeedLimit(c, -0.4), 0.6);
  EXPECT_DOUBLE_EQ(conv::jointSpeedLimit(c, 5.0), 1.0);
  // catch-up (gain 2): 속도 0 + 오차 0.1 → 0.2 + 0.2, 큰 오차 → max로 포화
  EXPECT_DOUBLE_EQ(conv::jointSpeedLimit(c, 0.0, 0.1), 0.4);
  EXPECT_DOUBLE_EQ(conv::jointSpeedLimit(c, 0.0, -0.1), 0.4);
  EXPECT_DOUBLE_EQ(conv::jointSpeedLimit(c, 0.0, 5.0), 1.0);
}

TEST(AkActuator, CommandFrame)
{
  AkServoActuator a(j2Config());
  std::vector<CanFrame> out;
  // 관절 0.6 rad → 액추에이터 -2.0 rad = -114.59 deg, 속도 NaN → 1.0 rad/s 관절 → 4 rad/s 출력
  a.commandFrames(0.6, std::nan(""), out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, ak::servo::makeId(101, ak::servo::PacketId::SetPosSpd));
  const int32_t pos = static_cast<int32_t>(bytes::getBe32(&out[0].data[0]));
  EXPECT_NEAR(pos / 10000.0, conv::rad2deg(-2.0), 1e-3);
  const int16_t spd = static_cast<int16_t>(bytes::getBe16(&out[0].data[4]));
  EXPECT_NEAR(spd * 10.0, conv::akRadSToErpm(a.config(), 4.0), 10.0);
}

TEST(AkActuator, FeedbackToJointState)
{
  AkServoActuator a(j2Config());
  CanFrame f;
  f.extended = true;
  f.id = 0x2965;  // id 101
  f.dlc = 8;
  // 출력축 -90.0 deg → 관절 = -1 × (-π/2)/4 + 0.1
  bytes::putBe16(&f.data[0], static_cast<uint16_t>(static_cast<int16_t>(-900)));
  f.data[6] = 42;
  EXPECT_TRUE(a.handleFrame(f, SteadyClock::now()));
  auto s = a.state();
  EXPECT_TRUE(s.valid);
  EXPECT_NEAR(s.position, kPi / 8 + 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(s.temperature, 42.0);

  f.id = 0x2966;  // 다른 모터
  EXPECT_FALSE(a.handleFrame(f, SteadyClock::now()));
}

TEST(MksActuator, CommandAndFeedback)
{
  auto cfg = j4Config();
  cfg.catchup_gain = 0.0;
  MksActuator a(cfg);
  std::vector<CanFrame> out;
  // 관절 π/2 → 모터 10π rad = 5 rev = 81920 count
  a.commandFrames(kPi / 2, 0.5, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].data[0], 0xF5);
  EXPECT_EQ(bytes::getBeInt24(&out[0].data[4]), 81920);
  // 속도: 0.5 × 1.5 = 0.75 rad/s 관절 → ×20 = 15 rad/s 모터 = 143.2 rpm → 143
  EXPECT_EQ(bytes::getBe16(&out[0].data[1]), 143);

  // 기본 catch-up(2/s): 오차 π/2 → 제한이 max_velocity(1.5 rad/s)로 포화 → 30 rad/s 모터 = 286 rpm
  MksActuator b(j4Config());
  out.clear();
  b.commandFrames(kPi / 2, 0.5, out);
  EXPECT_EQ(bytes::getBe16(&out[0].data[1]), 286);

  CanFrame r;
  r.id = 2;
  r.dlc = 8;
  r.data = {0x31, 0, 0, 0, 0x01, 0x40, 0x00, 0};  // 81920
  r.data[7] = mks::crc(2, r.data.data(), 7);
  EXPECT_TRUE(a.handleFrame(r, SteadyClock::now()));
  EXPECT_NEAR(a.state().position, kPi / 2, 1e-9);
}

TEST(MksActuator, PositionClamp)
{
  auto c = j4Config();
  c.max_position = 1.0;
  MksActuator a(c);
  std::vector<CanFrame> out;
  a.commandFrames(3.0, 0.0, out);
  EXPECT_EQ(bytes::getBeInt24(&out[0].data[4]), static_cast<int32_t>(std::lround(conv::mksRadToCounts(20.0))));
}
