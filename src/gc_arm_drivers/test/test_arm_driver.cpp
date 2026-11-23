#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include "gc_arm_drivers/arm_driver.hpp"
#include "gc_arm_drivers/simulated_bus.hpp"

using namespace gc_arm;
using namespace std::chrono_literals;

namespace
{

/// 현재 하드웨어 구성(AK 2대 + 42D 1대) + 나머지 mock
std::vector<JointConfig> partialArm()
{
  return {
    parseJointConfig("joint1", {{"type", "mock"}}),
    parseJointConfig("joint2", {{"type", "ak_servo"}, {"can_id", "101"}, {"ratio", "4"},
      {"ak_internal_ratio", "36"}, {"max_velocity", "2.0"}}),
    parseJointConfig("joint3", {{"type", "ak_servo"}, {"can_id", "102"}, {"ratio", "2"},
      {"ak_internal_ratio", "36"}, {"max_velocity", "2.0"}, {"direction", "-1"}}),
    parseJointConfig("joint4", {{"type", "mks"}, {"can_id", "2"}, {"ratio", "20"},
      {"max_velocity", "2.0"}, {"mks_max_rpm", "3000"}}),
    parseJointConfig("joint5", {{"type", "mock"}}),
    parseJointConfig("joint6", {{"type", "mock"}}),
  };
}

ArmDriver::Options simOptions()
{
  ArmDriver::Options o;
  o.can_interface = "sim";
  return o;
}

/// 목표로 일정 시간 명령을 보내며 100 Hz 루프 실행
void runLoop(ArmDriver & d, const std::vector<double> & target, std::chrono::milliseconds dur)
{
  std::vector<double> vel(target.size(), std::nan(""));
  const auto end = std::chrono::steady_clock::now() + dur;
  std::vector<ActuatorState> st;
  while (std::chrono::steady_clock::now() < end) {
    std::string err;
    d.read(st, &err);
    ASSERT_TRUE(d.write(target, vel, &err)) << err;
    std::this_thread::sleep_for(10ms);
  }
}

}  // namespace

TEST(ArmDriver, AllMockNeedsNoBus)
{
  ArmDriver d;
  std::vector<JointConfig> joints(6);
  for (int i = 0; i < 6; ++i) {joints[i].name = "j" + std::to_string(i);}
  ArmDriver::Options o;
  o.can_interface = "does_not_exist";  // 열지 않아야 함
  std::string err;
  ASSERT_TRUE(d.configure(o, joints, &err)) << err;
  ASSERT_TRUE(d.activate(&err)) << err;
  std::vector<double> pos = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6}, vel(6, 0.0);
  ASSERT_TRUE(d.write(pos, vel, &err));
  std::vector<ActuatorState> st;
  ASSERT_TRUE(d.read(st, &err)) << err;
  for (int i = 0; i < 6; ++i) {EXPECT_DOUBLE_EQ(st[i].position, pos[i]);}
}

TEST(ArmDriver, RejectsDuplicateIds)
{
  ArmDriver d;
  auto joints = partialArm();
  joints[2] = parseJointConfig("joint3", {{"type", "ak_servo"}, {"can_id", "101"}});
  std::string err;
  EXPECT_FALSE(d.configure(simOptions(), joints, &err));
  EXPECT_NE(err.find("중복"), std::string::npos);
}

TEST(ArmDriver, SimulatedPartialArmTracksTargets)
{
  ArmDriver d;
  std::string err;
  ASSERT_TRUE(d.configure(simOptions(), partialArm(), &err)) << err;
  ASSERT_TRUE(d.activate(&err)) << err;
  ASSERT_NE(d.simulatedBus(), nullptr);
  EXPECT_TRUE(d.simulatedBus()->mksEnabled(2));  // activate 시 enable 송신

  std::vector<ActuatorState> st;
  ASSERT_TRUE(d.read(st, &err)) << err;

  const std::vector<double> target = {0.3, 0.5, -0.4, 0.8, -0.2, 0.1};
  runLoop(d, target, 1500ms);

  ASSERT_TRUE(d.read(st, &err)) << err;
  // AK 피드백 분해능 0.1 deg(출력축) → 관절 기준 더 작음. MKS는 count 분해능
  for (size_t i = 0; i < target.size(); ++i) {
    EXPECT_NEAR(st[i].position, target[i], 2e-3) << "joint" << i + 1;
  }
  // 시뮬레이터 내부 값으로 단위 변환 검증: joint2 0.5 rad × 4 = 2.0 rad = 114.59 deg
  EXPECT_NEAR(d.simulatedBus()->akPositionDeg(101), 2.0 * 180.0 / M_PI, 0.2);
  // joint3 direction -1: -0.4 × 2 × -1 = 0.8 rad
  EXPECT_NEAR(d.simulatedBus()->akPositionDeg(102), 0.8 * 180.0 / M_PI, 0.2);
  // joint4: 0.8 × 20 = 16 rad → count
  EXPECT_NEAR(d.simulatedBus()->mksCounts(2), 16.0 * 16384 / (2 * M_PI), 2.0);
}

TEST(ArmDriver, SpeedLimitIsRespected)
{
  ArmDriver d;
  std::string err;
  ASSERT_TRUE(d.configure(simOptions(), partialArm(), &err)) << err;
  ASSERT_TRUE(d.activate(&err)) << err;
  // max_velocity 2 rad/s → 0.5 s 후 joint2는 1.0 rad 근처까지만 이동 가능
  std::vector<double> target = {0, 10.0, 0, 0, 0, 0};
  runLoop(d, target, 500ms);
  std::vector<ActuatorState> st;
  d.read(st, &err);
  EXPECT_LT(st[1].position, 1.3);
  EXPECT_GT(st[1].position, 0.7);
}

TEST(ArmDriver, FeedbackTimeoutIsReported)
{
  ArmDriver d;
  std::string err;
  ASSERT_TRUE(d.configure(simOptions(), partialArm(), &err)) << err;
  ASSERT_TRUE(d.activate(&err)) << err;
  d.simulatedBus()->setSilent(102, true);
  std::vector<double> pos(6, 0.0), vel(6, std::nan(""));
  std::vector<ActuatorState> st;
  for (int i = 0; i < 20; ++i) {
    d.write(pos, vel, &err);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_FALSE(d.read(st, &err));
  EXPECT_NE(err.find("joint3"), std::string::npos) << err;
}

TEST(ArmDriver, DriverErrorIsReported)
{
  ArmDriver d;
  std::string err;
  ASSERT_TRUE(d.configure(simOptions(), partialArm(), &err)) << err;
  ASSERT_TRUE(d.activate(&err)) << err;
  d.simulatedBus()->setAkError(101, 2);  // 과전류
  std::this_thread::sleep_for(30ms);
  std::vector<ActuatorState> st;
  EXPECT_FALSE(d.read(st, &err));
  EXPECT_NE(err.find("joint2"), std::string::npos) << err;
}

TEST(ArmDriver, ActivateFailsWhenMotorMissing)
{
  ArmDriver d;
  auto o = simOptions();
  o.activate_timeout = 200ms;
  std::string err;
  ASSERT_TRUE(d.configure(o, partialArm(), &err)) << err;
  d.simulatedBus()->setSilent(2, true);
  EXPECT_FALSE(d.activate(&err));
  EXPECT_NE(err.find("joint4"), std::string::npos) << err;
}
