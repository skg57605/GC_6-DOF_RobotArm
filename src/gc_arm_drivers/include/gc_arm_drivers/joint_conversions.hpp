#pragma once

#include <algorithm>
#include <cmath>

#include "gc_arm_drivers/joint_config.hpp"
#include "gc_arm_drivers/mks_protocol.hpp"

/// 관절 좌표 [rad, rad/s] ↔ 액추에이터 원시 단위 변환 (순수 함수)
namespace gc_arm::conv
{

constexpr double kPi = 3.14159265358979323846;
inline double deg2rad(double d) {return d * kPi / 180.0;}
inline double rad2deg(double r) {return r * 180.0 / kPi;}

/// 액추에이터 각도 [rad] → 관절 [rad]
inline double actuatorToJoint(const JointConfig & c, double actuator_rad)
{
  return c.direction * actuator_rad / c.ratio + c.offset;
}

/// 관절 [rad] → 액추에이터 각도 [rad]
inline double jointToActuator(const JointConfig & c, double joint_rad)
{
  return c.direction * (joint_rad - c.offset) * c.ratio;
}

/// 관절 속도 [rad/s] → 액추에이터 속도 [rad/s] (부호 포함)
inline double jointVelToActuator(const JointConfig & c, double v) {return c.direction * v * c.ratio;}
inline double actuatorVelToJoint(const JointConfig & c, double v) {return c.direction * v / c.ratio;}

// ---- AK (Servo 모드) ----
/// ERPM → 액추에이터 출력축 [rad/s]
inline double akErpmToRadS(const JointConfig & c, double erpm)
{
  return erpm / c.ak_pole_pairs / c.ak_internal_ratio * 2.0 * kPi / 60.0;
}
inline double akRadSToErpm(const JointConfig & c, double rad_s)
{
  return rad_s * 60.0 / (2.0 * kPi) * c.ak_internal_ratio * c.ak_pole_pairs;
}

// ---- MKS ----
inline double mksCountsToRad(int64_t counts)
{
  return static_cast<double>(counts) * 2.0 * kPi / mks::kCountsPerRev;
}
inline double mksRadToCounts(double rad) {return rad * mks::kCountsPerRev / (2.0 * kPi);}
inline double radSToRpm(double rad_s) {return rad_s * 60.0 / (2.0 * kPi);}

/// 관절 속도 명령과 위치 오차로부터 모터 속도 제한(관절 기준, 양수 [rad/s]) 계산
///   limit = clamp(max(|v_cmd| × margin, max_velocity × min_fraction) + catchup_gain × |pos_error|,
///                 0, max_velocity)
///   v_cmd가 NaN이면(속도 명령 없음) max_velocity
///   catch-up 항: 추종이 뒤처지거나 속도 0과 함께 먼 목표가 오면 오차에 비례해 제한을 키움
inline double jointSpeedLimit(const JointConfig & c, double v_cmd, double pos_error = 0.0)
{
  if (std::isnan(v_cmd)) {
    return c.max_velocity;
  }
  const double floor = c.max_velocity * c.min_velocity_fraction;
  const double base = std::max(std::fabs(v_cmd) * c.velocity_margin, floor);
  return std::min(base + c.catchup_gain * std::fabs(pos_error), c.max_velocity);
}

inline double clampPosition(const JointConfig & c, double q)
{
  return std::clamp(q, c.min_position, c.max_position);
}

}  // namespace gc_arm::conv
