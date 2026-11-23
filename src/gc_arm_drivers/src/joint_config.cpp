#include "gc_arm_drivers/joint_config.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

namespace gc_arm
{

namespace
{

double toDouble(const std::string & joint, const std::string & key, const std::string & v)
{
  try {
    size_t pos = 0;
    double d = std::stod(v, &pos);
    if (pos != v.size()) {
      throw std::invalid_argument("trailing");
    }
    return d;
  } catch (const std::exception &) {
    throw std::invalid_argument(joint + ": 파라미터 '" + key + "' 숫자 아님: '" + v + "'");
  }
}

}  // namespace

const char * toString(ActuatorType type)
{
  switch (type) {
    case ActuatorType::Mock: return "mock";
    case ActuatorType::AkServo: return "ak_servo";
    case ActuatorType::Mks: return "mks";
  }
  return "?";
}

JointConfig parseJointConfig(const std::string & name, const std::map<std::string, std::string> & params)
{
  static const std::set<std::string> known = {
    "type", "can_id", "ratio", "direction", "offset", "min_position", "max_position",
    "max_velocity", "velocity_margin", "min_velocity_fraction", "ak_pole_pairs",
    "ak_internal_ratio", "ak_accel", "ak_torque_constant", "mks_max_rpm", "mks_acc",
    "velocity_filter", "feedback_timeout", "max_temperature"};

  JointConfig c;
  c.name = name;
  for (const auto & [key, value] : params) {
    if (!known.count(key)) {
      throw std::invalid_argument(name + ": 알 수 없는 파라미터 '" + key + "'");
    }
  }
  auto num = [&](const std::string & key, double & out) {
      auto it = params.find(key);
      if (it != params.end()) {out = toDouble(name, key, it->second);}
    };

  if (auto it = params.find("type"); it != params.end()) {
    if (it->second == "mock") {
      c.type = ActuatorType::Mock;
    } else if (it->second == "ak_servo") {
      c.type = ActuatorType::AkServo;
    } else if (it->second == "mks") {
      c.type = ActuatorType::Mks;
    } else {
      throw std::invalid_argument(name + ": type은 mock|ak_servo|mks 중 하나: '" + it->second + "'");
    }
  }

  double can_id = 0, mks_max_rpm = c.mks_max_rpm, mks_acc = c.mks_acc;
  num("can_id", can_id);
  num("ratio", c.ratio);
  num("direction", c.direction);
  num("offset", c.offset);
  num("min_position", c.min_position);
  num("max_position", c.max_position);
  num("max_velocity", c.max_velocity);
  num("velocity_margin", c.velocity_margin);
  num("min_velocity_fraction", c.min_velocity_fraction);
  num("ak_pole_pairs", c.ak_pole_pairs);
  num("ak_internal_ratio", c.ak_internal_ratio);
  num("ak_accel", c.ak_accel_erpm_s2);
  num("ak_torque_constant", c.ak_torque_constant);
  num("mks_max_rpm", mks_max_rpm);
  num("mks_acc", mks_acc);
  num("velocity_filter", c.velocity_filter);
  num("feedback_timeout", c.feedback_timeout);
  num("max_temperature", c.max_temperature);

  // ---- 검증 ----
  auto fail = [&](const std::string & msg) {throw std::invalid_argument(name + ": " + msg);};
  if (c.type == ActuatorType::AkServo && (can_id < 1 || can_id > 255)) {fail("AK can_id는 1~255");}
  if (c.type == ActuatorType::Mks && (can_id < 1 || can_id > 0x7FF)) {fail("MKS can_id는 1~2047");}
  c.can_id = static_cast<uint16_t>(can_id);
  if (!(c.ratio > 0)) {fail("ratio > 0 이어야 함");}
  if (c.direction != 1.0 && c.direction != -1.0) {fail("direction은 1 또는 -1");}
  if (c.min_position >= c.max_position) {fail("min_position < max_position 이어야 함");}
  if (!(c.max_velocity > 0)) {fail("max_velocity > 0 이어야 함");}
  if (!(c.velocity_margin >= 1.0)) {fail("velocity_margin ≥ 1");}
  if (c.min_velocity_fraction < 0 || c.min_velocity_fraction > 1) {fail("min_velocity_fraction 0~1");}
  if (!(c.ak_pole_pairs > 0) || !(c.ak_internal_ratio > 0)) {fail("ak_pole_pairs, ak_internal_ratio > 0");}
  if (mks_max_rpm < 1 || mks_max_rpm > 3000) {fail("mks_max_rpm 1~3000");}
  if (mks_acc < 0 || mks_acc > 255) {fail("mks_acc 0~255");}
  if (c.velocity_filter <= 0 || c.velocity_filter > 1) {fail("velocity_filter (0, 1]");}
  if (!(c.feedback_timeout > 0)) {fail("feedback_timeout > 0");}
  c.mks_max_rpm = static_cast<uint16_t>(mks_max_rpm);
  c.mks_acc = static_cast<uint8_t>(mks_acc);
  return c;
}

}  // namespace gc_arm
