#include "gc_arm_drivers/actuator.hpp"

#include <cmath>

#include "gc_arm_drivers/ak_servo_protocol.hpp"
#include "gc_arm_drivers/joint_conversions.hpp"
#include "gc_arm_drivers/mks_protocol.hpp"

namespace gc_arm
{

// ------------------------------------------------------------------ Mock
MockActuator::MockActuator(JointConfig c)
: Actuator(std::move(c))
{
  state_.valid = true;
  state_.stamp = SteadyClock::now();
}

void MockActuator::commandFrames(double position, double velocity, std::vector<CanFrame> &)
{
  std::lock_guard<std::mutex> lk(mutex_);
  state_.position = conv::clampPosition(config_, position);
  state_.velocity = std::isnan(velocity) ? 0.0 : velocity;
  state_.stamp = SteadyClock::now();
}

void MockActuator::setPosition(double q)
{
  std::lock_guard<std::mutex> lk(mutex_);
  state_.position = q;
  state_.velocity = 0.0;
}

// ------------------------------------------------------------------ AK Servo
void AkServoActuator::commandFrames(double position, double velocity, std::vector<CanFrame> & out)
{
  const double q = conv::clampPosition(config_, position);
  const double act_deg = conv::rad2deg(conv::jointToActuator(config_, q));
  const double speed_joint = conv::jointSpeedLimit(config_, velocity, q - state().position);
  const double speed_erpm = conv::akRadSToErpm(config_, speed_joint * config_.ratio);
  out.push_back(ak::servo::makePositionSpeed(
      static_cast<uint8_t>(config_.can_id), act_deg, speed_erpm, config_.ak_accel_erpm_s2));
}

bool AkServoActuator::handleFrame(const CanFrame & frame, SteadyClock::time_point now)
{
  auto fb = ak::servo::parseFeedback(frame);
  if (!fb || fb->driver_id != config_.can_id) {
    return false;
  }
  std::lock_guard<std::mutex> lk(mutex_);
  state_.position = conv::actuatorToJoint(config_, conv::deg2rad(fb->position_deg));
  state_.velocity = conv::actuatorVelToJoint(config_, conv::akErpmToRadS(config_, fb->speed_erpm));
  state_.current = fb->current_a;
  state_.effort = config_.direction * fb->current_a * config_.ak_torque_constant * config_.ratio;
  state_.temperature = fb->temperature_c;
  state_.error = fb->error;
  state_.valid = true;
  state_.stamp = now;
  return true;
}

// ------------------------------------------------------------------ MKS
void MksActuator::activateFrames(std::vector<CanFrame> & out)
{
  out.push_back(mks::makeEnable(config_.can_id, true));
}

void MksActuator::pollFrames(std::vector<CanFrame> & out)
{
  out.push_back(mks::makeReadEncoderAddition(config_.can_id));
}

void MksActuator::commandFrames(double position, double velocity, std::vector<CanFrame> & out)
{
  const double q = conv::clampPosition(config_, position);
  const double counts = conv::mksRadToCounts(conv::jointToActuator(config_, q));
  const double speed_joint = conv::jointSpeedLimit(config_, velocity, q - state().position);
  const double rpm = conv::radSToRpm(speed_joint * config_.ratio);
  const uint16_t rpm_cmd = static_cast<uint16_t>(
    std::clamp(std::lround(rpm), 1L, static_cast<long>(config_.mks_max_rpm)));
  out.push_back(mks::makePosAbsAxis(
      config_.can_id, rpm_cmd, config_.mks_acc, static_cast<int32_t>(std::lround(counts))));
}

bool MksActuator::handleFrame(const CanFrame & frame, SteadyClock::time_point now)
{
  if (frame.error || frame.extended || frame.id != config_.can_id) {
    return false;
  }
  auto enc = mks::parseEncoderAddition(frame);
  if (!enc) {
    // 같은 ID의 다른 응답(F3/F5 상태 등)은 소비만 함
    return mks::checkCrc(frame);
  }
  const double q = conv::actuatorToJoint(config_, conv::mksCountsToRad(*enc));
  std::lock_guard<std::mutex> lk(mutex_);
  if (last_stamp_) {
    const double dt = std::chrono::duration<double>(now - *last_stamp_).count();
    if (dt > 1e-4) {
      const double v_raw = (q - last_position_) / dt;
      const double a = config_.velocity_filter;
      state_.velocity = a * v_raw + (1.0 - a) * state_.velocity;
    }
  }
  last_stamp_ = now;
  last_position_ = q;
  state_.position = q;
  state_.valid = true;
  state_.stamp = now;
  return true;
}

std::unique_ptr<Actuator> makeActuator(const JointConfig & config)
{
  switch (config.type) {
    case ActuatorType::AkServo: return std::make_unique<AkServoActuator>(config);
    case ActuatorType::Mks: return std::make_unique<MksActuator>(config);
    case ActuatorType::Mock: break;
  }
  return std::make_unique<MockActuator>(config);
}

}  // namespace gc_arm
