#include "gc_arm_hardware/gc_arm_system.hpp"

#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gc_arm_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{

constexpr char kCurrent[] = "current";
constexpr char kTemperature[] = "temperature";
const double kNaN = std::numeric_limits<double>::quiet_NaN();

}  // namespace

#ifdef GC_ARM_HW_HAS_INTERFACE_PARAMS
CallbackReturn GcArmSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return initFromInfo();
}
#else
CallbackReturn GcArmSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return initFromInfo();
}
#endif

CallbackReturn GcArmSystem::initFromInfo()
{

  // ---- 하드웨어 파라미터 ----
  if (auto it = info_.hardware_parameters.find("can_interface"); it != info_.hardware_parameters.end()) {
    options_.can_interface = it->second;
  }
  if (auto it = info_.hardware_parameters.find("activate_timeout_ms"); it != info_.hardware_parameters.end()) {
    options_.activate_timeout = std::chrono::milliseconds(std::stoi(it->second));
  }

  // ---- 관절 ----
  const size_t n = info_.joints.size();
  configs_.clear();
  initial_positions_.assign(n, 0.0);
  has_vel_cmd_.assign(n, false);

  const std::set<std::string> allowed_state = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT, kCurrent, kTemperature};

  for (size_t i = 0; i < n; ++i) {
    const auto & joint = info_.joints[i];
    try {
      std::map<std::string, std::string> params(joint.parameters.begin(), joint.parameters.end());
      configs_.push_back(gc_arm::parseJointConfig(joint.name, params));
    } catch (const std::exception & e) {
      RCLCPP_FATAL(logger_, "관절 설정 오류: %s", e.what());
      return CallbackReturn::ERROR;
    }

    bool has_pos_cmd = false;
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        has_pos_cmd = true;
      } else if (ci.name == hardware_interface::HW_IF_VELOCITY) {
        has_vel_cmd_[i] = true;
      } else {
        RCLCPP_FATAL(logger_, "%s: 지원하지 않는 command interface '%s'", joint.name.c_str(), ci.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    if (!has_pos_cmd) {
      RCLCPP_FATAL(logger_, "%s: position command interface 필수", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    for (const auto & si : joint.state_interfaces) {
      if (!allowed_state.count(si.name)) {
        RCLCPP_FATAL(logger_, "%s: 지원하지 않는 state interface '%s'", joint.name.c_str(), si.name.c_str());
        return CallbackReturn::ERROR;
      }
      if (si.name == hardware_interface::HW_IF_POSITION && !si.initial_value.empty()) {
        initial_positions_[i] = std::stod(si.initial_value);
      }
    }

    // AK는 전원 재투입 시 출력축 한 바퀴 내 위치만 유지 → 관절 범위가 한 바퀴를 넘으면 기준 자세 필수
    const auto & c = configs_[i];
    if (c.type == gc_arm::ActuatorType::AkServo && !c.startup_reference) {
      const double span_turns = (c.max_position - c.min_position) * c.ratio / (2.0 * M_PI);
      if (!std::isfinite(span_turns) || span_turns > 1.0) {
        RCLCPP_FATAL(
          logger_, "%s: 관절 범위가 모터 출력축 %.2f 바퀴에 해당 → startup_reference 필수 "
          "(전원 투입 시 놓이는 자세, joints.yaml startup_reference_deg)", joint.name.c_str(), span_turns);
        return CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(
      logger_, "%s: type=%s can_id=%u ratio=%.6g dir=%+.0f", joint.name.c_str(),
      gc_arm::toString(configs_[i].type), configs_[i].can_id, configs_[i].ratio, configs_[i].direction);
  }

  pos_state_.assign(n, 0.0);
  vel_state_.assign(n, 0.0);
  eff_state_.assign(n, 0.0);
  cur_state_.assign(n, 0.0);
  temp_state_.assign(n, 0.0);
  pos_cmd_.assign(n, kNaN);
  vel_cmd_.assign(n, kNaN);
  vel_for_driver_.assign(n, kNaN);
  RCLCPP_INFO(logger_, "can_interface=%s, 관절 %zu개", options_.can_interface.c_str(), n);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GcArmSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    for (const auto & si : joint.state_interfaces) {
      double * ptr = nullptr;
      if (si.name == hardware_interface::HW_IF_POSITION) {ptr = &pos_state_[i];}
      if (si.name == hardware_interface::HW_IF_VELOCITY) {ptr = &vel_state_[i];}
      if (si.name == hardware_interface::HW_IF_EFFORT) {ptr = &eff_state_[i];}
      if (si.name == kCurrent) {ptr = &cur_state_[i];}
      if (si.name == kTemperature) {ptr = &temp_state_[i];}
      out.emplace_back(joint.name, si.name, ptr);
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> GcArmSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    for (const auto & ci : joint.command_interfaces) {
      double * ptr = ci.name == hardware_interface::HW_IF_POSITION ? &pos_cmd_[i] : &vel_cmd_[i];
      out.emplace_back(joint.name, ci.name, ptr);
    }
  }
  return out;
}

void GcArmSystem::copyStates()
{
  for (size_t i = 0; i < states_.size(); ++i) {
    pos_state_[i] = states_[i].position;
    vel_state_[i] = states_[i].velocity;
    eff_state_[i] = states_[i].effort;
    cur_state_[i] = states_[i].current;
    temp_state_[i] = states_[i].temperature;
  }
}

CallbackReturn GcArmSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::string err;
  if (!driver_.configure(options_, configs_, &err)) {
    RCLCPP_ERROR(logger_, "configure 실패: %s", err.c_str());
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < configs_.size(); ++i) {
    if (configs_[i].type == gc_arm::ActuatorType::Mock) {
      driver_.setMockPosition(i, initial_positions_[i]);
    }
  }
  driver_.read(states_, nullptr);
  copyStates();
  RCLCPP_INFO(logger_, "configure 완료");
  return CallbackReturn::SUCCESS;
}

CallbackReturn GcArmSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::string err;
  if (!driver_.activate(&err)) {
    RCLCPP_ERROR(logger_, "activate 실패: %s", err.c_str());
    return CallbackReturn::ERROR;
  }
  if (!driver_.read(states_, &err)) {
    RCLCPP_ERROR(logger_, "activate 후 상태 이상: %s", err.c_str());
    driver_.deactivate();
    return CallbackReturn::ERROR;
  }
  copyStates();
  for (const auto & msg : driver_.startupLog()) {
    RCLCPP_INFO(logger_, "시작 자세 보정 %s", msg.c_str());
  }
  // 급격한 이동 방지: 명령 = 현재 위치
  for (size_t i = 0; i < pos_cmd_.size(); ++i) {
    pos_cmd_[i] = pos_state_[i];
    vel_cmd_[i] = kNaN;  // 컨트롤러가 velocity를 쓰기 전까지 "속도 명령 없음"
    RCLCPP_INFO(logger_, "%s 시작 위치 %.4f rad", info_.joints[i].name.c_str(), pos_state_[i]);
  }
  RCLCPP_INFO(logger_, "activate 완료");
  return CallbackReturn::SUCCESS;
}

CallbackReturn GcArmSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  driver_.deactivate();
  RCLCPP_INFO(logger_, "deactivate (모터는 마지막 목표 위치 유지)");
  return CallbackReturn::SUCCESS;
}

CallbackReturn GcArmSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  driver_.cleanup();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GcArmSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  driver_.cleanup();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GcArmSystem::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(logger_, "하드웨어 오류 → 드라이버 정리");
  driver_.cleanup();
  return CallbackReturn::SUCCESS;
}

return_type GcArmSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::string err;
  const bool ok = driver_.read(states_, &err);
  copyStates();
  if (!ok) {
    RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, 1000, "read 오류: %s", err.c_str());
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type GcArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < vel_cmd_.size(); ++i) {
    vel_for_driver_[i] = has_vel_cmd_[i] ? vel_cmd_[i] : kNaN;
  }
  std::string err;
  if (!driver_.write(pos_cmd_, vel_for_driver_, &err)) {
    RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, 1000, "write 오류: %s", err.c_str());
    return return_type::ERROR;
  }
  return return_type::OK;
}

}  // namespace gc_arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gc_arm_hardware::GcArmSystem, hardware_interface::SystemInterface)
