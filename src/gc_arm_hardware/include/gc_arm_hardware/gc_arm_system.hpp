#pragma once

#include <string>
#include <vector>

#include "gc_arm_drivers/arm_driver.hpp"
#include "gc_arm_drivers/joint_config.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

// 최신 Jazzy ros2_control은 on_init(HardwareComponentInterfaceParams)를 사용하고
// on_init(HardwareInfo)는 deprecated. 구버전에는 새 타입이 없으므로 헤더 존재 여부로 선택.
#if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#define GC_ARM_HW_HAS_INTERFACE_PARAMS 1
#endif

namespace gc_arm_hardware
{

/// GC 6축 로봇팔 ros2_control 시스템 인터페이스.
/// 실제 동작은 ROS 비의존 gc_arm::ArmDriver가 담당하고, 이 클래스는 ros2_control과의 연결만 한다.
///
/// 하드웨어 파라미터 (<hardware><param>):
///   can_interface        "can0" | "sim" (시뮬레이션 버스)          기본 can0
///   activate_timeout_ms  활성화 시 첫 피드백 대기 시간               기본 1000
/// 관절 파라미터 (<joint><param>): gc_arm::parseJointConfig 참고 (type, can_id, ratio, ...)
/// 관절 인터페이스:
///   command: position (필수), velocity (선택: 모터 속도 제한 계산에 사용)
///   state:   position, velocity, effort, current, temperature 중 선언한 것
class GcArmSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GcArmSystem)

#ifdef GC_ARM_HW_HAS_INTERFACE_PARAMS
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
#else
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
#endif
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// info_ (기반 클래스가 채움)로부터 설정 파싱 — on_init 두 버전 공통
  hardware_interface::CallbackReturn initFromInfo();
  void copyStates();

  gc_arm::ArmDriver driver_;
  gc_arm::ArmDriver::Options options_;
  std::vector<gc_arm::JointConfig> configs_;
  std::vector<double> initial_positions_;

  // ros2_control과 공유하는 버퍼 (관절 순서 = URDF 순서)
  std::vector<double> pos_state_, vel_state_, eff_state_, cur_state_, temp_state_;
  std::vector<double> pos_cmd_, vel_cmd_;
  std::vector<bool> has_vel_cmd_;

  std::vector<gc_arm::ActuatorState> states_;
  std::vector<double> vel_for_driver_;

  rclcpp::Logger logger_{rclcpp::get_logger("GcArmSystem")};
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
};

}  // namespace gc_arm_hardware
