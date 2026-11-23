#pragma once

#include <cstdint>
#include <optional>

#include "gc_arm_drivers/can_frame.hpp"

/// CubeMars AK 시리즈 — MIT(force control) 모드 프로토콜  [실험적 / 실기 미검증]
///
/// - 표준(11bit) 프레임, ID = driver_id
/// - 명령: p(16bit) v(12bit) kp(12bit) kd(12bit) t_ff(12bit) 를 [min,max] 범위로 선형 양자화
/// - 범위(Limits)는 모터 모델/펌웨어마다 다르므로 반드시 매뉴얼 값으로 설정할 것
/// - 응답 ID와 온도 오프셋은 펌웨어에 따라 다를 수 있음 → 전환 시 candump로 확인
namespace gc_arm::ak::mit
{

struct Limits
{
  double p_min{-12.5}, p_max{12.5};   ///< [rad]
  double v_min{-50.0}, v_max{50.0};   ///< [rad/s]  (모델별 확인 필요)
  double t_min{-25.0}, t_max{25.0};   ///< [Nm]     (모델별 확인 필요)
  double kp_min{0.0}, kp_max{500.0};
  double kd_min{0.0}, kd_max{5.0};
};

CanFrame makeEnterMotorMode(uint8_t driver_id);  ///< FF FF FF FF FF FF FF FC
CanFrame makeExitMotorMode(uint8_t driver_id);   ///< FF FF FF FF FF FF FF FD
CanFrame makeSetZero(uint8_t driver_id);         ///< FF FF FF FF FF FF FF FE

struct Command
{
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double kp{0.0};
  double kd{0.0};
  double torque_ff_nm{0.0};
};

CanFrame makeCommand(uint8_t driver_id, const Limits & limits, const Command & cmd);

struct Feedback
{
  uint8_t driver_id{0};
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double torque_nm{0.0};
  int8_t temperature_c{0};
  uint8_t error{0};
};

/// 응답 프레임 해석 (data[0] = driver id)
std::optional<Feedback> parseFeedback(const CanFrame & frame, const Limits & limits);

/// 내부 헬퍼 (테스트용 공개)
uint32_t floatToUint(double x, double x_min, double x_max, int bits);
double uintToFloat(uint32_t x, double x_min, double x_max, int bits);

}  // namespace gc_arm::ak::mit
