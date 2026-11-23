#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <string>

namespace gc_arm
{

enum class ActuatorType
{
  Mock,     ///< 하드웨어 없음: 명령을 그대로 상태로 반영
  AkServo,  ///< CubeMars AK, Servo 모드 (위치-속도 루프)
  Mks,      ///< MKS SERVO42D/57D, SR_vFOC + 0xF5 절대위치
};

/// 관절 하나의 설정. 관절 좌표 정의:
///   q_joint = direction × θ_actuator / ratio + offset
/// θ_actuator: AK = 액추에이터 출력축 각도(내장 감속 이후), MKS = 모터축 각도 [rad]
struct JointConfig
{
  std::string name;
  ActuatorType type{ActuatorType::Mock};
  uint16_t can_id{0};

  double ratio{1.0};        ///< 외부 감속비 (액추에이터 회전 / 관절 회전)
  double direction{1.0};    ///< +1 또는 -1
  double offset{0.0};       ///< [rad]

  double min_position{-std::numeric_limits<double>::infinity()};  ///< [rad] 명령 클램프
  double max_position{std::numeric_limits<double>::infinity()};
  double max_velocity{1.0};      ///< [rad/s] 관절 속도 상한 (모터 속도 제한 계산)
  double velocity_margin{1.5};   ///< 속도 명령 대비 모터 속도 제한 배율
  double min_velocity_fraction{0.2};  ///< 속도 제한 하한 = max_velocity × 이 값

  // CubeMars AK
  double ak_pole_pairs{14.0};       ///< 극쌍수 (상위 프로그램/데이터시트로 확인)
  double ak_internal_ratio{36.0};   ///< 내장 감속비 (AK45-36: 36, AK45-10: 10)
  double ak_accel_erpm_s2{30000.0}; ///< 위치-속도 모드 가속도 제한
  double ak_torque_constant{0.0};   ///< 출력축 기준 [Nm/A], 0 = 미설정(effort 0 보고)

  // MKS
  uint16_t mks_max_rpm{1000};  ///< 모터축 최대 rpm (≤ 3000)
  uint8_t mks_acc{0};          ///< 0xF5 가속도 파라미터 (0 = 가감속 없음)
  double velocity_filter{0.3}; ///< 위치 미분 속도 1차 저역통과 계수 (0~1)

  // 안전
  double feedback_timeout{0.1};  ///< [s] 이 시간 동안 피드백 없으면 오류
  double max_temperature{80.0};  ///< [°C] AK 온도 상한
};

/// 문자열 파라미터 맵(URDF <param>)에서 설정 생성. 잘못된 값이면 std::invalid_argument
///
/// 키: type(mock|ak_servo|mks), can_id, ratio, direction, offset, min_position, max_position,
///     max_velocity, velocity_margin, min_velocity_fraction,
///     ak_pole_pairs, ak_internal_ratio, ak_accel, ak_torque_constant,
///     mks_max_rpm, mks_acc, velocity_filter, feedback_timeout, max_temperature
JointConfig parseJointConfig(const std::string & name, const std::map<std::string, std::string> & params);

const char * toString(ActuatorType type);

}  // namespace gc_arm
