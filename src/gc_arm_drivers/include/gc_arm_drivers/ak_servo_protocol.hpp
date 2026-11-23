#pragma once

#include <cstdint>
#include <optional>

#include "gc_arm_drivers/can_frame.hpp"

/// CubeMars AK 시리즈 (펌웨어 v3) — Servo 모드 프로토콜
///
/// - 확장(29bit) 프레임, ID = (packet_id << 8) | driver_id
/// - 페이로드는 빅엔디언
/// - 피드백: 확장 ID = (0x29 << 8) | driver_id, 설정된 업로드 주기로 주기적 송신
///
/// 위치 단위: 액추에이터 출력축 [deg] (내장 감속기 이후) — Phase 0에서 실측 검증할 것
/// 속도 단위: ERPM (전기각 RPM = 로터 기계 RPM × 극쌍수)
namespace gc_arm::ak::servo
{

enum class PacketId : uint8_t
{
  SetDuty = 0,
  SetCurrent = 1,
  SetCurrentBrake = 2,
  SetRpm = 3,
  SetPos = 4,
  SetOrigin = 5,
  SetPosSpd = 6,
};

constexpr uint8_t kFeedbackPacketId = 0x29;

enum class OriginMode : uint8_t
{
  Temporary = 0,       ///< 전원 차단 시 사라지는 임시 원점
  Permanent = 1,       ///< 영구 원점 (플래시 저장)
  RestoreDefault = 2,  ///< 공장 기본 원점 복원
};

/// 확장 ID 생성
constexpr uint32_t makeId(uint8_t driver_id, PacketId packet)
{
  return (static_cast<uint32_t>(packet) << 8) | driver_id;
}

CanFrame makeDuty(uint8_t driver_id, double duty);                   ///< duty: -1.0 ~ 1.0
CanFrame makeCurrent(uint8_t driver_id, double current_a);            ///< q축 전류 [A]
CanFrame makeCurrentBrake(uint8_t driver_id, double current_a);       ///< 브레이크 전류 [A]
CanFrame makeRpm(uint8_t driver_id, double erpm);                     ///< 속도 루프 [ERPM]
CanFrame makePosition(uint8_t driver_id, double position_deg);        ///< 위치 루프 [deg]
/// 위치-속도 루프: 사다리꼴 프로파일. speed/accel은 최대값(제한)
CanFrame makePositionSpeed(
  uint8_t driver_id, double position_deg, double speed_erpm, double accel_erpm_s2);
CanFrame makeSetOrigin(uint8_t driver_id, OriginMode mode);

struct Feedback
{
  uint8_t driver_id{0};
  double position_deg{0.0};  ///< 분해능 0.1 deg, 범위 약 ±3276.7 deg
  double speed_erpm{0.0};    ///< 분해능 10 ERPM
  double current_a{0.0};     ///< 분해능 0.01 A
  int8_t temperature_c{0};
  uint8_t error{0};
};

bool isFeedback(const CanFrame & frame);
std::optional<Feedback> parseFeedback(const CanFrame & frame);

/// 피드백 error 코드 설명
const char * errorString(uint8_t error);

}  // namespace gc_arm::ak::servo
