#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "gc_arm_drivers/can_frame.hpp"

/// MKS SERVO42D / SERVO57D CAN 프로토콜
///
/// - 표준(11bit) 프레임, ID = 슬레이브 주소 (1 ~ 0x7FF, 0 = 브로드캐스트 → 사용 금지)
/// - data = [명령코드, 파라미터..., CRC],  CRC = (ID + 명령코드 + 파라미터 바이트 합) & 0xFF
/// - 응답도 같은 ID, data[0] = 명령코드
/// - 멀티바이트 필드는 빅엔디언
/// - 엔코더 "addition" 값: 1회전 = 0x4000 (16384) count
namespace gc_arm::mks
{

constexpr int32_t kCountsPerRev = 0x4000;
constexpr uint16_t kMaxSpeedRpm = 3000;
constexpr int32_t kAxisMax = 0x7FFFFF;    ///< int24 범위 (F4/F5)
constexpr int32_t kAxisMin = -0x800000;

enum class Cmd : uint8_t
{
  ReadEncoderCarry = 0x30,
  ReadEncoderAddition = 0x31,
  ReadSpeed = 0x32,
  ReadPulses = 0x33,
  ReadAngleError = 0x39,
  ReadEnPin = 0x3A,
  ReadHomeStatus = 0x3B,
  ReleaseStall = 0x3D,
  ReadProtection = 0x3E,
  Calibrate = 0x80,
  SetMode = 0x82,
  SetCurrent = 0x83,
  SetMicrostep = 0x84,
  SetDirection = 0x86,
  SetCanBitrate = 0x8A,
  SetCanId = 0x8B,
  SetResponse = 0x8C,
  GoHome = 0x91,
  SetZero = 0x92,
  QueryStatus = 0xF1,
  Enable = 0xF3,
  PosRelAxis = 0xF4,
  PosAbsAxis = 0xF5,
  Speed = 0xF6,
  EmergencyStop = 0xF7,
  PosRelPulses = 0xFD,
  PosAbsPulses = 0xFE,
};

enum class WorkMode : uint8_t
{
  CR_OPEN = 0, CR_CLOSE = 1, CR_vFOC = 2, SR_OPEN = 3, SR_CLOSE = 4, SR_vFOC = 5,
};

enum class Bitrate : uint8_t {k125 = 0, k250 = 1, k500 = 2, k1000 = 3};

uint8_t crc(uint16_t can_id, const uint8_t * data, size_t len);
bool checkCrc(const CanFrame & frame);

// ---- 읽기 명령 ----
CanFrame makeReadEncoderAddition(uint16_t id);
CanFrame makeReadEncoderCarry(uint16_t id);
CanFrame makeReadSpeed(uint16_t id);
CanFrame makeQueryStatus(uint16_t id);
CanFrame makeReadProtection(uint16_t id);

// ---- 동작 명령 ----
CanFrame makeEnable(uint16_t id, bool enable);
/// 속도 모드. rpm 부호 = 방향, |rpm| ≤ 3000, acc 0~255 (0 = 가감속 없음)
CanFrame makeSpeed(uint16_t id, int rpm, uint8_t acc);
/// 절대 위치(축 좌표, count). 동작 중 목표 갱신 가능
CanFrame makePosAbsAxis(uint16_t id, uint16_t speed_rpm, uint8_t acc, int32_t axis);
/// 상대 위치(축 좌표, count)
CanFrame makePosRelAxis(uint16_t id, uint16_t speed_rpm, uint8_t acc, int32_t rel_axis);
CanFrame makeEmergencyStop(uint16_t id);
CanFrame makeSetZero(uint16_t id);       ///< 현재 위치를 축 좌표 0으로
CanFrame makeReleaseStall(uint16_t id);  ///< 구속(stall) 보호 해제

// ---- 설정 명령 (플래시 저장 — 주의해서 사용) ----
CanFrame makeSetMode(uint16_t id, WorkMode mode);
CanFrame makeSetCurrent(uint16_t id, uint16_t milliamps);
CanFrame makeSetCanBitrate(uint16_t id, Bitrate rate);
CanFrame makeSetCanId(uint16_t id, uint16_t new_id);
/// respond: 명령 응답 여부, active: 위치 명령 완료 시 능동 업로드 여부
CanFrame makeSetResponse(uint16_t id, bool respond, bool active);

// ---- 응답 해석 ----
/// 유효한 MKS 응답(CRC 정상, dlc ≥ 2)이면 명령코드 반환
std::optional<uint8_t> responseCmd(const CanFrame & frame);
std::optional<int64_t> parseEncoderAddition(const CanFrame & frame);  ///< 0x31, int48 count
std::optional<int16_t> parseSpeed(const CanFrame & frame);            ///< 0x32, rpm
/// 1바이트 상태 응답 (F1, F3, F4, F5, F6, 92, 3D, 8x 설정 등)
std::optional<uint8_t> parseStatus(const CanFrame & frame, Cmd expected);

const char * queryStatusString(uint8_t status);   ///< F1
const char * positionStatusString(uint8_t status);  ///< F4/F5/FD/FE

/// 축 좌표 ↔ 모터축 각도
inline double countsToDeg(int64_t counts) {return counts * 360.0 / kCountsPerRev;}
inline double degToCounts(double deg) {return deg * kCountsPerRev / 360.0;}

}  // namespace gc_arm::mks
