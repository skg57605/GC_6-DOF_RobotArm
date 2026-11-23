#include "gc_arm_drivers/mks_protocol.hpp"

#include <algorithm>
#include <cstdlib>
#include <initializer_list>

#include "gc_arm_drivers/byte_utils.hpp"

namespace gc_arm::mks
{

namespace
{

/// [cmd, payload..., crc] 프레임 생성
CanFrame makeFrame(uint16_t id, Cmd cmd, std::initializer_list<uint8_t> payload)
{
  CanFrame f;
  f.id = id & 0x7FFu;
  f.extended = false;
  f.data[0] = static_cast<uint8_t>(cmd);
  uint8_t n = 1;
  for (uint8_t b : payload) {
    f.data[n++] = b;
  }
  f.data[n] = crc(f.id, f.data.data(), n);
  f.dlc = static_cast<uint8_t>(n + 1);
  return f;
}

CanFrame makeAxisCmd(uint16_t id, Cmd cmd, uint16_t speed_rpm, uint8_t acc, int32_t axis)
{
  speed_rpm = std::min(speed_rpm, kMaxSpeedRpm);
  axis = std::clamp(axis, kAxisMin, kAxisMax);
  uint8_t s[2];
  uint8_t a[3];
  bytes::putBe16(s, speed_rpm);
  bytes::putBe24(a, static_cast<uint32_t>(axis) & 0xFFFFFFu);
  return makeFrame(id, cmd, {s[0], s[1], acc, a[0], a[1], a[2]});
}

}  // namespace

uint8_t crc(uint16_t can_id, const uint8_t * data, size_t len)
{
  uint32_t sum = can_id;
  for (size_t i = 0; i < len; ++i) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFFu);
}

bool checkCrc(const CanFrame & frame)
{
  if (frame.error || frame.extended || frame.dlc < 2) {
    return false;
  }
  return frame.data[frame.dlc - 1] ==
         crc(static_cast<uint16_t>(frame.id), frame.data.data(), frame.dlc - 1);
}

CanFrame makeReadEncoderAddition(uint16_t id) {return makeFrame(id, Cmd::ReadEncoderAddition, {});}
CanFrame makeReadEncoderCarry(uint16_t id) {return makeFrame(id, Cmd::ReadEncoderCarry, {});}
CanFrame makeReadSpeed(uint16_t id) {return makeFrame(id, Cmd::ReadSpeed, {});}
CanFrame makeQueryStatus(uint16_t id) {return makeFrame(id, Cmd::QueryStatus, {});}
CanFrame makeReadProtection(uint16_t id) {return makeFrame(id, Cmd::ReadProtection, {});}

CanFrame makeEnable(uint16_t id, bool enable)
{
  return makeFrame(id, Cmd::Enable, {static_cast<uint8_t>(enable ? 1 : 0)});
}

CanFrame makeSpeed(uint16_t id, int rpm, uint8_t acc)
{
  const uint8_t dir = rpm < 0 ? 1 : 0;
  const uint16_t speed = static_cast<uint16_t>(std::min(std::abs(rpm), static_cast<int>(kMaxSpeedRpm)));
  const uint8_t b1 = static_cast<uint8_t>((dir << 7) | ((speed >> 8) & 0x0F));
  const uint8_t b2 = static_cast<uint8_t>(speed & 0xFF);
  return makeFrame(id, Cmd::Speed, {b1, b2, acc});
}

CanFrame makePosAbsAxis(uint16_t id, uint16_t speed_rpm, uint8_t acc, int32_t axis)
{
  return makeAxisCmd(id, Cmd::PosAbsAxis, speed_rpm, acc, axis);
}

CanFrame makePosRelAxis(uint16_t id, uint16_t speed_rpm, uint8_t acc, int32_t rel_axis)
{
  return makeAxisCmd(id, Cmd::PosRelAxis, speed_rpm, acc, rel_axis);
}

CanFrame makeEmergencyStop(uint16_t id) {return makeFrame(id, Cmd::EmergencyStop, {});}
CanFrame makeSetZero(uint16_t id) {return makeFrame(id, Cmd::SetZero, {});}
CanFrame makeReleaseStall(uint16_t id) {return makeFrame(id, Cmd::ReleaseStall, {});}

CanFrame makeSetMode(uint16_t id, WorkMode mode)
{
  return makeFrame(id, Cmd::SetMode, {static_cast<uint8_t>(mode)});
}

CanFrame makeSetCurrent(uint16_t id, uint16_t milliamps)
{
  return makeFrame(
    id, Cmd::SetCurrent,
    {static_cast<uint8_t>(milliamps >> 8), static_cast<uint8_t>(milliamps & 0xFF)});
}

CanFrame makeSetCanBitrate(uint16_t id, Bitrate rate)
{
  return makeFrame(id, Cmd::SetCanBitrate, {static_cast<uint8_t>(rate)});
}

CanFrame makeSetCanId(uint16_t id, uint16_t new_id)
{
  new_id &= 0x7FFu;
  return makeFrame(
    id, Cmd::SetCanId, {static_cast<uint8_t>(new_id >> 8), static_cast<uint8_t>(new_id & 0xFF)});
}

CanFrame makeSetResponse(uint16_t id, bool respond, bool active)
{
  return makeFrame(
    id, Cmd::SetResponse,
    {static_cast<uint8_t>(respond ? 1 : 0), static_cast<uint8_t>(active ? 1 : 0)});
}

std::optional<uint8_t> responseCmd(const CanFrame & frame)
{
  if (!checkCrc(frame)) {
    return std::nullopt;
  }
  return frame.data[0];
}

std::optional<int64_t> parseEncoderAddition(const CanFrame & frame)
{
  if (frame.dlc != 8 || !checkCrc(frame) ||
    frame.data[0] != static_cast<uint8_t>(Cmd::ReadEncoderAddition))
  {
    return std::nullopt;
  }
  return bytes::getBeInt48(&frame.data[1]);
}

std::optional<int16_t> parseSpeed(const CanFrame & frame)
{
  if (frame.dlc != 4 || !checkCrc(frame) || frame.data[0] != static_cast<uint8_t>(Cmd::ReadSpeed)) {
    return std::nullopt;
  }
  return static_cast<int16_t>(bytes::getBe16(&frame.data[1]));
}

std::optional<uint8_t> parseStatus(const CanFrame & frame, Cmd expected)
{
  if (frame.dlc != 3 || !checkCrc(frame) || frame.data[0] != static_cast<uint8_t>(expected)) {
    return std::nullopt;
  }
  return frame.data[1];
}

const char * queryStatusString(uint8_t status)
{
  switch (status) {
    case 0: return "조회 실패";
    case 1: return "정지";
    case 2: return "가속 중";
    case 3: return "감속 중";
    case 4: return "정속(최고속) 운전";
    case 5: return "원점 복귀 중";
    case 6: return "캘리브레이션 중";
    default: return "알 수 없음";
  }
}

const char * positionStatusString(uint8_t status)
{
  switch (status) {
    case 0: return "실패";
    case 1: return "동작 시작";
    case 2: return "동작 완료";
    case 3: return "리밋에 의해 정지";
    default: return "알 수 없음";
  }
}

}  // namespace gc_arm::mks
