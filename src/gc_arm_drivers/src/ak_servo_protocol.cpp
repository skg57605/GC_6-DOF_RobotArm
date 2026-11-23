#include "gc_arm_drivers/ak_servo_protocol.hpp"

#include "gc_arm_drivers/byte_utils.hpp"

namespace gc_arm::ak::servo
{

namespace
{

CanFrame makeFrame(uint8_t driver_id, PacketId packet, uint8_t dlc)
{
  CanFrame f;
  f.extended = true;
  f.id = makeId(driver_id, packet);
  f.dlc = dlc;
  return f;
}

CanFrame makeInt32(uint8_t driver_id, PacketId packet, double scaled)
{
  CanFrame f = makeFrame(driver_id, packet, 4);
  bytes::putBe32(f.data.data(), static_cast<uint32_t>(bytes::saturateRound<int32_t>(scaled)));
  return f;
}

}  // namespace

CanFrame makeDuty(uint8_t driver_id, double duty)
{
  return makeInt32(driver_id, PacketId::SetDuty, duty * 100000.0);
}

CanFrame makeCurrent(uint8_t driver_id, double current_a)
{
  return makeInt32(driver_id, PacketId::SetCurrent, current_a * 1000.0);
}

CanFrame makeCurrentBrake(uint8_t driver_id, double current_a)
{
  return makeInt32(driver_id, PacketId::SetCurrentBrake, current_a * 1000.0);
}

CanFrame makeRpm(uint8_t driver_id, double erpm)
{
  return makeInt32(driver_id, PacketId::SetRpm, erpm);
}

CanFrame makePosition(uint8_t driver_id, double position_deg)
{
  return makeInt32(driver_id, PacketId::SetPos, position_deg * 10000.0);
}

CanFrame makePositionSpeed(
  uint8_t driver_id, double position_deg, double speed_erpm, double accel_erpm_s2)
{
  CanFrame f = makeFrame(driver_id, PacketId::SetPosSpd, 8);
  bytes::putBe32(
    &f.data[0], static_cast<uint32_t>(bytes::saturateRound<int32_t>(position_deg * 10000.0)));
  bytes::putBe16(
    &f.data[4], static_cast<uint16_t>(bytes::saturateRound<int16_t>(speed_erpm / 10.0)));
  bytes::putBe16(
    &f.data[6], static_cast<uint16_t>(bytes::saturateRound<int16_t>(accel_erpm_s2 / 10.0)));
  return f;
}

CanFrame makeSetOrigin(uint8_t driver_id, OriginMode mode)
{
  CanFrame f = makeFrame(driver_id, PacketId::SetOrigin, 1);
  f.data[0] = static_cast<uint8_t>(mode);
  return f;
}

bool isFeedback(const CanFrame & frame)
{
  return !frame.error && frame.extended && ((frame.id >> 8) & 0xFFu) == kFeedbackPacketId &&
         (frame.id >> 16) == 0 && frame.dlc >= 8;
}

std::optional<Feedback> parseFeedback(const CanFrame & frame)
{
  if (!isFeedback(frame)) {
    return std::nullopt;
  }
  const uint8_t * d = frame.data.data();
  Feedback fb;
  fb.driver_id = static_cast<uint8_t>(frame.id & 0xFFu);
  fb.position_deg = static_cast<int16_t>(bytes::getBe16(&d[0])) * 0.1;
  fb.speed_erpm = static_cast<int16_t>(bytes::getBe16(&d[2])) * 10.0;
  fb.current_a = static_cast<int16_t>(bytes::getBe16(&d[4])) * 0.01;
  fb.temperature_c = static_cast<int8_t>(d[6]);
  fb.error = d[7];
  return fb;
}

const char * errorString(uint8_t error)
{
  switch (error) {
    case 0: return "정상";
    case 1: return "모터 과열";
    case 2: return "과전류";
    case 3: return "과전압";
    case 4: return "저전압";
    case 5: return "엔코더 이상";
    case 6: return "MOSFET 과열";
    case 7: return "모터 구속(stall)";
    default: return "알 수 없음";
  }
}

}  // namespace gc_arm::ak::servo
