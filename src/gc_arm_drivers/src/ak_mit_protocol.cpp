#include "gc_arm_drivers/ak_mit_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace gc_arm::ak::mit
{

namespace
{

CanFrame makeSpecial(uint8_t driver_id, uint8_t last)
{
  CanFrame f;
  f.id = driver_id;
  f.extended = false;
  f.dlc = 8;
  f.data.fill(0xFF);
  f.data[7] = last;
  return f;
}

}  // namespace

uint32_t floatToUint(double x, double x_min, double x_max, int bits)
{
  const double span = x_max - x_min;
  const uint32_t max_int = (1u << bits) - 1u;
  x = std::clamp(x, x_min, x_max);
  return static_cast<uint32_t>(std::lround((x - x_min) * max_int / span));
}

double uintToFloat(uint32_t x, double x_min, double x_max, int bits)
{
  const double span = x_max - x_min;
  const uint32_t max_int = (1u << bits) - 1u;
  return static_cast<double>(x) * span / max_int + x_min;
}

CanFrame makeEnterMotorMode(uint8_t driver_id) {return makeSpecial(driver_id, 0xFC);}
CanFrame makeExitMotorMode(uint8_t driver_id) {return makeSpecial(driver_id, 0xFD);}
CanFrame makeSetZero(uint8_t driver_id) {return makeSpecial(driver_id, 0xFE);}

CanFrame makeCommand(uint8_t driver_id, const Limits & l, const Command & c)
{
  const uint32_t p = floatToUint(c.position_rad, l.p_min, l.p_max, 16);
  const uint32_t v = floatToUint(c.velocity_rad_s, l.v_min, l.v_max, 12);
  const uint32_t kp = floatToUint(c.kp, l.kp_min, l.kp_max, 12);
  const uint32_t kd = floatToUint(c.kd, l.kd_min, l.kd_max, 12);
  const uint32_t t = floatToUint(c.torque_ff_nm, l.t_min, l.t_max, 12);

  CanFrame f;
  f.id = driver_id;
  f.extended = false;
  f.dlc = 8;
  f.data[0] = static_cast<uint8_t>(p >> 8);
  f.data[1] = static_cast<uint8_t>(p);
  f.data[2] = static_cast<uint8_t>(v >> 4);
  f.data[3] = static_cast<uint8_t>(((v & 0xF) << 4) | (kp >> 8));
  f.data[4] = static_cast<uint8_t>(kp);
  f.data[5] = static_cast<uint8_t>(kd >> 4);
  f.data[6] = static_cast<uint8_t>(((kd & 0xF) << 4) | (t >> 8));
  f.data[7] = static_cast<uint8_t>(t);
  return f;
}

std::optional<Feedback> parseFeedback(const CanFrame & frame, const Limits & l)
{
  if (frame.error || frame.extended || frame.dlc < 6) {
    return std::nullopt;
  }
  const auto & d = frame.data;
  const uint32_t p = (static_cast<uint32_t>(d[1]) << 8) | d[2];
  const uint32_t v = (static_cast<uint32_t>(d[3]) << 4) | (d[4] >> 4);
  const uint32_t t = (static_cast<uint32_t>(d[4] & 0xF) << 8) | d[5];

  Feedback fb;
  fb.driver_id = d[0];
  fb.position_rad = uintToFloat(p, l.p_min, l.p_max, 16);
  fb.velocity_rad_s = uintToFloat(v, l.v_min, l.v_max, 12);
  fb.torque_nm = uintToFloat(t, l.t_min, l.t_max, 12);
  if (frame.dlc >= 8) {
    fb.temperature_c = static_cast<int8_t>(d[6]);
    fb.error = d[7];
  }
  return fb;
}

}  // namespace gc_arm::ak::mit
