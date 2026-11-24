#include "gc_arm_drivers/simulated_bus.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

#include "gc_arm_drivers/ak_servo_protocol.hpp"
#include "gc_arm_drivers/byte_utils.hpp"
#include "gc_arm_drivers/mks_protocol.hpp"

namespace gc_arm
{

namespace
{

/// 위치를 목표로 최대 속도 이내에서 이동, 실제 속도 반환
double moveToward(double & pos, double target, double vmax, double dt)
{
  const double err = target - pos;
  const double max_step = std::fabs(vmax) * dt;
  const double step = std::clamp(err, -max_step, max_step);
  pos += step;
  return dt > 0 ? step / dt : 0.0;
}

}  // namespace

void SimulatedBus::addAk(uint8_t id, double deg_s_per_erpm, double initial_deg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  Ak a;
  a.k = deg_s_per_erpm;
  a.pos_deg = a.target_deg = initial_deg;
  ak_[id] = a;
}

void SimulatedBus::addMks(uint16_t id, int64_t initial_counts)
{
  std::lock_guard<std::mutex> lk(mutex_);
  Mks m;
  m.counts = m.target = static_cast<double>(initial_counts);
  mks_[id] = m;
}

void SimulatedBus::setSilent(uint32_t id, bool silent)
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (silent) {silent_.insert(id);} else {silent_.erase(id);}
}

void SimulatedBus::setAkError(uint8_t id, uint8_t error)
{
  std::lock_guard<std::mutex> lk(mutex_);
  ak_[id].error = error;
}

void SimulatedBus::setAkWrapFeedback(bool wrap)
{
  std::lock_guard<std::mutex> lk(mutex_);
  ak_wrap_feedback_ = wrap;
}

void SimulatedBus::setAkPosition(uint8_t id, double deg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  ak_[id].pos_deg = ak_[id].target_deg = deg;
}

bool SimulatedBus::akReleased(uint8_t id) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = ak_.find(id);
  return it != ak_.end() && it->second.released;
}

double SimulatedBus::akPositionDeg(uint8_t id) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = ak_.find(id);
  return it == ak_.end() ? 0.0 : it->second.pos_deg;
}

int64_t SimulatedBus::mksCounts(uint16_t id) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = mks_.find(id);
  return it == mks_.end() ? 0 : static_cast<int64_t>(std::llround(it->second.counts));
}

bool SimulatedBus::mksEnabled(uint16_t id) const
{
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = mks_.find(id);
  return it != mks_.end() && it->second.enabled;
}

size_t SimulatedBus::sentFrames() const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return sent_;
}

void SimulatedBus::step(Clock::time_point now)
{
  const double dt = std::chrono::duration<double>(now - last_step_).count();
  last_step_ = now;

  for (auto & [id, a] : ak_) {
    if (a.engaged && std::chrono::duration<double>(now - a.last_cmd).count() > ak_timeout_s) {
      a.engaged = false;  // 명령 timeout → 토크 해제 (실기 확인 동작)
      a.released = true;
      a.position_mode = false;
      a.vel_deg_s = 0;
    }
    if (a.position_mode) {
      a.vel_deg_s = moveToward(a.pos_deg, a.target_deg, a.vmax_deg_s, dt);
    } else {
      a.pos_deg += a.vel_deg_s * dt;
    }
    if (now >= a.next_fb && !silent_.count(id)) {
      a.next_fb = now + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / feedback_rate_hz));
      CanFrame f;
      f.extended = true;
      f.id = (static_cast<uint32_t>(ak::servo::kFeedbackPacketId) << 8) | id;
      f.dlc = 8;
      double reported = a.pos_deg;
      if (ak_wrap_feedback_) {
        reported = std::fmod(reported, 360.0);
        if (reported < 0) {reported += 360.0;}
      }
      bytes::putBe16(&f.data[0], static_cast<uint16_t>(bytes::saturateRound<int16_t>(reported * 10.0)));
      bytes::putBe16(&f.data[2], static_cast<uint16_t>(bytes::saturateRound<int16_t>(a.vel_deg_s / a.k / 10.0)));
      bytes::putBe16(&f.data[4], static_cast<uint16_t>(bytes::saturateRound<int16_t>(a.current * 100.0)));
      f.data[6] = 30;
      f.data[7] = a.error;
      rx_.push_back(f);
    }
  }

  for (auto & [id, m] : mks_) {
    if (!m.enabled) {
      m.vel_cps = 0;
      continue;
    }
    if (m.position_mode) {
      m.vel_cps = moveToward(m.counts, m.target, m.vmax_cps, dt);
    } else {
      m.counts += m.vel_cps * dt;
    }
  }
}

void SimulatedBus::pushMksReply(uint16_t id, std::initializer_list<uint8_t> data)
{
  if (silent_.count(id)) {
    return;
  }
  CanFrame f;
  f.id = id;
  uint8_t n = 0;
  for (uint8_t b : data) {f.data[n++] = b;}
  f.data[n] = mks::crc(id, f.data.data(), n);
  f.dlc = static_cast<uint8_t>(n + 1);
  rx_.push_back(f);
}

void SimulatedBus::handle(const CanFrame & f)
{
  if (f.extended) {
    const uint8_t id = static_cast<uint8_t>(f.id & 0xFF);
    const uint8_t packet = static_cast<uint8_t>((f.id >> 8) & 0xFF);
    auto it = ak_.find(id);
    if (it == ak_.end()) {return;}
    Ak & a = it->second;
    const uint8_t * d = f.data.data();
    a.last_cmd = Clock::now();
    a.engaged = true;
    a.released = false;
    switch (static_cast<ak::servo::PacketId>(packet)) {
      case ak::servo::PacketId::SetPos:
        a.position_mode = true;
        a.target_deg = static_cast<int32_t>(bytes::getBe32(d)) / 10000.0;
        a.vmax_deg_s = 360.0;
        break;
      case ak::servo::PacketId::SetPosSpd:
        a.position_mode = true;
        a.target_deg = static_cast<int32_t>(bytes::getBe32(d)) / 10000.0;
        a.vmax_deg_s = static_cast<int16_t>(bytes::getBe16(d + 4)) * 10.0 * a.k;
        break;
      case ak::servo::PacketId::SetRpm:
        a.position_mode = false;
        a.vel_deg_s = static_cast<int32_t>(bytes::getBe32(d)) * a.k;
        break;
      case ak::servo::PacketId::SetCurrent:
        a.position_mode = false;
        a.current = static_cast<int32_t>(bytes::getBe32(d)) / 1000.0;
        a.vel_deg_s = 0;
        break;
      case ak::servo::PacketId::SetOrigin:
        a.pos_deg = a.target_deg = 0;
        break;
      default:
        break;
    }
    return;
  }

  auto it = mks_.find(static_cast<uint16_t>(f.id));
  if (it == mks_.end() || !mks::checkCrc(f)) {return;}
  Mks & m = it->second;
  const uint16_t id = static_cast<uint16_t>(f.id);
  const uint8_t cmd = f.data[0];
  switch (static_cast<mks::Cmd>(cmd)) {
    case mks::Cmd::ReadEncoderAddition: {
        const int64_t c = static_cast<int64_t>(std::llround(m.counts));
        uint8_t b[6];
        for (int i = 0; i < 6; ++i) {b[i] = static_cast<uint8_t>(c >> (8 * (5 - i)));}
        pushMksReply(id, {cmd, b[0], b[1], b[2], b[3], b[4], b[5]});
        break;
      }
    case mks::Cmd::Enable:
      m.enabled = f.data[1] != 0;
      pushMksReply(id, {cmd, 1});
      break;
    case mks::Cmd::PosAbsAxis: {
        const uint16_t rpm = bytes::getBe16(&f.data[1]);
        m.position_mode = true;
        m.vmax_cps = rpm * mks::kCountsPerRev / 60.0;
        m.target = bytes::getBeInt24(&f.data[4]);
        pushMksReply(id, {cmd, static_cast<uint8_t>(m.enabled ? 1 : 0)});
        break;
      }
    case mks::Cmd::Speed: {
        const int dir = (f.data[1] & 0x80) ? -1 : 1;
        const int rpm = ((f.data[1] & 0x0F) << 8) | f.data[2];
        m.position_mode = false;
        m.vel_cps = dir * rpm * mks::kCountsPerRev / 60.0;
        pushMksReply(id, {cmd, 1});
        break;
      }
    case mks::Cmd::SetZero:
      m.counts = m.target = 0;
      pushMksReply(id, {cmd, 1});
      break;
    case mks::Cmd::EmergencyStop:
      m.position_mode = false;
      m.vel_cps = 0;
      pushMksReply(id, {cmd, 1});
      break;
    default:
      break;
  }
}

bool SimulatedBus::send(const CanFrame & frame)
{
  std::lock_guard<std::mutex> lk(mutex_);
  ++sent_;
  step(Clock::now());
  handle(frame);
  return true;
}

bool SimulatedBus::receive(CanFrame & out, std::chrono::microseconds timeout)
{
  const auto deadline = Clock::now() + timeout;
  while (true) {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      step(Clock::now());
      if (!rx_.empty()) {
        out = rx_.front();
        rx_.pop_front();
        return true;
      }
    }
    if (Clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
}

}  // namespace gc_arm
