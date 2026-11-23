#pragma once

#include <chrono>
#include <deque>
#include <initializer_list>
#include <map>
#include <mutex>
#include <set>

#include "gc_arm_drivers/can_bus.hpp"

namespace gc_arm
{

/// 실제 프로토콜 프레임으로 동작하는 가상 CAN 버스 + 모터 모델.
/// 하드웨어 없이 ArmDriver / ros2_control 전체 경로(프레임 생성·해석 포함)를 시험하기 위한 용도.
///
/// - AK(Servo): SetPos/SetPosSpd/SetRpm/SetCurrent(0=정지)/SetOrigin 처리,
///   feedback_rate_hz 주기로 0x29 피드백 송신. 속도 제한 내 등속 이동 (가속 무시)
/// - MKS: 0xF3 enable, 0xF5 절대위치, 0xF6 속도, 0x31 엔코더, 0x92 영점, 0xF7 정지 처리.
///   enable 상태에서만 이동
class SimulatedBus : public CanBus
{
public:
  /// @param deg_s_per_erpm  출력축 [deg/s] / ERPM  = 6 / (극쌍수 × 내장감속비)
  void addAk(uint8_t id, double deg_s_per_erpm, double initial_deg = 0.0);
  void addMks(uint16_t id, int64_t initial_counts = 0);

  bool send(const CanFrame & frame) override;
  bool receive(CanFrame & out, std::chrono::microseconds timeout) override;

  // ---- 시험용 장애 주입 / 조회 ----
  void setSilent(uint32_t id, bool silent);   ///< true면 해당 모터 응답/피드백 중단
  void setAkError(uint8_t id, uint8_t error);
  double akPositionDeg(uint8_t id) const;
  int64_t mksCounts(uint16_t id) const;
  bool mksEnabled(uint16_t id) const;
  size_t sentFrames() const;

  double feedback_rate_hz{200.0};

private:
  using Clock = std::chrono::steady_clock;

  struct Ak
  {
    double pos_deg{0}, target_deg{0}, vmax_deg_s{0}, vel_deg_s{0};
    double k{0};  // deg/s per erpm
    bool position_mode{false};
    double current{0};
    uint8_t error{0};
    Clock::time_point next_fb{};
  };
  struct Mks
  {
    double counts{0}, target{0}, vmax_cps{0}, vel_cps{0};
    bool enabled{false}, position_mode{false};
  };

  void step(Clock::time_point now);
  void handle(const CanFrame & f);
  void pushMksReply(uint16_t id, std::initializer_list<uint8_t> data);

  mutable std::mutex mutex_;
  std::map<uint8_t, Ak> ak_;
  std::map<uint16_t, Mks> mks_;
  std::set<uint32_t> silent_;
  std::deque<CanFrame> rx_;
  Clock::time_point last_step_{Clock::now()};
  size_t sent_{0};
};

}  // namespace gc_arm
