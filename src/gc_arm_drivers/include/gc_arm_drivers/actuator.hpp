#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "gc_arm_drivers/can_frame.hpp"
#include "gc_arm_drivers/joint_config.hpp"

namespace gc_arm
{

using SteadyClock = std::chrono::steady_clock;

/// 관절 좌표계 상태
struct ActuatorState
{
  double position{0.0};     ///< [rad]
  double velocity{0.0};     ///< [rad/s]
  double effort{0.0};       ///< [Nm] (AK: kt 설정 시)
  double current{0.0};      ///< [A]  (AK)
  double temperature{0.0};  ///< [°C] (AK)
  uint8_t error{0};         ///< 드라이버 에러 코드 (AK)
  bool valid{false};        ///< 피드백을 한 번이라도 받았는지
  SteadyClock::time_point stamp{};  ///< 마지막 피드백 수신 시각
};

/// 관절 하나를 구동하는 액추에이터. 프로토콜 프레임 생성/해석과 단위 변환을 담당.
/// - 명령/폴링 프레임 생성: 제어 스레드
/// - handleFrame(): 수신 스레드
/// 상태는 내부 뮤텍스로 보호된다.
class Actuator
{
public:
  explicit Actuator(JointConfig config) : config_(std::move(config)) {}
  virtual ~Actuator() = default;

  const JointConfig & config() const {return config_;}
  virtual bool usesBus() const {return true;}

  /// 활성화 시 보낼 프레임 (예: MKS enable)
  virtual void activateFrames(std::vector<CanFrame> & /*out*/) {}
  /// 첫 피드백 수신 후, 활성화 직전에 호출 (예: AK 회전수 결정). 설명 문자열 반환(로그용)
  virtual std::string resolveStartup() {return {};}
  /// 비활성화 시 보낼 프레임
  virtual void deactivateFrames(std::vector<CanFrame> & /*out*/) {}
  /// 매 주기 상태 조회 요청 (예: MKS 0x31)
  virtual void pollFrames(std::vector<CanFrame> & /*out*/) {}
  /// 매 주기 명령. position [rad], velocity [rad/s] (NaN = 속도 명령 없음)
  virtual void commandFrames(double position, double velocity, std::vector<CanFrame> & out) = 0;
  /// 수신 프레임 처리. 이 액추에이터의 프레임이면 true
  virtual bool handleFrame(const CanFrame & frame, SteadyClock::time_point now) = 0;

  ActuatorState state() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return state_;
  }

protected:
  JointConfig config_;
  mutable std::mutex mutex_;
  ActuatorState state_;
};

/// 하드웨어 없는 관절: 명령을 그대로 상태로 반영 (mock_components와 동일한 동작)
class MockActuator : public Actuator
{
public:
  explicit MockActuator(JointConfig c);
  bool usesBus() const override {return false;}
  void commandFrames(double position, double velocity, std::vector<CanFrame> & out) override;
  bool handleFrame(const CanFrame &, SteadyClock::time_point) override {return false;}
  /// 초기 위치 설정
  void setPosition(double q);
};

/// CubeMars AK, Servo 모드. 명령 = 위치-속도 루프(mode 6), 상태 = 주기 피드백(0x29)
///
/// 위치 처리:
///  - 피드백 각도를 연속값으로 이어 붙임(unwrap): 한 번에 180° 넘게 바뀌면 360° 경계 통과로 판단.
///    (피드백이 0~360°로 되돌아가거나 int16 범위를 넘는 경우 모두 대응)
///  - resolveStartup(): startup_reference에 가장 가까운 회전수를 골라 turn_offset(360° 배수)을 정함.
///    명령은 다시 모터 기준 각도(= 관절 환산 각도 - turn_offset)로 보낸다.
class AkServoActuator : public Actuator
{
public:
  explicit AkServoActuator(JointConfig c) : Actuator(std::move(c)) {}
  void commandFrames(double position, double velocity, std::vector<CanFrame> & out) override;
  bool handleFrame(const CanFrame & frame, SteadyClock::time_point now) override;
  std::string resolveStartup() override;

  /// 시험/로그용: 모터 기준 연속 각도 [deg], 선택된 회전 보정 [deg]
  double motorAngleDeg() const;
  double turnOffsetDeg() const;

private:
  bool have_raw_{false};
  double last_raw_deg_{0.0};
  double unwrapped_deg_{0.0};   // 모터 기준 연속 각도
  double turn_offset_deg_{0.0}; // 360° 배수
};

/// MKS SERVO42D/57D. 명령 = 0xF5 절대위치(축 좌표), 상태 = 0x31 폴링 + 위치 미분 속도
class MksActuator : public Actuator
{
public:
  explicit MksActuator(JointConfig c) : Actuator(std::move(c)) {}
  void activateFrames(std::vector<CanFrame> & out) override;
  void pollFrames(std::vector<CanFrame> & out) override;
  void commandFrames(double position, double velocity, std::vector<CanFrame> & out) override;
  bool handleFrame(const CanFrame & frame, SteadyClock::time_point now) override;

private:
  std::optional<SteadyClock::time_point> last_stamp_;
  double last_position_{0.0};
};

std::unique_ptr<Actuator> makeActuator(const JointConfig & config);

}  // namespace gc_arm
