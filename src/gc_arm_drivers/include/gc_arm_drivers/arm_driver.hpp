#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gc_arm_drivers/actuator.hpp"
#include "gc_arm_drivers/can_bus.hpp"
#include "gc_arm_drivers/joint_config.hpp"

namespace gc_arm
{

class SimulatedBus;

/// 로봇팔 전체 구동기 (ROS 비의존). ros2_control 하드웨어 플러그인이 이 클래스를 감싼다.
///
/// - 하나의 CAN 버스를 소유하고, 수신 전용 백그라운드 스레드가 프레임을 각 액추에이터로 분배
/// - write(): 명령 프레임 + 폴링 프레임 송신 (제어 스레드)
/// - read(): 최신 상태 복사 + 안전 검사 (블로킹 없음)
///
/// 수명주기: configure → activate → (read/write 반복) → deactivate → cleanup
class ArmDriver
{
public:
  struct Options
  {
    /// "can0" 등 SocketCAN 인터페이스, 또는 "sim" (SimulatedBus 사용)
    std::string can_interface{"can0"};
    /// activate 시 모든 버스 관절의 첫 피드백을 기다리는 시간
    std::chrono::milliseconds activate_timeout{1000};
  };

  ArmDriver();
  ~ArmDriver();
  ArmDriver(const ArmDriver &) = delete;
  ArmDriver & operator=(const ArmDriver &) = delete;

  bool configure(const Options & options, const std::vector<JointConfig> & joints, std::string * error);
  bool activate(std::string * error);
  /// 명령 루프 종료. 활성 상태였다면 위치 유지 스레드를 시작해 마지막 목표를 계속 송신한다
  /// (AK는 명령이 1 s 끊기면 토크 해제 → 중력 관절 처짐 방지). cleanup() 전까지 유지.
  void deactivate();
  /// 위치 유지 중단 + 버스 해제. AK는 이후 timeout 뒤 토크 해제됨
  void cleanup();
  bool holding() const {return holding_.load();}

  size_t size() const {return actuators_.size();}
  const JointConfig & jointConfig(size_t i) const {return actuators_[i]->config();}

  /// 최신 상태 복사. 피드백 타임아웃/드라이버 에러/과열/버스 오류 시 false
  bool read(std::vector<ActuatorState> & states, std::string * error);
  /// 명령 송신. velocities 원소가 NaN이면 속도 명령 없음으로 처리. 버스 송신 실패 시 false
  bool write(const std::vector<double> & positions, const std::vector<double> & velocities, std::string * error);

  /// mock 관절의 초기 위치 설정 (configure 이후)
  void setMockPosition(size_t i, double q);

  /// can_interface="sim"일 때 시뮬레이션 버스 접근 (시험용), 아니면 nullptr
  SimulatedBus * simulatedBus() {return sim_bus_;}

  /// 마지막 activate에서 결정된 시작 자세 보정 내역 (로그용)
  const std::vector<std::string> & startupLog() const {return startup_log_;}

  size_t errorFrameCount() const {return error_frames_.load();}
  size_t unknownFrameCount() const {return unknown_frames_.load();}

private:
  void rxLoop();
  void holdLoop();
  void startHold();
  void stopHold();
  bool sendAll(const std::vector<CanFrame> & frames, std::string * error);
  bool anyBusJoint() const;

  std::vector<std::unique_ptr<Actuator>> actuators_;
  std::unique_ptr<CanBus> bus_;
  SimulatedBus * sim_bus_{nullptr};
  std::thread rx_thread_;
  std::thread hold_thread_;
  std::atomic<bool> holding_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> bus_off_{false};
  std::atomic<size_t> error_frames_{0};
  std::atomic<size_t> unknown_frames_{0};
  bool active_{false};
  std::chrono::milliseconds activate_timeout_{1000};
  std::chrono::milliseconds hold_period_{50};
  std::vector<CanFrame> scratch_;
  std::vector<std::string> startup_log_;
};

}  // namespace gc_arm
