#include "gc_arm_drivers/arm_driver.hpp"

#include <linux/can/error.h>

#include <cmath>
#include <set>
#include <sstream>
#include <thread>

#include "gc_arm_drivers/joint_conversions.hpp"
#include "gc_arm_drivers/simulated_bus.hpp"

namespace gc_arm
{

namespace
{

void setError(std::string * error, const std::string & msg)
{
  if (error) {*error = msg;}
}

}  // namespace

ArmDriver::ArmDriver() = default;

ArmDriver::~ArmDriver()
{
  cleanup();
}

bool ArmDriver::anyBusJoint() const
{
  for (const auto & a : actuators_) {
    if (a->usesBus()) {return true;}
  }
  return false;
}

bool ArmDriver::configure(const Options & options, const std::vector<JointConfig> & joints, std::string * error)
{
  cleanup();

  // ID 중복 검사 (AK 확장 프레임과 MKS 표준 프레임은 ID 공간이 다름)
  std::set<uint16_t> ak_ids, mks_ids;
  for (const auto & j : joints) {
    auto & ids = j.type == ActuatorType::AkServo ? ak_ids : mks_ids;
    if (j.type != ActuatorType::Mock && !ids.insert(j.can_id).second) {
      setError(error, j.name + ": CAN ID " + std::to_string(j.can_id) + " 중복");
      return false;
    }
    actuators_.push_back(makeActuator(j));
  }

  if (!anyBusJoint()) {
    return true;  // 전부 mock → 버스 불필요
  }

  if (options.can_interface == "sim") {
    auto sim = std::make_unique<SimulatedBus>();
    for (const auto & j : joints) {
      if (j.type == ActuatorType::AkServo) {
        sim->addAk(static_cast<uint8_t>(j.can_id), 6.0 / (j.ak_pole_pairs * j.ak_internal_ratio));
      } else if (j.type == ActuatorType::Mks) {
        sim->addMks(j.can_id);
      }
    }
    sim_bus_ = sim.get();
    bus_ = std::move(sim);
  } else {
    auto sock = std::make_unique<SocketCanBus>();
    std::string err;
    if (!sock->open(options.can_interface, &err)) {
      setError(error, options.can_interface + " 열기 실패: " + err);
      actuators_.clear();
      return false;
    }
    bus_ = std::move(sock);
  }

  running_ = true;
  rx_thread_ = std::thread(&ArmDriver::rxLoop, this);
  activate_timeout_ = options.activate_timeout;
  return true;
}

void ArmDriver::rxLoop()
{
  CanFrame f;
  while (running_) {
    if (!bus_->receive(f, std::chrono::milliseconds(10))) {
      continue;
    }
    const auto now = SteadyClock::now();
    if (f.error) {
      ++error_frames_;
      if (f.id & CAN_ERR_BUSOFF) {bus_off_ = true;}
      continue;
    }
    bool handled = false;
    for (auto & a : actuators_) {
      if (a->usesBus() && a->handleFrame(f, now)) {
        handled = true;
        break;
      }
    }
    if (!handled) {++unknown_frames_;}
  }
}

bool ArmDriver::sendAll(const std::vector<CanFrame> & frames, std::string * error)
{
  for (const auto & f : frames) {
    if (!bus_->send(f)) {
      setError(error, "CAN 송신 실패 " + toString(f) + ": " + bus_->lastError());
      return false;
    }
  }
  return true;
}

bool ArmDriver::activate(std::string * error)
{
  if (!anyBusJoint()) {
    active_ = true;
    return true;
  }
  bus_off_ = false;

  scratch_.clear();
  for (auto & a : actuators_) {a->activateFrames(scratch_);}
  if (!sendAll(scratch_, error)) {return false;}

  // 모든 버스 관절의 첫 피드백 대기 (MKS는 폴링 필요)
  const auto deadline = SteadyClock::now() + activate_timeout_;
  while (SteadyClock::now() < deadline) {
    bool all_valid = true;
    scratch_.clear();
    for (auto & a : actuators_) {
      if (a->usesBus() && !a->state().valid) {
        all_valid = false;
        a->pollFrames(scratch_);
      }
    }
    if (all_valid) {
      active_ = true;
      return true;
    }
    if (!sendAll(scratch_, error)) {return false;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::ostringstream os;
  os << "피드백 없음:";
  for (auto & a : actuators_) {
    if (a->usesBus() && !a->state().valid) {
      os << ' ' << a->config().name << "(" << toString(a->config().type) << " id=" << a->config().can_id << ")";
    }
  }
  setError(error, os.str());
  return false;
}

void ArmDriver::deactivate()
{
  // 모터는 마지막 목표 위치를 유지한다 (AK 토크 해제 시 중력에 의해 팔이 떨어지므로 해제하지 않음)
  if (bus_ && active_) {
    scratch_.clear();
    for (auto & a : actuators_) {a->deactivateFrames(scratch_);}
    sendAll(scratch_, nullptr);
  }
  active_ = false;
}

void ArmDriver::cleanup()
{
  deactivate();
  running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  bus_.reset();
  sim_bus_ = nullptr;
  actuators_.clear();
}

void ArmDriver::setMockPosition(size_t i, double q)
{
  if (auto * m = dynamic_cast<MockActuator *>(actuators_.at(i).get())) {
    m->setPosition(q);
  }
}

bool ArmDriver::read(std::vector<ActuatorState> & states, std::string * error)
{
  states.resize(actuators_.size());
  const auto now = SteadyClock::now();
  bool ok = true;
  std::ostringstream os;
  for (size_t i = 0; i < actuators_.size(); ++i) {
    states[i] = actuators_[i]->state();
    const auto & c = actuators_[i]->config();
    if (!actuators_[i]->usesBus() || !active_) {
      continue;
    }
    const double age = std::chrono::duration<double>(now - states[i].stamp).count();
    if (!states[i].valid || age > c.feedback_timeout) {
      os << c.name << ": 피드백 타임아웃(" << static_cast<int>(age * 1000) << " ms) ";
      ok = false;
    }
    if (states[i].error != 0) {
      os << c.name << ": 드라이버 에러 코드 " << static_cast<int>(states[i].error) << ' ';
      ok = false;
    }
    if (states[i].temperature > c.max_temperature) {
      os << c.name << ": 과열 " << states[i].temperature << " C ";
      ok = false;
    }
  }
  if (bus_off_) {
    os << "CAN BUS-OFF ";
    ok = false;
  }
  if (!ok) {setError(error, os.str());}
  return ok;
}

bool ArmDriver::write(
  const std::vector<double> & positions, const std::vector<double> & velocities, std::string * error)
{
  if (positions.size() != actuators_.size() || velocities.size() != actuators_.size()) {
    setError(error, "명령 벡터 크기 불일치");
    return false;
  }
  scratch_.clear();
  for (size_t i = 0; i < actuators_.size(); ++i) {
    if (std::isnan(positions[i])) {
      continue;  // 아직 명령 없음 → 송신하지 않음 (모터는 현재 목표 유지)
    }
    actuators_[i]->commandFrames(positions[i], velocities[i], scratch_);
  }
  if (!active_ || !bus_) {
    return true;  // mock만 있거나 비활성 → 버스 송신 없음
  }
  for (auto & a : actuators_) {a->pollFrames(scratch_);}
  return sendAll(scratch_, error);
}

}  // namespace gc_arm
