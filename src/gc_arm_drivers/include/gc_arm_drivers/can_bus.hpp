#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "gc_arm_drivers/can_frame.hpp"
#include "gc_arm_drivers/socketcan.hpp"

namespace gc_arm
{

/// CAN 버스 추상 인터페이스 — 실제 SocketCAN과 시뮬레이션 버스를 같은 코드로 다루기 위함.
/// send()와 receive()는 서로 다른 스레드에서 동시에 호출될 수 있어야 한다.
class CanBus
{
public:
  virtual ~CanBus() = default;
  virtual bool send(const CanFrame & frame) = 0;
  virtual bool receive(CanFrame & out, std::chrono::microseconds timeout) = 0;
  virtual std::string lastError() const {return {};}
};

/// SocketCan → CanBus 어댑터
class SocketCanBus : public CanBus
{
public:
  bool open(const std::string & ifname, std::string * error_msg)
  {
    return socket_.open(ifname, true, error_msg);
  }
  bool send(const CanFrame & frame) override
  {
    return socket_.send(frame) == SocketCan::SendResult::Ok;
  }
  bool receive(CanFrame & out, std::chrono::microseconds timeout) override
  {
    return socket_.receive(out, timeout);
  }
  std::string lastError() const override {return socket_.lastError();}

private:
  SocketCan socket_;
};

}  // namespace gc_arm
