#pragma once

#include <chrono>
#include <string>

#include "gc_arm_drivers/can_frame.hpp"

namespace gc_arm
{

/// Linux SocketCAN(CAN_RAW) 래퍼. 하나의 버스(can0)를 이 객체 하나가 소유한다.
class SocketCan
{
public:
  enum class SendResult { Ok, BufferFull, Error };

  SocketCan() = default;
  ~SocketCan();
  SocketCan(const SocketCan &) = delete;
  SocketCan & operator=(const SocketCan &) = delete;

  /// @param receive_errors true면 에러 프레임(bus-off, error passive 등)도 수신
  bool open(const std::string & ifname, bool receive_errors, std::string * error_msg = nullptr);
  void close();
  bool isOpen() const {return fd_ >= 0;}

  /// 송신. 커널 TX 큐가 가득 차면 BufferFull (txqueuelen 증가 권장)
  SendResult send(const CanFrame & frame);

  /// timeout 동안 한 프레임 수신 대기. 수신 시 true. timeout=0 → 즉시 반환(폴링)
  bool receive(CanFrame & out, std::chrono::microseconds timeout);

  int fd() const {return fd_;}
  const std::string & lastError() const {return last_error_;}

private:
  int fd_{-1};
  std::string last_error_;
};

/// 에러 프레임(id = CAN_ERR_* 클래스 비트)을 사람이 읽을 수 있는 문자열로
std::string describeErrorFrame(const CanFrame & frame);

}  // namespace gc_arm
