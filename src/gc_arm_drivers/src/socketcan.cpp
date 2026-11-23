#include "gc_arm_drivers/socketcan.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace gc_arm
{

std::string toString(const CanFrame & frame)
{
  char buf[64];
  int n = 0;
  if (frame.error) {
    n = std::snprintf(buf, sizeof(buf), "ERR(0x%08X)#", frame.id);
  } else if (frame.extended) {
    n = std::snprintf(buf, sizeof(buf), "%08X#", frame.id);
  } else {
    n = std::snprintf(buf, sizeof(buf), "%03X#", frame.id);
  }
  for (uint8_t i = 0; i < frame.dlc && i < 8; ++i) {
    n += std::snprintf(buf + n, sizeof(buf) - n, "%02X", frame.data[i]);
  }
  return std::string(buf, n);
}

SocketCan::~SocketCan()
{
  close();
}

bool SocketCan::open(const std::string & ifname, bool receive_errors, std::string * error_msg)
{
  close();
  auto fail = [&](const std::string & what) {
      last_error_ = what + ": " + std::strerror(errno);
      if (error_msg) {
        *error_msg = last_error_;
      }
      close();
      return false;
    };

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    return fail("socket(PF_CAN)");
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    return fail("ioctl(SIOCGIFINDEX, " + ifname + ")");
  }

  if (receive_errors) {
    can_err_mask_t err_mask = CAN_ERR_MASK;
    ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    return fail("bind(" + ifname + ")");
  }

  // 논블로킹: send가 TX 큐 포화 시 블록되지 않도록
  int flags = ::fcntl(fd_, F_GETFL, 0);
  ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  return true;
}

void SocketCan::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

SocketCan::SendResult SocketCan::send(const CanFrame & frame)
{
  if (fd_ < 0) {
    last_error_ = "socket not open";
    return SendResult::Error;
  }
  struct can_frame raw {};
  raw.can_id = frame.extended ? ((frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG) : (frame.id & CAN_SFF_MASK);
  raw.can_dlc = frame.dlc > 8 ? 8 : frame.dlc;
  std::memcpy(raw.data, frame.data.data(), raw.can_dlc);

  ssize_t n = ::write(fd_, &raw, sizeof(raw));
  if (n == static_cast<ssize_t>(sizeof(raw))) {
    return SendResult::Ok;
  }
  if (n < 0 && (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK)) {
    last_error_ = "TX buffer full";
    return SendResult::BufferFull;
  }
  last_error_ = std::string("write: ") + std::strerror(errno);
  return SendResult::Error;
}

bool SocketCan::receive(CanFrame & out, std::chrono::microseconds timeout)
{
  if (fd_ < 0) {
    return false;
  }
  struct pollfd pfd {fd_, POLLIN, 0};
  int timeout_ms = static_cast<int>((timeout.count() + 999) / 1000);
  int r = ::poll(&pfd, 1, timeout_ms);
  if (r <= 0 || !(pfd.revents & POLLIN)) {
    return false;
  }
  struct can_frame raw {};
  ssize_t n = ::read(fd_, &raw, sizeof(raw));
  if (n != static_cast<ssize_t>(sizeof(raw))) {
    return false;
  }
  out = CanFrame{};
  out.error = (raw.can_id & CAN_ERR_FLAG) != 0;
  out.extended = (raw.can_id & CAN_EFF_FLAG) != 0;
  if (out.error) {
    out.id = raw.can_id & CAN_ERR_MASK;
  } else {
    out.id = out.extended ? (raw.can_id & CAN_EFF_MASK) : (raw.can_id & CAN_SFF_MASK);
  }
  out.dlc = raw.can_dlc > 8 ? 8 : raw.can_dlc;
  std::memcpy(out.data.data(), raw.data, out.dlc);
  return true;
}

std::string describeErrorFrame(const CanFrame & frame)
{
  std::ostringstream os;
  const uint32_t c = frame.id;
  if (c & CAN_ERR_TX_TIMEOUT) {os << "TX_TIMEOUT ";}
  if (c & CAN_ERR_LOSTARB) {os << "LOST_ARBITRATION ";}
  if (c & CAN_ERR_CRTL) {
    os << "CONTROLLER(";
    const uint8_t d1 = frame.data[1];
    if (d1 & CAN_ERR_CRTL_RX_OVERFLOW) {os << "rx_overflow ";}
    if (d1 & CAN_ERR_CRTL_TX_OVERFLOW) {os << "tx_overflow ";}
    if (d1 & CAN_ERR_CRTL_RX_WARNING) {os << "rx_warning ";}
    if (d1 & CAN_ERR_CRTL_TX_WARNING) {os << "tx_warning ";}
    if (d1 & CAN_ERR_CRTL_RX_PASSIVE) {os << "rx_passive ";}
    if (d1 & CAN_ERR_CRTL_TX_PASSIVE) {os << "tx_passive ";}
    os << ") ";
  }
  if (c & CAN_ERR_PROT) {os << "PROTOCOL ";}
  if (c & CAN_ERR_TRX) {os << "TRANSCEIVER ";}
  if (c & CAN_ERR_ACK) {os << "NO_ACK(다른 노드 응답 없음: 배선/비트레이트 확인) ";}
  if (c & CAN_ERR_BUSOFF) {os << "BUS_OFF ";}
  if (c & CAN_ERR_BUSERROR) {os << "BUS_ERROR ";}
  if (c & CAN_ERR_RESTARTED) {os << "RESTARTED ";}
  std::string s = os.str();
  return s.empty() ? "UNKNOWN" : s;
}

}  // namespace gc_arm
