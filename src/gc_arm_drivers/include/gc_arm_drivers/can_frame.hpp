#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace gc_arm
{

/// SocketCAN과 독립적인 CAN 프레임 표현 (프로토콜 계층은 이 타입만 사용 → 단위 테스트 가능)
struct CanFrame
{
  uint32_t id{0};          ///< 11bit(표준) 또는 29bit(확장) ID, 플래그 비트 제외
  bool extended{false};    ///< true: 29bit 확장 프레임
  bool error{false};       ///< true: SocketCAN 에러 프레임 (id = 에러 클래스)
  uint8_t dlc{0};          ///< 데이터 길이 (0~8)
  std::array<uint8_t, 8> data{};
};

/// "ID#DATA" 형태의 사람이 읽을 수 있는 문자열 (candump 유사)
std::string toString(const CanFrame & frame);

}  // namespace gc_arm
