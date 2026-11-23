#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace gc_arm::bytes
{

// 모든 함수는 빅엔디언(MSB first) — CubeMars, MKS 모두 빅엔디언 사용

inline void putBe16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v >> 8);
  p[1] = static_cast<uint8_t>(v);
}

inline void putBe24(uint8_t * p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v >> 16);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v);
}

inline void putBe32(uint8_t * p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v >> 24);
  p[1] = static_cast<uint8_t>(v >> 16);
  p[2] = static_cast<uint8_t>(v >> 8);
  p[3] = static_cast<uint8_t>(v);
}

inline uint16_t getBe16(const uint8_t * p)
{
  return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

inline uint32_t getBe32(const uint8_t * p)
{
  return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}

inline int32_t getBeInt24(const uint8_t * p)
{
  uint32_t v = (static_cast<uint32_t>(p[0]) << 16) | (static_cast<uint32_t>(p[1]) << 8) | p[2];
  if (v & 0x800000u) {
    v |= 0xFF000000u;
  }
  return static_cast<int32_t>(v);
}

inline int64_t getBeInt48(const uint8_t * p)
{
  uint64_t v = 0;
  for (int i = 0; i < 6; ++i) {
    v = (v << 8) | p[i];
  }
  if (v & 0x800000000000ull) {
    v |= 0xFFFF000000000000ull;
  }
  return static_cast<int64_t>(v);
}

/// double → 정수, 반올림 + 타입 범위로 포화(saturate)
template<typename IntT>
inline IntT saturateRound(double v)
{
  constexpr double lo = static_cast<double>(std::numeric_limits<IntT>::min());
  constexpr double hi = static_cast<double>(std::numeric_limits<IntT>::max());
  if (std::isnan(v)) {
    return 0;
  }
  if (v <= lo) {
    return std::numeric_limits<IntT>::min();
  }
  if (v >= hi) {
    return std::numeric_limits<IntT>::max();
  }
  return static_cast<IntT>(std::llround(v));
}

}  // namespace gc_arm::bytes
