// gc_motor_cli — Phase 0 모터/버스 테스트 도구
//
// 단일 CAN 버스(can0)에 연결된 CubeMars AK(Servo 모드)와 MKS SERVO42D/57D를
// 개별 구동/조회하고, 혼합 버스 부하·응답성을 측정한다.
// 사용법: gc_motor_cli --help

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "gc_arm_drivers/ak_servo_protocol.hpp"
#include "gc_arm_drivers/mks_protocol.hpp"
#include "gc_arm_drivers/socketcan.hpp"

using namespace gc_arm;
using Clock = std::chrono::steady_clock;
using std::chrono::duration;
using std::chrono::microseconds;
using std::chrono::milliseconds;

namespace
{

std::atomic<bool> g_stop{false};
void onSignal(int) {g_stop = true;}

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------- 인자 파싱
struct Args
{
  std::vector<std::string> pos;               // 위치 인자
  std::map<std::string, std::string> opt;     // --key value

  bool has(const std::string & k) const {return opt.count(k) > 0;}
  std::string str(const std::string & k, const std::string & def) const
  {
    auto it = opt.find(k);
    return it == opt.end() ? def : it->second;
  }
  double num(const std::string & k, double def) const
  {
    auto it = opt.find(k);
    return it == opt.end() ? def : std::stod(it->second);
  }
  std::vector<int> list(const std::string & k) const
  {
    std::vector<int> out;
    auto it = opt.find(k);
    if (it == opt.end()) {return out;}
    std::stringstream ss(it->second);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      if (!tok.empty()) {out.push_back(std::stoi(tok, nullptr, 0));}
    }
    return out;
  }
};

const std::set<std::string> kFlagOptions = {"yes", "hold", "help"};

Args parseArgs(int argc, char ** argv)
{
  Args a;
  for (int i = 1; i < argc; ++i) {
    std::string s = argv[i];
    if (s == "-h") {s = "--help";}
    if (s == "-i") {s = "--iface";}
    if (s.rfind("--", 0) == 0 && s.size() > 2) {
      std::string key = s.substr(2);
      if (kFlagOptions.count(key)) {
        a.opt[key] = "1";
      } else if (i + 1 < argc) {
        a.opt[key] = argv[++i];
      } else {
        throw std::runtime_error("옵션 값 누락: " + s);
      }
    } else {
      a.pos.push_back(s);
    }
  }
  return a;
}

// ---------------------------------------------------------------- 공통 헬퍼
double secondsSince(Clock::time_point t0)
{
  return duration<double>(Clock::now() - t0).count();
}

bool sendFrame(SocketCan & bus, const CanFrame & f)
{
  auto r = bus.send(f);
  if (r != SocketCan::SendResult::Ok) {
    std::fprintf(stderr, "[송신 실패] %s : %s\n", toString(f).c_str(), bus.lastError().c_str());
    return false;
  }
  return true;
}

/// deadline까지 수신 프레임마다 handler 호출. handler가 true를 반환하면 즉시 종료하고 true 반환
bool receiveUntil(
  SocketCan & bus, Clock::time_point deadline, const std::function<bool(const CanFrame &)> & handler)
{
  CanFrame f;
  while (!g_stop) {
    auto now = Clock::now();
    if (now >= deadline) {return false;}
    auto remain = std::chrono::duration_cast<microseconds>(deadline - now);
    if (bus.receive(f, remain)) {
      if (handler(f)) {return true;}
    }
  }
  return false;
}

std::optional<ak::servo::Feedback> waitAkFeedback(SocketCan & bus, uint8_t id, milliseconds timeout)
{
  std::optional<ak::servo::Feedback> out;
  receiveUntil(bus, Clock::now() + timeout, [&](const CanFrame & f) {
      auto fb = ak::servo::parseFeedback(f);
      if (fb && fb->driver_id == id) {out = fb; return true;}
      return false;
    });
  return out;
}

/// MKS 명령 송신 후, 같은 ID·명령코드의 응답 프레임 대기
std::optional<CanFrame> mksTransact(SocketCan & bus, const CanFrame & req, milliseconds timeout)
{
  if (!sendFrame(bus, req)) {return std::nullopt;}
  std::optional<CanFrame> out;
  receiveUntil(bus, Clock::now() + timeout, [&](const CanFrame & f) {
      if (!f.extended && !f.error && f.id == req.id && mks::checkCrc(f) && f.data[0] == req.data[0]) {
        out = f;
        return true;
      }
      return false;
    });
  return out;
}

std::optional<int64_t> mksReadEncoder(SocketCan & bus, uint16_t id, milliseconds timeout)
{
  auto r = mksTransact(bus, mks::makeReadEncoderAddition(id), timeout);
  return r ? mks::parseEncoderAddition(*r) : std::nullopt;
}

void printAkFeedback(const ak::servo::Feedback & fb)
{
  std::printf(
    "AK  id=%3u  pos=%9.1f deg  spd=%8.0f erpm  cur=%6.2f A  temp=%3d C  err=%u(%s)\n",
    fb.driver_id, fb.position_deg, fb.speed_erpm, fb.current_a, fb.temperature_c, fb.error,
    ak::servo::errorString(fb.error));
}

std::string decodeFrame(const CanFrame & f)
{
  char buf[256];
  if (f.error) {
    return "CAN 에러 프레임: " + describeErrorFrame(f);
  }
  if (auto fb = ak::servo::parseFeedback(f)) {
    std::snprintf(
      buf, sizeof(buf), "AK 피드백 id=%u pos=%.1fdeg spd=%.0ferpm cur=%.2fA T=%dC err=%u(%s)",
      fb->driver_id, fb->position_deg, fb->speed_erpm, fb->current_a, fb->temperature_c, fb->error,
      ak::servo::errorString(fb->error));
    return buf;
  }
  if (f.extended) {
    std::snprintf(
      buf, sizeof(buf), "AK 명령? packet=%u id=%u", (f.id >> 8) & 0xFF, f.id & 0xFF);
    return buf;
  }
  if (auto cmd = mks::responseCmd(f)) {
    if (auto enc = mks::parseEncoderAddition(f)) {
      std::snprintf(
        buf, sizeof(buf), "MKS id=%u 엔코더=%lld (%.2f deg)", f.id,
        static_cast<long long>(*enc), mks::countsToDeg(*enc));
      return buf;
    }
    if (auto spd = mks::parseSpeed(f)) {
      std::snprintf(buf, sizeof(buf), "MKS id=%u 속도=%d rpm", f.id, *spd);
      return buf;
    }
    if (f.dlc == 3) {
      const char * desc = "";
      if (*cmd == 0xF1) {desc = mks::queryStatusString(f.data[1]);}
      if (*cmd == 0xF4 || *cmd == 0xF5 || *cmd == 0xFD || *cmd == 0xFE) {
        desc = mks::positionStatusString(f.data[1]);
      }
      std::snprintf(
        buf, sizeof(buf), "MKS id=%u cmd=0x%02X status=%u %s", f.id, *cmd, f.data[1], desc);
      return buf;
    }
    std::snprintf(buf, sizeof(buf), "MKS id=%u cmd=0x%02X (CRC OK)", f.id, *cmd);
    return buf;
  }
  return "(미해석 — MKS CRC 불일치 또는 기타 장치, AK MIT 응답일 수 있음)";
}

// ---------------------------------------------------------------- monitor
int cmdMonitor(SocketCan & bus)
{
  std::printf("버스 모니터링 중... (Ctrl+C 종료)\n");
  auto t0 = Clock::now();
  CanFrame f;
  while (!g_stop) {
    if (bus.receive(f, milliseconds(100))) {
      std::printf("%10.4f  %-28s %s\n", secondsSince(t0), toString(f).c_str(), decodeFrame(f).c_str());
      std::fflush(stdout);
    }
  }
  return 0;
}

// ---------------------------------------------------------------- AK
int akSweep(SocketCan & bus, uint8_t id, const Args & a)
{
  if (a.pos.size() < 5) {
    std::fprintf(stderr, "사용법: ak <id> sweep <진폭deg> <주기s> [--cycles N] [--rate Hz] [--spd erpm] [--acc erpm/s2] [--csv 파일]\n");
    return 1;
  }
  const double amp = std::stod(a.pos[3]);
  const double period = std::stod(a.pos[4]);
  const double cycles = a.num("cycles", 3);
  const double rate = a.num("rate", 100);
  const double spd = a.num("spd", 10000);
  const double acc = a.num("acc", 30000);

  auto fb0 = waitAkFeedback(bus, id, milliseconds(1000));
  if (!fb0) {
    std::fprintf(stderr, "AK id=%u 피드백 없음 (업로드 주기 설정/배선 확인)\n", id);
    return 1;
  }
  const double p0 = fb0->position_deg;
  std::printf("시작 위치 %.1f deg, 진폭 ±%.1f deg, 주기 %.2f s, %g회, %g Hz\n", p0, amp, period, cycles, rate);

  std::ofstream csv;
  if (a.has("csv")) {
    csv.open(a.str("csv", ""));
    csv << "t,target_deg,meas_deg,speed_erpm,current_a,temp_c\n";
  }

  const auto dt = std::chrono::duration_cast<Clock::duration>(duration<double>(1.0 / rate));
  const auto t0 = Clock::now();
  auto next = t0;
  std::optional<ak::servo::Feedback> last;
  double max_err = 0.0;
  size_t fb_count = 0;
  while (!g_stop) {
    const double t = secondsSince(t0);
    if (t > cycles * period) {break;}
    const double target = p0 + amp * std::sin(2.0 * kPi * t / period);
    sendFrame(bus, ak::servo::makePositionSpeed(id, target, spd, acc));
    next += dt;
    receiveUntil(bus, next, [&](const CanFrame & f) {
        auto fb = ak::servo::parseFeedback(f);
        if (fb && fb->driver_id == id) {last = fb; ++fb_count;}
        return false;
      });
    if (last) {
      max_err = std::max(max_err, std::fabs(target - last->position_deg));
      if (csv) {
        csv << t << ',' << target << ',' << last->position_deg << ',' << last->speed_erpm << ','
            << last->current_a << ',' << static_cast<int>(last->temperature_c) << '\n';
      }
    }
  }
  sendFrame(bus, ak::servo::makePositionSpeed(id, p0, spd, acc));
  const double elapsed = secondsSince(t0);
  std::printf(
    "완료: 피드백 %.1f Hz, 최대 |목표-측정| %.2f deg (피드백 지연 포함)\n", fb_count / elapsed, max_err);
  return 0;
}

/// 목표에 도달할 때까지 위치-속도 명령을 계속 송신 (모터 timeout 방지), 이후 유지/관찰
int akMove(SocketCan & bus, uint8_t id, const Args & a)
{
  if (a.pos.size() < 4) {
    std::fprintf(stderr,
      "사용법: ak <id> move <목표deg> [--spd 3000] [--acc 10000] [--rate 100] [--tol 0.5] [--timeout_s 15]\n"
      "                    [--hold 초] [--observe 초]\n"
      "  --hold    : 도달 후 목표를 계속 송신하며 유지하는 시간\n"
      "  --observe : 이후 송신을 멈추고 위치·전류 관찰 (모터 timeout 동작 확인용)\n");
    return 1;
  }
  const double target = std::stod(a.pos[3]);
  const double spd = a.num("spd", 3000);
  const double acc = a.num("acc", 10000);
  const double rate = a.num("rate", 100);
  const double tol = a.num("tol", 0.5);
  const double timeout_s = a.num("timeout_s", 15);
  const double hold_s = a.num("hold", 0);
  const double observe_s = a.num("observe", 0);

  auto fb0 = waitAkFeedback(bus, id, milliseconds(1000));
  if (!fb0) {
    std::fprintf(stderr, "AK id=%u 피드백 없음\n", id);
    return 1;
  }
  std::printf("시작 %.1f deg → 목표 %.1f deg (%.0f ERPM, %.0f ERPM/s, %g Hz 송신)\n",
    fb0->position_deg, target, spd, acc, rate);

  const auto dt = std::chrono::duration_cast<Clock::duration>(duration<double>(1.0 / rate));
  std::optional<ak::servo::Feedback> last = fb0;
  auto pump = [&](Clock::time_point until, bool send) {
      auto next = Clock::now();
      while (!g_stop && Clock::now() < until) {
        if (send) {sendFrame(bus, ak::servo::makePositionSpeed(id, target, spd, acc));}
        next += dt;
        receiveUntil(bus, next, [&](const CanFrame & f) {
            auto fb = ak::servo::parseFeedback(f);
            if (fb && fb->driver_id == id) {last = fb;}
            return false;
          });
      }
    };

  // 1) 이동
  const auto t0 = Clock::now();
  double next_print = 0.0;
  bool reached = false;
  while (!g_stop && secondsSince(t0) < timeout_s) {
    pump(Clock::now() + dt, true);
    const double t = secondsSince(t0);
    if (t >= next_print) {
      std::printf("  %5.2fs  pos=%9.1f deg  spd=%7.0f erpm  cur=%6.2f A\n",
        t, last->position_deg, last->speed_erpm, last->current_a);
      next_print += 0.2;
    }
    if (std::fabs(last->position_deg - target) < tol && std::fabs(last->speed_erpm) < 50) {
      reached = true;
      break;
    }
  }
  std::printf("%s: pos=%.1f deg (%.2f s)\n", reached ? "도달" : "시간 초과/중단", last->position_deg, secondsSince(t0));

  // 2) 유지
  if (hold_s > 0 && !g_stop) {
    pump(Clock::now() + std::chrono::duration_cast<Clock::duration>(duration<double>(hold_s)), true);
    std::printf("유지 %.1f s 후: pos=%.1f deg cur=%.2f A\n", hold_s, last->position_deg, last->current_a);
  }

  // 3) 송신 중단 후 관찰 (모터 timeout 동작)
  if (observe_s > 0 && !g_stop) {
    std::printf("송신 중단 → %.1f s 관찰 (모터 timeout 이후 위치 유지/토크 해제 여부)\n", observe_s);
    const auto t1 = Clock::now();
    double np = 0.0;
    while (!g_stop && secondsSince(t1) < observe_s) {
      pump(Clock::now() + dt, false);
      const double t = secondsSince(t1);
      if (t >= np) {
        std::printf("  +%5.2fs  pos=%9.1f deg  cur=%6.2f A\n", t, last->position_deg, last->current_a);
        np += 0.25;
      }
    }
  }
  return reached ? 0 : 2;
}

int cmdAk(SocketCan & bus, const Args & a)
{
  if (a.pos.size() < 3) {
    std::fprintf(stderr, "사용법: ak <id> <status|pos|posspd|move|rpm|current|brake|release|origin|sweep> ...\n");
    return 1;
  }
  const uint8_t id = static_cast<uint8_t>(std::stoi(a.pos[1], nullptr, 0));
  const std::string sub = a.pos[2];
  auto arg = [&](size_t i) -> double {
      if (a.pos.size() <= i) {throw std::runtime_error("인자 부족: ak " + sub);}
      return std::stod(a.pos[i]);
    };

  if (sub == "status") {
    const int count = static_cast<int>(a.num("count", 1));
    for (int i = 0; i < count && !g_stop; ++i) {
      auto fb = waitAkFeedback(bus, id, milliseconds(1000));
      if (!fb) {
        std::fprintf(stderr, "AK id=%u 피드백 없음 (1s). 업로드 주기 설정/비트레이트/배선 확인\n", id);
        return 1;
      }
      printAkFeedback(*fb);
    }
    return 0;
  }
  if (sub == "sweep") {
    return akSweep(bus, id, a);
  }
  if (sub == "move") {
    return akMove(bus, id, a);
  }

  CanFrame f;
  if (sub == "pos") {
    f = ak::servo::makePosition(id, arg(3));
  } else if (sub == "posspd") {
    f = ak::servo::makePositionSpeed(id, arg(3), a.num("spd", 10000), a.num("acc", 30000));
  } else if (sub == "rpm") {
    f = ak::servo::makeRpm(id, arg(3));
  } else if (sub == "current") {
    f = ak::servo::makeCurrent(id, arg(3));
  } else if (sub == "brake") {
    f = ak::servo::makeCurrentBrake(id, arg(3));
  } else if (sub == "release") {
    f = ak::servo::makeCurrent(id, 0.0);
  } else if (sub == "origin") {
    const std::string m = a.pos.size() > 3 ? a.pos[3] : "temp";
    ak::servo::OriginMode mode = ak::servo::OriginMode::Temporary;
    if (m == "perm") {
      if (!a.has("yes")) {
        std::fprintf(stderr, "영구 원점은 플래시에 저장됩니다. 계속하려면 --yes 추가\n");
        return 1;
      }
      mode = ak::servo::OriginMode::Permanent;
    } else if (m == "default") {
      mode = ak::servo::OriginMode::RestoreDefault;
    }
    f = ak::servo::makeSetOrigin(id, mode);
  } else {
    std::fprintf(stderr, "알 수 없는 ak 명령: %s\n", sub.c_str());
    return 1;
  }

  std::printf("송신: %s\n", toString(f).c_str());
  if (!sendFrame(bus, f)) {return 1;}
  if (auto fb = waitAkFeedback(bus, id, milliseconds(300))) {printAkFeedback(*fb);}
  return 0;
}

// ---------------------------------------------------------------- MKS
int mksSweep(SocketCan & bus, uint16_t id, const Args & a)
{
  if (a.pos.size() < 5) {
    std::fprintf(stderr,
      "사용법: mks <id> sweep <진폭deg> <주기s> [--mode pos|speed] [--cycles N] [--rate Hz]\n"
      "                       [--rpm 최대rpm] [--acc 0-255] [--kp 1/s] [--csv 파일]\n");
    return 1;
  }
  const double amp = std::stod(a.pos[3]);
  const double period = std::stod(a.pos[4]);
  const std::string mode = a.str("mode", "pos");
  const double cycles = a.num("cycles", 3);
  const double rate = a.num("rate", 100);
  const uint16_t max_rpm = static_cast<uint16_t>(a.num("rpm", 300));
  const uint8_t acc = static_cast<uint8_t>(a.num("acc", 0));
  const double kp = a.num("kp", 5.0);  // speed 모드 외부 위치루프 게인 [1/s]

  auto c0 = mksReadEncoder(bus, id, milliseconds(100));
  if (!c0) {
    std::fprintf(stderr, "MKS id=%u 엔코더 응답 없음\n", id);
    return 1;
  }
  const double p0 = mks::countsToDeg(*c0);
  std::printf("모드=%s 시작 %.2f deg, 진폭 ±%.1f deg, 주기 %.2f s, %g Hz\n",
    mode.c_str(), p0, amp, period, rate);

  std::ofstream csv;
  if (a.has("csv")) {
    csv.open(a.str("csv", ""));
    csv << "t,target_deg,meas_deg,cmd_rpm\n";
  }

  const auto dt = std::chrono::duration_cast<Clock::duration>(duration<double>(1.0 / rate));
  const auto t0 = Clock::now();
  auto next = t0;
  double max_err = 0.0;
  size_t replies = 0, ticks = 0;
  std::optional<double> meas;

  while (!g_stop) {
    const double t = secondsSince(t0);
    if (t > cycles * period) {break;}
    const double w = 2.0 * kPi / period;
    const double target = p0 + amp * std::sin(w * t);
    const double v_ff = amp * w * std::cos(w * t);  // deg/s
    int cmd_rpm = 0;
    ++ticks;
    next += dt;

    if (mode == "speed") {
      // 1) 위치 읽기 → 2) 속도 명령 (v_ff + kp·오차)
      sendFrame(bus, mks::makeReadEncoderAddition(id));
      receiveUntil(bus, next - dt / 2, [&](const CanFrame & f) {
          if (f.id == id) {
            if (auto enc = mks::parseEncoderAddition(f)) {meas = mks::countsToDeg(*enc); ++replies; return true;}
          }
          return false;
        });
      const double err = meas ? target - *meas : 0.0;
      const double v = v_ff + kp * err;           // deg/s
      cmd_rpm = static_cast<int>(std::lround(v / 6.0));  // 1 rpm = 6 deg/s (분해능 한계)
      cmd_rpm = std::clamp(cmd_rpm, -static_cast<int>(max_rpm), static_cast<int>(max_rpm));
      sendFrame(bus, mks::makeSpeed(id, cmd_rpm, acc));
    } else {
      sendFrame(bus, mks::makePosAbsAxis(id, max_rpm, acc,
        static_cast<int32_t>(std::lround(mks::degToCounts(target)))));
      sendFrame(bus, mks::makeReadEncoderAddition(id));
    }

    receiveUntil(bus, next, [&](const CanFrame & f) {
        if (f.id == id) {
          if (auto enc = mks::parseEncoderAddition(f)) {meas = mks::countsToDeg(*enc); ++replies;}
        }
        return false;
      });

    if (meas) {
      max_err = std::max(max_err, std::fabs(target - *meas));
      if (csv) {csv << t << ',' << target << ',' << *meas << ',' << cmd_rpm << '\n';}
    }
  }
  if (mode == "speed") {
    sendFrame(bus, mks::makeSpeed(id, 0, acc));
  } else {
    sendFrame(bus, mks::makePosAbsAxis(id, max_rpm, acc, static_cast<int32_t>(*c0)));
  }
  std::printf("완료: 엔코더 응답 %zu/%zu, 최대 |목표-측정| %.2f deg\n", replies, ticks, max_err);
  return 0;
}

int cmdMks(SocketCan & bus, const Args & a)
{
  if (a.pos.size() < 3) {
    std::fprintf(stderr, "사용법: mks <id> <read|enable|disable|abs|rel|speed|stop|estop|zero|sweep|...> ...\n");
    return 1;
  }
  const uint16_t id = static_cast<uint16_t>(std::stoi(a.pos[1], nullptr, 0));
  const std::string sub = a.pos[2];
  const auto timeout = milliseconds(static_cast<int>(a.num("timeout", 100)));
  auto arg = [&](size_t i) -> double {
      if (a.pos.size() <= i) {throw std::runtime_error("인자 부족: mks " + sub);}
      return std::stod(a.pos[i]);
    };
  auto needYes = [&](const char * what) {
      if (!a.has("yes")) {
        std::fprintf(stderr, "%s 는 드라이버 설정(플래시)을 바꿉니다. 계속하려면 --yes 추가\n", what);
        return false;
      }
      return true;
    };

  if (sub == "read") {
    auto t_start = Clock::now();
    auto enc = mksReadEncoder(bus, id, timeout);
    const double latency_ms = secondsSince(t_start) * 1e3;
    if (!enc) {
      std::fprintf(stderr, "MKS id=%u 응답 없음 (ID/비트레이트/배선 확인)\n", id);
      return 1;
    }
    std::printf("MKS id=%u 엔코더 %lld count = %.2f deg (응답 %.2f ms)\n",
      id, static_cast<long long>(*enc), mks::countsToDeg(*enc), latency_ms);
    if (auto r = mksTransact(bus, mks::makeReadSpeed(id), timeout)) {
      if (auto s = mks::parseSpeed(*r)) {std::printf("  속도 %d rpm\n", *s);}
    }
    if (auto r = mksTransact(bus, mks::makeQueryStatus(id), timeout)) {
      if (auto s = mks::parseStatus(*r, mks::Cmd::QueryStatus)) {
        std::printf("  상태 %u (%s)\n", *s, mks::queryStatusString(*s));
      }
    }
    if (auto r = mksTransact(bus, mks::makeReadProtection(id), timeout)) {
      if (auto s = mks::parseStatus(*r, mks::Cmd::ReadProtection)) {
        std::printf("  구속 보호 %s\n", *s ? "발생(release-stall 필요)" : "없음");
      }
    }
    return 0;
  }
  if (sub == "sweep") {
    return mksSweep(bus, id, a);
  }

  CanFrame f;
  mks::Cmd expect;
  if (sub == "enable" || sub == "disable") {
    f = mks::makeEnable(id, sub == "enable");
    expect = mks::Cmd::Enable;
  } else if (sub == "abs") {
    f = mks::makePosAbsAxis(id, static_cast<uint16_t>(a.num("rpm", 100)),
      static_cast<uint8_t>(a.num("acc", 50)), static_cast<int32_t>(std::lround(mks::degToCounts(arg(3)))));
    expect = mks::Cmd::PosAbsAxis;
  } else if (sub == "rel") {
    f = mks::makePosRelAxis(id, static_cast<uint16_t>(a.num("rpm", 100)),
      static_cast<uint8_t>(a.num("acc", 50)), static_cast<int32_t>(std::lround(mks::degToCounts(arg(3)))));
    expect = mks::Cmd::PosRelAxis;
  } else if (sub == "speed") {
    f = mks::makeSpeed(id, static_cast<int>(arg(3)), static_cast<uint8_t>(a.num("acc", 50)));
    expect = mks::Cmd::Speed;
  } else if (sub == "stop") {
    f = mks::makeSpeed(id, 0, static_cast<uint8_t>(a.num("acc", 50)));
    expect = mks::Cmd::Speed;
  } else if (sub == "estop") {
    f = mks::makeEmergencyStop(id);
    expect = mks::Cmd::EmergencyStop;
  } else if (sub == "zero") {
    f = mks::makeSetZero(id);
    expect = mks::Cmd::SetZero;
  } else if (sub == "release-stall") {
    f = mks::makeReleaseStall(id);
    expect = mks::Cmd::ReleaseStall;
  } else if (sub == "set-bitrate") {
    if (!needYes("set-bitrate")) {return 1;}
    const int kbps = static_cast<int>(arg(3));
    mks::Bitrate r;
    switch (kbps) {
      case 125: r = mks::Bitrate::k125; break;
      case 250: r = mks::Bitrate::k250; break;
      case 500: r = mks::Bitrate::k500; break;
      case 1000: r = mks::Bitrate::k1000; break;
      default: std::fprintf(stderr, "비트레이트: 125|250|500|1000\n"); return 1;
    }
    f = mks::makeSetCanBitrate(id, r);
    expect = mks::Cmd::SetCanBitrate;
  } else if (sub == "set-id") {
    if (!needYes("set-id")) {return 1;}
    f = mks::makeSetCanId(id, static_cast<uint16_t>(arg(3)));
    expect = mks::Cmd::SetCanId;
  } else if (sub == "set-response") {
    if (!needYes("set-response")) {return 1;}
    f = mks::makeSetResponse(id, arg(3) != 0, arg(4) != 0);
    expect = mks::Cmd::SetResponse;
  } else if (sub == "set-mode") {
    if (!needYes("set-mode")) {return 1;}
    f = mks::makeSetMode(id, static_cast<mks::WorkMode>(static_cast<int>(arg(3))));
    expect = mks::Cmd::SetMode;
  } else if (sub == "set-current") {
    if (!needYes("set-current")) {return 1;}
    f = mks::makeSetCurrent(id, static_cast<uint16_t>(arg(3)));
    expect = mks::Cmd::SetCurrent;
  } else {
    std::fprintf(stderr, "알 수 없는 mks 명령: %s\n", sub.c_str());
    return 1;
  }

  std::printf("송신: %s\n", toString(f).c_str());
  auto r = mksTransact(bus, f, timeout);
  if (!r) {
    std::fprintf(stderr, "응답 없음 (설정 명령은 응답 후 새 설정이 적용될 수 있음)\n");
    return 1;
  }
  if (auto s = mks::parseStatus(*r, expect)) {
    std::printf("응답 status=%u", *s);
    if (expect == mks::Cmd::PosAbsAxis || expect == mks::Cmd::PosRelAxis) {
      std::printf(" (%s)", mks::positionStatusString(*s));
    } else {
      std::printf(" (%s)", *s ? "성공" : "실패");
    }
    std::printf("\n");
  } else {
    std::printf("응답: %s  %s\n", toString(*r).c_str(), decodeFrame(*r).c_str());
  }
  return 0;
}

// ---------------------------------------------------------------- bustest
struct Stat
{
  size_t rx{0};
  size_t timeouts{0};
  double lat_sum_ms{0.0};
  double lat_max_ms{0.0};
  void reset() {*this = Stat{};}
};

int cmdBusTest(SocketCan & bus, const Args & a)
{
  const auto ak_ids = a.list("ak");
  const auto mks_ids = a.list("mks");
  const double rate = a.num("rate", 100);
  const double duration_s = a.num("duration", 10);
  const bool hold = a.has("hold");
  const double spd = a.num("spd", 10000);
  const double acc = a.num("acc", 30000);

  if (ak_ids.empty() && mks_ids.empty()) {
    std::fprintf(stderr, "사용법: bustest --ak 101,102,103 --mks 1,2,3 [--rate 100] [--duration 10] [--hold]\n");
    return 1;
  }

  // --hold: 현재 위치를 목표로 매 주기 명령 송신 (실제 제어 트래픽 재현)
  std::map<int, double> ak_hold;
  std::map<int, int64_t> mks_hold;
  if (hold) {
    for (int id : ak_ids) {
      auto fb = waitAkFeedback(bus, static_cast<uint8_t>(id), milliseconds(1000));
      if (!fb) {std::fprintf(stderr, "AK id=%d 피드백 없음 → 중단\n", id); return 1;}
      ak_hold[id] = fb->position_deg;
    }
    for (int id : mks_ids) {
      auto c = mksReadEncoder(bus, static_cast<uint16_t>(id), milliseconds(100));
      if (!c) {std::fprintf(stderr, "MKS id=%d 응답 없음 → 중단\n", id); return 1;}
      mks_hold[id] = *c;
    }
  }

  std::map<int, Stat> ak_win, ak_tot, mks_win, mks_tot;
  std::map<int, Clock::time_point> mks_sent;
  std::map<int, bool> mks_pending;
  size_t err_frames = 0, send_fail = 0, other_rx = 0;

  const auto dt = std::chrono::duration_cast<Clock::duration>(duration<double>(1.0 / rate));
  const auto t0 = Clock::now();
  auto next = t0;
  auto win_start = t0;
  size_t cycles = 0;

  std::printf("버스 테스트: %g Hz, %g s, hold=%s (Ctrl+C 중단)\n", rate, duration_s, hold ? "on" : "off");
  std::printf("주의: AK는 업로드 주기로 송신하는 피드백을 수동 계수, MKS는 매 주기 0x31 폴링\n");

  auto handle = [&](const CanFrame & f) {
      if (f.error) {++err_frames; return false;}
      if (auto fb = ak::servo::parseFeedback(f)) {
        if (ak_win.count(fb->driver_id) || std::count(ak_ids.begin(), ak_ids.end(), fb->driver_id)) {
          ++ak_win[fb->driver_id].rx;
          ++ak_tot[fb->driver_id].rx;
          if (fb->error) {
            std::fprintf(stderr, "  AK id=%u 에러 %u(%s)\n", fb->driver_id, fb->error, ak::servo::errorString(fb->error));
          }
          return false;
        }
      }
      if (!f.extended && mks::parseEncoderAddition(f)) {
        const int id = static_cast<int>(f.id);
        if (mks_pending[id]) {
          const double ms = duration<double, std::milli>(Clock::now() - mks_sent[id]).count();
          for (auto * s : {&mks_win[id], &mks_tot[id]}) {
            ++s->rx;
            s->lat_sum_ms += ms;
            s->lat_max_ms = std::max(s->lat_max_ms, ms);
          }
          mks_pending[id] = false;
        }
        return false;
      }
      ++other_rx;
      return false;
    };

  while (!g_stop && secondsSince(t0) < duration_s) {
    ++cycles;
    for (int id : ak_ids) {
      if (hold && !sendFrame(bus, ak::servo::makePositionSpeed(static_cast<uint8_t>(id), ak_hold[id], spd, acc))) {
        ++send_fail;
      }
    }
    for (int id : mks_ids) {
      if (hold && !sendFrame(bus, mks::makePosAbsAxis(static_cast<uint16_t>(id), 100, 0,
        static_cast<int32_t>(mks_hold[id]))))
      {
        ++send_fail;
      }
      if (sendFrame(bus, mks::makeReadEncoderAddition(static_cast<uint16_t>(id)))) {
        mks_sent[id] = Clock::now();
        mks_pending[id] = true;
      } else {
        ++send_fail;
      }
    }
    next += dt;
    receiveUntil(bus, next, handle);
    for (int id : mks_ids) {
      if (mks_pending[id]) {
        ++mks_win[id].timeouts;
        ++mks_tot[id].timeouts;
        mks_pending[id] = false;
      }
    }

    const double win = secondsSince(win_start);
    if (win >= 1.0) {
      std::printf("[%5.1fs]", secondsSince(t0));
      for (int id : ak_ids) {std::printf("  AK%d %.0fHz", id, ak_win[id].rx / win);}
      for (int id : mks_ids) {
        auto & s = mks_win[id];
        std::printf("  MKS%d %.0fHz lat %.2f/%.2fms to=%zu", id, s.rx / win,
          s.rx ? s.lat_sum_ms / s.rx : 0.0, s.lat_max_ms, s.timeouts);
      }
      std::printf("  err=%zu sendfail=%zu\n", err_frames, send_fail);
      std::fflush(stdout);
      for (auto & kv : ak_win) {kv.second.reset();}
      for (auto & kv : mks_win) {kv.second.reset();}
      win_start = Clock::now();
    }
  }

  const double total = secondsSince(t0);
  std::printf("\n===== 결과 (%.1f s, %zu 주기, 실제 %.1f Hz) =====\n", total, cycles, cycles / total);
  for (int id : ak_ids) {
    std::printf("AK  id=%3d  피드백 %.1f Hz\n", id, ak_tot[id].rx / total);
  }
  for (int id : mks_ids) {
    auto & s = mks_tot[id];
    std::printf("MKS id=%3d  응답 %zu/%zu (%.1f%%)  지연 평균 %.2f ms  최대 %.2f ms  타임아웃 %zu\n",
      id, s.rx, cycles, 100.0 * s.rx / std::max<size_t>(cycles, 1),
      s.rx ? s.lat_sum_ms / s.rx : 0.0, s.lat_max_ms, s.timeouts);
  }
  std::printf("에러 프레임 %zu, 송신 실패 %zu, 기타 수신 %zu\n", err_frames, send_fail, other_rx);
  return 0;
}

void printUsage()
{
  std::printf(R"(gc_motor_cli — GC 6축 로봇팔 Phase 0 모터/버스 테스트 도구

사용법: gc_motor_cli [-i can0] <명령> ...

  monitor                                   모든 프레임 수신·해석 출력

  ak <id> status [--count N]                Servo 모드 피드백 출력
  ak <id> pos <deg>                         위치 루프 (mode 4)
  ak <id> posspd <deg> [--spd erpm] [--acc erpm/s2]   위치-속도 루프 (mode 6), 1회 송신
                                            ⚠️ 모터 timeout(1 s) 안에 도달 못 하면 멈춤 → 큰 이동은 move 사용
  ak <id> move <deg> [--spd 3000] [--acc 10000] [--hold s] [--observe s]
                                            도달할 때까지 100 Hz로 계속 송신, 이후 유지/송신 중단 관찰
  ak <id> rpm <erpm>                        속도 루프
  ak <id> current <A> | brake <A>           전류 / 브레이크 전류
  ak <id> release                           전류 0 (토크 해제 — 중력 부하 주의!)
  ak <id> origin [temp|default|perm --yes]  원점 설정
  ak <id> sweep <amp_deg> <period_s> [--cycles 3] [--rate 100] [--spd] [--acc] [--csv f]

  mks <id> read                             엔코더/속도/상태/보호 조회 + 응답시간
  mks <id> enable | disable
  mks <id> abs <deg> [--rpm 100] [--acc 50] 절대 위치 (모터축 deg, 0xF5)
  mks <id> rel <deg> [--rpm 100] [--acc 50] 상대 위치 (0xF4)
  mks <id> speed <rpm> [--acc 50] | stop [--acc 50] | estop
  mks <id> zero | release-stall
  mks <id> sweep <amp_deg> <period_s> [--mode pos|speed] [--rpm 300] [--acc 0] [--kp 5] [--rate 100] [--csv f]
  mks <id> set-bitrate <125|250|500|1000> --yes
  mks <id> set-id <new_id> --yes
  mks <id> set-response <respond 0|1> <active 0|1> --yes
  mks <id> set-mode <0-5> --yes             (5 = SR_vFOC)
  mks <id> set-current <mA> --yes

  bustest --ak 101,102,103 --mks 1,2,3 [--rate 100] [--duration 10] [--hold]
                                            혼합 버스 응답률/지연 측정
                                            --hold: 현재 위치 유지 명령을 매 주기 송신

공통 옵션: -i/--iface <can0>   --timeout <ms>
)");
}

}  // namespace

int main(int argc, char ** argv)
{
  Args a;
  try {
    a = parseArgs(argc, argv);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
  if (a.has("help") || a.pos.empty()) {
    printUsage();
    return a.pos.empty() && !a.has("help") ? 1 : 0;
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  SocketCan bus;
  std::string err;
  const std::string iface = a.str("iface", "can0");
  if (!bus.open(iface, true, &err)) {
    std::fprintf(stderr, "%s 열기 실패: %s\n", iface.c_str(), err.c_str());
    std::fprintf(stderr, "  sudo ip link set %s up type can bitrate 1000000\n", iface.c_str());
    return 1;
  }

  try {
    const std::string cmd = a.pos[0];
    if (cmd == "monitor") {return cmdMonitor(bus);}
    if (cmd == "ak") {return cmdAk(bus, a);}
    if (cmd == "mks") {return cmdMks(bus, a);}
    if (cmd == "bustest") {return cmdBusTest(bus, a);}
    std::fprintf(stderr, "알 수 없는 명령: %s (--help)\n", cmd.c_str());
    return 1;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "오류: %s\n", e.what());
    return 1;
  }
}
