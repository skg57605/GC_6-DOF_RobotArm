#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

/// 궤적 추종 시험(sweep) 결과 분석: 목표 대비 측정값의 시간 지연과 진폭 비율
namespace gc_arm::analysis
{

struct TrackingResult
{
  double delay_s{0.0};          ///< 측정값이 목표를 가장 잘 따라가는 시간 지연
  double rms_error{0.0};        ///< 지연 0 기준 RMS 오차
  double rms_after_delay{0.0};  ///< 지연 보정 후 RMS 오차 (남는 오차 = 진폭/형상 차이)
  double amplitude_ratio{0.0};  ///< 측정 진폭 / 목표 진폭 (peak-to-peak)
  bool valid{false};
};

/// 시간 t에서 (t, y) 시계열 선형 보간. 범위 밖이면 끝값
inline double interpolate(const std::vector<double> & t, const std::vector<double> & y, double x)
{
  if (x <= t.front()) {return y.front();}
  if (x >= t.back()) {return y.back();}
  auto it = std::upper_bound(t.begin(), t.end(), x);
  const size_t i = static_cast<size_t>(it - t.begin());
  const double a = (x - t[i - 1]) / (t[i] - t[i - 1]);
  return y[i - 1] + a * (y[i] - y[i - 1]);
}

/// @param max_delay_s 탐색할 최대 지연, @param step_s 탐색 간격
inline TrackingResult analyzeTracking(
  const std::vector<double> & t, const std::vector<double> & target, const std::vector<double> & meas,
  double max_delay_s = 0.5, double step_s = 0.002)
{
  TrackingResult r;
  const size_t n = t.size();
  if (n < 10 || target.size() != n || meas.size() != n) {
    return r;
  }
  auto rms_for = [&](double d) {
      double sum = 0.0;
      size_t cnt = 0;
      for (size_t i = 0; i < n; ++i) {
        if (t[i] - d < t.front()) {continue;}
        const double e = meas[i] - interpolate(t, target, t[i] - d);
        sum += e * e;
        ++cnt;
      }
      return cnt ? std::sqrt(sum / cnt) : 1e300;
    };
  r.rms_error = rms_for(0.0);
  double best = r.rms_error;
  for (double d = step_s; d <= max_delay_s + 1e-12; d += step_s) {
    const double e = rms_for(d);
    if (e < best) {
      best = e;
      r.delay_s = d;
    }
  }
  r.rms_after_delay = best;
  const auto [tmin, tmax] = std::minmax_element(target.begin(), target.end());
  const auto [mmin, mmax] = std::minmax_element(meas.begin(), meas.end());
  const double tpp = *tmax - *tmin;
  r.amplitude_ratio = tpp > 1e-9 ? (*mmax - *mmin) / tpp : 0.0;
  r.valid = true;
  return r;
}

}  // namespace gc_arm::analysis
