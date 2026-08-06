
#pragma once

#include <cmath>

namespace TripBased {

struct ProbabilityLabel {

  double cost;

  static constexpr ProbabilityLabel infinity() noexcept { return {INFTY}; }

  static ProbabilityLabel fromProbability(const double p) noexcept {
    Assert(p > 0.0 && p <= 1.0, "Probability must be in (0,1]!");
    return {-std::log(p)};
  }

  double probability() const noexcept { return std::exp(-cost); }

  ProbabilityLabel &operator+=(const ProbabilityLabel &other) noexcept {
    cost += other.cost;
    return *this;
  }

  friend ProbabilityLabel operator+(ProbabilityLabel a,
                                    const ProbabilityLabel &b) noexcept {
    a += b;
    return a;
  }

  friend bool operator<=(const ProbabilityLabel &a,
                         const ProbabilityLabel &b) noexcept {
    return a.cost <= b.cost;
  }

  friend bool operator<(const ProbabilityLabel &a,
                        const ProbabilityLabel &b) noexcept {
    return a.cost < b.cost;
  }
};

} // namespace TripBased
