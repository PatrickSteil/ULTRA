#pragma once

#include <cmath>
#include <limits>

inline constexpr double ProbabilityCostInfinity =
    std::numeric_limits<double>::infinity();

inline constexpr double ProbabilityCostEpsilon = 1e-9;
inline constexpr double ProbabilityCostGranularity = 1e-6;

constexpr bool costLessEqual(const double a, const double b) noexcept {
  return a <= b + ProbabilityCostEpsilon;
}

constexpr bool costLess(const double a, const double b) noexcept {
  return a < b - ProbabilityCostEpsilon;
}

constexpr bool costGreaterEqual(const double a, const double b) noexcept {
  return costLessEqual(b, a);
}

constexpr bool costGreater(const double a, const double b) noexcept {
  return costLess(b, a);
}

constexpr bool costEqual(const double a, const double b) noexcept {
  return !costLess(a, b) && !costLess(b, a);
}

inline double quantizeCost(const double cost) noexcept {
  if (cost == ProbabilityCostInfinity)
    return cost;
  return std::round(cost / ProbabilityCostGranularity) *
         ProbabilityCostGranularity;
}

inline double probabilityToCost(const double probability) noexcept {
  return quantizeCost(-std::log(probability));
}

inline double costToProbability(const double cost) noexcept {
  return std::exp(-cost);
}
