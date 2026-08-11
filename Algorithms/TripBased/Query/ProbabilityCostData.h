#pragma once

#include <cmath>
#include <limits>

#include "ThirdCriteriaData.h"

namespace TripBased {

struct ProbabilityCostLabel {

  double cost;

  static constexpr ProbabilityCostLabel infinity() noexcept {
    return {std::numeric_limits<double>::infinity()};
  }

  friend constexpr bool operator<=(const ProbabilityCostLabel &a,
                                   const ProbabilityCostLabel &b) noexcept {
    return a.cost <= b.cost;
  }
};

using ProbabilityData = TripCriterionData<ProbabilityCostLabel>;

inline double probabilityToCost(const double probability) noexcept {
  return -std::log(probability);
}

inline double costToProbability(const double cost) noexcept {
  return std::exp(-cost);
}

} // namespace TripBased
