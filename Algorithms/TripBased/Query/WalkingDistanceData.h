#pragma once

#include "ThirdCriteriaData.h"

namespace TripBased {

struct WalkingDistanceLabel {

  int distance;

  static constexpr WalkingDistanceLabel infinity() noexcept { return {INFTY}; }

  friend constexpr bool operator<=(const WalkingDistanceLabel &a,
                                   const WalkingDistanceLabel &b) noexcept {
    return a.distance <= b.distance;
  }
};

} // namespace TripBased
