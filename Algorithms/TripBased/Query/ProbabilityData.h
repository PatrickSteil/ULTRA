#pragma once

#include "../../../Helpers/ProbabilityCost.h"

#include "ThirdCriteriaData.h"

namespace TripBased {

struct ProbabilityCostLabel {
  double cost;

  static constexpr ProbabilityCostLabel infinity() noexcept {
    return {ProbabilityCostInfinity};
  }

  friend constexpr bool operator<=(const ProbabilityCostLabel &a,
                                   const ProbabilityCostLabel &b) noexcept {
    return costLessEqual(a.cost, b.cost);
  }
};

using ProbabilityData = TripCriterionData<ProbabilityCostLabel>;

} // namespace TripBased
