#pragma once

#include <cassert>
#include <limits>
#include <vector>

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class ProbabilityCostData {

public:
  static constexpr double Infinity = std::numeric_limits<double>::infinity();

  ProbabilityCostData(const Data &data)
      : cost(data.numberOfStopEvents(), Infinity) {}

  inline void clear() noexcept {
    std::fill(cost.begin(), cost.end(), Infinity);
  }

  inline double operator()(const StopEventId stopEvent) const noexcept {
    assert(stopEvent < cost.size());
    return cost[stopEvent];
  }

  inline StopEventId getScanEnd(const StopEventId begin, const StopEventId end,
                                const double newCost) const noexcept {
    assert(begin < cost.size());
    assert(end < cost.size());
    assert(begin <= end);
    for (StopEventId i = begin; i < end; i++) {
      if (cost[i] <= newCost)
        return i;
    }
    return end;
  }

  inline void update(const StopEventId begin, const StopEventId tripEnd,
                     const StopEventId /*routeEnd*/,
                     const StopIndex /*tripLength*/,
                     const double newCost) noexcept {
    assert(begin < cost.size());
    assert(tripEnd < cost.size());
    assert(begin <= tripEnd);
    for (StopEventId i = begin; i < tripEnd; i++) {
      if (cost[i] <= newCost)
        break;
      cost[i] = newCost;
    }
  }

private:
  std::vector<double> cost;
};

} // namespace TripBased
