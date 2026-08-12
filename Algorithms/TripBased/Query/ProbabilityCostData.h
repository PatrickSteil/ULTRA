#pragma once
#include "../../../DataStructures/TripBased/Data.h"
#include <cassert>
#include <limits>
#include <vector>
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
    assert(begin >= end || begin < cost.size());
    assert(begin >= end || end <= cost.size());
    for (StopEventId i = begin; i < end; i++) {
      if (cost[i] <= newCost)
        return i;
    }
    return end;
  }
  inline void update(const StopEventId stopEvent, const StopEventId tripEnd,
                     const StopEventId routeEnd, const StopIndex tripLength,
                     const double newCost) noexcept {
    assert(tripEnd <= cost.size() &&
           "tripEnd exceeds numberOfStopEvents() -- check "
           "data.firstStopEventOfTrip's sentinel entry");
    assert(routeEnd <= cost.size() &&
           "routeEnd exceeds numberOfStopEvents() -- check "
           "data.firstTripOfRoute's sentinel entry for the last route");
    StopEventId currentStart = stopEvent;
    StopEventId currentEnd = tripEnd;
    for (; currentStart < routeEnd;
         currentStart += tripLength, currentEnd += tripLength) {
      const StopEventId clampedEnd =
          (currentEnd <= routeEnd) ? currentEnd : routeEnd;
      for (StopEventId event = currentStart; event < clampedEnd; event++) {
        assert(event < cost.size());
        if (cost[event] <= newCost)
          break;
        cost[event] = newCost;
      }
    }
  }

private:
  std::vector<double> cost;
};
} // namespace TripBased
