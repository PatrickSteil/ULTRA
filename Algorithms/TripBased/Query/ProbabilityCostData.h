#pragma once
#include "../../../DataStructures/TripBased/Data.h"
#include <cassert>
#include <limits>
#include <vector>
namespace TripBased {
class ProbabilityCostData {
public:
  static constexpr double Infinity = std::numeric_limits<double>::infinity();
  static constexpr double PROB_COST_EPS = 1e-9;
  static bool costLessEqual(const double a, const double b) noexcept {
    return a <= b + PROB_COST_EPS;
  }
  static bool costLess(const double a, const double b) noexcept {
    return a < b - PROB_COST_EPS;
  }
  static bool costEqual(const double a, const double b) noexcept {
    return !costLess(a, b) && !costLess(b, a);
  }

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
      if (costLessEqual(cost[i], newCost))
        return i;
    }
    return end;
  }
  inline void update(const StopEventId stopEvent, const StopEventId tripEnd,
                     const StopEventId routeEnd, const StopIndex tripLength,
                     const double newCost) noexcept {
    assert(tripEnd <= cost.size());
    assert(routeEnd <= cost.size());
    StopEventId currentStart = stopEvent;
    StopEventId currentEnd = tripEnd;
    for (; currentStart < routeEnd;
         currentStart += tripLength, currentEnd += tripLength) {
      const StopEventId clampedEnd =
          (currentEnd <= routeEnd) ? currentEnd : routeEnd;
      for (StopEventId event = currentStart; event < clampedEnd; event++) {
        assert(event < cost.size());
        if (costLessEqual(cost[event], newCost))
          break;
        cost[event] = newCost;
      }
    }
  }

private:
  std::vector<double> cost;
};
} // namespace TripBased
