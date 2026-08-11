#pragma once

#include <limits>
#include <vector>

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

/*
 * ProbabilityCostData
 * =====================
 *
 * Tracks, for every stop event, the best (lowest) additive probability
 * cost `-log(probability)` with which it has been reached so far during
 * the current query. This plays exactly the role `WalkingDistanceData`
 * plays in the plain McQuery: it lets a trip scan stop early once it
 * enters a region that some other (already processed) label has already
 * covered with an equal-or-better cost, since everything past that point
 * is then guaranteed to be dominated.
 *
 * NOTE: the real `WalkingDistanceData` additionally shares array slots
 * between different trips of the same route via an `offsets` trick, to
 * save memory and to prune across trips, not just within one. That
 * optimization wasn't available to reproduce here, so this version keeps
 * one slot per stop event and only prunes within a single trip's scan.
 * It is correct, just not as tight -- if you have the real
 * WalkingDistanceData-style class available, it can be swapped in here
 * since the interface (operator(), update(), getScanEnd(), clear())
 * matches.
 */
class ProbabilityCostData {

public:
  static constexpr double Infinity = std::numeric_limits<double>::infinity();

  ProbabilityCostData(const Data &data)
      : cost(data.numberOfStopEvents(), Infinity) {}

  inline void clear() noexcept {
    std::fill(cost.begin(), cost.end(), Infinity);
  }

  inline double operator()(const StopEventId stopEvent) const noexcept {
    return cost[stopEvent];
  }

  // Returns the first stop event in [begin, end) whose recorded cost is
  // already <= newCost, i.e. the point from which on a scan carrying
  // `newCost` no longer needs to continue. Returns `end` if no such
  // point exists in the range.
  inline StopEventId getScanEnd(const StopEventId begin, const StopEventId end,
                                const double newCost) const noexcept {
    for (StopEventId i = begin; i < end; i++) {
      if (cost[i] <= newCost)
        return i;
    }
    return end;
  }

  // Registers `newCost` as the best known cost for every stop event in
  // [begin, tripEnd), stopping as soon as an equal-or-better cost is
  // already recorded (everything after that point is then already
  // covered).
  inline void update(const StopEventId begin, const StopEventId tripEnd,
                     const StopEventId /*routeEnd*/,
                     const StopIndex /*tripLength*/,
                     const double newCost) noexcept {
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
