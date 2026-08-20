#pragma once

#include "../../../DataStructures/TripBased/Data.h"
#include "../../../Helpers/ProbabilityCost.h"

#include <cstdint>
#include <limits>

namespace TripBased {

class TimestampedProbabilityCostData {
public:
  TimestampedProbabilityCostData(const Data &data)
      : data(data), labels(data.numberOfStopEvents(), ProbabilityCostInfinity),
        timestamps(data.numberOfStopEvents(), 0), timestamp(0) {}

  inline void clear() noexcept { timestamp++; }

  inline double operator()(const StopEventId stopEvent) noexcept {
    Assert(stopEvent < labels.size(),
           "StopEvent " << stopEvent << " is out of bounds!");
    return getLabel(stopEvent);
  }

  inline StopEventId getScanEnd(const StopEventId stopEvent,
                                const StopEventId tripEnd,
                                const double probabilityCost) noexcept {
    for (StopEventId event = stopEvent; event < tripEnd; event++) {
      if (getLabel(event) <= probabilityCost)
        return event;
    }
    return tripEnd;
  }

  inline void update(const StopEventId stopEvent, const StopEventId tripEnd,
                     const StopEventId routeEnd, const StopIndex tripLength,
                     const double probabilityCost) noexcept {
    StopEventId currentStart = stopEvent;
    StopEventId currentEnd = tripEnd;
    for (; currentStart < routeEnd;
         currentStart += tripLength, currentEnd += tripLength) {
      const StopEventId clampedEnd =
          (currentEnd <= routeEnd) ? currentEnd : routeEnd;
      for (StopEventId event = currentStart; event < clampedEnd; event++) {
        double &label = getLabel(event);
        if (label <= probabilityCost)
          break;
        label = probabilityCost;
      }
    }
  }

private:
  inline double &getLabel(const StopEventId stopEvent) noexcept {
    if (timestamps[stopEvent] != timestamp) {
      labels[stopEvent] = ProbabilityCostInfinity;
      timestamps[stopEvent] = timestamp;
    }
    return labels[stopEvent];
  }

  const Data &data;

  std::vector<double> labels;
  std::vector<std::uint16_t> timestamps;
  std::uint16_t timestamp;
};

} // namespace TripBased
