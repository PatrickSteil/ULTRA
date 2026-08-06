#pragma once

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

template <typename Label> class TripCriterionData {

public:
  using LabelType = Label;

  TripCriterionData(const Data &data, const Label &infinity = Label::infinity())
      : data(data), infinity(infinity),
        labels(data.numberOfStopEvents(), infinity) {}

  inline void clear() noexcept {
    std::fill(labels.begin(), labels.end(), infinity);
  }

  inline const Label &operator()(const StopEventId stopEvent) const noexcept {
    Assert(stopEvent < labels.size(),
           "StopEvent " << stopEvent << " is out of bounds!");
    return labels[stopEvent];
  }

  inline StopEventId getScanEnd(const StopEventId stopEvent,
                                const StopEventId tripEnd,
                                const Label &label) const noexcept {
    for (StopEventId event = stopEvent; event < tripEnd; event++) {
      if (labels[event] <= label)
        return event;
    }
    return tripEnd;
  }

  inline void update(const StopEventId stopEvent, const StopEventId tripEnd,
                     const StopEventId routeEnd, const StopIndex tripLength,
                     const Label &label) noexcept {
    StopEventId currentStart = stopEvent;
    StopEventId currentEnd = tripEnd;

    for (; currentStart < routeEnd;
         currentStart += tripLength, currentEnd += tripLength) {
      for (StopEventId event = currentStart; event < currentEnd; event++) {
        if (labels[event] <= label)
          break;
        labels[event] = label;
      }
    }
  }

private:
  const Data &data;
  Label infinity;
  std::vector<Label> labels;
};

} // namespace TripBased
