#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "../../Helpers/Assert.h"
#include "../HubLabels/Hubs.h"
#include "../TripBased/Data.h"

namespace StopPTL {

using ChainLabel = HL::ChainLabel<>;

class Data {
public:
  Data(const TripBased::Data &tbData)
      : data(tbData),
        labels{std::vector<ChainLabel>(tbData.numberOfStopEvents()),
               std::vector<ChainLabel>(tbData.numberOfStopEvents())} {}

public:
  inline const TripBased::Data &tripBasedData() const noexcept { return data; }

  inline ChainLabel &forwardLabel(const StopEventId event) noexcept {
    return labels[FORWARD][event];
  }

  inline const ChainLabel &
  forwardLabel(const StopEventId event) const noexcept {
    return labels[FORWARD][event];
  }

  inline ChainLabel &backwardLabel(const StopEventId event) noexcept {
    return labels[BACKWARD][event];
  }

  inline const ChainLabel &
  backwardLabel(const StopEventId event) const noexcept {
    return labels[BACKWARD][event];
  }

private:
  TripBased::Data data;

  std::array<std::vector<ChainLabel>, 2> labels;
};

} // namespace StopPTL
