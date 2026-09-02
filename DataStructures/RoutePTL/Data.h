#pragma once

#include <array>
#include <vector>

#include "../HubLabels/Hubs.h"
#include "../TripBased/Data.h"

namespace RoutePTL {

class Data {
  using HLLabel = GroupLabel<RouteId, std::uint32_t, std::uint16_t>;

public:
  Data(const TripBased::Data &tbData)
      : data(tbData), labels{} {
          // TODO fill both labels[0] and [1] with empty vector
        };

private:
  TripBased::Data data;

  std::array<std::vector<HLLabel>, 2> labels;
};

} // namespace RoutePTL
