#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "../../Helpers/Assert.h"
#include "../HubLabels/Hubs.h"
#include "../TripBased/Data.h"

namespace RoutePTL {

template <typename ChainType, typename IndexType> struct StaircaseEntry {
  int time;
  ChainType chain;
  IndexType index;
  StopEventId event;

  StaircaseEntry(const int time = never, const ChainType chain = ChainType(),
                 const IndexType index = IndexType(),
                 const StopEventId event = noStopEvent)
      : time(time), chain(chain), index(index), event(event) {}

  friend std::ostream &operator<<(std::ostream &out,
                                  const StaircaseEntry &e) noexcept {
    return out << "(t=" << e.time << ", chain=" << e.chain
               << ", index=" << e.index << ")";
  }
};

template <typename ChainType, typename IndexType, bool FORWARD_DIRECTION>
class Staircase {
public:
  using Entry = StaircaseEntry<ChainType, IndexType>;

  inline bool insert(const int time, const ChainType chain,
                     const IndexType index,
                     const StopEventId event = noStopEvent) noexcept {
    for (const Entry &e : entries) {
      if (dominates(e, time, chain, index))
        return false;
    }
    entries.erase(std::remove_if(entries.begin(), entries.end(),
                                 [&](const Entry &e) {
                                   return dominates(time, chain, index, e.time,
                                                    e.chain, e.index);
                                 }),
                  entries.end());
    entries.emplace_back(time, chain, index, event);
    return true;
  }

  inline void sortByChain() noexcept {
    std::sort(entries.begin(), entries.end(),
              [](const Entry &a, const Entry &b) { return a.chain < b.chain; });
  }

  inline std::size_t size() const noexcept { return entries.size(); }

  inline bool empty() const noexcept { return entries.empty(); }

  std::vector<Entry> entries;

private:
  static inline bool dominates(const int t1, const ChainType c1,
                               const IndexType i1, const int t2,
                               const ChainType c2,
                               const IndexType i2) noexcept {
    if constexpr (FORWARD_DIRECTION) {
      return t1 >= t2 && c1 <= c2 && i1 <= i2;
    } else {
      return t1 <= t2 && c1 >= c2 && i1 >= i2;
    }
  }

  static inline bool dominates(const Entry &e, const int t, const ChainType c,
                               const IndexType i) noexcept {
    return dominates(e.time, e.chain, e.index, t, c, i);
  }
};

using ChainIndexType = std::uint32_t;
using PositionIndexType = std::uint16_t;

using ForwardStaircase = Staircase<ChainIndexType, PositionIndexType, true>;
using BackwardStaircase = Staircase<ChainIndexType, PositionIndexType, false>;

template <typename STAIRCASE> class RouteStopLabel {
public:
  inline STAIRCASE &operator[](const RouteId route) noexcept {
    for (auto &entry : byRoute) {
      if (entry.first == route)
        return entry.second;
    }
    byRoute.emplace_back(route, STAIRCASE());
    return byRoute.back().second;
  }

  inline const STAIRCASE *find(const RouteId route) const noexcept {
    for (const auto &entry : byRoute) {
      if (entry.first == route)
        return &entry.second;
    }
    return nullptr;
  }

  inline void sort() noexcept {
    std::sort(byRoute.begin(), byRoute.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
    for (auto &entry : byRoute) {
      entry.second.sortByChain();
    }
  }

  inline void clear() noexcept { byRoute.clear(); }

  inline bool empty() const noexcept { return byRoute.empty(); }

  inline std::size_t size() const noexcept { return byRoute.size(); }

  inline auto begin() const noexcept { return byRoute.begin(); }

  inline auto end() const noexcept { return byRoute.end(); }

private:
  std::vector<std::pair<RouteId, STAIRCASE>> byRoute;
};

using ForwardRouteStopLabel = RouteStopLabel<ForwardStaircase>;
using BackwardRouteStopLabel = RouteStopLabel<BackwardStaircase>;

class Data {
public:
  using HLLabel = HL::GroupLabel<RouteId, ChainIndexType, PositionIndexType>;

public:
  Data(const TripBased::Data &tbData)
      : data(tbData), labels{std::vector<HLLabel>(tbData.numberOfStopEvents()),
                             std::vector<HLLabel>(tbData.numberOfStopEvents())},
        forwardStopLabels(tbData.numberOfStops()),
        backwardStopLabels(tbData.numberOfStops()) {}

public:
  inline const TripBased::Data &tripBasedData() const noexcept { return data; }

  inline bool isDepartureEvent(const TripId trip,
                               const StopIndex index) const noexcept {
    return index + 1 < data.numberOfStopsInTrip(trip);
  }

  inline bool isArrivalEvent(const TripId trip,
                             const StopIndex index) const noexcept {
    return index > 0;
  }

  inline StopIndex
  earliestTransferPosition(const TripId trip, const StopIndex index,
                           const bool isDeparture) const noexcept {
    Assert(data.isTrip(trip),
           "The id " << trip << " does not represent a trip!");
    return isDeparture ? StopIndex(index + 1) : index;
  }

  inline ChainIndexType tripRankInRoute(const TripId trip) const noexcept {
    Assert(data.isTrip(trip),
           "The id " << trip << " does not represent a trip!");
    return static_cast<ChainIndexType>(
        trip - data.firstTripOfRoute[data.routeOfTrip[trip]]);
  }

  inline HLLabel &forwardLabel(const StopEventId event) noexcept {
    return labels[FORWARD][event];
  }

  inline const HLLabel &forwardLabel(const StopEventId event) const noexcept {
    return labels[FORWARD][event];
  }

  inline HLLabel &backwardLabel(const StopEventId event) noexcept {
    return labels[BACKWARD][event];
  }

  inline const HLLabel &backwardLabel(const StopEventId event) const noexcept {
    return labels[BACKWARD][event];
  }

  inline ForwardRouteStopLabel &forwardStopLabel(const StopId stop) noexcept {
    return forwardStopLabels[stop];
  }

  inline const ForwardRouteStopLabel &
  forwardStopLabel(const StopId stop) const noexcept {
    return forwardStopLabels[stop];
  }

  inline BackwardRouteStopLabel &backwardStopLabel(const StopId stop) noexcept {
    return backwardStopLabels[stop];
  }

  inline const BackwardRouteStopLabel &
  backwardStopLabel(const StopId stop) const noexcept {
    return backwardStopLabels[stop];
  }

  inline void buildRouteStopLabels() noexcept {
    for (ForwardRouteStopLabel &rsl : forwardStopLabels)
      rsl.clear();
    for (BackwardRouteStopLabel &rsl : backwardStopLabels)
      rsl.clear();

    for (const TripId trip : data.trips()) {
      const std::size_t tripLength = data.numberOfStopsInTrip(trip);

      for (StopIndex index = StopIndex(0); index < tripLength; index++) {
        const StopEventId event = data.getStopEventId(trip, index);
        const StopId stop = data.getStop(trip, index);

        if (isDepartureEvent(trip, index)) {
          for (const HLLabel::Entry &hub : forwardLabel(event).entries) {
            forwardStopLabel(stop)[hub.group].insert(
                data.departureTime(event), hub.chain, hub.index, event);
          }
        }

        if (isArrivalEvent(trip, index)) {
          for (const HLLabel::Entry &hub : backwardLabel(event).entries) {
            backwardStopLabel(stop)[hub.group].insert(
                data.arrivalTime(event), hub.chain, hub.index, event);
          }
        }
      }
    }

    for (ForwardRouteStopLabel &rsl : forwardStopLabels)
      rsl.sort();
    for (BackwardRouteStopLabel &rsl : backwardStopLabels)
      rsl.sort();
  }

  inline int query(const StopId source, const StopId target,
                   const int departureTime) const noexcept {
    Assert(data.isStop(source),
           "The id " << source << " does not represent a stop!");
    Assert(data.isStop(target),
           "The id " << target << " does not represent a stop!");

    int best = never;
    const ForwardRouteStopLabel &F = forwardStopLabels[source];
    const BackwardRouteStopLabel &B = backwardStopLabels[target];

    auto fIt = F.begin();
    auto bIt = B.begin();
    while (fIt != F.end() && bIt != B.end()) {
      if (fIt->first < bIt->first) {
        ++fIt;
        continue;
      }
      if (bIt->first < fIt->first) {
        ++bIt;
        continue;
      }

      best = std::min(best, sweep(fIt->second, bIt->second, departureTime));
      ++fIt;
      ++bIt;
    }

    return best;
  }

private:
  static inline int sweep(const ForwardStaircase &F, const BackwardStaircase &B,
                          const int departureTime) noexcept {
    int best = never;
    PositionIndexType minIndex = std::numeric_limits<PositionIndexType>::max();
    std::size_t pf = 0;

    for (const auto &b : B.entries) {
      while (pf < F.entries.size() && F.entries[pf].chain <= b.chain) {
        if (F.entries[pf].time >= departureTime) {
          minIndex = std::min(minIndex, F.entries[pf].index);
        }
        pf++;
      }
      if (minIndex <= b.index) {
        best = std::min(best, b.time);
      }
    }

    return best;
  }

private:
  TripBased::Data data;

  std::array<std::vector<HLLabel>, 2> labels;

  std::vector<ForwardRouteStopLabel> forwardStopLabels;
  std::vector<BackwardRouteStopLabel> backwardStopLabels;
};

} // namespace RoutePTL
