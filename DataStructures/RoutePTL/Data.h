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

// ---------------------------------------------------------------------------
// Stop labels (Section 7): RSL->(s) / RSL<-(s).
// ---------------------------------------------------------------------------
//
// An event-level route hub entry (group = line, chain = trip index, index =
// stop position) has no notion of time. A physical stop, however, hosts many
// departure/arrival events at different times, each carrying its own such
// entry for a given line -- and, as Section 7.2 shows, none of them need
// dominate another: a later departure with a smaller (chain, index) is
// strictly more useful in the (chain, index) coordinates but strictly worse
// in time, so a stop generally has to remember a small "staircase" of
// pairwise non-dominated (time, chain, index) triples per line, not a single
// scalar (Definition 8/9).

// One point of a staircase: the time (departure time for a forward point,
// arrival time for a backward point) together with the route-hub coordinate
// (chain, index) that a specific stop event contributed. `event` records the
// witness stop event so that a journey can later be extracted (Remark 3);
// it plays no role in domination.
template <typename ChainType, typename IndexType>
struct StaircaseEntry {
  int time;
  ChainType chain;
  IndexType index;
  StopEventId event;

  StaircaseEntry(const int time = never, const ChainType chain = ChainType(),
                 const IndexType index = IndexType(),
                 const StopEventId event = noStopEvent)
      : time(time), chain(chain), index(index), event(event) {}

  friend std::ostream& operator<<(std::ostream& out,
                                  const StaircaseEntry& e) noexcept {
    return out << "(t=" << e.time << ", chain=" << e.chain
               << ", index=" << e.index << ")";
  }
};

// Definition 8: a set of pairwise non-dominated triples for one (stop, line)
// pair. FORWARD_DIRECTION selects which of Section 7.2's two domination
// rules applies:
//   - forward  (departures): (t', i', p') dominates (t, i, p) iff
//     t' >= t, i' <= i, p' <= p -- a departure that is at least as flexible,
//     boards no later and needs less of the target line to have unrolled.
//   - backward (arrivals):   (t', j', q') dominates (t, j, q) iff
//     t' <= t, j' >= j, q' >= q -- an arrival that comes earlier, from a
//     trip that is easier to catch up to, requiring less of the source to
//     have already boarded.
// This is exactly Algorithm 3's InsertFwd / InsertBwd, unified into one
// template so the two directions share a single implementation.
template <typename ChainType, typename IndexType, bool FORWARD_DIRECTION>
class Staircase {
 public:
  using Entry = StaircaseEntry<ChainType, IndexType>;

  // Inserts a new candidate, discarding it if it is dominated by an
  // existing point and otherwise discarding every existing point that it
  // dominates itself (Algorithm 3, lines 1-14). Returns whether the
  // candidate was actually kept.
  inline bool insert(const int time, const ChainType chain,
                     const IndexType index,
                     const StopEventId event = noStopEvent) noexcept {
    for (const Entry& e : entries) {
      if (dominates(e, time, chain, index)) return false;
    }
    entries.erase(std::remove_if(entries.begin(), entries.end(),
                                 [&](const Entry& e) {
                                   return dominates(time, chain, index, e.time,
                                                    e.chain, e.index);
                                 }),
                  entries.end());
    entries.emplace_back(time, chain, index, event);
    return true;
  }

  // Sorts by chain ascending, as required by Algorithm 4's sweep
  // (RSL->(s)[l] by i, RSL<-(s)[l] by j).
  inline void sortByChain() noexcept {
    std::sort(entries.begin(), entries.end(),
              [](const Entry& a, const Entry& b) { return a.chain < b.chain; });
  }

  inline std::size_t size() const noexcept { return entries.size(); }

  inline bool empty() const noexcept { return entries.empty(); }

  std::vector<Entry> entries;

 private:
  // Is (t1, c1, i1) at least as good as (t2, c2, i2), in the sense of this
  // staircase's direction? ("At least as good" rather than "strictly
  // better" -- this mirrors the weak inequalities of Algorithm 3, which
  // already prevents an infinite insert/evict cycle: an exact duplicate is
  // caught by the very first check above and never inserted.)
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

  static inline bool dominates(const Entry& e, const int t, const ChainType c,
                               const IndexType i) noexcept {
    return dominates(e.time, e.chain, e.index, t, c, i);
  }
};

using ChainIndexType = std::uint32_t;
using PositionIndexType = std::uint16_t;

using ForwardStaircase = Staircase<ChainIndexType, PositionIndexType, true>;
using BackwardStaircase = Staircase<ChainIndexType, PositionIndexType, false>;

// Definition 9: RSL->(s) or RSL<-(s), i.e. a small map from RouteId to that
// route's staircase at stop s. A physical stop is typically served by a
// handful of lines out of possibly thousands network-wide (Section 8.3), so
// this is kept as a flat, RouteId-sorted vector -- a linear/binary scan over
// a couple of entries beats hashing at this size, and a sorted layout is
// exactly what the merge-join of Algorithm 4 wants.
template <typename STAIRCASE>
class RouteStopLabel {
 public:
  // Returns the staircase for `route`, creating an empty one if this is
  // the first entry seen for it. Used while building the label
  // (Algorithm 3's InsertFwd/InsertBwd are called through this).
  inline STAIRCASE& operator[](const RouteId route) noexcept {
    for (auto& entry : byRoute) {
      if (entry.first == route) return entry.second;
    }
    byRoute.emplace_back(route, STAIRCASE());
    return byRoute.back().second;
  }

  inline const STAIRCASE* find(const RouteId route) const noexcept {
    for (const auto& entry : byRoute) {
      if (entry.first == route) return &entry.second;
    }
    return nullptr;
  }

  // Sorts the map by RouteId, and every route's staircase by chain --
  // freezing the label into the layout the query below relies on.
  inline void sort() noexcept {
    std::sort(byRoute.begin(), byRoute.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    for (auto& entry : byRoute) {
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

// ---------------------------------------------------------------------------
// RoutePTL::Data
// ---------------------------------------------------------------------------
//
// Holds, on top of a Trip-Based network:
//   - labels[FORWARD] / labels[BACKWARD]: the event-level route hub labels
//     H->(e), H<-(e) of Section 4.5 (one HLLabel per stop event). Only
//     departure events ever get a meaningful forward label, and only
//     arrival events a backward one (Section 4.1); the rest stay empty.
//   - forwardStopLabels / backwardStopLabels: the per-stop route stop
//     labels RSL->(s), RSL<-(s) of Definition 9, obtained by merging the
//     event labels of every event at a stop (Algorithm 3).
//
// This class only holds the label logic and data structures; actually
// *computing* H->/H<- (Algorithms 1-2, the pruned BFS with AlreadyCovered)
// is preprocessing and is intentionally not implemented here.
class Data {
 public:
  using HLLabel = HL::GroupLabel<RouteId, ChainIndexType, PositionIndexType>;

 public:
  Data(const TripBased::Data& tbData)
      : data(tbData),
        labels{std::vector<HLLabel>(tbData.numberOfStopEvents()),
               std::vector<HLLabel>(tbData.numberOfStopEvents())},
        forwardStopLabels(tbData.numberOfStops()),
        backwardStopLabels(tbData.numberOfStops()) {}

 public:
  inline const TripBased::Data& tripBasedData() const noexcept { return data; }

  // ---- Section 4.1/4.4: departure/arrival classification & tau(.) ----

  // Only a trip's first n-1 stops are meaningful boarding points -- you
  // cannot depart from the terminus.
  inline bool isDepartureEvent(const TripId trip,
                               const StopIndex index) const noexcept {
    return index + 1 < data.numberOfStopsInTrip(trip);
  }

  // Only a trip's last n-1 stops are meaningful alighting points -- there
  // is no arrival at the origin.
  inline bool isArrivalEvent(const TripId trip,
                             const StopIndex index) const noexcept {
    return index > 0;
  }

  // Section 4.4's tau(.): the earliest position, on this trip or any
  // later trip of the same line, at which a passenger starting at this
  // event could conceivably transfer onward. An arrival can transfer
  // immediately; a departure must first ride on to the next stop.
  inline StopIndex earliestTransferPosition(
      const TripId trip, const StopIndex index,
      const bool isDeparture) const noexcept {
    Assert(data.isTrip(trip),
           "The id " << trip << " does not represent a trip!");
    return isDeparture ? StopIndex(index + 1) : index;
  }

  // A trip's rank within its route (Section 4.3's total order t1 < t2 <
  // ... < tm): trips of the same route are laid out contiguously and in
  // FIFO order by construction, so this is simply the offset from the
  // route's first trip.
  inline ChainIndexType tripRankInRoute(const TripId trip) const noexcept {
    Assert(data.isTrip(trip),
           "The id " << trip << " does not represent a trip!");
    return static_cast<ChainIndexType>(
        trip - data.firstTripOfRoute[data.routeOfTrip[trip]]);
  }

  // ---- Event-level route hub labels H->(e), H<-(e) (Section 4.5) ----

  inline HLLabel& forwardLabel(const StopEventId event) noexcept {
    return labels[FORWARD][event];
  }

  inline const HLLabel& forwardLabel(const StopEventId event) const noexcept {
    return labels[FORWARD][event];
  }

  inline HLLabel& backwardLabel(const StopEventId event) noexcept {
    return labels[BACKWARD][event];
  }

  inline const HLLabel& backwardLabel(const StopEventId event) const noexcept {
    return labels[BACKWARD][event];
  }

  // ---- Per-stop route stop labels RSL->(s), RSL<-(s) (Definition 9) ----

  inline ForwardRouteStopLabel& forwardStopLabel(const StopId stop) noexcept {
    return forwardStopLabels[stop];
  }

  inline const ForwardRouteStopLabel& forwardStopLabel(
      const StopId stop) const noexcept {
    return forwardStopLabels[stop];
  }

  inline BackwardRouteStopLabel& backwardStopLabel(const StopId stop) noexcept {
    return backwardStopLabels[stop];
  }

  inline const BackwardRouteStopLabel& backwardStopLabel(
      const StopId stop) const noexcept {
    return backwardStopLabels[stop];
  }

  // Algorithm 3: merges the event-level labels of every departure/arrival
  // event into the route stop labels of the stop it belongs to. This is
  // pure bookkeeping over whatever labels[FORWARD]/labels[BACKWARD]
  // currently hold -- it performs no graph search of its own, so it does
  // not depend on how (or whether) those event labels were computed.
  inline void buildRouteStopLabels() noexcept {
    for (ForwardRouteStopLabel& rsl : forwardStopLabels) rsl.clear();
    for (BackwardRouteStopLabel& rsl : backwardStopLabels) rsl.clear();

    for (const TripId trip : data.trips()) {
      const std::size_t tripLength = data.numberOfStopsInTrip(trip);

      for (StopIndex index = StopIndex(0); index < tripLength; index++) {
        const StopEventId event = data.getStopEventId(trip, index);
        const StopId stop = data.getStop(trip, index);

        // Definition 9: Ds(l) / As(l) collect, for every event at s,
        // the (time, chain, index) triple contributed by each of
        // that event's own route-hub entries -- nothing is invented
        // here, this only reshuffles what Algorithms 1-2 (not
        // implemented here) would have put into labels[.][event].
        if (isDepartureEvent(trip, index)) {
          for (const HLLabel::Entry& hub : forwardLabel(event).entries) {
            forwardStopLabel(stop)[hub.group].insert(
                data.departureTime(event), hub.chain, hub.index, event);
          }
        }

        if (isArrivalEvent(trip, index)) {
          for (const HLLabel::Entry& hub : backwardLabel(event).entries) {
            backwardStopLabel(stop)[hub.group].insert(
                data.arrivalTime(event), hub.chain, hub.index, event);
          }
        }
      }
    }

    for (ForwardRouteStopLabel& rsl : forwardStopLabels) rsl.sort();
    for (BackwardRouteStopLabel& rsl : backwardStopLabels) rsl.sort();
  }

  // ---- Query (Algorithm 4 / Theorem 5) ----

  // Earliest arrival time at `target`, departing `source` no earlier than
  // `departureTime`, using only the route stop labels (no walking/transfer
  // graph). Returns `never` if unreachable within the labelled network.
  //
  // Implemented as the linear sweep of Section 7.6 rather than the naive
  // nested loop: shared lines are found via a merge-join over the two
  // RouteId-sorted stop labels, and for each shared line the two
  // chain-sorted staircases are swept once (a running minimum position
  // over eligible forward points, checked against each backward point in
  // increasing chain order).
  inline int query(const StopId source, const StopId target,
                   const int departureTime) const noexcept {
    Assert(data.isStop(source),
           "The id " << source << " does not represent a stop!");
    Assert(data.isStop(target),
           "The id " << target << " does not represent a stop!");

    int best = never;
    const ForwardRouteStopLabel& F = forwardStopLabels[source];
    const BackwardRouteStopLabel& B = backwardStopLabels[target];

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
  // The inner loop of Algorithm 4 for a single shared line: F is sorted by
  // trip index i ascending, B by trip index j ascending. Runs in
  // O(|F| + |B|) since the forward pointer only ever advances.
  static inline int sweep(const ForwardStaircase& F, const BackwardStaircase& B,
                          const int departureTime) noexcept {
    int best = never;
    PositionIndexType minIndex = std::numeric_limits<PositionIndexType>::max();
    std::size_t pf = 0;

    for (const auto& b : B.entries) {
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

}  // namespace RoutePTL
