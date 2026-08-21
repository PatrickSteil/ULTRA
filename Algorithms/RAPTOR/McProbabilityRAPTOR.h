#pragma once

#include <cmath>
#include <vector>

#include "../../DataStructures/Container/IndexedSet.h"
#include "../../DataStructures/Container/Map.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../DataStructures/RAPTOR/Entities/Bags.h"
#include "../../Helpers/ProbabilityCost.h"
#include "Profiler.h"

namespace RAPTOR {

template <bool TARGET_PRUNING, bool TRANSITIVE, typename PROFILER = NoProfiler>
class McProbabilityRAPTOR {
public:
  static constexpr bool TargetPruning = TARGET_PRUNING;
  static constexpr bool Transitive = TRANSITIVE;
  using Profiler = PROFILER;
  using Type = McProbabilityRAPTOR<TargetPruning, Transitive, Profiler>;
  using InitialTransferGraph = TransferGraph;
  using SourceType = StopId;

private:
  struct Label {
    Label()
        : arrivalTime(never), varArrival(0.0),
          probabilityCost(std::numeric_limits<double>::infinity()),
          parentStop(noStop), parentIndex(-1), parentDepartureTime(never),
          routeId(noRouteId) {}

    Label(const Label &parentLabel, const StopId stop, const size_t parentIndex)
        : arrivalTime(parentLabel.arrivalTime),
          varArrival(parentLabel.varArrival),
          probabilityCost(parentLabel.probabilityCost), parentStop(stop),
          parentIndex(parentIndex),
          parentDepartureTime(parentLabel.arrivalTime), transferId(noEdge) {}

    Label(const int departureTime, const StopId sourceStop)
        : arrivalTime(departureTime), varArrival(0.0), probabilityCost(0.0),
          parentStop(sourceStop), parentIndex(-1),
          parentDepartureTime(departureTime), routeId(noRouteId) {}

    int arrivalTime;
    double varArrival;
    double probabilityCost;

    StopId parentStop;
    size_t parentIndex;
    int parentDepartureTime;
    union {
      RouteId routeId;
      Edge transferId;
    };

    inline bool dominates(const Label &other) const noexcept {
      return arrivalTime <= other.arrivalTime &&
             costLessEqual(probabilityCost, other.probabilityCost);
    }
  };

  struct BestLabel {
    BestLabel()
        : arrivalTime(never),
          probabilityCost(std::numeric_limits<double>::infinity()) {}

    BestLabel(const int arrivalTime, const double probabilityCost)
        : arrivalTime(arrivalTime), probabilityCost(probabilityCost) {}

    template <typename LABEL>
    BestLabel(const LABEL &label)
        : arrivalTime(label.arrivalTime),
          probabilityCost(label.probabilityCost) {}

    template <typename LABEL>
    inline bool dominates(const LABEL &other) const noexcept {
      return arrivalTime <= other.arrivalTime &&
             costLessEqual(probabilityCost, other.probabilityCost);
    }

    int arrivalTime;
    double probabilityCost;
  };

  struct RouteLabel {
    const StopEvent *trip;
    double probabilityCost;
    StopIndex parentStop;
    size_t parentIndex;

    inline bool dominates(const RouteLabel &other) const noexcept {
      return trip <= other.trip &&
             costLessEqual(probabilityCost, other.probabilityCost);
    }
  };

  struct SeparatedBestBag {
    inline Bag<BestLabel> &byRoute() noexcept { return labelsByRoute; }
    inline Bag<BestLabel> &byTransfer() noexcept { return labelsByTransfer; }
    Bag<BestLabel> labelsByRoute;
    Bag<BestLabel> labelsByTransfer;
  };

  struct CombinedBestBag {
    inline Bag<BestLabel> &byRoute() noexcept { return labels; }
    inline Bag<BestLabel> &byTransfer() noexcept { return labels; }
    Bag<BestLabel> labels;
  };

  using BagType = Bag<Label>;
  using BestBag =
      std::conditional_t<Transitive, CombinedBestBag, SeparatedBestBag>;
  using Round = std::vector<BagType>;
  using RouteBagType = RouteBag<RouteLabel>;

public:
  McProbabilityRAPTOR(const Data &data,
                      const Profiler &profilerTemplate = Profiler())
      : data(data), bestLabels(data.numberOfStops()),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        routesServingUpdatedStops(data.numberOfRoutes()), sourceStop(noStop),
        targetStop(noStop), sourceDepartureTime(never),
        profiler(profilerTemplate) {
    Assert(data.hasImplicitBufferTimes(),
           "Departure buffer times have to be implicit!");

    profiler.registerExtraRounds(
        {EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
    profiler.registerPhases(
        {PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
    profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS,
                              METRIC_EDGES, METRIC_STOPS_BY_TRIP,
                              METRIC_STOPS_BY_TRANSFER});
    profiler.initialize();
  }

  template <typename ATTRIBUTE>
  McProbabilityRAPTOR(const Data &data, const InitialTransferGraph &,
                      const InitialTransferGraph &, const ATTRIBUTE,
                      const Profiler & = Profiler())
      : McProbabilityRAPTOR(data) {}

  inline void setMinProbability(const double pMin) noexcept {
    maxProbabilityCost = (pMin <= 0.0) ? std::numeric_limits<double>::infinity()
                                       : probabilityToCost(pMin);
  }

  inline void run(const StopId source, const int departureTime,
                  const StopId target = noStop,
                  const size_t maxRounds = INFTY) noexcept {
    profiler.start();
    profiler.startExtraRound(EXTRA_ROUND_CLEAR);
    clear();
    profiler.doneRound();

    profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
    profiler.startPhase();
    initialize(source, departureTime, target);
    profiler.donePhase(PHASE_INITIALIZATION);
    profiler.startPhase();
    relaxTransfers();
    profiler.donePhase(PHASE_TRANSFERS);
    profiler.doneRound();

    for (size_t i = 0; i < maxRounds; i++) {
      profiler.startRound();
      profiler.startPhase();
      startNewRound();
      profiler.donePhase(PHASE_INITIALIZATION);
      profiler.startPhase();
      collectRoutesServingUpdatedStops();
      profiler.donePhase(PHASE_COLLECT);
      profiler.startPhase();
      scanRoutes();
      profiler.donePhase(PHASE_SCAN);
      if (stopsUpdatedByRoute.empty()) {
        profiler.doneRound();
        break;
      }
      profiler.startPhase();
      startNewRound();
      profiler.donePhase(PHASE_INITIALIZATION);
      profiler.startPhase();
      relaxTransfers();
      profiler.donePhase(PHASE_TRANSFERS);
      profiler.doneRound();
    }
    profiler.done();
  }

  inline std::vector<Journey> getJourneys() const noexcept {
    return getJourneys(targetStop);
  }

  inline std::vector<Journey> getJourneys(const StopId stop) const noexcept {
    std::vector<Journey> journeys;
    for (size_t round = 0; round < rounds.size(); round += 2) {
      const size_t trueRound = std::min(round + 1, rounds.size() - 1);
      for (size_t i = 0; i < rounds[trueRound][stop].size(); i++) {
        getJourney(journeys, trueRound, stop, i);
      }
    }
    return journeys;
  }

  inline std::vector<ProbabilityParetoLabel> getResults() const noexcept {
    return getResults(targetStop);
  }

  inline std::vector<ProbabilityParetoLabel>
  getResults(const StopId stop) const noexcept {
    std::vector<ProbabilityParetoLabel> result;
    for (size_t round = 0; round < rounds.size(); round += 2) {
      const size_t trueRound = std::min(round + 1, rounds.size() - 1);
      for (const Label &label : rounds[trueRound][stop].labels) {
        result.emplace_back(label, round / 2);
      }
    }
    return result;
  }

  template <bool RESET_CAPACITIES = false> inline void clear() noexcept {
    stopsUpdatedByRoute.clear();
    stopsUpdatedByTransfer.clear();
    routesServingUpdatedStops.clear();
    targetStop = noStop;
    sourceDepartureTime = never;
    if constexpr (RESET_CAPACITIES) {
      std::vector<Round>().swap(rounds);
      std::vector<BestBag>(bestLabels.size()).swap(bestLabels);
    } else {
      rounds.clear();
      Vector::fill(bestLabels);
    }
  }

  inline void reset() noexcept { clear<true>(); }

  inline const Profiler &getProfiler() const noexcept { return profiler; }

private:
  inline void initialize(const StopId source, const int departureTime,
                         const StopId target) noexcept {
    sourceStop = source;
    targetStop = target;
    sourceDepartureTime = departureTime;
    startNewRound();
    Label initialLabel(sourceDepartureTime, sourceStop);
    arrivalByRoute(source, initialLabel);
    startNewRound();
  }

  inline void collectRoutesServingUpdatedStops() noexcept {
    for (const StopId stop : stopsUpdatedByTransfer) {
      Assert(data.isStop(stop), "Stop " << stop << " is out of range!");
      for (const RouteSegment &route : data.routesContainingStop(stop)) {
        Assert(data.isRoute(route.routeId),
               "Route " << route.routeId << " is out of range!");
        Assert(data.stopIds[data.firstStopIdOfRoute[route.routeId] +
                            route.stopIndex] == stop,
               "RAPTOR data contains invalid route segments!");
        if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId))
          continue;
        if (routesServingUpdatedStops.contains(route.routeId)) {
          routesServingUpdatedStops[route.routeId] = std::min(
              routesServingUpdatedStops[route.routeId], route.stopIndex);
        } else {
          routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
        }
      }
    }
  }

  inline const StopEvent *
  findFirstTripAtOrAfter(const StopEvent *firstTrip, const StopEvent *lastTrip,
                         const StopIndex stopIndex, const size_t tripSize,
                         const int time) const noexcept {
    size_t lo = 0;
    size_t hi = (lastTrip - firstTrip) / tripSize;
    while (lo < hi) {
      const size_t mid = lo + (hi - lo) / 2;
      const std::size_t stopEventIdx =
          (mid * tripSize) + stopIndex + (firstTrip - data.stopEvents.data());
      if (data.delayDistribution[stopEventIdx].second.mean() < time) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    return firstTrip + lo * tripSize;
  }

  inline void scanRoutes() noexcept {
    constexpr double UPPER_LIMIT = 0.8;
    // constexpr int LOOKBACK = 0;
    constexpr int LOOKBACK = 2 * 60 * 60;
    constexpr double EPSILON = 1e-3;

    stopsUpdatedByRoute.clear();
    for (const RouteId route : routesServingUpdatedStops.getKeys()) {
      profiler.countMetric(METRIC_ROUTES);
      StopIndex stopIndex = routesServingUpdatedStops[route];
      const size_t tripSize = data.numberOfStopsInRoute(route);
      Assert(stopIndex < tripSize - 1,
             "Cannot scan a route starting at/after the last stop (Route: "
                 << route << ", StopIndex: " << stopIndex
                 << ", TripSize: " << tripSize << ")!");

      const StopId *stops = data.stopArrayOfRoute(route);
      StopId stop = stops[stopIndex];

      const StopEvent *basePtr = data.stopEvents.data();

      const StopEvent *firstTrip = data.firstTripOfRoute(route);
      const StopEvent *lastTrip = data.lastTripOfRoute(route);

      RouteBagType routeBag;

      while (stopIndex < tripSize - 1) {
        for (size_t i = 0; i < previousRound()[stop].size(); i++) {
          const Label &label = previousRound()[stop][i];

          const StopEvent *trip =
              findFirstTripAtOrAfter(firstTrip, lastTrip, stopIndex, tripSize,
                                     label.arrivalTime - LOOKBACK);

          while (trip < lastTrip) {
            const size_t stopEvent = trip - basePtr;
            Assert(stopEvent < data.delayDistribution.size(),
                   "StopEvent is out of bounds!");

            const auto &departureDist =
                data.delayDistribution[stopEvent].second;

            const double prop =
                (label.varArrival <= 0.0)
                    ? (1.0 - departureDist.cdf(label.arrivalTime))
                    : normalSurvival(
                          departureDist.mean() - label.arrivalTime,
                          departureDist.variance() + label.varArrival, 0.0);

            if (prop >= EPSILON) {
              const double boardingCost = probabilityToCost(prop);
              const double totalCost = label.probabilityCost + boardingCost;

              if (costLessEqual(totalCost, maxProbabilityCost)) {
                RouteLabel newLabel;
                newLabel.trip = trip;
                newLabel.probabilityCost = totalCost;
                newLabel.parentStop = stopIndex;
                newLabel.parentIndex = i;
                routeBag.merge(newLabel);
              }
            }

            trip += tripSize;
            if (prop >= UPPER_LIMIT)
              break;
          }
        }

        stopIndex++;
        stop = stops[stopIndex];
        profiler.countMetric(METRIC_ROUTE_SEGMENTS);
        for (const RouteLabel &label : routeBag.labels) {
          Label newLabel;
          const std::size_t stopEvent = label.trip - basePtr + stopIndex;
          Assert(stopEvent < data.delayDistribution.size(),
                 "StopEvent is out of bounds!");
          const auto &arrivalDist = data.delayDistribution[stopEvent].first;

          newLabel.arrivalTime = arrivalDist.mean();
          newLabel.varArrival = arrivalDist.variance();
          newLabel.probabilityCost = label.probabilityCost;
          newLabel.parentStop = stops[label.parentStop];
          newLabel.parentIndex = label.parentIndex;
          newLabel.parentDepartureTime =
              label.trip[label.parentStop].departureTime;
          newLabel.routeId = route;
          arrivalByRoute(stop, newLabel);
        }
      }
    }
  }

  inline void relaxTransfers() noexcept {
    stopsUpdatedByTransfer.clear();
    routesServingUpdatedStops.clear();
    for (const StopId stop : stopsUpdatedByRoute) {
      stopsUpdatedByTransfer.insert(stop);
      const BagType &bag = previousRound()[stop];
      currentRound()[stop].resize(bag.size());
      for (size_t i = 0; i < bag.size(); i++) {
        currentRound()[stop][i] = Label(bag[i], stop, i);
      }
    }

    for (const StopId stop : stopsUpdatedByRoute) {
      const BagType &bag = previousRound()[stop];
      for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
        profiler.countMetric(METRIC_EDGES);
        const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
        Assert(data.isStop(toStop),
               "Graph contains edges to non-stop vertices!");
        const int travelTime = data.transferGraph.get(TravelTime, edge);
        for (size_t i = 0; i < bag.size(); i++) {
          Label newLabel;
          newLabel.arrivalTime = bag[i].arrivalTime + travelTime;
          newLabel.probabilityCost = bag[i].probabilityCost;
          newLabel.varArrival = bag[i].varArrival;
          newLabel.parentStop = stop;
          newLabel.parentIndex = i;
          newLabel.parentDepartureTime = bag[i].arrivalTime;
          newLabel.transferId = edge;
          arrivalByTransfer(toStop, newLabel);
        }
      }
    }
  }

  inline Round &currentRound() noexcept {
    Assert(!rounds.empty(),
           "Cannot return current round, because no round exists!");
    return rounds.back();
  }

  inline Round &previousRound() noexcept {
    Assert(rounds.size() >= 2,
           "Cannot return previous round, because less than two rounds exist!");
    return rounds[rounds.size() - 2];
  }

  inline void startNewRound() noexcept {
    rounds.emplace_back(data.numberOfStops());
  }

  inline void arrivalByTransfer(const StopId stop,
                                const Label &label) noexcept {
    Assert(data.isStop(stop), "Stop " << stop << " is out of range!");
    if constexpr (TargetPruning)
      if (bestLabels[targetStop].byTransfer().dominates(label))
        return;
    if (!bestLabels[stop].byTransfer().merge(BestLabel(label)))
      return;
    profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
    currentRound()[stop].mergeUndominated(label);
    Assert(bestLabels[stop].byTransfer().dominates(currentRound()[stop]),
           "Best bag does not dominate current bag! (arrivalByTransfer)");
    stopsUpdatedByTransfer.insert(stop);
  }

  inline void arrivalByRoute(const StopId stop, const Label &label) noexcept {
    Assert(data.isStop(stop), "Stop " << stop << " is out of range!");
    if constexpr (TargetPruning)
      if (bestLabels[targetStop].byTransfer().dominates(label))
        return;
    if (!bestLabels[stop].byRoute().merge(BestLabel(label)))
      return;
    bestLabels[stop].byTransfer().merge(BestLabel(label));
    profiler.countMetric(METRIC_STOPS_BY_TRIP);
    currentRound()[stop].mergeUndominated(label);
    Assert(bestLabels[stop].byTransfer().dominates(currentRound()[stop]),
           "Best bag does not dominate current bag! (arrivalByRoute)");
    stopsUpdatedByRoute.insert(stop);
  }

  inline void getJourney(std::vector<Journey> &journeys, size_t round,
                         StopId stop, size_t index) const noexcept {
    Journey journey;
    do {
      Assert(round != size_t(-1), "Backtracking parent pointers did "
                                  "not pass through the source stop!");
      const Label &label = rounds[round][stop][index];
      journey.emplace_back(label.parentStop, stop, label.parentDepartureTime,
                           label.arrivalTime, round % 2 == 0, label.routeId);
      stop = label.parentStop;
      index = label.parentIndex;
      round--;
    } while (journey.back().from != sourceStop);
    journeys.emplace_back(Vector::reverse(journey));
  }

private:
  const Data &data;

  std::vector<Round> rounds;

  std::vector<BestBag> bestLabels;

  IndexedSet<false, StopId> stopsUpdatedByRoute;
  IndexedSet<false, StopId> stopsUpdatedByTransfer;
  IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

  double maxProbabilityCost = std::numeric_limits<double>::infinity();

  StopId sourceStop;
  StopId targetStop;
  int sourceDepartureTime;

  Profiler profiler;
};

} // namespace RAPTOR
