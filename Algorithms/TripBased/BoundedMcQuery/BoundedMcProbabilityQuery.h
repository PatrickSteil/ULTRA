#pragma once

#include <cmath>
#include <iostream>

#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Bags.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../Helpers/ProbabilityCost.h"

#include "../Query/ProbabilityData.h"
#include "../Query/Profiler.h"
#include "BackwardPruningQuery.h"
#include "ForwardPruningQuery.h"
#include "TimestampedProbabilityCostData.h"

namespace TripBased {

template <typename PROFILER = NoProfiler> class BoundedMcProbabilityQuery {

public:
  using Profiler = PROFILER;
  using Type = BoundedMcProbabilityQuery<Profiler>;

private:
  struct TripLabel {
    TripLabel(
        const StopEventId begin = noStopEvent,
        const StopEventId end = noStopEvent,
        const double probabilityCost = std::numeric_limits<double>::infinity(),
        const u_int32_t parent = -1)
        : begin(begin), end(end), probabilityCost(probabilityCost),
          parent(parent), edgeBegin(noEdge), edgeEnd(noEdge) {}
    StopEventId begin;
    StopEventId end;
    double probabilityCost;
    u_int32_t parent;
    Edge edgeBegin;
    Edge edgeEnd;
  };

  struct TripInfo {
    StopEventId tripStart;
    StopEventId tripEnd;
    StopEventId routeEnd;
    StopIndex tripLength;
    TripId reverseTrip;
  };

  struct EdgeLabel {
    double probabilityCost;
    StopEventId stopEvent;
    TripId trip;
    TripId reverseTrip;
    StopIndex reverseStopIndex;
    StopEventId tripEnd;
    StopEventId routeEnd;
    StopIndex tripLength;
  };

  struct TargetLabel {
    TargetLabel(
        const int arrivalTime = never,
        const double probabilityCost = std::numeric_limits<double>::infinity(),
        const u_int32_t parent = -1)
        : arrivalTime(arrivalTime), probabilityCost(probabilityCost),
          parent(parent) {}

    inline bool dominates(const TargetLabel &other) const noexcept {
      return arrivalTime <= other.arrivalTime &&
             costLessEqual(probabilityCost, other.probabilityCost);
    }

    int arrivalTime;
    double probabilityCost;
    u_int32_t parent;
  };

  using TargetBag = RAPTOR::Bag<TargetLabel>;

public:
  BoundedMcProbabilityQuery(const Data &data, const Data &forwardBoundedData,
                            const Data &backwardBoundedData)
      : data(data), transferGraph(data.getTransferGraph()),
        reverseTransferGraph(transferGraph),
        transferFromSource(data.numberOfStops(), INFTY),
        transferToTarget(data.numberOfStops(), INFTY),
        forwardPruningQuery(forwardBoundedData, transferGraph,
                            reverseTransferGraph, transferFromSource,
                            transferToTarget, profiler),
        backwardPruningQuery(backwardBoundedData, transferGraph,
                             reverseTransferGraph, transferFromSource,
                             transferToTarget, forwardPruningQuery, profiler),
        probabilityCostData(data), targetBags(1),
        tripInfo(data.numberOfTrips()),
        edgeLabels(data.stopEventGraph.numEdges()),
        offsets(data.numberOfStopEvents()), sourceStop(noVertex),
        targetStop(noVertex), sourceDepartureTime(never), maxTrips(-1) {
    reverseTransferGraph.revert();
    queue.reserve(data.numberOfStopEvents());
    for (const TripId trip : data.trips()) {
      tripInfo[trip].tripStart = data.firstStopEventOfTrip[trip];
      tripInfo[trip].tripEnd = data.firstStopEventOfTrip[trip + 1];
      tripInfo[trip].routeEnd =
          data.firstStopEventOfTrip
              [data.firstTripOfRoute[data.routeOfTrip[trip] + 1]];
      tripInfo[trip].tripLength = StopIndex(data.numberOfStopsInTrip(trip));
      const RouteId route = data.routeOfTrip[trip];
      const TripId tripOffset = trip - data.firstTripOfRoute[route];
      tripInfo[trip].reverseTrip =
          TripId(data.firstTripOfRoute[route + 1] - tripOffset - 1);
    }
    for (const Edge edge : data.stopEventGraph.edges()) {
      edgeLabels[edge].probabilityCost =
          probabilityToCost(data.stopEventGraph.get(Probability, edge));
      edgeLabels[edge].stopEvent =
          StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
      edgeLabels[edge].trip =
          data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
      edgeLabels[edge].reverseTrip =
          tripInfo[edgeLabels[edge].trip].reverseTrip;
      const StopIndex index =
          data.indexOfStopEvent[edgeLabels[edge].stopEvent - 1];
      edgeLabels[edge].reverseStopIndex = StopIndex(
          data.numberOfStopsInTrip(edgeLabels[edge].trip) - index - 1);
      edgeLabels[edge].tripEnd = tripInfo[edgeLabels[edge].trip].tripEnd;
      edgeLabels[edge].routeEnd = tripInfo[edgeLabels[edge].trip].routeEnd;
      edgeLabels[edge].tripLength = tripInfo[edgeLabels[edge].trip].tripLength;
    }
    for (StopEventId stopEvent(0); stopEvent < data.numberOfStopEvents();
         stopEvent++) {
      const TripId trip = data.tripOfStopEvent[stopEvent];
      const bool hasPreviousTrip =
          trip > data.firstTripOfRoute[data.routeOfTrip[trip]];
      offsets[stopEvent] = hasPreviousTrip ? data.numberOfStopsInTrip(trip) : 0;
    }
    profiler.registerPhases({PHASE_FORWARD, PHASE_BACKWARD, PHASE_MAIN});
    profiler.registerMetrics(
        {METRIC_ROUNDS, METRIC_SCANNED_TRIPS, METRIC_SCANNED_STOPS,
         METRIC_ENQUEUES, METRIC_ADD_JOURNEYS, METRIC_FORWARD_ADD_JOURNEYS});
  }

  inline void setMinProbability(const double minProbability) noexcept {
    minProbabilityCost = (minProbability > 0.0)
                             ? probabilityToCost(minProbability)
                             : std::numeric_limits<double>::infinity();
  }

  inline void setProbabilityExponent(const double exponent = 2.0) noexcept {
    probabilityExponent = exponent;
  }

  inline double currentMaxProbabilityCost() const noexcept {
    if (probabilityExponent == 1.0 ||
        fastestProbabilityCost == std::numeric_limits<double>::infinity())
      return minProbabilityCost;

    // std::expm1(x) calculates exp(x) - 1 accurately when x is close to 0
    // std::log1p(x) calculates log(1 + x) accurately when x is close to 0
    const double failureProb = -std::expm1(-fastestProbabilityCost);
    const double cost = -std::log1p(-failureProb / probabilityExponent);

    return std::min(minProbabilityCost, cost);
  }

  inline void run(const StopId source, const int departureTime,
                  const StopId target, const double arrivalSlack,
                  const double tripSlack) noexcept {
    profiler.start();
    profiler.startPhase();
    clear();
    sourceStop = source;
    targetStop = target;
    sourceDepartureTime = departureTime;
    profiler.donePhase(PHASE_MAIN);

    profiler.startPhase();
    forwardPruningQuery.run(source, departureTime, target, arrivalSlack,
                            tripSlack);
    profiler.donePhase(PHASE_FORWARD);
    if (forwardPruningQuery.getAnchorLabels().empty())
      return;
    maxTrips = forwardPruningQuery.getMaxTrips();
    profiler.startPhase();
    backwardPruningQuery.run(target, departureTime, source, arrivalSlack,
                             tripSlack);
    profiler.donePhase(PHASE_BACKWARD);

    profiler.startPhase();
    computeInitialAndFinalTransfers();
    evaluateInitialTransfers();
    scanTrips();
    profiler.donePhase(PHASE_MAIN);
    profiler.done();
  }

  inline void verify(const double arrivalSlack, const double tripSlack,
                     const int departureTime) const noexcept {
    const std::vector<RAPTOR::ArrivalLabel> &anchorLabels =
        forwardPruningQuery.getAnchorLabels();
    for (const RAPTOR::ArrivalLabel &anchorLabel : anchorLabels) {
      Ensure(isContained(anchorLabel), "Anchor label with arrival time "
                                           << anchorLabel.arrivalTime << " and "
                                           << anchorLabel.numberOfTrips
                                           << " was not found!");
    }
    for (const RAPTOR::ProbabilityParetoLabel &label : getResults()) {
      if (!label.isWithinSlack(anchorLabels, departureTime, arrivalSlack,
                               tripSlack)) {
        std::cout << "No anchor label found for " << label << std::endl;
        std::cout << "Anchor labels:" << std::endl;
        for (const RAPTOR::ArrivalLabel &anchorLabel : anchorLabels) {
          std::cout << "\t" << anchorLabel << std::endl;
        }
        Ensure(false, "");
      }
    }
  }

  inline const std::vector<RAPTOR::ArrivalLabel> &
  getAnchorLabels() const noexcept {
    return forwardPruningQuery.getAnchorLabels();
  }

  inline std::vector<RAPTOR::Journey> getJourneys() const noexcept {
    std::vector<RAPTOR::Journey> result;
    for (const TargetBag &bag : targetBags) {
      for (const TargetLabel &label : bag) {
        result.emplace_back(getJourney(label));
      }
    }
    return result;
  }

  inline std::vector<RAPTOR::ProbabilityParetoLabel>
  getResults() const noexcept {
    std::vector<RAPTOR::ProbabilityParetoLabel> result;
    for (size_t i = 0; i < targetBags.size(); i++) {
      for (const TargetLabel &label : targetBags[i]) {
        result.emplace_back(label, i);
      }
    }
    return result;
  }

  inline Profiler &getProfiler() noexcept { return profiler; }

private:
  inline void clear() noexcept {
    queue.clear();
    probabilityCostData.clear();
    targetBags.resize(1);
    targetBags[0].clear();
    bestTargetBag.clear();
    fastestArrivalTime = never;
    fastestProbabilityCost = std::numeric_limits<double>::infinity();
  }

  inline void computeInitialAndFinalTransfers() noexcept {
    // NOTE forwardPruningQuery has already filled the transferFromSource and
    // transferToTarget vectors!
    if (transferToTarget[sourceStop] != INFTY) {
      addTargetLabel(
          TargetLabel(sourceDepartureTime + transferToTarget[sourceStop], 0.0));
    }
  }

  inline void evaluateInitialTransfers() noexcept {
    constexpr int LOOKBACK = 2 * 60 * 60;
    constexpr double UPPER_LIMIT = 0.8;

    const double maxCost = currentMaxProbabilityCost();

    auto collectTrips = [&](const StopId stop, const int timeOffset = 0) {
      const int stopDepartureTime = sourceDepartureTime + timeOffset;
      // stop-level admissibility check: unrelated to which trip we might
      // board, so it only needs to be evaluated once per stop
      const int arrivalTime =
          -backwardPruningQuery.getArrivalTime(stop, maxTrips);
      if (stopDepartureTime > arrivalTime)
        return;

      for (const RAPTOR::RouteSegment &segment :
           data.routesContainingStop(stop)) {
        TripId trip =
            data.getEarliestTrip(segment, stopDepartureTime - LOOKBACK);
        const TripId lastTrip = data.firstTripOfRoute[segment.routeId + 1];

        while (trip < lastTrip) {
          const StopEventId event =
              data.getStopEventId(trip, segment.stopIndex);
          const double prop =
              (1 - data.raptorData.delayDistribution[event].second.cdf(
                       stopDepartureTime));
          const double cost = probabilityToCost(prop);

          if (costLessEqual(cost, maxCost)) {
            enqueue(trip, StopIndex(segment.stopIndex + 1), cost);
          }
          trip++;
          if (prop >= UPPER_LIMIT)
            break;
        }
      }
    };

    collectTrips(sourceStop);
    for (const Edge edge : transferGraph.edgesFrom(sourceStop)) {
      const StopId stop = StopId(transferGraph.get(ToVertex, edge));
      const int travelTime = transferGraph.get(TravelTime, edge);

      collectTrips(stop, travelTime);
    }
  }

  inline void scanTrips() noexcept {
    size_t roundBegin = 0;
    size_t roundEnd = queue.size();
    while (targetBags.size() <= maxTrips && roundBegin < roundEnd) {
      profiler.countMetric(METRIC_ROUNDS);
      targetBags.emplace_back();
      // Find the range of stop events for each trip
      for (size_t i = roundBegin; i < roundEnd; i++) {
        TripLabel &label = queue[i];
        profiler.countMetric(METRIC_SCANNED_TRIPS);
        for (StopEventId j(label.begin + 1); j < label.end; j++) {
          const double probabilityCost = probabilityCostData(j);
          if (costLess(probabilityCost, label.probabilityCost)) {
            label.end = j;
          } else if (costEqual(probabilityCost, label.probabilityCost) &&
                     offsets[j] != 0) {
            const u_int8_t offset = offsets[j];
            for (; j < label.end; j++) {
              if (costEqual(probabilityCostData(StopEventId(j - offset)),
                            label.probabilityCost))
                label.end = j;
            }
            break;
          }
        }
      }
      // Evaluate final transfers in order to check if the target is reachable
      for (size_t i = roundBegin; i < roundEnd; i++) {
        const TripLabel &label = queue[i];
        for (StopEventId j = label.begin; j < label.end; j++) {
          profiler.countMetric(METRIC_SCANNED_STOPS);
          const int timeToTarget = transferToTarget[data.arrivalEvents[j].stop];
          if (timeToTarget == INFTY)
            continue;
          const int arrivalTime =
              data.arrivalEvents[j].arrivalTime + timeToTarget;
          const int deadline = -backwardPruningQuery.getDepartureTime(
              maxTrips - currentNumberOfTrips());
          if (arrivalTime > deadline)
            continue;
          const TargetLabel targetLabel(arrivalTime, label.probabilityCost, i);
          addTargetLabel(targetLabel);
        }
      }
      // Find the range of transfers for each trip
      for (size_t i = roundBegin; i < roundEnd; i++) {
        TripLabel &label = queue[i];
        label.edgeBegin =
            data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
        label.edgeEnd = data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
      }
      // Relax the transfers for each trip
      for (size_t i = roundBegin; i < roundEnd; i++) {
        const TripLabel &label = queue[i];
        const TargetLabel pruningLabel(
            data.arrivalEvents[label.begin].arrivalTime, label.probabilityCost);
        if (bestTargetBag.dominates(pruningLabel))
          continue;
        for (Edge edge = label.edgeBegin; edge < label.edgeEnd; edge++) {
          enqueue(edge, label.probabilityCost, i);
        }
      }
      roundBegin = roundEnd;
      roundEnd = queue.size();
    }
  }

  inline size_t currentNumberOfTrips() const noexcept {
    return targetBags.size() - 1;
  }

  inline void enqueue(const TripId trip, const StopIndex index,
                      const double probabilityCost) noexcept {
    profiler.countMetric(METRIC_ENQUEUES);
    const TripInfo &info = tripInfo[trip];
    const StopEventId stopEvent = StopEventId(info.tripStart + index);
    if (!costLess(probabilityCost, probabilityCostData(stopEvent)))

      return;
    const StopIndex reverseStopIndex(info.tripLength - index);
    if (backwardPruningQuery.getReachedIndex(
            info.reverseTrip, maxTrips - currentNumberOfTrips()) >
        reverseStopIndex)
      return;

    const StopEventId end = probabilityCostData.getScanEnd(
        StopEventId(stopEvent + 1), info.tripEnd, probabilityCost);
    queue.emplace_back(stopEvent, end, probabilityCost);
    probabilityCostData.update(stopEvent, info.tripEnd, info.routeEnd,
                               info.tripLength, probabilityCost);
  }

  inline void enqueue(const Edge edge, double probabilityCost,
                      const u_int32_t parent) noexcept {
    profiler.countMetric(METRIC_ENQUEUES);
    const EdgeLabel &label = edgeLabels[edge];
    probabilityCost += label.probabilityCost;
    if (!costLess(probabilityCost, probabilityCostData(label.stopEvent)))
      return;
    if (backwardPruningQuery.getReachedIndex(
            label.reverseTrip, maxTrips - currentNumberOfTrips()) >
        label.reverseStopIndex)
      return;
    const StopEventId end = probabilityCostData.getScanEnd(
        StopEventId(label.stopEvent + 1), label.tripEnd, probabilityCost);
    queue.emplace_back(label.stopEvent, end, probabilityCost, parent);
    probabilityCostData.update(label.stopEvent, label.tripEnd, label.routeEnd,
                               label.tripLength, probabilityCost);
  }

  inline void addTargetLabel(const TargetLabel &newLabel) noexcept {
    if (!costLessEqual(newLabel.probabilityCost, currentMaxProbabilityCost()))
      return;
    profiler.countMetric(METRIC_ADD_JOURNEYS);
    if (!bestTargetBag.merge(newLabel))
      return;
    targetBags.back().mergeUndominated(newLabel);
    if (newLabel.arrivalTime < fastestArrivalTime) {
      fastestArrivalTime = newLabel.arrivalTime;
      fastestProbabilityCost = newLabel.probabilityCost;
    }
  }

  inline int getTravelTime(const StopId fromStop, const StopId toStop) const {
    if (fromStop == toStop)
      return 0;
    for (const Edge edge : transferGraph.edgesFrom(Vertex(fromStop))) {
      if (StopId(transferGraph.get(ToVertex, edge) == toStop))
        return transferGraph.get(TravelTime, edge);
    }
    return INFTY;
  }

  inline RAPTOR::Journey
  getJourney(const TargetLabel &targetLabel) const noexcept {
    RAPTOR::Journey result;
    u_int32_t parent = targetLabel.parent;
    if (parent == u_int32_t(-1)) {
      result.emplace_back(sourceStop, targetStop, sourceDepartureTime,
                          targetLabel.arrivalTime, false);
      return result;
    }
    StopEventId departureStopEvent = noStopEvent;
    Vertex departureStop = targetStop;
    while (parent != u_int32_t(-1)) {
      Assert(parent < queue.size(), "Parent " << parent << " is out of range!");
      const TripLabel &label = queue[parent];
      StopEventId arrivalStopEvent;
      Edge edge;
      std::tie(arrivalStopEvent, edge) =
          (departureStopEvent == noStopEvent)
              ? getParent(label, targetLabel)
              : getParent(label, StopEventId(departureStopEvent + 1));

      const StopId arrivalStop = data.getStopOfStopEvent(arrivalStopEvent);
      const int arrivalTime = data.arrivalTime(arrivalStopEvent);
      const int travelTime =
          getTravelTime(StopId(departureStop), StopId(arrivalStop));
      assert(travelTime != INFTY);
      const int transferArrivalTime =
          (edge == noEdge) ? targetLabel.arrivalTime : arrivalTime + travelTime;
      result.emplace_back(arrivalStop, departureStop, arrivalTime,
                          transferArrivalTime, edge);

      departureStopEvent = StopEventId(label.begin - 1);
      departureStop = data.getStopOfStopEvent(departureStopEvent);
      const RouteId route = data.getRouteOfStopEvent(departureStopEvent);
      const int departureTime = data.departureTime(departureStopEvent);
      result.emplace_back(departureStop, arrivalStop, departureTime,
                          arrivalTime, true, route);

      parent = label.parent;
    }
    const int timeFromSource = transferFromSource[departureStop];
    result.emplace_back(sourceStop, departureStop, sourceDepartureTime,
                        sourceDepartureTime + timeFromSource, noEdge);
    Vector::reverse(result);
    return result;
  }

  inline std::pair<StopEventId, Edge>
  getParent(const TripLabel &parentLabel,
            const StopEventId departureStopEvent) const noexcept {
    for (StopEventId i = parentLabel.begin; i < parentLabel.end; i++) {
      for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(i))) {
        if (edgeLabels[edge].stopEvent == departureStopEvent)
          return std::make_pair(i, edge);
      }
    }
    Ensure(false, "Could not find parent stop event!");
    return std::make_pair(noStopEvent, noEdge);
  }

  inline std::pair<StopEventId, Edge>
  getParent(const TripLabel &parentLabel,
            const TargetLabel &targetLabel) const noexcept {
    for (StopEventId i = parentLabel.begin; i < parentLabel.end; i++) {
      const int timeToTarget = transferToTarget[data.arrivalEvents[i].stop];
      if (timeToTarget == INFTY)
        continue;
      if (data.arrivalEvents[i].arrivalTime + timeToTarget ==
          targetLabel.arrivalTime)
        return std::make_pair(i, noEdge);
    }
    Ensure(false, "Could not find parent stop event!");
    return std::make_pair(noStopEvent, noEdge);
  }

  inline bool
  isContained(const RAPTOR::ArrivalLabel &anchorLabel) const noexcept {
    Ensure(anchorLabel.numberOfTrips < targetBags.size(),
           "Label with " << anchorLabel.numberOfTrips << " is out of bounds!");
    for (const TargetLabel &label : targetBags[anchorLabel.numberOfTrips]) {
      if (label.arrivalTime == anchorLabel.arrivalTime)
        return true;
    }
    return false;
  }

private:
  const Data &data;

  const TransferGraph &transferGraph;
  TransferGraph reverseTransferGraph;
  std::vector<int> transferFromSource;
  std::vector<int> transferToTarget;

  ForwardPruningQuery<Profiler> forwardPruningQuery;
  BackwardPruningQuery<Profiler> backwardPruningQuery;

  std::vector<TripLabel> queue;
  TimestampedProbabilityCostData probabilityCostData;

  std::vector<TargetBag> targetBags;
  TargetBag bestTargetBag;

  std::vector<TripInfo> tripInfo;
  std::vector<EdgeLabel> edgeLabels;
  std::vector<u_int8_t> offsets;

  double minProbabilityCost = std::numeric_limits<double>::infinity();
  double probabilityExponent = 1.0;

  int fastestArrivalTime = never;
  double fastestProbabilityCost = std::numeric_limits<double>::infinity();

  StopId sourceStop;
  StopId targetStop;
  int sourceDepartureTime;

  size_t maxTrips;

  Profiler profiler;
};

} // namespace TripBased
