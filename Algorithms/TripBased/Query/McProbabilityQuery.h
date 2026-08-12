#pragma once

#include "ProbabilityCostData.h"
#include "ProbabilityData.h"

#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Bags.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/RouteLabel.h"

#include "Profiler.h"

namespace TripBased {
template <typename PROFILER = NoProfiler> class McProbabilityQuery {

public:
  using Profiler = PROFILER;
  using Type = McProbabilityQuery<Profiler>;

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
  };

  struct EdgeLabel {
    double probabilityCost;
    StopEventId stopEvent;
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
             probabilityCost <= other.probabilityCost;
    }

    int arrivalTime;
    double probabilityCost;
    u_int32_t parent;
  };

  using TargetBag = RAPTOR::Bag<TargetLabel>;

public:
  McProbabilityQuery(const Data &data)
      : data(data), transferGraph(data.getTransferGraph()),
        reverseTransferGraph(transferGraph),
        transferFromSource(data.numberOfStops(), INFTY),
        transferToTarget(data.numberOfStops(), INFTY), lastSource(StopId(0)),
        lastTarget(StopId(0)), probabilityCostData(data), targetBags(1),
        tripInfo(data.numberOfTrips()),
        edgeLabels(data.stopEventGraph.numEdges()),
        offsets(data.numberOfStopEvents()), sourceStop(noStop),
        targetStop(noStop), sourceDepartureTime(never) {
    reverseTransferGraph.revert();
    queue.reserve(data.numberOfStopEvents());
    for (const TripId trip : data.trips()) {
      tripInfo[trip].tripStart = data.firstStopEventOfTrip[trip];
      tripInfo[trip].tripEnd = data.firstStopEventOfTrip[trip + 1];
      tripInfo[trip].routeEnd =
          data.firstStopEventOfTrip
              [data.firstTripOfRoute[data.routeOfTrip[trip] + 1]];
      tripInfo[trip].tripLength = StopIndex(data.numberOfStopsInTrip(trip));
    }
    for (const Edge edge : data.stopEventGraph.edges()) {
      edgeLabels[edge].probabilityCost =
          probabilityToCost(data.stopEventGraph.get(Probability, edge));
      edgeLabels[edge].stopEvent =
          StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
      const TripId trip =
          data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
      edgeLabels[edge].tripEnd = tripInfo[trip].tripEnd;
      edgeLabels[edge].routeEnd = tripInfo[trip].routeEnd;
      edgeLabels[edge].tripLength = tripInfo[trip].tripLength;
    }
    routeLabels.reserve(data.numberOfRoutes());
    for (const RouteId route : data.routes()) {
      routeLabels.emplace_back(data, route);
    }
    for (StopEventId stopEvent(0); stopEvent < data.numberOfStopEvents();
         stopEvent++) {
      const TripId trip = data.tripOfStopEvent[stopEvent];
      const bool hasPreviousTrip =
          trip > data.firstTripOfRoute[data.routeOfTrip[trip]];
      offsets[stopEvent] = hasPreviousTrip ? data.numberOfStopsInTrip(trip) : 0;
    }
    profiler.registerPhases(
        {PHASE_SCAN_INITIAL, PHASE_EVALUATE_INITIAL, PHASE_SCAN_TRIPS});
    profiler.registerMetrics({METRIC_ROUNDS, METRIC_SCANNED_TRIPS,
                              METRIC_SCANNED_STOPS, METRIC_RELAXED_TRANSFERS,
                              METRIC_ENQUEUES, METRIC_ADD_JOURNEYS});
  }

  inline void run(const Vertex source, const int departureTime,
                  const Vertex target) noexcept {
    Assert(data.isStop(source), "Source " << source << " is not a stop!");
    Assert(data.isStop(target), "Target " << target << " is not a stop!");
    run(StopId(source), departureTime, StopId(target));
  }

  inline void run(const StopId source, const int departureTime,
                  const StopId target) noexcept {
    profiler.start();
    clear();
    sourceStop = source;
    targetStop = target;
    sourceDepartureTime = departureTime;
    computeInitialAndFinalTransfers();
    evaluateInitialTransfers();
    scanTrips();
    profiler.done();
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
  }

  inline void computeInitialAndFinalTransfers() noexcept {
    profiler.startPhase();
    transferFromSource[lastSource] = INFTY;
    for (const Edge edge : transferGraph.edgesFrom(lastSource)) {
      const Vertex stop = transferGraph.get(ToVertex, edge);
      transferFromSource[stop] = INFTY;
    }
    transferToTarget[lastTarget] = INFTY;
    for (const Edge edge : reverseTransferGraph.edgesFrom(lastTarget)) {
      const Vertex stop = reverseTransferGraph.get(ToVertex, edge);
      transferToTarget[stop] = INFTY;
    }
    transferFromSource[sourceStop] = 0;
    for (const Edge edge : transferGraph.edgesFrom(sourceStop)) {
      const Vertex stop = transferGraph.get(ToVertex, edge);
      transferFromSource[stop] = transferGraph.get(TravelTime, edge);
    }
    transferToTarget[targetStop] = 0;
    if (sourceStop == targetStop)
      addTargetLabel(TargetLabel(sourceDepartureTime, 0.0));
    for (const Edge edge : reverseTransferGraph.edgesFrom(targetStop)) {
      const Vertex stop = reverseTransferGraph.get(ToVertex, edge);
      if (stop == sourceStop)
        addTargetLabel(TargetLabel(
            sourceDepartureTime + reverseTransferGraph.get(TravelTime, edge),
            0.0));
      transferToTarget[stop] = reverseTransferGraph.get(TravelTime, edge);
    }
    lastSource = sourceStop;
    lastTarget = targetStop;
    profiler.donePhase(PHASE_SCAN_INITIAL);
  }

  inline void evaluateInitialTransfers() noexcept {
    profiler.startPhase();
    std::vector<bool> reachedRoutes(data.numberOfRoutes(), false);
    for (const RAPTOR::RouteSegment &route :
         data.routesContainingStop(sourceStop)) {
      reachedRoutes[route.routeId] = true;
    }
    for (const Edge edge : transferGraph.edgesFrom(sourceStop)) {
      const Vertex stop = transferGraph.get(ToVertex, edge);
      for (const RAPTOR::RouteSegment &route :
           data.routesContainingStop(StopId(stop))) {
        reachedRoutes[route.routeId] = true;
      }
    }
    for (const RouteId route : data.routes()) {
      if (!reachedRoutes[route])
        continue;
      const RouteLabel &label = routeLabels[route];
      const StopIndex endIndex = label.end();
      const TripId firstTrip = data.firstTripOfRoute[route];
      for (StopIndex stopIndex(0); stopIndex < endIndex; stopIndex++) {
        const StopId stop = data.getStop(firstTrip, stopIndex);
        const int timeFromSource = transferFromSource[stop];
        if (timeFromSource == INFTY)
          continue;
        const int stopDepartureTime = sourceDepartureTime + timeFromSource;
        TripId tripIndex = noTripId;
        if (!label.findEarliestTripBinary(stopIndex, stopDepartureTime,
                                          tripIndex))
          continue;
        enqueue(firstTrip + tripIndex, StopIndex(stopIndex + 1), 0.0);
      }
    }
    profiler.donePhase(PHASE_EVALUATE_INITIAL);
  }

  inline void scanTrips() noexcept {
    profiler.startPhase();
    size_t roundBegin = 0;
    size_t roundEnd = queue.size();
    while (roundBegin < roundEnd) {
      profiler.countMetric(METRIC_ROUNDS);
      targetBags.emplace_back();
      // Find the range of stop events for each trip
      for (size_t i = roundBegin; i < roundEnd; i++) {
        TripLabel &label = queue[i];
        profiler.countMetric(METRIC_SCANNED_TRIPS);
        for (StopEventId j(label.begin + 1); j < label.end; j++) {
          const double probabilityCost = probabilityCostData(j);
          if (probabilityCost < label.probabilityCost) {
            label.end = j;
          } else {
            if (probabilityCost == label.probabilityCost && offsets[j] != 0) {
              const u_int8_t offset = offsets[j];
              for (; j < label.end; j++) {
                if (probabilityCostData(StopEventId(j - offset)) ==
                    label.probabilityCost)
                  label.end = j;
              }
              break;
            }
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
          profiler.countMetric(METRIC_RELAXED_TRANSFERS);
          enqueue(edge, label.probabilityCost, i);
        }
      }
      roundBegin = roundEnd;
      roundEnd = queue.size();
    }
    profiler.donePhase(PHASE_SCAN_TRIPS);
  }

  inline void enqueue(const TripId trip, const StopIndex index,
                      const double probabilityCost) noexcept {
    profiler.countMetric(METRIC_ENQUEUES);
    const TripInfo &info = tripInfo[trip];
    const StopEventId stopEvent = StopEventId(info.tripStart + index);
    if (probabilityCost >= probabilityCostData(stopEvent))
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

    // std::cout << "ENQ Edge " << (int)edge
    //           << ", p(edge)=" << (double)label.probabilityCost << " ("
    //           << costToProbability(label.probabilityCost) << " %)"
    //           << ", current_p=" << (double)probabilityCost << " ("
    //           << costToProbability(probabilityCost) << " %)"
    //           << ", reachedIndex(stopEvent)="
    //           << probabilityCostData(label.stopEvent) << " ("
    //           << costToProbability(probabilityCostData(label.stopEvent))
    //           << " %)\n";
    if (probabilityCost >= probabilityCostData(label.stopEvent))
      return;
    const StopEventId end = probabilityCostData.getScanEnd(
        StopEventId(label.stopEvent + 1), label.tripEnd, probabilityCost);
    queue.emplace_back(label.stopEvent, end, probabilityCost, parent);
    probabilityCostData.update(label.stopEvent, label.tripEnd, label.routeEnd,
                               label.tripLength, probabilityCost);
  }

  inline void addTargetLabel(const TargetLabel &newLabel) noexcept {
    profiler.countMetric(METRIC_ADD_JOURNEYS);
    if (!bestTargetBag.merge(newLabel))
      return;
    targetBags.back().mergeUndominated(newLabel);
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
      // TODO this is current hardcoded to be instant, no TravelTime stored for
      // this edge
      const int transferArrivalTime =
          (edge == noEdge) ? targetLabel.arrivalTime : arrivalTime;
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

private:
  const Data &data;

  const TransferGraph &transferGraph;
  TransferGraph reverseTransferGraph;
  std::vector<int> transferFromSource;
  std::vector<int> transferToTarget;
  StopId lastSource;
  StopId lastTarget;

  std::vector<TripLabel> queue;
  ProbabilityCostData probabilityCostData;

  std::vector<TargetBag> targetBags;
  TargetBag bestTargetBag;

  std::vector<TripInfo> tripInfo;
  std::vector<EdgeLabel> edgeLabels;
  std::vector<RouteLabel> routeLabels;
  std::vector<u_int8_t> offsets;

  StopId sourceStop;
  StopId targetStop;
  int sourceDepartureTime;

  Profiler profiler;
};

} // namespace TripBased
