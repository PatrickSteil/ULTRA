#pragma once

#include <vector>

#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/RouteLabel.h"
#include "../Query/Profiler.h"
#include "../Query/ReachedIndex.h"

#include "StopArrivalTimes.h"

namespace TripBased {

template <typename PROFILER = NoProfiler> class ForwardPruningQuery {

public:
  using Profiler = PROFILER;
  using Type = ForwardPruningQuery<Profiler>;

private:
  struct TripLabel {
    TripLabel(const StopEventId begin = noStopEvent,
              const StopEventId end = noStopEvent)
        : begin(begin), end(end) {}
    StopEventId begin;
    StopEventId end;
  };

  struct EdgeRange {
    EdgeRange() : begin(noEdge), end(noEdge) {}

    Edge begin;
    Edge end;
  };

  struct EdgeLabel {
    EdgeLabel(const StopEventId stopEvent = noStopEvent,
              const TripId trip = noTripId,
              const StopEventId firstEvent = noStopEvent)
        : stopEvent(stopEvent), trip(trip), firstEvent(firstEvent) {}
    StopEventId stopEvent;
    TripId trip;
    StopEventId firstEvent;
  };

public:
  ForwardPruningQuery(const Data &data, const TransferGraph &transferGraph,
                      const TransferGraph &reverseTransferGraph,
                      std::vector<int> &transferFromSource,
                      std::vector<int> &transferToTarget, Profiler &profiler)
      : data(data), transferGraph(transferGraph),
        reverseTransferGraph(reverseTransferGraph),
        transferFromSource(transferFromSource),
        transferToTarget(transferToTarget), queue(data.numberOfStopEvents()),
        edgeRanges(data.numberOfStopEvents()), queueSize(0), reachedIndex(data),
        stopArrivalTimes(data), targetLabels(1), minArrivalTime(INFTY),
        edgeLabels(data.stopEventGraph.numEdges()), sourceStop(noStop),
        targetStop(noStop), sourceDepartureTime(never), arrivalSlack(INFTY),
        maxTrips(0), profiler(profiler) {
    for (const Edge edge : data.stopEventGraph.edges()) {
      edgeLabels[edge].stopEvent =
          StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
      edgeLabels[edge].trip =
          data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
      edgeLabels[edge].firstEvent =
          data.firstStopEventOfTrip[edgeLabels[edge].trip];
    }
    routeLabels.reserve(data.numberOfRoutes());
    for (const RouteId route : data.routes()) {
      routeLabels.emplace_back(data, route);
    }
  }

  inline void run(const StopId source, const int departureTime,
                  const StopId target, const double arrSlack,
                  const double tripSlack) noexcept {
    clear();
    sourceStop = source;
    targetStop = target;
    sourceDepartureTime = departureTime;
    arrivalSlack = arrSlack;
    computeInitialAndFinalTransfers();
    evaluateInitialTransfers();
    scanTrips();
    computeAnchorLabels(tripSlack);
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
      addTargetLabel(sourceDepartureTime);
    for (const Edge edge : reverseTransferGraph.edgesFrom(targetStop)) {
      const Vertex stop = reverseTransferGraph.get(ToVertex, edge);
      if (stop == sourceStop)
        addTargetLabel(sourceDepartureTime +
                       reverseTransferGraph.get(TravelTime, edge));
      transferToTarget[stop] = reverseTransferGraph.get(TravelTime, edge);
    }
    lastSource = sourceStop;
    lastTarget = targetStop;
    profiler.donePhase(PHASE_SCAN_INITIAL);
  }

  inline void evaluateInitialTransfers() noexcept {
    for (const RAPTOR::RouteSegment &segment :
         data.routesContainingStop(sourceStop)) {
      const TripId trip = data.getEarliestTrip(segment, sourceDepartureTime);
      if (trip != noTripId) {
        enqueue(trip, StopIndex(segment.stopIndex + 1));
      }
    }

    for (const Edge edge : transferGraph.edgesFrom(sourceStop)) {
      const Vertex stop = transferGraph.get(ToVertex, edge);
      const int stopDepartureTime =
          sourceDepartureTime + transferGraph.get(TravelTime, edge);
      for (const RAPTOR::RouteSegment &segment :
           data.routesContainingStop(StopId(stop))) {
        const TripId trip = data.getEarliestTrip(segment, stopDepartureTime);
        if (trip != noTripId) {
          enqueue(trip, StopIndex(segment.stopIndex + 1));
        }
      }
    }
  }

  inline int getTargetArrivalTime(const size_t numTrips) const noexcept {
    return targetLabels[std::min(numTrips, targetLabels.size() - 1)];
  }

  inline const std::vector<RAPTOR::ArrivalLabel> &
  getAnchorLabels() const noexcept {
    return anchorLabels;
  }

  inline size_t getMaxTrips() const noexcept { return maxTrips; }

  inline int getArrivalTime(const StopId stop,
                            const size_t round) const noexcept {
    return stopArrivalTimes(stop, round);
  }

private:
  inline void clear() noexcept {
    queueSize = 0;
    reachedIndex.clear();
    stopArrivalTimes.clear();
    std::vector<int>(1, INFTY).swap(targetLabels);
    minArrivalTime = INFTY;
    anchorLabels.clear();
    maxTrips = 0;
  }

  inline void scanTrips() noexcept {
    size_t roundBegin = 0;
    size_t roundEnd = queueSize;
    while (roundBegin < roundEnd) {
      stopArrivalTimes.startNewRound();
      targetLabels.emplace_back(targetLabels.back());
      // Evaluate final transfers in order to check if the target is reachable
      for (size_t i = roundBegin; i < roundEnd; i++) {
        const TripLabel &label = queue[i];
        profiler.countMetric(METRIC_SCANNED_TRIPS);
        for (StopEventId j = label.begin; j < label.end; j++) {
          profiler.countMetric(METRIC_SCANNED_STOPS);
          if (data.arrivalEvents[j].arrivalTime >= minArrivalTime)
            break;
          const int timeToTarget = transferToTarget[data.arrivalEvents[j].stop];
          if (timeToTarget != INFTY)
            addTargetLabel(data.arrivalEvents[j].arrivalTime + timeToTarget);
        }
      }
      // Find the range of transfers for each trip
      for (size_t i = roundBegin; i < roundEnd; i++) {
        TripLabel &label = queue[i];
        for (StopEventId j = label.begin; j < label.end; j++) {
          if (data.arrivalEvents[j].arrivalTime >= minArrivalTime)
            label.end = j;
        }
        edgeRanges[i].begin =
            data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
        edgeRanges[i].end =
            data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
      }
      // Update stops
      for (size_t i = roundBegin; i < roundEnd; i++) {
        const TripLabel &label = queue[i];
        for (StopEventId j = label.begin; j < label.end; j++) {
          stopArrivalTimes.update(j);
        }
      }
      // Relax the transfers for each trip
      for (size_t i = roundBegin; i < roundEnd; i++) {
        const EdgeRange &label = edgeRanges[i];
        for (Edge edge = label.begin; edge < label.end; edge++) {
          enqueue(edge);
        }
      }
      roundBegin = roundEnd;
      roundEnd = queueSize;
    }
  }

  inline void enqueue(const TripId trip, const StopIndex index) noexcept {
    profiler.countMetric(METRIC_ENQUEUES);
    if (reachedIndex.alreadyReached(trip, index))
      return;
    const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
    queue[queueSize] = TripLabel(StopEventId(firstEvent + index),
                                 StopEventId(firstEvent + reachedIndex(trip)));
    queueSize++;
    Assert(queueSize <= queue.size(), "Queue is overfull!");
    reachedIndex.update(trip, index);
  }

  inline void enqueue(const Edge edge) noexcept {
    profiler.countMetric(METRIC_ENQUEUES);
    const EdgeLabel &label = edgeLabels[edge];
    if (reachedIndex.alreadyReached(label.trip,
                                    label.stopEvent - label.firstEvent))
      return;
    queue[queueSize] =
        TripLabel(label.stopEvent,
                  StopEventId(label.firstEvent + reachedIndex(label.trip)));
    queueSize++;
    Assert(queueSize <= queue.size(), "Queue is overfull!");
    reachedIndex.update(label.trip,
                        StopIndex(label.stopEvent - label.firstEvent));
  }

  inline void addTargetLabel(const int newArrivalTime) noexcept {
    profiler.countMetric(METRIC_ADD_JOURNEYS);
    if (newArrivalTime < targetLabels.back()) {
      targetLabels.back() = newArrivalTime;
      minArrivalTime = (newArrivalTime - sourceDepartureTime) * arrivalSlack +
                       sourceDepartureTime;
    }
  }

  inline void computeAnchorLabels(const double tripSlack) noexcept {
    for (size_t i = 0; i < targetLabels.size(); i++) {
      if (targetLabels[i] >=
          (anchorLabels.empty() ? never : anchorLabels.back().arrivalTime))
        continue;
      anchorLabels.emplace_back(targetLabels[i], i);
      maxTrips = i;
    }
    maxTrips = std::ceil(maxTrips * tripSlack);
    Vector::reverse(anchorLabels);
  }

private:
  const Data &data;

  const TransferGraph &transferGraph;
  const TransferGraph &reverseTransferGraph;
  std::vector<int> &transferFromSource;
  std::vector<int> &transferToTarget;
  StopId lastSource;
  StopId lastTarget;

  std::vector<TripLabel> queue;
  std::vector<EdgeRange> edgeRanges;
  size_t queueSize;
  ReachedIndex reachedIndex;
  StopArrivalTimes stopArrivalTimes;

  std::vector<int> targetLabels;
  int minArrivalTime;

  std::vector<EdgeLabel> edgeLabels;
  std::vector<RouteLabel> routeLabels;

  StopId sourceStop;
  StopId targetStop;
  int sourceDepartureTime;
  double arrivalSlack;

  std::vector<RAPTOR::ArrivalLabel> anchorLabels;
  size_t maxTrips;

  Profiler &profiler;
};

} // namespace TripBased
