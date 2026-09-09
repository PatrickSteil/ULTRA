#pragma once

#include "../../../DataStructures/Container/IndexedSet.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/RouteLabel.h"

namespace StopPTL {

class Runner {
public:
  using Type = Runner;

private:
  struct TripLabel {
    TripLabel(const StopEventId begin = noStopEvent,
              const StopEventId end = noStopEvent)
        : begin(begin), end(end) {}
    StopEventId begin;
    StopEventId end;
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
  Runner(const TripBased::Data &data)
      : data(data), lastSource(StopId(0)), queue(data.numberOfStopEvents()),
        reachedIndex(data) {
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

  inline void run(const StopId source) noexcept {
    clear();

    for (const auto [trip, index] : tripOfStopEvent(source)) {
      queueSize = 0;
      enqueue(trip, index);
      run(source, trip, index);
    }
  }

private:
  inline void clear() noexcept {
    queueSize = 0;
    reachedIndex.clear();
  }

  inline void run(const StopId stop, const TripId trip,
                  const StopIndex index) noexcept {
    size_t roundBegin = 0;
    size_t roundEnd = queueSize;

    while (roundBegin < roundEnd) {
      for (size_t i = roundBegin; i < roundEnd; i++) {
        const TripLabel &label = queue[i];
        const auto begin =
            data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
        const auto end = data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
        for (Edge edge = begin; edge < end; edge++) {
          enqueue(edge, i);
        }
      }
      const TripLabel &label = queue[i];
      for (StopEventId e = label.begin; e < label.end; ++e) {
        // TODO add (stop, departure time of (trip, index)) to chain labels of e
      }

      roundBegin = roundEnd;
      roundEnd = queueSize;
    }
  }

  inline void enqueue(const TripId trip, const StopIndex index) noexcept {
    const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
    queue[queueSize] = TripLabel(StopEventId(firstEvent + index),
                                 StopEventId(firstEvent + reachedIndex(trip)));
    queueSize++;
    Assert(queueSize <= queue.size(), "Queue is overfull!");
    reachedIndex.update(trip, index);
  }

  inline void enqueue(const Edge edge) noexcept {
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

private:
  const TripBased::Data &data;

  std::vector<TripLabel> queue;
  size_t queueSize;
  ReachedIndex reachedIndex;

  std::vector<EdgeLabel> edgeLabels;
  std::vector<RouteLabel> routeLabels;
};

} // namespace StopPTL
