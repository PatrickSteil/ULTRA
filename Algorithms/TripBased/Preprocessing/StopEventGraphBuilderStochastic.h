#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <ostream>
#include <vector>

#include "../../../DataStructures/Container/Distribution.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/MultiThreading.h"

namespace TripBased {

struct StochasticConfig {
  double phaseAWindowK = 3.0;
  double phaseAProbThreshold = 0.99;
  double phaseAStagnationEpsilon = 0.005;
  int phaseAStagnationPatience = 3;
  double phaseAMaxLookahead = 3600.0;
  int phaseAMaxCandidates = 100;
  double feasibilityEpsilon = 0.01;
};

class StochasticStopEventGraphBuilder {

private:
  struct StochasticLabel {
    double mean;
    double variance;
    double probability;

    inline bool dominates(const StochasticLabel &other) const noexcept {
      return mean <= other.mean && variance <= other.variance &&
             probability >= other.probability;
    }
  };

  struct StopLabel {
  public:
    StopLabel() : timestamp(0) {}

    inline bool insertAt(const int newTimestamp,
                         const StochasticLabel &candidate) noexcept {
      checkTimestamp(newTimestamp);
      return insert(candidate);
    }

    inline void checkTimestamp(const int newTimestamp) noexcept {
      if (timestamp != newTimestamp)
        frontier.clear();
      timestamp = newTimestamp;
    }

    inline bool insert(const StochasticLabel &candidate) noexcept {
      for (const StochasticLabel &existing : frontier) {
        if (existing.dominates(candidate))
          return false;
      }
      frontier.erase(std::remove_if(frontier.begin(), frontier.end(),
                                    [&](const StochasticLabel &existing) {
                                      return candidate.dominates(existing);
                                    }),
                     frontier.end());
      frontier.push_back(candidate);
      return true;
    }

    std::vector<StochasticLabel> frontier;
    int timestamp;
  };

  struct RouteTransfer {
    RouteTransfer(const RouteId toRoute, const StopIndex fromIndex,
                  const StopIndex toIndex, const int transferTime)
        : toRoute(toRoute), fromIndex(fromIndex), toIndex(toIndex),
          transferTime(transferTime) {}

    RouteId toRoute;
    StopIndex fromIndex;
    StopIndex toIndex;
    int transferTime;

    inline std::tuple<RouteId, StopIndex, StopIndex> getTuple() const noexcept {
      return std::make_tuple(toRoute, -fromIndex, toIndex);
    }

    inline bool operator<(const RouteTransfer &other) const noexcept {
      return getTuple() < other.getTuple();
    }
  };

  struct TransferInfo {
    StopEventId toEvent;
    double transferTime;
    double rho;
    double probability;
  };

  // A single candidate transfer discovered while scanning a target route,
  // before it has been recorded into transfersByEvent.
  struct TransferCandidate {
    TripId trip;
    StopEventId toEvent;
    double rho;
    double probability;
  };

  // Result of scanning a target route for feasible connecting trips.
  struct RouteScanResult {
    TripId earliestScanned;
    std::vector<TransferCandidate> candidates;
  };

  friend inline std::ostream &operator<<(std::ostream &os,
                                         const StochasticLabel &label) {
    return os << "{mean=" << label.mean << ", variance=" << label.variance
              << ", probability=" << label.probability << "}";
  }

  friend inline std::ostream &operator<<(std::ostream &os,
                                         const StopLabel &stopLabel) {
    os << "{timestamp=" << stopLabel.timestamp << ", frontier=[";

    for (std::size_t i = 0; i < stopLabel.frontier.size(); ++i) {
      if (i > 0)
        os << ", ";
      os << stopLabel.frontier[i];
    }

    return os << "]}";
  }

  friend inline std::ostream &operator<<(std::ostream &os,
                                         const RouteTransfer &transfer) {
    return os << "{toRoute=" << (size_t)transfer.toRoute
              << ", fromIndex=" << (size_t)transfer.fromIndex
              << ", toIndex=" << (size_t)transfer.toIndex
              << ", transferTime=" << transfer.transferTime << "}";
  }

  friend inline std::ostream &operator<<(std::ostream &os,
                                         const TransferInfo &transfer) {
    return os << "{toEvent=" << (size_t)transfer.toEvent
              << ", transferTime=" << transfer.transferTime
              << ", rho=" << transfer.rho
              << ", probability=" << transfer.probability << "}";
  }

public:
  using CorrelationFunction =
      std::function<double(const StopEventId, const StopEventId)>;

  StochasticStopEventGraphBuilder(
      const Data &data, const StochasticConfig &config = StochasticConfig{},
      CorrelationFunction correlationOf = [](const StopEventId,
                                             const StopEventId) { return 0.0; })
      : data(data), config(config), correlationOf(std::move(correlationOf)),
        labels(data.numberOfStops()), timestamp(0) {
    generatedTransfers.addVertices(data.numberOfStopEvents());
    keptTransfers.addVertices(data.numberOfStopEvents());
    transfersByEvent.resize(data.numberOfStopEvents());
  }

public:
  inline void generateRouteBasedTransfers(const RouteId fromRoute) noexcept {
    if (generatedTransfers.numEdges() > 1000000) {
      generatedTransfers.clear();
      generatedTransfers.addVertices(data.numberOfStopEvents());
      for (auto &transfers : transfersByEvent)
        transfers.clear();
    }
    const std::vector<RouteTransfer> routeTransfers =
        generateRouteTransfers(fromRoute);
    for (const TripId fromTrip : data.tripsOfRoute(fromRoute)) {
      RouteId toRoute = noRouteId;
      std::vector<TripId> earliestTrip;
      for (const RouteTransfer &routeTransfer : routeTransfers) {
        if (routeTransfer.toRoute != toRoute) {
          toRoute = routeTransfer.toRoute;
          std::vector<TripId>(data.numberOfStopsInRoute(toRoute), noTripId)
              .swap(earliestTrip);
        }
        const StopEventId fromEvent =
            data.getStopEventId(fromTrip, routeTransfer.fromIndex);
        const GaussianDist &arrival = arrivalDist(fromEvent);
        const double walkTime = static_cast<double>(routeTransfer.transferTime);

        const RouteScanResult scan = scanTargetRoute(
            arrival, walkTime, toRoute, routeTransfer.toIndex, fromEvent,
            [&](const TripId toTrip) {
              return !((toRoute == fromRoute) && (toTrip >= fromTrip) &&
                       (routeTransfer.toIndex >= routeTransfer.fromIndex));
            });
        if (scan.earliestScanned == noTripId)
          continue;
        if (scan.earliestScanned >= earliestTrip[routeTransfer.toIndex])
          continue;

        for (const TransferCandidate &candidate : scan.candidates) {
          generatedTransfers.addEdge(Vertex(fromEvent),
                                     Vertex(candidate.toEvent));
          transfersByEvent[size_t(fromEvent)].push_back(
              TransferInfo{candidate.toEvent, walkTime, candidate.rho,
                           candidate.probability});
        }

        for (StopIndex i = routeTransfer.toIndex;
             i < data.numberOfStopsInRoute(toRoute); i++) {
          earliestTrip[i] = std::min(earliestTrip[i], scan.earliestScanned);
        }
      }
    }
  }

  inline void generateFullTransfers(const TripId trip) noexcept {
    if (generatedTransfers.numEdges() > 1000000) {
      generatedTransfers.clear();
      generatedTransfers.addVertices(data.numberOfStopEvents());
      for (auto &transfers : transfersByEvent)
        transfers.clear();
    }
    const StopId *stops = data.stopArrayOfTrip(trip);
    for (StopIndex i = StopIndex(1); i < data.numberOfStopsInTrip(trip); i++) {
      const StopId stop = stops[i];
      const StopEventId fromEvent = data.getStopEventId(trip, i);
      findTransfers(trip, i, fromEvent, stop, 0);
      for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stop)) {
        const StopId toStop =
            StopId(data.raptorData.transferGraph.get(ToVertex, edge));
        const int transferTime =
            data.raptorData.transferGraph.get(TravelTime, edge);
        findTransfers(trip, i, fromEvent, toStop, transferTime);
      }
    }
  }

  inline void reduceTransfers(const TripId trip,
                              const bool verbose = false) noexcept {
    timestamp++;
    const StopId *stops = data.stopArrayOfTrip(trip);
    if (verbose)
      std::cout << "\n=== reduceTransfers(trip=" << size_t(trip) << ") ===\n";
    for (StopIndex i = StopIndex(data.numberOfStopsInTrip(trip) - 1); i > 0;
         i--) {
      const StopEventId tripEvent = data.getStopEventId(trip, i);
      const GaussianDist &continuingArrival = arrivalDist(tripEvent);

      if (verbose)
        std::cout << "\n--- stop index " << size_t(i)
                  << " (stop=" << size_t(stops[i])
                  << ", event=" << size_t(tripEvent) << ") ---\n";

      const StochasticLabel continuingLabel{continuingArrival.mean(),
                                            continuingArrival.variance(), 1.0};
      const bool insertedContinuing =
          labels[stops[i]].insertAt(timestamp, continuingLabel);
      if (verbose)
        std::cout << "  continue-on-trip label at stop " << size_t(stops[i])
                  << ": " << continuingLabel
                  << (insertedContinuing ? " [inserted]" : " [dominated]")
                  << "\n";

      for (const Edge edge :
           data.raptorData.transferGraph.edgesFrom(stops[i])) {
        const StopId toStop =
            StopId(data.raptorData.transferGraph.get(ToVertex, edge));
        const int transferTime =
            data.raptorData.transferGraph.get(TravelTime, edge);
        const GaussianDist shiftedContinuing =
            continuingArrival.shifted(transferTime);
        const StochasticLabel footpathLabel{shiftedContinuing.mean(),
                                            shiftedContinuing.variance(), 1.0};
        const bool insertedFootpath =
            labels[toStop].insertAt(timestamp, footpathLabel);
        if (verbose)
          std::cout << "  continue-on-trip + footpath(" << transferTime
                    << ") to stop " << size_t(toStop) << ": " << footpathLabel
                    << (insertedFootpath ? " [inserted]" : " [dominated]")
                    << "\n";
      }

      std::vector<TransferInfo> transfers = transfersByEvent[size_t(tripEvent)];
      std::stable_sort(transfers.begin(), transfers.end(),
                       [&](const TransferInfo &a, const TransferInfo &b) {
                         return arrivalDist(a.toEvent).mean() <
                                arrivalDist(b.toEvent).mean();
                       });

      if (verbose) {
        std::cout << "  " << transfers.size()
                  << " candidate transfer(s) generated here, sorted by "
                     "target arrival mean:\n";
        for (const TransferInfo &t : transfers)
          std::cout << "    " << t << "\n";
      }

      std::vector<TransferInfo> keepTransfers;
      for (const TransferInfo &transfer : transfers) {
        bool keep = false;
        const StopEventId toEvent = transfer.toEvent;
        const StopIndex toIndex = data.indexOfStopEvent[toEvent];
        const TripId toTrip = data.tripOfStopEvent[toEvent];
        const StopId *toStops = data.stopArrayOfTrip(toTrip) + toIndex;

        if (verbose)
          std::cout << "  processing transfer " << transfer << ":\n";

        for (size_t j = data.numberOfStopsInTrip(toTrip) - toIndex - 1; j > 0;
             j--) {
          const StopId destinationStop = toStops[j];
          const StopEventId destinationEvent = StopEventId(toEvent + j);
          const GaussianDist &destinationArrival =
              arrivalDist(destinationEvent);

          const StochasticLabel destinationLabel{destinationArrival.mean(),
                                                 destinationArrival.variance(),
                                                 transfer.probability};
          const bool insertedDestination =
              labels[destinationStop].insertAt(timestamp, destinationLabel);
          if (insertedDestination)
            keep = true;
          if (verbose)
            std::cout << "    reachable stop " << size_t(destinationStop)
                      << " (event=" << size_t(destinationEvent)
                      << "): " << destinationLabel
                      << (insertedDestination ? " [inserted -> KEEP]"
                                              : " [dominated]")
                      << "\n";

          for (const Edge edge :
               data.raptorData.transferGraph.edgesFrom(destinationStop)) {
            const StopId arrivalStop =
                StopId(data.raptorData.transferGraph.get(ToVertex, edge));
            const int transferTime =
                data.raptorData.transferGraph.get(TravelTime, edge);
            const GaussianDist shiftedDestination =
                destinationArrival.shifted(transferTime);
            const StochasticLabel footpathLabel{shiftedDestination.mean(),
                                                shiftedDestination.variance(),
                                                transfer.probability};
            const bool insertedFootpath =
                labels[arrivalStop].insertAt(timestamp, footpathLabel);
            if (insertedFootpath)
              keep = true;
            if (verbose)
              std::cout << "      + footpath(" << transferTime << ") to stop "
                        << size_t(arrivalStop) << ": " << footpathLabel
                        << (insertedFootpath ? " [inserted -> KEEP]"
                                             : " [dominated]")
                        << "\n";
          }
        }
        if (verbose)
          std::cout << "  => transfer "
                    << (keep ? "KEPT" : "REJECTED (dominated everywhere)")
                    << "\n";
        if (keep)
          keepTransfers.push_back(transfer);
      }

      for (const TransferInfo &transfer : keepTransfers) {
        keptTransfers.addEdge(Vertex(tripEvent), Vertex(transfer.toEvent));
      }
      if (verbose)
        std::cout << "  kept " << keepTransfers.size() << "/"
                  << transfers.size() << " transfer(s) at this stop event\n";
    }
  }

  inline void reduceTransfers(const RouteId route,
                              const bool verbose = false) noexcept {
    for (const TripId trip : data.tripsOfRoute(route)) {
      reduceTransfers(trip, verbose);
    }
  }

  inline const SimpleDynamicGraph &getStopEventGraph() const noexcept {
    return keptTransfers;
  }

  inline SimpleDynamicGraph &getStopEventGraph() noexcept {
    return keptTransfers;
  }

  inline void showStats(const size_t numSampleEvents = 5,
                        const size_t maxTransfersPerEvent = 10) const noexcept {
    const size_t numEvents = transfersByEvent.size();

    std::vector<size_t> degrees;
    degrees.reserve(numEvents);
    size_t totalTransfers = 0;
    size_t eventsWithTransfers = 0;
    size_t maxDegree = 0;
    for (const auto &transfers : transfersByEvent) {
      degrees.push_back(transfers.size());
      totalTransfers += transfers.size();
      if (!transfers.empty())
        eventsWithTransfers++;
      maxDegree = std::max(maxDegree, transfers.size());
    }

    std::cout << "=== Stochastic Transfer Generation Stats ===\n";
    std::cout << "Stop events:                 " << numEvents << "\n";
    std::cout << "Events with >=1 transfer:    " << eventsWithTransfers << "\n";
    std::cout << "Total generated transfers:   " << totalTransfers << "\n";
    std::cout << "Avg transfers / event:       "
              << (numEvents ? static_cast<double>(totalTransfers) / numEvents
                            : 0.0)
              << "\n";
    std::cout << "Avg transfers / active event:"
              << (eventsWithTransfers ? static_cast<double>(totalTransfers) /
                                            eventsWithTransfers
                                      : 0.0)
              << "\n";
    std::cout << "Max out-degree:              " << maxDegree << "\n";

    std::vector<size_t> histogram(maxDegree + 1, 0);
    for (const size_t d : degrees)
      histogram[d]++;

    std::cout << "\n--- Out-degree distribution ---\n";
    for (size_t d = 0; d < histogram.size(); d++) {
      if (histogram[d] == 0)
        continue;
      std::cout << "  " << d << " transfers: " << histogram[d] << " events\n";
    }

    std::vector<size_t> sortedDegrees = degrees;
    std::sort(sortedDegrees.begin(), sortedDegrees.end());
    auto percentile = [&](const double p) -> size_t {
      if (sortedDegrees.empty())
        return 0;
      const size_t idx = static_cast<size_t>(p * (sortedDegrees.size() - 1));
      return sortedDegrees[idx];
    };
    std::cout << "\n--- Percentiles (out-degree) ---\n";
    std::cout << "  p50: " << percentile(0.50) << "\n";
    std::cout << "  p90: " << percentile(0.90) << "\n";
    std::cout << "  p99: " << percentile(0.99) << "\n";

    std::cout << "\n--- Sample events (with transfer details) ---\n";
    size_t shown = 0;
    for (size_t event = 0; event < numEvents && shown < numSampleEvents;
         event++) {
      if (transfersByEvent[event].empty())
        continue;
      shown++;
      const StopEventId fromEvent = StopEventId(event);
      const GaussianDist &arrival = arrivalDist(fromEvent);
      std::cout << "\nEvent " << event << " (arrival mean=" << arrival.mean()
                << ") -> " << transfersByEvent[event].size() << " transfers:\n";

      size_t printed = 0;
      for (const TransferInfo &info : transfersByEvent[event]) {
        if (printed >= maxTransfersPerEvent) {
          std::cout << "  ... (" << (transfersByEvent[event].size() - printed)
                    << " more)\n";
          break;
        }
        const GaussianDist &departure = departureDist(info.toEvent);
        std::cout << "  -> event " << size_t(info.toEvent)
                  << " | trip=" << size_t(data.tripOfStopEvent[info.toEvent])
                  << " | route="
                  << size_t(
                         data.routeOfTrip[data.tripOfStopEvent[info.toEvent]])
                  << " | transferTime=" << info.transferTime
                  << " | rho=" << info.rho
                  << " | p(feasible)=" << info.probability
                  << " | departure mean=" << departure.mean() << "\n";
        printed++;
      }
    }
  }

private:
  inline const GaussianDist &
  arrivalDist(const StopEventId event) const noexcept {
    return data.raptorData.delayDistribution[event].first;
  }

  inline const GaussianDist &
  departureDist(const StopEventId event) const noexcept {
    return data.raptorData.delayDistribution[event].second;
  }

  template <typename FUNC>
  inline RouteScanResult scanTargetRoute(
      const GaussianDist &arrival, const double walkTime, const RouteId route,
      const StopIndex index, const StopEventId fromEvent,
      const FUNC &&isValidTrip = [](TripId) { return true; }) const noexcept {
    RouteScanResult result{noTripId, {}};

    const double lo =
        arrival.feasibleWindow(walkTime, config.phaseAWindowK, 0.0).lo;
    const TripId earliest =
        data.getEarliestTrip(route, index, static_cast<int>(std::floor(lo)));
    if (earliest == noTripId)
      return result;
    result.earliestScanned = earliest;

    const Range<TripId> routeTrips = data.tripsOfRoute(route);
    const double maxDeparture =
        arrival.mean() + walkTime + config.phaseAMaxLookahead;
    const double meanDeparture = arrival.mean() + walkTime;

    bool foundFeasible = false;
    bool scoredAny = false;
    bool passedAnchor = false;
    double lastProbability = -1.0;
    int stagnationCount = 0;
    int scanned = 0;

    for (auto it = Range<TripId>::Iterator(earliest); it != routeTrips.end();
         ++it) {
      if (!isValidTrip(*it))
        continue;

      const StopEventId toEvent = data.getStopEventId(*it, index);
      const GaussianDist &departure = departureDist(toEvent);

      if (scoredAny && departure.mean() > maxDeparture)
        break;
      if (++scanned > config.phaseAMaxCandidates)
        break;

      const double rho = correlationOf(fromEvent, toEvent);
      const double p =
          transferFeasibilityProbability(arrival, departure, rho, walkTime);
      scoredAny = true;

      if (departure.mean() >= meanDeparture)
        passedAnchor = true;

      if (p >= config.feasibilityEpsilon) {
        result.candidates.push_back(TransferCandidate{*it, toEvent, rho, p});
        foundFeasible = true;
      }

      // Only allow early termination once we've covered the deterministic case.
      if (passedAnchor) {
        if (p >= config.phaseAProbThreshold)
          break;

        if (foundFeasible) {
          if (lastProbability >= 0.0 &&
              std::abs(p - lastProbability) < config.phaseAStagnationEpsilon) {
            if (++stagnationCount >= config.phaseAStagnationPatience)
              break;
          } else {
            stagnationCount = 0;
          }
          lastProbability = p;
        }
      }
    }

    return result;
  }

  inline std::vector<RouteTransfer>
  generateRouteTransfers(const RouteId fromRoute) const noexcept {
    std::vector<RouteTransfer> routeTransfers;
    const StopId *stops = data.raptorData.stopArrayOfRoute(fromRoute);
    for (StopIndex i(data.numberOfStopsInRoute(fromRoute) - 1); i > 0; i--) {
      const StopId fromStop = stops[i];
      for (const RAPTOR::RouteSegment &toSegment :
           data.raptorData.routesContainingStop(fromStop)) {
        if (toSegment.routeId == fromRoute && toSegment.stopIndex == i)
          continue;
        routeTransfers.emplace_back(toSegment.routeId, i, toSegment.stopIndex,
                                    0);
      }
      for (const Edge edge :
           data.raptorData.transferGraph.edgesFrom(fromStop)) {
        const StopId toStop =
            StopId(data.raptorData.transferGraph.get(ToVertex, edge));
        const int transferTime =
            data.raptorData.transferGraph.get(TravelTime, edge);
        for (const RAPTOR::RouteSegment &toRouteSegment :
             data.raptorData.routesContainingStop(toStop)) {
          routeTransfers.emplace_back(toRouteSegment.routeId, i,
                                      toRouteSegment.stopIndex, transferTime);
        }
      }
    }
    std::sort(routeTransfers.begin(), routeTransfers.end());
    return routeTransfers;
  }

  inline void findTransfers(const TripId fromTrip, const StopIndex fromIndex,
                            const StopEventId fromEvent, const StopId toStop,
                            const int walkTime) noexcept {
    const RouteId fromRoute = data.routeOfTrip[fromTrip];
    const GaussianDist &arrival = arrivalDist(fromEvent);
    const double walkTimeD = static_cast<double>(walkTime);

    for (const RAPTOR::RouteSegment &toSegment :
         data.raptorData.routesContainingStop(toStop)) {
      const RouteScanResult scan = scanTargetRoute(
          arrival, walkTimeD, toSegment.routeId, toSegment.stopIndex, fromEvent,
          [&](const TripId toTrip) {
            return !((toSegment.routeId == fromRoute) && (toTrip >= fromTrip) &&
                     (toSegment.stopIndex >= fromIndex));
          });

      for (const TransferCandidate &candidate : scan.candidates) {
        generatedTransfers.addEdge(Vertex(fromEvent),
                                   Vertex(candidate.toEvent));
        transfersByEvent[size_t(fromEvent)].push_back(
            TransferInfo{candidate.toEvent, walkTimeD, candidate.rho,
                         candidate.probability});
      }
    }
  }

private:
  const Data &data;
  StochasticConfig config;
  CorrelationFunction correlationOf;

  SimpleDynamicGraph generatedTransfers;
  SimpleDynamicGraph keptTransfers;

  std::vector<std::vector<TransferInfo>> transfersByEvent;

  std::vector<StopLabel> labels;
  int timestamp;
};

inline void ComputeStochasticStopEventGraph(Data &data) noexcept {
  Progress progress(data.numberOfTrips());
  StochasticStopEventGraphBuilder builder(
      data, StochasticConfig{},
      [&data](const StopEventId a, const StopEventId b) {
        return data.raptorData.getCorrelation(a, b);
      });

  for (const TripId trip : data.trips()) {
    builder.generateFullTransfers(trip);
    builder.reduceTransfers(trip);
    progress++;
  }
  builder.showStats();
  Graph::move(std::move(builder.getStopEventGraph()), data.stopEventGraph);
  data.stopEventGraph.sortEdges(ToVertex);
  progress.finished();
}

inline void
ComputeStochasticStopEventGraph(Data &data, const int numberOfThreads,
                                const int pinMultiplier = 1) noexcept {
  Progress progress(data.numberOfTrips());
  SimpleEdgeList stopEventGraph;
  stopEventGraph.addVertices(data.numberOfStopEvents());

  const int numCores = numberOfCores();

  omp_set_num_threads(numberOfThreads);
#pragma omp parallel
  {
    int threadId = omp_get_thread_num();
    pinThreadToCoreId((threadId * pinMultiplier) % numCores);
    Assert(omp_get_num_threads() == numberOfThreads,
           "Number of threads is " << omp_get_num_threads()
                                   << ", but should be " << numberOfThreads
                                   << "!");

    StochasticStopEventGraphBuilder builder(
        data, StochasticConfig{},
        [&data](const StopEventId a, const StopEventId b) {
          return data.raptorData.getCorrelation(a, b);
        });

    const size_t numberOfTrips = data.numberOfTrips();

#pragma omp for schedule(dynamic, 1)
    for (size_t i = 0; i < numberOfTrips; i++) {
      const TripId trip = TripId(i);
      builder.generateFullTransfers(trip);
      builder.reduceTransfers(trip);
      progress++;
    }

#pragma omp critical
    {
      for (const auto [edge, from] :
           builder.getStopEventGraph().edgesWithFromVertex()) {
        stopEventGraph.addEdge(from,
                               builder.getStopEventGraph().get(ToVertex, edge));
      }
    }
  }

  Graph::move(std::move(stopEventGraph), data.stopEventGraph);
  data.stopEventGraph.sortEdges(ToVertex);
  progress.finished();
}

inline void ComputeStochasticStopEventGraphRouteBased(Data &data) noexcept {
  Progress progress(data.numberOfRoutes());
  StochasticStopEventGraphBuilder builder(
      data, StochasticConfig{},
      [&data](const StopEventId a, const StopEventId b) {
        return data.raptorData.getCorrelation(a, b);
      });
  for (const RouteId route : data.routes()) {
    builder.generateRouteBasedTransfers(route);
    builder.reduceTransfers(route);
    progress++;
  }
  builder.showStats();
  Graph::move(std::move(builder.getStopEventGraph()), data.stopEventGraph);
  data.stopEventGraph.sortEdges(ToVertex);
  progress.finished();
}

inline void ComputeStochasticStopEventGraphRouteBased(
    Data &data, const int numberOfThreads,
    const int pinMultiplier = 1) noexcept {
  Progress progress(data.numberOfRoutes());
  SimpleEdgeList stopEventGraph;
  stopEventGraph.addVertices(data.numberOfStopEvents());

  const int numCores = numberOfCores();

  omp_set_num_threads(numberOfThreads);
#pragma omp parallel
  {
    int threadId = omp_get_thread_num();
    pinThreadToCoreId((threadId * pinMultiplier) % numCores);
    Assert(omp_get_num_threads() == numberOfThreads,
           "Number of threads is " << omp_get_num_threads()
                                   << ", but should be " << numberOfThreads
                                   << "!");

    StochasticStopEventGraphBuilder builder(
        data, StochasticConfig{},
        [&data](const StopEventId a, const StopEventId b) {
          return data.raptorData.getCorrelation(a, b);
        });
    const size_t numberOfRoutes = data.numberOfRoutes();

#pragma omp for schedule(dynamic, 1)
    for (size_t i = 0; i < numberOfRoutes; i++) {
      const RouteId route = RouteId(i);
      builder.generateRouteBasedTransfers(route);
      builder.reduceTransfers(route);
      progress++;
    }

#pragma omp critical
    {
      for (const auto [edge, from] :
           builder.getStopEventGraph().edgesWithFromVertex()) {
        stopEventGraph.addEdge(from,
                               builder.getStopEventGraph().get(ToVertex, edge));
      }
    }
  }

  Graph::move(std::move(stopEventGraph), data.stopEventGraph);
  data.stopEventGraph.sortEdges(ToVertex);
  progress.finished();
}

} // namespace TripBased
