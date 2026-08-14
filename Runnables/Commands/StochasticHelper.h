#pragma once

#include <algorithm>
#include <iostream>
#include <set>
#include <vector>

#include "../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../DataStructures/TripBased/Data.h"

inline void printTripTransferStats(const TripBased::Data &data,
                                   const TripId trip,
                                   std::ostream &os = std::cout) {
  const size_t numStops = data.numberOfStopsInTrip(trip);
  const StopId *stops = data.stopArrayOfTrip(trip);

  os << "=== Transfer stats for trip " << size_t(trip) << " ===\n";
  os << "Route: " << size_t(data.routeOfTrip[trip])
     << " | stops in trip: " << numStops << "\n";

  size_t totalTransfers = 0;
  double probSum = 0.0;
  double probMin = 1.0, probMax = 0.0;
  std::vector<double> allProbs;

  for (StopIndex i = StopIndex(0); i < numStops; i++) {
    const StopEventId fromEvent = data.getStopEventId(trip, i);
    const GaussianDist &arrival =
        data.raptorData.delayDistribution[fromEvent].first;

    size_t degree = data.stopEventGraph.outDegree(Vertex(fromEvent));
    if (degree == 0)
      continue;

    os << "\n-- stop index " << size_t(i) << " (stop=" << size_t(stops[i])
       << ", event=" << size_t(fromEvent) << ", arrival mean=" << arrival.mean()
       << ", sigma=" << arrival.stddev() << ") --\n";
    os << "  outgoing transfers: " << degree << "\n";

    for (const Edge e : data.stopEventGraph.edgesFrom(Vertex(fromEvent))) {
      const StopEventId toEvent =
          StopEventId(data.stopEventGraph.get(ToVertex, e));
      const TripId toTrip = data.tripOfStopEvent[toEvent];
      const StopIndex toIndex = data.indexOfStopEvent[toEvent];
      const RouteId toRoute = data.routeOfTrip[toTrip];
      const StopId toStop = data.stopArrayOfTrip(toTrip)[toIndex];

      const GaussianDist &departure =
          data.raptorData.delayDistribution[toEvent].second;

      double walkTime = 0.0;
      bool footpathFound = (toStop == stops[i]);
      if (!footpathFound) {
        for (const Edge fe :
             data.raptorData.transferGraph.edgesFrom(stops[i])) {
          if (StopId(data.raptorData.transferGraph.get(ToVertex, fe)) ==
              toStop) {
            walkTime = data.raptorData.transferGraph.get(TravelTime, fe);
            footpathFound = true;
            break;
          }
        }
      }

      const double rho = data.raptorData.getCorrelation(fromEvent, toEvent);
      const double p = data.stopEventGraph.get(Probability, e);
      // transferFeasibilityProbability(arrival, departure, rho, walkTime);

      totalTransfers++;
      probSum += p;
      probMin = std::min(probMin, p);
      probMax = std::max(probMax, p);
      allProbs.push_back(p);

      os << "   -> event " << size_t(toEvent) << " | trip=" << size_t(toTrip)
         << " | route=" << size_t(toRoute) << " | toStop=" << size_t(toStop)
         << " | walkTime=" << walkTime
         << (footpathFound ? "" : " (unresolved!)") << " | rho=" << rho
         << " | departure mean=" << departure.mean()
         << " | sigma=" << departure.stddev() << " | p(feasible)=" << p << "\n";
    }
  }

  os << "\n=== Summary ===\n";
  os << "Total outgoing transfers: " << totalTransfers << "\n";
  if (totalTransfers > 0) {
    os << "Mean p(feasible):   " << (probSum / totalTransfers) << "\n";
    os << "Min p(feasible):    " << probMin << "\n";
    os << "Max p(feasible):    " << probMax << "\n";

    std::sort(allProbs.begin(), allProbs.end());
    auto percentile = [&](double q) {
      const size_t idx = static_cast<size_t>(q * (allProbs.size() - 1));
      return allProbs[idx];
    };
    os << "Median p(feasible): " << percentile(0.5) << "\n";
    os << "p10 p(feasible):    " << percentile(0.10) << "\n";
    os << "p90 p(feasible):    " << percentile(0.90) << "\n";
  } else {
    os << "(no outgoing transfers found for this trip)\n";
  }
}
inline std::vector<RouteId>
routeSignature(const RAPTOR::Journey &journey) noexcept {
  std::vector<RouteId> signature;
  for (const auto &leg : journey) {
    if (leg.usesRoute)
      signature.push_back(RouteId(leg.routeId));
  }
  return signature;
}

inline double jaccardSimilarity(const std::vector<RouteId> &a,
                                const std::vector<RouteId> &b) noexcept {
  const std::set<RouteId> A(a.begin(), a.end());
  const std::set<RouteId> B(b.begin(), b.end());
  if (A.empty() && B.empty())
    return 1.0;
  size_t intersectionSize = 0;
  for (const RouteId &route : A) {
    if (B.count(route) > 0)
      intersectionSize++;
  }
  const size_t unionSize = A.size() + B.size() - intersectionSize;
  if (unionSize == 0)
    return 1.0;
  return static_cast<double>(intersectionSize) / static_cast<double>(unionSize);
}

inline std::vector<size_t>
selectDiverseJourneys(const std::vector<RAPTOR::Journey> &journeys,
                      const double similarityThreshold = 1.0) noexcept {
  std::vector<size_t> kept;
  std::vector<std::vector<RouteId>> keptSignatures;
  kept.reserve(journeys.size());
  keptSignatures.reserve(journeys.size());
  for (size_t i = 0; i < journeys.size(); i++) {
    const std::vector<RouteId> signature = routeSignature(journeys[i]);
    bool tooSimilar = false;
    for (const std::vector<RouteId> &keptSignature : keptSignatures) {
      if (jaccardSimilarity(signature, keptSignature) >= similarityThreshold) {
        tooSimilar = true;
        break;
      }
    }
    if (!tooSimilar) {
      kept.push_back(i);
      keptSignatures.push_back(signature);
    }
  }
  return kept;
}

struct RouteLegInfo {
  RouteId routeId;
  int departureTime;
  int arrivalTime;
};

inline int firstScheduledDeparture(const RAPTOR::Journey &journey) noexcept {
  for (const auto &leg : journey) {
    if (leg.usesRoute)
      return leg.departureTime;
  }
  return journey.empty() ? 0
                         : journey.back().arrivalTime; // no route legs at all
}

inline std::vector<RouteLegInfo>
routeLegInfos(const RAPTOR::Journey &journey) noexcept {
  std::vector<RouteLegInfo> infos;
  for (const auto &leg : journey) {
    if (leg.usesRoute) {
      infos.push_back(
          {RouteId(leg.routeId), leg.departureTime, leg.arrivalTime});
    }
  }
  return infos;
}

struct SharedSpan {
  size_t legs = 0;
  double minutesA = 0.0;
  double minutesB = 0.0;
};

inline SharedSpan sharedPrefix(const RAPTOR::Journey &a,
                               const RAPTOR::Journey &b) noexcept {
  const std::vector<RouteLegInfo> infoA = routeLegInfos(a);
  const std::vector<RouteLegInfo> infoB = routeLegInfos(b);
  SharedSpan span;
  while (span.legs < infoA.size() && span.legs < infoB.size() &&
         infoA[span.legs].routeId == infoB[span.legs].routeId) {
    span.legs++;
  }
  if (span.legs > 0 && !a.empty() && !b.empty()) {
    span.minutesA =
        (infoA[span.legs - 1].arrivalTime - a.front().departureTime) / 60.0;
    span.minutesB =
        (infoB[span.legs - 1].arrivalTime - b.front().departureTime) / 60.0;
  }
  return span;
}

inline SharedSpan sharedSuffix(const RAPTOR::Journey &a,
                               const RAPTOR::Journey &b) noexcept {
  const std::vector<RouteLegInfo> infoA = routeLegInfos(a);
  const std::vector<RouteLegInfo> infoB = routeLegInfos(b);
  SharedSpan span;
  while (span.legs < infoA.size() && span.legs < infoB.size() &&
         infoA[infoA.size() - 1 - span.legs].routeId ==
             infoB[infoB.size() - 1 - span.legs].routeId) {
    span.legs++;
  }
  if (span.legs > 0 && !a.empty() && !b.empty()) {
    const int startA = infoA[infoA.size() - span.legs].departureTime;
    const int startB = infoB[infoB.size() - span.legs].departureTime;
    span.minutesA = (a.back().arrivalTime - startA) / 60.0;
    span.minutesB = (b.back().arrivalTime - startB) / 60.0;
  }
  return span;
}

struct TimeSpread {
  int minDeparture = std::numeric_limits<int>::max();
  int maxDeparture = std::numeric_limits<int>::min();
  int minArrival = std::numeric_limits<int>::max();
  int maxArrival = std::numeric_limits<int>::min();

  inline int departureSpread() const noexcept {
    return (minDeparture > maxDeparture) ? 0 : maxDeparture - minDeparture;
  }
  inline int arrivalSpread() const noexcept {
    return (minArrival > maxArrival) ? 0 : maxArrival - minArrival;
  }
};

inline TimeSpread
computeTimeSpread(const std::vector<RAPTOR::Journey> &journeys,
                  const std::vector<size_t> &indices) noexcept {
  TimeSpread spread;
  for (const size_t i : indices) {
    if (journeys[i].empty())
      continue;
    const int dep = firstScheduledDeparture(journeys[i]);
    const int arr = journeys[i].back().arrivalTime;
    spread.minDeparture = std::min(spread.minDeparture, dep);
    spread.maxDeparture = std::max(spread.maxDeparture, dep);
    spread.minArrival = std::min(spread.minArrival, arr);
    spread.maxArrival = std::max(spread.maxArrival, arr);
  }
  return spread;
}

inline std::set<StopId> transferStops(const RAPTOR::Journey &journey) noexcept {
  std::set<StopId> stops;
  for (const auto &leg : journey) {
    if (!leg.usesRoute && leg.from != leg.to) {
      stops.insert(StopId(leg.from));
    }
  }
  return stops;
}

inline std::set<StopId>
allTransferStops(const std::vector<RAPTOR::Journey> &journeys,
                 const std::vector<size_t> &indices) noexcept {
  std::set<StopId> stops;
  for (const size_t i : indices) {
    const std::set<StopId> s = transferStops(journeys[i]);
    stops.insert(s.begin(), s.end());
  }
  return stops;
}

struct DiversityDiagnostics {
  double avgPairwiseJaccard = 0.0;
  size_t maxSharedPrefixLegs = 0;
  double maxSharedPrefixMinutes = 0.0;
  size_t maxSharedSuffixLegs = 0;
  double maxSharedSuffixMinutes = 0.0;
  int departureSpreadSeconds = 0;
  int arrivalSpreadSeconds = 0;
  size_t distinctTransferStops = 0;
};

inline DiversityDiagnostics
computeDiversityDiagnostics(const std::vector<RAPTOR::Journey> &journeys,
                            const std::vector<size_t> &keptIndices) noexcept {
  DiversityDiagnostics diag;
  const TimeSpread spread = computeTimeSpread(journeys, keptIndices);
  diag.departureSpreadSeconds = spread.departureSpread();
  diag.arrivalSpreadSeconds = spread.arrivalSpread();
  diag.distinctTransferStops = allTransferStops(journeys, keptIndices).size();

  if (keptIndices.size() < 2)
    return diag;

  double jaccardSum = 0.0;
  size_t pairCount = 0;
  for (size_t a = 0; a < keptIndices.size(); a++) {
    for (size_t b = a + 1; b < keptIndices.size(); b++) {
      const RAPTOR::Journey &ja = journeys[keptIndices[a]];
      const RAPTOR::Journey &jb = journeys[keptIndices[b]];
      jaccardSum += jaccardSimilarity(routeSignature(ja), routeSignature(jb));
      pairCount++;

      const SharedSpan prefix = sharedPrefix(ja, jb);
      const SharedSpan suffix = sharedSuffix(ja, jb);
      diag.maxSharedPrefixLegs =
          std::max(diag.maxSharedPrefixLegs, prefix.legs);
      diag.maxSharedPrefixMinutes =
          std::max(diag.maxSharedPrefixMinutes,
                   std::max(prefix.minutesA, prefix.minutesB));
      diag.maxSharedSuffixLegs =
          std::max(diag.maxSharedSuffixLegs, suffix.legs);
      diag.maxSharedSuffixMinutes =
          std::max(diag.maxSharedSuffixMinutes,
                   std::max(suffix.minutesA, suffix.minutesB));
    }
  }
  diag.avgPairwiseJaccard = pairCount > 0 ? jaccardSum / pairCount : 0.0;
  return diag;
}

class DiversityThresholdSweep {
public:
  explicit DiversityThresholdSweep(std::vector<double> thresholds = {1.0, 0.8,
                                                                     0.6, 0.4})
      : thresholds(std::move(thresholds)), totals(this->thresholds.size(), 0),
        numQueries(0) {}

  inline void addQuery(const std::vector<RAPTOR::Journey> &journeys) noexcept {
    for (size_t t = 0; t < thresholds.size(); t++) {
      totals[t] += selectDiverseJourneys(journeys, thresholds[t]).size();
    }
    numQueries++;
  }

  inline void print(std::ostream &os = std::cout) const noexcept {
    os << "Diversity threshold sweep (avg. surviving journeys per query):\n";
    for (size_t t = 0; t < thresholds.size(); t++) {
      const double avg = numQueries == 0 ? 0.0
                                         : static_cast<double>(totals[t]) /
                                               static_cast<double>(numQueries);
      os << "  threshold " << thresholds[t] << " -> avg " << avg << "\n";
    }
  }

private:
  std::vector<double> thresholds;
  std::vector<size_t> totals;
  size_t numQueries;
};
