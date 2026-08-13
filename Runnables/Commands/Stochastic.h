#pragma once

#include <algorithm>
#include <iostream>
#include <set>
#include <string>

#include "../../Algorithms/TripBased/BoundedMcQuery/BoundedMcProbabilityQuery.h"
#include "../../Algorithms/TripBased/Preprocessing/ProbabilityShortcutAugmenter.h"
#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilderStochastic.h"
#include "../../Algorithms/TripBased/Query/McProbabilityQuery.h"

#include "../../DataStructures/Queries/Queries.h"

#include "../../Helpers/MultiThreading.h"
#include "../../Helpers/String/String.h"

#include "../../Shell/Shell.h"

using namespace Shell;

namespace TripBased {

inline void printTripTransferStats(const Data &data, const TripId trip,
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

      // Recover walk time: 0 if same stop, else look up the footpath edge.
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

// --- journey diversity ------------------------------------------------
// The ordered list of route ids a journey uses (transfer legs are
// skipped). Two journeys that "take the same routes, just later" end up
// with identical or near-identical signatures.
inline std::vector<RouteId>
routeSignature(const RAPTOR::Journey &journey) noexcept {
  std::vector<RouteId> signature;
  for (const auto &leg : journey) {
    if (leg.usesRoute)
      signature.push_back(RouteId(leg.routeId));
  }
  return signature;
}

// Set-based (order-agnostic) similarity: |A ∩ B| / |A ∪ B|. 1.0 means the
// two journeys use exactly the same set of routes; 0.0 means they share
// none. Two empty signatures (e.g. two direct-walk journeys) are treated
// as identical.
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

// Greedily keeps journeys in the given order, dropping any journey whose
// route signature is at least `similarityThreshold` similar (Jaccard) to
// one already kept.
//   similarityThreshold == 1.0  -> only exact-signature duplicates are
//                                  removed ("same lines, later departure").
//   similarityThreshold  < 1.0  -> also merges journeys that mostly, but
//                                  not entirely, overlap in which routes
//                                  they use.
// `journeys` should already be sorted by preference (e.g. by probability
// or arrival time), since the first journey encountered for a given
// "cluster" of similar signatures is the one that gets kept.
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
// ------------------------------------------------------------------------

} // namespace TripBased

class IntermediateToRAPTORRandomDelay : public ParameterizedCommand {

public:
  IntermediateToRAPTORRandomDelay(BasicShell &shell)
      : ParameterizedCommand(shell, "intermediateToRAPTORWithRandomDelay",
                             "Converts binary intermediate data to RAPTOR "
                             "network format and adds random delay.") {
    addParameter("Input file");
    addParameter("Output file");
    addParameter("Seed", "42");
    addParameter("Sigma Initial", "30.0");
    addParameter("Phi", "0.85");
    addParameter("Process Noise", "20.0");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file");
    const std::string outputFile = getParameter("Output file");
    const int seed = getParameter<int>("Seed");
    const double sigma_initial = getParameter<double>("Sigma Initial");
    const double phi = getParameter<double>("Phi");
    const double process_noise = getParameter<double>("Process Noise");

    Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
    inter.printInfo();
    RAPTOR::Data data = RAPTOR::Data::FromIntermediate(inter);
    data.printInfo();
    Graph::printInfo(data.transferGraph);
    data.transferGraph.printAnalysis();

    data.applySimpleDelayScenario(seed, sigma_initial, phi, process_noise);
    data.serialize(outputFile);
  }
};

class StochasticStopEventGraphBuilder : public ParameterizedCommand {

public:
  StochasticStopEventGraphBuilder(BasicShell &shell)
      : ParameterizedCommand(shell, "buildStochasticStopEventGraph",
                             "Computes a Stochastic StopEventGraph and saves "
                             "it in Trip-Based format.") {
    addParameter("Input file");
    addParameter("Output file");
    addParameter("Route-based pruning?");
    addParameter("Number of threads", "max");
    addParameter("Pin multiplier", "1");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file");
    const std::string outputFile = getParameter("Output file");
    const bool routeBasedPruning = getParameter<bool>("Route-based pruning?");
    const int numberOfThreads = getNumberOfThreads();
    const int pinMultiplier = getParameter<int>("Pin multiplier");

    RAPTOR::Data raptor(inputFile);
    raptor.printInfo();
    TripBased::Data data(raptor);

    if (numberOfThreads == 0) {
      if (routeBasedPruning) {
        TripBased::ComputeStochasticStopEventGraphRouteBased(data);
      } else {
        TripBased::ComputeStochasticStopEventGraph(data);
      }
    } else {
      if (routeBasedPruning) {
        TripBased::ComputeStochasticStopEventGraphRouteBased(
            data, numberOfThreads, pinMultiplier);
      } else {
        TripBased::ComputeStochasticStopEventGraph(data, numberOfThreads,
                                                   pinMultiplier);
      }
    }

    data.printInfo();
    data.serialize(outputFile);
  }

private:
  inline int getNumberOfThreads() const noexcept {
    if (getParameter("Number of threads") == "max") {
      return numberOfCores();
    } else {
      return getParameter<int>("Number of threads");
    }
  }
};

class PrintTripTransferStats : public ParameterizedCommand {
public:
  PrintTripTransferStats(BasicShell &shell)
      : ParameterizedCommand(shell, "printTripTransferStats",
                             "Prints outgoing transfers, feasibility "
                             "probabilities, and stats for a given trip.") {
    addParameter("Input file (TB)");
    addParameter("Trip Id");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Input file (TB)");
    const int tripIdValue = getParameter<int>("Trip Id");

    TripBased::Data data(inputFile);

    const TripId trip = TripId(tripIdValue);
    TripBased::printTripTransferStats(data, trip);
  }
};

class RunMCProbabilityQueries : public ParameterizedCommand {

public:
  RunMCProbabilityQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runMCProbabilityQueries",
                             "Runs random Multi-Criteria Trip-Based queries "
                             "maximizing arrival probability, "
                             "with arrival time and number of trips as the "
                             "other two criteria.") {
    addParameter("Trip-Based input file");
    addParameter("Number of queries", "500");
    addParameter("Seed", "42");
    addParameter("Min probability (%)", "0");
    addParameter("Route similarity threshold", "1.0");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Trip-Based input file");
    const size_t numQueries = getParameter<size_t>("Number of queries");
    const double minProbabilityPercent =
        getParameter<double>("Min probability (%)");
    const double similarityThreshold =
        getParameter<double>("Route similarity threshold");

    TripBased::Data data(inputFile);
    data.printInfo();

    TripBased::McProbabilityQuery<TripBased::AggregateProfiler> algo(data);
    if (minProbabilityPercent > 0.0)
      algo.setMinProbability(minProbabilityPercent / 100.0);

    std::size_t numJourneys = 0;
    std::size_t numDiverseJourneys = 0;
    std::vector<size_t> journeyCounts;
    std::vector<size_t> diverseJourneyCounts;
    journeyCounts.reserve(numQueries);
    diverseJourneyCounts.reserve(numQueries);

    const std::vector<StopQuery> queries =
        generateRandomStopQueries(data.numberOfStops(), numQueries);

    for (const StopQuery &query : queries) {
      algo.run(query.source, query.departureTime, query.target);
      const auto journeys = algo.getJourneys();
      const std::vector<size_t> diverseIndices =
          TripBased::selectDiverseJourneys(journeys, similarityThreshold);

      numJourneys += journeys.size();
      numDiverseJourneys += diverseIndices.size();
      journeyCounts.push_back(journeys.size());
      diverseJourneyCounts.push_back(diverseIndices.size());
    }

    algo.getProfiler().printStatistics();
    std::cout << "Avg. journeys:         "
              << String::prettyDouble(numJourneys /
                                      static_cast<double>(numQueries))
              << std::endl;
    std::cout << "Avg. diverse journeys: "
              << String::prettyDouble(numDiverseJourneys /
                                      static_cast<double>(numQueries))
              << " (route similarity threshold " << similarityThreshold << ")"
              << std::endl;
    if (numJourneys > 0) {
      std::cout << "Avg. diversity ratio:  "
                << String::prettyDouble(100.0 * numDiverseJourneys /
                                        static_cast<double>(numJourneys))
                << " % of journeys were route-distinct" << std::endl;
    }

    printDistribution("Journeys per query", journeyCounts);
    printDistribution("Diverse journeys per query", diverseJourneyCounts);
  }

private:
  inline void printDistribution(const std::string &label,
                                std::vector<size_t> values) const noexcept {
    if (values.empty())
      return;
    std::sort(values.begin(), values.end());
    auto percentile = [&](double q) {
      const size_t idx = static_cast<size_t>(q * (values.size() - 1));
      return values[idx];
    };
    std::cout << label << " -> min: " << values.front()
              << ", median: " << percentile(0.5) << ", p90: " << percentile(0.9)
              << ", max: " << values.back() << std::endl;
  }
};

class RunBoundedMCProbabilityQueries : public ParameterizedCommand {

public:
  RunBoundedMCProbabilityQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runBoundedMCProbabilityQueries",
            "Runs the given number of random Bounded Mc Prob-TB queries.") {
    addParameter("Trip-Based input file");
    addParameter("Bounded forward Trip-Based input file");
    addParameter("Bounded backward Trip-Based input file");
    addParameter("Number of queries");
    addParameter("Arrival slack");
    addParameter("Trip slack");
    addParameter("Min probability (%)", "0");
    addParameter("Sufficient probability (%)", "0");
  }

  virtual void execute() noexcept {
    TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
    tripBasedData.printInfo();
    TripBased::Data forwardBoundedData(
        getParameter("Bounded forward Trip-Based input file"));
    forwardBoundedData.printInfo();
    TripBased::Data backwardBoundedData(
        getParameter("Bounded backward Trip-Based input file"));
    backwardBoundedData.printInfo();
    TripBased::BoundedMcProbabilityQuery<TripBased::AggregateProfiler> algo(
        tripBasedData, forwardBoundedData, backwardBoundedData);

    const double arrivalSlack = getParameter<double>("Arrival slack");
    const double tripSlack = getParameter<double>("Trip slack");
    const double minProbabilityPercent =
        getParameter<double>("Min probability (%)");
    const double sufficientProbabilityPercent =
        getParameter<double>("Sufficient probability (%)");
    if (minProbabilityPercent > 0.0)
      algo.setMinProbability(minProbabilityPercent / 100.0);
    if (sufficientProbabilityPercent > 0.0)
      algo.setSufficientProbability(sufficientProbabilityPercent / 100.0);

    const size_t n = getParameter<size_t>("Number of queries");
    const std::vector<VertexQuery> queries =
        generateRandomVertexQueries(tripBasedData.numberOfStops(), n);

    double numJourneys = 0;
    for (const VertexQuery &query : queries) {
      algo.run(StopId(query.source), query.departureTime, StopId(query.target),
               arrivalSlack, tripSlack);
      numJourneys += algo.getJourneys().size();
    }
    algo.getProfiler().printStatistics();
    std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n)
              << std::endl;
  }
};

// RUN ONE BOUNDED MC PROBABILITY QUERY
class RunBoundedMCProbabilityQuery : public ParameterizedCommand {

public:
  RunBoundedMCProbabilityQuery(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runBoundedMCProbabilityQuery",
            "Runs a single Bounded Multi-Criteria Trip-Based query "
            "maximizing arrival probability, and prints the resulting "
            "journeys.") {
    addParameter("Trip-Based input file");
    addParameter("Bounded forward Trip-Based input file");
    addParameter("Bounded backward Trip-Based input file");
    addParameter("Source stop");
    addParameter("Target stop");
    addParameter("Departure time");
    addParameter("Arrival slack");
    addParameter("Trip slack");
    addParameter("Min probability (%)", "0");
    addParameter("Sufficient probability (%)", "0");
    addParameter("Route similarity threshold", "1.0");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Trip-Based input file");
    const std::string forwardBoundedFile =
        getParameter("Bounded forward Trip-Based input file");
    const std::string backwardBoundedFile =
        getParameter("Bounded backward Trip-Based input file");
    const StopId source = StopId(getParameter<size_t>("Source stop"));
    const StopId target = StopId(getParameter<size_t>("Target stop"));
    const int departureTime = getParameter<int>("Departure time");
    const double arrivalSlack = getParameter<double>("Arrival slack");
    const double tripSlack = getParameter<double>("Trip slack");
    const double minProbabilityPercent =
        getParameter<double>("Min probability (%)");
    const double sufficientProbabilityPercent =
        getParameter<double>("Sufficient probability (%)");
    const double similarityThreshold =
        getParameter<double>("Route similarity threshold");

    TripBased::Data tripBasedData(inputFile);
    tripBasedData.printInfo();
    TripBased::Data forwardBoundedData(forwardBoundedFile);
    forwardBoundedData.printInfo();
    TripBased::Data backwardBoundedData(backwardBoundedFile);
    backwardBoundedData.printInfo();

    TripBased::BoundedMcProbabilityQuery<TripBased::AggregateProfiler> algo(
        tripBasedData, forwardBoundedData, backwardBoundedData);
    if (minProbabilityPercent > 0.0)
      algo.setMinProbability(minProbabilityPercent / 100.0);
    if (sufficientProbabilityPercent > 0.0)
      algo.setSufficientProbability(sufficientProbabilityPercent / 100.0);

    algo.run(source, departureTime, target, arrivalSlack, tripSlack);

    algo.getProfiler().printStatistics();
    const auto journeys = algo.getJourneys();
    const auto paretoFront = algo.getResults();

    const std::vector<size_t> diverseIndices =
        TripBased::selectDiverseJourneys(journeys, similarityThreshold);

    std::cout << "Found " << journeys.size() << " Pareto-optimal journeys, "
              << diverseIndices.size()
              << " after route-diversity filtering (threshold "
              << similarityThreshold << "):" << std::endl;
    for (const size_t i : diverseIndices) {
      std::cout << "Journey: " << (int)i
                << ", ArrTime: " << (int)paretoFront[i].arrivalTime
                << ", Nr Trips: " << (int)paretoFront[i].numberOfTrips
                << ", Prob: " << (paretoFront[i].probability() * 100.0)
                << " %\n";
      const auto &j = journeys[i];
      for (const auto &leg : j) {
        std::cout << "from: " << leg.from << ", to: " << leg.to
                  << ", dep-Time: " << leg.departureTime
                  << ", arr-Time: " << leg.arrivalTime;
        if (leg.usesRoute) {
          std::cout << ", route: " << leg.routeId << "\n";
        } else {
          std::cout << ", transfer: " << leg.routeId << " ("
                    << (Edge(leg.routeId) != noEdge
                            ? (tripBasedData.stopEventGraph.get(
                                   Probability, Edge(leg.routeId)) *
                               100.0)
                            : 100.0)
                    << " %)\n";
        }
      }
      std::cout << std::endl;
    }
  }
};

// RUN ONE QUERY
class RunMCProbabilityQuery : public ParameterizedCommand {

public:
  RunMCProbabilityQuery(BasicShell &shell)
      : ParameterizedCommand(shell, "runMCProbabilityQuery",
                             "Runs a single Multi-Criteria Trip-Based query "
                             "maximizing arrival probability.") {
    addParameter("Trip-Based input file");
    addParameter("Source stop");
    addParameter("Target stop");
    addParameter("Departure time");
    addParameter("Min probability (%)", "0");
    addParameter("Route similarity threshold", "1.0");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Trip-Based input file");
    const StopId source = StopId(getParameter<size_t>("Source stop"));
    const StopId target = StopId(getParameter<size_t>("Target stop"));
    const int departureTime = getParameter<int>("Departure time");
    const double minProbabilityPercent =
        getParameter<double>("Min probability (%)");
    const double similarityThreshold =
        getParameter<double>("Route similarity threshold");

    TripBased::Data data(inputFile);

    TripBased::McProbabilityQuery<TripBased::AggregateProfiler> algo(data);
    if (minProbabilityPercent > 0.0)
      algo.setMinProbability(minProbabilityPercent / 100.0);

    algo.run(source, departureTime, target);

    algo.getProfiler().printStatistics();
    const auto journeys = algo.getJourneys();
    const auto paretoFront = algo.getResults();

    const std::vector<size_t> diverseIndices =
        TripBased::selectDiverseJourneys(journeys, similarityThreshold);

    std::cout << "Found " << journeys.size() << " Pareto-optimal journeys, "
              << diverseIndices.size()
              << " after route-diversity filtering (threshold "
              << similarityThreshold << "):" << std::endl;
    for (const size_t i : diverseIndices) {
      std::cout << "Journey: " << (int)i
                << ", ArrTime: " << (int)paretoFront[i].arrivalTime
                << ", Nr Trips: " << (int)paretoFront[i].numberOfTrips
                << ", Prob: " << (paretoFront[i].probability() * 100.0)
                << " %\n";
      const auto &j = journeys[i];
      for (const auto &leg : j) {
        std::cout << "from: " << leg.from << ", to: " << leg.to
                  << ", dep-Time: " << leg.departureTime
                  << ", arr-Time: " << leg.arrivalTime;
        if (leg.usesRoute) {
          std::cout << ", route: " << leg.routeId << "\n";
        } else {
          std::cout << ", transfer: " << leg.routeId << " ("
                    << (Edge(leg.routeId) != noEdge
                            ? (data.stopEventGraph.get(Probability,
                                                       Edge(leg.routeId)) *
                               100.0)
                            : 100.0)
                    << " %)\n";
        }
      }
      std::cout << std::endl;
    }
  }
};

class AugmentProbabilityTripBasedShortcuts : public ParameterizedCommand {

public:
  AugmentProbabilityTripBasedShortcuts(BasicShell &shell)
      : ParameterizedCommand(shell, "augmentProbabilityTripBasedShortcuts",
                             "Augments Probability Trip-Based shortcuts for "
                             "bounded multicriteria search.") {
    addParameter("Input file");
    addParameter("Forward output file");
    addParameter("Backward output file");
    addParameter("Remove superfluous shortcuts?");
    addParameter("Trip limit", "1073741823");
  }

  virtual void execute() noexcept {
    TripBased::Data data(getParameter("Input file"));
    data.printInfo();
    TripBased::Data reverseData = data.reverseNetwork();
    TripBased::ProbabilityShortcutAugmenter augmenter;
    const size_t tripLimit = getParameter<size_t>("Trip limit");
    augmenter.augmentShortcuts(data, tripLimit);
    augmenter.augmentShortcuts(reverseData, tripLimit);
    if (getParameter<bool>("Remove superfluous shortcuts?")) {
      augmenter.removeSuperfluousShortcuts(data);
      augmenter.removeSuperfluousShortcuts(reverseData);
    }
    data.serialize(getParameter("Forward output file"));
    reverseData.serialize(getParameter("Backward output file"));
  }
};
