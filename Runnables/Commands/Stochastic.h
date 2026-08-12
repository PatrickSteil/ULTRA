#pragma once

#include <iostream>
#include <string>

#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilderStochastic.h"
#include "../../Algorithms/TripBased/Query/McPropabilityQuery.h"

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
      const double p =
          transferFeasibilityProbability(arrival, departure, rho, walkTime);

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

class RunMCPropabilityQueries : public ParameterizedCommand {

public:
  RunMCPropabilityQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runMCPropabilityQueries",
                             "Runs random Multi-Criteria Trip-Based queries "
                             "maximizing arrival probability, "
                             "with arrival time and number of trips as the "
                             "other two criteria.") {
    addParameter("Trip-Based input file");
    addParameter("Number of queries");
    addParameter("Seed", "42");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Trip-Based input file");
    const size_t numQueries = getParameter<size_t>("Number of queries");
    const size_t seed = getParameter<size_t>("Seed");

    TripBased::Data data(inputFile);
    data.printInfo();

    std::mt19937 randomGenerator(seed);
    std::uniform_int_distribution<> stopDistribution(0,
                                                     data.numberOfStops() - 1);
    std::uniform_int_distribution<> timeDistribution(0, 24 * 60 * 60);

    TripBased::McPropabilityQuery<TripBased::AggregateProfiler> query(data);

    double totalTime = 0.0;
    size_t totalResults = 0;
    size_t queriesWithNoResult = 0;

    for (size_t i = 0; i < numQueries; i++) {
      const StopId source = StopId(stopDistribution(randomGenerator));
      const StopId target = StopId(stopDistribution(randomGenerator));
      const int departureTime = timeDistribution(randomGenerator);
      if (source == target)
        continue;

      Timer timer;
      query.run(source, departureTime, target);
      totalTime += timer.elapsedMicroseconds();

      const std::vector<RAPTOR::ProbabilityParetoLabel> results =
          query.getResults();
      totalResults += results.size();
      if (results.empty())
        queriesWithNoResult++;
    }

    std::cout << "Ran " << String::prettyInt(numQueries) << " queries."
              << std::endl;
    std::cout << "Average running time:      "
              << String::prettyDouble(totalTime / numQueries) << " microseconds"
              << std::endl;
    std::cout << "Average result-set size:   "
              << String::prettyDouble((double)totalResults / numQueries)
              << std::endl;
    std::cout << "Queries without a result:  "
              << String::prettyInt(queriesWithNoResult) << std::endl;
  }
};

class RunMCPropabilityQuery : public ParameterizedCommand {

public:
  RunMCPropabilityQuery(BasicShell &shell)
      : ParameterizedCommand(shell, "runMCPropabilityQuery",
                             "Runs a single Multi-Criteria Trip-Based query "
                             "maximizing arrival probability.") {
    addParameter("Trip-Based input file");
    addParameter("Source stop");
    addParameter("Target stop");
    addParameter("Departure time");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Trip-Based input file");
    const StopId source = StopId(getParameter<size_t>("Source stop"));
    const StopId target = StopId(getParameter<size_t>("Target stop"));
    const int departureTime = getParameter<int>("Departure time");

    TripBased::Data data(inputFile);

    TripBased::McPropabilityQuery<TripBased::AggregateProfiler> query(data);
    Timer timer;
    query.run(source, departureTime, target);
    const double runningTime = timer.elapsedMicroseconds();

    const std::vector<RAPTOR::ProbabilityParetoLabel> results =
        query.getResults();
    std::cout << "Running time: " << String::prettyDouble(runningTime)
              << " microseconds" << std::endl;
    std::cout << "Found " << results.size()
              << " Pareto-optimal journeys:" << std::endl;
    for (const auto &result : results) {
      std::cout << result << std::endl;
    }
  }
};
