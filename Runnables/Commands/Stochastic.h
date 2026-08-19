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

#include "StochasticHelper.h"

using namespace Shell;

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
    printTripTransferStats(data, trip);
  }
};

class RunMcProbabilityQueries : public ParameterizedCommand {

public:
  RunMcProbabilityQueries(BasicShell &shell)
      : ParameterizedCommand(shell, "runMcProbabilityQueries",
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

    // new: diagnostics for (4)
    std::vector<double> avgPairwiseJaccardPerQuery;
    std::vector<size_t> maxSharedPrefixLegsPerQuery;
    std::vector<double> maxSharedPrefixMinutesPerQuery;
    std::vector<size_t> maxSharedSuffixLegsPerQuery;
    std::vector<double> maxSharedSuffixMinutesPerQuery;
    std::vector<int> departureSpreadPerQuery;
    std::vector<int> arrivalSpreadPerQuery;
    std::vector<size_t> distinctTransferStopsPerQuery;
    DiversityThresholdSweep sweep;

    const std::vector<StopQuery> queries =
        generateRandomStopQueries(data.numberOfStops(), numQueries);

    for (const StopQuery &query : queries) {
      algo.run(query.source, query.departureTime, query.target);
      const auto journeys = algo.getJourneys();
      const std::vector<size_t> diverseIndices =
          selectDiverseJourneys(journeys, similarityThreshold);

      numJourneys += journeys.size();
      numDiverseJourneys += diverseIndices.size();
      journeyCounts.push_back(journeys.size());
      diverseJourneyCounts.push_back(diverseIndices.size());

      // new: skip queries with < 2 kept journeys, nothing pairwise to say
      if (!journeys.empty()) {
        const DiversityDiagnostics diag =
            computeDiversityDiagnostics(journeys, diverseIndices);
        if (diverseIndices.size() >= 2) {
          avgPairwiseJaccardPerQuery.push_back(diag.avgPairwiseJaccard);
          maxSharedPrefixLegsPerQuery.push_back(diag.maxSharedPrefixLegs);
          maxSharedPrefixMinutesPerQuery.push_back(diag.maxSharedPrefixMinutes);
          maxSharedSuffixLegsPerQuery.push_back(diag.maxSharedSuffixLegs);
          maxSharedSuffixMinutesPerQuery.push_back(diag.maxSharedSuffixMinutes);
        }
        departureSpreadPerQuery.push_back(diag.departureSpreadSeconds);
        arrivalSpreadPerQuery.push_back(diag.arrivalSpreadSeconds);
        distinctTransferStopsPerQuery.push_back(diag.distinctTransferStops);
        sweep.addQuery(journeys);
      }
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

    // new: print diagnostics
    std::cout << "\n--- Diversity diagnostics (kept/diverse journeys) ---\n";
    printDistribution("Avg. pairwise route-Jaccard (queries w/ >=2 kept)",
                      avgPairwiseJaccardPerQuery);
    printDistribution("Max shared-prefix legs (queries w/ >=2 kept)",
                      maxSharedPrefixLegsPerQuery);
    printDistribution("Max shared-prefix minutes (queries w/ >=2 kept)",
                      maxSharedPrefixMinutesPerQuery);
    printDistribution("Max shared-suffix legs (queries w/ >=2 kept)",
                      maxSharedSuffixLegsPerQuery);
    printDistribution("Max shared-suffix minutes (queries w/ >=2 kept)",
                      maxSharedSuffixMinutesPerQuery);
    printDistribution("Departure time spread (s)", departureSpreadPerQuery);
    printDistribution("Arrival time spread (s)", arrivalSpreadPerQuery);
    printDistribution("Distinct transfer stops used",
                      distinctTransferStopsPerQuery);
    std::cout << std::endl;
    sweep.print();
  }

private:
  template <typename T>
  inline void printDistribution(const std::string &label,
                                std::vector<T> values) const noexcept {
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

class RunBoundedMcProbabilityQueries : public ParameterizedCommand {

public:
  RunBoundedMcProbabilityQueries(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runBoundedMcProbabilityQueries",
            "Runs the given number of random Bounded Mc Prob-TB queries.") {
    addParameter("Trip-Based input file");
    addParameter("Bounded forward Trip-Based input file");
    addParameter("Bounded backward Trip-Based input file");
    addParameter("Number of queries");
    addParameter("Arrival slack");
    addParameter("Trip slack");
    addParameter("Min Probability [%]", "0");
    addParameter("Softness", "0.5");
    addParameter("Max Margin", "2.0");
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
    const size_t n = getParameter<size_t>("Number of queries");

    const size_t pMin = getParameter<double>("Min Probability [%]");
    const size_t softness = getParameter<double>("Softness");
    const size_t maxMargin = getParameter<double>("Max Margin");
    algo.setMinProbability(pMin, softness, maxMargin);

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

// RUN ONE BOUNDED Mc PROBABILITY QUERY
class RunBoundedMcProbabilityQuery : public ParameterizedCommand {

public:
  RunBoundedMcProbabilityQuery(BasicShell &shell)
      : ParameterizedCommand(
            shell, "runBoundedMcProbabilityQuery",
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
    addParameter("Min Probability [%]", "0");
    addParameter("Softness", "0.5");
    addParameter("Max Margin", "2.0");
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

    TripBased::Data tripBasedData(inputFile);
    tripBasedData.printInfo();
    TripBased::Data forwardBoundedData(forwardBoundedFile);
    forwardBoundedData.printInfo();
    TripBased::Data backwardBoundedData(backwardBoundedFile);
    backwardBoundedData.printInfo();

    TripBased::BoundedMcProbabilityQuery<TripBased::AggregateProfiler> algo(
        tripBasedData, forwardBoundedData, backwardBoundedData);

    const size_t pMin = getParameter<double>("Min Probability [%]");
    const size_t softness = getParameter<double>("Softness");
    const size_t maxMargin = getParameter<double>("Max Margin");
    algo.setMinProbability(pMin, softness, maxMargin);

    algo.run(source, departureTime, target, arrivalSlack, tripSlack);

    algo.getProfiler().printStatistics();
    const auto journeys = algo.getJourneys();
    const auto paretoFront = algo.getResults();

    std::cout << "Found " << journeys.size() << " Pareto-optimal journeys\n";
    for (size_t i = 0; i < journeys.size(); ++i) {
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
class RunMcProbabilityQuery : public ParameterizedCommand {

public:
  RunMcProbabilityQuery(BasicShell &shell)
      : ParameterizedCommand(shell, "runMcProbabilityQuery",
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
        selectDiverseJourneys(journeys, similarityThreshold);

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
