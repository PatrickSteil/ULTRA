#pragma once

#include <random>
#include <string>
#include <vector>

#include "../../Algorithms/TripBased/Query/MCPropabilityQuery.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../Helpers/Console/CommandLineParser.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Timer.h"

#include "../../Shell/Shell.h"

namespace Shell {

class RunMCPropabilityQueries : public Command {

public:
  RunMCPropabilityQueries(BasicShell &shell)
      : Command(shell, "runMCPropabilityQueries",
                "Runs random Multi-Criteria Trip-Based queries maximizing "
                "arrival probability, "
                "with arrival time and number of trips as the other two "
                "criteria.") {
    addParameter("Trip-Based input file");
    addParameter("Number of queries");
    addParameter("Max rounds", "16");
    addParameter("Seed", "42");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Trip-Based input file");
    const size_t numQueries = getParameter<size_t>("Number of queries");
    const size_t maxRounds = getParameter<size_t>("Max rounds");
    const size_t seed = getParameter<size_t>("Seed");

    TripBased::Data data(inputFile);
    data.printInfo();

    std::mt19937 randomGenerator(seed);
    std::uniform_int_distribution<> stopDistribution(0,
                                                     data.numberOfStops() - 1);
    std::uniform_int_distribution<> timeDistribution(0, 24 * 60 * 60);

    TripBased::MCPropabilityQuery query(data, maxRounds);

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

      const std::vector<TripBased::MCPropabilityQuery::Result> results =
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

class RunMCPropabilityQuery : public Command {

public:
  RunMCPropabilityQuery(BasicShell &shell)
      : Command(shell, "runMCPropabilityQuery",
                "Runs a single Multi-Criteria Trip-Based query maximizing "
                "arrival probability.") {
    addParameter("Trip-Based input file");
    addParameter("Source stop");
    addParameter("Target stop");
    addParameter("Departure time");
    addParameter("Max rounds", "16");
  }

  virtual void execute() noexcept {
    const std::string inputFile = getParameter("Trip-Based input file");
    const StopId source = StopId(getParameter<size_t>("Source stop"));
    const StopId target = StopId(getParameter<size_t>("Target stop"));
    const int departureTime = getParameter<int>("Departure time");
    const size_t maxRounds = getParameter<size_t>("Max rounds");

    TripBased::Data data(inputFile);

    TripBased::MCPropabilityQuery query(data, maxRounds);
    Timer timer;
    query.run(source, departureTime, target);
    const double runningTime = timer.elapsedMicroseconds();

    const std::vector<TripBased::MCPropabilityQuery::Result> results =
        query.getResults();
    std::cout << "Running time: " << String::prettyDouble(runningTime)
              << " microseconds" << std::endl;
    std::cout << "Found " << results.size()
              << " Pareto-optimal journeys:" << std::endl;
    for (const TripBased::MCPropabilityQuery::Result &result : results) {
      std::cout << "   Arrival time: " << String::secToTime(result.arrivalTime)
                << ", trips: " << result.numberOfTrips
                << ", probability: " << result.probability
                << ", legs: " << result.journey.size() << std::endl;
    }
  }
};

} // namespace Shell
