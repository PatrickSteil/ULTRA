#include "Commands/NetworkIO.h"
#include "Commands/NetworkTools.h"
#include "Commands/Stochastic.h"

#include "../Helpers/Console/CommandLineParser.h"
#include "../Helpers/MultiThreading.h"

#include "../Shell/Shell.h"
using namespace Shell;

int main(int argc, char **argv) {
  CommandLineParser clp(argc, argv);
  pinThreadToCoreId(clp.value<int>("core", 1));
  checkAsserts();
  ::Shell::Shell shell;

  new IntermediateToRAPTOR(shell);
  new IntermediateToRAPTORRandomDelay(shell);

  new AugmentProbabilityTripBasedShortcuts(shell);
  // new ReverseTripBasedData(shell);

  new StochasticStopEventGraphBuilder(shell);
  new PrintTripTransferStats(shell);

  // new RunTransitiveTBQueries(shell);
  new RunMcProbabilityQueries(shell);
  new RunMcProbabilityQuery(shell);

  new RunBoundedMcProbabilityQueries(shell);
  new RunBoundedMcProbabilityQuery(shell);

  shell.run();
  return 0;
}
