#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {
class McPropabilityQuery {

public:
  static constexpr double InitialProbability = 1.0;
  static constexpr double FootpathProbability = 1.0;

  struct JourneyLeg {
    JourneyLeg(const StopId from, const StopId to, const int departureTime,
               const int arrivalTime, const bool usesTrip,
               const TripId trip = noTripId)
        : from(from), to(to), departureTime(departureTime),
          arrivalTime(arrivalTime), usesTrip(usesTrip), trip(trip) {}

    StopId from;
    StopId to;
    int departureTime;
    int arrivalTime;
    bool usesTrip;
    TripId trip;
  };
  using Journey = std::vector<JourneyLeg>;

  struct Result {
    int arrivalTime;
    int numberOfTrips;
    double probability;
    Journey journey;
  };

private:
  struct ArrivalLabel {
    int arrivalTime{INFTY};
    int numberOfTrips{INFTY};
    double probability{0.0};

    bool isSource{false};

    TripId trip{noTripId};
    StopIndex tripIndex{noStopIndex};
    size_t boardLabelIndex{invalid};

    StopId footpathFrom{noStop};
    size_t footpathFromLabelIndex{invalid};

    static constexpr size_t invalid = std::numeric_limits<size_t>::max();

    inline bool dominates(const ArrivalLabel &other) const noexcept {
      return arrivalTime <= other.arrivalTime &&
             numberOfTrips <= other.numberOfTrips &&
             probability >= other.probability;
    }
  };

  struct BoardLabel {
    StopIndex boardIndex{noStopIndex};
    int numberOfTrips{INFTY};
    double probability{0.0};

    StopId fromStop{noStop};
    size_t fromArrivalLabelIndex{ArrivalLabel::invalid};

    inline bool dominates(const BoardLabel &other) const noexcept {
      return boardIndex <= other.boardIndex &&
             numberOfTrips <= other.numberOfTrips &&
             probability >= other.probability;
    }
  };

  template <typename LABEL> class ParetoBag {
  public:
    inline bool merge(const LABEL &label) noexcept {
      for (const LABEL &existing : labels) {
        if (existing.dominates(label))
          return false;
      }
      labels.emplace_back(label);
      return true;
    }

    inline size_t size() const noexcept { return labels.size(); }
    inline bool empty() const noexcept { return labels.empty(); }
    inline const LABEL &operator[](const size_t i) const noexcept {
      return labels[i];
    }
    inline void clear() noexcept { labels.clear(); }

    std::vector<LABEL> labels;
  };

public:
  McPropabilityQuery(const Data &data, const size_t maxRounds = 16)
      : data(data), maxRounds(maxRounds), sourceStop(noStop),
        targetStop(noStop), sourceDepartureTime(never) {}

  inline void run(const StopId source, const int departureTime,
                  const StopId target) noexcept {
    clear();
    sourceStop = source;
    targetStop = target;
    sourceDepartureTime = departureTime;

    initialize();

    for (size_t round = 0; round < maxRounds; round++) {
      if (boardingQueue.empty())
        break;
      scanRound();
    }
  }

  inline std::vector<Result> getResults() const noexcept {
    std::vector<Result> results;
    if (!data.isStop(targetStop))
      return results;
    const ParetoBag<ArrivalLabel> &targetLabels = arrivalLabels[targetStop];
    for (size_t i = 0; i < targetLabels.size(); i++) {
      const ArrivalLabel &label = targetLabels[i];
      if (label.isSource)
        continue;
      bool dominated = false;
      for (size_t j = 0; j < targetLabels.size(); j++) {
        if (i == j || targetLabels[j].isSource)
          continue;
        const ArrivalLabel &other = targetLabels[j];
        const bool otherDominates = other.dominates(label);
        const bool mutuallyEqual = otherDominates && label.dominates(other);
        if (otherDominates && !(mutuallyEqual && j > i)) {
          dominated = true;
          break;
        }
      }
      if (dominated)
        continue;
      results.emplace_back(Result{label.arrivalTime, label.numberOfTrips,
                                  label.probability,
                                  buildJourney(targetStop, i)});
    }
    return results;
  }

private:
  inline void clear() noexcept {
    arrivalLabels.assign(data.numberOfStops(), ParetoBag<ArrivalLabel>());
    boardingLabels.assign(data.numberOfTrips(), ParetoBag<BoardLabel>());
    boardingQueue.clear();
    nextBoardingQueue.clear();
    queuedThisRound.assign(data.numberOfTrips(), false);
  }

  inline double edgeProbability(const Edge edge) const noexcept {
    return data.stopEventGraph.get(Propability, edge);
  }

  inline void initialize() noexcept {
    addArrivalLabel(sourceStop, ArrivalLabel{sourceDepartureTime, 0,
                                             InitialProbability, true, noTripId,
                                             noStopIndex, ArrivalLabel::invalid,
                                             noStop, ArrivalLabel::invalid});

    boardInitialTrips(sourceStop);
    for (const Edge edge : data.getTransferGraph().edgesFrom(sourceStop)) {
      const StopId toStop = StopId(data.getTransferGraph().get(ToVertex, edge));
      if (!data.isStop(toStop))
        continue;
      const int travelTime = data.getTransferGraph().get(TravelTime, edge);
      addArrivalLabel(toStop,
                      makeFootpathLabel(sourceStop, 0, toStop,
                                        sourceDepartureTime + travelTime));
      boardInitialTrips(toStop);
    }

    boardingQueue.swap(nextBoardingQueue);
    nextBoardingQueue.clear();
    std::fill(queuedThisRound.begin(), queuedThisRound.end(), false);
  }

  inline void boardInitialTrips(const StopId stop) noexcept {
    const ParetoBag<ArrivalLabel> &stopLabels = arrivalLabels[stop];
    for (size_t i = 0; i < stopLabels.size(); i++) {
      const ArrivalLabel &label = stopLabels[i];
      for (const RAPTOR::RouteSegment &route :
           data.routesContainingStop(stop)) {
        if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId))
          continue;
        const TripId trip = data.getEarliestTrip(route, label.arrivalTime);
        if (trip == noTripId)
          continue;
        BoardLabel boardLabel;
        boardLabel.boardIndex = route.stopIndex;
        boardLabel.numberOfTrips = label.numberOfTrips + 1;
        boardLabel.probability = label.probability;
        boardLabel.fromStop = stop;
        boardLabel.fromArrivalLabelIndex = i;
        enqueueBoardLabel(trip, boardLabel);
      }
    }
  }

  inline ArrivalLabel makeFootpathLabel(const StopId fromStop,
                                        const size_t fromLabelIndex,
                                        const StopId /*toStop*/,
                                        const int arrivalTime) const noexcept {
    const ArrivalLabel &from = arrivalLabels[fromStop][fromLabelIndex];
    ArrivalLabel label;
    label.arrivalTime = arrivalTime;
    label.numberOfTrips = from.numberOfTrips;
    label.probability = from.probability * FootpathProbability;
    label.footpathFrom = fromStop;
    label.footpathFromLabelIndex = fromLabelIndex;
    return label;
  }

  inline void addArrivalLabel(const StopId stop,
                              const ArrivalLabel &label) noexcept {
    arrivalLabels[stop].merge(label);
  }

  inline void enqueueBoardLabel(const TripId trip,
                                const BoardLabel &label) noexcept {
    if (!boardingLabels[trip].merge(label))
      return;
    if (!queuedThisRound[trip]) {
      queuedThisRound[trip] = true;
      nextBoardingQueue.push_back(trip);
    }
  }

  inline void scanRound() noexcept {
    nextBoardingQueue.clear();
    std::fill(queuedThisRound.begin(), queuedThisRound.end(), false);

    for (const TripId trip : boardingQueue) {
      const ParetoBag<BoardLabel> &tripLabels = boardingLabels[trip];
      const size_t tripLength = data.numberOfStopsInTrip(trip);
      for (size_t li = 0; li < tripLabels.size(); li++) {
        const BoardLabel &boardLabel = tripLabels[li];
        for (StopIndex index = StopIndex(boardLabel.boardIndex + 1);
             index < tripLength; index++) {
          const RAPTOR::StopEvent &stopEvent = data.getStopEvent(trip, index);
          const StopId stop = data.getStop(trip, index);

          ArrivalLabel arrival;
          arrival.arrivalTime = stopEvent.arrivalTime;
          arrival.numberOfTrips = boardLabel.numberOfTrips;
          arrival.probability = boardLabel.probability;
          arrival.trip = trip;
          arrival.tripIndex = index;
          arrival.boardLabelIndex = li;
          const bool improved = arrivalLabels[stop].merge(arrival);
          const size_t arrivalLabelIndex = arrivalLabels[stop].size() - 1;
          if (!improved)
            continue;

          if (stop != targetStop) {
            for (const Edge edge : data.getTransferGraph().edgesFrom(stop)) {
              const StopId toStop =
                  StopId(data.getTransferGraph().get(ToVertex, edge));
              if (toStop != targetStop)
                continue;
              const int travelTime =
                  data.getTransferGraph().get(TravelTime, edge);
              addArrivalLabel(
                  targetStop,
                  makeFootpathLabel(stop, arrivalLabelIndex, targetStop,
                                    arrival.arrivalTime + travelTime));
            }
          }

          const StopEventId event = data.getStopEventId(trip, index);
          for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(event))) {
            const StopEventId toEvent =
                StopEventId(data.stopEventGraph.get(ToVertex, edge));
            const TripId toTrip = data.tripOfStopEvent[toEvent];
            const StopIndex toIndex = data.indexOfStopEvent[toEvent];

            BoardLabel newBoardLabel;
            newBoardLabel.boardIndex = toIndex;
            newBoardLabel.numberOfTrips = boardLabel.numberOfTrips + 1;
            newBoardLabel.probability =
                boardLabel.probability * edgeProbability(edge);
            newBoardLabel.fromStop = stop;
            newBoardLabel.fromArrivalLabelIndex = arrivalLabelIndex;
            enqueueBoardLabel(toTrip, newBoardLabel);
          }
        }
      }
    }

    boardingQueue.swap(nextBoardingQueue);
  }

  inline Journey buildJourney(StopId stop, size_t labelIndex) const noexcept {
    Journey journey;
    while (true) {
      const ArrivalLabel &label = arrivalLabels[stop][labelIndex];
      if (label.isSource)
        break;
      if (label.footpathFrom != noStop) {
        const StopId fromStop = label.footpathFrom;
        const ArrivalLabel &fromLabel =
            arrivalLabels[fromStop][label.footpathFromLabelIndex];
        journey.emplace_back(fromStop, stop, fromLabel.arrivalTime,
                             label.arrivalTime, false);
        stop = fromStop;
        labelIndex = label.footpathFromLabelIndex;
      } else {
        const BoardLabel &boardLabel =
            boardingLabels[label.trip][label.boardLabelIndex];
        const StopId boardStop =
            data.getStop(label.trip, boardLabel.boardIndex);
        const int departureTime =
            data.getStopEvent(label.trip, boardLabel.boardIndex).departureTime;
        journey.emplace_back(boardStop, stop, departureTime, label.arrivalTime,
                             true, label.trip);
        stop = boardLabel.fromStop;
        labelIndex = boardLabel.fromArrivalLabelIndex;
      }
    }
    std::reverse(journey.begin(), journey.end());
    return journey;
  }

private:
  const Data &data;
  const size_t maxRounds;

  StopId sourceStop;
  StopId targetStop;
  int sourceDepartureTime;

  std::vector<ParetoBag<ArrivalLabel>> arrivalLabels; // indexed by StopId
  std::vector<ParetoBag<BoardLabel>> boardingLabels;  // indexed by TripId

  std::vector<TripId> boardingQueue;
  std::vector<TripId> nextBoardingQueue;
  std::vector<bool> queuedThisRound;
};

} // namespace TripBased
