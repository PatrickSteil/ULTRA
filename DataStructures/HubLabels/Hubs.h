#pragma once

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

namespace HL {

struct HubLabelReach {
  std::vector<Vertex> hubs;

  void insert(const Vertex h) {
    const auto it = std::lower_bound(hubs.begin(), hubs.end(), h);
    hubs.insert(it, h);
  }

  void add(const Vertex h) { hubs.emplace_back(h); }

  bool contains(const Vertex h) const {
    return std::find(hubs.begin(), hubs.end(), h) != hubs.end();
  }

  void sort() { std::sort(hubs.begin(), hubs.end()); }
};

template <typename DIST = std::uint32_t> struct HubLabel {
  std::vector<Vertex> hubs;
  std::vector<DIST> distance;

  void insert(const Vertex h, const DIST d) {
    const auto it = std::lower_bound(hubs.begin(), hubs.end(), h);
    const auto index = static_cast<std::size_t>(it - hubs.begin());

    hubs.insert(it, h);
    distance.insert(distance.begin() + index, d);
  }

  void add(const Vertex h, const DIST d) {
    hubs.emplace_back(h);
    distance.emplace_back(d);
  }

  bool contains(const Vertex h) const {
    return std::find(hubs.begin(), hubs.end(), h) != hubs.end();
  }

  void sort() {
    std::vector<std::pair<Vertex, DIST>> entries;
    entries.reserve(hubs.size());

    for (std::size_t i = 0; i < hubs.size(); ++i) {
      entries.emplace_back(hubs[i], distance[i]);
    }

    std::sort(entries.begin(), entries.end());

    for (std::size_t i = 0; i < entries.size(); ++i) {
      hubs[i] = entries[i].first;
      distance[i] = entries[i].second;
    }
  }
};

// TODO implement GroupLabel<GroupType, ChainType, IndexType>

} // namespace HL
