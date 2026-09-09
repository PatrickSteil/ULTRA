#pragma once
#include "../../Helpers/Assert.h"
#include "../../Helpers/Types.h"
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>
namespace HL {
template <typename ChainType = std::uint32_t,
          typename PositionType = std::uint32_t>
struct ChainHub {
  ChainType chainId;
  PositionType position;
  inline ChainType getChain() const noexcept {
    return static_cast<ChainType>(chainId);
  }
  inline PositionType getIndex() const noexcept {
    return static_cast<PositionType>(position);
  }
  inline bool operator==(const ChainHub &other) const noexcept {
    return std::tie(chainId, position) ==
           std::tie(other.chainId, other.position);
  }
  inline bool operator<(const ChainHub &other) const noexcept {
    return std::tie(chainId, position) <
           std::tie(other.chainId, other.position);
  }
  friend std::ostream &operator<<(std::ostream &out,
                                  const ChainHub &e) noexcept {
    return out << "(" << e.chainId << ", " << e.position << ")";
  }
  ChainHub(const ChainType cId = ChainType(),
           const PositionType pos = PositionType())
      : chainId(cId), position(pos) {}
};

template <typename ChainType = std::uint32_t,
          typename PositionType = std::uint32_t>
struct ChainLabel {
  using Entry = ChainHub<ChainType, PositionType>;
  std::vector<Entry> entries;
  inline std::size_t size() const noexcept { return entries.size(); }
  inline bool empty() const noexcept { return entries.empty(); }
  inline void clear() noexcept { entries.clear(); }
  inline void sort() noexcept { std::sort(entries.begin(), entries.end()); }

  template <bool FWD = true>
  inline void insert(const ChainType cId, const PositionType pos) {
    auto it = std::lower_bound(
        entries.begin(), entries.end(), cId,
        [](const Entry &e, const ChainType &c) { return e.getChain() < c; });

    if (it != entries.end() && it->getChain() == cId) {
      if constexpr (FWD) {
        if (pos < it->getIndex())
          it->position = pos;
      } else {
        if (pos > it->getIndex())
          it->position = pos;
      }
    } else {
      entries.insert(it, Entry(cId, pos));
    }
  }
};

template <typename LabelType>
inline bool reachable(const LabelType &forwardLabel,
                      const LabelType &backwardLabel) noexcept {
  using Entry = typename LabelType::Entry;
  const std::vector<Entry> &F = forwardLabel.entries;
  const std::vector<Entry> &B = backwardLabel.entries;
  std::size_t f = 0;
  std::size_t b = 0;
  while (f < F.size() && b < B.size()) {
    if (F[f].getChain() == B[b].getChain()) {
      if (F[f].getIndex() <= B[b].getIndex())
        return true;
      f++;
      b++;
      continue;
    }
    if (F[f].getChain() < B[b].getChain())
      f++;
    else
      b++;
  }
  return false;
}
} // namespace HL
