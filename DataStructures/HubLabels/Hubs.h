#pragma once

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "../../Helpers/Assert.h"
#include "../../Helpers/Types.h"

namespace HL {
template <typename GroupType, typename ChainType, typename IndexType>
struct GroupLabel {
  struct Entry {
    static constexpr int GroupBits = 25;
    static constexpr int ChainBits = 20;
    static constexpr int IndexBits = 19;
    static_assert(
        GroupBits + ChainBits + IndexBits == 64,
        "Section 8.1's packed layout must fill exactly one 64-bit word.");

    std::uint64_t group : GroupBits;
    std::uint64_t chain : ChainBits;
    std::uint64_t index : IndexBits;

    Entry(const GroupType group = GroupType(),
          const ChainType chain = ChainType(),
          const IndexType index = IndexType())
        : group(pack(group, GroupBits)),
          chain(pack(chain, ChainBits)),
          index(pack(index, IndexBits)) {}

    inline GroupType getGroup() const noexcept {
      return static_cast<GroupType>(group);
    }
    inline ChainType getChain() const noexcept {
      return static_cast<ChainType>(chain);
    }
    inline IndexType getIndex() const noexcept {
      return static_cast<IndexType>(index);
    }
    inline bool operator==(const Entry& other) const noexcept {
      return std::tie(group, chain, index) ==
             std::tie(other.group, other.chain, other.index);
    }

    inline bool operator<(const Entry& other) const noexcept {
      return std::tie(group, chain, index) <
             std::tie(other.group, other.chain, other.index);
    }

    friend std::ostream& operator<<(std::ostream& out,
                                    const Entry& e) noexcept {
      return out << "(" << e.group << ", " << e.chain << ", " << e.index << ")";
    }

   private:
    template <typename T>
    static inline std::uint64_t pack(const T value, const int bits) noexcept {
      const std::uint64_t raw = static_cast<std::uint64_t>(value);
      Assert(raw < (std::uint64_t(1) << bits),
             "Value " << raw << " does not fit into " << bits << " bits!");
      return raw;
    }
  };

  std::vector<Entry> entries;

  inline bool insertForward(const GroupType group, const ChainType chain,
                            const IndexType index,
                            const bool isNative = false) noexcept {
    return insert(group, chain, index, isNative, true);
  }

  inline bool insertBackward(const GroupType group, const ChainType chain,
                             const IndexType index,
                             const bool isNative = false) noexcept {
    return insert(group, chain, index, isNative, false);
  }

  inline std::vector<Entry> entriesOf(const GroupType group) const noexcept {
    const auto range = groupRange(group);
    return std::vector<Entry>(range.first, range.second);
  }

  inline bool contains(const GroupType group) const noexcept {
    const auto range = groupRange(group);
    return range.first != range.second;
  }

  inline std::size_t size() const noexcept { return entries.size(); }

  inline bool empty() const noexcept { return entries.empty(); }

  inline void clear() noexcept { entries.clear(); }

 private:
  inline std::pair<typename std::vector<Entry>::const_iterator,
                   typename std::vector<Entry>::const_iterator>
  groupRange(const GroupType group) const noexcept {
    return std::equal_range(
        entries.begin(), entries.end(), group,
        [](const auto& lhs, const auto& rhs) {
          if constexpr (std::is_same_v<std::decay_t<decltype(lhs)>,
                                       GroupType>) {
            return lhs < rhs.group;
          } else {
            return lhs.group < rhs;
          }
        });
  }

  inline bool insert(const GroupType group, const ChainType chain,
                     const IndexType index, const bool isNative,
                     const bool forward) noexcept {
    for (const Entry& e : entries) {
      if (e.getGroup() != group) continue;
      const bool dominatedByExisting =
          forward ? (e.getChain() <= chain && e.getIndex() <= index)
                  : (e.getChain() >= chain && e.getIndex() >= index);
      if (dominatedByExisting) return false;
    }

    entries.erase(std::remove_if(entries.begin(), entries.end(),
                                 [&](const Entry& e) {
                                   if (e.getGroup() != group) return false;
                                   return forward ? (chain <= e.getChain() &&
                                                     index <= e.getIndex())
                                                  : (chain >= e.getChain() &&
                                                     index >= e.getIndex());
                                 }),
                  entries.end());

    const Entry entry(group, chain, index, isNative);
    entries.insert(std::lower_bound(entries.begin(), entries.end(), entry),
                   entry);
    return true;
  }
};

template <typename GroupType, typename ChainType, typename IndexType>
inline bool reachable(
    const GroupLabel<GroupType, ChainType, IndexType>& forwardLabel,
    const GroupLabel<GroupType, ChainType, IndexType>& backwardLabel) noexcept {
  using Entry = typename GroupLabel<GroupType, ChainType, IndexType>::Entry;
  const std::vector<Entry>& F = forwardLabel.entries;
  const std::vector<Entry>& B = backwardLabel.entries;

  std::size_t f = 0;
  std::size_t b = 0;
  while (f < F.size() && b < B.size()) {
    if (F[f].getGroup() < B[b].getGroup()) {
      f++;
      continue;
    }
    if (B[b].getGroup() < F[f].getGroup()) {
      b++;
      continue;
    }

    const GroupType group = F[f].getGroup();
    std::size_t fEnd = f;
    while (fEnd < F.size() && F[fEnd].getGroup() == group) fEnd++;
    std::size_t bEnd = b;
    while (bEnd < B.size() && B[bEnd].getGroup() == group) bEnd++;

    IndexType minIndex = std::numeric_limits<IndexType>::max();
    std::size_t pf = f;
    for (std::size_t pb = b; pb < bEnd; pb++) {
      while (pf < fEnd && F[pf].getChain() <= B[pb].getChain()) {
        minIndex = std::min(minIndex, F[pf].getIndex());
        pf++;
      }
      if (minIndex <= B[pb].getIndex()) return true;
    }

    f = fEnd;
    b = bEnd;
  }
  return false;
}

}  // namespace HL
