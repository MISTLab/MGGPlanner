#ifndef MGG_MAP_OCTOMAP_CELL_INDEX_H_
#define MGG_MAP_OCTOMAP_CELL_INDEX_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mgg {

// Flat, read-only open-addressed index into the original cell vectors. Keeping
// the sorted vectors preserves point-cloud order and duplicate extraction.
// Slot zero is empty; odd slots refer to occupied cells, even to free cells.
template <class Cell>
class CellIndex {
 public:
  CellIndex() = default;
  CellIndex(const std::vector<Cell>& occupied, const std::vector<Cell>& free) {
    build(occupied, free);
  }

  void build(const std::vector<Cell>& occupied, const std::vector<Cell>& free) {
    const std::size_t count = occupied.size() + free.size();
    if (count == 0) {
      slots_.clear();
      return;
    }
    std::size_t capacity = 2;
    // Keep at most half the slots occupied even with disjoint input cells.
    while (capacity / 2 < count) capacity *= 2;
    slots_.assign(capacity, 0);
    for (std::size_t i = 0; i < occupied.size(); ++i)
      insert(occupied[i], (std::uint64_t(i) + 1) * 2 + 1, occupied, free);
    for (std::size_t i = 0; i < free.size(); ++i)
      insert(free[i], (std::uint64_t(i) + 1) * 2, occupied, free);
  }

  // 2 occupied, 1 free, 0 unknown. Occupied wins if present in both lists.
  int status(const Cell& key, const std::vector<Cell>& occupied,
             const std::vector<Cell>& free) const {
    if (slots_.empty()) return 0;
    std::size_t pos = hash(key) & (slots_.size() - 1);
    while (std::uint64_t slot = slots_[pos]) {
      const Cell& existing = (slot & 1) ? occupied[(slot >> 1) - 1]
                                        : free[(slot >> 1) - 1];
      if (existing.x == key.x && existing.y == key.y && existing.z == key.z)
        return (slot & 1) ? 2 : 1;
      pos = (pos + 1) & (slots_.size() - 1);
    }
    return 0;
  }

  std::size_t bytes() const { return slots_.size() * sizeof(std::uint64_t); }

 private:
  static std::uint64_t mix(std::uint64_t x) {
    x ^= x >> 30;
    x *= 0xbf58476d1ce4e5b9ULL;
    x ^= x >> 27;
    x *= 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
  }
  static std::uint64_t hash(const Cell& k) {
    const auto x = mix(std::uint64_t(k.x));
    const auto y = mix(std::uint64_t(k.y) + 0x9e3779b97f4a7c15ULL);
    const auto z = mix(std::uint64_t(k.z) + 0xd1b54a32d192ed03ULL);
    return mix(x ^ y ^ z);
  }
  void insert(const Cell& key, std::uint64_t encoded,
              const std::vector<Cell>& occupied, const std::vector<Cell>& free) {
    std::size_t pos = hash(key) & (slots_.size() - 1);
    while (std::uint64_t slot = slots_[pos]) {
      const Cell& existing = (slot & 1) ? occupied[(slot >> 1) - 1]
                                        : free[(slot >> 1) - 1];
      if (existing.x == key.x && existing.y == key.y && existing.z == key.z)
        return;
      pos = (pos + 1) & (slots_.size() - 1);
    }
    slots_[pos] = encoded;
  }
  std::vector<std::uint64_t> slots_;
};

}  // namespace mgg
#endif
