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

// Flat, read-only open-addressed index of the XY columns of two cell
// vectors sorted by (x, y, z): for each column, the contiguous range of its
// cells in each vector. A box query then reads one column per XY cell, in z
// order, instead of looking up every voxel of the box.
template <class Cell>
class ColumnIndex {
 public:
  struct Span {
    std::uint32_t occupied_begin = 0, occupied_end = 0;
    std::uint32_t free_begin = 0, free_end = 0;
  };

  void build(const std::vector<Cell>& occupied, const std::vector<Cell>& free) {
    keys_.clear();
    spans_.clear();
    used_.clear();
    std::size_t columns = 0;
    const auto count = [&](const std::vector<Cell>& cells) {
      for (std::size_t i = 0; i < cells.size(); ++i)
        if (i == 0 || cells[i].x != cells[i - 1].x || cells[i].y != cells[i - 1].y)
          ++columns;
    };
    count(occupied);
    count(free);
    if (columns == 0) return;
    std::size_t capacity = 2;
    while (capacity / 2 < columns) capacity *= 2;
    keys_.assign(capacity, Key{});
    spans_.assign(capacity, Span{});
    used_.assign(capacity, 0);
    const auto add = [&](const std::vector<Cell>& cells, bool is_occupied) {
      std::size_t i = 0;
      while (i < cells.size()) {
        std::size_t j = i + 1;
        while (j < cells.size() && cells[j].x == cells[i].x &&
               cells[j].y == cells[i].y)
          ++j;
        Span& span = slot(cells[i].x, cells[i].y);
        if (is_occupied) {
          span.occupied_begin = static_cast<std::uint32_t>(i);
          span.occupied_end = static_cast<std::uint32_t>(j);
        } else {
          span.free_begin = static_cast<std::uint32_t>(i);
          span.free_end = static_cast<std::uint32_t>(j);
        }
        i = j;
      }
    };
    add(occupied, true);
    add(free, false);
  }

  /// The column's ranges, or null when neither vector has a cell in it.
  const Span* find(std::int64_t x, std::int64_t y) const {
    if (keys_.empty()) return nullptr;
    std::size_t pos = hash(x, y) & (keys_.size() - 1);
    while (used_[pos]) {
      if (keys_[pos].x == x && keys_[pos].y == y) return &spans_[pos];
      pos = (pos + 1) & (keys_.size() - 1);
    }
    return nullptr;
  }

  std::size_t bytes() const {
    return keys_.size() * (sizeof(Key) + sizeof(Span) + 1);
  }

 private:
  struct Key {
    std::int64_t x = 0, y = 0;
  };
  static std::uint64_t mix(std::uint64_t x) {
    x ^= x >> 30;
    x *= 0xbf58476d1ce4e5b9ULL;
    x ^= x >> 27;
    x *= 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
  }
  static std::uint64_t hash(std::int64_t x, std::int64_t y) {
    return mix(mix(std::uint64_t(x)) ^
               mix(std::uint64_t(y) + 0x9e3779b97f4a7c15ULL));
  }
  Span& slot(std::int64_t x, std::int64_t y) {
    std::size_t pos = hash(x, y) & (keys_.size() - 1);
    while (used_[pos]) {
      if (keys_[pos].x == x && keys_[pos].y == y) return spans_[pos];
      pos = (pos + 1) & (keys_.size() - 1);
    }
    used_[pos] = 1;
    keys_[pos] = Key{x, y};
    return spans_[pos];
  }
  std::vector<Key> keys_;
  std::vector<Span> spans_;
  std::vector<std::uint8_t> used_;
};

}  // namespace mgg
#endif
