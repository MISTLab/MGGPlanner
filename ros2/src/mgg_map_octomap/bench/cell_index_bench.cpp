// Standalone: g++ -O3 -std=c++17 -I../include cell_index_bench.cpp -o cell_index_bench
// Run on workstation and botman; same tunnel/ray workload as host_bench.cpp.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <unordered_set>
#include <vector>

#include "mgg_map_octomap/cell_index.h"

struct Cell {
  std::int64_t x, y, z;
  bool operator==(const Cell& b) const { return x == b.x && y == b.y && z == b.z; }
  bool operator<(const Cell& b) const {
    if (x != b.x) return x < b.x;
    if (y != b.y) return y < b.y;
    return z < b.z;
  }
};
struct Hash {
  std::size_t operator()(const Cell& c) const {
    std::uint64_t h = std::uint64_t(c.x) * 0x9e3779b97f4a7c15ULL;
    h ^= std::uint64_t(c.y) * 0xc2b2ae3d27d4eb4fULL + (h << 6) + (h >> 2);
    h ^= std::uint64_t(c.z) * 0x165667b19e3779f9ULL + (h << 6) + (h >> 2);
    return h;
  }
};

int main() {
  std::mt19937_64 rng(42);
  for (std::size_t target : {250000ul, 1000000ul, 4000000ul}) {
    std::vector<Cell> occupied, free;
    std::unordered_set<Cell, Hash> seen;
    seen.reserve(target * 2);
    std::uniform_int_distribution<int> xy(-2000, 2000), z(-10, 30);
    while (occupied.size() + free.size() < target) {
      Cell c{xy(rng), xy(rng), z(rng)};
      for (int i = 0; i < 64 && occupied.size() + free.size() < target; ++i)
        for (int w = 0; w < 4 && occupied.size() + free.size() < target; ++w) {
          Cell q{c.x + i, c.y + w, c.z};
          if (seen.insert(q).second)
            (w == 0 || w == 3 ? occupied : free).push_back(q);
        }
    }
    std::sort(occupied.begin(), occupied.end());
    std::sort(free.begin(), free.end());
    mgg::CellIndex<Cell> index(occupied, free);
    std::vector<Cell> queries;
    queries.reserve(1000000);
    std::uniform_int_distribution<std::size_t> pick(0, free.size() - 1);
    std::uniform_int_distribution<int> dir(-1, 1);
    while (queries.size() < 1000000) {
      Cell c = free[pick(rng)];
      int dx = dir(rng), dy = dir(rng), dz = dir(rng);
      for (int i = 0; i < 40 && queries.size() < 1000000; ++i)
        queries.push_back({c.x + dx * i, c.y + dy * i, c.z + dz * i});
    }
    auto measure = [&](auto lookup) {
      volatile std::uint64_t sink = 0;
      auto start = std::chrono::steady_clock::now();
      for (int rep = 0; rep < 3; ++rep)
        for (const Cell& c : queries) sink += lookup(c);
      auto end = std::chrono::steady_clock::now();
      return std::chrono::duration<double, std::nano>(end - start).count() /
             (3 * queries.size());
    };
    const auto binary = measure([&](const Cell& c) {
      return std::binary_search(occupied.begin(), occupied.end(), c) ? 2 :
             std::binary_search(free.begin(), free.end(), c) ? 1 : 0;
    });
    const auto flat = measure([&](const Cell& c) {
      return index.status(c, occupied, free);
    });
    std::printf("cells=%zu binary_ns=%.1f flat_ns=%.1f speedup=%.2f "
                "old_bytes_per_cell=%.1f new_bytes_per_cell=%.1f\n",
                target, binary, flat, binary / flat,
                double((occupied.size() + free.size()) * sizeof(Cell)) / target,
                double((occupied.size() + free.size()) * sizeof(Cell) + index.bytes()) / target);
  }
}
