#include "mgg_core/tour_solver.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mgg {
namespace {

/// Stands in for an unreachable leg inside the search, so every cost there
/// stays finite and comparable; openTourCost reports the true sum.
constexpr double kDisconnectedLegCost = 1e9;
/// A local move counts only when it lowers the cost by more than this, or by
/// more than kRelativeImproveEps times the sum of the legs it compares,
/// whichever is larger. The relative part keeps the threshold far above the
/// rounding error of those sums (a few ulps of the sum, about 2e-16 each),
/// which a fixed 1e-9 is not once a kDisconnectedLegCost leg is among them:
/// then a tie could look like an improvement both ways round and the search
/// would cycle until kMaxImprovingMoves. With it every accepted move lowers
/// the exact cost of the order, so no order repeats.
constexpr double kImproveEps = 1e-9;
constexpr double kRelativeImproveEps = 1e-12;
/// Pseudo-indices: where the robot stands, and the open end of the tour.
constexpr int kStart = -1;
constexpr int kEnd = -2;

/// Whether `delta`, computed from legs summing to `legs`, lowers the cost.
bool improves(double delta, double legs) {
  return delta < -std::max(kImproveEps, kRelativeImproveEps * legs);
}

class LegCost {
 public:
  LegCost(const std::vector<double>& from_start,
          const std::vector<std::vector<double>>& between)
      : from_start_(from_start), between_(between) {}

  double operator()(int from, int to) const {
    if (to == kEnd) return 0.0;
    const double cost = from == kStart ? from_start_[to] : between_[from][to];
    return std::isfinite(cost) ? cost : kDisconnectedLegCost;
  }

 private:
  const std::vector<double>& from_start_;
  const std::vector<std::vector<double>>& between_;
};

/// First improving 2-opt move: reverse order[i..j].
bool twoOptMove(std::vector<int>& order, const LegCost& leg, std::size_t lo) {
  const std::size_t n = order.size();
  for (std::size_t i = lo; i + 1 < n; ++i) {
    const int before = i == 0 ? kStart : order[i - 1];
    for (std::size_t j = i + 1; j < n; ++j) {
      const int after = j + 1 < n ? order[j + 1] : kEnd;
      const double added = leg(before, order[j]) + leg(order[i], after);
      const double removed = leg(before, order[i]) + leg(order[j], after);
      if (improves(added - removed, added + removed)) {
        std::reverse(order.begin() + i, order.begin() + j + 1);
        return true;
      }
    }
  }
  return false;
}

/// First improving Or-opt move: a segment of one to three clusters moved
/// elsewhere, either way round.
bool orOptMove(std::vector<int>& order, const LegCost& leg, std::size_t lo) {
  const std::size_t n = order.size();
  for (std::size_t length = 1; length <= 3; ++length) {
    for (std::size_t i = lo; i + length <= n; ++i) {
      const std::size_t k = i + length - 1;
      const int first = order[i];
      const int last = order[k];
      const int a = i == 0 ? kStart : order[i - 1];
      const int b = k + 1 < n ? order[k + 1] : kEnd;
      const double removed = leg(a, first) + leg(last, b) - leg(a, b);
      std::vector<int> rest;
      rest.reserve(n - length);
      rest.insert(rest.end(), order.begin(), order.begin() + i);
      rest.insert(rest.end(), order.begin() + k + 1, order.end());
      for (std::size_t p = lo; p <= rest.size(); ++p) {
        if (p == i) continue;  // where the segment came from
        const int u = p == 0 ? kStart : rest[p - 1];
        const int v = p < rest.size() ? rest[p] : kEnd;
        const double base = leg(u, v);
        const double forward = leg(u, first) + leg(last, v) - base;
        const double backward = leg(u, last) + leg(first, v) - base;
        const bool reverse = backward < forward;
        const double delta = (reverse ? backward : forward) - removed;
        if (delta < -kImproveEps &&
            improves(delta, leg(a, first) + leg(last, b) + leg(a, b) +
                                leg(u, first) + leg(last, v) +
                                leg(u, last) + leg(first, v) + base)) {
          std::vector<int> segment(order.begin() + i, order.begin() + k + 1);
          if (reverse) std::reverse(segment.begin(), segment.end());
          rest.insert(rest.begin() + p, segment.begin(), segment.end());
          order.swap(rest);
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace

double openTourCost(const std::vector<int>& order,
                    const std::vector<double>& from_start,
                    const std::vector<std::vector<double>>& between) {
  if (order.empty()) return 0.0;
  double cost = from_start[order.front()];
  for (std::size_t i = 1; i < order.size(); ++i) {
    cost += between[order[i - 1]][order[i]];
  }
  return cost;
}

OpenTour improveOpenTour(std::vector<int> order,
                         const std::vector<double>& from_start,
                         const std::vector<std::vector<double>>& between,
                         bool fixed_first) {
  const LegCost leg(from_start, between);
  const std::size_t lo = fixed_first ? 1 : 0;
  int moves = 0;
  while (moves < kMaxImprovingMoves &&
         (twoOptMove(order, leg, lo) || orOptMove(order, leg, lo))) {
    ++moves;
  }
  OpenTour tour;
  tour.improving_moves = moves;
  tour.cost = openTourCost(order, from_start, between);
  tour.order = std::move(order);
  return tour;
}

OpenTour solveOpenTour(const std::vector<double>& from_start,
                       const std::vector<std::vector<double>>& between,
                       int first) {
  OpenTour tour;
  const int n = static_cast<int>(from_start.size());
  if (first >= 0 && (first >= n || !std::isfinite(from_start[first]))) {
    tour.cost = kUnreachableCost;
    return tour;
  }
  std::vector<int> rest;
  for (int i = 0; i < n; ++i) {
    if (i != first && std::isfinite(from_start[i])) rest.push_back(i);
  }
  if (rest.empty() && first < 0) return tour;
  const LegCost leg(from_start, between);
  std::vector<int> order;
  if (first >= 0) order.push_back(first);

  if (rest.size() <= kExactTourMaxClusters) {
    // Every order of the rest; `rest` is ascending, as next_permutation
    // needs to visit them all.
    std::vector<int> best = rest;
    double best_cost = kUnreachableCost;
    do {
      int previous = first >= 0 ? first : kStart;
      double cost = first >= 0 ? leg(kStart, first) : 0.0;
      for (const int index : rest) {
        cost += leg(previous, index);
        previous = index;
      }
      // Enumeration cannot cycle, so the tight absolute test: half a
      // millimetre still picks the first cluster before a disconnected leg.
      if (cost < best_cost - kImproveEps) {
        best_cost = cost;
        best = rest;
      }
    } while (std::next_permutation(rest.begin(), rest.end()));
    order.insert(order.end(), best.begin(), best.end());
    tour.cost = openTourCost(order, from_start, between);
    tour.order = std::move(order);
    return tour;
  }

  // Greedy nearest neighbour from the start (lower index on a tie), then
  // local improvement.
  std::vector<bool> placed(n, false);
  int current = first >= 0 ? first : kStart;
  for (std::size_t step = 0; step < rest.size(); ++step) {
    int next = -1;
    double next_cost = kUnreachableCost;
    for (const int index : rest) {
      if (placed[index]) continue;
      const double cost = leg(current, index);
      if (next < 0 || cost < next_cost) {
        next_cost = cost;
        next = index;
      }
    }
    placed[next] = true;
    order.push_back(next);
    current = next;
  }
  return improveOpenTour(std::move(order), from_start, between, first >= 0);
}

Insertion cheapestInsertion(const std::vector<int>& order, int index,
                            const std::vector<double>& from_start,
                            const std::vector<std::vector<double>>& between,
                            bool keep_first) {
  const auto cost = [&](int from, int to) {
    return from == kStart ? from_start[to] : between[from][to];
  };
  Insertion best;
  const std::size_t lo = keep_first && !order.empty() ? 1 : 0;
  for (std::size_t p = lo; p <= order.size(); ++p) {
    const int u = p == 0 ? kStart : order[p - 1];
    const double in = cost(u, index);
    const double out = p < order.size() ? between[index][order[p]] : 0.0;
    const double base = p < order.size() ? cost(u, order[p]) : 0.0;
    if (!std::isfinite(in) || !std::isfinite(out) || !std::isfinite(base)) {
      continue;
    }
    const double added = in + out - base;
    if (added < best.added_cost) {
      best.added_cost = added;
      best.position = p;
    }
  }
  return best;
}

}  // namespace mgg
