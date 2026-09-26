// The open tour over frontier clusters (tour-exploration design §2.3): the
// robot visits every reachable cluster once, starting where it stands, and
// does not return.
//
// Pure: it works on a cost matrix, so the tour (§2) and the fleet auction
// (§3.4) share it and it is tested without a graph. Up to
// kExactTourMaxClusters clusters every order is tried; beyond that a greedy
// nearest-neighbour start is improved with 2-opt and Or-opt until no move
// improves it.

#ifndef MGG_CORE_TOUR_SOLVER_H_
#define MGG_CORE_TOUR_SOLVER_H_

#include <cstddef>
#include <limits>
#include <vector>

namespace mgg {

/// A leg no route covers.
inline constexpr double kUnreachableCost =
    std::numeric_limits<double>::infinity();
/// Up to this many clusters after a fixed first one, solveOpenTour tries
/// every order (7! = 5040).
inline constexpr std::size_t kExactTourMaxClusters = 7;
/// improveOpenTour stops after this many moves. Every move lowers the cost,
/// so the search ends long before; it only bounds a pathological input.
inline constexpr int kMaxImprovingMoves = 100000;

struct OpenTour {
  /// Cluster indices in visiting order.
  std::vector<int> order;
  /// from_start of the first plus every leg between; kUnreachableCost when
  /// a leg is, or when a required first cluster cannot be reached.
  double cost = 0.0;
  /// The 2-opt and Or-opt moves made; kMaxImprovingMoves when the search
  /// stopped at its cap rather than converging. 0 when every order was
  /// tried.
  int improving_moves = 0;
};

/// Cost of visiting `order`: from_start[order[0]] plus `between` along it,
/// no return leg. Zero for an empty order.
double openTourCost(const std::vector<int>& order,
                    const std::vector<double>& from_start,
                    const std::vector<std::vector<double>>& between);

/// 2-opt and Or-opt (segments of one to three clusters, either way round)
/// from `order` until no move lowers the cost. `fixed_first` keeps order[0]
/// first. `between` must be symmetric.
OpenTour improveOpenTour(std::vector<int> order,
                         const std::vector<double>& from_start,
                         const std::vector<std::vector<double>>& between,
                         bool fixed_first = false);

/// The open tour over every index whose from_start is finite; the others
/// are left out until they can be reached. With `first` >= 0 the tour
/// starts there; when that index cannot be reached the tour is empty with
/// cost kUnreachableCost. `between` must be symmetric.
OpenTour solveOpenTour(const std::vector<double>& from_start,
                       const std::vector<std::vector<double>>& between,
                       int first = -1);

/// Where `index` joins a tour order for the least added cost.
struct Insertion {
  /// `index` goes before order[position]; order.size() appends it.
  std::size_t position = 0;
  double added_cost = kUnreachableCost;
};

/// The cheapest insertion of `index` into `order`, never before order[0]
/// when `keep_first`. added_cost is kUnreachableCost when no position has
/// finite legs.
Insertion cheapestInsertion(const std::vector<int>& order, int index,
                            const std::vector<double>& from_start,
                            const std::vector<std::vector<double>>& between,
                            bool keep_first);

}  // namespace mgg

#endif  // MGG_CORE_TOUR_SOLVER_H_
