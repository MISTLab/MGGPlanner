// Cooperative request cancellation, scoped to the planner thread. Geometry
// callers outside a request have no token and retain their ordinary behavior.
#ifndef MGG_CORE_PLANNING_CANCELLATION_H_
#define MGG_CORE_PLANNING_CANCELLATION_H_

#include <functional>
#include <utility>

namespace mgg {
struct PlanningInterrupted {};

inline thread_local const std::function<bool()>* planning_cancelled = nullptr;

class PlanningCancellationScope {
 public:
  explicit PlanningCancellationScope(std::function<bool()> cancelled)
      : cancelled_(std::move(cancelled)), previous_(planning_cancelled) {
    planning_cancelled = &cancelled_;
  }
  ~PlanningCancellationScope() { planning_cancelled = previous_; }
  PlanningCancellationScope(const PlanningCancellationScope&) = delete;
  PlanningCancellationScope& operator=(const PlanningCancellationScope&) = delete;
 private:
  std::function<bool()> cancelled_;
  const std::function<bool()>* previous_;
};

inline void planningCheckpoint() {
  if (planning_cancelled && (*planning_cancelled)()) throw PlanningInterrupted{};
}
}  // namespace mgg
#endif
