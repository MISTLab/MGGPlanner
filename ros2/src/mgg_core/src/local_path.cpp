#include "mgg_core/local_path.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace mgg {
namespace {
void validate(const BrakingBounds& b) {
  if (!std::isfinite(b.deceleration_mps2) || b.deceleration_mps2 <= 0 ||
      !std::isfinite(b.latency_s) || b.latency_s < 0 ||
      !std::isfinite(b.planning_latency_s) || b.planning_latency_s < 0 ||
      !std::isfinite(b.margin_m) || b.margin_m < 0)
    throw std::invalid_argument("invalid braking bounds");
}
}  // namespace
double pathLength(const LocalPathPlan& p) {
  double length = 0;
  for (size_t i = 1; i < p.poses.size(); ++i)
    length += (p.poses[i] - p.poses[i - 1]).head<3>().norm();
  return length;
}
double commitmentLength(double speed, const BrakingBounds& b) {
  validate(b);
  if (!std::isfinite(speed)) throw std::invalid_argument("nonfinite speed");
  const double v = std::abs(speed);
  return std::max(2 * v, v * v / (2 * b.deceleration_mps2) +
                             v * (b.latency_s + b.planning_latency_s) +
                             b.margin_m);
}
double commitmentSpeedCap(double available, const BrakingBounds& b) {
  validate(b);
  const double t = b.latency_s + b.planning_latency_s;
  const double at = b.deceleration_mps2 * t;
  return std::max(
      0.0, std::sqrt(at * at + 2 * b.deceleration_mps2 *
                                   std::max(0.0, available - b.margin_m)) -
               at);
}
LocalPathPlan committedPrefix(const LocalPathPlan& p, double progress,
                              double length) {
  LocalPathPlan out = p;
  if (p.poses.empty()) return out;
  size_t first = 0, last = 0;
  double along = 0;
  progress = std::max(0.0, progress);
  for (size_t i = 1; i < p.poses.size(); ++i) {
    along += (p.poses[i] - p.poses[i - 1]).head<3>().norm();
    if (along <= progress + 1e-9) first = i;
    last = i;
    if (along >= progress + length - 1e-9) break;
  }
  out.poses.assign(p.poses.begin() + first, p.poses.begin() + last + 1);
  out.reverse.clear();
  for (size_t i = first; i <= last; ++i)
    out.reverse.push_back(i < p.reverse.size() && p.reverse[i]);
  out.edge_dependencies.clear();
  if (p.edge_dependencies.size() + 1 == p.poses.size())
    out.edge_dependencies.assign(p.edge_dependencies.begin() + first,
                                 p.edge_dependencies.begin() + last);
  out.prefix_length = out.poses.size();
  out.commit_length_m = pathLength(out);
  out.reaches_goal = p.reaches_goal && last + 1 == p.poses.size();
  return out;
}
LocalPathPlan splicePath(const LocalPathPlan& prefix,
                         const LocalPathPlan& extension) {
  if (prefix.poses.empty()) return extension;
  if (extension.poses.empty() || prefix.poses.back() != extension.poses.front())
    throw std::invalid_argument(
        "extension must start at exact prefix endpoint");
  LocalPathPlan out = extension;
  out.kind = LocalPathKind::kExtend;
  out.extends_sequence_id = prefix.sequence_id;
  out.poses = prefix.poses;
  out.poses.insert(out.poses.end(), extension.poses.begin() + 1,
                   extension.poses.end());
  out.reverse = prefix.reverse;
  if (!out.reverse.empty() && !extension.reverse.empty())
    out.reverse.back() = extension.reverse.front();
  if (!extension.reverse.empty())
    out.reverse.insert(out.reverse.end(), extension.reverse.begin() + 1,
                       extension.reverse.end());
  out.edge_dependencies = prefix.edge_dependencies;
  out.edge_dependencies.insert(out.edge_dependencies.end(),
                               extension.edge_dependencies.begin(),
                               extension.edge_dependencies.end());
  out.prefix_length = prefix.poses.size();
  out.commit_length_m = prefix.commit_length_m;
  return out;
}
}  // namespace mgg
