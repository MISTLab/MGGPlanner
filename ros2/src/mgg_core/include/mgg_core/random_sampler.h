// Uniform sampling in a box around a state.
//
// Ported from planner_common/random_sampler.h, reduced to what the global
// graph expansion (Rrg::expandGlobalGraphTimerCallback) draws from it:
// RandomSampler::generate (random_sampler.cpp:252) with every axis kUniform,
// X, Y and Z in kLocal mode over the local sampling box, and Heading in
// kManual mode over [-pi, pi], which is how every shipped configuration set
// SamplerForExploration (mggplanner_config.yaml:65). The kNormal and kCauchy
// distributions, the per-axis kConst offset and the sample buffers kept for
// RViz are not carried across.
//
// Seedable, so a test can repeat a run; upstream seeded from
// std::random_device (random_sampler.cpp:242), which the node still does.

#ifndef MGG_CORE_RANDOM_SAMPLER_H_
#define MGG_CORE_RANDOM_SAMPLER_H_

#include <cstddef>
#include <random>

#include <Eigen/Dense>

#include "mgg_core/types.h"

namespace mgg {

class RandomSampler {
 public:
  RandomSampler() = default;
  RandomSampler(const Eigen::Vector3d& min_val, const Eigen::Vector3d& max_val,
                unsigned seed);

  /// The box, relative to the state a sample is generated around. This is
  /// the local bound the local graph is built in (GridGraphParams::min_val
  /// and max_val), as upstream's kLocal mode took BoundedSpaceParams/Local.
  void setBound(const Eigen::Vector3d& min_val, const Eigen::Vector3d& max_val);
  void reset(unsigned seed);

  /// A state uniformly distributed in the box centred on `current_state`,
  /// with a yaw uniformly distributed around the current yaw
  /// (random_sampler.cpp:252 RandomSampler::generate, without rotation).
  void generate(const StateVec& current_state, StateVec& sample_state);

  /// A uniformly distributed index in [0, count). Stands in for the
  /// rand() % size the cluster seed was drawn with (rrg.cpp:2581), so the
  /// whole expansion follows one seed.
  std::size_t index(std::size_t count);

 private:
  Eigen::Vector3d min_val_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_val_ = Eigen::Vector3d::Zero();
  std::mt19937 generator_;
};

}  // namespace mgg

#endif  // MGG_CORE_RANDOM_SAMPLER_H_
