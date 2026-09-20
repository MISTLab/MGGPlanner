#include "mgg_core/random_sampler.h"

#include <cmath>

namespace mgg {

RandomSampler::RandomSampler(const Eigen::Vector3d& min_val,
                             const Eigen::Vector3d& max_val, unsigned seed) {
  setBound(min_val, max_val);
  reset(seed);
}

void RandomSampler::setBound(const Eigen::Vector3d& min_val,
                             const Eigen::Vector3d& max_val) {
  min_val_ = min_val.cwiseMin(max_val);
  max_val_ = max_val.cwiseMax(min_val);
}

void RandomSampler::reset(unsigned seed) { generator_.seed(seed); }

void RandomSampler::generate(const StateVec& current_state,
                             StateVec& sample_state) {
  for (int i = 0; i < 3; ++i) {
    std::uniform_real_distribution<double> axis(min_val_[i], max_val_[i]);
    sample_state[i] = current_state[i] + axis(generator_);
  }
  std::uniform_real_distribution<double> heading(-M_PI, M_PI);
  sample_state[3] = current_state[3] + heading(generator_);
}

std::size_t RandomSampler::index(std::size_t count) {
  if (count <= 1) return 0;
  std::uniform_int_distribution<std::size_t> pick(0, count - 1);
  return pick(generator_);
}

}  // namespace mgg
