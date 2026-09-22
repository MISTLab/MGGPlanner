#include "mgg_core/params.h"

namespace mgg {

Eigen::Vector3d RobotParams::getPlanningSize() const {
  switch (bound_mode) {
    case BoundModeType::kExtendedBound:
      return size + size_extension;
    case BoundModeType::kRelaxedBound:
      return size + relax_ratio * size_extension_min +
             (1.0 - relax_ratio) * size_extension;
    case BoundModeType::kMinBound:
      return size + size_extension_min;
    case BoundModeType::kExactBound:
      return size;
    case BoundModeType::kNoBound:
      return Eigen::Vector3d::Zero();
  }
  return Eigen::Vector3d::Zero();
}

void BoundedSpaceParams::updateRotation() {
  const Eigen::Matrix3d rot_w2b =
      (Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  rot_b2w_ = rot_w2b.transpose();
}

void BoundedSpaceParams::setCenter(const Eigen::Vector3d& root,
                                   bool use_extension) {
  root_pos_ = root;
  if (use_extension) {
    min_val_total_ = min_val + min_extension;
    max_val_total_ = max_val + max_extension;
    radius_total_ = radius + radius_extension;
  } else {
    min_val_total_ = min_val;
    max_val_total_ = max_val;
    radius_total_ = radius;
  }
  updateRotation();
}

void BoundedSpaceParams::setCenter(const StateVec& state, bool use_extension) {
  setCenter(Eigen::Vector3d(state[0], state[1], state[2]), use_extension);
}

void BoundedSpaceParams::setBound(const Eigen::Vector3d& min_in,
                                  const Eigen::Vector3d& max_in) {
  min_val = min_in;
  max_val = max_in;
}

void BoundedSpaceParams::setRotation(const Eigen::Vector3d& rotations_in) {
  rotations = rotations_in;
  updateRotation();
}

bool BoundedSpaceParams::isInsideSpace(const Eigen::Vector3d& pos) const {
  if (type == BoundedSpaceType::kSphere) {
    const double r = radius_total_ > 0.0 ? radius_total_ : radius;
    return (pos - root_pos_).norm() <= r;
  }
  // kCuboid. Uses the totals if setCenter was called, falling back to min_val/max_val.
  const Eigen::Vector3d pos_b = rot_b2w_ * (pos - root_pos_);
  const Eigen::Vector3d min_b = (min_val_total_ != max_val_total_) ? min_val_total_ : min_val;
  const Eigen::Vector3d max_b = (min_val_total_ != max_val_total_) ? max_val_total_ : max_val;
  for (int i = 0; i < 3; ++i) {
    if (pos_b[i] < min_b[i] || pos_b[i] > max_b[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace mgg
