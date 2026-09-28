// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "mimosa/lidar/geometric_config.hpp"
#include "mimosa/state.hpp"

namespace mimosa
{
namespace lidar
{

/** A pose observation produced by the unmodified KISS-ICP pipeline. */
class KISSICPFactor : public gtsam::NonlinearFactor
{
public:
  using Ptr = std::shared_ptr<KISSICPFactor>;

  // Retained for compatibility with the existing debug message and marker code.
  enum class RejectStatus
  {
    Unprocessed = 0,
    InsufficientCorresPoints,
    CorresMaxDist,
    EigenSolverFail,
    MinEigenValueLow,
    Line,
    CorresPlaneInvalid,
    MaxError,
    Valid
  };

  KISSICPFactor(
    const gtsam::Key key, const gtsam::Pose3 & measured_pose,
    const RegistrationConfig & config)
  : gtsam::NonlinearFactor(std::vector<gtsam::Key>{key}), measured_pose_(measured_pose)
  {
    // KISS-ICP does not estimate covariance. Keep fusion confidence explicit,
    // bounded, and independent of scan resolution/correspondence count.
    const double weight = std::max(1e-12F, config.lidar_information_weight);
    const double sigma_scale = 1.0 / std::sqrt(weight);
    V6D sigmas;
    sigmas.head<3>().setConstant(deg2rad(config.lidar_pose_rotation_std_dev_deg) * sigma_scale);
    sigmas.tail<3>().setConstant(config.lidar_pose_translation_std_dev * sigma_scale);
    noise_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    localizability_rot_.setConstant(1.0 / std::pow(sigmas(0), 2));
    localizability_trans_.setConstant(1.0 / std::pow(sigmas(3), 2));
  }

  ~KISSICPFactor() override = default;

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new KISSICPFactor(*this)));
  }

  size_t dim() const override { return 6; }
  double error(const gtsam::Values & values) const override { return makePrior().error(values); }

  std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values & values) const override
  {
    ++linearize_count_;
    return makePrior().linearize(values);
  }

  const std::vector<RejectStatus> & getStatuses() const { return statuses_; }
  const std::vector<V3D> & getCorresMeansTarget() const { return empty_points_; }
  const std::vector<V3D> & getTransformedSource() const { return empty_points_; }
  int getLinearizeCount() const { return linearize_count_; }
  const gtsam::Pose3 & measuredPose() const { return measured_pose_; }

  void getLocalizabilities(
    V3D & trans_comp, V3D & rot_comp, V3D & trans_final, V3D & rot_final,
    M33 & eigenvectors_trans, M33 & eigenvectors_rot) const
  {
    trans_comp = trans_final = localizability_trans_;
    rot_comp = rot_final = localizability_rot_;
    eigenvectors_trans.setIdentity();
    eigenvectors_rot.setIdentity();
  }

private:
  gtsam::PriorFactor<gtsam::Pose3> makePrior() const
  {
    return gtsam::PriorFactor<gtsam::Pose3>(keys().front(), measured_pose_, noise_);
  }

  gtsam::Pose3 measured_pose_;
  gtsam::noiseModel::Diagonal::shared_ptr noise_;
  V3D localizability_trans_ = Z31;
  V3D localizability_rot_ = Z31;
  std::vector<RejectStatus> statuses_;
  std::vector<V3D> empty_points_;
  mutable int linearize_count_ = 0;
};

}  // namespace lidar
}  // namespace mimosa
