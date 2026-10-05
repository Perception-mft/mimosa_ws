// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <gtsam/geometry/Pose3.h>

#include <Eigen/Core>
#include <memory>
#include <tuple>
#include <vector>

namespace mimosa
{
namespace lidar
{
namespace kiss_icp
{

// Self-contained KISS-ICP configuration. The names intentionally match the
// upstream implementation so the ROS configuration remains unchanged.
struct Config
{
  double voxel_size = 1.0;
  double max_range = 100.0;
  double min_range = 0.0;
  int max_points_per_voxel = 20;

  double min_motion_threshold = 0.1;
  double initial_threshold = 2.0;

  int max_num_iterations = 500;
  double convergence_criterion = 0.0001;
  int max_num_threads = 0;

  bool deskew = true;
};

/**
 * Local implementation of the KISS-ICP odometry pipeline.
 *
 * It performs range filtering, constant-velocity deskewing, two-stage voxel
 * downsampling, adaptive-threshold point-to-point ICP, and local-map updates.
 * It depends only on libraries Mimosa already uses (Eigen, GTSAM, and TBB).
 */
class KissICP
{
public:
  using Vector3dVector = std::vector<Eigen::Vector3d>;
  using RegistrationResult = std::tuple<Vector3dVector, Vector3dVector>;

  explicit KissICP(const Config & config);
  ~KissICP();

  KissICP(const KissICP &) = delete;
  KissICP & operator=(const KissICP &) = delete;

  RegistrationResult registerFrame(
    const Vector3dVector & frame, const std::vector<double> & timestamps);

  Vector3dVector localMap() const;
  const gtsam::Pose3 & pose() const;
  const gtsam::Pose3 & delta() const;
  void reset();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace kiss_icp
}  // namespace lidar
}  // namespace mimosa
