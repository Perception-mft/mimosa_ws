// This implementation is derived from KISS-ICP and is distributed under its
// MIT license. See mimosa/lidar/kiss_icp.hpp for the complete license notice.

#include "mimosa/lidar/kiss_icp.hpp"

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>

namespace mimosa
{
namespace lidar
{
namespace kiss_icp
{
namespace
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3x6d = Eigen::Matrix<double, 3, 6>;
using Voxel = Eigen::Vector3i;
using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
using LinearSystem = std::pair<Matrix6d, Vector6d>;

struct VoxelHash
{
  size_t operator()(const Voxel & voxel) const
  {
    const auto x = static_cast<uint32_t>(voxel.x());
    const auto y = static_cast<uint32_t>(voxel.y());
    const auto z = static_cast<uint32_t>(voxel.z());
    return x * 73856093U ^ y * 19349669U ^ z * 83492791U;
  }
};

Voxel pointToVoxel(const Eigen::Vector3d & point, const double voxel_size)
{
  return Voxel(
    static_cast<int>(std::floor(point.x() / voxel_size)),
    static_cast<int>(std::floor(point.y() / voxel_size)),
    static_cast<int>(std::floor(point.z() / voxel_size)));
}

std::vector<Eigen::Vector3d> voxelDownsample(
  const std::vector<Eigen::Vector3d> & frame, const double voxel_size)
{
  std::unordered_map<Voxel, Eigen::Vector3d, VoxelHash> grid;
  grid.reserve(frame.size());
  for (const Eigen::Vector3d & point : frame) {
    grid.emplace(pointToVoxel(point, voxel_size), point);
  }

  std::vector<Eigen::Vector3d> downsampled;
  downsampled.reserve(grid.size());
  for (const auto & voxel_and_point : grid) {
    downsampled.emplace_back(voxel_and_point.second);
  }
  return downsampled;
}

// KISS/Sophus orders an SE(3) tangent as [translation, rotation], while GTSAM
// orders Pose3 tangents as [rotation, translation]. These helpers preserve the
// original KISS convention at the ICP boundary.
gtsam::Pose3 expKiss(const Vector6d & tangent)
{
  Vector6d gtsam_tangent;
  gtsam_tangent.head<3>() = tangent.tail<3>();
  gtsam_tangent.tail<3>() = tangent.head<3>();
  return gtsam::Pose3::Expmap(gtsam_tangent);
}

Vector6d logKiss(const gtsam::Pose3 & pose)
{
  const Vector6d gtsam_tangent = gtsam::Pose3::Logmap(pose);
  Vector6d tangent;
  tangent.head<3>() = gtsam_tangent.tail<3>();
  tangent.tail<3>() = gtsam_tangent.head<3>();
  return tangent;
}

void transformPoints(const gtsam::Pose3 & pose, std::vector<Eigen::Vector3d> & points)
{
  std::transform(
    points.cbegin(), points.cend(), points.begin(),
    [&](const Eigen::Vector3d & point) { return pose.transformFrom(point); });
}

class VoxelHashMap
{
public:
  VoxelHashMap(
    const double voxel_size, const double max_distance,
    const unsigned int max_points_per_voxel)
  : voxel_size_(voxel_size),
    max_distance_(max_distance),
    max_points_per_voxel_(max_points_per_voxel)
  {
  }

  void clear() { map_.clear(); }
  bool empty() const { return map_.empty(); }

  std::tuple<Eigen::Vector3d, double> closestNeighbor(const Eigen::Vector3d & query) const
  {
    static const std::array<Voxel, 27> voxel_shifts{{
      Voxel{0, 0, 0}, Voxel{1, 0, 0}, Voxel{-1, 0, 0}, Voxel{0, 1, 0}, Voxel{0, -1, 0},
      Voxel{0, 0, 1}, Voxel{0, 0, -1}, Voxel{1, 1, 0}, Voxel{1, -1, 0}, Voxel{-1, 1, 0},
      Voxel{-1, -1, 0}, Voxel{1, 0, 1}, Voxel{1, 0, -1}, Voxel{-1, 0, 1},
      Voxel{-1, 0, -1}, Voxel{0, 1, 1}, Voxel{0, 1, -1}, Voxel{0, -1, 1},
      Voxel{0, -1, -1}, Voxel{1, 1, 1}, Voxel{1, 1, -1}, Voxel{1, -1, 1},
      Voxel{1, -1, -1}, Voxel{-1, 1, 1}, Voxel{-1, 1, -1}, Voxel{-1, -1, 1},
      Voxel{-1, -1, -1}}};

    const Voxel voxel = pointToVoxel(query, voxel_size_);
    Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
    double closest_distance = std::numeric_limits<double>::max();

    for (const Voxel & shift : voxel_shifts) {
      const auto search = map_.find(voxel + shift);
      if (search == map_.end()) continue;

      for (const Eigen::Vector3d & candidate : search->second) {
        const double distance = (candidate - query).norm();
        if (distance < closest_distance) {
          closest_neighbor = candidate;
          closest_distance = distance;
        }
      }
    }
    return {closest_neighbor, closest_distance};
  }

  std::vector<Eigen::Vector3d> pointcloud() const
  {
    std::vector<Eigen::Vector3d> points;
    points.reserve(map_.size() * static_cast<size_t>(max_points_per_voxel_));
    for (const auto & voxel_and_points : map_) {
      points.insert(points.end(), voxel_and_points.second.cbegin(), voxel_and_points.second.cend());
    }
    return points;
  }

  void update(const std::vector<Eigen::Vector3d> & points, const gtsam::Pose3 & pose)
  {
    std::vector<Eigen::Vector3d> transformed(points);
    transformPoints(pose, transformed);
    addPoints(transformed);
    removePointsFarFrom(pose.translation());
  }

private:
  void addPoints(const std::vector<Eigen::Vector3d> & points)
  {
    const double map_resolution = voxel_size_ / std::sqrt(max_points_per_voxel_);
    for (const Eigen::Vector3d & point : points) {
      auto & voxel_points = map_[pointToVoxel(point, voxel_size_)];
      if (voxel_points.size() == max_points_per_voxel_) continue;
      if (std::any_of(
            voxel_points.cbegin(), voxel_points.cend(), [&](const Eigen::Vector3d & stored) {
              return (stored - point).norm() < map_resolution;
            })) {
        continue;
      }
      voxel_points.emplace_back(point);
    }
  }

  void removePointsFarFrom(const Eigen::Vector3d & origin)
  {
    const double max_distance_squared = max_distance_ * max_distance_;
    for (auto it = map_.begin(); it != map_.end();) {
      if ((it->second.front() - origin).squaredNorm() >= max_distance_squared) {
        it = map_.erase(it);
      } else {
        ++it;
      }
    }
  }

  double voxel_size_;
  double max_distance_;
  unsigned int max_points_per_voxel_;
  std::unordered_map<Voxel, std::vector<Eigen::Vector3d>, VoxelHash> map_;
};

Correspondences dataAssociation(
  const std::vector<Eigen::Vector3d> & points, const VoxelHashMap & voxel_map,
  const double max_correspondence_distance)
{
  Correspondences correspondences;
  correspondences.reserve(points.size());
  tbb::parallel_for(
    tbb::blocked_range<size_t>{0, points.size()}, [&](const tbb::blocked_range<size_t> & range) {
      for (size_t i = range.begin(); i < range.end(); ++i) {
        const auto [closest_neighbor, distance] = voxel_map.closestNeighbor(points[i]);
        if (distance < max_correspondence_distance) {
          correspondences.emplace_back(points[i], closest_neighbor);
        }
      }
    });
  return correspondences;
}

LinearSystem buildLinearSystem(
  const Correspondences & correspondences, const double kernel_scale)
{
  const auto add = [](LinearSystem lhs, const LinearSystem & rhs) {
      lhs.first += rhs.first;
      lhs.second += rhs.second;
      return lhs;
    };

  return tbb::parallel_reduce(
    tbb::blocked_range<Correspondences::const_iterator>{
      correspondences.cbegin(), correspondences.cend()},
    LinearSystem(Matrix6d::Zero(), Vector6d::Zero()),
    [&](const tbb::blocked_range<Correspondences::const_iterator> & range, LinearSystem system) {
      return std::transform_reduce(
        range.begin(), range.end(), system, add, [&](const auto & correspondence) {
          const auto & source = correspondence.first;
          const auto & target = correspondence.second;
          const Eigen::Vector3d residual = source - target;

          Matrix3x6d jacobian;
          jacobian.block<3, 3>(0, 0).setIdentity();
          jacobian.block<3, 3>(0, 3) = -gtsam::skewSymmetric(source);

          const double residual_squared = residual.squaredNorm();
          const double weight =
            std::pow(kernel_scale, 2) / std::pow(kernel_scale + residual_squared, 2);
          return LinearSystem{
            jacobian.transpose() * weight * jacobian,
            jacobian.transpose() * weight * residual};
        });
    },
    add);
}

}  // namespace

class KissICP::Impl
{
public:
  explicit Impl(const Config & config)
  : config_(config),
    local_map_(
      config.voxel_size, config.max_range,
      static_cast<unsigned int>(config.max_points_per_voxel)),
    model_sse_(config.initial_threshold * config.initial_threshold)
  {
    const int threads =
      config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
    thread_limit_ = std::make_unique<tbb::global_control>(
      tbb::global_control::max_allowed_parallelism, static_cast<size_t>(threads));
  }

  KissICP::RegistrationResult registerFrame(
    const KissICP::Vector3dVector & frame, const std::vector<double> & timestamps)
  {
    const auto preprocessed = preprocess(frame, timestamps);
    const auto frame_downsample = voxelDownsample(preprocessed, config_.voxel_size * 0.5);
    const auto source = voxelDownsample(frame_downsample, config_.voxel_size * 1.5);

    const double sigma = std::sqrt(model_sse_ / num_samples_);
    const gtsam::Pose3 initial_guess = last_pose_.compose(last_delta_);
    const gtsam::Pose3 new_pose = alignPointsToMap(source, initial_guess, 3.0 * sigma, sigma);

    updateAdaptiveThreshold(initial_guess.inverse().compose(new_pose));
    local_map_.update(frame_downsample, new_pose);
    last_delta_ = last_pose_.inverse().compose(new_pose);
    last_pose_ = new_pose;
    return {preprocessed, source};
  }

  std::vector<Eigen::Vector3d> preprocess(
    const std::vector<Eigen::Vector3d> & frame, const std::vector<double> & timestamps) const
  {
    std::vector<Eigen::Vector3d> deskewed(frame);
    if (config_.deskew && timestamps.size() == frame.size() && !timestamps.empty()) {
      const auto [min_it, max_it] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
      const double duration = *max_it - *min_it;
      if (duration > 0.0) {
        const Vector6d relative_motion = logKiss(last_delta_);
        tbb::parallel_for(
          tbb::blocked_range<size_t>{0, frame.size()}, [&](const tbb::blocked_range<size_t> & range) {
            for (size_t i = range.begin(); i < range.end(); ++i) {
              const double stamp = (timestamps[i] - *min_it) / duration;
              deskewed[i] = expKiss((stamp - 1.0) * relative_motion).transformFrom(frame[i]);
            }
          });
      }
    }

    std::vector<Eigen::Vector3d> filtered;
    filtered.reserve(deskewed.size());
    for (const Eigen::Vector3d & point : deskewed) {
      const double range = point.norm();
      if (range < config_.max_range && range > config_.min_range) filtered.emplace_back(point);
    }
    return filtered;
  }

  gtsam::Pose3 alignPointsToMap(
    const std::vector<Eigen::Vector3d> & frame, const gtsam::Pose3 & initial_guess,
    const double max_distance, const double kernel_scale) const
  {
    if (local_map_.empty()) return initial_guess;

    std::vector<Eigen::Vector3d> source(frame);
    transformPoints(initial_guess, source);
    gtsam::Pose3 icp;
    for (int iteration = 0; iteration < config_.max_num_iterations; ++iteration) {
      const Correspondences correspondences =
        dataAssociation(source, local_map_, max_distance);
      if (correspondences.empty()) break;

      const auto [jtj, jtr] = buildLinearSystem(correspondences, kernel_scale);
      const Vector6d increment = jtj.ldlt().solve(-jtr);
      if (!increment.allFinite()) break;

      const gtsam::Pose3 estimation = expKiss(increment);
      transformPoints(estimation, source);
      icp = estimation.compose(icp);
      if (increment.norm() < config_.convergence_criterion) break;
    }
    return icp.compose(initial_guess);
  }

  void updateAdaptiveThreshold(const gtsam::Pose3 & model_deviation)
  {
    const double theta = Eigen::AngleAxisd(model_deviation.rotation().matrix()).angle();
    const double rotation_error = 2.0 * config_.max_range * std::sin(theta / 2.0);
    const double model_error = model_deviation.translation().norm() + rotation_error;
    if (model_error > config_.min_motion_threshold) {
      model_sse_ += model_error * model_error;
      ++num_samples_;
    }
  }

  Config config_;
  VoxelHashMap local_map_;
  gtsam::Pose3 last_pose_;
  gtsam::Pose3 last_delta_;
  double model_sse_;
  int num_samples_ = 1;
  std::unique_ptr<tbb::global_control> thread_limit_;
};

KissICP::KissICP(const Config & config) : impl_(std::make_unique<Impl>(config)) {}
KissICP::~KissICP() = default;

KissICP::RegistrationResult KissICP::registerFrame(
  const Vector3dVector & frame, const std::vector<double> & timestamps)
{
  return impl_->registerFrame(frame, timestamps);
}

KissICP::Vector3dVector KissICP::localMap() const { return impl_->local_map_.pointcloud(); }
const gtsam::Pose3 & KissICP::pose() const { return impl_->last_pose_; }
const gtsam::Pose3 & KissICP::delta() const { return impl_->last_delta_; }

void KissICP::reset()
{
  impl_->last_pose_ = gtsam::Pose3();
  impl_->last_delta_ = gtsam::Pose3();
  impl_->local_map_.clear();
  impl_->model_sse_ = impl_->config_.initial_threshold * impl_->config_.initial_threshold;
  impl_->num_samples_ = 1;
}

}  // namespace kiss_icp
}  // namespace lidar
}  // namespace mimosa
