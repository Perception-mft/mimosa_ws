// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// OpenMP
#include <omp.h>

// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// mimosa
#include "mimosa/lidar/geometric_config.hpp"
#include "mimosa/lidar/geometric_factor.hpp"
#include "mimosa/lidar/utils.hpp"
#include "mimosa/state.hpp"
#include "mimosa/stopwatch.hpp"
#include "mimosa/utils.hpp"
#include "mimosa_msgs/msg/lidar_geometric_debug.hpp"

// ROS
#include <geometry_msgs/msg/pose_array.hpp>

#include <kiss_icp/pipeline/KissICP.hpp>

namespace mimosa
{
namespace lidar
{
class Geometric
{
public:
  const GeometricConfig config;

private:
  std::unique_ptr<spdlog::logger> logger_;

  // Member variables
  pcl::PointCloud<Point>::Ptr Be_cloud_;
  pcl::PointCloud<Point> sm_Be_cloud_ds_;
  KISSICPFactor::Ptr factor_;
  std::unique_ptr<kiss_icp::pipeline::KissICP> kiss_icp_;
  std::vector<Eigen::Vector3d> kiss_frame_;
  std::vector<double> kiss_timestamps_;
  gtsam::Pose3 T_W_K_;
  bool kiss_pose_initialized_ = false;

  // Variables for map
  std::vector<gtsam::Pose3> map_poses_;
  geometry_msgs::msg::PoseArray keyframe_poses_;

  double ts_;
  mimosa_msgs::msg::LidarGeometricDebug debug_msg_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sm_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sm_cloud_ds_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_sm_correspondances_ma_;
  rclcpp::Publisher<mimosa_msgs::msg::LidarGeometricDebug>::SharedPtr pub_debug_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_localizability_marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_degen_marker_array_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_keyframe_poses_;

  void fillMarkerArray(
    const KISSICPFactor & factor, visualization_msgs::msg::MarkerArray & ma,
    const std::string & frame_id, const double ts);
public:
  Geometric(rclcpp::Node & pnh);
  void preprocess(
    const pcl::PointCloud<Point> & points_raw, const std::vector<size_t> & idxs,
    const double ts);
  void getFactors(
    const gtsam::Key & key, const gtsam::Values & values, gtsam::NonlinearFactorGraph & graph,
    M66 & eigenvectors_block_matrix, V6D & degen_directions);
  void updateMap(const gtsam::Key key, const gtsam::Values & values);
  void publishClouds();
  void publishDebug();
};
}  // namespace lidar
}  // namespace mimosa
