// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/lidar/geometric_config.hpp"

namespace mimosa
{
namespace lidar
{
void declare_config(RegistrationConfig & config)
{
  using namespace config;
  name("Registration Config");
  field(config.voxel_size, "voxel_size", "m");
  field(config.max_range, "max_range", "m");
  field(config.min_range, "min_range", "m");
  field(config.max_points_per_voxel, "max_points_per_voxel", "num");
  field(config.initial_threshold, "initial_threshold", "m");
  field(config.min_motion_threshold, "min_motion_threshold", "m");
  field(config.max_num_iterations, "max_num_iterations", "num");
  field(config.convergence_criterion, "convergence_criterion", "SE(3) tangent norm");
  field(config.max_num_threads, "max_num_threads", "num (0 uses TBB default)");
  field(config.deskew, "deskew", "bool");
  field(config.lidar_pose_translation_std_dev, "lidar_pose_translation_std_dev", "m");
  field(config.lidar_pose_rotation_std_dev_deg, "lidar_pose_rotation_std_dev_deg", "deg");
  field(config.lidar_information_weight, "lidar_information_weight", "scale");
  field(config.reg_4_dof, "reg_4_dof", "bool");
  field(config.project_on_degneneracy, "project_on_degneneracy", "bool");
  field(config.degen_thresh_rot, "degen_thresh_rot", "num_virtual_features");
  field(config.degen_thresh_trans, "degen_thresh_trans", "num_virtual_features");

  check(config.voxel_size, GE, 0.01, "voxel_size");
  check(config.max_range, GT, 0.0, "max_range");
  check(config.min_range, GE, 0.0, "min_range");
  checkCondition(config.min_range < config.max_range, "min_range must be smaller than max_range");
  check(config.max_points_per_voxel, GE, 1, "max_points_per_voxel");
  check(config.initial_threshold, GT, 0.0, "initial_threshold");
  check(config.min_motion_threshold, GE, 0.0, "min_motion_threshold");
  check(config.max_num_iterations, GE, 1, "max_num_iterations");
  check(config.convergence_criterion, GT, 0.0, "convergence_criterion");
  check(config.max_num_threads, GE, 0, "max_num_threads");
  check(config.lidar_pose_translation_std_dev, GT, 0.0, "lidar_pose_translation_std_dev");
  check(config.lidar_pose_rotation_std_dev_deg, GT, 0.0, "lidar_pose_rotation_std_dev_deg");
  check(config.lidar_information_weight, GT, 0.0, "lidar_information_weight");
}

void declare_config(GeometricConfig & config)
{
  using namespace config;
  name("Lidar Geometric Config");

  field(config.logs_directory, "logs_directory", "directory_path");
  field(config.map_frame, "map_frame", "str");
  field(config.body_frame, "body_frame", "str");

  {
    NameSpace ns("lidar");
    field(config.T_B_L, "T_B_S", "gtsam::Pose3");
    field(config.sensor_frame, "sensor_frame", "str");
    {
      NameSpace ns("geometric");
      field(config.enabled, "enabled", "bool");
      field(config.log_level, "log_level", "trace|debug|info|warn|error|critical");
      field(
        config.point_skip_divisor, "point_skip_divisor", "divisor for downsampling the pointcloud");
      field(
        config.ring_skip_divisor, "ring_skip_divisor", "divisor for downsampling the pointcloud");
      field(config.map_keyframe_trans_thresh, "map_keyframe_trans_thresh", "m");
      field(config.map_keyframe_rot_thresh_deg, "map_keyframe_rot_thresh_deg", "deg");
      field(config.initial_clouds_to_force_map_update, "initial_clouds_to_force_map_update", "num");
      field(config.scan_to_map, "scan_to_map");
    }
  }

  check(config.point_skip_divisor, GE, 1, "point_skip_divisor");
  check(config.ring_skip_divisor, GE, 1, "ring_skip_divisor");
}

}  // namespace lidar
}  // namespace mimosa
