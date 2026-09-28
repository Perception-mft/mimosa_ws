// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "mimosa/lidar/utils.hpp"
#include "mimosa/utils.hpp"

namespace mimosa
{
namespace lidar
{

struct RegistrationConfig
{
  float voxel_size = 1.0;
  float max_range = 100.0;
  float min_range = 0.0;
  size_t max_points_per_voxel = 20;
  float initial_threshold = 2.0;
  float min_motion_threshold = 0.1;
  int max_num_iterations = 500;
  float convergence_criterion = 1e-4;
  int max_num_threads = 0;
  bool deskew = true;
  float lidar_pose_translation_std_dev = 0.10;
  float lidar_pose_rotation_std_dev_deg = 2.0;
  float lidar_information_weight = 1.0;
  bool reg_4_dof = false;
  bool project_on_degneneracy = true;
  float degen_thresh_rot = 10;
  float degen_thresh_trans = 15;
};

void declare_config(RegistrationConfig & config);

struct GeometricConfig
{
  std::string logs_directory = "/tmp/";
  bool enabled = true;
  std::string log_level = "info";

  std::string map_frame = "mimosa_map";
  std::string sensor_frame = "mimosa_lidar";
  std::string body_frame = "mimosa_body";
  gtsam::Pose3 T_B_L = gtsam::Pose3::Identity();

  int point_skip_divisor = 1;
  int ring_skip_divisor = 1;
  float map_keyframe_trans_thresh = 0.1;
  float map_keyframe_rot_thresh_deg = 10;
  size_t initial_clouds_to_force_map_update = 10;
  RegistrationConfig scan_to_map;
};

void declare_config(GeometricConfig & config);
}  // namespace lidar
}  // namespace mimosa
