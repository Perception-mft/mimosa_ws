// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

// IMU and Graph managers
#include "mimosa/graph/manager.hpp"
#include "mimosa/imu/manager.hpp"

// Exteroceptive sensor managers
#include "mimosa/lidar/manager.hpp"
#include "mimosa/odometry/manager.hpp"
#include "mimosa/radar/manager.hpp"

// ROS 2
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  config::Settings().print_missing = true;
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("mimosa_node", options);

  auto imu_manager = std::make_shared<mimosa::imu::Manager>(*node);
  auto graph_manager = std::make_shared<mimosa::graph::Manager>(*node, imu_manager);

  // Exteroceptive sensor managers
  mimosa::lidar::Manager lidar_manager(*node, imu_manager, graph_manager);
  mimosa::radar::Manager radar_manager(*node, imu_manager, graph_manager);
  mimosa::odometry::Manager odometry_manager(*node, imu_manager, graph_manager);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
