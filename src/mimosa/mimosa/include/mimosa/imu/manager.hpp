// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// mimosa
#include "mimosa/state.hpp"
#include "mimosa/stopwatch.hpp"
#include "mimosa/utils.hpp"
#include "mimosa_msgs/msg/factor_graph_state.hpp"

// mimosa_msgs
#include "mimosa_msgs/msg/imu_manager_debug.hpp"

// ROS
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

// GTSAM
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

// C++
#include <deque>
#include <optional>

namespace mimosa
{
typedef std::pair<double, V6D> ImuMeasurement;  // < time, (acc, omega)>
typedef std::deque<ImuMeasurement> ImuBuffer;

namespace imu
{
enum class InitialYawSource
{
  SIMULATION,
  CONFIGURATION,
};

struct PreintegrationConfig
{
  float acc_noise_density = 0.0013886655606357616;
  float acc_bias_random_walk = 8.538212723310593e-05;
  float gyro_noise_density = 5.4380545102010436e-05;
  float gyro_bias_random_walk = 1.6587925152480572e-06;
  float integration_sigma = 1.0e-4;
  bool use_2nd_order_coriolis = false;
  bool use_estimated_gravity = false;
  V3D gravity = V3D(0.0, 0.0, -9.81);
  bool gravity_aligned_initialization = true;
};

void declare_config(PreintegrationConfig & config);

struct ManagerConfig
{
  std::string logs_directory = "/tmp/";
  std::string log_level = "info";
  std::string map_frame = "mimosa_map";
  std::string body_frame = "mimosa_body";
  float ts_offset = 0.0;                   // s
  bool reliable_qos = false;
  float max_buffer_duration = 2.0;         // s
  float pose_init_wait_secs = 1.0;         // s
  float interpolation_max_ts_diff = 0.01;  // s
  float extrapolation_max_ts_diff = 0.01;  // s
  float acc_scale_factor = 1.0;            // used only when IMU measurements are in units of g
  float max_acceleration_magnitude = 0.0;  // m/s^2; zero disables the limit
  InitialYawSource initial_yaw = InitialYawSource::CONFIGURATION;
  float initial_yaw_deg = 0.0;

  PreintegrationConfig preintegration;
};

void declare_config(ManagerConfig & config);

class Manager
{
private:
  const ManagerConfig config_;
  std::unique_ptr<spdlog::logger> logger_;

  // Inputs
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr initial_yaw_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Member variables
  ImuBuffer buffer_;
  std::mutex buffer_mutex_;
  std::optional<double> orientation_yaw_;
  bool has_recieved_first_message_ = false;
  std::shared_ptr<gtsam::PreintegrationParams> preintegrator_params_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> preintegrator_;
  std::mutex preintegrator_mutex_;
  mimosa_msgs::msg::ImuManagerDebug debug_msg_;

  // Propagation
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> propagation_preintegrator_;
  State propagation_base_state_;
  double propagated_upto_ts_ = 0;
  std::mutex propagation_mutex_;

  // Outputs
  rclcpp::Publisher<mimosa_msgs::msg::ImuManagerDebug>::SharedPtr pub_debug_;
  rclcpp::Publisher<mimosa_msgs::msg::FactorGraphState>::SharedPtr pub_factor_graph_state_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_localizability_marker_array_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;

  void updatePreintegrationTo(
    const double ts_start, const double ts_end, const gtsam::imuBias::ConstantBias & imu_bias,
    std::unique_ptr<gtsam::PreintegratedImuMeasurements> & preintegrator);
  std::shared_ptr<gtsam::PreintegrationParams> generatePreintegratorParams() const;
  std::shared_ptr<gtsam::PreintegrationParams> generatePreintegratorParams(
    const V3D & gravity) const;
  void addImuFactorNoLock(
    const double ts_0, const double ts_1, const gtsam::imuBias::ConstantBias & bias_0,
    const gtsam::Key key_0, const gtsam::Key key_1, gtsam::NonlinearFactorGraph & graph);
  V6D interpolateMeasurement(
    const double ts1, const V6D & meas1, const double ts2, const V6D & meas2, const double ts);
  void initialYawCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

public:
  using Ptr = std::shared_ptr<Manager>;
  Manager(rclcpp::Node & pnh);
  // * This function is public to allow other sensor managers to create their own preintegrators if needed
  inline std::shared_ptr<gtsam::PreintegrationParams> getPreintegratorParams() const
  {
    return preintegrator_params_;
  }
  inline bool hasRecievedFirstMessage() const { return has_recieved_first_message_; }
  void callback(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  bool estimateAttitude(gtsam::Rot3 & R_W_B, V3D & estimated_acc_bias, V3D & estimated_gyro_bias);
  void getInterpolatedMeasurements(
    const double ts_start, const double ts_end, ImuBuffer & measurements,
    const bool dont_interpolate_first_measurement = false);
  size_t getNumMeasurementsBetween(const double t1, const double t2);
  void addImuFactor(
    const double ts_0, const double ts_1, const gtsam::imuBias::ConstantBias & bias_0,
    const gtsam::Key key_0, const gtsam::Key key_1, gtsam::NonlinearFactorGraph & graph);
  void addImuFactorAndGetNavState(
    const State & state_0, const double ts_1, const gtsam::Key key_1,
    gtsam::NonlinearFactorGraph & graph, gtsam::NavState & ns_1);

  const ManagerConfig & config() const { return config_; }
  void setPropagationBaseState(const State & state);
  inline std::string getSubscribedTopic() const { return sub_ ? sub_->get_topic_name() : ""; }
};

}  // namespace imu
}  // namespace mimosa
