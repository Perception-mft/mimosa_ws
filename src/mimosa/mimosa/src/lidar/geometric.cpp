// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/lidar/geometric.hpp"

namespace mimosa
{
namespace lidar
{
Geometric::Geometric(rclcpp::Node & pnh)
: config(config::checkValid(config::fromRos<GeometricConfig>(pnh)))
{
  // Prepare config
  logger_ = createLogger(
    config.logs_directory + "lidar_geometric.log", "lidar::Geometric", config.log_level);
  logger_->info("lidar::Geometric initialized with params:\n {}", config::toString(config));

  Be_cloud_.reset(new pcl::PointCloud<Point>);
  kiss_icp::pipeline::KISSConfig kiss_config;
  kiss_config.voxel_size = config.scan_to_map.voxel_size;
  kiss_config.max_range = config.scan_to_map.max_range;
  kiss_config.min_range = config.scan_to_map.min_range;
  kiss_config.max_points_per_voxel = config.scan_to_map.max_points_per_voxel;
  kiss_config.min_motion_th = config.scan_to_map.min_motion_threshold;
  kiss_config.initial_threshold = config.scan_to_map.initial_threshold;
  kiss_config.max_num_iterations = config.scan_to_map.max_num_iterations;
  kiss_config.convergence_criterion = config.scan_to_map.convergence_criterion;
  kiss_config.max_num_threads = config.scan_to_map.max_num_threads;
  kiss_config.deskew = config.scan_to_map.deskew;
  kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(kiss_config);

  pub_sm_cloud_ = pnh.create_publisher<sensor_msgs::msg::PointCloud2>("lidar/geometric/sm_cloud", 1);
  pub_sm_cloud_ds_ = pnh.create_publisher<sensor_msgs::msg::PointCloud2>("lidar/geometric/sm_cloud_ds", 1);
  pub_sm_correspondances_ma_ =
    pnh.create_publisher<visualization_msgs::msg::MarkerArray>("lidar/geometric/sm_correspondances_ma", 1);
  pub_debug_ = pnh.create_publisher<mimosa_msgs::msg::LidarGeometricDebug>("lidar/geometric/debug", 1);
  pub_map_ = pnh.create_publisher<sensor_msgs::msg::PointCloud2>("lidar/geometric/map", 1);
  pub_localizability_marker_array_ = pnh.create_publisher<visualization_msgs::msg::MarkerArray>(
    "lidar/geometric/localizability_marker_array", rclcpp::QoS(1).transient_local());
  pub_degen_marker_array_ =
    pnh.create_publisher<visualization_msgs::msg::MarkerArray>("lidar/geometric/degen_marker_array", rclcpp::QoS(1).transient_local());
  pub_keyframe_poses_ =
    pnh.create_publisher<geometry_msgs::msg::PoseArray>("lidar/geometric/keyframe_poses", 1);

  keyframe_poses_.header.frame_id = config.map_frame;
}

void Geometric::preprocess(
  const pcl::PointCloud<Point> & points_raw, const std::vector<size_t> & idxs, const double ts)
{
  if (!config.enabled) return;

  Stopwatch sw;
  logger_->trace("Preprocess start");
  ts_ = ts;
  Be_cloud_->clear();
  Be_cloud_->reserve(points_raw.size());
  kiss_frame_.clear();
  kiss_frame_.reserve(points_raw.size());
  kiss_timestamps_.clear();
  kiss_timestamps_.reserve(points_raw.size());

  const M3F R_B_L = config.T_B_L.rotation().matrix().cast<float>();
  const V3F t_B_L = config.T_B_L.translation().cast<float>();

  // The standalone KISS frontend gives the pipeline every input point. KISS's
  // own range filter and two-stage voxelizer perform all scan reduction.
  (void)idxs;
  for (const Point & input : points_raw) {
    auto & p = Be_cloud_->points.emplace_back(input);
    p.getVector3fMap() = R_B_L * p.getVector3fMap() + t_B_L;
    kiss_frame_.emplace_back(p.getVector3fMap().cast<double>());
    kiss_timestamps_.emplace_back(static_cast<double>(p.t));
  }
  Be_cloud_->width = Be_cloud_->size();
  Be_cloud_->height = 1;

  // This is the same normalization done by KISS-ICP's ROS adapter.
  if (kiss_timestamps_.size() > 1) {
    const auto [min_it, max_it] =
      std::minmax_element(kiss_timestamps_.cbegin(), kiss_timestamps_.cend());
    const double min_time = *min_it;
    const double duration = *max_it - min_time;
    if (duration > 0.0) {
      for (double & stamp : kiss_timestamps_) stamp = (stamp - min_time) / duration;
    } else {
      kiss_timestamps_.clear();
    }
  }

  debug_msg_.n_points_in = points_raw.size();
  debug_msg_.n_points_in_sm_ds = 0;

  logger_->trace("Preprocess end");
  debug_msg_.t_preprocess = sw.elapsedMs();
}

void Geometric::publishClouds()
{
  if (pub_sm_cloud_->get_subscription_count()) {
    publishCloud(pub_sm_cloud_, *Be_cloud_, config.body_frame, ts_);
  }

  if (pub_sm_cloud_ds_->get_subscription_count()) {
    publishCloud(pub_sm_cloud_ds_, sm_Be_cloud_ds_, config.body_frame, ts_);
  }
}

void Geometric::getFactors(
  const gtsam::Key & key, const gtsam::Values & values, gtsam::NonlinearFactorGraph & graph,
  M66 & eigenvectors_block_matrix, V6D & degen_directions)
{
  if (!config.enabled) return;

  Stopwatch sw;

  (void)values;
  if (kiss_frame_.empty()) {
    logger_->warn("Skipping KISS-ICP factor for an empty LiDAR frame");
    factor_.reset();
    eigenvectors_block_matrix.setIdentity();
    degen_directions.setOnes();
    return;
  }

  // This single call is the exact standalone KISS-ICP algorithm. Internally it
  // performs deskew, two-stage voxelization, adaptive-threshold registration,
  // constant-velocity prediction, map insertion, and delta/pose updates.
  const auto [frame, source] = kiss_icp_->RegisterFrame(kiss_frame_, kiss_timestamps_);

  Be_cloud_->clear();
  Be_cloud_->reserve(frame.size());
  for (const V3D & p : frame) {
    Point point;
    point.getVector3fMap() = p.cast<float>();
    Be_cloud_->push_back(point);
  }
  sm_Be_cloud_ds_.clear();
  sm_Be_cloud_ds_.reserve(source.size());
  for (const V3D & p : source) {
    Point point;
    point.getVector3fMap() = p.cast<float>();
    sm_Be_cloud_ds_.push_back(point);
  }
  debug_msg_.n_points_in_sm_ds = source.size();

  const Sophus::SE3d & kiss_pose = kiss_icp_->pose();
  if (!kiss_pose.matrix().allFinite()) {
    logger_->error("KISS-ICP returned a non-finite pose; skipping its GTSAM factor");
    factor_.reset();
    eigenvectors_block_matrix.setIdentity();
    degen_directions.setOnes();
    return;
  }
  const gtsam::Pose3 T_K_B(
    gtsam::Rot3(kiss_pose.rotationMatrix()), kiss_pose.translation());
  const gtsam::Pose3 measured_pose = T_W_K_ * T_K_B;
  factor_ = std::make_shared<KISSICPFactor>(key, measured_pose, config.scan_to_map);

  if (pub_sm_correspondances_ma_->get_subscription_count()) {
    visualization_msgs::msg::MarkerArray ma;
    fillMarkerArray(*factor_, ma, config.map_frame, ts_);
    pub_sm_correspondances_ma_->publish(ma);
  }

  // Get the localizability
  V3D localizability_trans_comp, localizability_rot_comp, localizability_trans_final,
    localizability_rot_final;
  M33 eigenvectors_trans, eigenvectors_rot;
  factor_->getLocalizabilities(
    localizability_trans_comp, localizability_rot_comp, localizability_trans_final,
    localizability_rot_final, eigenvectors_trans, eigenvectors_rot);

  {
    // M33 degen_eigenvectors_trans, degen_eigenvectors_rot;
    // V3D degen_rot, degen_trans;

    // factor_->getDegenInfo(degen_rot, degen_eigenvectors_rot, degen_trans, degen_eigenvectors_trans);

    eigenvectors_block_matrix.setZero();
    eigenvectors_block_matrix.block<3, 3>(0, 0) = eigenvectors_rot;
    eigenvectors_block_matrix.block<3, 3>(3, 3) = eigenvectors_trans;

    degen_directions.setZero();
    degen_directions(0) = localizability_rot_comp(0) < config.scan_to_map.degen_thresh_rot;
    degen_directions(1) = localizability_rot_comp(1) < config.scan_to_map.degen_thresh_rot;
    degen_directions(2) = localizability_rot_comp(2) < config.scan_to_map.degen_thresh_rot;
    degen_directions(3) = localizability_trans_comp(0) < config.scan_to_map.degen_thresh_trans;
    degen_directions(4) = localizability_trans_comp(1) < config.scan_to_map.degen_thresh_trans;
    degen_directions(5) = localizability_trans_comp(2) < config.scan_to_map.degen_thresh_trans;

    // debug_msg_.degen_rot_val[0] = degen_rot(0);
    // debug_msg_.degen_rot_val[1] = degen_rot(1);
    // debug_msg_.degen_rot_val[2] = degen_rot(2);
    // debug_msg_.degen_trans_val[0] = degen_trans(0);
    // debug_msg_.degen_trans_val[1] = degen_trans(1);
    // debug_msg_.degen_trans_val[2] = degen_trans(2);
    debug_msg_.degen_rot_bool[0] = degen_directions(0);
    debug_msg_.degen_rot_bool[1] = degen_directions(1);
    debug_msg_.degen_rot_bool[2] = degen_directions(2);
    debug_msg_.degen_trans_bool[0] = degen_directions(3);
    debug_msg_.degen_trans_bool[1] = degen_directions(4);
    debug_msg_.degen_trans_bool[2] = degen_directions(5);

    // logger_->info("degen_directions: {}", degen_directions.transpose());

    if (pub_degen_marker_array_->get_subscription_count()) {
      // Create the marker for the axis
      visualization_msgs::msg::MarkerArray ma;
      addTriadMarker(eigenvectors_trans, config.body_frame, ts_, "DegenTrans", ma);
      addTriadMarker(eigenvectors_rot, config.body_frame, ts_, "DegenRot", ma);

      pub_degen_marker_array_->publish(ma);
    }
  }

  // // Enforce convention of positive x for the leading eigenvector
  // // This does not make a difference to the localizability
  // if (eigenvectors_trans(0, 0) < 0) {
  //   // Flip all the eigenvectors
  //   eigenvectors_trans *= -1;
  // }
  // if (eigenvectors_rot(0, 0) < 0) {
  //   // Flip all the eigenvectors
  //   eigenvectors_rot *= -1;
  // }

  if (pub_localizability_marker_array_->get_subscription_count()) {
    // Create the marker for the axis
    visualization_msgs::msg::MarkerArray ma;
    addTriadMarker(eigenvectors_trans, config.body_frame, ts_, "LocalizabilityTrans", ma);
    addTriadMarker(eigenvectors_rot, config.body_frame, ts_, "LocalizabilityRot", ma);

    pub_localizability_marker_array_->publish(ma);
  }

  convert(localizability_trans_comp, debug_msg_.localizability_trans_comp);
  convert(localizability_rot_comp, debug_msg_.localizability_rot_comp);
  convert(localizability_trans_final, debug_msg_.localizability_trans_final);
  convert(localizability_rot_final, debug_msg_.localizability_rot_final);

  const std::vector<KISSICPFactor::RejectStatus> & statuses = factor_->getStatuses();
  debug_msg_.n_unprocessed = 0;
  debug_msg_.n_rejected_insufficient_corres_points = 0;
  debug_msg_.n_rejected_max_corres_dist = 0;
  debug_msg_.n_rejected_eigensolver_fail = 0;
  debug_msg_.n_rejected_min_eigen_value_low = 0;
  debug_msg_.n_rejected_line = 0;
  debug_msg_.n_rejected_plane = 0;
  debug_msg_.n_rejected_max_error = 0;
  debug_msg_.n_correspondances = 0;
  for (size_t i = 0; i < statuses.size(); i++) {
    switch (statuses[i]) {
      case KISSICPFactor::RejectStatus::Unprocessed:
        debug_msg_.n_unprocessed++;
        break;
      case KISSICPFactor::RejectStatus::InsufficientCorresPoints:
        debug_msg_.n_rejected_insufficient_corres_points++;
        break;
      case KISSICPFactor::RejectStatus::CorresMaxDist:
        debug_msg_.n_rejected_max_corres_dist++;
        break;
      case KISSICPFactor::RejectStatus::EigenSolverFail:
        debug_msg_.n_rejected_eigensolver_fail++;
        break;
      case KISSICPFactor::RejectStatus::MinEigenValueLow:
        debug_msg_.n_rejected_min_eigen_value_low++;
        break;
      case KISSICPFactor::RejectStatus::Line:
        debug_msg_.n_rejected_line++;
        break;
      case KISSICPFactor::RejectStatus::CorresPlaneInvalid:
        debug_msg_.n_rejected_plane++;
        break;
      case KISSICPFactor::RejectStatus::MaxError:
        debug_msg_.n_rejected_max_error++;
        break;
      case KISSICPFactor::RejectStatus::Valid:
        debug_msg_.n_correspondances++;
        break;

      default:
        break;
    }
  }

  graph.add(factor_);

  debug_msg_.t_get_factors = sw.elapsedMs();
}

void Geometric::fillMarkerArray(
  const KISSICPFactor & factor, visualization_msgs::msg::MarkerArray & ma,
  const std::string & frame_id, const double ts)
{
  const std::vector<KISSICPFactor::RejectStatus> & statuses = factor.getStatuses();
  const std::vector<V3D> & corres_means_target = factor.getCorresMeansTarget();
  const std::vector<V3D> & transformed_source = factor.getTransformedSource();

  visualization_msgs::msg::Marker triangles;
  triangles.header.frame_id = frame_id;
  triangles.header.stamp = toStamp(ts);
  triangles.ns = "kiss_icp_targets";
  triangles.id = 0;
  triangles.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  triangles.action = visualization_msgs::msg::Marker::ADD;
  triangles.pose.orientation.w = 1.0;
  triangles.scale.x = 1.0;
  triangles.scale.y = 1.0;
  triangles.scale.z = 1.0;
  triangles.color.r = 0.0;
  triangles.color.g = 1.0;
  triangles.color.b = 0.0;
  triangles.color.a = 1.0;

  visualization_msgs::msg::Marker normals;
  normals.header.frame_id = frame_id;
  normals.header.stamp = toStamp(ts);
  normals.ns = "kiss_icp_correspondences";
  normals.id = 0;
  normals.type = visualization_msgs::msg::Marker::LINE_LIST;
  normals.action = visualization_msgs::msg::Marker::ADD;
  normals.pose.orientation.w = 1.0;
  normals.scale.x = 0.01;
  normals.color.r = 1.0;
  normals.color.g = 0.0;
  normals.color.b = 0.0;
  normals.color.a = 1.0;

  float triangle_size = config.scan_to_map.voxel_size;

  for (size_t i = 0; i < corres_means_target.size(); i++) {
    if (statuses[i] != KISSICPFactor::RejectStatus::Valid) {
      continue;
    }

    // Target point and its point-to-point KISS-ICP correspondence.
    const Eigen::Vector3d & p = corres_means_target[i];
    const Eigen::Vector3d delta = transformed_source[i] - p;
    const Eigen::Vector3d n =
      delta.squaredNorm() > 1e-12 ? delta.normalized() : Eigen::Vector3d::UnitZ();

    // Create vertices for the triangle with centroid p
    Eigen::Vector3d u = n.unitOrthogonal();       // Vector orthogonal to the normal
    Eigen::Vector3d v = n.cross(u).normalized();  // Another orthogonal vector

    // Scale the vectors to half the triangle size (since p is the centroid)
    u *= triangle_size / 2.0;
    v *= triangle_size / 2.0;

    // Vertices of the triangle
    Eigen::Vector3d p1 = p + u + v;
    Eigen::Vector3d p2 = p - u + v;
    Eigen::Vector3d p3 = p - u - v;

    // Add the three vertices of the triangle
    geometry_msgs::msg::Point vertex;
    vertex.x = p1.x();
    vertex.y = p1.y();
    vertex.z = p1.z();
    triangles.points.push_back(vertex);

    vertex.x = p2.x();
    vertex.y = p2.y();
    vertex.z = p2.z();
    triangles.points.push_back(vertex);

    vertex.x = p3.x();
    vertex.y = p3.y();
    vertex.z = p3.z();
    triangles.points.push_back(vertex);

    // Draw the residual from the map point to the transformed scan point.
    geometry_msgs::msg::Point start, end;
    start.x = p.x();
    start.y = p.y();
    start.z = p.z();
    end.x = transformed_source[i].x();
    end.y = transformed_source[i].y();
    end.z = transformed_source[i].z();

    normals.points.push_back(start);
    normals.points.push_back(end);
  }

  ma.markers.push_back(triangles);
  ma.markers.push_back(normals);
}

void Geometric::updateMap(const gtsam::Key key, const gtsam::Values & values)
{
  if (!config.enabled) return;

  Stopwatch sw;

  if (factor_ != nullptr) {
    debug_msg_.n_linearize_calls = factor_->getLinearizeCount();
  }

  const gtsam::Pose3 & T_W_Be = values.at<gtsam::Pose3>(key);

  if (!kiss_pose_initialized_) {
    // KISS keeps its own odometry frame with the first pose at identity. T_W_K
    // connects that frame to Mimosa's graph/world frame for the GTSAM factor.
    T_W_K_ = T_W_Be;
    if (!kiss_frame_.empty()) {
      const auto [frame, source] = kiss_icp_->RegisterFrame(kiss_frame_, kiss_timestamps_);
      Be_cloud_->clear();
      Be_cloud_->reserve(frame.size());
      for (const V3D & p : frame) {
        Point point;
        point.getVector3fMap() = p.cast<float>();
        Be_cloud_->push_back(point);
      }
      sm_Be_cloud_ds_.clear();
      sm_Be_cloud_ds_.reserve(source.size());
      for (const V3D & p : source) {
        Point point;
        point.getVector3fMap() = p.cast<float>();
        sm_Be_cloud_ds_.push_back(point);
      }
      debug_msg_.n_points_in_sm_ds = source.size();
    }
    kiss_pose_initialized_ = true;
  }

  bool update_map = true;
  if (map_poses_.size()) {
    // Check if the pose has changed significantly with respect to the poses that are already in the map
    float min_diff_trans = std::numeric_limits<float>::max();
    size_t min_diff_index = 0;
    size_t pose_index = 0;
    for (const auto & pose : map_poses_) {
      float diff_trans = (pose.translation() - T_W_Be.translation()).norm();
      if (diff_trans < min_diff_trans) {
        min_diff_trans = diff_trans;
        min_diff_index = pose_index;
      }
      pose_index++;
    }

    auto rot_diff = config.T_B_L.rotation().inverse() *
                    map_poses_[min_diff_index].rotation().between(T_W_Be.rotation()) *
                    config.T_B_L.rotation();

    V3D ypr = rot_diff.ypr().cwiseAbs();

    // Rules for global map update:
    // 1. First update should always happen
    // 2. If there is significant difference in the information content of the scan with respect to what is already in the map
    //        - Determining this is hard, but we use the change in the pose as a proxy
    // If the pose is close to an existing pose, then do not update the map
    if (min_diff_trans > config.map_keyframe_trans_thresh) {
      update_map = true;
    } else if (ypr.maxCoeff() > DEG2RAD(config.map_keyframe_rot_thresh_deg)) {
      update_map = true;
    } else {
      update_map = false;
    }
  }

  static size_t initial_clouds_to_force_map_update = config.initial_clouds_to_force_map_update;
  if (initial_clouds_to_force_map_update > 0) {
    update_map = true;
    initial_clouds_to_force_map_update--;
  }

  if (update_map) {
    Stopwatch sw;
    // The KISS map is updated for every frame in getFactors(). Keyframes are
    // retained only for Mimosa's visualization and bookkeeping.
    debug_msg_.t_insertion = sw.lapMs();

    map_poses_.push_back(T_W_Be);
    geometry_msgs::msg::Pose p;
    convert(T_W_Be, p);
    keyframe_poses_.poses.push_back(p);

    if (pub_map_->get_subscription_count()) {
      // Publish the updated map
      pcl::PointCloud<Point> W_map;
      for (const V3D & point_K : kiss_icp_->LocalMap()) {
        const V3D point_W = T_W_K_ * point_K;
        Point point;
        point.getVector3fMap() = point_W.cast<float>();
        W_map.push_back(point);
      }
      publishCloud(pub_map_, W_map, config.map_frame, ts_);
    }

    keyframe_poses_.header.stamp = toStamp(ts_);
    pub_keyframe_poses_->publish(keyframe_poses_);
  }
  debug_msg_.t_update_map = sw.elapsedMs();
}

void Geometric::publishDebug()
{
  debug_msg_.header.stamp = toStamp(ts_);
  pub_debug_->publish(debug_msg_);
}

}  // namespace lidar
}  // namespace mimosa
