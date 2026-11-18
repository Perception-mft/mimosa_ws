#!/usr/bin/env python3
"""Generic ROS 2 launch entry point for all MIMOSA sensor profiles."""

import json
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PROFILES = {
    "enwide": ("/ouster/imu", "/ouster/points", "", "", "os_enwide.json"),
    "euroc": ("/imu0", "", "", "/rovio/odometry", "../os_dummy.json"),
    "hornbill": ("/vectornav_driver_node/imu/data", "/ouster/points", "/radar/cloud", "", "os_hornbill.json"),
    "lapwing": ("/vectornav_driver_node/imu/data", "/livox/lidar", "/radar/cloud", "/rovio/odometry", "../os_dummy.json"),
    "magpie": ("/vectornav_driver_node/imu/data", "/rslidar_points", "/radar/cloud", "", "../os_dummy.json"),
    "newer_college": ("/os_cloud_node/imu", "/os_cloud_node/points", "", "", "os_newer_college.json"),
    "parrot": ("/vectornav_driver_node/imu/data", "/lidar_points", "/radar/cloud", "/rovio/odometry", "../os_dummy.json"),
    "m113": ("/sensing/imu/imu_error", "/sensing/lidar/Front/pointcloud_raw", "/radar/cloud", "", "../os_dummy.json"),
}


def _flatten(value, prefix=""):
    result = {}
    for key, item in value.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(item, dict):
            result.update(_flatten(item, name))
        else:
            if isinstance(item, list):
                if any(isinstance(element, (list, dict)) for element in item):
                    # ROS 2 parameters do not support nested arrays. The config
                    # adapter recognizes this tagged YAML representation.
                    item = "__yaml__:" + yaml.safe_dump(item, default_flow_style=True).strip()
                elif any(isinstance(element, float) for element in item):
                    # ROS 2 arrays are homogeneous; normalize mixed int/float arrays.
                    item = [float(element) for element in item]
            result[name] = item
    return result


def _launch(context):
    profile = LaunchConfiguration("profile").perform(context)
    if profile not in PROFILES:
        raise RuntimeError(f"Unknown MIMOSA profile '{profile}'. Choose from: {', '.join(PROFILES)}")

    share = get_package_share_directory("mimosa")
    profile_dir = os.path.join(share, "config", profile)
    with open(os.path.join(profile_dir, "params.yaml"), encoding="utf-8") as stream:
        params = _flatten(yaml.safe_load(stream))

    with open(os.path.join(share, "model", "m113.urdf"), encoding="utf-8") as stream:
        robot_description = stream.read()

    metadata_path = os.path.normpath(os.path.join(profile_dir, PROFILES[profile][4]))
    with open(metadata_path, encoding="utf-8") as stream:
        params.update(_flatten(json.load(stream), "lidar.sensor"))

    override = LaunchConfiguration("config_override").perform(context)
    if override:
        with open(override, encoding="utf-8") as stream:
            params.update(_flatten(yaml.safe_load(stream)))

    params["use_sim_time"] = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    defaults = list(PROFILES[profile][:4])
    use_imu_error = LaunchConfiguration("imu_error").perform(context).lower() == "true"
    imu_error_input_topic = LaunchConfiguration("imu_error_input_topic").perform(context)
    imu_error_output_topic = LaunchConfiguration("imu_error_output_topic").perform(context)
    if profile == "m113":
        defaults[0] = imu_error_output_topic if use_imu_error else imu_error_input_topic

    inputs = [
        LaunchConfiguration("imu_topic").perform(context) or defaults[0],
        LaunchConfiguration("lidar_topic").perform(context) or defaults[1],
        LaunchConfiguration("radar_topic").perform(context) or defaults[2],
        LaunchConfiguration("odometry_topic").perform(context) or defaults[3],
    ]
    internal = [
        "imu/manager/imu_in", "lidar/manager/lidar_in",
        "radar/manager/radar_in", "odometry/manager/odometry_in",
    ]
    remappings = [(source, target) for source, target in zip(internal, inputs) if target]

    rviz_env = {}
    if LaunchConfiguration("rviz_gpu").perform(context).lower() == "true":
        rviz_env = {
            "QT_OPENGL": "desktop",
            "LIBGL_ALWAYS_SOFTWARE": "0",
            "DRI_PRIME": "1",
            "__NV_PRIME_RENDER_OFFLOAD": "1",
            "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
            "__VK_LAYER_NV_optimus": "NVIDIA_only",
        }

    nodes = []
    if profile == "m113" and use_imu_error:
        nodes.append(
            Node(
                package="imu_error", executable="imu_error", name="imu_error",
                output="screen",
                parameters=[{
                    "input_topic": imu_error_input_topic,
                    "output_topic": imu_error_output_topic,
                    "use_sim_time": params["use_sim_time"],
                }],
            )
        )

    nodes.extend([
        Node(
            package="mimosa", executable="mimosa_node", name="mimosa_node",
            namespace="mimosa_node", output="screen", parameters=[params],
            remappings=remappings,
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="mimosa_robot_state_publisher", output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": params["use_sim_time"],
            }],
            condition=IfCondition(LaunchConfiguration("viz")),
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", os.path.join(share, "rviz", "mimosa.rviz")],
            additional_env=rviz_env,
            condition=IfCondition(LaunchConfiguration("viz")), output="screen",
        ),
    ])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="hornbill"),
        DeclareLaunchArgument("viz", default_value="true"),
        DeclareLaunchArgument("rviz_gpu", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("config_override", default_value=""),
        DeclareLaunchArgument("imu_error", default_value="false"),
        DeclareLaunchArgument("imu_error_input_topic", default_value="/sensing/imu/imu_raw"),
        DeclareLaunchArgument("imu_error_output_topic", default_value="/sensing/imu/imu_error"),
        DeclareLaunchArgument("imu_topic", default_value=""),
        DeclareLaunchArgument("lidar_topic", default_value=""),
        DeclareLaunchArgument("radar_topic", default_value=""),
        DeclareLaunchArgument("odometry_topic", default_value=""),
        OpaqueFunction(function=_launch),
    ])
