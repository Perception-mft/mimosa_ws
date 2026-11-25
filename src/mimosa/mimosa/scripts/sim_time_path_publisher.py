#!/usr/bin/env python3
"""
Sim-Time Path Publisher - Publishes a growing nav_msgs/Path from a CSV of
absolute-timestamped positions, synced to ROS sim time (/clock).

Unlike tum_path_publisher.py (which republishes a whole static path at a
fixed wall-clock rate), this node only appends poses whose timestamp has
already been reached in sim time, so the path grows in step with a
`ros2 bag play --clock` replay it's run alongside. Needs use_sim_time:=true
and a /clock source (e.g. bag playback) to advance at all.

CSV format: t_abs,x,y,z (no header requirement beyond matching these names).
"""
import argparse
import csv

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Publish a growing Path from a CSV, synced to sim time"
    )
    parser.add_argument("csv_file", type=str, help="CSV with t_abs,x,y,z columns")
    parser.add_argument("--frame_id", type=str, default="mimosa_map")
    parser.add_argument("--topic_name", type=str, default="gt/path")
    parser.add_argument("--hz", type=float, default=5.0, help="Publish rate in Hz")
    return parser.parse_args()


def load_rows(csv_file):
    rows = []
    with open(csv_file) as f:
        for row in csv.DictReader(f):
            rows.append((float(row["t_abs"]), float(row["x"]), float(row["y"]), float(row["z"])))
    rows.sort(key=lambda r: r[0])
    return rows


def main():
    args = parse_arguments()
    rclpy.init()
    node = Node("sim_time_path_publisher")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    rows = load_rows(args.csv_file)
    node.get_logger().info(f"Loaded {len(rows)} poses from {args.csv_file}")

    pub = node.create_publisher(
        Path, args.topic_name, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    )

    path_msg = Path()
    path_msg.header.frame_id = args.frame_id
    state = {"idx": 0, "started": False}

    def on_timer():
        now = node.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return  # no /clock received yet
        if not state["started"]:
            node.get_logger().info(f"Sim clock active at t={now:.3f}, starting playback")
            state["started"] = True

        idx = state["idx"]
        n = len(rows)
        advanced = False
        while idx < n and rows[idx][0] <= now:
            t, x, y, z = rows[idx]
            ps = PoseStamped()
            ps.header.frame_id = args.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
            idx += 1
            advanced = True
        state["idx"] = idx

        if advanced:
            path_msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(path_msg)
            if idx == n:
                node.get_logger().info(f"Full path published ({n} poses) at sim t={now:.3f}")

    node.create_timer(1.0 / args.hz, on_timer)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
