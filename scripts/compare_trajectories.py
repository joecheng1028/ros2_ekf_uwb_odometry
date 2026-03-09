#!/usr/bin/env python3
"""
compare_trajectories.py

Collects /uwb_pose, /odometry/filtered, and /ekf_pose trajectories
during a ROS 2 bag replay and plots them on a single figure.

All three trajectories are origin-shifted to (0, 0) at t=0 so they
can be compared on the same axes regardless of their original frames.

UWB coordinates are divided by 10 (raw values are in decimetres).

Usage:
    # Terminal 1 — start this script
    python3 compare_trajectories.py

    # Terminal 2 — run the EKF node
    ros2 run <package> ekf_node

    # Terminal 3 — replay the bag file
    ros2 bag play <path_to_bag>

    The script collects data until the bag finishes (or Ctrl+C),
    then saves the plot as trajectory_comparison.png.

Dependencies:
    pip install matplotlib
    ROS 2 Humble with rclpy, nav_msgs, geometry_msgs
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import matplotlib
matplotlib.use("Agg")   # non-interactive backend — saves to file
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np



class TrajectoryCollector(Node):
    def __init__(self):
        super().__init__("trajectory_collector")

        self.uwb_xy   = []   # raw UWB positions (after /10 conversion)
        self.odom_xy  = []   # raw odometry positions
        self.ekf_xy   = []   # EKF fused positions

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/uwb_pose",
            self.uwb_callback,
            10
        )
        self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odom_callback,
            10
        )
        self.create_subscription(
            PoseStamped,
            "/ekf_pose",
            self.ekf_callback,
            10
        )

        self.get_logger().info(
            "TrajectoryCollector running. "
            "Play the bag file now. Press Ctrl+C to stop and save plot."
        )

    def uwb_callback(self, msg: PoseWithCovarianceStamped):
        # Raw UWB values are in decimetres — divide by 10 to get metres
        x = msg.pose.pose.position.x / 10.0
        y = msg.pose.pose.position.y / 10.0
        self.uwb_xy.append((x, y))

    def odom_callback(self, msg: Odometry):
        # Negate x and y to match UWB frame convention (same as Script 1)
        x = -msg.pose.pose.position.x
        y = -msg.pose.pose.position.y
        self.odom_xy.append((x, y))

    def ekf_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.ekf_xy.append((x, y))


def origin_shift(points):
    """Shift a list of (x, y) tuples so the first point is at (0, 0)."""
    if not points:
        return points
    x0, y0 = points[0]
    return [(x - x0, y - y0) for x, y in points]


def save_plot(node: TrajectoryCollector):
    # UWB: origin-shift using its own first point (node handles /10, plot handles shift)
    uwb_raw  = node.uwb_xy
    if uwb_raw:
        x0, y0 = uwb_raw[0]
        uwb = [(x - x0, y - y0) for x, y in uwb_raw]
    else:
        uwb = []

    # Odometry: origin-shift using its own first point
    odom = origin_shift(node.odom_xy)

    # EKF: already origin-shifted by the node (starts at 0,0)
    ekf = node.ekf_xy

    node.get_logger().info(
        f"Collected — UWB: {len(uwb)} pts | "
        f"Odometry: {len(odom)} pts | "
        f"EKF: {len(ekf)} pts"
    )

    if not any([uwb, odom, ekf]):
        node.get_logger().warn("No data collected — plot not saved.")
        return

    fig, ax = plt.subplots(figsize=(10, 8))

    if odom:
        ox, oy = zip(*odom)
        ax.plot(ox, oy, color="#aaaaaa", linewidth=0.8,
                label="Raw Odometry (/odometry/filtered)", zorder=1)

    if uwb:
        ux, uy = zip(*uwb)
        ax.scatter(ux, uy, color="#e63946", s=6, alpha=0.6,
                   label="Raw UWB (/uwb_pose)", zorder=2)

    if ekf:
        ex, ey = zip(*ekf)
        ax.plot(ex, ey, color="#2e86ab", linewidth=1.5,
                label="EKF Fused (/ekf_pose)", zorder=3)

    # Mark origin
    ax.plot(0, 0, "k*", markersize=10, label="Start (origin)", zorder=4)

    ax.set_xlabel("X (metres)", fontsize=11)
    ax.set_ylabel("Y (metres)", fontsize=11)
    ax.set_title(
        "Trajectory Comparison — Raw Odometry vs Raw UWB vs EKF Fused\n"
        "LibraryRun_High_M4 | TU Chemnitz | 2025",
        fontsize=12
    )
    ax.legend(fontsize=10, loc="best")
    ax.set_aspect("equal")
    ax.grid(True, linestyle="--", alpha=0.4)

    output_path = "trajectory_comparison.png"
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    node.get_logger().info(f"Plot saved to {output_path}")
    plt.close(fig)


def main():
    rclpy.init()
    node = TrajectoryCollector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    print("\nStopping — saving plot...")
    save_plot(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
