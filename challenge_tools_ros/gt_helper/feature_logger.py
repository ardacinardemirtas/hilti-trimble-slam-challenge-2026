#!/usr/bin/env python3
"""
Log OpenVINS feature tracks to a file for later COLMAP conversion.

Subscribes to the `loop_feats` topic (sensor_msgs/PointCloud) published by
OpenVINS at each camera frame. Each message contains:
  - points[i].{x,y,z}       : 3D position of feature i in world frame
  - channels[i].values[2]    : raw pixel u (cam0)
  - channels[i].values[3]    : raw pixel v (cam0)
  - channels[i].values[4]    : feature ID (stable integer across frames)

By accumulating these messages we reconstruct full tracks:
  feature_id → list of (timestamp_s, u, v, X, Y, Z)

Output file format (TSV, one observation per line):
  feat_id  timestamp_s  u  v  X_world  Y_world  Z_world

Usage (inside ROS2/Apptainer):
  python3 feature_logger.py <output.txt> --ros-args -p use_sim_time:=true
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud


class FeatureLogger(Node):
    def __init__(self, output_filename):
        super().__init__('feature_logger')
        self.sub = self.create_subscription(
            PointCloud, '/ov_msckf/loop_feats', self.callback, 10
        )
        self.out = open(output_filename, 'w')
        self.out.write('# feat_id timestamp_s u v X_world Y_world Z_world\n')
        self.out.flush()
        self.n_obs = 0
        self.get_logger().info(f'Logging features to: {output_filename}')

    def callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for point, channel in zip(msg.points, msg.channels):
            if len(channel.values) < 5:
                continue
            u        = channel.values[2]
            v        = channel.values[3]
            feat_id  = int(channel.values[4])
            x, y, z  = point.x, point.y, point.z
            self.out.write(f'{feat_id} {ts:.9f} {u:.4f} {v:.4f} '
                           f'{x:.9f} {y:.9f} {z:.9f}\n')
        self.out.flush()
        self.n_obs += len(msg.points)
        if self.n_obs % 10000 < len(msg.points):
            self.get_logger().info(f'{self.n_obs} observations logged')


def main(args=None):
    rclpy.init(args=args)
    cli_args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    if not cli_args:
        print('Usage: feature_logger.py <output.txt> --ros-args -p use_sim_time:=true')
        rclpy.shutdown()
        return
    node = FeatureLogger(cli_args[0])
    rclpy.spin(node)
    node.out.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
