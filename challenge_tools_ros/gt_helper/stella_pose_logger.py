#!/usr/bin/env python3
"""Log /camera_pose (PoseStamped) to a TUM-format trajectory file."""
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PoseLogger(Node):
    def __init__(self, out_file: str):
        super().__init__('stella_pose_logger')
        self._f = open(out_file, 'w')
        self._f.write('# timestamp tx ty tz qx qy qz qw\n')
        self.create_subscription(PoseStamped, '/camera_pose', self._cb, 100)
        self.get_logger().info(f'Logging /camera_pose -> {out_file}')

    def _cb(self, msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        self._f.write(
            f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
            f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n'
        )
        self._f.flush()

    def destroy_node(self):
        self._f.close()
        super().destroy_node()


def main():
    if len(sys.argv) < 2:
        print('Usage: stella_pose_logger.py <output_file> [--ros-args ...]')
        sys.exit(1)
    rclpy.init()
    node = PoseLogger(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
