#!/usr/bin/env python3
"""
Extract IMU data from a ROS2 bag → numpy array for vi_optimization.py.

Output format matches vi_optimization.py's raw_imu_data:
  shape (N, 7): [timestamp_ns, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
  timestamps are int64 nanoseconds

Usage (inside ROS2/Apptainer container with rosbag2_py available):
  python3 bag_to_imu_npy.py <bag_dir> <output.npy>

Example:
  python3 bag_to_imu_npy.py \\
      /data/floor_1/2025-05-05/run_1/rosbag/ \\
      floor_1_2025-05-05_run_1_imu.npy
"""

import sys
import numpy as np

IMU_TOPIC = '/imu/data_raw'


def extract_imu(bag_dir, out_path):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu
        import rclpy
    except ImportError:
        print('ERROR: rosbag2_py not available. Run inside the ROS2 Apptainer container.')
        sys.exit(1)

    storage = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    conv    = rosbag2_py.ConverterOptions('', '')
    reader  = rosbag2_py.SequentialReader()
    reader.open(storage, conv)

    rclpy.init()
    rows = []
    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        if topic != IMU_TOPIC:
            continue
        msg = deserialize_message(data, Imu)
        # Use header stamp (bag clock) converted to ns
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        rows.append([
            stamp_ns,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
    rclpy.shutdown()

    arr = np.array(rows, dtype=np.float64)
    arr[:, 0] = arr[:, 0].astype(np.int64)  # timestamps as int64
    np.save(out_path, arr)
    print(f'Saved {len(arr)} IMU measurements → {out_path}')
    print(f'  Time range: {arr[0,0]/1e9:.3f}s to {arr[-1,0]/1e9:.3f}s')
    print(f'  Approx rate: {len(arr) / ((arr[-1,0] - arr[0,0]) / 1e9):.1f} Hz')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    extract_imu(sys.argv[1], sys.argv[2])
