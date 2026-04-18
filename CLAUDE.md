# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the **Hilti × Trimble 360 Visual-Inertial SLAM Challenge 2026** — a benchmark dataset and tooling for evaluating SLAM and localization algorithms on active construction sites. Two tasks:
- **SLAM**: Estimate camera trajectory in any reference frame
- **Localization**: Estimate camera trajectory in the floorplan reference frame

Sensor: Insta360 One-RS (dual fisheye cameras + IMU). Ground truth from Hesai XT32 LiDAR (not in bags).

## Build & Setup

Requires ROS2 Jazzy on Ubuntu 24.04.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src && git clone <repo>
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

No standalone test suite — evaluation is done by submitting trajectories to https://submit.hilti-challenge.com/.

## Key Commands

**Launch OpenVINS SLAM:**
```bash
ros2 launch hilti_trimble_slam_challenge run_openvins.launch.py namespace:=/ config:=hilti_openvins verbosity:=WARNING max_cameras:=2
```

**Play a bag:**
```bash
ros2 bag play data/floor_X/YYYY-MM-DD/run_Z/rosbag/ --clock
```

**Convert ROS2 bag to EuRoC format:**
```bash
python3 challenge_tools_ros/bag_helper/ros2bag_to_euroc.py
```

**Launch floorplan map server:**
```bash
ros2 launch hilti_trimble_slam_challenge map_server.launch.py run_name:=<run>
```

**Launch ground truth server:**
```bash
ros2 launch hilti_trimble_slam_challenge groundtruth_server.launch.py run_name:=<run>
```

## Architecture

```
challenge_tools_ros/
├── bag_helper/         # Bag conversion (ROS2→EuRoC, ROS2→ROS1, fisheye stitching)
├── gt_helper/          # Ground truth: loading, publishing, trajectory logging/plotting
│   └── challenge_tools_lib.py  # Core: ReactPose, ReactTime, ChallengeToolsLib (TUM format I/O)
├── runtime_helper/     # ROS2 nodes: image decompression, map publishing, masking, rotation
└── download_data/      # Dataset download scripts (Drive or rclone)

config/
├── hilti_openvins/     # OpenVINS config: estimator_config.yaml, kalibr calibration, rs_config.yaml
├── hilti_stella_vslam/ # Stella-VSLAM config and ORB vocabulary
└── rviz/               # RViz configs

launch/                 # ROS2 launch files for each subsystem
groundtruth/            # TUM-format ground truth for 5 released runs
```

## Data Details

- **ROS topics:** `/cam0/image_raw/compressed`, `/cam1/image_raw/compressed` (30 Hz), `/imu/data_raw` (1000 Hz)
- **Trajectory format (TUM):** `timestamp tx ty tz qx qy qz qw`
- **Timestamp:** All bags apply a +10000 s offset to avoid negative timestamps; ground truth evaluation skips the first 5 s
- **Dataset layout:** `data/floor_X/YYYY-MM-DD/run_Z/rosbag/{rosbag.db3, metadata.yaml}`

## Camera Models

Two models are provided in calibration files:
- **Pinhole Equidistant** — for standard fisheye use
- **EUCM** (Enhanced Unified Camera Model) — requires the custom OpenVINS fork with EUCM support

## Submission Format

Submit a single `.zip` containing per-run TUM trajectory files named `<run_name>.txt`. Scoring uses an exponential metric (max 2500 pts SLAM, 2400 pts Localization).
