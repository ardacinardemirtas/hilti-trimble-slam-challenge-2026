# Running OpenVINS on ETH Euler HPC (from scratch)

This documents every step and non-obvious fix needed to run the Hilti × Trimble OpenVINS
pipeline on Euler (or any similar SLURM cluster without sudo/GPU, using Apptainer).

---

## Overview

The cluster has no ROS2 install and no sudo. The solution is:

1. Build a **ROS2 Jazzy Apptainer container** with all dependencies
2. Create a **ROS2 workspace** with the challenge tools + Hilti OpenVINS fork
3. Fix **bag metadata** incompatibilities with Jazzy's rosbag2
4. Run everything inside a **single `apptainer exec`** per SLURM job so all ROS nodes share the same DDS discovery context

---

## 1. Build the Apptainer container

> Requires internet on the build node. On Euler, run `module load eth_proxy` first
> in any SLURM job that needs internet.

Save as `ros2_jazzy.def`:

```
Bootstrap: docker
From: ros:jazzy-ros-base

%post
    apt-get update -y
    apt-get install -y --no-install-recommends \
        libeigen3-dev libboost-all-dev libceres-dev \
        python3-colcon-common-extensions python3-rosdep \
        python3-pip python3-opencv \
        ros-jazzy-cv-bridge ros-jazzy-image-transport \
        ros-jazzy-compressed-image-transport \
        ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-rviz2 ros-jazzy-nav-msgs ros-jazzy-geometry-msgs \
        ros-jazzy-sensor-msgs ros-jazzy-rosbag2 ros-jazzy-rosbag2-py \
        ros-jazzy-rosbag2-storage-default-plugins \
        git cmake build-essential ninja-build wget \
    && rm -rf /var/lib/apt/lists/*

%environment
    source /opt/ros/jazzy/setup.bash
```

Build it (requires `--fakeroot` on Euler):

```bash
apptainer build --fakeroot ros2_jazzy.sif ros2_jazzy.def
```

---

## 2. Set up the workspace

```bash
SCRATCH=/cluster/scratch/$USER
mkdir -p $SCRATCH/ros2_ws/src

# Link the challenge tools package
ln -s $SCRATCH/code/hilti-trimble-slam-challenge-2026 \
      $SCRATCH/ros2_ws/src/hilti-trimble-slam-challenge-2026

# Clone the Hilti OpenVINS fork (supports EUCM camera model)
git clone https://github.com/Hilti-Research/open_vins.git \
          $SCRATCH/ros2_ws/src/open_vins
```

Build inside the container. **Use `--cleanenv`** — without it, the cluster's spack
`CC`/`CXX`/`CMAKE_PREFIX_PATH` leak in and break the CMake build silently.

```bash
apptainer exec --cleanenv \
    --home "$HOME" \
    --env TERM=xterm \
    --bind "$SCRATCH/ros2_ws:/ros2_ws" \
    ros2_jazzy.sif \
    bash -c "
        source /opt/ros/jazzy/setup.bash
        cd /ros2_ws
        colcon build --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            --parallel-workers \$(nproc)
    "
```

`--symlink-install` means edits to Python files and launch files take effect
immediately without rebuilding.

---

## 3. Fix bag metadata (one-time)

The Hilti bags were recorded with `offered_qos_profiles` as an empty YAML sequence
on a separate indented line. Jazzy's rosbag2 (version 9 metadata) decodes this field
as `std::vector<rclcpp::QoS>`, not a string. The indented form causes a yaml-cpp
parse error.

**Symptom:** `Exception on parsing info file: yaml-cpp: error at line 14, column 31: bad conversion`

**Fix** — run once after downloading the dataset:

```python
import glob, re

for f in glob.glob("/path/to/data/**/*.yaml", recursive=True):
    txt = open(f).read()
    # Fix 1: indented empty list → inline empty list
    new = re.sub(r'offered_qos_profiles:\s*\n\s*\[\]',
                 'offered_qos_profiles: []', txt)
    # Fix 2: empty string (wrong type) → empty list
    new = new.replace('offered_qos_profiles: ""', 'offered_qos_profiles: []')
    if new != txt:
        open(f, 'w').write(new)
```

---

## 4. Key code fixes

Several bugs exist in the challenge tools out of the box.

### 4a. `image_conversion_node.py` — remove TimeSynchronizer

The original uses `message_filters.TimeSynchronizer` for the two cameras, which
requires **exact** timestamp matches. Any mismatch silently drops all messages.
OpenVINS already has its own ApproximateTime synchronizer downstream.

Replace the synchronizer with independent per-camera subscriptions:

```python
self._subs = [
    self.create_subscription(
        CompressedImage, input_topics[i],
        lambda msg, idx=i: self._convert_and_publish(idx, msg),
        10,
    )
    for i in range(self._num_topics)
]
```

### 4b. `trajectory_logger.py` — wrong TF frame

OpenVINS broadcasts TF as `global → imu`, not `map → imu`.
Change the fixed frame in `trajectory_logger.py`:

```python
# Wrong:
self.fixed_frame = "map"
from_frame = 'map'

# Correct:
self.fixed_frame = "global"
from_frame = 'global'
```

### 4c. `run_openvins.launch.py` — `use_sim_time`

All nodes must use the bag's simulated clock (bags have a +10000 s timestamp offset
to avoid negative values). Without `use_sim_time: true`, node clocks return wall
time (~1.78 billion seconds) while message stamps are ~10000 seconds, causing
OpenVINS to silently discard all messages as stale.

For the OpenVINS node, add to its parameters list:
```python
{"use_sim_time": True},
```

For the image_conversion_node, add via `arguments` (not `parameters`):
```python
arguments=[
    "/cam0/image_raw/compressed",
    "/cam1/image_raw/compressed",
    "/cam0/image_raw",
    "/cam1/image_raw",
    "--ros-args", "-p", "use_sim_time:=true",
],
```

> **Why not `parameters=[{"use_sim_time": True}]` for the image_conversion_node?**
> Using `parameters=` causes ros2_launch to create a temporary YAML file
> (`/tmp/launch_params_XXXXX`) and pass its path as `--params-file /tmp/...`.
> The node's custom argument parser filters out `--`-prefixed flags but not
> bare file paths, so `/tmp/launch_params_XXXXX` leaks through and gets
> interpreted as the `desired_encoding` argument — breaking every image.

---

## 5. SLURM job script

The critical rules for running ROS2 inside SLURM on Euler:

| Rule | Why |
|------|-----|
| `--cleanenv` | Prevents spack variables from breaking ROS/CMake |
| `--home $SCRATCH/.ros_home/$SLURM_JOB_ID` | Home directory is read-only inside container; ROS needs `~/.ros/log`. **Do not** use `--env HOME=` — cluster policy blocks it |
| `--env ROS_LOG_DIR=...` | Redirects ROS logs to writable scratch |
| Single `apptainer exec` for all nodes | Each `apptainer exec` is a separate DDS domain — nodes in different invocations cannot communicate |
| `ROS_DOMAIN_ID=$(( SLURM_JOB_ID % 200 ))` | Prevents DDS cross-talk between parallel jobs |
| `ros2 bag play --clock` (no `-p`) | `-p` means `--start-paused`; with no terminal attached, the bag never unpauses and zero data flows |
| `--ros-args -p use_sim_time:=true` on trajectory_logger | Must match the sim clock used by other nodes |

Full sbatch template:

```bash
#!/usr/bin/env bash
#SBATCH --account=public
#SBATCH --partition=normal.4h
#SBATCH --time=2:00:00
#SBATCH --cpus-per-task=8
#SBATCH --mem-per-cpu=2G
#SBATCH --output=/cluster/scratch/$USER/logs/%x_%j.out
#SBATCH --error=/cluster/scratch/$USER/logs/%x_%j.err

# Pass via --export: BAG_DIR, RUN_NAME, OUT_FILE
set -euo pipefail

SCRATCH=/cluster/scratch/$USER
SIF=$SCRATCH/ros2_jazzy.sif
ROS2_WS=$SCRATCH/ros2_ws
export ROS_DOMAIN_ID=$(( SLURM_JOB_ID % 200 ))
ROS_SCRATCH_HOME=$SCRATCH/.ros_home/$SLURM_JOB_ID
mkdir -p "$ROS_SCRATCH_HOME/.ros/log"
mkdir -p "$(dirname "$OUT_FILE")"

apptainer exec --cleanenv \
    --home "$ROS_SCRATCH_HOME" \
    --env ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    --env ROS_LOG_DIR="$ROS_SCRATCH_HOME/.ros/log" \
    --bind "$ROS2_WS:/ros2_ws" \
    --bind "$SCRATCH:$SCRATCH" \
    "$SIF" \
    bash -c '
        source /opt/ros/jazzy/setup.bash
        source /ros2_ws/install/setup.bash

        # Launch OpenVINS + image converter
        ros2 launch challenge_tools_ros run_openvins.launch.py \
            rviz_enable:=false verbosity:=WARNING \
            max_cameras:=2 image_conversion:=true &
        OV_PID=$!
        sleep 10   # wait for OpenVINS to subscribe

        # Launch trajectory logger
        cd "/path/to/challenge_tools_ros/gt_helper"
        python3 trajectory_logger.py "'"$OUT_FILE"'" \
            --ros-args -p use_sim_time:=true &
        LOG_PID=$!
        sleep 3

        # Play bag (no -p flag!)
        ros2 bag play "'"$BAG_DIR"'" --clock

        sleep 15   # let last poses flush
        kill $LOG_PID $OV_PID 2>/dev/null || true
        wait $LOG_PID $OV_PID 2>/dev/null || true
    '
```

---

## 6. Evaluation

```bash
cd /path/to/hilti-trimble-slam-challenge-2026
python3 evaluate.py \
    --gt groundtruth/floor_1_2025-05-05_run_1.txt \
    --pred /path/to/results/floor_1_2025-05-05_run_1.txt \
    --plot --plot-dir /path/to/plots
```

---

## 7. Pitfall summary

| Symptom | Cause | Fix |
|---------|-------|-----|
| `yaml-cpp: error at line 14, column 31: bad conversion` | `offered_qos_profiles: []` stored as YAML sequence, Jazzy parser type mismatch | Fix all `metadata.yaml` files (see §3) |
| Zero poses in output, no errors | Bag played paused forever | Remove `-p` from `ros2 bag play` |
| Zero poses, nodes don't communicate | Each node in separate `apptainer exec` = separate DDS domain | Put all nodes in one `apptainer exec` bash -c block |
| `Failed to create log directory ~/.ros/log: error 30 (EROFS)` | Home dir read-only inside container | Use `--home $SCRATCH/.ros_home/$JOB_ID` |
| `WARNING: Overriding HOME via APPTAINERENV_HOME is not permitted` | Cluster blocks `--env HOME=` | Use `--home` flag instead |
| Image conversion errors: `Unrecognized image encoding [/tmp/launch_params_XXXXX]` | `parameters=[{"use_sim_time": True}]` creates a temp file whose path leaks into node args | Pass `use_sim_time` via `--ros-args` in the `arguments` list |
| Zero poses, OpenVINS receives data but never initializes | `use_sim_time` missing — node clock vs message stamp are 1.77 billion seconds apart | Add `use_sim_time: True` to all nodes |
| TF lookup fails silently, zero poses written | `trajectory_logger.py` looks up `map → imu` but OpenVINS publishes `global → imu` | Change `fixed_frame` and `from_frame` to `"global"` |
| OpenVINS sees 0 features despite images arriving | `TimeSynchronizer` in image_conversion_node drops all frames on any timestamp mismatch | Replace with independent per-camera subscriptions |
| `sbatch: error: Requesting memory by node is not supported` | Used `--mem=` instead of `--mem-per-cpu=` | Use `--mem-per-cpu=2G` |
