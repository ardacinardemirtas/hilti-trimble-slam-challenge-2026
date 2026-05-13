# Running Stella-VSLAM on ETH Euler HPC (from scratch)

This is a sibling guide to `OPENVINS_HPC_SETUP.md`. Read that first — the Apptainer
container, workspace layout, bag metadata fixes, and SLURM patterns are all identical.
This document covers only what is **different** for Stella-VSLAM.

---

## Why Stella-VSLAM instead of OpenVINS

OpenVINS is a filter-based VIO (Visual-Inertial Odometry) system. Once visual tracking
fails on a sequence, the IMU integration runs unconstrained and the trajectory diverges
to thousands of metres. Stella-VSLAM is a graph-based pure-visual SLAM system with
**loop closure**: when it revisits a known place it snaps the map back together,
preventing unbounded drift. It has no IMU, which is a disadvantage for fast motion but
an advantage for stability.

Key differences from OpenVINS:

| | OpenVINS | Stella-VSLAM |
|---|---|---|
| Sensor fusion | Visual + IMU | Visual only |
| Drift model | IMU integration → fast, explosive divergence | Slow ORB-feature drift, corrected by loop closure |
| Initialization | Requires IMU excitation + feature disparity | Pure visual, just needs enough ORB features |
| Camera model | EUCM fisheye (per-camera) | Equirectangular panorama **or** fisheye |
| ROS2 package | `ov_msckf` (Hilti fork pre-built) | `stella_vslam_ros` (must build from source) |

---

## 1. Two operating modes

### Mode A — Equirectangular (recommended)

Stitch cam0 + cam1 into a 360° panoramic image **before** running SLAM.
Stella-VSLAM then sees the full environment in every frame, maximising ORB features.

```
raw bag (/cam0/image_raw/compressed + /cam1/image_raw/compressed)
    │
    ▼  image_stitching.py  (runs inside container)
stitched bag (/pano/image_raw/compressed)
    │
    ▼  run_stella_vslam.launch.py  (config: equirectangular.yaml)
```

Config already present: `config/hilti_stella_vslam/equirectangular.yaml`
- 2944 × 1472 resolution, 30 fps, equirectangular model

### Mode B — Fisheye (cam0 only)

Use cam0 directly as a single fisheye camera (no stitching needed).
Works on bags where cam0 has features; fails on the same bags OpenVINS fails on.

Config already present: `config/hilti_stella_vslam/fisheye.yaml`
- 1472 × 1440, fisheye model with calibrated k1–k4

> **Recommendation:** Start with equirectangular. It uses both cameras, giving
> Stella-VSLAM the full 360° field of view and making loop closure much easier.

---

## 2. Build the Apptainer container

The existing `ros2_jazzy.sif` does **not** include Stella-VSLAM. You need a new
container (or extend the existing one). Stella-VSLAM requires:

- OpenCV ≥ 4 (already in the base image)
- Eigen3 (already present)
- g2o (graph optimiser) — **not** the same as Ceres
- FBoW (fast bag-of-words) — Stella's vocabulary loader
- `stella_vslam` core C++ library
- `stella_vslam_ros` ROS2 wrapper

Save as `ros2_jazzy_stella.def`:

```
Bootstrap: localimage
From: ros2_jazzy.sif

%post
    apt-get update -y
    apt-get install -y --no-install-recommends \
        libgoogle-glog-dev libatlas-base-dev \
        libsuitesparse-dev libyaml-cpp-dev \
        libopenexr-dev libopencv-dev \
        libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

    # ── g2o ──────────────────────────────────────────────────────────────
    git clone --depth 1 https://github.com/RainerKuemmerle/g2o.git /opt/g2o_src
    cmake -S /opt/g2o_src -B /opt/g2o_build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DG2O_BUILD_EXAMPLES=OFF
    cmake --build /opt/g2o_build --parallel $(nproc)
    cmake --install /opt/g2o_build
    rm -rf /opt/g2o_src /opt/g2o_build

    # ── FBoW ─────────────────────────────────────────────────────────────
    git clone --depth 1 https://github.com/rmsalinas/fbow.git /opt/fbow_src
    cmake -S /opt/fbow_src -B /opt/fbow_build -DCMAKE_BUILD_TYPE=Release
    cmake --build /opt/fbow_build --parallel $(nproc)
    cmake --install /opt/fbow_build
    rm -rf /opt/fbow_src /opt/fbow_build

%environment
    source /opt/ros/jazzy/setup.bash
```

Build (internet required — use `module load eth_proxy` in the SLURM job):

```bash
apptainer build --fakeroot ros2_jazzy_stella.sif ros2_jazzy_stella.def
```

---

## 3. Set up the workspace

Clone `stella_vslam_ros` into the existing workspace:

```bash
SCRATCH=/cluster/scratch/$USER

git clone https://github.com/stella-cv/stella_vslam_ros.git \
          $SCRATCH/ros2_ws/src/stella_vslam_ros

# Also clone the core library (stella_vslam_ros depends on it at build time)
git clone --depth 1 https://github.com/stella-cv/stella_vslam.git \
          $SCRATCH/ros2_ws/src/stella_vslam
```

Build (using the **new** container):

```bash
SIF=$SCRATCH/ros2_jazzy_stella.sif

apptainer exec --cleanenv \
    --home "$HOME" \
    --env TERM=xterm \
    --bind "$SCRATCH/ros2_ws:/ros2_ws" \
    "$SIF" \
    bash -c "
        source /opt/ros/jazzy/setup.bash
        cd /ros2_ws
        colcon build --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
            --packages-select stella_vslam stella_vslam_ros challenge_tools_ros \
            --parallel-workers \$(nproc)
    "
```

---

## 4. Pre-process: stitch bags into equirectangular

This is the main extra step vs OpenVINS. `image_stitching.py` takes a raw bag and
writes a new bag with `/pano/image_raw/compressed` + `/imu/data_raw`.

The script uses the Kalibr calibration YAML to compute the stitch maps. It needs to
run **inside the container** (uses `rosbag2_py`).

```bash
SCRATCH=/cluster/scratch/$USER
SIF=$SCRATCH/ros2_jazzy_stella.sif
CALIB=$SCRATCH/code/hilti-trimble-slam-challenge-2026/config/hilti_openvins/kalibr_calib.yaml
DATA=$SCRATCH/datasets/HILTIxTRIMBLE/data

BAG_IN=$DATA/floor_1/2025-05-05/run_1/rosbag
BAG_OUT=$SCRATCH/datasets/stitched/floor_1_2025-05-05_run_1

mkdir -p "$BAG_OUT"

apptainer exec --cleanenv \
    --home "$HOME" \
    --bind "$SCRATCH:$SCRATCH" \
    "$SIF" \
    bash -c "
        source /opt/ros/jazzy/setup.bash
        source $SCRATCH/ros2_ws/install/setup.bash
        python3 $SCRATCH/code/hilti-trimble-slam-challenge-2026/challenge_tools_ros/bag_helper/image_stitching.py \
            --calib '$CALIB' \
            --input-bag '$BAG_IN' \
            --output-bag '$BAG_OUT'
    "
```

> Stitching is CPU-intensive. Budget ~30 min per bag on 8 cores.
> Submit as a separate SLURM job array (one per bag) before the SLAM jobs.

Fix the output bag metadata the same way as for raw bags (see OPENVINS_HPC_SETUP.md §3).

---

## 5. Trajectory extraction

Stella-VSLAM's launch file sets `publish_tf: False`, so the trajectory_logger.py
used for OpenVINS (which does TF lookups) **will not work as-is**.

Two options:

### Option A — Enable TF publishing (simplest)

Override in the SLURM script. Add `--ros-args -p publish_tf:=true` to the
`stella_vslam` node, or modify the launch file:

```python
node_slam = Node(
    ...
    parameters=[{"publish_tf": True, "use_sim_time": True}],
)
```

Then use trajectory_logger.py with the correct frame names.
Stella-VSLAM publishes TF as `map → camera` (verify by running
`ros2 run tf2_tools view_frames` once the node is up).

### Option B — Subscribe to `/camera_pose` directly

Stella-VSLAM always publishes `geometry_msgs/PoseStamped` on `/camera_pose`.
Write a simple logger:

```python
#!/usr/bin/env python3
import sys, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseLogger(Node):
    def __init__(self, out_file):
        super().__init__('pose_logger')
        self._f = open(out_file, 'w')
        self._f.write('# timestamp tx ty tz qx qy qz qw\n')
        self.create_subscription(PoseStamped, '/camera_pose',
                                 self._cb, 100)

    def _cb(self, msg):
        t  = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p  = msg.pose.position
        q  = msg.pose.orientation
        self._f.write(f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
                      f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n')
        self._f.flush()

rclpy.init()
node = PoseLogger(sys.argv[1])
rclpy.spin(node)
```

Save as `$SCRATCH/code/hilti-trimble-slam-challenge-2026/challenge_tools_ros/gt_helper/stella_pose_logger.py`.

---

## 6. SLURM job script

```bash
#!/usr/bin/env bash
#SBATCH --job-name=sv_%j
#SBATCH --account=public
#SBATCH --partition=normal.4h
#SBATCH --time=2:00:00
#SBATCH --cpus-per-task=8
#SBATCH --mem-per-cpu=2G
#SBATCH --output=/cluster/scratch/$USER/logs/stella/%x_%j.out
#SBATCH --error=/cluster/scratch/$USER/logs/stella/%x_%j.err

# Args via --export: BAG_DIR (stitched bag), RUN_NAME, OUT_FILE

set -euo pipefail

SCRATCH=/cluster/scratch/$USER
SIF=$SCRATCH/ros2_jazzy_stella.sif
ROS2_WS=$SCRATCH/ros2_ws

export ROS_DOMAIN_ID=$(( SLURM_JOB_ID % 200 ))

ROS_SCRATCH_HOME=$SCRATCH/.ros_home/$SLURM_JOB_ID
mkdir -p "$ROS_SCRATCH_HOME/.ros/log"
mkdir -p "$(dirname "$OUT_FILE")"

mkdir -p "$SCRATCH/logs/stella"

apptainer exec --cleanenv \
    --home "$ROS_SCRATCH_HOME" \
    --env ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    --env ROS_LOG_DIR="$ROS_SCRATCH_HOME/.ros/log" \
    --bind "$ROS2_WS:/ros2_ws" \
    --bind "$SCRATCH:$SCRATCH" \
    "$SIF" \
    bash -c '
        set -e
        source /opt/ros/jazzy/setup.bash
        source /ros2_ws/install/setup.bash

        BAG_DIR="'"$BAG_DIR"'"
        OUT_FILE="'"$OUT_FILE"'"
        SCRATCH="'"$SCRATCH"'"
        REPO="$SCRATCH/code/hilti-trimble-slam-challenge-2026"

        # ── 1. Stella-VSLAM ──────────────────────────────────────────────
        ros2 launch challenge_tools_ros run_stella_vslam.launch.py \
            --ros-args -p use_sim_time:=true \
            -p publish_tf:=true &
        SV_PID=$!

        sleep 15   # Stella-VSLAM loads vocabulary + builds initial map

        # ── 2. Pose logger (Option B) ─────────────────────────────────────
        python3 "$REPO/challenge_tools_ros/gt_helper/stella_pose_logger.py" \
            "$OUT_FILE" \
            --ros-args -p use_sim_time:=true &
        LOG_PID=$!

        sleep 3

        # ── 3. Play stitched bag ──────────────────────────────────────────
        # Note: no -p flag (start-paused), use --clock for sim time
        # Slow down to 0.5x if Stella-VSLAM can'"'"'t keep up with 30fps
        echo "Playing bag: $BAG_DIR"
        ros2 bag play "$BAG_DIR" --clock --rate 0.5
        echo "Bag finished at $(date)"

        sleep 20   # Let loop closure finish

        kill $LOG_PID $SV_PID 2>/dev/null || true
        wait $LOG_PID $SV_PID 2>/dev/null || true
    '

echo "=== Done: $OUT_FILE ==="
wc -l "$OUT_FILE" || true
```

> **`--rate 0.5`**: Stella-VSLAM is more CPU-intensive than OpenVINS at 30fps.
> Start at half speed; if it keeps up, try 1.0 in later runs.

---

## 7. Submit all bags

```bash
SCRATCH=/cluster/scratch/$USER
SBATCH=$SCRATCH/run_stella_single.sbatch
DATA=$SCRATCH/datasets/stitched        # pre-stitched bags
RESULTS=$SCRATCH/results/stella

find "$DATA" -name "*.db3" | while read db; do
    bag_dir=$(dirname "$db")
    run_name=$(basename "$bag_dir")    # adjust naming to match your stitched dirs
    out="$RESULTS/${run_name}.txt"
    sbatch --export="BAG_DIR=$bag_dir,RUN_NAME=$run_name,OUT_FILE=$out" "$SBATCH"
done
```

---

## 8. Known pitfalls

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Could not open vocabulary file` | `orb_vocab.fbow` path wrong | Verify `get_package_share_directory` resolves to the installed config; alternatively pass `-v /absolute/path/to/orb_vocab.fbow` |
| Zero poses, Stella initialises but immediately loses track | Too few ORB features (dark, blurry, or textureless frames at start) | Lower `ini_fast_threshold` to 10 in the YAML; or use `--start-offset 10` to skip the initial shake |
| Stella never initialises | Bag played at full speed → CPU overloaded → frames dropped before ORB extraction | Use `--rate 0.5` |
| `publish_tf` has no effect | Parameter passed via `--ros-args` after launch args — order matters | Put `publish_tf:=true` in `parameters=[...]` in the launch file, rebuild with `--symlink-install` (no rebuild needed for Python launch files) |
| Trajectory logger gets 0 poses | Using trajectory_logger.py (TF-based) but `publish_tf: False` | Use `stella_pose_logger.py` (Option B) or enable TF |
| Loop closure never fires | `min_distance_on_graph: 50` too high for small rooms | Lower to 10–20 in `equirectangular.yaml` |
| Stitching produces no frames | Timestamp sync tolerance too tight between cam0 and cam1 | Add `--sync-tol-ns 50000000` (50ms) to the stitching command |
| `yaml-cpp` error on stitched bag | Stitched bag also has `offered_qos_profiles: ""` | Apply the same metadata fix from OPENVINS_HPC_SETUP.md §3 |

---

## 9. Key differences from OpenVINS setup

1. **Extra pre-processing step**: stitching must run first (separate SLURM job or job dependency with `--dependency=afterok:<stitch_job_id>`)
2. **Larger container**: stella + g2o + fbow ≈ extra 400 MB in the SIF
3. **Slower playback**: start at `--rate 0.5`, OpenVINS runs fine at 1x
4. **Different trajectory output**: use `stella_pose_logger.py`, not `trajectory_logger.py`
5. **No IMU**: if the bag has deliberate shaking at the start to excite IMU, this does NOT help or hurt Stella-VSLAM — it just needs texture
6. **Loop closure**: the main advantage — even long sequences stay bounded

---

## 10. Context from the OpenVINS runs

The following patterns were observed when running OpenVINS on all 30 bags:

- **10 bags produced good trajectories** (20–50m realistic spans)
- **~15 bags exploded** (IMU divergence to 1,000–100,000m) — these are Stella's primary target
- **5 bags failed initialization** entirely (likely very aggressive initial motion)

The 15 exploded OpenVINS bags all initialised successfully but lost visual tracking mid-sequence, causing IMU integration to diverge. Stella-VSLAM's loop closure should prevent this — but it still needs enough ORB features to maintain tracking. The equirectangular mode (full 360° view) gives it the best chance.

The `init_window_time` issue in OpenVINS (features need to survive `0.5 * init_window_time` to trigger initialization) is not relevant to Stella-VSLAM; its initialization is purely feature-count based.
