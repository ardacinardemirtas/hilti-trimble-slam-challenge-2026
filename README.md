<div align="center">

# Solution: Hilti × Trimble 360 Visual-Inertial SLAM Challenge 2026

**Arda Çınar Demirtaş · Berkay Şekeroğlu · Atakan Topaloğlu**

[<img src="https://img.shields.io/badge/Challenge_Page-red" alt="Challenge Page">](https://hilti-trimble-challenge.com)
[<img src="https://img.shields.io/badge/Dataset-4285F4?logo=googledrive&logoColor=white" alt="Dataset">](https://drive.google.com/drive/u/1/folders/1BWFIfEL40Nvj-yeyre5O9dOiYCTWatv5)
[<img src="https://img.shields.io/badge/Challenge_Repo-black?logo=github" alt="Challenge Repo">](https://github.com/Hilti-Research/hilti-trimble-slam-challenge-2026)

</div>

This repository contains our solution to the [Hilti × Trimble 360 Visual-Inertial SLAM Challenge 2026](https://hilti-trimble-challenge.com), which benchmarks SLAM and localization algorithms on active construction sites using an Insta360 One-RS dual fisheye camera with IMU.

---

## Results

### SLAM Task — 5 early-release runs (with ground truth)

| Sequence | ATE RMSE (m) | Coverage | Score / 100 |
|---|---|---|---|
| floor\_1 / 2025-05-05 / run\_1 | 0.298 | 99.7% | 88.5 |
| floor\_2 / 2025-05-05 / run\_1 | 0.696 | 99.6% | 75.9 |
| floor\_2 / 2025-10-28 / run\_1 | 0.248 | 99.4% | 89.7 |
| floor\_2 / 2025-10-28 / run\_2 | 0.262 | 100.0% | 89.5 |
| floor\_UG1 / 2025-10-16 / run\_1 (+ VI-BA) | 0.371 | 99.6% | ~86.5 |
| **Mean / Total** | **0.375 m** | | **~430 / 500** |

Dynamic initialization at 0.5× playback rate was the single most impactful technique, accounting for 77% of submitted runs. The VI-BA refinement step reduced ATE by 29% on the hardest basement sequence.

---

## Approach

The pipeline has two stages:

### Stage 1 — OpenVINS sliding-window filter (all 30 runs)

We use [OpenVINS](https://github.com/Hilti-Research/open_vins) (Hilti fork, with EUCM camera model support) as the core VI odometry engine. KLT optical flow tracks features across both fisheye cameras at 30 Hz; IMU pre-integration runs at 1000 Hz within the MSCKF sliding window.

No single configuration generalizes across all 30 sequences — construction sites introduce dynamic starts, low-texture rooms, and variable lighting. We designed five config variants and select the best per-run empirically:

| Config | Key change | Best for |
|---|---|---|
| `baseline` | (defaults) | well-textured, static-start sequences |
| `dynA` | dynamic init, 3 s window | sequences that start mid-motion |
| `fastinit` | dynamic init, 1 s window | early initialization failure |
| `imuB` | IMU noise ×5 | dark rooms, slow walking with hard-to-observe bias |
| `expC` | dynA + imuB + more features/clones + CLAHE | long featureless corridors |

Running bags at **0.5× playback** was necessary on most runs. At real-time speed, the CPU processes only 16–23 frames/s against the 30 Hz stream, causing larger inter-frame displacements that break KLT tracks. Half-speed processing restores near-nominal frame rate, significantly improving tracking stability.

### Stage 2 — Offline Visual-Inertial Bundle Adjustment (GT runs)

For runs with released ground truth we applied an offline VI-BA built on the [`features/imu` branch of COLMAP](https://github.com/B1ueber2y/colmap/tree/features/imu) ([Liu, 2024]). The filter's KLT tracks and 3D landmarks are exported to COLMAP format; COLMAP then minimizes joint reprojection and IMU pre-integration residuals (Forster et al., 2017) over the full trajectory. This yielded a **29% ATE reduction** on the basement sequence (0.522 m → 0.371 m).

### Localization Task

SE2 alignment is computed using the provided ground-truth anchor pose at t ≈ 10005 s, then applied to the full SLAM trajectory. Pitch and roll are forced to zero (gravity-aligned).

---

## Repository Structure

```
.
├── evaluate.py                      # Local evaluation: ATE, score, plots, best-result tracking
├── openvins_to_colmap.py            # Convert OpenVINS TUM → COLMAP init reconstruction
├── features_to_colmap.py            # Convert OpenVINS feature tracks → COLMAP with 3D points
├── run_vi_hilti.py                  # VI-BA: joint visual-inertial bundle adjustment (COLMAP+IMU)
├── localize_from_5s.py              # Localization: SE2 map alignment using GT anchor pose
├── bag_to_imu_npy.py                # Extract IMU data from a ROS2 bag → numpy array
├── plot_config_comparison.py        # Plot multiple OpenVINS configs side-by-side for a run
├── plot_overlay.py                  # Overlay a trajectory on the building floorplan
│
├── config/
│   ├── hilti_openvins/              # Baseline config (Kalibr calibration, estimator, masks)
│   ├── hilti_openvins_dynA/         # Dynamic init, 3 s window
│   ├── hilti_openvins_fastinit/     # Dynamic init, 1 s window
│   ├── hilti_openvins_imuB*/        # IMU noise ×5 variants (+ CLAHE, ZUPT, etc.)
│   └── hilti_openvins_expC/         # Combined: dynA + imuB + more features/clones + CLAHE
│
├── challenge_tools_ros/
│   ├── bag_helper/                  # Bag conversion: ROS2→EuRoC, fisheye stitching
│   ├── gt_helper/                   # Ground truth: loading, publishing, trajectory logging
│   ├── runtime_helper/              # ROS2 nodes: decompression, map server, masking
│   └── colmap_to_tum.py             # Convert COLMAP VI-BA output → TUM format
│
├── launch/                          # ROS2 launch files
├── config/rviz/                     # RViz configs
├── groundtruth/                     # GT TUM files for 5 early-release runs
├── floorplans/                      # Building floorplan PNG images (with/without windows)
├── BEST_CONFIG_SELECTION.md         # Per-run config choices with verification table
└── REPORT.md                        # Full technical report
```

---

## Installation

### Requirements

- Ubuntu 24.04 with ROS2 Jazzy
- Python 3.11+ (`numpy`, `pyyaml`, `matplotlib`)

### 1. Clone and build the ROS2 package

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/<your-handle>/hilti-trimble-slam-challenge-2026.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Install OpenVINS (Hilti fork — EUCM support required)

```bash
sudo apt install libeigen3-dev libboost-all-dev libceres-dev

cd ~/ros2_ws/src
git clone https://github.com/Hilti-Research/open_vins.git
cd ~/ros2_ws
colcon build --symlink-install
```

### 3. Build COLMAP features/imu branch (VI-BA only)

The VI-BA step (Stage 2) requires the `features/imu` fork of COLMAP with its `pycolmap` and `pyceres` Python bindings compiled. Follow the [upstream build instructions](https://github.com/B1ueber2y/colmap/tree/features/imu). Once built, update the path in `run_vi_hilti.py`:

```python
_COLMAP_IMU = Path(__file__).parent.parent / "colmap_imu"  # adjust to your build dir
```

---

## Running the Pipeline

### Dataset

Download bags from the [Google Drive folder](https://drive.google.com/drive/u/1/folders/1BWFIfEL40Nvj-yeyre5O9dOiYCTWatv5). Place them as:

```
data/floor_X/YYYY-MM-DD/run_Z/rosbag/{rosbag.db3, metadata.yaml}
```

### Step 1 — Run OpenVINS

**Terminal 1** — start OpenVINS (choose a config, see Configuration section):

```bash
ros2 launch hilti_trimble_slam_challenge run_openvins.launch.py \
    namespace:=/ \
    config:=hilti_openvins_dynA \
    verbosity:=WARNING \
    max_cameras:=2
```

**Terminal 2** — play the bag at half speed:

```bash
ros2 bag play data/floor_X/YYYY-MM-DD/run_Z/rosbag/ --clock --rate 0.5
```

`trajectory_logger.py` (part of the launch file) writes a TUM file automatically. See `BEST_CONFIG_SELECTION.md` for verified per-run config and playback rate choices.

### Step 2 — Offline VI-BA Refinement (optional, GT runs)

> Requires the COLMAP `features/imu` Python bindings.

**2a. Extract images from bag:**

```bash
python3 challenge_tools_ros/bag_helper/ros2bag_to_euroc.py \
    --bag data/floor_X/.../rosbag/ \
    --output euroc_floor_X/ \
    --topics /cam0/image_raw/compressed
```

**2b. Extract IMU data from bag:**

```bash
python3 bag_to_imu_npy.py \
    data/floor_X/.../rosbag/ \
    floor_X_imu.npy
```

**2c. Option A — Poses only (no feature tracks):**

Converts OpenVINS TUM poses to COLMAP init format. Run COLMAP feature extraction + matching + triangulation separately after this.

```bash
python3 openvins_to_colmap.py \
    --tum results/floor_X_run_Z.txt \
    --calib config/hilti_openvins/kalibr_imucam_chain.yaml \
    --output colmap_init/ \
    --images euroc_floor_X/cam0/data/
```

**2c. Option B — Poses + KLT feature tracks (recommended):**

Requires running `feature_logger.py` alongside OpenVINS during Step 1. Produces a richer initialization with 3D landmarks and 2D observations already in COLMAP format.

```bash
python3 features_to_colmap.py \
    --features run_features.txt \
    --tum results/floor_X_run_Z.txt \
    --calib config/hilti_openvins/kalibr_imucam_chain.yaml \
    --output colmap_with_tracks/
```

**2d. Run VI bundle adjustment:**

```bash
python3 run_vi_hilti.py \
    --colmap colmap_with_tracks/ \
    --imu floor_X_imu.npy \
    --calib config/hilti_openvins/kalibr_imucam_chain.yaml \
    --output results/floor_X_vi_ba/
```

The refined TUM trajectory is written to `results/floor_X_vi_ba/refined.txt`.

### Step 3 — Localization Task

Align the SLAM trajectory to the floorplan coordinate frame using the provided GT anchor pose:

```bash
# Edit SLAM_DIR at the top of localize_from_5s.py to point to your results
python3 localize_from_5s.py
```

Output files land in `localization_submission/`, correctly named for submission.

### Step 4 — Evaluate Locally

```bash
# Single run (SLAM task)
python3 evaluate.py \
    --gt groundtruth/floor_2_2025-05-05_run_1.txt \
    --pred results/floor_2_2025-05-05_run_1.txt

# All released GT runs, with plots and best-result tracking
python3 evaluate.py \
    --gt groundtruth/ \
    --pred results/ \
    --plot-dir plots/ \
    --best-dir best_results/

# Localization task
python3 evaluate.py \
    --gt groundtruth/ \
    --pred localization_submission/ \
    --task localization
```

Output: per-run ATE RMSE, exponential score, error plots, and coverage.

### Step 5 — Package Submission

```bash
find best_results/ -name "*.txt" ! -name "*_report.txt" \
    | xargs -I{} cp {} submission/
zip submission.zip submission/*.txt
```

Upload to https://submit.hilti-challenge.com/.

---

## Configuration Details

All configs share the same Kalibr calibration (`config/hilti_openvins/kalibr_imucam_chain.yaml`) and camera masks. Key parameter differences:

| Parameter | baseline | dynA | fastinit | imuB | expC |
|---|---|---|---|---|---|
| `init_dyn_use` | false | **true** | **true** | false | **true** |
| `init_window_time` | 3.0 s | 3.0 s | **1.0 s** | 3.0 s | 3.0 s |
| IMU noise scale | 1× | 1× | 1× | **5×** | **5×** |
| Max features | 400 | 400 | 400 | 400 | **800** |
| SLAM landmarks | 50 | 50 | 50 | 50 | **100** |
| Clone window | 11 | 11 | 11 | 11 | **15** |
| CLAHE | off | off | off | on (`_clahe` variant) | **on** |

**How to choose:**

1. Start with `dynA` at 0.5× — it works for the majority of runs.
2. If divergence occurs after a dark/featureless segment, switch to `imuB` (inflated noise keeps the filter from over-trusting a poorly observable IMU bias).
3. If initialization takes > 30 s, use `fastinit` (1 s window fires on earlier motion).
4. For long corridor sequences with sparse texture, try `expC`.

See `BEST_CONFIG_SELECTION.md` for the verified config choice per run.

---

## Sensor & Calibration

- **Camera**: Insta360 One-RS, dual fisheye (Leica Summicron 6.52 mm f/2.2, ~200° FoV per lens), 1472×1440, 30 Hz, rolling shutter
- **IMU**: Bosch BMI085, 6-axis, ~1000 Hz
- **Camera model**: Pinhole equidistant (Kannala-Brandt), fitted to EUCM Kalibr calibration
- **Timestamp offset**: All bags apply a +10 000 s shift; images start at t = 10 000 s

IMU noise parameters (from BMI085 datasheet, consistent with Kalibr):

| Parameter | Value |
|---|---|
| Gyro noise density | 1.7 × 10⁻⁴ rad/s/√Hz |
| Accel noise density | 2.0 × 10⁻³ m/s²/√Hz |
| Gyro bias random walk | 1.9 × 10⁻⁵ rad/s²/√Hz |
| Accel bias random walk | 3.0 × 10⁻⁴ m/s³/√Hz |

---

## References

Geneva, P., Eckenhoff, K., Lee, W., Yang, Y., & Huang, G. (2020). OpenVINS: A Research Platform for Visual-Inertial Estimation. *ICRA 2020 Workshop on Open Source Systems in Robotics*.

Schönberger, J. L., & Frahm, J.-M. (2016). Structure-from-Motion Revisited. *CVPR 2016*.

Liu, S. (2024). COLMAP `features/imu`: Visual-Inertial Bundle Adjustment with IMU Preintegration Factors. https://github.com/B1ueber2y/colmap/tree/features/imu

Forster, C., Carlone, L., Dellaert, F., & Scaramuzza, D. (2017). On-Manifold Preintegration for Real-Time Visual-Inertial Odometry. *IEEE T-RO*, 33(1).

---

## Citation

If you find this work useful, please also cite the challenge dataset:

```bibtex
@online{slamchallenge2026,
   title  = {{Hilti}-{Trimble}-{Oxford} Dataset: 360 Visual-Inertial Benchmark with Floor Plan Priors for SLAM and Localization},
   year   = {2026},
   url    = {https://github.com/Hilti-Research/hilti-trimble-slam-challenge-2026},
   author = {Centanni, Samuele and Zhang, Yuhao and Tao, Yifu and Kindle, Julien and Neuhaus, Frank and Ko{\ss}, Tilman and Patel, Aryaman and Helmberger, Michael and Szymańska, Emilia and Gräber, Torben and Fallon, Maurice},
}
```
