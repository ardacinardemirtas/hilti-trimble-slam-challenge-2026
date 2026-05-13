# Running ORB_SLAM3 on ETH Euler HPC

This is a sibling guide to `OPENVINS_HPC_SETUP.md` and `STELLA_VSLAM_HPC_SETUP.md`.
Read the OpenVINS guide first — the Apptainer and SLURM patterns are the same.
This document covers only what is **different** for ORB_SLAM3.

---

## Why ORB_SLAM3

| | OpenVINS | Stella-VSLAM | ORB_SLAM3 |
|---|---|---|---|
| Sensor | Visual + IMU | Visual only | Visual + IMU (optional) |
| Camera model | EUCM fisheye | Equirectangular / Fisheye | KannalaBrandt8 fisheye ✓ |
| Loop closure | No | Yes | Yes |
| Pre-processing | None | Stitch bags first | None |
| Output format | TF → TUM | `/camera_pose` → TUM | `SaveTrajectoryEuRoC` → convert |
| Back-to-back fisheye | Yes (per-cam) | Mono only | Stereo-Inertial (both cams) |

ORB_SLAM3 uses both fisheye cameras simultaneously in Stereo-Inertial mode, combines IMU
pre-integration (like OpenVINS) with loop closure (like Stella-VSLAM), and directly
supports the KannalaBrandt8 model — no image stitching required.

---

## Camera / IMU Configuration

Config: `config/hilti_orbslam3/hilti_orbslam3.yaml`

Calibration derived from `config/hilti_openvins/kalibr_imucam_chain.yaml` (Pinhole Equidistant section):

| Parameter | Value |
|-----------|-------|
| Camera model | `KannalaBrandt8` (= Kalibr "equidistant" / KB4) |
| Resolution | 1472 × 1440 per camera |
| fps | 30 Hz |
| IMU | 1000 Hz, noise params from `kalibr_imu_chain.yaml` |
| Baseline | ≈ 40 mm (back-to-back, opposite directions) |
| `IMU.T_b_c1` | T_{IMU from cam0} = inv(T_cam0_imu from Kalibr) |
| `Stereo.T_c1_c2` | T_{cam0 from cam1} = T_cam0_imu · inv(T_cam1_imu) |

**Back-to-back geometry**: cam0 and cam1 face opposite directions (~180° rotation in the
stereo baseline). Features directly in front of one camera are behind the other. ORB_SLAM3
still triangulates the equatorial-belt features that appear in both images, and treats
all other features as monocular/"far" points. The `Stereo.ThDepth: 600.0` keeps the
close-range stereo window at up to 24 m (600 × 0.040 m baseline).

**Timestamp offset**: Kalibr reports `timeshift_cam_imu ≈ −6.57 ms`. ORB_SLAM3 has no
explicit IMU-camera time-offset parameter; the effect is absorbed by the IMU
pre-integration window.

---

## 1. Build the Apptainer container

`orbslam3.def` builds Ubuntu 22.04 with OpenCV, Eigen3, and Pangolin from source.
Pangolin is required at **compile time** only — the binary runs headless
(`bUseViewer=false` is hard-coded in `stereo_inertial_euroc.cc`).

> Requires internet. On Euler: `module load eth_proxy` before building.

```bash
SCRATCH=/cluster/scratch/$USER
apptainer build --fakeroot $SCRATCH/orbslam3.sif \
    $SCRATCH/code/hilti-trimble-slam-challenge-2026/orbslam3.def
```

---

## 2. Build ORB_SLAM3 from source

Run **once** inside the container. The compiled binaries stay in
`$SCRATCH/code/ORB_SLAM3/` and all SLURM jobs reuse them.

```bash
SCRATCH=/cluster/scratch/$USER

apptainer exec --cleanenv \
    --home "$HOME" \
    --bind "$SCRATCH:$SCRATCH" \
    $SCRATCH/orbslam3.sif \
    bash $SCRATCH/code/hilti-trimble-slam-challenge-2026/build_orbslam3.sh
```

This builds DBoW2 → g2o → Sophus → extracts the ORB vocabulary → builds ORB_SLAM3.
Expect 10–20 minutes on 8 cores.

---

## 3. Run a single bag

The SLURM script (`run_orbslam3_single.sbatch`) does three things in one job:

1. **Bag → EuRoC**: uses the existing `ros2_jazzy.sif` + `ros2bag_to_euroc.py`
2. **ORB_SLAM3**: runs `stereo_inertial_euroc` inside `orbslam3.sif`
3. **Format conversion**: converts ORB_SLAM3's nanosecond timestamps to TUM seconds

```bash
SCRATCH=/cluster/scratch/$USER
SBATCH=$SCRATCH/code/hilti-trimble-slam-challenge-2026/run_orbslam3_single.sbatch

BAG_DIR=$SCRATCH/datasets/HILTIxTRIMBLE/data/floor_1/2025-05-05/run_1/rosbag
RUN_NAME=floor_1_2025-05-05_run_1
OUT_FILE=$SCRATCH/results/orbslam3/floor_1/2025-05-05_run_1.txt

sbatch --export="BAG_DIR=$BAG_DIR,RUN_NAME=$RUN_NAME,OUT_FILE=$OUT_FILE" "$SBATCH"
```

---

## 4. Run all bags

```bash
SCRATCH=/cluster/scratch/$USER
SBATCH=$SCRATCH/code/hilti-trimble-slam-challenge-2026/run_orbslam3_single.sbatch
DATA=$SCRATCH/datasets/HILTIxTRIMBLE/data

for rosbag_dir in "$DATA"/floor_*/*/run_*/rosbag; do
    # e.g. .../floor_1/2025-05-05/run_1/rosbag
    parts=($(echo "$rosbag_dir" | tr '/' '\n'))
    n=${#parts[@]}
    run_num=${parts[$((n-2))]}           # run_1
    date=${parts[$((n-3))]}              # 2025-05-05
    floor=${parts[$((n-4))]}             # floor_1
    RUN_NAME="${floor}_${date}_${run_num}"
    OUT_FILE="$SCRATCH/results/orbslam3/$floor/${date}_${run_num}.txt"
    sbatch --export="BAG_DIR=$rosbag_dir,RUN_NAME=$RUN_NAME,OUT_FILE=$OUT_FILE" "$SBATCH"
done
```

---

## 5. Evaluate

The output is standard TUM format — plug straight into `evaluate.py`:

```bash
cd $SCRATCH/code/hilti-trimble-slam-challenge-2026
source .venv/bin/activate

python3 evaluate.py \
    --gt groundtruth/ \
    --pred $SCRATCH/results/orbslam3/ \
    --plot-dir $SCRATCH/results/orbslam3_plots/ \
    --results-dir $SCRATCH/results/orbslam3_plots/ \
    --best-dir best_results/
```

---

## 6. Known pitfalls

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Pangolin::PangolinUnsupportedException` at startup | Pangolin tries to open a display | Not an issue — `stereo_inertial_euroc.cc` already passes `bUseViewer=false`. If you see this, the SIF was built without OSMesa/headless support; rebuild `orbslam3.def` |
| `Cannot open vocabulary file` | ORBvoc.txt not extracted | Run `cd ORB_SLAM3/Vocabulary && tar -xf ORBvoc.txt.tar.gz` |
| Zero poses, ORB_SLAM3 never initializes | Camera images not in grayscale; or resolution mismatch | Pass `--image-encoding mono8` to `ros2bag_to_euroc.py` (already set in sbatch) |
| ORB_SLAM3 initializes but loses tracking immediately | Too few ORB features in dark / motion-blurred frames | Lower `ORBextractor.minThFAST` to 3 in `hilti_orbslam3.yaml` |
| `yaml-cpp: error` on bag conversion | Hilti bag metadata format issue | The sbatch script runs the metadata fix automatically |
| `No module named 'rosbag2_py'` | EuRoC conversion not running inside ROS2 container | Confirm `ROS2_SIF` points to `ros2_jazzy.sif` and the workspace is built |
| EuRoC output timestamps mismatch | Images and IMU timestamps have +10 000 s offset | This is expected — ORB_SLAM3 and the TUM output will also have +10 000 s offsets; `evaluate.py` handles this correctly |
| `stereo_inertial_euroc: command not found` | ORB_SLAM3 not built | Run `build_orbslam3.sh` inside `orbslam3.sif` first |
| Trajectory has only a few dozen poses | IMU divergence or re-initialisation after lost tracking | Reduce bag playback area to skip bad segments; or try `stereo_euroc` (no IMU) for comparison |

---

## 7. Key differences from OpenVINS and Stella-VSLAM

1. **No ROS2 at runtime**: ORB_SLAM3 runs as a standalone binary directly on EuRoC files.
   Only the bag-extraction step needs the ROS2 container.
2. **Separate containers**: bag extraction uses `ros2_jazzy.sif`; ORB_SLAM3 runs in
   `orbslam3.sif`. Both can run sequentially in the same SLURM job (as in the sbatch).
3. **Disk usage**: EuRoC extraction writes raw PNG images (~5–15 GB per bag). Budget
   accordingly or use `--skip-images` + post-process only when iterating on config.
4. **No velocity/bias output**: ORB_SLAM3 outputs camera pose only (no IMU state).
5. **Loop closure**: included via the DBoW2 place-recognition module — long sequences
   are more stable than OpenVINS.
