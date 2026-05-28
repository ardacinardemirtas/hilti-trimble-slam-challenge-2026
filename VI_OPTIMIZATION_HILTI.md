# VI Optimization for Hilti OpenVINS Reconstructions

Reference for running `run_vi_hilti.py` on OpenVINS-derived reconstructions
without a COLMAP database, using the `colmap_imu` pycolmap bindings.

---

## Pipeline Overview

```
OpenVINS run (--rate 0.5)
    │  trajectory_logger.py → {run}.txt  (TUM, global→imu)
    │  feature_logger.py    → {run}_feats.txt  (feat_id ts u v X Y Z)
    ▼
features_to_colmap.py
    → {run}_colmap/
        cameras.txt          (OPENCV_FISHEYE intrinsics from Kalibr)
        images.txt           (cam_from_world poses + 2D observations)
        points3D.txt         (3D world positions + per-image tracks)
        image_timestamps.npy ({image_id → timestamp_ns})
    ▼
bag_to_imu_npy.py  (inside Apptainer ros2_jazzy.sif)
    → {run}_imu.npy  (N×7: [ts_ns ax ay az gx gy gz])
    ▼
run_vi_hilti.py  (Python 3.11 + setup_env.sh)
    → {run}_vi/
        refined.txt          (TUM, world_from_cam)
        cameras.bin/images.bin/points3D.bin  (refined COLMAP binary)
```

---

## Scripts

| Script | Location | Runs where |
|--------|----------|-----------|
| `features_to_colmap.py` | repo root | login node (uv run) |
| `bag_to_imu_npy.py` | repo root | inside Apptainer (SLURM) |
| `run_vi_hilti.py` | repo root | Python 3.11 + setup_env.sh (SLURM) |
| `run_imu_extract.sbatch` | `$SCRATCH/` | SLURM |
| `run_vi_optimization.sbatch` | `$SCRATCH/` | SLURM |

---

## Running

### Step 1 — features_to_colmap.py (login node, fast)

```bash
cd /cluster/scratch/ademirtas/code/hilti-trimble-slam-challenge-2026

UV_NO_CONFIG=1 uv run python3 features_to_colmap.py \
    --features results/openvins_vi_ready/{run}_feats.txt \
    --tum      results/openvins_vi_ready/{run}.txt \
    --calib    config/hilti_openvins/kalibr_imucam_chain.yaml \
    --output   results/openvins_vi_ready/{run}_colmap
```

Typical output: ~20k–50k 3D points, ~800k observations.

### Step 2 — Submit IMU extraction + VI optimization as a dependent pair

```bash
SCRATCH=/cluster/scratch/ademirtas
RUN=floor_1_2025-05-05_run_1
BAG_DIR=$SCRATCH/datasets/HILTIxTRIMBLE/data/floor_1/2025-05-05/run_1/rosbag
VI_READY=$SCRATCH/results/openvins_vi_ready

IMU_JOB=$(sbatch --parsable \
  --export="BAG_DIR=$BAG_DIR,OUT_FILE=$VI_READY/${RUN}_imu.npy" \
  $SCRATCH/run_imu_extract.sbatch)

sbatch --dependency=afterok:$IMU_JOB \
  --export="COLMAP_DIR=$VI_READY/${RUN}_colmap,\
IMU_FILE=$VI_READY/${RUN}_imu.npy,\
OUTPUT_DIR=$VI_READY/${RUN}_vi,\
RUN_NAME=$RUN,\
TUM_OUT=$VI_READY/${RUN}_vi/refined.txt" \
  $SCRATCH/run_vi_optimization.sbatch
```

### Step 3 — Evaluate

```bash
UV_NO_CONFIG=1 uv run python3 evaluate.py \
  --gt  groundtruth/{run}.txt \
  --pred results/openvins_vi_ready/{run}_vi/refined.txt
```

---

## Key Learnings

### 1. Python binary: use `.venv/bin/python3.11`, not `python3` or `python3.11` from modules

The `colmap_imu` venv was created with Python 3.11.9 from the **2025-06 spack stack**
(see `colmap_imu/.venv/pyvenv.cfg`). Three wrong paths to avoid:

| What you try | Why it fails |
|---|---|
| System `python3` / `python3.10` | Wrong version — `_core.so` built for 3.11 |
| `module load python/3.11.6` → `python3` | (a) `module` not available in SLURM batch jobs on `normal.4h`; (b) 2024-06 stack Python links against OpenSSL 3.1.3, but `setup_env.sh` puts 2025-06 OpenSSL 3.4.0 in `LD_LIBRARY_PATH` → `OPENSSL_3.3.0 not found` crash |
| `.venv/bin/python3` | Symlinks to `/usr/bin/python3` (system 3.10), not to Python 3.11 |

**Correct binary**: `.venv/bin/python3.11` — symlinks to the 2025-06 stack Python 3.11.9,
has numpy/pyceres/scipy installed, and matches the OpenSSL used to build `_core.so`.

In sbatch:
```bash
PYTHON311=$COLMAP_IMU/.venv/bin/python3.11
source "$COLMAP_IMU/setup_env.sh"
$PYTHON311 run_vi_hilti.py ...
```

### 2. pycolmap import path

The built `_core.so` lives in `python/build/`, not in the installed package.
Both `python/build/` and `python/` must be on `sys.path`. The venv
site-packages provides `pyceres` and is already on the path when using `.venv/bin/python3.11`:

```python
import sys
COLMAP_IMU = "/cluster/scratch/ademirtas/code/colmap_imu"
sys.path.insert(0, f"{COLMAP_IMU}/python/build")
sys.path.insert(0, f"{COLMAP_IMU}/python")
# pyceres found automatically via .venv site-packages
import pycolmap, pyceres
```

### 3. No COLMAP database needed for OpenVINS reconstructions

`vi_optimization.py`'s `iterative_refine()` function uses `IncrementalMapper`,
which requires a COLMAP SQLite database for `complete_and_merge_tracks` and
`retriangulate`. These operations need SIFT descriptors and match tables — which
don't exist for OpenVINS KLT tracks.

**Solution**: skip `IncrementalMapper` entirely. Call `solve_bundle_adjustment`
directly on the `Reconstruction` object. The IMU residuals and visual BA both
work without a database. Track completion is a no-op for us anyway (no
descriptors).

### 4. Frames and rigs ARE set up when loading from text files

When `pycolmap.Reconstruction()` loads a text-format reconstruction, each image
gets a `frame` with a trivial rig (`non_ref_sensors == 0`). The
`AnalyticalVisualCentricImuPreintegrationCost` assert passes without any extra
setup:

```python
assert len(image_i.frame.rig.non_ref_sensors) == 0  # ✓ for our reconstructions
```

### 5. features_to_colmap.py vs openvins_to_colmap.py

| | `features_to_colmap.py` | `openvins_to_colmap.py` |
|---|---|---|
| Input | `_feats.txt` + TUM | TUM only |
| points3D.txt | Populated (OpenVINS landmarks) | Empty |
| Suitable for VI BA | Yes — has 3D points and observations | No — needs COLMAP triangulation first |
| When to use | Default (if feats file available) | When feats file is missing; follow with `colmap triangulate_points` |

Always use `features_to_colmap.py` if a `_feats.txt` file exists.

### 6. IMU timestamp offset carries through

All timestamps in the Hilti bags have a +10 000 s offset applied at recording
time. `bag_to_imu_npy.py` reads `msg.header.stamp`, which already has this
offset baked in. `image_timestamps.npy` (from `features_to_colmap.py`) stores
the same offset. The preintegration interval `[t1, t2]` is in absolute
nanoseconds so the offset cancels — no correction needed.

The exported TUM trajectory also keeps the offset, matching the GT evaluation
convention.

### 7. IMU rate setting

The Bosch BMI085 on the Hilti sensor publishes at ~995 Hz in the bag. Set
`imu_calib.imu_rate = 1000.0` (the nominal rate). The actual interval between
measurements is used by `ImuPreintegrator.feed_imu()` from the timestamps
directly, so the rate parameter only affects noise density scaling — using 1000
for a ~995 Hz sensor is negligible.

### 8. The rig blocker in vi_optimization.py

`add_imu_residuals` asserts a trivial rig (single camera per frame).
Our `_colmap/` directories contain **cam0-only** data — cam1 is excluded by
`features_to_colmap.py` (uses only `cam0` poses from the TUM file). This
constraint is satisfied automatically. No workaround needed.

---

## File Sizes (floor_1_2025-05-05_run_1)

| File | Size |
|------|------|
| `_feats.txt` | ~786k rows (800 MB) |
| `_colmap/points3D.txt` | 20 429 3D points |
| `_colmap/images.txt` | 3 974 images, 776 001 observations |
| `_imu.npy` | ~120k rows at 1000 Hz × 120 s |

---

## IMU Calibration (Bosch BMI085 — Hilti sensor)

```python
imu_calib.imu_rate                  = 1000.0    # Hz
imu_calib.gravity_magnitude         = 9.81
imu_calib.gyro_noise_density        = 1.7e-4    # rad/s/√Hz
imu_calib.accel_noise_density       = 2.0e-3    # m/s²/√Hz
imu_calib.bias_gyro_random_walk_sigma  = 1.9e-5
imu_calib.bias_accel_random_walk_sigma = 3.0e-4
```

Source: BMI085 datasheet; also in `VI_OPTIMIZATION_PATHS.md` and the
openvins_vi_ready `README.md`.

---

## Evaluation Baseline

| Run | Pre-VI score | Target |
|-----|-------------|--------|
| floor_1_2025-05-05_run_1 | 88.38 / 100 (RMSE 0.300 m) | improve |
| floor_2_2025-05-05_run_1 | 83.28 / 100 (RMSE 0.446 m) | improve |
| floor_2_2025-10-28_run_1 | 89.70 / 100 | improve |
| floor_2_2025-10-28_run_2 | 89.50 / 100 | improve |
| floor_UG1_2025-10-16_run_1 | 62.60 / 100 (RMSE 1.119 m) | largest gap |

The runs with existing `_colmap/` dirs (floor_2_2025-10-28_run_1 and run_2) are
ready for VI immediately — just need `bag_to_imu_npy.py` run for each.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `ModuleNotFoundError: No module named 'numpy'` | Using bare Python 3.11.9, not the venv | Use `.venv/bin/python3.11`, not the raw stack Python |
| `ModuleNotFoundError: No module named 'pycolmap'` | Using system Python 3.10 | Use `.venv/bin/python3.11` |
| `OPENSSL_3.3.0 not found` on pycolmap import | Python from 2024-06 stack + `setup_env.sh` 2025-06 OpenSSL in LD_LIBRARY_PATH | Use `.venv/bin/python3.11` (2025-06 Python, matching OpenSSL) |
| `module: command not found` in SLURM | `module` is a shell function from `~/.bashrc`, not available in batch jobs | Use absolute paths — never `module load` in sbatch |
| `.venv/bin/python3` gives Python 3.10 | Symlink points to `/usr/bin/python3` (system) | Use `.venv/bin/python3.11` explicitly |
| `cannot import name '_core'` | `python/build/` missing from sys.path | Add both `python/build` and `python` to sys.path |
| `ImuCalibration: False` | pycolmap loaded from wrong location | Ensure `python/build` is first on sys.path before any other pycolmap |
| `AttributeError: module 'importlib.util' has no attribute 'load_from_spec'` | Typo — correct call is `module_from_spec` | Use `importlib.util.module_from_spec(_spec)` then `_spec.loader.exec_module(_vi)` |
| `AssertionError: non_ref_sensors` | Stereo rig in reconstruction | Only feed cam0 poses — `features_to_colmap.py` does this automatically |
| `len(ms) == 0` for all intervals | IMU timestamps don't overlap image timestamps | Check +10000 s offset is consistent in both `_imu.npy` and `image_timestamps.npy` |
| Empty `points3D.txt` | Used `openvins_to_colmap.py` instead of `features_to_colmap.py` | Re-run with `features_to_colmap.py` (needs `_feats.txt`) |
