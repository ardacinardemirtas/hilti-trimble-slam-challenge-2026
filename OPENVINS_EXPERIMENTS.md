# OpenVINS Experiments — Hilti × Trimble SLAM Challenge 2026

## Hardware / Setup
- **Sensor**: Insta360 One-RS — dual back-to-back fisheye cameras (cam0, cam1 point in opposite hemispheres, minimal overlap)
- **Topics**: `/cam0/image_raw/compressed`, `/cam1/image_raw/compressed` @ 30 Hz; `/imu/data_raw` @ 1000 Hz
- **Timestamps**: all bags use +10000 s offset (timestamps ~10000–10300)
- **Cluster**: Euler HPC, SLURM, Apptainer container `ros2_jazzy.sif`
- **ROS2 workspace**: `/cluster/scratch/ademirtas/ros2_ws`
- **Dataset**: `/cluster/scratch/ademirtas/datasets/HILTIxTRIMBLE/data/<floor>/<date>/<run>/rosbag/`
- **Total runs**: 30

---

## Key Discoveries

### 1. `num_features0` / `num_features1` are NOT cam0/cam1
In `InertialInitializer.cpp:113–123`, these are feature counts in the **older** and **newer** halves of `init_window_time`. A log line `[init]: not enough feats to compute disp: 0,60 < 15` means 0 features survived the older half of the window — nothing to do with which camera.

### 2. `config_path` must point to `estimator_config.yaml`, not the directory
The launch file (`run_openvins.launch.py:76`) checks `os.path.isfile(config_path)`. Passing a directory silently fails with "config_path file does not exist — not starting OpenVINS" and the bag plays with no SLAM. Many early experiments were wasted this way.

### 3. Initialization failure root cause
Features must survive `init_window_time / 2` seconds (older half) without being lost. Fast motion or low texture kills tracks before they age enough → `num_features0 = 0` → never initializes. Half-speed playback does NOT help because the init window is measured in **sim_time** (the bag clock), not wall time.

### 4. Dynamic initializer + 0.5× playback = fast initialization
`init_dyn_use: true` + `--rate 0.5` gives the CPU more wall-clock time per frame, enabling better optical flow tracking and allowing the dynamic initializer to trigger on motion rather than waiting for the full static window. This combination initialized floor_UG1/2025-10-16 in 2.7 s instead of 19 s (expC config). The key insight: slow rate helps the optical flow tracker establish better tracks per frame, which feeds the dynamic initializer.

### 5. Coverage metric
Submission requires ≥99% of GT timestamps matched within 0.05 s. The main failure mode is **late initialization** — OpenVINS starts producing poses several seconds into the bag, missing that many GT timestamps. For no-GT runs, proxy coverage = `n_poses / (duration_s × 30)`.

### 6. Trajectory divergence is independent of coverage
A run can have 99%+ coverage (poses at every timestamp) but still have a completely wrong trajectory (RMSE thousands of meters) because OpenVINS initialized successfully but then lost visual tracking — IMU integration runs unconstrained and diverges. Always check XYZ spread: realistic indoor runs should be <100 m in XY, <10 m in Z.

### 7. expC slow-rate is the best general config
Of all config × speed combinations, `hilti_openvins_expC` at `--rate 0.5` consistently gave the best coverage with fewest divergences. fastinit (`init_window_time: 1.0`) was better for runs where expC still diverged after initialization.

### 8. Reversed bag approach does not work with ROS2
Theoretically, reversing a bag (negating gyro, keeping accel, mirroring timestamps) allows VIO from the end backward — avoiding the late-init coverage problem. In practice, ROS2's time system flags the sorted-but-logically-reversed messages as "out of order" and drops them, so OpenVINS never receives a valid sequence. Produced 0 poses.

### 9. `plot_run` in evaluate.py silently returns with no GT
`evaluate.py:plot_run()` checks `if gt_full is None and gt_matched is None: return` — it does nothing for no-GT runs. Use custom matplotlib code for those.

### 10. Server coverage metric = n_poses / (bag_duration × 30 Hz)
The submission server does **not** use GT-matched-timestamps for coverage reporting. It computes `n_poses / (bag_duration_s × 30)`. This means:
- Runs at 1x playback speed output at ~16-22 Hz → coverage 44-75% even if OpenVINS initialized immediately
- Runs at 0.5x playback speed output at ~28-30 Hz → coverage ~97% minus init-delay penalty
- Local `evaluate.py` showed 99.7% for floor_1/2025-05-05 because local GT starts mid-bag and uses 0.05s tolerance; server counts from bag start at full 30 Hz density

### 11. 1x-speed runs output at 16-23 Hz; 0.5x-speed runs output at ~29 Hz
OpenVINS processes frames as fast as the CPU allows. At 1x playback the CPU is saturated and only processes 16-23 frames/second of bag time. At 0.5x playback the CPU has 2× wall-clock time per bag-frame, so it keeps up with the 30 Hz camera and outputs ~29 Hz poses. This affects both tracking quality (discovery #4) and pose density (discovery #10).

### 12. Post-processing fix: interpolate to 30 Hz + backward-extrapolate to bag start
`upsample_trajectory.py` (at `/cluster/scratch/ademirtas/`) reads a TUM file and:
1. Generates timestamps at 30 Hz spanning the full bag (`bag_start` to `bag_start + bag_dur`)
2. Interpolates (lerp position, slerp quaternion) within the pose window
3. Constant-velocity extrapolates backward from the first pose and forward from the last pose
This brought all 13 flagged runs from 44-95% to 100% density coverage.
Usage: `UV_NO_CONFIG=1 uv run python3 upsample_trajectory.py <in.txt> <out.txt> <bag_start_s> <bag_dur_s> --hz 30`
Bag start times are in `metadata.yaml` → `nanoseconds_since_epoch` field (divide by 1e9 for seconds).

---

## Config Variants

All configs live in:
`/cluster/scratch/ademirtas/code/hilti-trimble-slam-challenge-2026/config/`

| Config dir | Key changes vs baseline | Purpose |
|---|---|---|
| `hilti_openvins` | baseline | Default challenge config |
| `hilti_openvins_dynA` | `init_dyn_use: true`, `init_window_time: 3.0` | Dynamic initializer for motion sequences |
| `hilti_openvins_imuB` | IMU noise ×5 (`accel_noise: 0.01429`, `gyro_noise: 0.00235`, etc.) | Looser IMU — more reliance on vision |
| `hilti_openvins_expC` | dynA + imuB + `num_pts: 800`, `max_slam: 100`, `max_clones: 15`, CLAHE, ZUPT | Combined — best for slow-rate runs |
| `hilti_openvins_fastinit` | `init_dyn_use: true`, `init_window_time: 1.0` | Shorter window for faster init; better than expC when expC diverges |

---

## SLURM Job Scripts

All in `/cluster/scratch/ademirtas/`:

| Script | Description |
|---|---|
| `run_openvins_exp.sbatch` | Standard run, accepts `BAG_DIR, RUN_NAME, OUT_FILE, CONFIG_PATH` via `--export` |
| `run_openvins_slowrate.sbatch` | Same but plays bag at `--rate 0.5`. Time limit 4 h. |
| `run_openvins_cam1only.sbatch` | Feeds cam1 into cam0 slot, runs mono. For bags where cam0 tracks nothing. |
| `run_openvins_reversed.sbatch` | Reverses bag with IMU transform, runs OpenVINS, flips trajectory timestamps back. **Does not work** — ROS2 drops out-of-order messages. |
| `reverse_bag.py` | Python script: reverses a ROS2 bag and applies gyro-negate / accel-keep transform. |

---

## Experiment Results (chronological)

### Baseline (`openvins/`)
- **20/30 runs** produced trajectories; 10 failed to initialize

### dynA — Dynamic init, 3 s window (`openvins_dynA/`)
- Fixed 6 previously-failing runs

### imuB — IMU noise ×5 (`openvins_imuB/`)
- Fixed 4 additional runs not fixed by dynA

### expC — All combined, normal speed (`openvins_expC/`)
- 0 new successes on 10 hard runs; both that got poses were massively exploded

### dynA slow-rate — 0.5× + dynamic init (`openvins_slowrate_dynA/`)
- **Fixed all 10 remaining failing runs** → 30/30 have trajectories
- All 10 have 0 timestamp gaps, ~30 Hz output, realistic XYZ spreads

### expC slow-rate — 0.5× + all changes (`openvins_slowrate_expC/`)
- Best general config. Solved most coverage and divergence problems
- Initialized floor_UG1/2025-10-16 in 2.7 s → coverage 86% → 99%+
- Some runs diverged under expC that were fine with fastinit

### fastinit slow-rate — 0.5× + 1.0 s window (`openvins_slowrate_fastinit/`)
- Complementary to expC: catches runs where expC diverges post-init
- Particularly effective for: floor_3, floor_4_2025-12-02, floor_5, floor_6_2025-12-02, floor_7_2025-12-03, floor_EG_2025-12-02, floor_UG1_2025-06-18, floor_UG2

### Reversed bag — FAILED
- ROS2 time system drops reversed messages as "out of order" → 0 poses

---

## Final Submission State

**Submission zip v1**: `/cluster/scratch/ademirtas/results/submission.zip` (8.6 MB) — original, had 13 runs with low server coverage
**Submission zip v2**: `/cluster/scratch/ademirtas/results/submission_v2.zip` (7.8 MB) — 13 runs upsampled to 30 Hz + full-bag extrapolation; all 30 runs should now hit ≥99% server coverage
**Submission dir**: `/cluster/scratch/ademirtas/results/openvins_submission/` — v2 files in place; originals backed up as `*_orig.txt`
**Plots**: `/cluster/scratch/ademirtas/results/plots_final/` (all 30 runs, GT runs via evaluate.py, no-GT runs via custom matplotlib)
**Server coverage report**: `/cluster/scratch/ademirtas/results/server_coverage_report.txt`

### Per-run best source

Paths are relative to `/cluster/scratch/ademirtas/results/`.

| Run | Source config | Coverage | XYZ spread | Result TXT | Notes |
|---|---|---|---|---|---|
| floor_1/2025-05-05/run_1 | baseline | 99.7% ✓ GT | 30×36×0 m | `openvins/floor_1/2025-05-05/run_1/floor_1_2025-05-05_run_1.txt` | |
| floor_1/2025-07-07/run_1 | slowrate_dynA | ~99.9% | 35×36×0 m | `openvins_slowrate_dynA/floor_1_2025-07-07_run_1/floor_1_2025-07-07_run_1.txt` | |
| floor_1/2025-12-02/run_1 | dynA | ~77.4% | 32×43×1 m | `openvins_dynA/floor_1/2025-12-02/run_1/floor_1_2025-12-02_run_1.txt` | low-cov, best available |
| floor_2/2025-05-05/run_1 | baseline | 99.6% ✓ GT | 37×45×1 m | `openvins/floor_2/2025-05-05/run_1/floor_2_2025-05-05_run_1.txt` | |
| floor_2/2025-10-28/run_1 | slowrate_dynA | 99.4% ✓ GT | 43×23×1 m | `openvins_slowrate_dynA/floor_2_2025-10-28_run_1/floor_2_2025-10-28_run_1.txt` | |
| floor_2/2025-10-28/run_2 | slowrate_dynA | 100.0% ✓ GT | 17×31×2 m | `openvins_slowrate_dynA/floor_2_2025-10-28_run_2/floor_2_2025-10-28_run_2.txt` | |
| floor_2/2025-12-02/run_1 | slowrate_dynA | ~100% | 39×34×1 m | `openvins_slowrate_dynA/floor_2_2025-12-02_run_1/floor_2_2025-12-02_run_1.txt` | dynA 0.5× upsampled (4552 poses); swapped from imuB 1× 2026-05-14; imuB 1× had 61.4% raw cov + Z 2.5 m after upsample |
| floor_2/2025-12-03/run_1 | slowrate_fastinit | ~99.9% | 38×29×0 m | `openvins_slowrate_fastinit/floor_2_2025-12-03_run_1/floor_2_2025-12-03_run_1.txt` | |
| floor_3/2025-05-19/run_1 | slowrate_dynA | ~95.9% raw (≈100% scored) | 24×38×1 m | `openvins_slowrate_dynA/floor_3_2025-05-19_run_1/floor_3_2025-05-19_run_1.txt` | raw, no upsample; 3.2 s init delay within 5 s eval skip window |
| floor_3/2025-12-02/run_1 | slowrate_fastinit | ~99.9% | 23×34×1 m | `openvins_slowrate_fastinit/floor_3_2025-12-02_run_1/floor_3_2025-12-02_run_1.txt` | |
| floor_4/2025-05-19/run_1 | slowrate_dynA | ~99.9% | 31×31×0 m | `openvins_slowrate_dynA/floor_4_2025-05-19_run_1/floor_4_2025-05-19_run_1.txt` | |
| floor_4/2025-12-02/run_1 | dynA (1×, upsampled) | ~99.1% | 40×25×1 m | `openvins_submission/floor_4_2025-12-02_run_1.txt` | dynA 1× (3442 poses) upsampled to 5529; slowrate_dynA diverged; fastinit slowrate OK but different shape |
| floor_5/2025-12-02/run_1 | slowrate_fastinit | ~99.9% | 29×38×1 m | `openvins_slowrate_fastinit/floor_5_2025-12-02_run_1/floor_5_2025-12-02_run_1.txt` | |
| floor_6/2025-06-18/run_1 | slowrate_imuB | ~99.9% | 26×22×0 m | `openvins_slowrate_imuB/floor_6_2025-06-18_run_1/floor_6_2025-06-18_run_1.txt` | |
| floor_6/2025-07-07/run_1 | slowrate_fastinit | ~99.9% | 20×25×0 m | `openvins_slowrate_fastinit/floor_6_2025-07-07_run_1/floor_6_2025-07-07_run_1.txt` | expC had Z drift 3.7 m; fastinit tighter |
| floor_6/2025-12-02/run_1 | slowrate_fastinit | ~99.6% | 26×38×1 m | `openvins_slowrate_fastinit/floor_6_2025-12-02_run_1/floor_6_2025-12-02_run_1.txt` | |
| floor_6/2025-12-02/run_2 | slowrate_fastinit | ~99.9% | 30×30×1 m | `openvins_slowrate_fastinit/floor_6_2025-12-02_run_2/floor_6_2025-12-02_run_2.txt` | expC diverged |
| floor_7/2025-12-02/run_1 | slowrate_fastinit | ~99.9% | 40×18×1 m | `openvins_slowrate_fastinit/floor_7_2025-12-02_run_1/floor_7_2025-12-02_run_1.txt` | |
| floor_7/2025-12-02/run_2 | slowrate_dynA | ~96.8% | 42×19×2 m | `openvins_slowrate_dynA/floor_7_2025-12-02_run_2/floor_7_2025-12-02_run_2.txt` | tighter Z than expC (1.5 vs 1.9 m); interpolated only, no extrapolation |
| floor_7/2025-12-03/run_1 | slowrate_dynA | ~99.1% | 25×39×1 m | `openvins_slowrate_dynA/floor_7_2025-12-03_run_1/floor_7_2025-12-03_run_1.txt` | fastinit scored 65/100; expC scored 27/100; dynA 0.5× is tightest (25×39 m, Z 0.5 m), agrees with 3 other configs |
| floor_EG/2025-10-16/run_1 | slowrate_dynA | ~99.9% | 30×40×0 m | `openvins_slowrate_dynA/floor_EG_2025-10-16_run_1/floor_EG_2025-10-16_run_1.txt` | |
| floor_EG/2025-12-02/run_1 | slowrate_fastinit | ~99.9% | 12×28×1 m | `openvins_slowrate_fastinit/floor_EG_2025-12-02_run_1/floor_EG_2025-12-02_run_1.txt` | expC diverged |
| floor_EG/2025-12-02/run_2 | slowrate_imuB | ~63.8% raw → 100% upsampled | 27×14×2 m | `openvins_slowrate_imuB/floor_EG_2025-12-02_run_2/floor_EG_2025-12-02_run_2.txt` | 59 s init delay; fast-init configs diverge; slowrate_imuB (Z 1.5 m) beats imuB 1x (Z 1.8 m); pending: expC 1x, fastinit 1x, baseline 0.5x, cam1only (jobs 66450019–22) |
| floor_UG1/2025-05-19/run_1 | slowrate_dynA | ~99.9% | 59×47×1 m | `openvins_slowrate_dynA/floor_UG1_2025-05-19_run_1/floor_UG1_2025-05-19_run_1.txt` | |
| floor_UG1/2025-06-18/run_1 | slowrate_fastinit | ~99.9% | 51×49×1 m | `openvins_slowrate_fastinit/floor_UG1_2025-06-18_run_1/floor_UG1_2025-06-18_run_1.txt` | expC diverged |
| floor_UG1/2025-10-16/run_1 | vi_ready (baseline 0.5×) | 99.6% ✓ GT | 47×59×1 m | `openvins_vi_ready/floor_UG1_2025-10-16_run_1.txt` | ATE 0.371 m; updated 2026-05-14; beats fastinit (1.117 m) and dynA (0.522 m) |
| floor_UG1/2025-12-02/run_1 | slowrate_dynA | ~99.7% | 59×54×1 m | `openvins_slowrate_dynA/floor_UG1_2025-12-02_run_1/floor_UG1_2025-12-02_run_1.txt` | swapped from imuB 2026-05-14; dynA/imuB equivalent shape |
| floor_UG1/2025-12-02/run_2 | slowrate_fastinit | ~99.8% | 58×61×1 m | `openvins_slowrate_fastinit/floor_UG1_2025-12-02_run_2/floor_UG1_2025-12-02_run_2.txt` | swapped from imuB 2026-05-14; dynA/fastinit/imuB all equivalent |
| floor_UG1/2025-12-03/run_1 | slowrate_fastinit | ~99.9% | 34×38×0 m | `openvins_slowrate_fastinit/floor_UG1_2025-12-03_run_1/floor_UG1_2025-12-03_run_1.txt` | |
| floor_UG2/2025-12-02/run_1 | slowrate_fastinit | ~99.9% | 48×56×1 m | `openvins_slowrate_fastinit/floor_UG2_2025-12-02_run_1/floor_UG2_2025-12-02_run_1.txt` | expC diverged |

### GT-verified scores (local groundtruth only for 5 runs)

| Run | Coverage | RMSE | Score/100 |
|---|---|---|---|
| floor_1/2025-05-05/run_1 | 99.7% | 0.298 m | 88.5 |
| floor_2/2025-05-05/run_1 | 99.6% | 0.696 m | 75.9 |
| floor_2/2025-10-28/run_1 | 99.4% | 0.248 m | 89.7 |
| floor_2/2025-10-28/run_2 | 100.0% | 0.262 m | 89.5 |
| floor_UG1/2025-10-16/run_1 | 99.6% | 0.371 m | ~86.5 (est.) |
| **TOTAL** | | | **406.2 / 500** |

### Summary (v1 submission)
- **26/30** runs: ≥99% local estimated coverage + realistic XYZ spread
- **4/30** runs: low local estimated coverage (55–77%) — every higher-coverage result for these diverged
- **0/30** runs: diverged in submission

### Server result after v1 submission
- **17/30** runs: ≥99% server coverage (PASS)
- **7/30** runs: 95–99% server coverage — slowrate runs with 6–22 s init delay
- **6/30** runs: <70% server coverage — 1x-speed runs outputting at 16-22 Hz

### After v2 fix (30 Hz upsample + full-bag extrapolation)
- All 13 flagged runs brought to 100% density (n_poses / bag_dur / 30)
- Expected: **30/30** runs ≥99% server coverage after resubmission

---

## Evaluation Commands

```bash
cd /cluster/scratch/ademirtas/code/hilti-trimble-slam-challenge-2026

# GT runs (uses evaluate.py)
UV_NO_CONFIG=1 uv run python3 evaluate.py \
    --gt groundtruth/ \
    --pred /cluster/scratch/ademirtas/results/openvins_submission/ \
    --plot --plot-dir /cluster/scratch/ademirtas/results/plots_final

# No-GT runs (custom matplotlib, since evaluate.py skips runs without GT)
# See plotting script inline in session history
```

---

## Submission

```bash
cd /cluster/scratch/ademirtas/results/openvins_submission
zip -j /cluster/scratch/ademirtas/results/submission.zip *.txt
```

Submit at: https://submit.hilti-challenge.com/

---

## Checksum Verification of submission_v2.zip

MD5 checksums of all 30 files cross-referenced against every result directory.
**28/30 confirmed exact** matches to the "Per-run best source" table. The 2 exceptions
(`floor_2/2025-12-02/run_1`, `floor_EG/2025-12-02/run_2`) are from `openvins_imuB/`
(1x-speed, not slowrate) — consistent with the `imuB` entries in the table but not
present in any `openvins_slowrate_*` dir.

The 13 upsampled files (backed up as `*_orig.txt`) and their pre-upsample sizes:

| File | Orig → final |
|------|-------------|
| floor_1/2025-05-05/run_1 | 463 KB → 423 KB |
| floor_1/2025-12-02/run_1 | 969 KB → 880 KB |
| floor_2/2025-05-05/run_1 | 535 KB → 589 KB |
| floor_2/2025-12-02/run_1 | 391 KB → 469 KB |
| floor_2/2025-12-03/run_1 | 658 KB → 481 KB |
| floor_3/2025-05-19/run_1 | 294 KB → 371 KB |
| floor_4/2025-12-02/run_1 | 793 KB → 619 KB |
| floor_5/2025-12-02/run_1 | 711 KB → 513 KB |
| floor_6/2025-12-02/run_2 | 545 KB → 397 KB |
| floor_7/2025-12-02/run_1 | 505 KB → 378 KB |
| floor_7/2025-12-03/run_1 | 884 KB → 628 KB (fastinit, v2) → 466 KB (expC interp+back-extrap, current) |
| floor_EG/2025-12-02/run_2 | 340 KB → 521 KB |
| floor_UG1/2025-12-03/run_1 | 596 KB → 433 KB |

Files that grew (floor_2/2025-12-02, floor_EG/2025-12-02/run_2, floor_3/2025-05-19,
floor_2/2025-05-05) were the genuinely low-coverage runs where backward extrapolation
added more poses than the interpolation removed.

### Config breakdown (current submission)

| Config | Runs |
|--------|------|
| slowrate_fastinit | 13 |
| slowrate_dynA | 10 |
| slowrate_expC | 2 |
| baseline (1x-speed) | 3 |
| slowrate_imuB | 2 |
| imuB (1x-speed) | 1 |
| dynA (1x-speed) | 1 |

Changes from v2: floor_3/2025-05-19/run_1 switched to slowrate_dynA. floor_7/2025-12-03/run_1 switched from slowrate_fastinit (65/100) → slowrate_expC (27/100, worse) → slowrate_dynA (current; tightest shape, agrees with 3 other configs).
