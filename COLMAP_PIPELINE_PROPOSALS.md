# COLMAP Pipeline Analysis and Proposals for Insta360 on Construction Site

**Date**: 2026-04-21
**Context**: Hilti SLAM Challenge 2026 — multi-floor construction site sequences, Insta360 One X2 stereo fisheye rig
**Based on**: Analysis of `/cluster/scratch/bsekeroglu/dataset/colmap_dataset` runs

---

## Where We Stand

The existing work is well-documented and methodical. Honest state of the pipeline:

| Config | Coverage | BA Cost | Status |
|---|---|---|---|
| Original (5Hz, overlap=10) | 0–36% | — | Obsolete |
| `cfg_universal_10hz` | 100% (3/3) | 0.713–0.750 px | Production baseline |
| `cfg_10hz_jointBA` | 99.8–100% | 0.644–0.728 px | GT study, running |
| `cfg_10hz_jointBA_robust` | 100% (5/5) | 0.644–0.728 px | Production, running |
| `cfg_10hz_refineDistortion` | 100% (expected) | ≤0.728 px | Running in parallel |
| `cfg_30hz_jointBA` | 100% (expected) | ~0.678 px (exp_D) | GT study, running |

**Coverage is solved.** The remaining question is ATE on the Hilti leaderboard, and the GT study is correctly designed to answer it. What follows addresses gaps in the current experiments that could meaningfully improve absolute pose accuracy beyond what the current 2×2 matrix will tell you.

---

## Root Causes of Residual Error the Current Pipeline Does Not Address

### 1. SIFT's Weakness on Construction-Site Images

SIFT is rotation/scale-invariant but struggles with:

- **Motion blur**: at 30 Hz with a handheld rig traversing construction site geometry, individual frames have significant translational blur. SIFT keypoint detection is gradient-based and degrades sharply under blur.
- **Repetitive textures**: scaffolding grid, rebar, tiling, concrete forms. SIFT descriptors on these are nearly identical across spatial locations, producing ambiguous matches that RANSAC must sort through. The `abs_pose_min_inlier_ratio=0.10` setting is a symptom of this — frames where 90% of putative matches are wrong are being accepted.
- **Low-texture surfaces**: fresh concrete, plywood shuttering, bare metal. SIFT produces few features per frame — the `abs_pose_min_num_inliers=15` fix was needed precisely because of this.

SIFT is already at or near its ceiling for this scene type.

### 2. Temporal-Only Frame Selection Ignores Motion

The 10Hz subsampling selects frames at fixed time intervals regardless of camera motion. In a construction site walk:
- Slow sections (camera stationary, checking work): high temporal overlap, redundant frames selected
- Fast sections (walking through corridors, stairwells): large inter-frame motion, increased blur

The mapper would benefit from selecting frames based on **parallax quality** (enough baseline for good triangulation) and **feature quality** (low motion blur) rather than clock ticks.

### 3. Loop Closure is Vocabulary-Tree Limited

The pipeline uses a generic vocabulary tree. Construction sites have highly domain-specific appearance: the same visual vocabulary appears on every floor (floor tiles, windows, cable runs). A generic VT trained on outdoor/mixed scenes has lower recall for within-building revisitation. The `loop_detection_num_images=100` setting partially compensates but cannot fix the underlying retrieval problem.

### 4. The "Initial Pair Lottery" is Not Fully Eliminated

`cfg_10hz_jointBA` got 99.8% on floor_2/run_1 (one trial) and exp_A got 2.5% on the same data with the same thresholds. The difference is random initial pair selection. `abs_pose_min_num_inliers=15` mitigates this by making early mapper growth possible regardless of which pair is chosen — but a bad initial pair still sets the map's orientation and initial scale, propagating into all subsequent triangulation. IMU could eliminate this lottery entirely.

### 5. Factory Calibration Residual (Principal Point)

The `cfg_10hz_refineDistortion` experiment tests distortion (k1–k4) correctly. However, only refining k1–k4 in the final BA may be too conservative. Factory fisheye calibrations on the Insta360 One X2 are known to have small but non-zero residual errors in the principal point (cx, cy shifts up to 2–3 px from temperature and mechanical variation). These are correlated with lateral translation bias in long corridors — exactly the geometry present in construction site floor-walking sequences. This is not currently tested.

---

## Proposed Next Pipeline: `cfg_30hz_superpoint_imu`

This is a staged proposal. Each stage builds on the last. Run them in order of increasing effort.

---

## Stage 1: Low-Effort Additions to the Current GT Study (This Week)

### 1a. Run `cfg_30hz_refineDistortion` Now

There is a planned v3 follow-up (`cfg_30hz_refineDistortion`) being deferred until both 10Hz and 30Hz results are in. Don't wait. The 30Hz mapper is the long pole; adding distortion refinement to its final BA costs ~0 extra compute. Submit it in parallel with the existing GT configs. The conditional logic from §13.7 of the GT analysis doc is overly conservative — the distortion parameters are separable from density, and the 8 extra DoFs are negligible against a 30Hz problem size. The full 2×2 matrix is wanted, not the partial one.

```bash
# For each GT run, add cfg_30hz_refineDistortion as a fourth config
# Identical to cfg_30hz_jointBA except refine_extra_params=1 in the final BA
```

### 1b. Increase Final BA Iterations for the 30Hz Configs

`max_num_iterations=200` was chosen as "2× default" for the 10Hz problem. At 30Hz, the joint problem has 3× more poses and proportionally more 3D points. The problem may not converge in 200 iterations. Consider `cfg_30hz_jointBA_400iter` or just increase to 400 for all 30Hz runs. The cost is linear in iterations and typically < 30 min additional for 400 vs 200.

### 1c. Check Whether BA Actually Converges (Do This First)

Parse the `final_ba_*.err` SLURM logs for the convergence line. COLMAP reports per-iteration diagnostics. If the final iteration shows a non-negligible `cost_change`, BA hit the iteration cap before converging — a free 10–15% ATE improvement sits there.

```bash
for log in /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_gt_runs/*/*/cfg_30hz_jointBA/slurm_scripts/final_ba_*.err; do
    echo "=== $log ==="
    grep "Iteration" "$log" | tail -5
    grep "Final cost" "$log"
done
```

If `cost_change` is still > 1e-6 on the last iteration, the 30Hz BA needs more iterations and the current accuracy numbers are not the best achievable with this pipeline.

---

## Stage 2: IMU-Guided Frame Selection and Matching (1–2 Days of Engineering)

The Hilti sensor platform has a high-quality IMU. Using it changes the two most non-deterministic parts of the pipeline: frame selection and initial pair choice.

### 2a. Motion-Adaptive Frame Selection Instead of Fixed 10Hz

Replace `10hz_frames.txt` (every 3rd frame) with a **velocity-gated frame list**:

```python
# Pseudo-code: compute per-frame angular velocity from IMU, select frames where:
# - angular velocity is below a blur threshold (avoids blurry frames)
# - inter-frame translation since last selected frame exceeds a minimum baseline
# - inter-frame rotation since last selected frame is below a maximum rotation
# Target: ~same frame count as 10Hz but geometrically informed

frames = []
last_pose = identity
for frame, imu_pose in enumerate(preintegrated_poses):
    delta_t = norm(imu_pose.t - last_pose.t)
    delta_r = angle(imu_pose.R @ last_pose.R.T)
    angular_vel = imu_omega[frame]

    if angular_vel < 1.5  # rad/s
    and delta_t > 0.05    # metres
    and delta_r < 20:     # degrees
        frames.append(frame)
        last_pose = imu_pose
```

Expected gains: fewer degenerate same-pose frames, fewer high-blur frames, better triangulation angles in the mapper.

### 2b. IMU-Seeded Initial Pair Selection

Add a pre-processing step that uses IMU preintegration to identify frame pairs with:
- Translation norm between 0.3–2.0 m (good triangulation baseline)
- Rotation < 30° (features still visible in both)
- Both frames in the low-blur category

Write these as `init_image_id1` / `init_image_id2` candidates. Either pass them to the mapper via COLMAP's `--Mapper.init_image_id1` flag or enumerate the top-5 candidates and retry the mapper if the first choice leads to small model growth. This eliminates the lottery entirely.

### 2c. IMU-Guided Spatial Matching (Supplement to Sequential Matcher)

The sequential matcher covers temporal neighbors (overlap=50, quadratic). It relies on the vocabulary tree for spatial revisitation. A complementary approach: use IMU-preintegrated poses to find **spatially close but temporally distant** frame pairs and add them as additional match candidates.

```bash
# After feature extraction, before sequential matcher:
# 1. Preintegrate IMU → rough pose per frame
# 2. Build k-d tree on rough positions
# 3. For each frame, query k nearest spatial neighbors not already in temporal window
# 4. Export these as additional pairs to COLMAP's custom matching list
colmap custom_matcher \
    --database_path database.db \
    --CustomMatching.match_list_path spatial_pairs.txt \
    --CustomMatching.match_type pairs
```

This is especially valuable for re-visiting construction areas. A worker walks through the same corridor at different times, and the sequential matcher will not bridge those visits (temporal distance > overlap=50), but the spatial matcher will.

---

## Stage 3: Learning-Based Features — The Highest Ceiling Change

This is the single change with the most headroom for improvement. Replace SIFT with **SuperPoint + LightGlue** (or MASt3R features as an alternative).

### Why This Matters Here Specifically

- SuperPoint is trained end-to-end for repeatability on diverse scenes including indoor/industrial
- It detects corners and blob-like structures, not just gradient peaks — better on concrete and metal
- LightGlue performs cross-attention matching: it reasons about which matches are geometrically consistent jointly, not independently per keypoint. This dramatically reduces the outlier ratio, making `abs_pose_min_inlier_ratio=0.10` unnecessary
- Both handle motion blur better than SIFT because they operate on learned feature maps, not raw gradient magnitude

### Implementation Path Using hloc

```bash
# 1. Install hloc (hierarchical localization toolbox)
pip install hloc  # or clone from github.com/cvg/Hierarchical-Localization

# 2. Extract SuperPoint features from fisheye images
python -c "
from hloc import extract_features, match_features
from pathlib import Path

image_dir = Path('/cluster/scratch/bsekeroglu/dataset/colmap_dataset/data_30Hz/floor_2/2025-10-28_run_1/images/rig1/cam0')
outputs = Path('/tmp/hloc_features')

feature_conf = extract_features.confs['superpoint_aachen']
feature_conf['model']['max_keypoints'] = 4096
features = extract_features.main(feature_conf, image_dir, outputs)
"

# 3. Import SuperPoint features into COLMAP database via hloc's colmap_from_matches.py
# Key: use --skip_geometric_verification=True since LightGlue already verifies

# 4. Run LightGlue matching (sequential pairs + loop closure retrieval via NetVLAD/MixVPR)
python -c "
from hloc import match_features, pairs_from_retrieval
# NetVLAD global retrieval for loop closure (replaces vocab tree)
retrieval_conf = extract_features.confs['netvlad']
# ...
"
```

### Configuration Comparison

| Stage | Current | Proposed |
|---|---|---|
| Feature detector | SIFT (COLMAP built-in) | SuperPoint (hloc → COLMAP DB import) |
| Feature descriptor | SIFT | SuperPoint |
| Sequential matching | COLMAP SequentialMatcher | LightGlue on temporal pairs |
| Loop closure retrieval | Vocab tree (BoW) | NetVLAD → LightGlue verification |
| Match filter | Mutual NN + RANSAC | LightGlue confidence threshold (no RANSAC needed) |
| `max_num_features` | 8192 (default SIFT) | 4096 SuperPoint (higher quality, fewer outliers) |

Expected outcome: `abs_pose_min_inlier_ratio` can be raised back toward 0.20+ because the outlier ratio drops from ~90% (SIFT on construction) to ~20–30% (LightGlue). `abs_pose_min_num_inliers` can potentially be raised to 20–25. Fewer degenerate registrations → better 3D point quality → better final BA.

**Compute estimate**: SuperPoint extraction is ~2× slower than SIFT on CPU, ~2× faster on GPU. LightGlue matching is faster than SIFT mutual-NN at the same pair count (no FLANN tree). Overall pipeline time is similar or slightly faster on GPU.

---

## Stage 4: Principal Point Refinement (Targeted Experiment)

From the GT analysis, refining focal length and principal point was rejected wholesale. But the risk profile is asymmetric:

- **Focal length**: correlated with scale → dangerous, keep frozen ✓
- **Principal point (cx, cy)**: correlated with lateral translation bias. On sequences with dominant lateral motion (walking down corridors), principal point error manifests as a consistent lateral offset across the whole trajectory — exactly the kind of systematic error that hurts ATE without much affecting reprojection.

**Targeted test**: Run a single `cfg_10hz_jointBA_refinePP` config on floor_UG1 (the easy, already-100% sequence with the cleanest baseline). Refine cx/cy in the final BA while keeping focal, distortion, and rig frozen. Check:

1. How much do cx/cy move from factory values? (> 3px = suspicious)
2. Does the Sim3 scale residual change? (scale coupling = dangerous)
3. Does ATE improve?

If the answer is consistent improvement with stable scale, add principal point refinement to the production config. If scale drifts, drop it. Run on the easy sequence first to avoid confounding with coverage issues.

---

## Summary: Expected Impact vs Effort

| Proposal | Effort | Expected ATE Gain | Coverage Risk | Priority |
|---|---|---|---|---|
| `cfg_30hz_refineDistortion` (Stage 1a) | 1 hour (script copy) | Small–medium | Zero | **Do immediately** |
| Check BA convergence on 30Hz (Stage 1c) | 30 min | Potentially large if BA not converging | Zero | **Do immediately** |
| IMU motion-adaptive frame selection (Stage 2a) | 1–2 days | Medium | Low | High |
| IMU spatial matching pairs (Stage 2c) | 1 day | Medium (helps hard sequences) | Zero | High |
| SuperPoint + LightGlue features (Stage 3) | 3–5 days | **High (largest ceiling)** | Low (more matches) | High, after GT results |
| IMU-seeded initial pair (Stage 2b) | 1 day | Medium (eliminates lottery) | Positive (reduces variance) | Medium |
| Principal point refinement test (Stage 4) | 2 hours + eval | Small–medium | Medium (test easy sequence first) | After GT results |
| `cfg_30hz_jointBA_400iter` | 1 hour | Small (if BA was converging) | Zero | Low |

---

## Recommended Sequence

1. **Now**: Check BA convergence on all running 30Hz jobs (Stage 1c). Submit `cfg_30hz_refineDistortion` (Stage 1a).
2. **This week**: Wait for GT study results. Parse ATE numbers with `evo_ape` / `evo_rpe` (protocol already in `1_GT_ACCURACY_EXPERIMENTS.md §7`).
3. **After GT results**: If ATE gap is > 5 cm RMSE on hard sequences, invest in SuperPoint + LightGlue (Stage 3) — this is the largest remaining lever. If gap is < 5 cm, IMU-guided improvements (Stage 2) are likely sufficient.
4. **Parallel**: IMU spatial matching (Stage 2c) is low-risk and complements everything — can be developed alongside the GT study.

The highest-leverage single change is **SuperPoint + LightGlue** (Stage 3). Construction site Insta360 data is precisely the use case where SIFT degrades most — repetitive industrial textures, variable lighting, motion blur — and where learned features demonstrate the largest gap in the literature. Everything else is incremental. Get the ATE numbers from the current GT study first, then invest in feature replacement if the gap to ground truth is still meaningful.
