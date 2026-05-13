# VI Optimization Input Paths: OpenVINS vs COLMAP

Reference for feeding OpenVINS output into `colmap_imu/python/examples/vi_optimization.py`.

---

## Overview

`vi_optimization.py` needs a COLMAP Reconstruction with:
- Camera intrinsics (`cameras.txt`)
- Per-frame poses as `cam_from_world` (`images.txt`)
- 3D landmark positions with per-image 2D observation lists (`points3D.txt`)
- Image timestamps as a numpy dict `{colmap_image_id → timestamp_ns}`
- IMU measurements as a numpy array `(N, 7): [ts_ns, ax, ay, az, gx, gy, gz]`

Two paths produce this from an OpenVINS run.

---

## Path A — OpenVINS direct (implemented)

OpenVINS's `loop_feats` topic publishes at each camera frame:
- 3D position of every currently-active tracked feature (world frame)
- Raw pixel `(u, v)` in cam0
- Stable `feature_id` integer (unique per track lifetime)

By accumulating these messages over a run, the full track history is recovered without any additional feature extraction or matching.

### Steps

1. **Re-run OpenVINS** with `run_openvins_with_tracks.sbatch`
   — same config + 0.5× rate as the submission runs, adds `feature_logger.py` as a parallel subscriber
2. **Post-process automatically** — `features_to_colmap.py` runs at the end of the sbatch and writes the COLMAP reconstruction

### Scripts

| Script | Location |
|---|---|
| `feature_logger.py` | `challenge_tools_ros/gt_helper/feature_logger.py` |
| `features_to_colmap.py` | repo root |
| `run_openvins_with_tracks.sbatch` | `/cluster/scratch/ademirtas/` |

### Output layout (per run)

```
results/openvins_vi_ready/<run_name>.txt           ← TUM trajectory
results/openvins_vi_ready/<run_name>_feats.txt     ← raw feature log
results/openvins_vi_ready/<run_name>_colmap/
    cameras.txt
    images.txt
    points3D.txt
    image_timestamps.npy
```

IMU data is extracted separately with `bag_to_imu_npy.py` (runs inside Apptainer).

### Limitation: broken tracks

OpenVINS uses KLT (optical flow), which has no feature descriptors. When a track
is lost (occlusion, fast motion), the feature is dropped. If the same physical
point is re-detected later it receives a **new feature_id** — no connection to
the previous track. This produces:

- Duplicate 3D points for the same physical location
- Shorter tracks → weaker reprojection constraints in BA

For slow indoor traversals with 0.5× playback this is minor. For environments
with heavy occlusion or fast motion, tracks are short and the visual BA term
contributes less.

---

## Path B — COLMAP (not yet implemented)

Runs SIFT or ALIKED feature extraction on images extracted from the bag, then
uses descriptor matching to find correspondences (including across track breaks),
then triangulates 3D points using the fixed OpenVINS poses.

### Steps

1. **Extract images from bag**
   ```bash
   python3 challenge_tools_ros/bag_helper/ros2bag_to_euroc.py \
       --bag <bag_dir> --output <euroc_dir> --topics /cam0/image_raw/compressed
   ```

2. **Convert OpenVINS TUM → COLMAP pose init**
   ```bash
   python3 openvins_to_colmap.py \
       --tum <run>.txt \
       --calib config/hilti_openvins/kalibr_imucam_chain.yaml \
       --output colmap_init/ \
       --images <euroc_dir>/cam0/data/
   ```

3. **COLMAP feature extraction** (slow — ~12k images per run at 30Hz)
   ```bash
   colmap feature_extractor \
       --database_path db.db \
       --image_path <euroc_dir>/cam0/data/ \
       --ImageReader.camera_model OPENCV_FISHEYE \
       --ImageReader.camera_params "465.3,465.3,730.0,720.1,0.026,-0.011,-0.0017,0.00015"
   ```
   Speed up by subsampling: only extract every Nth frame (e.g. `--ImageReader.single_camera 1`
   and pre-filter the image directory to keyframes).

4. **COLMAP sequential matching**
   ```bash
   colmap sequential_matcher \
       --database_path db.db \
       --SequentialMatching.overlap 10
   ```

5. **Triangulate with fixed poses**
   ```bash
   colmap triangulate_points \
       --database_path db.db \
       --image_path <euroc_dir>/cam0/data/ \
       --input_path colmap_init/ \
       --output_path colmap_triangulated/ \
       --Mapper.fix_existing_images 1
   ```

6. **Extract IMU data** (inside Apptainer)
   ```bash
   python3 bag_to_imu_npy.py <bag_dir> imu.npy
   ```

7. **Run vi_optimization.py** with `colmap_triangulated/` + `image_timestamps.npy` + `imu.npy`

### Scripts

| Script | Location |
|---|---|
| `openvins_to_colmap.py` | repo root |
| `bag_to_imu_npy.py` | repo root |

### Advantage over Path A

- Descriptor matching (SIFT/ALIKED) re-identifies features across track breaks → single merged track per physical point → longer, more constraining observations for BA
- ALIKED + LightGlue (if `colmap_imu` built with `COLMAP_ONNX_ENABLED`) is significantly more robust than SIFT on low-texture construction surfaces

---

## Comparison

| | Path A (OpenVINS) | Path B (COLMAP) |
|---|---|---|
| Extra compute | Re-run OpenVINS (~same time as submission) | Image extraction + feature extraction + matching (slow) |
| Images needed | No | Yes |
| Feature descriptors | No | Yes (SIFT or ALIKED) |
| Track re-ID across loss | No — new ID per re-detect | Yes — descriptor matching merges broken tracks |
| Track length | Shorter (lost when out of view) | Longer (re-matched across breaks) |
| Visual BA strength | Weaker for occluded/fast sequences | Stronger |
| Implementation status | Done | Scripts written, not tested end-to-end |
| When to use | Default — sufficient for slow indoor runs | If vi_optimization.py BA doesn't converge well |

---

## IMU calibration for vi_optimization.py (Hilti sensor — Bosch BMI085)

Replace the Project Aria defaults in `vi_optimization.py`'s `run()`:

```python
imu_calib = pycolmap.ImuCalibration()
imu_calib.imu_rate                  = 200.0          # Hz (Insta360 IMU, not 1000Hz raw)
imu_calib.gravity_magnitude         = 9.81
imu_calib.gyro_noise_density        = 1.7e-4         # rad/s/sqrt(Hz)
imu_calib.accel_noise_density       = 2.0e-3         # m/s^2/sqrt(Hz)
imu_calib.bias_gyro_random_walk_sigma  = 1.9e-5      # rad/s^2/sqrt(Hz)
imu_calib.bias_accel_random_walk_sigma = 3.0e-4      # m/s^3/sqrt(Hz)
```

Verify against `kalibr_imu_chain.yaml` in the config directory — the OpenVINS
noise values there are the most run-specific measurements available.

---

## Rig constraint in vi_optimization.py

`vi_optimization.py` asserts a trivial rig (single camera per frame). The Hilti
bags use stereo (cam0 + cam1 as a rigid pair), so only cam0 poses are fed to
vi_optimization.py. See `COLMAP_IMU_FORK_CAPABILITIES.md` for the cam1
recomposition workaround if stereo output is needed post-optimization.
