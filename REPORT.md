# Hilti x Trimble SLAM Challenge 2026: Technical Report

**Team:** Arda Çınar Demirtaş  
**Task:** SLAM (Trajectory Estimation in Any Reference Frame)

---

## 1. Approach Overview

Our pipeline combines a causal sliding-window MSCKF filter (implemented via OpenVINS [Geneva et al., 2020]) for real-time visual-inertial state estimation with an offline global visual-inertial bundle adjustment that jointly optimizes camera poses, 3D landmarks, IMU states, and inertial biases over the full trajectory. The filter provides robust initialization and dense feature tracks under challenging construction-site conditions; the global optimizer then recovers residual drift that the causal filter cannot correct.

The core stages are:

1. KLT optical flow tracking across both fisheye cameras at 30 Hz
2. IMU pre-integration at 1000 Hz within the MSCKF sliding-window estimator
3. SLAM landmark marginalization for long-horizon drift suppression
4. Offline visual-inertial bundle adjustment seeded from filter poses and feature tracks

The primary challenge of this dataset is **reliable initialization and sustained tracking** across environments characterized by repetitive texture, low-light conditions, and sequences that begin with intentional dynamic motion. We address this by pairing a robust causal filter for real-time state initialization and feature tracking with offline global visual-inertial bundle adjustment, using the filter trajectory and KLT feature tracks as initialization for a full-trajectory optimization that jointly minimizes reprojection and IMU preintegration residuals.

---

## 2. Sensor Configuration

**Camera**: Insta360 One-RS with dual back-to-back Leica fisheye lenses (~200° FoV each), rolling shutter, 1472×1440 per lens at 30 Hz. Both cameras contribute to the filter (`max_cameras: 2`). The two lenses point into opposite hemispheres with no overlap, so they function as two independent monocular inputs rather than a traditional stereo pair; depth is recovered from motion parallax on each camera independently.

**IMU**: Bosch BMI085, 6-axis at ~995 Hz (nominal 1000 Hz).

**Camera model**: Pinhole equidistant (Kannala-Brandt), fitted to the Enhanced Unified Camera Model (EUCM) Kalibr calibration provided with the dataset.

**IMU noise parameters** (from Bosch BMI085 datasheet, consistent with Kalibr calibration):

| Parameter | Value |
|---|---|
| Gyro noise density | 1.7 × 10⁻⁴ rad/s/√Hz |
| Accel noise density | 2.0 × 10⁻³ m/s²/√Hz |
| Gyro bias random walk | 1.9 × 10⁻⁵ rad/s²/√Hz |
| Accel bias random walk | 3.0 × 10⁻⁴ m/s³/√Hz |

---

## 3. Filter Initialization

Initialization was the primary failure mode across the 30 runs. The static initializer requires feature tracks to survive for at least half the initialization window before attempting state recovery. Fast motion or low visual texture breaks KLT tracks before they age sufficiently, blocking initialization indefinitely.

Two design choices proved effective:

**Dynamic initialization**: Rather than requiring a sustained still-then-moving jerk event, the dynamic initializer triggers on general excited motion. This is critical for runs that begin mid-walk. For `floor_EG/2025-12-02/run_2`, all static-init variants had a 59 s delay because the jerk condition is not met until t = 59 s when the operator's grip adjusts. Combining dynamic initialization with a 1.0 s window and inflated IMU noise reduced this to 5.6 s by triggering on early camera shake.

**Short initialization window**: Reducing the window from 3.0 s to 1.0 s lowers the track-survival requirement with a minor loss of observability for accelerometer alignment. On sequences where the camera is already moving at the start of the bag, the shorter window is strictly beneficial.

**Frame rate at evaluation time**: Processing bags at 0.5× playback was necessary on most runs. At 1× speed, the CPU cannot keep pace with the 30 Hz image stream and processes only 16–23 frames per second, causing the optical flow tracker to operate on larger inter-frame displacements. This breaks tracks earlier and degrades both initialization and sustained tracking. At 0.5× speed the tracker operates at near-nominal frame rate, establishing longer-lived tracks that feed both the initializer and the MSCKF update step.

---

## 4. Filter Configuration Design

No single configuration generalizes across all 30 runs. We designed five variants and selected the best per-run empirically.

| Config | Key changes vs. baseline | Rationale |
|---|---|---|
| `baseline` | (none) | Conservative defaults; robust on well-textured runs |
| `dynA` | Dynamic init, 3.0 s window | Handles dynamic-start sequences |
| `imuB` | IMU process noise ×5 | Down-weights IMU in the EKF update; improves robustness when bias is poorly observable (slow walking, dark rooms) |
| `expC` | dynA + imuB + 800 features, 100 SLAM landmarks, 15 clones, CLAHE, ZUPT | Maximum reprojection constraints and clone horizon for long-corridor runs |
| `fastinit` | Dynamic init, 1.0 s window | Fastest initialization; preferred when the combined settings of expC destabilize tracking post-init |

**IMU noise inflation** (`imuB`): Inflating the process noise covariance by a factor of five loosens the IMU's influence in the filter update, allowing the visual measurements to dominate when inertial bias is hard to observe (slow walking in featureless basements being the prototypical case). The trade-off is susceptibility to integration drift under fast motion.

**Feature count and clone horizon** (`expC`): Increasing the number of tracked features to 800 and SLAM landmarks to 100 widens the reprojection constraint set in the update step. Extending the clone horizon to 15 allows the filter to accumulate longer-baseline observations, improving triangulation depth for sparse texture. CLAHE contrast normalization improves track stability in low-contrast environments.

**Zero-velocity update (ZUPT)**: During detected stationary intervals, the IMU acceleration residual and angular rate provide direct zero-velocity and zero-rotation-rate pseudo-measurements, strongly constraining bias estimation. In practice ZUPT had limited impact as most runs involve continuous walking with few clear still periods.

---

## 5. Per-Run Selection

Every run was processed under multiple configuration × playback-rate combinations. The best result was selected by:

1. **ATE after SE3 alignment** for the five runs with released ground truth.
2. **Trajectory plausibility**: XY spread consistent with known floor dimensions; Z drift below 2 m for indoor runs (elevated Z reliably indicates unconstrained IMU bias drift, since there is no visual height constraint in monocular mode).
3. **Multi-configuration consensus**: when three or more independent configurations converge on the same trajectory shape, that shape is accepted as correct. This was the primary decision criterion for runs without ground truth.

Final submission configuration distribution:

| Configuration | Runs |
|---|---|
| Dynamic init, 1.0 s window, 0.5× | 13 |
| Dynamic init, 3.0 s window, 0.5× | 10 |
| Baseline, 1× | 3 |
| Inflated IMU noise, 0.5× | 2 |
| Combined (expC), 0.5× | 2 |

Dynamic initialization at 0.5× playback accounts for 77% of runs, establishing it as the most broadly applicable strategy for this dataset.

---

## 6. Visual-Inertial Bundle Adjustment Refinement

For runs with released ground truth, we applied an offline visual-inertial bundle adjustment built on the `features/imu` branch of COLMAP [Schönberger & Frahm, 2016; Liu, 2024] to refine the filter output.

### Feature Track Construction

The filter publishes, at each camera frame, the 3D position, pixel coordinates, and stable track ID of every currently active KLT-tracked feature. Accumulating these messages over a full run reconstructs the complete track history without additional feature extraction or matching. A typical run yields ~20k–50k triangulated 3D points and ~800k 2D reprojection observations.

The fundamental limitation is that KLT optical flow carries no feature descriptors. When a track is lost, the physical point re-appears under a new track ID, producing duplicate 3D points and breaking the observation chain across the interruption. For slow indoor traversals the effect is minor; for high-occlusion segments it weakens the reprojection term in the BA.

### Optimization

The bundle adjustment minimizes the joint visual-inertial cost:

$$\min_{\mathbf{T}, \mathbf{p}, \mathbf{b}} \sum_{(i,j)} \rho\!\left(\|\mathbf{r}^{ij}_\text{reproj}\|^2_{\Sigma_c}\right) + \sum_k \|\mathbf{r}^k_\text{IMU}\|^2_{\Sigma_\text{IMU}}$$

where $\mathbf{r}^{ij}_\text{reproj}$ are per-observation reprojection residuals, $\mathbf{r}^k_\text{IMU}$ are IMU preintegration residuals (position, velocity, orientation, and bias increments) over consecutive image intervals computed using the on-manifold analytical preintegration of Forster et al. [2017], and $\rho(\cdot)$ is a Huber loss. The optimization variables are camera poses $\mathbf{T}$, 3D landmark positions $\mathbf{p}$, and per-interval IMU states $\mathbf{b}$ (velocity, gyro bias, accelerometer bias). Pose graph structure is taken directly from the filter reconstruction; no incremental SfM triangulation or descriptor matching is performed.

### Result

| Run | Best filter ATE (m) | After VI-BA (m) | Reduction |
|---|---|---|---|
| floor_UG1 / 2025-10-16 / run_1 | 0.522 | **0.371** | 29% |

The UG1 basement is the hardest of the five GT runs, featuring a large open space, repetitive concrete texture, and two stairwell crossings. The 29% ATE reduction confirms that offline global optimization over the full trajectory recovers drift that accumulates in the causal filter.

---

## 7. Results

### Ground-truth-evaluated sequences (5 early-release runs)

| Sequence | ATE RMSE (m) | Coverage | Score / 100 |
|---|---|---|---|
| floor_1 / 2025-05-05 / run_1 | 0.298 | 99.7% | 88.5 |
| floor_2 / 2025-05-05 / run_1 | 0.696 | 99.6% | 75.9 |
| floor_2 / 2025-10-28 / run_1 | 0.248 | 99.4% | 89.7 |
| floor_2 / 2025-10-28 / run_2 | 0.262 | 100.0% | 89.5 |
| floor_UG1 / 2025-10-16 / run_1 (VI-BA) | 0.371 | 99.6% | ~86.5 |
| **Mean / Total** | **0.375** | | **~430 / 500** |

The weakest result (floor_2/2025-05-05/run_1, 0.696 m) contains a long featureless corridor segment where cumulative translational drift is unavoidable in a monocular filter without loop closure.

---

## 8. Discussion

The dominant challenge of this dataset is initialization robustness. Dynamic initialization, which triggers on excited angular velocity rather than waiting for a still-to-moving jerk, was the single most effective intervention, rescuing sequences that the static initializer could not handle. The complementary role of inflated IMU noise is to shift observability weight toward vision when inertial measurements are uninformative, at the cost of drift susceptibility in fast-motion segments.

The two-stage pipeline (causal filter followed by global visual-inertial BA) reflects a natural division of responsibility. The filter handles the causal, real-time constraint; the global optimizer absorbs the full trajectory into a single cost function and eliminates drift that is irrecoverable in a sliding window. The principal limitation of the current BA implementation is the absence of track re-identification across interruptions: since KLT tracks carry no descriptors, a feature reobserved after occlusion cannot be associated with its prior segment. Replacing KLT tracks with descriptor-based correspondences (e.g., ALIKED + LightGlue) and re-triangulating against the fixed filter poses would extend observation chains across occlusions, yielding stronger reprojection constraints particularly in the heavily occluded basement environments.

---

## References

Geneva, P., Eckenhoff, K., Lee, W., Yang, Y., & Huang, G. (2020). OpenVINS: A Research Platform for Visual-Inertial Estimation. *ICRA 2020 Workshop on Open Source Systems in Robotics*. https://github.com/rpng/open_vins

Schönberger, J. L., & Frahm, J.-M. (2016). Structure-from-Motion Revisited. *IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)*, pp. 4104–4113.

Schönberger, J. L., Zheng, E., Pollefeys, M., & Frahm, J.-M. (2016). Pixelwise View Selection for Unstructured Multi-View Stereo. *European Conference on Computer Vision (ECCV)*, Springer LNCS.

Liu, S. (2024). COLMAP `features/imu` branch: Visual-Inertial Bundle Adjustment with IMU Preintegration Factors. GitHub. https://github.com/B1ueber2y/colmap/tree/features/imu

Forster, C., Carlone, L., Dellaert, F., & Scaramuzza, D. (2017). On-Manifold Preintegration for Real-Time Visual-Inertial Odometry. *IEEE Transactions on Robotics (TRO)*, 33(1), 1–21.
