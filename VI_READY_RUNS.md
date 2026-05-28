# VI-Ready Runs (openvins_best, 2026-05-14)

Trajectory + feature files produced by `run_openvins_best_slowrate.sbatch` (best
config per run, 0.5× playback). All files under:
`/cluster/scratch/ademirtas/results/openvins_best/`

Each run has `{run}.txt` (TUM) and `{run}_feats.txt` (feature tracks).
Feed into `features_to_colmap.py` → `bag_to_imu_npy.py` → `run_vi_hilti.py`.

---

## Ready Runs

| Run | Poses | Config |
|-----|-------|--------|
| floor_1_2025-07-07_run_1 | 4002 | dynA 0.5× |
| floor_1_2025-12-02_run_1 | 8148 | dynA 0.5× |
| floor_2_2025-05-05_run_1 | 5561 | baseline 0.5× |
| floor_2_2025-10-28_run_1 | 3045 | dynA 0.5× |
| floor_2_2025-10-28_run_2 | 2567 | dynA 0.5× |
| floor_2_2025-12-02_run_1 | 4216 | dynA 0.5× |
| floor_2_2025-12-03_run_1 | 4286 | fastinit 0.5× |
| floor_3_2025-05-19_run_1 | 3439 | dynA 0.5× |
| floor_3_2025-12-02_run_1 | 3855 | fastinit 0.5× |
| floor_4_2025-05-19_run_1 | 2651 | dynA 0.5× |
| floor_4_2025-12-02_run_1 | 5204 | fastinit 0.5× (different shape from submission dynA 1×) |
| floor_5_2025-12-02_run_1 | 4677 | fastinit 0.5× |
| floor_6_2025-06-18_run_1 | 2140 | imuB 0.5× |
| floor_6_2025-07-07_run_1 | 2176 | fastinit 0.5× |
| floor_6_2025-12-02_run_1 | 5078 | fastinit 0.5× |
| floor_6_2025-12-02_run_2 | 3594 | fastinit 0.5× |
| floor_7_2025-12-02_run_1 | 3338 | fastinit 0.5× |
| floor_7_2025-12-02_run_2 | 4606 | dynA 0.5× |
| floor_7_2025-12-03_run_1 | 5842 | dynA 0.5× |
| floor_EG_2025-12-02_run_1 | 3666 | fastinit 0.5× |
| floor_UG1_2025-12-02_run_2 | 6202 | fastinit 0.5× |
| floor_UG1_2025-12-03_run_1 | 3923 | fastinit 0.5× |
| floor_UG2_2025-12-02_run_1 | 6580 | fastinit 0.5× |

---

## Skipped (diverged or failed to init in all attempts)

| Run | Fallback |
|-----|---------|
| floor_1_2025-05-05_run_1 | use existing `openvins_vi_ready/` result |
| floor_EG_2025-10-16_run_1 | — |
| floor_EG_2025-12-02_run_2 | — |
| floor_UG1_2025-05-19_run_1 | — |
| floor_UG1_2025-06-18_run_1 | — |
| floor_UG1_2025-10-16_run_1 | — |
| floor_UG1_2025-12-02_run_1 | — |
