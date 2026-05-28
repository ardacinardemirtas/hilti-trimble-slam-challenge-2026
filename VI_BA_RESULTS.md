# VI-BA vs Baseline — GT Runs

Baseline = `openvins_submission/{run}.txt` (curated best OpenVINS config per run).  
VI-BA = `openvins_best/{run}_vi/refined.txt` (pycolmap BA + IMU preintegration).  
Local scores from `evaluate.py`. Server scores from submission v8.

---

## Results

| Run | Baseline Score | VI-BA Score | Δ Score | Baseline RMSE | VI-BA RMSE | Δ RMSE |
|-----|:--------------:|:-----------:|:-------:|:-------------:|:----------:|:------:|
| floor_1_2025-05-05_run_1   | 85.7  | 88.3  | +2.6 | 0.300 | 0.233 | −0.067 |
| floor_2_2025-05-05_run_1   | 81.4  | 83.3  | +1.9 | 0.446 | 0.390 | −0.056 |
| floor_2_2025-10-28_run_1   | 89.7  | 92.5  | +2.8 | 0.248 | 0.184 | −0.064 |
| floor_2_2025-10-28_run_2   | 89.5  | 91.9  | +2.4 | 0.262 | 0.192 | −0.070 |
| floor_UG1_2025-10-16_run_1 | 82.5  | 85.1  | +2.6 | 0.378 | 0.311 | −0.067 |
| **Average**                | **85.8** | **88.2** | **+2.5** | **0.327** | **0.262** | **−0.065** |

---

## Notes

- All VI-BA scores are server results. Baseline scores for floor_1, floor_2_2025-05-05, and floor_UG1 derived from local evaluation.
- Consistent **+2.5 pt / −0.065 m RMSE** improvement across all 5 runs (~20% RMSE reduction).
- floor_2_2025-10-28 runs see the largest relative gain (~27% RMSE reduction) — shorter sequences benefit more from IMU regularization.
- floor_2_2025-05-05 shows the smallest score gain (+1.9) — longer sequence with more accumulated drift.
