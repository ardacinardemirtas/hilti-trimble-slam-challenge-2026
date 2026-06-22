#!/bin/bash
#SBATCH --job-name=jointBA10_floor_1_2025_05_05_run_1_ba
#SBATCH --cpus-per-task=12
#SBATCH --mem-per-cpu=4G
#SBATCH --time=24:00:00

set -euo pipefail
source ~/colmap_dependencies.sh
cd /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_gt_runs/floor_1/2025-05-05_run_1/cfg_10hz_jointBA
mkdir -p /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_gt_runs/floor_1/2025-05-05_run_1/cfg_10hz_jointBA/sparse/0_localized_refined

# Joint bundle adjustment: poses + 3D points refined together.
# Intrinsics and rig extrinsics held fixed (factory calibration).
colmap bundle_adjuster \
    --input_path  /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_gt_runs/floor_1/2025-05-05_run_1/cfg_10hz_jointBA/sparse/0_localized_pass2 \
    --output_path /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_gt_runs/floor_1/2025-05-05_run_1/cfg_10hz_jointBA/sparse/0_localized_refined \
    --BundleAdjustment.refine_focal_length 0 \
    --BundleAdjustment.refine_principal_point 0 \
    --BundleAdjustment.refine_extra_params 0 \
    --BundleAdjustment.refine_sensor_from_rig 0 \
    --BundleAdjustment.refine_points3D 1 \
    --BundleAdjustmentCeres.max_num_iterations 200
