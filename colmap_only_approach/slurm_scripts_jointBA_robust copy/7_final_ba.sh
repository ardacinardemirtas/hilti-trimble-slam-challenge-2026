#!/bin/bash
#SBATCH --job-name=run2_ba_floor_2_2025_05_05_run_1
#SBATCH --cpus-per-task=12
#SBATCH --mem-per-cpu=4G
#SBATCH --time=24:00:00

set -euo pipefail
source ~/colmap_dependencies.sh
cd /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust
mkdir -p /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/sparse/0_localized_refined

# Joint BA: poses + 3D points + fisheye distortion (k1-k4)
# Focal, principal point, and rig extrinsics remain FROZEN.
colmap bundle_adjuster \
    --input_path  /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/sparse/0_localized_pass2 \
    --output_path /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/sparse/0_localized_refined \
    --BundleAdjustment.refine_focal_length 0 \
    --BundleAdjustment.refine_principal_point 0 \
    --BundleAdjustment.refine_extra_params 1 \
    --BundleAdjustment.refine_sensor_from_rig 0 \
    --BundleAdjustment.refine_points3D 1 \
    --BundleAdjustmentCeres.max_num_iterations 200
