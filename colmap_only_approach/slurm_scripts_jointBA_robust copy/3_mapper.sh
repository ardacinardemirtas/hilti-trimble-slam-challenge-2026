#!/bin/bash
#SBATCH --job-name=run2_map_floor_2_2025_05_05_run_1
#SBATCH --cpus-per-task=12
#SBATCH --mem-per-cpu=2G
#SBATCH --time=24:00:00

set -euo pipefail
source ~/colmap_dependencies.sh
cd /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust
mkdir -p /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/sparse

# 10Hz mapping with SAFE threshold relaxation:
#   abs_pose_min_num_inliers=15 (critical for mapper growth on hard sequences)
#   init_min_tri_angle=16.0 (DEFAULT — avoids degenerate pair risk)
colmap mapper \
    --database_path /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/database.db \
    --image_path    /cluster/scratch/bsekeroglu/dataset/colmap_dataset/data_30Hz/floor_2/2025-05-05_run_1/images \
    --output_path   /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/sparse \
    --Mapper.image_list_path /cluster/scratch/bsekeroglu/dataset/colmap_dataset/data_30Hz/floor_2/2025-05-05_run_1/10hz_frames.txt \
    --Mapper.num_threads $SLURM_CPUS_PER_TASK \
    --Mapper.ba_refine_sensor_from_rig 0 \
    --Mapper.ba_refine_focal_length 0 \
    --Mapper.ba_refine_principal_point 0 \
    --Mapper.ba_refine_extra_params 0 \
    --Mapper.ba_use_gpu 0 \
    --Mapper.multiple_models 0 \
    --Mapper.ba_global_frames_freq 200 \
    --Mapper.abs_pose_min_num_inliers 15 \
    --Mapper.abs_pose_min_inlier_ratio 0.10 \
    --Mapper.init_min_num_inliers 50
