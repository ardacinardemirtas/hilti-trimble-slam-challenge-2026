#!/bin/bash
#SBATCH --job-name=jointBA10_floor_1_2025_05_05_run_1_map
#SBATCH --cpus-per-task=12
#SBATCH --mem-per-cpu=2G
#SBATCH --time=24:00:00

set -euo pipefail
source ~/colmap_dependencies.sh
cd /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_gt_runs/floor_1/2025-05-05_run_1/cfg_10hz_jointBA
mkdir -p /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_gt_runs/floor_1/2025-05-05_run_1/cfg_10hz_jointBA/sparse

colmap mapper \
    --database_path /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_gt_runs/floor_1/2025-05-05_run_1/cfg_10hz_jointBA/database.db \
    --image_path    /cluster/scratch/bsekeroglu/dataset/colmap_dataset/data_30Hz/floor_1/2025-05-05_run_1/images \
    --output_path   /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_gt_runs/floor_1/2025-05-05_run_1/cfg_10hz_jointBA/sparse \
    --Mapper.image_list_path /cluster/scratch/bsekeroglu/dataset/colmap_dataset/data_30Hz/floor_1/2025-05-05_run_1/10hz_frames.txt \
    --Mapper.num_threads $SLURM_CPUS_PER_TASK \
    --Mapper.ba_refine_sensor_from_rig 0 \
    --Mapper.ba_refine_focal_length 0 \
    --Mapper.ba_refine_principal_point 0 \
    --Mapper.ba_refine_extra_params 0 \
    --Mapper.ba_use_gpu 0 \
    --Mapper.multiple_models 0 \
    --Mapper.ba_global_frames_freq 200
