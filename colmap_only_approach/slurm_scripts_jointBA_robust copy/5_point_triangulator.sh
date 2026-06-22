#!/bin/bash
#SBATCH --job-name=run2_ptri_floor_2_2025_05_05_run_1
#SBATCH --cpus-per-task=12
#SBATCH --mem-per-cpu=2G
#SBATCH --time=24:00:00

set -euo pipefail
source ~/colmap_dependencies.sh
cd /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust
mkdir -p /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/sparse/0_localized_triangulated

colmap point_triangulator \
    --database_path /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/database.db \
    --image_path    /cluster/scratch/bsekeroglu/dataset/colmap_dataset/data_30Hz/floor_2/2025-05-05_run_1/images \
    --input_path    /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/sparse/0_localized \
    --output_path   /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/sparse/0_localized_triangulated \
    --clear_points 0 \
    --refine_intrinsics 0 \
    --Mapper.ba_refine_sensor_from_rig 0 \
    --Mapper.num_threads $SLURM_CPUS_PER_TASK \
    --Mapper.ba_use_gpu 0
