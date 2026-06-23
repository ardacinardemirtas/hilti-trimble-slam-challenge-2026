#!/bin/bash
#SBATCH --job-name=run2_txt_floor_2_2025_05_05_run_1
#SBATCH --cpus-per-task=1
#SBATCH --mem-per-cpu=4G
#SBATCH --time=01:00:00

set -euo pipefail
source ~/colmap_dependencies.sh
cd /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust
mkdir -p /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/sparse_txt/0_localized_refined

colmap model_converter \
    --input_path  /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/sparse/0_localized_refined \
    --output_path /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/sparse_txt/0_localized_refined \
    --output_type TXT
