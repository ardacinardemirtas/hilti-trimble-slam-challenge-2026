#!/bin/bash
#SBATCH --job-name=run2_match_floor_2_2025_05_05_run_1
#SBATCH --cpus-per-task=12
#SBATCH --mem-per-cpu=2G
#SBATCH --time=24:00:00

set -euo pipefail
source ~/colmap_dependencies.sh
cd /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust

colmap sequential_matcher \
    --database_path /cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/database.db \
    --FeatureMatching.use_gpu 0 \
    --FeatureMatching.rig_verification 0 \
    --SequentialMatching.overlap 50 \
    --SequentialMatching.quadratic_overlap 1 \
    --SequentialMatching.loop_detection 1 \
    --SequentialMatching.loop_detection_num_images 100 \
    --SequentialMatching.vocab_tree_path /cluster/scratch/bsekeroglu/dataset/colmap_dataset/vocab_tree_faiss_flickr100K_words256K.bin \
    --SequentialMatching.num_threads $SLURM_CPUS_PER_TASK \
    --FeatureMatching.num_threads $SLURM_CPUS_PER_TASK
