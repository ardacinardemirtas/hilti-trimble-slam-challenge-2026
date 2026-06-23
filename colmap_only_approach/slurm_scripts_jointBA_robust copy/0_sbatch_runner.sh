#!/bin/bash
set -euo pipefail
JOB_FE=${1:-}
if [ -z "$JOB_FE" ]; then echo "Usage: $0 <FE_JOB_ID>"; exit 1; fi
SD=/cluster/scratch/bsekeroglu/dataset/colmap_dataset/colmap_full_run2/floor_2/2025-05-05_run_1/cfg_10hz_jointBA_robust/slurm_scripts
TIMESTAMP=$(date "+%Y-%m-%d %H:%M:%S")

echo "=== cfg_10hz_jointBA_robust: floor_2/2025-05-05_run_1 ==="
echo "Depends on FE job: $JOB_FE"

JOB2=$(sbatch --parsable --dependency=afterok:$JOB_FE \
    --output="$SD/sequential_matcher_%j.out" --error="$SD/sequential_matcher_%j.err" \
    "$SD/2_sequential_matcher.sh")
JOB3=$(sbatch --parsable --dependency=afterok:$JOB2 \
    --output="$SD/mapper_%j.out" --error="$SD/mapper_%j.err" \
    "$SD/3_mapper.sh")
JOB4=$(sbatch --parsable --dependency=afterok:$JOB3 \
    --output="$SD/image_registrator_p1_%j.out" --error="$SD/image_registrator_p1_%j.err" \
    "$SD/4_image_registrator.sh")
JOB5=$(sbatch --parsable --dependency=afterok:$JOB4 \
    --output="$SD/point_triangulator_%j.out" --error="$SD/point_triangulator_%j.err" \
    "$SD/5_point_triangulator.sh")
JOB6=$(sbatch --parsable --dependency=afterok:$JOB5 \
    --output="$SD/image_registrator_p2_%j.out" --error="$SD/image_registrator_p2_%j.err" \
    "$SD/6_image_registrator_pass2.sh")
JOB7=$(sbatch --parsable --dependency=afterok:$JOB6 \
    --output="$SD/final_ba_%j.out" --error="$SD/final_ba_%j.err" \
    "$SD/7_final_ba.sh")
JOB8=$(sbatch --parsable --dependency=afterok:$JOB7 \
    --output="$SD/to_txt_%j.out" --error="$SD/to_txt_%j.err" \
    "$SD/8_to_txt.sh")

echo "Chain: $JOB_FE -> $JOB2 -> $JOB3 -> $JOB4 -> $JOB5 -> $JOB6 -> $JOB7 -> $JOB8"

{
  echo "========================================"
  echo "Submitted: $TIMESTAMP"
  echo "FE $JOB_FE  Match $JOB2  Map $JOB3  Reg1 $JOB4  Tri $JOB5  Reg2 $JOB6  BA $JOB7  TXT $JOB8"
  echo ""
} >> "$SD/0_sbatch_runner.txt"
