#!/usr/bin/env bash
#SBATCH --job-name=build_orbslam3_sif
#SBATCH --account=public
#SBATCH --partition=normal.4h
#SBATCH --time=1:00:00
#SBATCH --cpus-per-task=4
#SBATCH --mem-per-cpu=4G
#SBATCH --output=/cluster/scratch/%u/logs/orbslam3/build_sif_%j.out
#SBATCH --error=/cluster/scratch/%u/logs/orbslam3/build_sif_%j.err

set -euo pipefail

module load eth_proxy

SCRATCH=/cluster/scratch/$USER
mkdir -p "$SCRATCH/logs/orbslam3"

echo "Building orbslam3.sif on $(hostname) at $(date)"

apptainer build --fakeroot "$SCRATCH/orbslam3.sif" \
    "$SCRATCH/code/hilti-trimble-slam-challenge-2026/orbslam3.def"

echo "Done at $(date): $SCRATCH/orbslam3.sif"
ls -lh "$SCRATCH/orbslam3.sif"
