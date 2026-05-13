#!/usr/bin/env bash
# Build ORB_SLAM3 from source inside the orbslam3.sif Apptainer container.
#
# Usage (on Euler, from any node with $SCRATCH set):
#
#   apptainer exec --cleanenv \
#       --home "$HOME" \
#       --bind "$SCRATCH:$SCRATCH" \
#       $SCRATCH/orbslam3.sif \
#       bash $SCRATCH/code/hilti-trimble-slam-challenge-2026/build_orbslam3.sh
#
# Run once. Subsequent SLURM jobs can reuse the compiled binaries in-place.

set -euo pipefail

ORBSLAM3_DIR="${ORBSLAM3_DIR:-/cluster/scratch/${USER}/code/ORB_SLAM3}"
NPROC="${NPROC:-$(nproc)}"

echo "=== Building ORB_SLAM3 at $ORBSLAM3_DIR with $NPROC cores ==="

cd "$ORBSLAM3_DIR"

# ── 1. Thirdparty: DBoW2 ────────────────────────────────────────────────────
echo "--- DBoW2 ---"
cmake -S Thirdparty/DBoW2 -B Thirdparty/DBoW2/build \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build Thirdparty/DBoW2/build --parallel "$NPROC"

# ── 2. Thirdparty: g2o ──────────────────────────────────────────────────────
echo "--- g2o ---"
cmake -S Thirdparty/g2o -B Thirdparty/g2o/build \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build Thirdparty/g2o/build --parallel "$NPROC"

# ── 3. Thirdparty: Sophus ───────────────────────────────────────────────────
echo "--- Sophus ---"
cmake -S Thirdparty/Sophus -B Thirdparty/Sophus/build \
    -DCMAKE_BUILD_TYPE=Release
cmake --build Thirdparty/Sophus/build --parallel "$NPROC"

# ── 4. ORB vocabulary ───────────────────────────────────────────────────────
if [ ! -f Vocabulary/ORBvoc.txt ]; then
    echo "--- Extracting vocabulary ---"
    cd Vocabulary && tar -xf ORBvoc.txt.tar.gz && cd ..
fi

# ── 5. ORB_SLAM3 ────────────────────────────────────────────────────────────
echo "--- ORB_SLAM3 ---"
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel "$NPROC"

echo "=== Build complete ==="
echo "Binary: $ORBSLAM3_DIR/Examples/Stereo-Inertial/stereo_inertial_euroc"
