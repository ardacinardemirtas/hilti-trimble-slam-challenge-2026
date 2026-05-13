#!/usr/bin/env bash
#SBATCH --job-name=orb_%j
#SBATCH --account=public
#SBATCH --partition=normal.4h
#SBATCH --time=2:00:00
#SBATCH --cpus-per-task=8
#SBATCH --mem-per-cpu=4G
#SBATCH --output=/cluster/scratch/%u/logs/orbslam3/%x_%j.out
#SBATCH --error=/cluster/scratch/%u/logs/orbslam3/%x_%j.err

# Required args (pass via --export):
#   BAG_DIR   — path to the ROS2 bag directory (contains .db3 + metadata.yaml)
#   RUN_NAME  — e.g. floor_1_2025-05-05_run_1
#   OUT_FILE  — output TUM trajectory path (e.g. $SCRATCH/results/orbslam3/floor_1/2025-05-05_run_1.txt)

set -euo pipefail

SCRATCH=/cluster/scratch/$USER
SIF=$SCRATCH/orbslam3.sif
ORBSLAM3=$SCRATCH/code/ORB_SLAM3
REPO=$SCRATCH/code/hilti-trimble-slam-challenge-2026

EUROC_DIR=$SCRATCH/datasets/euroc_hilti/$RUN_NAME
TRAJ_RAW=$SCRATCH/results/orbslam3_raw/f_${RUN_NAME}.txt

mkdir -p "$SCRATCH/logs/orbslam3"
mkdir -p "$(dirname "$OUT_FILE")"
mkdir -p "$(dirname "$TRAJ_RAW")"
mkdir -p "$EUROC_DIR"

# ── Step 1: Convert ROS2 bag → EuRoC (needs ROS2 container) ─────────────────
# Reuse the existing ros2_jazzy.sif for bag extraction (has rosbag2_py + cv2).
ROS2_SIF=$SCRATCH/ros2_jazzy.sif
ROS2_WS=$SCRATCH/ros2_ws

# Fix bag metadata once (idempotent)
python3 - <<'PYEOF'
import glob, re, os
for f in glob.glob(os.environ["BAG_DIR"] + "/**/*.yaml", recursive=True):
    txt = open(f).read()
    new = re.sub(r'offered_qos_profiles:\s*\n\s*\[\]', 'offered_qos_profiles: []', txt)
    new = new.replace('offered_qos_profiles: ""', 'offered_qos_profiles: []')
    if new != txt:
        open(f, 'w').write(new)
        print(f"Fixed: {f}")
PYEOF

echo "Converting bag to EuRoC: $BAG_DIR → $EUROC_DIR"

apptainer exec --cleanenv \
    --home "$HOME" \
    --bind "$SCRATCH:$SCRATCH" \
    --bind "$ROS2_WS:/ros2_ws" \
    "$ROS2_SIF" \
    bash -c "
        source /opt/ros/jazzy/setup.bash
        source /ros2_ws/install/setup.bash
        python3 '$REPO/challenge_tools_ros/bag_helper/ros2bag_to_euroc.py' \
            --bag '$BAG_DIR' \
            --out '$EUROC_DIR' \
            --cam-topics /cam0/image_raw/compressed /cam1/image_raw/compressed \
            --imu-topic /imu/data_raw \
            --image-encoding mono8
    "

# Generate timestamps file for ORB_SLAM3
python3 "$REPO/challenge_tools_ros/bag_helper/euroc_make_timestamps.py" "$EUROC_DIR"
TIMESTAMPS=$EUROC_DIR/mav0/cam0/timestamps.txt

echo "EuRoC conversion done. Running ORB_SLAM3..."

# ── Step 2: Run ORB_SLAM3 stereo-inertial ────────────────────────────────────
apptainer exec --cleanenv \
    --home "$HOME" \
    --bind "$SCRATCH:$SCRATCH" \
    "$SIF" \
    bash -c "
        set -e
        cd '$SCRATCH/results/orbslam3_raw'
        '$ORBSLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc' \
            '$ORBSLAM3/Vocabulary/ORBvoc.txt' \
            '$REPO/config/hilti_orbslam3/hilti_orbslam3.yaml' \
            '$EUROC_DIR' \
            '$TIMESTAMPS' \
            '${RUN_NAME}'
    "

# ORB_SLAM3 writes: <RUN_NAME>_CameraTrajectory.txt and <RUN_NAME>_KeyFrameTrajectory.txt
# stereo_inertial_euroc writes f_<name>.txt when a name arg is provided
RAW_TRAJ=$SCRATCH/results/orbslam3_raw/f_${RUN_NAME}.txt

if [ ! -f "$RAW_TRAJ" ]; then
    echo "ERROR: ORB_SLAM3 did not produce $RAW_TRAJ" >&2
    exit 1
fi

# ── Step 3: Convert EuRoC output (ns timestamps) → TUM format (s timestamps) ─
python3 - <<PYEOF
import sys

raw = "$RAW_TRAJ"
out = "$OUT_FILE"

with open(raw) as fin, open(out, "w") as fout:
    for line in fin:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        ts_s = float(parts[0]) / 1e9
        fout.write(f"{ts_s:.9f} {' '.join(parts[1:])}\n")

print(f"Converted {raw} → {out}")
PYEOF

echo "=== Done: $OUT_FILE ==="
wc -l "$OUT_FILE" || true
