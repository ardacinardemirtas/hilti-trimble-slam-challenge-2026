#!/usr/bin/env python3
"""
Localization submission builder.

Mirrors the logic of static_transform_publisher.py from challenge_tools_ros:

  1. Convert SLAM IMU pose → cam0 pose using T_imu_cam0 = inv(T_cam0_imu)
  2. T_map_global = T_map_cam0 * inv(T_global_cam0)   (SE3, SLAM-global → map)
  3. Force pitch/roll to zero (gravity is aligned) → pure SE2 (yaw + translation)
  4. Apply SE2 to all IMU poses, then apply IMU→cam0 extrinsic to get
     cam0 poses in the floorplan map frame.

Output: localization_submission/<run_name>.txt  (flat, submission-ready)

Usage:
    python3 localize_from_5s.py --slam-dir /path/to/slam/trajectories/
    python3 localize_from_5s.py --slam-dir results/ --out-dir my_localization/
"""

import argparse
import csv
import math
import numpy as np
import yaml
from pathlib import Path


# ── Quaternion / matrix helpers ───────────────────────────────────────────────

def quat_to_mat(q):
    """[qx,qy,qz,qw] → 3×3 rotation matrix (Hamilton, world-from-body)."""
    qx, qy, qz, qw = q
    return np.array([
        [1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
    ])


def mat_to_quat(R):
    trace = R[0,0]+R[1,1]+R[2,2]
    if trace > 0:
        s = 0.5/np.sqrt(trace+1.0)
        return np.array([(R[2,1]-R[1,2])*s,(R[0,2]-R[2,0])*s,(R[1,0]-R[0,1])*s,0.25/s])
    elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
        s = 2.0*np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])
        return np.array([0.25*s,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s,(R[2,1]-R[1,2])/s])
    elif R[1,1]>R[2,2]:
        s = 2.0*np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])
        return np.array([(R[0,1]+R[1,0])/s,0.25*s,(R[1,2]+R[2,1])/s,(R[0,2]-R[2,0])/s])
    else:
        s = 2.0*np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])
        return np.array([(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,0.25*s,(R[1,0]-R[0,1])/s])


def pose_to_mat(pos, q):
    """(pos[3], q[4] xyzw) → 4×4 SE3 matrix."""
    T = np.eye(4)
    T[:3,:3] = quat_to_mat(q)
    T[:3,3]  = pos
    return T


def mat_to_pose(T):
    """4×4 SE3 → (pos[3], q[4] xyzw)."""
    R = T[:3,:3]
    q = mat_to_quat(R)
    q /= np.linalg.norm(q)
    return T[:3,3].copy(), q


def rot_to_euler_zyx(R):
    """3×3 → (roll, pitch, yaw) in ZYX convention."""
    pitch = math.atan2(-R[2,0], math.sqrt(R[2,1]**2 + R[2,2]**2))
    roll  = math.atan2(R[2,1]/math.cos(pitch), R[2,2]/math.cos(pitch))
    yaw   = math.atan2(R[1,0]/math.cos(pitch), R[0,0]/math.cos(pitch))
    return roll, pitch, yaw


def rz(yaw):
    """Pure yaw rotation matrix."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def load_tum(filepath):
    timestamps, positions, quaternions = [], [], []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'): continue
            p = line.split()
            if len(p) != 8: continue
            timestamps.append(float(p[0]))
            positions.append([float(x) for x in p[1:4]])
            quaternions.append([float(x) for x in p[4:8]])
    return np.array(timestamps), np.array(positions), np.array(quaternions)


# ── Load calibration T_cam0_imu ───────────────────────────────────────────────

ROOT = Path(__file__).parent
calib_path = ROOT / 'config' / 'hilti_openvins' / 'kalibr_imucam_chain.yaml'
with open(calib_path) as f:
    content = f.read()
# Strip OpenCV YAML header (%YAML:1.0) which is not valid standard YAML
content = '\n'.join(ln for ln in content.splitlines() if not ln.startswith('%YAML'))
calib = yaml.safe_load(content)

T_cam0_imu = np.array(calib['cam0']['T_cam_imu'], dtype=float)  # 4×4
T_imu_cam0 = np.linalg.inv(T_cam0_imu)                          # 4×4

# ── Load GT init poses ────────────────────────────────────────────────────────

ap = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--slam-dir', required=True,
                help='Directory containing SLAM TUM trajectory files (*.txt)')
ap.add_argument('--out-dir', default=None,
                help='Output directory (default: localization_submission/ next to this script)')
ap.add_argument('--init-gt', default=None,
                help='Path to init_gt_poses.csv (default: groundtruth/init_gt_poses.csv)')
args = ap.parse_args()

INIT_GT_CSV = Path(args.init_gt) if args.init_gt else ROOT / 'groundtruth' / 'init_gt_poses.csv'
SLAM_DIR    = Path(args.slam_dir)
OUT_DIR     = Path(args.out_dir) if args.out_dir else ROOT / 'localization_submission'
OUT_DIR.mkdir(exist_ok=True)

init_poses = {}
with open(INIT_GT_CSV) as f:
    reader = csv.DictReader(f)
    seq_key = next(k for k in reader.fieldnames if 'Sequence Name' in k)
    for row in reader:
        name = row[seq_key].strip()
        init_poses[name] = {
            'timestamp': float(row['timestamp']),
            'pos': np.array([float(row['tx']), float(row['ty']), float(row['tz'])]),
            'q':   np.array([float(row[x]) for x in ['qx','qy','qz','qw']]),
        }

# ── Process each run ──────────────────────────────────────────────────────────

processed = []
for traj_file in sorted(SLAM_DIR.glob('*.txt')):
    if any(tag in traj_file.stem for tag in
           ('_orig', '_prev', '_extrap', '_imu', '_1x', '_v2', '_expC')):
        continue
    run_name = traj_file.stem

    if run_name not in init_poses:
        print(f'[skip] {run_name} — not in init_gt_poses.csv')
        continue

    gt = init_poses[run_name]
    timestamps, positions, quaternions = load_tum(traj_file)

    # Find SLAM IMU pose closest to GT init timestamp
    idx = int(np.argmin(np.abs(timestamps - gt['timestamp'])))
    dt  = abs(timestamps[idx] - gt['timestamp'])
    if dt > 0.1:
        print(f'[warn] {run_name}: nearest pred pose is {dt:.4f}s from init GT')

    # SLAM IMU pose → cam0 pose (in SLAM global frame)
    T_global_imu_anchor = pose_to_mat(positions[idx], quaternions[idx])
    T_global_cam0_anchor = T_global_imu_anchor @ T_imu_cam0

    # GT cam0 pose in map frame
    T_map_cam0_anchor = pose_to_mat(gt['pos'], gt['q'])

    # T_map_global: maps SLAM global frame to map frame
    T_map_global = T_map_cam0_anchor @ np.linalg.inv(T_global_cam0_anchor)

    HYBRID_RUNS = {'floor_2_2025-12-02_run_1', 'floor_2_2025-12-03_run_1'}
    orig_file = SLAM_DIR / f'{run_name}_orig.txt'
    hybrid_note = ''
    if run_name in HYBRID_RUNS and orig_file.exists():
        orig_ts, orig_pos, orig_quats = load_tum(orig_file)
        if len(orig_ts):
            T_global_imu_real  = pose_to_mat(orig_pos[0], orig_quats[0])
            T_global_cam0_real = T_global_imu_real @ T_imu_cam0
            # Yaw: align real SLAM cam0 rotation with GT cam0 rotation at anchor
            R_for_yaw = T_map_cam0_anchor[:3,:3] @ T_global_cam0_real[:3,:3].T
            _, _, yaw = rot_to_euler_zyx(R_for_yaw)
            R_se2 = rz(yaw)
            # Translation: map the extrapolated t=5s SLAM cam0 position to the GT position
            p_slam_5s = T_global_cam0_anchor[:3, 3]
            p_gt_5s   = T_map_cam0_anchor[:3, 3]
            t_hybrid  = p_gt_5s - R_se2 @ p_slam_5s
            T_map_global_se2 = np.eye(4)
            T_map_global_se2[:3,:3] = R_se2
            T_map_global_se2[:3,3]  = t_hybrid
            hybrid_note = f'  [hybrid: rot from t={orig_ts[0]:.3f}s]'
        else:
            # Force pitch and roll to zero (gravity-aligned SLAM)
            _, _, yaw = rot_to_euler_zyx(T_map_global[:3,:3])
            R_se2 = rz(yaw)
            T_map_global_se2 = np.eye(4)
            T_map_global_se2[:3,:3] = R_se2
            T_map_global_se2[:3,3]  = T_map_global[:3,3]
    else:
        # Force pitch and roll to zero (gravity-aligned SLAM)
        _, _, yaw = rot_to_euler_zyx(T_map_global[:3,:3])
        R_se2 = rz(yaw)
        T_map_global_se2 = np.eye(4)
        T_map_global_se2[:3,:3] = R_se2
        T_map_global_se2[:3,3]  = T_map_global[:3,3]

    # Apply to all SLAM IMU poses + convert to cam0 frame
    pos_out  = np.zeros((len(timestamps), 3))
    quat_out = np.zeros((len(timestamps), 4))
    for i, (p, q) in enumerate(zip(positions, quaternions)):
        T_global_imu_i  = pose_to_mat(p, q)
        T_map_cam0_i    = T_map_global_se2 @ T_global_imu_i @ T_imu_cam0
        pos_out[i], q_i = mat_to_pose(T_map_cam0_i)
        quat_out[i]     = q_i

    out_file = OUT_DIR / f'{run_name}.txt'
    with open(out_file, 'w') as f:
        f.write('# timestamp tx ty tz qx qy qz qw  (localization: SE2 map-aligned cam0)\n')
        for ts, p, q in zip(timestamps, pos_out, quat_out):
            f.write(f'{ts:.9f} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} '
                    f'{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n')

    print(f'[ok] {run_name}: yaw={math.degrees(yaw):.1f}°  dt={dt:.4f}s{hybrid_note}')
    processed.append(run_name)

print(f'\nDone. {len(processed)} runs → {OUT_DIR}/')
