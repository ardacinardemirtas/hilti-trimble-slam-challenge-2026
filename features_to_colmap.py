#!/usr/bin/env python3
"""
Convert OpenVINS feature log → COLMAP text reconstruction with 3D points and
2D observations, for use with colmap_imu's vi_optimization.py.

Inputs:
  --features   feature_logger output (feat_id ts u v X Y Z per line)
  --tum        OpenVINS TUM trajectory (timestamp tx ty tz qx qy qz qw)
  --calib      kalibr_imucam_chain.yaml
  --output     output directory (cameras.txt, images.txt, points3D.txt,
                                 image_timestamps.npy)

How observations are matched to images:
  Each row in the features file has a timestamp_s.  We match it to the
  nearest TUM pose within `--ts_tol` seconds (default 0.02 s = half a 30 Hz
  frame).  Poses with no observations are still included in images.txt.

The resulting reconstruction has:
  - Full cam0_from_world poses for every TUM pose
  - 3D points with their world positions (taken as the median of all
    reported positions across observations; OpenVINS updates landmark
    positions as the filter runs, so later observations are more accurate)
  - Per-image POINTS2D observation lists linking images to 3D points

Usage:
  python3 features_to_colmap.py \\
      --features run_features.txt \\
      --tum      run.txt \\
      --calib    config/hilti_openvins/kalibr_imucam_chain.yaml \\
      --output   colmap_with_tracks/
"""

import argparse
import math
import os
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml


# ---------------------------------------------------------------------------
# Re-use pose helpers from openvins_to_colmap (copied inline for standalone use)
# ---------------------------------------------------------------------------

def quat_to_rot(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def rot_to_quat(R):
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0); w = 0.25/s
        x = (R[2,1]-R[1,2])*s; y = (R[0,2]-R[2,0])*s; z = (R[1,0]-R[0,1])*s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2*math.sqrt(1+R[0,0]-R[1,1]-R[2,2]); w = (R[2,1]-R[1,2])/s
        x = 0.25*s; y = (R[0,1]+R[1,0])/s; z = (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = 2*math.sqrt(1+R[1,1]-R[0,0]-R[2,2]); w = (R[0,2]-R[2,0])/s
        x = (R[0,1]+R[1,0])/s; y = 0.25*s; z = (R[1,2]+R[2,1])/s
    else:
        s = 2*math.sqrt(1+R[2,2]-R[0,0]-R[1,1]); w = (R[1,0]-R[0,1])/s
        x = (R[0,2]+R[2,0])/s; y = (R[1,2]+R[2,1])/s; z = 0.25*s
    q = np.array([x, y, z, w]); return q / np.linalg.norm(q)


def imu_pose_to_cam_from_world(t_imu_in_world, q_world_from_imu, T_cam_imu):
    R_cam_imu = T_cam_imu[:3, :3]; t_cam_imu = T_cam_imu[:3, 3]
    R_w_imu   = quat_to_rot(q_world_from_imu)
    R_imu_w   = R_w_imu.T
    t_imu_w   = -R_imu_w @ t_imu_in_world
    R_cam_w   = R_cam_imu @ R_imu_w
    t_cam_w   = R_cam_imu @ t_imu_w + t_cam_imu
    return rot_to_quat(R_cam_w), t_cam_w


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------

def load_tum(path):
    poses = []
    for line in open(path):
        if line.startswith('#') or not line.strip(): continue
        p = line.split()
        if len(p) == 8:
            poses.append(tuple(float(x) for x in p))
    return poses  # (ts, tx, ty, tz, qx, qy, qz, qw)


def load_kalibr(yaml_path, cam='cam0'):
    content = '\n'.join(l for l in open(yaml_path) if not l.startswith('%'))
    d = yaml.safe_load(content)[cam]
    return np.array(d['T_cam_imu']), d['intrinsics'], d['distortion_coeffs'], d['resolution']


def load_features(path):
    """
    Returns dict: feat_id → list of (timestamp_s, u, v, X, Y, Z)
    """
    tracks = defaultdict(list)
    for line in open(path):
        if line.startswith('#') or not line.strip(): continue
        parts = line.split()
        if len(parts) == 7:
            fid = int(parts[0])
            ts  = float(parts[1])
            u, v = float(parts[2]), float(parts[3])
            X, Y, Z = float(parts[4]), float(parts[5]), float(parts[6])
            tracks[fid].append((ts, u, v, X, Y, Z))
    return tracks


# ---------------------------------------------------------------------------
# Writers
# ---------------------------------------------------------------------------

def write_cameras_txt(path, intr, dist, res):
    fx, fy, cx, cy = intr; k1, k2, k3, k4 = dist; w, h = res
    with open(path, 'w') as f:
        f.write('# Camera list\n#   CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n')
        f.write(f'1 OPENCV_FISHEYE {w} {h} {fx} {fy} {cx} {cy} {k1} {k2} {k3} {k4}\n')


def write_images_txt(path, image_rows):
    """
    image_rows: list of (image_id, qw, qx, qy, qz, tx, ty, tz, name,
                          points2d)  where points2d = list of (x, y, point3d_id)
    """
    with open(path, 'w') as f:
        f.write('# Image list\n#   IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n')
        f.write('#   POINTS2D[] as (X Y POINT3D_ID)\n')
        for iid, qw, qx, qy, qz, tx, ty, tz, name, pts2d in image_rows:
            f.write(f'{iid} {qw:.9f} {qx:.9f} {qy:.9f} {qz:.9f} '
                    f'{tx:.9f} {ty:.9f} {tz:.9f} 1 {name}\n')
            if pts2d:
                obs_str = '  '.join(f'{u:.2f} {v:.2f} {pid}' for u, v, pid in pts2d)
                f.write(obs_str + '\n')
            else:
                f.write('\n')


def write_points3d_txt(path, points3d):
    """
    points3d: list of (point3d_id, X, Y, Z, track)
    track: list of (image_id, point2d_idx)
    """
    with open(path, 'w') as f:
        f.write('# 3D point list\n#   POINT3D_ID X Y Z R G B ERROR TRACK[]\n')
        for pid, X, Y, Z, track in points3d:
            track_str = '  '.join(f'{iid} {p2idx}' for iid, p2idx in track)
            f.write(f'{pid} {X:.9f} {Y:.9f} {Z:.9f} 128 128 128 1.0 {track_str}\n')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--features', required=True)
    parser.add_argument('--tum',      required=True)
    parser.add_argument('--calib',    required=True)
    parser.add_argument('--output',   required=True)
    parser.add_argument('--cam',      default='cam0')
    parser.add_argument('--ts_tol',   type=float, default=0.02,
                        help='Max timestamp difference to match obs→pose (s)')
    parser.add_argument('--min_track_len', type=int, default=3,
                        help='Discard tracks shorter than this many observations')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    T_cam_imu, intr, dist, res = load_kalibr(args.calib, args.cam)
    poses_raw = load_tum(args.tum)
    tracks = load_features(args.features)

    print(f'Loaded {len(poses_raw)} poses, {len(tracks)} feature tracks')

    # Build timestamp → image_id map (TUM timestamps as float)
    pose_ts = np.array([p[0] for p in poses_raw])

    # Assign COLMAP image IDs
    image_id_of_ts = {}
    for image_id, p in enumerate(poses_raw, start=1):
        image_id_of_ts[p[0]] = image_id

    # For each feature observation, find the nearest TUM pose
    # Build: image_id → list of (u, v, feat_id, point3d_id)
    # (point3d_id assigned below after filtering)
    feat_to_point3d = {}      # feat_id → point3d_id (1-indexed)
    feat_3d_pos     = {}      # feat_id → median (X, Y, Z)
    # image_obs[image_id] = list of (u, v, feat_id)
    image_obs = defaultdict(list)

    point3d_id = 0
    for feat_id, obs_list in tracks.items():
        if len(obs_list) < args.min_track_len:
            continue

        # Match each observation to nearest pose
        matched = []
        for ts, u, v, X, Y, Z in obs_list:
            idx = np.argmin(np.abs(pose_ts - ts))
            if np.abs(pose_ts[idx] - ts) > args.ts_tol:
                continue
            iid = image_id_of_ts[poses_raw[idx][0]]
            matched.append((iid, u, v, X, Y, Z))

        if len(matched) < args.min_track_len:
            continue

        point3d_id += 1
        feat_to_point3d[feat_id] = point3d_id

        # Use median of reported 3D positions (later obs more accurate in EKF)
        Xs = np.array([m[3] for m in matched])
        Ys = np.array([m[4] for m in matched])
        Zs = np.array([m[5] for m in matched])
        feat_3d_pos[feat_id] = (np.median(Xs), np.median(Ys), np.median(Zs))

        for iid, u, v, X, Y, Z in matched:
            image_obs[iid].append((u, v, feat_id))

    print(f'After filtering (min_track={args.min_track_len}): '
          f'{point3d_id} 3D points')

    # Build image rows
    image_rows = []
    image_timestamps = {}
    for image_id, p in enumerate(poses_raw, start=1):
        ts, tx, ty, tz, qx, qy, qz, qw = p
        q_cfw, t_cfw = imu_pose_to_cam_from_world(
            np.array([tx, ty, tz]),
            np.array([qx, qy, qz, qw]),
            T_cam_imu,
        )
        qw_c, qx_c, qy_c, qz_c = q_cfw[3], q_cfw[0], q_cfw[1], q_cfw[2]

        ts_ns = int(round(ts * 1e9))
        name = f'frame_{ts_ns:019d}.png'
        image_timestamps[image_id] = ts_ns

        # POINTS2D: list of (u, v, point3d_id), with p2d index for track
        obs = image_obs.get(image_id, [])
        pts2d = []
        for u, v, fid in obs:
            pid = feat_to_point3d.get(fid, -1)
            pts2d.append((u, v, pid))

        image_rows.append((image_id, qw_c, qx_c, qy_c, qz_c,
                           t_cfw[0], t_cfw[1], t_cfw[2], name, pts2d))

    # Build reverse index: point3d_id → list of (image_id, point2d_idx)
    point3d_track: dict[int, list] = defaultdict(list)
    for irow in image_rows:
        iid = irow[0]; pts2d = irow[9]
        for idx, (u, v, pid) in enumerate(pts2d):
            if pid != -1:
                point3d_track[pid].append((iid, idx))

    points3d_out = []
    for feat_id, pid in feat_to_point3d.items():
        X, Y, Z = feat_3d_pos[feat_id]
        track = point3d_track.get(pid, [])
        if track:
            points3d_out.append((pid, X, Y, Z, track))

    # Write outputs
    write_cameras_txt(os.path.join(args.output, 'cameras.txt'), intr, dist, res)
    write_images_txt(os.path.join(args.output, 'images.txt'), image_rows)
    write_points3d_txt(os.path.join(args.output, 'points3D.txt'), points3d_out)
    np.save(os.path.join(args.output, 'image_timestamps.npy'), image_timestamps)

    print(f'Written to {args.output}:')
    print(f'  cameras.txt')
    print(f'  images.txt     ({len(image_rows)} images, {sum(len(r[9]) for r in image_rows)} observations)')
    print(f'  points3D.txt   ({len(points3d_out)} points)')
    print(f'  image_timestamps.npy')
    print()
    print('This reconstruction has poses + 3D points + observations.')
    print('You can feed it directly to vi_optimization.py WITHOUT COLMAP triangulation.')
    print('Note: no feature descriptors → you cannot run COLMAP matching on top of this.')


if __name__ == '__main__':
    main()
