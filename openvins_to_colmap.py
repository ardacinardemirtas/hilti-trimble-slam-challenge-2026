#!/usr/bin/env python3
"""
Convert OpenVINS TUM trajectory + Kalibr calibration → COLMAP text reconstruction
for use with colmap_imu's vi_optimization.py.

The TUM file contains IMU-in-world poses (from trajectory_logger.py which looks up
the global→imu TF transform). This script converts them to cam0_from_world poses
using T_cam_imu from kalibr_imucam_chain.yaml, then writes COLMAP text format.

Outputs (in output_dir/):
  cameras.txt          — cam0 intrinsics in OPENCV_FISHEYE model
  images.txt           — one entry per pose, cam0_from_world
  points3D.txt         — empty (triangulate with COLMAP after this)
  image_timestamps.npy — dict: colmap_image_id → timestamp_ns (for vi_optimization.py)

Usage:
  python3 openvins_to_colmap.py \\
      --tum     /path/to/run.txt \\
      --calib   config/hilti_openvins/kalibr_imucam_chain.yaml \\
      --output  /path/to/colmap_init/ \\
      --images  /path/to/extracted_images/   # cam0 images, optional, for NAME field

The output reconstruction has no 3D points. Run COLMAP triangulation after:
  colmap triangulate_points \\
      --database_path database.db \\
      --image_path images/ \\
      --input_path colmap_init/ \\
      --output_path colmap_triangulated/ \\
      --Mapper.fix_existing_images 1
"""

import argparse
import math
import os
import re
from pathlib import Path

import numpy as np
import yaml


# ---------------------------------------------------------------------------
# Quaternion helpers  (all in [x, y, z, w] convention except where noted)
# ---------------------------------------------------------------------------

def quat_conjugate(q):
    """Conjugate (= inverse for unit quat). q = [x,y,z,w] → [-x,-y,-z,w]"""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_multiply(q1, q2):
    """Hamilton product q1 * q2, both [x,y,z,w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def quat_to_rot(q):
    """Unit quaternion [x,y,z,w] → 3×3 rotation matrix."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def rot_to_quat(R):
    """3×3 rotation matrix → unit quaternion [x,y,z,w]."""
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w]) / np.linalg.norm([x, y, z, w])


# ---------------------------------------------------------------------------
# Pose math
# ---------------------------------------------------------------------------

def imu_pose_to_cam_from_world(t_imu_in_world, q_world_from_imu, T_cam_imu):
    """
    Convert TUM IMU-in-world pose to COLMAP cam_from_world.

    Parameters
    ----------
    t_imu_in_world : (3,) position of IMU in world
    q_world_from_imu : (4,) [x,y,z,w] quaternion: world_from_imu rotation
    T_cam_imu : (4,4) cam0-from-IMU rigid transform (from Kalibr T_cam_imu field)

    Returns
    -------
    q_cam_from_world : (4,) [x,y,z,w]
    t_cam_from_world : (3,)  such that  p_cam = R*p_world + t
    """
    R_cam_from_imu = T_cam_imu[:3, :3]
    t_cam_from_imu = T_cam_imu[:3, 3]

    # Invert IMU-in-world to get imu_from_world
    R_world_from_imu = quat_to_rot(q_world_from_imu)
    R_imu_from_world = R_world_from_imu.T
    t_imu_from_world = -R_imu_from_world @ t_imu_in_world

    # Chain: cam_from_world = cam_from_imu * imu_from_world
    R_cam_from_world = R_cam_from_imu @ R_imu_from_world
    t_cam_from_world = R_cam_from_imu @ t_imu_from_world + t_cam_from_imu

    q_cam_from_world = rot_to_quat(R_cam_from_world)
    return q_cam_from_world, t_cam_from_world


# ---------------------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------------------

def load_tum(path):
    """Return list of (timestamp_s, tx, ty, tz, qx, qy, qz, qw)."""
    poses = []
    for line in open(path):
        if line.startswith('#') or not line.strip():
            continue
        parts = line.split()
        if len(parts) == 8:
            poses.append(tuple(float(x) for x in parts))
    return poses


def load_kalibr_cam_imu(yaml_path, cam='cam0'):
    """
    Parse kalibr_imucam_chain.yaml.
    Returns T_cam_imu (4×4 np.array), intrinsics dict.
    """
    content = open(yaml_path).read()
    # Strip %YAML:1.0 header which standard PyYAML doesn't support
    content = '\n'.join(l for l in content.splitlines() if not l.startswith('%'))
    data = yaml.safe_load(content)
    cam_data = data[cam]
    T = np.array(cam_data['T_cam_imu'])
    intr = cam_data['intrinsics']   # [fx, fy, cx, cy]
    dist = cam_data['distortion_coeffs']  # [k1,k2,k3,k4] equidistant
    res  = cam_data['resolution']   # [width, height]
    return T, intr, dist, res


# ---------------------------------------------------------------------------
# Writers
# ---------------------------------------------------------------------------

def write_cameras_txt(path, intr, dist, res):
    """
    Write COLMAP cameras.txt with OPENCV_FISHEYE model.
    OPENCV_FISHEYE params: fx fy cx cy k1 k2 k3 k4
    """
    fx, fy, cx, cy = intr
    k1, k2, k3, k4 = dist
    w, h = res
    with open(path, 'w') as f:
        f.write('# Camera list with one line of data per camera:\n')
        f.write('#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n')
        f.write(f'1 OPENCV_FISHEYE {w} {h} {fx} {fy} {cx} {cy} {k1} {k2} {k3} {k4}\n')


def write_images_txt(path, colmap_poses):
    """
    colmap_poses: list of (image_id, qw, qx, qy, qz, tx, ty, tz, name)
    """
    with open(path, 'w') as f:
        f.write('# Image list with two lines of data per image:\n')
        f.write('#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n')
        f.write('#   POINTS2D[] as (X, Y, POINT3D_ID)\n')
        for (iid, qw, qx, qy, qz, tx, ty, tz, name) in colmap_poses:
            f.write(f'{iid} {qw:.9f} {qx:.9f} {qy:.9f} {qz:.9f} '
                    f'{tx:.9f} {ty:.9f} {tz:.9f} 1 {name}\n')
            f.write('\n')  # empty POINTS2D line (no observations yet)


def write_points3d_txt(path):
    with open(path, 'w') as f:
        f.write('# 3D point list with one line of data per point:\n')
        f.write('#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--tum',    required=True, help='OpenVINS TUM trajectory file')
    parser.add_argument('--calib',  required=True, help='kalibr_imucam_chain.yaml path')
    parser.add_argument('--output', required=True, help='Output directory for COLMAP text files')
    parser.add_argument('--cam',    default='cam0', help='Which camera to use (cam0/cam1)')
    parser.add_argument('--images', default=None,
                        help='Directory of extracted cam images. If provided, image names are '
                             'matched by timestamp; otherwise names are auto-generated.')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    # Load calibration
    T_cam_imu, intr, dist, res = load_kalibr_cam_imu(args.calib, args.cam)
    print(f'Loaded {args.cam} calibration: fx={intr[0]:.1f}, res={res}')
    print(f'T_cam_imu:\n{T_cam_imu}')

    # Load image name index (if available) keyed by timestamp_ns
    image_by_ts = {}
    if args.images:
        for fn in sorted(Path(args.images).iterdir()):
            # Expected format: frame_NNNNNNNNNNNNNNNNN.png (ns timestamp)
            m = re.search(r'(\d{10,19})\.', fn.name)
            if m:
                image_by_ts[int(m.group(1))] = fn.name

    # Load TUM poses
    poses = load_tum(args.tum)
    print(f'Loaded {len(poses)} poses from {args.tum}')

    # Convert and build COLMAP pose list
    colmap_poses = []
    image_timestamps = {}  # colmap image_id → timestamp_ns

    for image_id, (ts, tx, ty, tz, qx, qy, qz, qw) in enumerate(poses, start=1):
        t_imu_in_world = np.array([tx, ty, tz])
        q_world_from_imu = np.array([qx, qy, qz, qw])

        q_cfr_w, t_cfr_w = imu_pose_to_cam_from_world(
            t_imu_in_world, q_world_from_imu, T_cam_imu
        )

        # COLMAP quaternion convention: qw first
        qw_c, qx_c, qy_c, qz_c = q_cfr_w[3], q_cfr_w[0], q_cfr_w[1], q_cfr_w[2]
        tx_c, ty_c, tz_c = t_cfr_w

        # Image name
        ts_ns = int(round(ts * 1e9))
        if image_by_ts:
            # Find closest image by timestamp (within 5 ms)
            closest = min(image_by_ts.keys(), key=lambda k: abs(k - ts_ns))
            if abs(closest - ts_ns) < 5_000_000:
                name = image_by_ts[closest]
            else:
                name = f'frame_{ts_ns:019d}.png'
        else:
            name = f'frame_{ts_ns:019d}.png'

        colmap_poses.append((image_id, qw_c, qx_c, qy_c, qz_c, tx_c, ty_c, tz_c, name))
        image_timestamps[image_id] = ts_ns

    # Write outputs
    cameras_path = os.path.join(args.output, 'cameras.txt')
    images_path  = os.path.join(args.output, 'images.txt')
    points_path  = os.path.join(args.output, 'points3D.txt')
    ts_path      = os.path.join(args.output, 'image_timestamps.npy')

    write_cameras_txt(cameras_path, intr, dist, res)
    write_images_txt(images_path, colmap_poses)
    write_points3d_txt(points_path)
    np.save(ts_path, image_timestamps)

    print(f'Written:')
    print(f'  {cameras_path}')
    print(f'  {images_path}  ({len(colmap_poses)} images)')
    print(f'  {points_path}  (empty — run COLMAP triangulation next)')
    print(f'  {ts_path}')
    print()
    print('Next steps:')
    print('  1. Extract cam0 images from bag (ros2bag_to_euroc.py or similar)')
    print('  2. colmap feature_extractor --database_path db.db --image_path images/ --ImageReader.camera_model OPENCV_FISHEYE')
    print('  3. colmap exhaustive_matcher --database_path db.db  (or sequential_matcher)')
    print('  4. colmap triangulate_points --database_path db.db --image_path images/ \\')
    print('         --input_path <output> --output_path <triangulated> --Mapper.fix_existing_images 1')
    print('  5. Run vi_optimization.py with triangulated/ and image_timestamps.npy')


if __name__ == '__main__':
    main()


# ---------------------------------------------------------------------------
# Full pipeline summary (append as module docstring extension):
#
# STEP 1 — Extract cam0 images from bag (run inside Apptainer):
#   python3 challenge_tools_ros/bag_helper/ros2bag_to_euroc.py \
#       --bag <bag_dir> --output <euroc_dir> --topics /cam0/image_raw/compressed
#   Images land in <euroc_dir>/cam0/data/*.png with timestamp filenames.
#
# STEP 2 — Convert OpenVINS TUM to COLMAP init (any environment with numpy+yaml):
#   python3 openvins_to_colmap.py \
#       --tum <run>.txt \
#       --calib config/hilti_openvins/kalibr_imucam_chain.yaml \
#       --output colmap_init/ \
#       --images <euroc_dir>/cam0/data/
#
# STEP 3 — COLMAP feature extraction (standard COLMAP binary):
#   colmap feature_extractor \
#       --database_path db.db \
#       --image_path <euroc_dir>/cam0/data/ \
#       --ImageReader.camera_model OPENCV_FISHEYE \
#       --ImageReader.camera_params "465.3,465.3,730.0,720.1,0.026,-0.011,-0.0017,0.00015"
#
# STEP 4 — COLMAP matching (sequential for video):
#   colmap sequential_matcher --database_path db.db --SequentialMatching.overlap 10
#
# STEP 5 — Triangulation with fixed OpenVINS poses:
#   colmap triangulate_points \
#       --database_path db.db \
#       --image_path <euroc_dir>/cam0/data/ \
#       --input_path colmap_init/ \
#       --output_path colmap_triangulated/ \
#       --Mapper.fix_existing_images 1
#
# STEP 6 — Extract IMU data from bag (run inside Apptainer):
#   python3 bag_to_imu_npy.py <bag_dir> imu.npy
#
# STEP 7 — Run VI optimization:
#   python3 /path/to/colmap_imu/python/examples/vi_optimization.py
#   (adapt run() to use colmap_triangulated/, db.db, image_timestamps.npy, imu.npy
#    and set imu_calib per COLMAP_IMU_FORK_CAPABILITIES.md Hilti section)
# ---------------------------------------------------------------------------
