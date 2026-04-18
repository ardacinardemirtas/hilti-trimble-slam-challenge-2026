#!/usr/bin/env python3
"""
Convert COLMAP sparse_txt output to TUM trajectory format for challenge evaluation.

COLMAP stores camera-from-world poses (R_cw, t_cw) where:
    p_camera = R_cw @ p_world + t_cw

TUM format needs world-from-camera: timestamp tx ty tz qx qy qz qw

Usage:
    # Convert a specific run
    python3 challenge_tools_ros/colmap_to_tum.py \
        --colmap-dir /path/to/colmap_gt_runs/floor_2/2025-05-05_run_1/cfg_10hz_jointBA/sparse_txt/0_localized_refined \
        --output groundtruth/floor_2_2025-05-05_run_1.txt

    # Auto-discover and convert all runs
    python3 challenge_tools_ros/colmap_to_tum.py \
        --colmap-root /path/to/colmap_gt_runs \
        --output-dir groundtruth/
"""

import argparse
import re
from pathlib import Path

import numpy as np


def parse_images_txt(path):
    """Return {image_id: timestamp_seconds} by parsing filenames like *_<ns>.png."""
    id_to_ts = {}
    with open(path) as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith('#')]
    i = 0
    while i < len(lines):
        parts = lines[i].split()
        # IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME
        if len(parts) >= 10:
            image_id = int(parts[0])
            name = parts[9]
            m = re.search(r'_(\d{10,})\.', name)
            if m:
                ts_ns = int(m.group(1))
                id_to_ts[image_id] = ts_ns / 1e9
        i += 2  # skip the POINTS2D line
    return id_to_ts


def parse_frames_txt(path):
    """
    Return list of (timestamp, qw, qx, qy, qz, tx, ty, tz) from frames.txt.

    frames.txt format per line:
        FRAME_ID RIG_ID QW QX QY QZ TX TY TZ NUM_DATA_IDS SENSOR_TYPE SENSOR_ID DATA_ID ...
    """
    frames = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            # FRAME_ID RIG_ID QW QX QY QZ TX TY TZ NUM_DATA_IDS [SENSOR_TYPE SENSOR_ID DATA_ID ...]
            if len(parts) < 10:
                continue
            qw, qx, qy, qz = float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5])
            tx, ty, tz = float(parts[6]), float(parts[7]), float(parts[8])
            num_data = int(parts[9])
            # Parse DATA_IDs to later look up timestamps
            data_ids = []
            for k in range(num_data):
                base = 10 + k * 3
                if base + 2 < len(parts):
                    data_ids.append(int(parts[base + 2]))
            frames.append((data_ids, qw, qx, qy, qz, tx, ty, tz))
    return frames


def colmap_pose_to_tum(qw, qx, qy, qz, tx, ty, tz):
    """
    Convert COLMAP camera-from-world pose to TUM world-from-camera.

    COLMAP: p_cam = R_cw @ p_world + t_cw
    Camera center in world: t_wc = -R_cw^T @ t_cw
    Orientation: q_wc = q_cw^{-1} (conjugate for unit quaternion)
    """
    # Invert quaternion: conjugate of (qw, qx, qy, qz)
    qw_wc, qx_wc, qy_wc, qz_wc = qw, -qx, -qy, -qz

    # Rotate -t_cw by R_wc to get camera center in world
    # R_wc applied to vector v: use quaternion sandwich product
    t_cw = np.array([tx, ty, tz])
    t_wc = _quat_rotate(qw_wc, qx_wc, qy_wc, qz_wc, -t_cw)

    return t_wc[0], t_wc[1], t_wc[2], qx_wc, qy_wc, qz_wc, qw_wc


def _quat_rotate(qw, qx, qy, qz, v):
    """Rotate vector v by quaternion (qw, qx, qy, qz)."""
    # Rodrigues via quaternion: v' = q * [0,v] * q^{-1}
    # Expanded form (efficient):
    t = 2.0 * np.cross([qx, qy, qz], v)
    return v + qw * t + np.cross([qx, qy, qz], t)


def convert_dir(colmap_dir, output_file):
    colmap_dir = Path(colmap_dir)
    images_txt = colmap_dir / "images.txt"
    frames_txt = colmap_dir / "frames.txt"

    if not images_txt.exists() or not frames_txt.exists():
        raise FileNotFoundError(f"Missing images.txt or frames.txt in {colmap_dir}")

    id_to_ts = parse_images_txt(images_txt)
    frames = parse_frames_txt(frames_txt)

    poses = []
    missing_ts = 0
    for data_ids, qw, qx, qy, qz, tx, ty, tz in frames:
        ts = None
        for did in data_ids:
            if did in id_to_ts:
                ts = id_to_ts[did]
                break
        if ts is None:
            missing_ts += 1
            continue
        wx, wy, wz, qx_w, qy_w, qz_w, qw_w = colmap_pose_to_tum(qw, qx, qy, qz, tx, ty, tz)
        poses.append((ts, wx, wy, wz, qx_w, qy_w, qz_w, qw_w))

    if missing_ts:
        print(f"  [warn] {missing_ts} frames had no matching timestamp in images.txt")

    poses.sort(key=lambda p: p[0])

    output_file = Path(output_file)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for ts, wx, wy, wz, qx_w, qy_w, qz_w, qw_w in poses:
            f.write(f"{ts:.9f} {wx:.9f} {wy:.9f} {wz:.9f} "
                    f"{qx_w:.9f} {qy_w:.9f} {qz_w:.9f} {qw_w:.9f}\n")

    print(f"  Wrote {len(poses)} poses -> {output_file}")
    return len(poses)


def auto_discover(colmap_root, output_dir, cfg="cfg_10hz_jointBA", model="0_localized_refined"):
    """Find all sparse_txt dirs and convert each one into output_dir/floor/date_run.txt."""
    colmap_root = Path(colmap_root)
    output_dir = Path(output_dir)

    pattern = f"*/*/{cfg}/sparse_txt/{model}"
    found = sorted(colmap_root.glob(pattern))
    if not found:
        print(f"No runs found matching {colmap_root / pattern}")
        return

    for sparse_dir in found:
        # floor_X/YYYY-MM-DD_run_Z/cfg.../sparse_txt/model
        parts = sparse_dir.relative_to(colmap_root).parts
        floor = parts[0]      # e.g. floor_2
        date_run = parts[1]   # e.g. 2025-05-05_run_1
        out_file = output_dir / floor / f"{date_run}.txt"
        print(f"Converting {floor}/{date_run} ...")
        convert_dir(sparse_dir, out_file)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--colmap-dir',
                       help="Path to a single sparse_txt/model dir (images.txt + frames.txt)")
    group.add_argument('--colmap-root',
                       help="Root of colmap_gt_runs; auto-discovers all runs")

    parser.add_argument('--output', help="Output TUM file (used with --colmap-dir)")
    parser.add_argument('--output-dir', default='groundtruth',
                        help="Output directory for auto-discovery (default: groundtruth/)")
    parser.add_argument('--cfg', default='cfg_10hz_jointBA',
                        help="COLMAP config subfolder (default: cfg_10hz_jointBA)")
    parser.add_argument('--model', default='0_localized_refined',
                        help="Model subfolder inside sparse_txt (default: 0_localized_refined)")
    args = parser.parse_args()

    if args.colmap_dir:
        if not args.output:
            parser.error("--output is required when using --colmap-dir")
        convert_dir(args.colmap_dir, args.output)
    else:
        auto_discover(args.colmap_root, args.output_dir, cfg=args.cfg, model=args.model)


if __name__ == '__main__':
    main()
