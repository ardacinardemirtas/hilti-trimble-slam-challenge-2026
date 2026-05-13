#!/usr/bin/env python3
"""
Hilti x Trimble SLAM Challenge 2026 - Local Evaluation Script

Computes per-run exponential scores and produces a detailed accuracy report.

Usage:
    # Evaluate a single run (SLAM task):
    python evaluate.py --gt groundtruth/floor_1_2025-05-05_run_1.txt \
                       --pred results/floor_1_2025-05-05_run_1.txt

    # Evaluate all runs in a folder (SLAM task):
    python evaluate.py --gt groundtruth/ --pred results/

    # Localization task (no SE3 alignment, z ignored):
    python evaluate.py --gt groundtruth/ --pred results/ --task localization

    # Save plots:
    python evaluate.py --gt groundtruth/ --pred results/ --plot
"""

import argparse
import math
import re
import shutil
import sys
from pathlib import Path

import numpy as np


# ── Challenge constants ────────────────────────────────────────────────────────
SCORE_A = 100.0
SCORE_C = 0.46051701859880917  # derived so that error=0→100, error=10m→1
SKIP_SECONDS = 5.0             # first 5 s of each run excluded (LiDAR startup)
COVERAGE_MIN = 99.0            # % of GT poses that must be matched
MAX_TIMESTAMP_DIFF = 0.05      # seconds — tolerance for timestamp matching


# ── I/O ────────────────────────────────────────────────────────────────────────

def load_tum(filepath):
    """Return (timestamps[N], positions[N,3], quaternions[N,4]) from a TUM file."""
    timestamps, positions, quaternions = [], [], []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) != 8:
                continue
            timestamps.append(float(parts[0]))
            positions.append([float(x) for x in parts[1:4]])
            quaternions.append([float(x) for x in parts[4:8]])
    if not timestamps:
        raise ValueError(f"No valid poses found in {filepath}")
    return np.array(timestamps), np.array(positions), np.array(quaternions)


# ── Trajectory matching ────────────────────────────────────────────────────────

def match_by_timestamp(gt_times, pred_times, max_diff=MAX_TIMESTAMP_DIFF):
    """
    For each GT timestamp find the closest prediction timestamp.
    Returns (gt_idx, pred_idx) arrays for pairs within max_diff.
    """
    pred_arr = np.asarray(pred_times)
    gt_idx, pred_idx = [], []
    for i, t in enumerate(gt_times):
        j = int(np.argmin(np.abs(pred_arr - t)))
        if abs(pred_arr[j] - t) <= max_diff:
            gt_idx.append(i)
            pred_idx.append(j)
    return np.array(gt_idx, dtype=int), np.array(pred_idx, dtype=int)


# ── Quaternion helpers (no scipy) ─────────────────────────────────────────────

def mat_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion [qx, qy, qz, qw]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return np.array([(R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s,
                         (R[1,0]-R[0,1])*s, 0.25/s])
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return np.array([0.25*s, (R[0,1]+R[1,0])/s,
                         (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s])
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return np.array([(R[0,1]+R[1,0])/s, 0.25*s,
                         (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return np.array([(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s,
                         0.25*s, (R[1,0]-R[0,1])/s])


def quat_mul_batch(q1, q2):
    """Hamilton product of q1 (single [xyzw]) with q2 (N×4 [xyzw])."""
    x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
    x2, y2, z2, w2 = q2[:,0], q2[:,1], q2[:,2], q2[:,3]
    return np.stack([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ], axis=1)


# ── Sim(3) alignment (Umeyama, with scale) ────────────────────────────────────

def align_se3(src, dst):
    """
    Find rotation R and translation t minimising ||dst - (R @ src.T).T - t||.
    src, dst: (N, D) arrays (D=3 for SLAM, D=2 for Localization-xy).
    Returns (R [DxD], t [D], s=1.0).
    """
    n, d = src.shape
    mu_s = src.mean(0)
    mu_d = dst.mean(0)
    src_c = src - mu_s
    dst_c = dst - mu_d
    H = src_c.T @ dst_c / n
    U, S_vals, Vt = np.linalg.svd(H)
    W = np.eye(d)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        W[d - 1, d - 1] = -1
    R = Vt.T @ W @ U.T
    s = 1.0
    t = mu_d - R @ mu_s
    return R, t, s


# ── Scoring ────────────────────────────────────────────────────────────────────

def per_pose_score(errors):
    """Return array of per-pose scores (before capping)."""
    return SCORE_A * np.exp(-SCORE_C * errors)


def run_score(errors):
    """Mean of per-pose scores, capped at 100."""
    return min(float(per_pose_score(errors).mean()), 100.0)


# ── Single-run evaluation ──────────────────────────────────────────────────────

def evaluate_run(gt_file, pred_file, task='slam'):
    """
    Evaluate one predicted trajectory against ground truth.

    Returns a dict with all metrics, or 'score'=0 and 'error' string on failure.
    """
    pred_path = Path(pred_file)
    # Build a descriptive run name: use GT stem (always floor_X_date_run)
    run_name = Path(gt_file).stem
    r = dict(run=run_name, task=task,
             gt_file=str(gt_file), pred_file=str(pred_file),
             score=0.0, coverage=0.0, n_gt=0, n_pred=0, n_matched=0,
             ate_mean=None, ate_rmse=None, ate_median=None,
             ate_max=None, ate_std=None,
             p25=None, p50=None, p75=None, p90=None, p95=None,
             errors=None, error_msg=None,
             gt_pos_full=None, pred_pos_full=None,
             gt_pos_matched=None, pred_pos_aligned=None,
             pred_times_full=None, pred_quat_full=None,
             align_R=None, align_t=None, align_s=None)

    try:
        gt_times, gt_pos, _ = load_tum(gt_file)
        pred_times, pred_pos, pred_quat = load_tum(pred_file)
    except Exception as e:
        r['error_msg'] = str(e)
        return r

    r['gt_pos_full'] = gt_pos
    r['pred_pos_full'] = pred_pos
    r['pred_times_full'] = pred_times
    r['pred_quat_full'] = pred_quat

    # Drop first SKIP_SECONDS of GT
    t0 = gt_times[0] + SKIP_SECONDS
    mask = gt_times >= t0
    gt_times, gt_pos = gt_times[mask], gt_pos[mask]

    r['n_gt'] = len(gt_times)
    r['n_pred'] = len(pred_times)

    if r['n_gt'] == 0:
        r['error_msg'] = "No GT poses remain after skipping first 5 s"
        return r

    # Timestamp matching
    gi, pi = match_by_timestamp(gt_times, pred_times)
    r['n_matched'] = len(gi)
    r['coverage'] = 100.0 * r['n_matched'] / r['n_gt']

    if r['coverage'] < COVERAGE_MIN:
        r['error_msg'] = (f"Coverage {r['coverage']:.2f}% < required {COVERAGE_MIN}% "
                          f"— score set to 0")
        r['gt_pos_matched'] = gt_pos[gi]
        r['pred_pos_aligned'] = pred_pos[pi]  # unaligned — no SE3 fit yet
        return r

    gt_m = gt_pos[gi]       # matched GT positions
    pred_m = pred_pos[pi]   # matched predicted positions

    # For localization: ignore z
    if task == 'localization':
        gt_eval = gt_m[:, :2]
        pred_eval = pred_m[:, :2]
    else:
        gt_eval = gt_m
        pred_eval = pred_m

    # Align (SLAM: SE3 in 3-D; Localization: SE2 in 2-D, but we use the same routine)
    try:
        R, t, s = align_se3(pred_eval, gt_eval)
        pred_aligned = s * (R @ pred_eval.T).T + t
    except Exception as e:
        r['error_msg'] = f"Alignment failed: {e}"
        return r

    errors = np.linalg.norm(gt_eval - pred_aligned, axis=1)
    r['errors'] = errors
    r['gt_pos_matched'] = gt_m
    r['pred_pos_aligned'] = s * (R @ pred_m.T).T + t  # always 3-D aligned
    r['align_R'] = R
    r['align_t'] = t
    r['align_s'] = s
    r['score'] = run_score(errors)
    r['ate_mean']   = float(errors.mean())
    r['ate_rmse']   = float(np.sqrt((errors ** 2).mean()))
    r['ate_median'] = float(np.median(errors))
    r['ate_max']    = float(errors.max())
    r['ate_std']    = float(errors.std())
    r['p25']  = float(np.percentile(errors, 25))
    r['p50']  = float(np.percentile(errors, 50))
    r['p75']  = float(np.percentile(errors, 75))
    r['p90']  = float(np.percentile(errors, 90))
    r['p95']  = float(np.percentile(errors, 95))
    return r


# ── Reporting ──────────────────────────────────────────────────────────────────

def print_run_report(r):
    name = r['run']
    if r['error_msg']:
        status = f"  ERROR: {r['error_msg']}"
    else:
        status = ""

    print(f"\n{'─'*60}")
    print(f"  Run : {name}")
    print(f"  Task: {r['task'].upper()}")
    if r['error_msg'] and r['score'] == 0:
        print(f"  Score: 0.00  ← {r['error_msg']}")
        print(f"  Coverage: {r['coverage']:.2f}%  "
              f"(matched {r['n_matched']} / {r['n_gt']} GT poses)")
        return

    print(f"  Score   : {r['score']:7.3f} / 100")
    print(f"  Coverage: {r['coverage']:.2f}%  "
          f"({r['n_matched']} matched / {r['n_gt']} GT / {r['n_pred']} pred)")
    print(f"  ── Translational Error (m) ──────────────────")
    print(f"     Mean   : {r['ate_mean']:.4f}")
    print(f"     RMSE   : {r['ate_rmse']:.4f}")
    print(f"     Median : {r['ate_median']:.4f}")
    print(f"     Std    : {r['ate_std']:.4f}")
    print(f"     Max    : {r['ate_max']:.4f}")
    print(f"  ── Percentiles ──────────────────────────────")
    print(f"     25th   : {r['p25']:.4f} m")
    print(f"     50th   : {r['p50']:.4f} m")
    print(f"     75th   : {r['p75']:.4f} m")
    print(f"     90th   : {r['p90']:.4f} m")
    print(f"     95th   : {r['p95']:.4f} m")
    if r['error_msg']:
        print(f"  Note: {r['error_msg']}")


def print_summary(results):
    print(f"\n{'═'*60}")
    print("  SUMMARY")
    print(f"{'═'*60}")
    print(f"  {'Run':<40} {'Score':>7}  {'RMSE (m)':>9}  {'Coverage':>9}")
    print(f"  {'─'*40} {'─'*7}  {'─'*9}  {'─'*9}")
    total = 0.0
    n_valid = 0
    for r in results:
        score_s = f"{r['score']:7.3f}"
        rmse_s  = f"{r['ate_rmse']:.4f}" if r['ate_rmse'] is not None else "    N/A"
        cov_s   = f"{r['coverage']:.1f}%"
        flag    = "  *" if r['error_msg'] else ""
        print(f"  {r['run']:<40} {score_s}  {rmse_s:>9}  {cov_s:>9}{flag}")
        total += r['score']
        if r['score'] > 0:
            n_valid += 1

    max_possible = 100.0 * len(results)
    print(f"  {'─'*40} {'─'*7}")
    print(f"  {'TOTAL':<40} {total:7.3f}  (max: {max_possible:.0f})")
    print(f"  Runs with score > 0: {n_valid} / {len(results)}")
    if any(r['ate_rmse'] is not None for r in results):
        valid_rmse = [r['ate_rmse'] for r in results if r['ate_rmse'] is not None]
        print(f"  Mean RMSE across valid runs: {np.mean(valid_rmse):.4f} m")
    print(f"  * = score set to 0 due to low coverage or error")


# ── Optional plotting ──────────────────────────────────────────────────────────

def _run_output_path(run_name, out_dir, ext):
    """Return structured output path: out_dir/floor_X/YYYY-MM-DD_run_Z.<ext>"""
    m = re.match(r'^(floor_\w+?)_((\d{4}-.+))$', run_name)
    if m:
        return Path(out_dir) / m.group(1) / f"{m.group(2)}.{ext}"
    return Path(out_dir) / f"{run_name}.{ext}"


def write_run_report(r, results_dir):
    """Write per-run metrics to a text file mirroring the colmap_plots structure."""
    path = _run_output_path(r['run'], results_dir, 'txt')
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        f.write(f"Run  : {r['run']}\n")
        f.write(f"Task : {r['task'].upper()}\n")
        if r['error_msg'] and r['score'] == 0:
            f.write(f"Score: 0.00\n")
            f.write(f"Error: {r['error_msg']}\n")
            f.write(f"Coverage: {r['coverage']:.2f}%  "
                    f"({r['n_matched']} matched / {r['n_gt']} GT / {r['n_pred']} pred)\n")
        else:
            f.write(f"Score   : {r['score']:.3f} / 100\n")
            f.write(f"Coverage: {r['coverage']:.2f}%  "
                    f"({r['n_matched']} matched / {r['n_gt']} GT / {r['n_pred']} pred)\n")
            f.write(f"ATE Mean  : {r['ate_mean']:.4f} m\n")
            f.write(f"ATE RMSE  : {r['ate_rmse']:.4f} m\n")
            f.write(f"ATE Median: {r['ate_median']:.4f} m\n")
            f.write(f"ATE Std   : {r['ate_std']:.4f} m\n")
            f.write(f"ATE Max   : {r['ate_max']:.4f} m\n")
            f.write(f"P25 : {r['p25']:.4f} m\n")
            f.write(f"P50 : {r['p50']:.4f} m\n")
            f.write(f"P75 : {r['p75']:.4f} m\n")
            f.write(f"P90 : {r['p90']:.4f} m\n")
            f.write(f"P95 : {r['p95']:.4f} m\n")
        if r['error_msg']:
            f.write(f"Note: {r['error_msg']}\n")
    print(f"  [report] saved to {path}")


def save_best_result(r, best_dir, plot_dir=None, aligned=True):
    """
    Copy this run's results to best_dir if its score beats the current best.

    Layout:
      best_dir/floor_X/floor_X_date_run.txt         — TUM trajectory (submission-ready)
      best_dir/floor_X/floor_X_date_run.png         — plot
      best_dir/floor_X/floor_X_date_run_report.txt  — evaluation metrics

    aligned: if True, write the Sim3-aligned trajectory; if False, copy the raw file.
    """
    # Files use the full run name (floor_X_YYYY-MM-DD_run_Z) so TUM is submission-ready.
    # Stored under best_dir/floor_X/ to mirror colmap_plots structure.
    m = re.match(r'^(floor_\w+?)_((\d{4}-.+))$', r['run'])
    sub_dir = Path(best_dir) / m.group(1) if m else Path(best_dir)
    traj_path   = sub_dir / f"{r['run']}.txt"
    plot_path   = sub_dir / f"{r['run']}.png"
    report_path = sub_dir / f"{r['run']}_report.txt"

    # Read existing best score from the report file if present
    best_score = -1.0
    if report_path.exists():
        for line in report_path.read_text().splitlines():
            if line.startswith('Score'):
                try:
                    best_score = float(line.split(':')[1].split('/')[0].strip())
                except ValueError:
                    pass
                break

    if r['score'] <= best_score:
        print(f"  [best] {r['run']}: score {r['score']:.3f} ≤ current best "
              f"{best_score:.3f} — skipping")
        return

    action = 'updated' if best_score >= 0 else 'saved'
    traj_path.parent.mkdir(parents=True, exist_ok=True)

    # TUM trajectory — raw copy or Sim3-aligned
    can_align = (aligned and r['align_R'] is not None
                 and r['pred_times_full'] is not None)
    if can_align:
        R_al, t_al, s_al = r['align_R'], r['align_t'], r['align_s']
        pos_al = s_al * (R_al @ r['pred_pos_full'].T).T + t_al
        q_align = mat_to_quat(R_al)
        quat_al = quat_mul_batch(q_align, r['pred_quat_full'])
        # normalise
        quat_al /= np.linalg.norm(quat_al, axis=1, keepdims=True)
        with open(traj_path, 'w') as f:
            f.write("# timestamp tx ty tz qx qy qz qw  (Sim3-aligned)\n")
            for ts, p, q in zip(r['pred_times_full'], pos_al, quat_al):
                f.write(f"{ts:.9f} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} "
                        f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
    else:
        shutil.copy2(r['pred_file'], traj_path)

    # Plot (copy from plot_dir if available, skip otherwise)
    if plot_dir:
        src_plot = _run_output_path(r['run'], plot_dir, 'png')
        if src_plot.exists():
            shutil.copy2(src_plot, plot_path)
        else:
            print(f"  [best] plot not found at {src_plot} — skipping plot copy")

    # Metrics report
    with open(report_path, 'w') as f:
        f.write(f"Run  : {r['run']}\n")
        f.write(f"Task : {r['task'].upper()}\n")
        if r['error_msg'] and r['score'] == 0:
            f.write(f"Score: 0.00\n")
            f.write(f"Error: {r['error_msg']}\n")
            f.write(f"Coverage: {r['coverage']:.2f}%  "
                    f"({r['n_matched']} matched / {r['n_gt']} GT / {r['n_pred']} pred)\n")
        else:
            f.write(f"Score   : {r['score']:.3f} / 100\n")
            f.write(f"Coverage: {r['coverage']:.2f}%  "
                    f"({r['n_matched']} matched / {r['n_gt']} GT / {r['n_pred']} pred)\n")
            f.write(f"ATE Mean  : {r['ate_mean']:.4f} m\n")
            f.write(f"ATE RMSE  : {r['ate_rmse']:.4f} m\n")
            f.write(f"ATE Median: {r['ate_median']:.4f} m\n")
            f.write(f"ATE Std   : {r['ate_std']:.4f} m\n")
            f.write(f"ATE Max   : {r['ate_max']:.4f} m\n")
            f.write(f"P25 : {r['p25']:.4f} m\n")
            f.write(f"P50 : {r['p50']:.4f} m\n")
            f.write(f"P75 : {r['p75']:.4f} m\n")
            f.write(f"P90 : {r['p90']:.4f} m\n")
            f.write(f"P95 : {r['p95']:.4f} m\n")
        if r['error_msg']:
            f.write(f"Note: {r['error_msg']}\n")

    prev = f" (was {best_score:.3f})" if best_score >= 0 else ""
    mode = "sim3-aligned" if can_align else "raw"
    print(f"  [best] {action} ({mode}): {r['run']} — score {r['score']:.3f}{prev} → {traj_path}")


def plot_run(r, out_dir=None):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
        from matplotlib.collections import LineCollection
    except ImportError:
        print("  [plot] matplotlib not available — skipping plots")
        return

    gt_full    = r.get('gt_pos_full')
    gt_matched = r.get('gt_pos_matched')
    pred_aligned = r.get('pred_pos_aligned')

    if gt_full is None and gt_matched is None:
        return

    has_errors = r.get('errors') is not None

    title = f"{r['run']}  |  score={r['score']:.2f}"
    if has_errors:
        title += f"  |  RMSE={r['ate_rmse']:.4f} m  |  cov={r['coverage']:.1f}%"
    else:
        title += f"  |  cov={r['coverage']:.1f}%  [{r['error_msg']}]"

    fig = plt.figure(figsize=(20, 5))
    gs = gridspec.GridSpec(1, 3, figure=fig, wspace=0.4)
    ax_traj = fig.add_subplot(gs[0])
    ax_err  = fig.add_subplot(gs[1])
    ax_cdf  = fig.add_subplot(gs[2])
    fig.suptitle(title, fontsize=10)

    # ── Trajectory overlay (XY top-down, pred colored by Z deviation) ─────────
    if gt_full is not None:
        ax_traj.plot(gt_full[:, 0], gt_full[:, 1],
                     color='tab:green', linewidth=1.2, label='GT', zorder=2)

    cb = None
    if gt_matched is not None and pred_aligned is not None and len(pred_aligned) > 1:
        z_diff = pred_aligned[:, 2] - gt_matched[:, 2]
        abs_max = max(np.abs(z_diff).max(), 1e-6)
        norm = plt.Normalize(vmin=-abs_max, vmax=abs_max)
        cmap = plt.get_cmap('turbo')

        pts = pred_aligned[:, :2].reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        z_mid = (z_diff[:-1] + z_diff[1:]) / 2
        lc = LineCollection(segs, cmap=cmap, norm=norm, linewidth=1.2,
                            alpha=0.85, zorder=3)
        lc.set_array(z_mid)
        ax_traj.add_collection(lc)
        cb = fig.colorbar(lc, ax=ax_traj, shrink=0.8, pad=0.02)
        cb.set_label('Z deviation (m)', fontsize=8)
        ax_traj.autoscale()
        ax_traj.legend(fontsize=7)

    ax_traj.set_xlabel('X (m)')
    ax_traj.set_ylabel('Y (m)')
    ax_traj.set_title('Trajectory (XY, pred colored by ΔZ)')
    ax_traj.set_aspect('equal', adjustable='datalim')
    ax_traj.grid(True, alpha=0.3)

    # ── Error over time ────────────────────────────────────────────────────────
    if has_errors:
        errors = r['errors']
        ax_err.plot(errors, linewidth=0.8, color='steelblue')
        ax_err.axhline(r['ate_mean'], color='red', linestyle='--',
                       label=f"mean={r['ate_mean']:.3f} m")
        ax_err.set_xlabel("Pose index")
        ax_err.set_ylabel("Translational error (m)")
        ax_err.set_title("Error per pose")
        ax_err.legend()
        ax_err.grid(True, alpha=0.3)
    else:
        ax_err.text(0.5, 0.5, f"No error data\n(coverage {r['coverage']:.1f}%)",
                    ha='center', va='center', transform=ax_err.transAxes, fontsize=10)
        ax_err.set_title("Error per pose")

    # ── CDF ───────────────────────────────────────────────────────────────────
    if has_errors:
        errors = r['errors']
        sorted_e = np.sort(errors)
        cdf = np.arange(1, len(sorted_e) + 1) / len(sorted_e)
        ax_cdf.plot(sorted_e, cdf, color='steelblue')
        for pct, val in [(0.5, r['p50']), (0.75, r['p75']), (0.95, r['p95'])]:
            ax_cdf.axvline(val, linestyle=':', alpha=0.7,
                           label=f"p{int(pct*100)}={val:.3f} m")
        ax_cdf.set_xlabel("Translational error (m)")
        ax_cdf.set_ylabel("CDF")
        ax_cdf.set_title("Error CDF")
        ax_cdf.legend()
        ax_cdf.grid(True, alpha=0.3)
    else:
        ax_cdf.text(0.5, 0.5, f"No error data\n(coverage {r['coverage']:.1f}%)",
                    ha='center', va='center', transform=ax_cdf.transAxes, fontsize=10)
        ax_cdf.set_title("Error CDF")

    if out_dir:
        path = _run_output_path(r['run'], out_dir, 'png')
        path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(path, dpi=120, bbox_inches='tight')
        print(f"  [plot] saved to {path}")
    else:
        plt.show()
    plt.close(fig)


# ── Entry point ────────────────────────────────────────────────────────────────

def collect_pairs(gt_path, pred_path):
    """
    Return list of (gt_file, pred_file) path pairs.

    GT files are always flat: groundtruth/floor_X_YYYY-MM-DD_run_Z.txt
    Prediction files may be flat (same naming) or structured:
        predictions/floor_X/YYYY-MM-DD_run_Z.txt
    Both layouts are detected automatically.
    """
    gt_path = Path(gt_path)
    pred_path = Path(pred_path)

    if gt_path.is_file() and pred_path.is_file():
        return [(gt_path, pred_path)]

    if gt_path.is_dir() and pred_path.is_file():
        stem = pred_path.stem
        gt_file = gt_path / f"{stem}.txt"
        if gt_file.exists():
            return [(gt_file, pred_path)]
        raise FileNotFoundError(f"No GT file found for {stem} in {gt_path}")

    if gt_path.is_dir() and pred_path.is_dir():
        pairs = []
        for gt_file in sorted(gt_path.glob("*.txt")):
            stem = gt_file.stem  # floor_X_YYYY-MM-DD_run_Z
            # Try flat layout first: pred_dir/floor_X_YYYY-MM-DD_run_Z.txt
            flat = pred_path / f"{stem}.txt"
            if flat.exists():
                pairs.append((gt_file, flat))
                continue
            # Try structured layout: pred_dir/floor_X/YYYY-MM-DD_run_Z.txt
            # stem format: floor_X_YYYY-MM-DD_run_Z  (floor may be floor_1, floor_UG1, …)
            # Split on the first date-like segment (YYYY-)
            m = re.match(r'^(floor_\w+?)_((\d{4}-.+))$', stem)
            if m:
                floor, date_run = m.group(1), m.group(2)
                structured = pred_path / floor / f"{date_run}.txt"
                if structured.exists():
                    pairs.append((gt_file, structured))
                    continue
            print(f"  [warn] No prediction found for {gt_file.name} — skipping")
        return pairs

    raise ValueError(f"Cannot pair GT={gt_path} with pred={pred_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Hilti x Trimble SLAM Challenge 2026 — local evaluator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--gt',   required=True,
                        help="GT trajectory file or directory of GT .txt files")
    parser.add_argument('--pred', required=True,
                        help="Predicted trajectory file or directory of .txt files")
    parser.add_argument('--task', choices=['slam', 'localization'], default='slam',
                        help="Evaluation task (default: slam)")
    parser.add_argument('--plot', action='store_true',
                        help="Show/save error plots for each run")
    parser.add_argument('--plot-dir', default=None,
                        help="Directory to save plots (default: display interactively)")
    parser.add_argument('--results-dir', default=None,
                        help="Directory to save per-run result .txt files")
    parser.add_argument('--best-dir', default=None,
                        help="Directory to accumulate best per-run results (TUM + plot + report)")
    parser.add_argument('--best-aligned', action='store_true', default=True,
                        help="Save Sim3-aligned trajectories to best-dir (default: on)")
    parser.add_argument('--best-raw', dest='best_aligned', action='store_false',
                        help="Save raw (unaligned) trajectories to best-dir")
    parser.add_argument('--max-ts-diff', type=float, default=MAX_TIMESTAMP_DIFF,
                        help=f"Max timestamp diff for matching (default: {MAX_TIMESTAMP_DIFF} s)")
    args = parser.parse_args()

    try:
        pairs = collect_pairs(args.gt, args.pred)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    if not pairs:
        print("No GT/prediction pairs found — nothing to evaluate.")
        sys.exit(0)

    print(f"\nHilti x Trimble SLAM Challenge 2026 — Evaluation")
    print(f"Task  : {args.task.upper()}")
    print(f"Runs  : {len(pairs)}")
    print(f"Score : a={SCORE_A}, c={SCORE_C:.6f}")
    print(f"        (error=0m → 100 pts, error=10m → 1 pt)")

    results = []
    for gt_file, pred_file in pairs:
        r = evaluate_run(gt_file, pred_file, task=args.task)
        print_run_report(r)
        if args.plot or args.plot_dir:
            plot_run(r, out_dir=args.plot_dir)
        if args.results_dir:
            write_run_report(r, args.results_dir)
        if args.best_dir:
            save_best_result(r, args.best_dir, plot_dir=args.plot_dir,
                             aligned=args.best_aligned)
        results.append(r)

    if len(results) > 1:
        print_summary(results)


if __name__ == '__main__':
    main()
