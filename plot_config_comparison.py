"""
Plot all available config results for a given run side by side.
When a GT file exists the GT trajectory is overlaid and ATE (RMSE) is computed
via Umeyama SE(3) alignment on matched timestamps.

Usage:
    python3 plot_config_comparison.py <run_name> <out_png> [--results-dir DIR] [--gt-dir DIR]

  e.g. python3 plot_config_comparison.py \\
           floor_2_2025-05-05_run_1 /tmp/out.png \\
           --results-dir results/ \\
           --gt-dir groundtruth/

RESULTS_DIR should contain per-config subdirectories matching the CONFIGS table below.
"""

import argparse
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.spatial.transform import Rotation

_ap = argparse.ArgumentParser(add_help=False)
_ap.add_argument('run_name', nargs='?', default=None)
_ap.add_argument('out_png',  nargs='?', default=None)
_ap.add_argument('--results-dir', default=None)
_ap.add_argument('--gt-dir',      default=None)
_args, _ = _ap.parse_known_args()

RESULTS  = Path(_args.results_dir) if _args.results_dir else Path("results")
GT_DIR   = Path(_args.gt_dir)      if _args.gt_dir      else Path("groundtruth")

# (label, path template, slowrate flag)
CONFIGS = [
    ("baseline\n(1×)",        "openvins/{floor}/{date}/{run}/{name}.txt",              False),
    ("dynA\n(1×)",             "openvins_dynA/{floor}/{date}/{run}/{name}.txt",         False),
    ("dynA\n(1×, flat)",       "openvins_dynA/{name}/{name}.txt",                       False),
    ("imuB\n(1×)",             "openvins_imuB/{floor}/{date}/{run}/{name}.txt",         False),
    ("imuB\n(1×, flat)",       "openvins_imuB/{name}/{name}.txt",                       False),
    ("baseline\nslowrate",      "openvins_slowrate_baseline/{name}/{name}.txt",          True),
    ("baseline\nslowrate (old)","openvins_slowrate/{name}/{name}.txt",                   True),
    ("expC\nslowrate",         "openvins_slowrate_expC/{name}/{name}.txt",              True),
    ("dynA\nslowrate",         "openvins_slowrate_dynA/{name}/{name}.txt",              True),
    ("fastinit\nslowrate",     "openvins_slowrate_fastinit/{name}/{name}.txt",          True),
    ("imuB\nslowrate",         "openvins_slowrate_imuB/{name}/{name}.txt",              True),
    ("imuB_dynfastinit\nslowrate", "openvins_slowrate_imuB_dynfastinit/{name}/{name}.txt", True),
    ("vi_ready\n(slowrate rerun,\nno BA yet)",  "openvins_vi_ready/{name}.txt",          True),
    ("submission\n(upsampled)","openvins_submission/{name}.txt",                        None),
]

DIVERGE_THRESH_XY = 500   # metres
MATCH_TOL         = 0.05  # seconds — GT timestamp match tolerance


# ── helpers ──────────────────────────────────────────────────────────────────

def load(path):
    try:
        d = np.loadtxt(path, comments="#")
        if d.ndim != 2 or d.shape[1] < 8 or len(d) < 10:
            return None
        return d
    except Exception:
        return None


def spread(d):
    x, y, z = d[:, 1], d[:, 2], d[:, 3]
    return x.max() - x.min(), y.max() - y.min(), z.max() - z.min()


def run_name_parts(name):
    parts = name.split("_")
    floor = f"floor_{parts[1]}"
    rest  = parts[2:]
    date  = rest[0]
    run   = "_".join(rest[1:])
    return floor, date, run


def match_timestamps(pred, gt, tol=MATCH_TOL):
    """Return (pred_xyz, gt_xyz) for timestamps matched within tol."""
    t_p = pred[:, 0];  t_g = gt[:, 0]
    idx_p, idx_g = [], []
    for i, t in enumerate(t_p):
        j = np.searchsorted(t_g, t)
        for k in [j - 1, j]:
            if 0 <= k < len(t_g) and abs(t_g[k] - t) < tol:
                idx_p.append(i); idx_g.append(k)
                break
    return pred[idx_p, 1:4], gt[idx_g, 1:4]


def umeyama_align(src, dst):
    """Align src → dst with SE(3) (no scale). Returns R, t, aligned_src."""
    mu_s = src.mean(0); mu_d = dst.mean(0)
    ss = src - mu_s;    ds = dst - mu_d
    H  = ss.T @ ds
    U, _, Vt = np.linalg.svd(H)
    d  = np.linalg.det(Vt.T @ U.T)
    D  = np.diag([1, 1, d])
    R  = Vt.T @ D @ U.T
    t  = mu_d - R @ mu_s
    return R, t, (src @ R.T + t)


def compute_ate(pred, gt):
    """ATE RMSE in metres after SE(3) alignment."""
    p_xyz, g_xyz = match_timestamps(pred, gt)
    if len(p_xyz) < 10:
        return None, None
    _, _, aligned = umeyama_align(p_xyz, g_xyz)
    errs = np.linalg.norm(aligned - g_xyz, axis=1)
    return float(np.sqrt(np.mean(errs**2))), len(p_xyz)


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)

    run_name = sys.argv[1]
    out_png  = sys.argv[2]
    floor, date, run = run_name_parts(run_name)

    gt_path = GT_DIR / f"{run_name}.txt"
    gt      = load(gt_path) if gt_path.exists() else None
    has_gt  = gt is not None

    results = []
    for label, template, slowrate in CONFIGS:
        path = RESULTS / template.format(floor=floor, date=date, run=run, name=run_name)
        d    = load(path)
        if d is None:
            continue
        # deduplicate: skip if same file content as an earlier entry
        if any(np.array_equal(r["d"], d) for r in results):
            continue
        sx, sy, sz = spread(d)
        diverged   = sx > DIVERGE_THRESH_XY or sy > DIVERGE_THRESH_XY
        ate, n_matched = (compute_ate(d, gt) if has_gt and not diverged else (None, None))
        results.append({
            "label":     label,
            "slowrate":  slowrate,
            "path":      path,
            "d":         d,
            "sx": sx, "sy": sy, "sz": sz,
            "n":         len(d),
            "diverged":  diverged,
            "ate":       ate,
            "n_matched": n_matched,
        })

    if not results:
        print("No results found for", run_name); sys.exit(1)

    n_plots = len(results)
    ncols   = min(n_plots, 4)
    nrows   = (n_plots + ncols - 1) // ncols

    fig, axes = plt.subplots(nrows, ncols,
                             figsize=(4.8 * ncols, 4.8 * nrows),
                             squeeze=False)
    fig.suptitle(run_name.replace("_", "/") + ("  [GT available]" if has_gt else ""),
                 fontsize=13, fontweight="bold", y=1.01)

    for i, r in enumerate(results):
        ax  = axes[i // ncols][i % ncols]
        d   = r["d"]

        if has_gt and not r["diverged"]:
            # align prediction into GT frame, then plot both
            p_xyz, g_xyz = match_timestamps(d, gt)
            if len(p_xyz) >= 10:
                R, t, _ = umeyama_align(p_xyz, g_xyz)
                all_pred_aligned = d[:, 1:4] @ R.T + t
            else:
                all_pred_aligned = d[:, 1:4]
            x, y = all_pred_aligned[:, 0], all_pred_aligned[:, 1]
            gx, gy = gt[:, 1], gt[:, 2]
            ax.plot(gx, gy, lw=1.5, color="black", alpha=0.55,
                    ls="--", label="GT", zorder=1)
        else:
            x, y = d[:, 1], d[:, 2]

        color = "#d62728" if r["diverged"] else "#1f77b4"
        ax.plot(x, y, lw=0.9, color=color, alpha=0.85, label="pred", zorder=2)
        ax.plot(x[0], y[0], "go", ms=5, zorder=5)
        ax.plot(x[-1], y[-1], "rs", ms=5, zorder=5)

        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, lw=0.4, alpha=0.4)

        sr_tag = {True: "  [0.5×]", False: "  [1×]", None: ""}[r["slowrate"]]
        status = "DIVERGED" if r["diverged"] else "OK"
        ate_str = ""
        if r["ate"] is not None:
            ate_str = f"\nATE {r['ate']:.3f} m  ({r['n_matched']} matched)"

        title_color = "#d62728" if r["diverged"] else "#2ca02c"
        ax.set_title(
            f"{r['label']}{sr_tag}\n"
            f"XY {r['sx']:.0f}×{r['sy']:.0f} m   Z {r['sz']:.1f} m\n"
            f"n={r['n']}  [{status}]{ate_str}",
            fontsize=8.5, color=title_color, pad=4,
        )
        ax.set_xlabel("x (m)", fontsize=7)
        ax.set_ylabel("y (m)", fontsize=7)
        ax.tick_params(labelsize=7)
        if has_gt and not r["diverged"]:
            ax.legend(fontsize=6, loc="best")

    for j in range(len(results), nrows * ncols):
        axes[j // ncols][j % ncols].set_visible(False)

    plt.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"Saved → {out_png}")

    has_gt_str = "yes" if has_gt else "no"
    print(f"\nGT available: {has_gt_str}")
    print(f"\n{'Config':<28} {'SR':<4} {'n':>6}  {'XY (m)':>18}  {'Z':>7}  {'ATE':>9}  Status")
    print("-" * 88)
    for r in results:
        sr    = {True: "0.5×", False: "1×", None: "—"}[r["slowrate"]]
        label = r["label"].replace("\n", " ")
        ate   = f"{r['ate']:.3f} m" if r["ate"] is not None else "—"
        status = "DIVERGED" if r["diverged"] else "OK"
        print(f"{label:<28} {sr:<4} {r['n']:>6}  {r['sx']:>8.0f}×{r['sy']:<7.0f}  {r['sz']:>5.1f} m  {ate:>9}  {status}")


if __name__ == "__main__":
    main()
