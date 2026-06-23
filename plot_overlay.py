"""
Plot all available config results for a given run overlaid on one set of axes.
Non-diverged trajectories are Umeyama-aligned to a common reference (the first
non-diverged non-upsampled result). Diverged ones are shown faded in the background.

Usage:
    python3 plot_overlay.py <run_name> <out_png> [--results-dir DIR] [--gt-dir DIR]

  e.g. python3 plot_overlay.py \\
           floor_2_2025-05-05_run_1 /tmp/out.png \\
           --results-dir results/ \\
           --gt-dir groundtruth/
"""

import argparse
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path

_ap = argparse.ArgumentParser(add_help=False)
_ap.add_argument('run_name', nargs='?', default=None)
_ap.add_argument('out_png',  nargs='?', default=None)
_ap.add_argument('--results-dir', default=None)
_ap.add_argument('--gt-dir',      default=None)
_args, _ = _ap.parse_known_args()

RESULTS = Path(_args.results_dir) if _args.results_dir else Path("results")
GT_DIR  = Path(_args.gt_dir)      if _args.gt_dir      else Path("groundtruth")

CONFIGS = [
    ("baseline (1×)",        "openvins/{floor}/{date}/{run}/{name}.txt",           False),
    ("dynA (1×)",            "openvins_dynA/{floor}/{date}/{run}/{name}.txt",       False),
    ("imuB (1×)",            "openvins_imuB/{floor}/{date}/{run}/{name}.txt",       False),
    ("expC (1×)",            "openvins_expC/{floor}/{date}/{run}/{name}.txt",       False),
    ("fastinit (1×)",        "openvins_fastinit/{floor}/{date}/{run}/{name}.txt",   False),
    ("baseline (0.5×)",      "openvins_slowrate_baseline/{name}/{name}.txt",        True),
    ("dynA (0.5×)",          "openvins_slowrate_dynA/{name}/{name}.txt",            True),
    ("imuB (0.5×)",          "openvins_slowrate_imuB/{name}/{name}.txt",            True),
    ("expC (0.5×)",          "openvins_slowrate_expC/{name}/{name}.txt",            True),
    ("fastinit (0.5×)",      "openvins_slowrate_fastinit/{name}/{name}.txt",        True),
    ("imuB_dynfastinit (0.5×)", "openvins_slowrate_imuB_dynfastinit/{name}/{name}.txt", True),
    ("submission",           "openvins_submission/{name}.txt",                      None),
]

DIVERGE_THRESH = 500  # metres XY

COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
    "#aec7e8", "#ffbb78",
]


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


def match_by_timestamp(src_tum, dst_tum, tol=0.05):
    """Return matched (src_xyz, dst_xyz) pairs within tol seconds."""
    ts, td = src_tum[:, 0], dst_tum[:, 0]
    si, di = [], []
    for i, t in enumerate(ts):
        j = np.searchsorted(td, t)
        for k in [j - 1, j]:
            if 0 <= k < len(td) and abs(td[k] - t) < tol:
                si.append(i); di.append(k)
                break
    return src_tum[si, 1:4], dst_tum[di, 1:4]


def umeyama_R_t(src_pts, dst_pts):
    """SE(3) rigid body: align src_pts → dst_pts. Returns R, t."""
    mu_s = src_pts.mean(0); mu_d = dst_pts.mean(0)
    H = (src_pts - mu_s).T @ (dst_pts - mu_d)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.linalg.det(Vt.T @ U.T)])
    R = Vt.T @ D @ U.T
    t = mu_d - R @ mu_s
    return R, t


def align_to_ref(src_tum, ref_tum):
    """Align src trajectory (TUM array) to ref using timestamp-matched Umeyama.
    Falls back to index-subsampled alignment if < 10 timestamp matches."""
    s_xyz, r_xyz = match_by_timestamp(src_tum, ref_tum)
    if len(s_xyz) < 10:
        # fallback: subsample by index
        n = min(len(src_tum), len(ref_tum), 500)
        si = np.linspace(0, len(src_tum) - 1, n, dtype=int)
        ri = np.linspace(0, len(ref_tum) - 1, n, dtype=int)
        s_xyz, r_xyz = src_tum[si, 1:4], ref_tum[ri, 1:4]
    R, t = umeyama_R_t(s_xyz, r_xyz)
    return src_tum[:, 1:4] @ R.T + t


def main():
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)

    run_name = sys.argv[1]
    out_png  = sys.argv[2]
    floor, date, run = run_name_parts(run_name)

    gt_path = GT_DIR / f"{run_name}.txt"
    gt      = load(gt_path) if gt_path.exists() else None

    results = []
    for label, template, slowrate in CONFIGS:
        path = RESULTS / template.format(floor=floor, date=date, run=run, name=run_name)
        d = load(path)
        if d is None:
            continue
        if any(np.array_equal(r["d"], d) for r in results):
            continue
        sx, sy, sz = spread(d)
        diverged = sx > DIVERGE_THRESH or sy > DIVERGE_THRESH
        results.append({
            "label": label, "slowrate": slowrate,
            "d": d, "sx": sx, "sy": sy, "sz": sz,
            "n": len(d), "diverged": diverged,
        })

    if not results:
        print("No results found for", run_name); sys.exit(1)

    # Reference: first non-diverged, non-submission result
    ref = next((r for r in results if not r["diverged"] and r["slowrate"] is not None), None)
    if ref is None:
        ref = results[0]
    ref_xyz = ref["d"][:, 1:4]

    fig, ax = plt.subplots(figsize=(10, 9))
    fig.suptitle(run_name.replace("_", "/") + ("  [GT available]" if gt is not None else ""),
                 fontsize=13, fontweight="bold")

    color_idx = 0
    for r in results:
        xyz = r["d"][:, 1:4]

        if r["diverged"]:
            r["aligned"] = xyz[:, :2]
            continue

        # Align to reference using timestamp-matched Umeyama
        if r is ref:
            aligned = xyz
        else:
            aligned = align_to_ref(r["d"], ref["d"])
        r["aligned"] = aligned[:, :2]

        color = COLORS[color_idx % len(COLORS)]
        color_idx += 1
        sr_tag = {True: " 0.5×", False: " 1×", None: ""}[r["slowrate"]]
        label = f"{r['label']}  XY {r['sx']:.0f}×{r['sy']:.0f} m  Z {r['sz']:.1f} m  n={r['n']}"
        ax.plot(aligned[:, 0], aligned[:, 1], lw=1.2, alpha=0.85,
                color=color, label=label, zorder=3)
        ax.plot(aligned[0, 0], aligned[0, 1], "o", ms=5, color=color, zorder=5)
        ax.plot(aligned[-1, 0], aligned[-1, 1], "s", ms=5, color=color, zorder=5)

    if gt is not None:
        ax.plot(gt[:, 1], gt[:, 2], lw=2, color="black", ls="--",
                alpha=0.7, label="GT", zorder=4)

    # Set axis limits from non-diverged trajectories only, with 20% padding
    good = [r for r in results if not r["diverged"]]
    if good:
        all_xy = np.vstack([r["aligned"] for r in good])
        xpad = (all_xy[:, 0].max() - all_xy[:, 0].min()) * 0.2 + 1
        ypad = (all_xy[:, 1].max() - all_xy[:, 1].min()) * 0.2 + 1
        ax.set_xlim(all_xy[:, 0].min() - xpad, all_xy[:, 0].max() + xpad)
        ax.set_ylim(all_xy[:, 1].min() - ypad, all_xy[:, 1].max() + ypad)

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, lw=0.4, alpha=0.4)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.legend(fontsize=7.5, loc="best", framealpha=0.85)

    plt.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"Saved → {out_png}")

    print(f"\nGT: {'yes' if gt is not None else 'no'}")
    print(f"\n{'Config':<28} {'SR':<5} {'n':>6}  {'XY (m)':>18}  {'Z':>7}  Status")
    print("-" * 80)
    for r in results:
        sr     = {True: "0.5×", False: "1×", None: "—"}[r["slowrate"]]
        status = "DIVERGED" if r["diverged"] else "OK"
        print(f"{r['label']:<28} {sr:<5} {r['n']:>6}  "
              f"{r['sx']:>8.0f}×{r['sy']:<7.0f}  {r['sz']:>5.1f} m  {status}")


if __name__ == "__main__":
    main()
