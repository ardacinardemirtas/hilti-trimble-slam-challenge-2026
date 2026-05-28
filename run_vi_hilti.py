#!/usr/bin/env python3
"""
VI optimization for Hilti OpenVINS reconstructions (no COLMAP database needed).

Loads an openvins_vi_ready _colmap/ dir (produced by features_to_colmap.py),
integrates IMU measurements between consecutive frames, and runs Ceres BA
with IMU preintegration factors to refine poses, scale, gravity, and biases.

Usage:
    python3 run_vi_hilti.py \
        --colmap   results/openvins_vi_ready/floor_1_2025-05-05_run_1_colmap/ \
        --imu      results/openvins_vi_ready/floor_1_2025-05-05_run_1_imu.npy \
        --calib    config/hilti_openvins/kalibr_imucam_chain.yaml \
        --output   results/openvins_vi_ready/floor_1_2025-05-05_run_1_vi/
"""

import argparse
import math
import os
import sys
from pathlib import Path
from typing import Any

import numpy as np

# ── pycolmap / pyceres from colmap_imu build ──────────────────────────────────
_COLMAP_IMU = Path(__file__).parent.parent / "colmap_imu"
sys.path.insert(0, str(_COLMAP_IMU / "python" / "build"))
sys.path.insert(0, str(_COLMAP_IMU / "python"))
sys.path.insert(0, str(_COLMAP_IMU / ".venv" / "lib" / "python3.11" / "site-packages"))

import pycolmap
import pycolmap.cost_functions
import pyceres

if not hasattr(pyceres, "ProductManifold"):
    from pycolmap._core.pyceres import ProductManifold as _PM
    pyceres.ProductManifold = _PM
    del _PM


# ── Calibration loading ───────────────────────────────────────────────────────

def load_imu_from_cam(calib_yaml: str) -> pycolmap.Rigid3d:
    """Return imu_from_cam (inverse of cam0 T_cam_imu from Kalibr YAML).

    Hardcoded from config/hilti_openvins/kalibr_imucam_chain.yaml — same
    sensor unit across all Hilti runs, so this is stable.
    """
    # cam0 T_cam_imu (cam_from_imu), from kalibr_imucam_chain.yaml
    T_cam_imu = np.array([
        [ 0.017214474772216132, -0.0008034642120502422, -0.9998514971252359,  0.020670851120764513],
        [ 0.9998263174555488,   -0.007128426214556394,   0.017219769539067287, 0.015539085669546057],
        [-0.007141203091335369, -0.9999742696614562,     0.0006806125511055194,-0.01575188948566258],
        [ 0.0,                   0.0,                    0.0,                   1.0],
    ])
    T_imu_cam = np.linalg.inv(T_cam_imu)
    return pycolmap.Rigid3d(T_imu_cam[:3, :])


# ── IMU calibration (Bosch BMI085 — Hilti sensor) ─────────────────────────────
def hilti_imu_calib() -> pycolmap.ImuCalibration:
    c = pycolmap.ImuCalibration()
    c.imu_rate = 1000.0                  # Hz (bag records at ~995 Hz)
    c.gravity_magnitude = 9.81
    c.gyro_noise_density = 1.7e-4        # rad/s/√Hz  (BMI085 spec)
    c.accel_noise_density = 2.0e-3       # m/s²/√Hz
    c.bias_gyro_random_walk_sigma = 1.9e-5
    c.bias_accel_random_walk_sigma = 3.0e-4
    return c


# ── Inlined from vi_optimization.py (avoids `import wget` at module level) ────

class ImuReintegrationCallback(pyceres.IterationCallback):
    """Reintegrates IMU data when optimized biases drift past linearization."""

    def __init__(self, options: pycolmap.ImuReintegrationOptions) -> None:
        pyceres.IterationCallback.__init__(self)
        self.options = options
        self.edges: list[tuple[
            pycolmap.ImuPreintegrator,
            pycolmap.PreintegratedImuData,
            pycolmap.ImuState,
        ]] = []

    def add_edge(
        self,
        integrator: pycolmap.ImuPreintegrator,
        data: pycolmap.PreintegratedImuData,
        imu_state: pycolmap.ImuState,
    ) -> None:
        self.edges.append((integrator, data, imu_state))

    def _should_reintegrate(
        self, data: pycolmap.PreintegratedImuData, biases: np.ndarray
    ) -> bool:
        diff = biases - data.biases
        delta_t = data.delta_t
        if np.linalg.norm(diff[:3]) * delta_t > self.options.reintegrate_angle_norm_thres:
            return True
        return bool(np.linalg.norm(diff[3:]) * delta_t > self.options.reintegrate_vel_norm_thres)

    def __call__(self, summary: pyceres.IterationSummary) -> pyceres.CallbackReturnType:
        for integrator, data, imu_state in self.edges:
            biases = imu_state.params[3:9]
            if self._should_reintegrate(data, biases):
                integrator.reintegrate(biases)
                integrator.update(data)
        return pyceres.CallbackReturnType.SOLVER_CONTINUE


def add_imu_residuals(
    prob: pyceres.Problem,
    reconstruction: pycolmap.Reconstruction,
    imu_data: dict[int, pycolmap.PreintegratedImuData],
    variables: dict[str, Any],
    optimize_scale: bool = True,
    optimize_gravity: bool = True,
    optimize_imu_from_cam: bool = True,
    optimize_bias: bool = True,
) -> pyceres.Problem:
    loss = pyceres.TrivialLoss()
    for image_id, integrated_m in imu_data.items():
        image_i = reconstruction.images[image_id]
        image_j = reconstruction.images[image_id + 1]
        assert image_i.frame is not None
        assert image_i.frame.rig is not None
        assert len(image_i.frame.rig.non_ref_sensors) == 0
        assert image_j.frame is not None
        assert image_j.frame.rig is not None
        assert len(image_j.frame.rig.non_ref_sensors) == 0
        i_from_world = image_i.frame.rig_from_world
        j_from_world = image_j.frame.rig_from_world
        assert i_from_world is not None
        assert j_from_world is not None

        prob.add_residual_block(
            pycolmap.cost_functions.AnalyticalVisualCentricImuPreintegrationCost(integrated_m),
            loss,
            [
                variables["log_scale"],
                variables["gravity"],
                variables["imu_from_cam"].params,
                i_from_world.params,
                variables["imu_states"][image_id].params,
                j_from_world.params,
                variables["imu_states"][image_id + 1].params,
            ],
        )
    prob.set_manifold(variables["gravity"], pyceres.SphereManifold(3))
    prob.set_manifold(
        variables["imu_from_cam"].params,
        pyceres.ProductManifold(
            pyceres.EigenQuaternionManifold(), pyceres.EuclideanManifold(3)
        ),
    )
    if not optimize_scale:
        prob.set_parameter_block_constant(variables["log_scale"])
    if not optimize_gravity:
        prob.set_parameter_block_constant(variables["gravity"])
    if not optimize_imu_from_cam:
        prob.set_parameter_block_constant(variables["imu_from_cam"].params)
    if not optimize_bias:
        constant_idxs = np.arange(3, 9)
        for image_id in variables["imu_states"]:
            prob.set_manifold(
                variables["imu_states"][image_id].params,
                pyceres.SubsetManifold(9, constant_idxs),
            )
    return prob


def solve_bundle_adjustment(
    reconstruction: pycolmap.Reconstruction,
    ba_options: pycolmap.BundleAdjustmentOptions,
    ba_config: pycolmap.BundleAdjustmentConfig,
    integrators: dict[int, pycolmap.ImuPreintegrator],
    imu_data: dict[int, pycolmap.PreintegratedImuData],
    variables: dict[str, Any],
    optimize_imu_from_cam: bool = False,
) -> pyceres.SolverSummary:
    bundle_adjuster = pycolmap.create_default_ceres_bundle_adjuster(
        ba_options, ba_config, reconstruction
    )
    problem = bundle_adjuster.problem
    add_imu_residuals(problem, reconstruction, imu_data, variables,
                      optimize_imu_from_cam=optimize_imu_from_cam)
    solver_options = pyceres.SolverOptions(
        ba_options.ceres.create_solver_options(ba_config, problem)
    )
    solver_options.minimizer_progress_to_stdout = True
    callback = ImuReintegrationCallback(pycolmap.ImuReintegrationOptions())
    for image_id in integrators:
        callback.add_edge(
            integrators[image_id],
            imu_data[image_id],
            variables["imu_states"][image_id],
        )
    solver_options.callbacks.append(callback)
    solver_options.update_state_every_iteration = True
    summary = pyceres.SolverSummary()
    pyceres.solve(solver_options, problem, summary)
    print(summary.BriefReport())
    return summary


# ── Pipeline ──────────────────────────────────────────────────────────────────

def load_imu(path: str) -> pycolmap.ImuMeasurements:
    raw = np.load(path, allow_pickle=True)
    ms = pycolmap.ImuMeasurements()
    for row in raw:
        ms.insert(pycolmap.ImuMeasurement(
            timestamp=int(row[0]),
            accel=np.array(row[1:4]),
            gyro=np.array(row[4:7]),
        ))
    return ms


def preintegrate(
    recon: pycolmap.Reconstruction,
    image_timestamps: dict[int, int],
    imu_measurements: pycolmap.ImuMeasurements,
    imu_calib: pycolmap.ImuCalibration,
) -> tuple[dict, dict]:
    opts = pycolmap.ImuPreintegrationOptions()
    image_ids = sorted(recon.images.keys())
    integrators: dict[int, pycolmap.ImuPreintegrator] = {}
    preintegrated: dict[int, pycolmap.PreintegratedImuData] = {}
    skipped = 0
    for i in range(len(image_ids) - 1):
        id_i, id_j = image_ids[i], image_ids[i + 1]
        t1, t2 = image_timestamps[id_i], image_timestamps[id_j]
        ms = imu_measurements.extract_measurements_contain_edge(t1, t2)
        if len(ms) == 0:
            skipped += 1
            continue
        integrators[id_i] = pycolmap.ImuPreintegrator(opts, imu_calib, t1, t2)
        integrators[id_i].feed_imu(ms)
        preintegrated[id_i] = integrators[id_i].extract()
    print(f"Preintegrated {len(integrators)} intervals, skipped {skipped}")
    return integrators, preintegrated


def init_variables(
    recon: pycolmap.Reconstruction,
    image_timestamps: dict[int, int],
    imu_from_cam: pycolmap.Rigid3d | None = None,
) -> dict[str, Any]:
    image_ids = sorted(recon.images.keys())
    variables: dict[str, Any] = {
        "imu_from_cam": imu_from_cam if imu_from_cam is not None else pycolmap.Rigid3d(),
        "gravity":      np.array([0.0, 0.0, -1.0]),
        "log_scale":    np.array([0.0]),
        "imu_states":   {},
    }
    for i in range(len(image_ids) - 1):
        id_i = image_ids[i]
        id_j = image_ids[i + 1]
        t1 = image_timestamps[id_i]
        t2 = image_timestamps[id_j]
        dt = (t2 - t1) / 1e9
        if dt <= 0:
            dt = 1.0 / 30.0
        pi = recon.images[id_i].cam_from_world().inverse().translation
        pj = recon.images[id_j].cam_from_world().inverse().translation
        variables["imu_states"][id_i] = pycolmap.ImuState()
        variables["imu_states"][id_i].velocity = (pj - pi) / dt
    last_id = image_ids[-1]
    variables["imu_states"][last_id] = pycolmap.ImuState()
    variables["imu_states"][last_id].velocity = np.zeros(3)
    return variables


def run_ba_with_imu(
    recon: pycolmap.Reconstruction,
    integrators: dict,
    preintegrated: dict,
    variables: dict,
    n_iterations: int = 3,
    optimize_imu_from_cam: bool = False,
) -> None:
    ba_options = pycolmap.BundleAdjustmentOptions()
    ba_options.print_summary = True

    for iteration in range(n_iterations):
        print(f"\n=== BA+IMU iteration {iteration + 1}/{n_iterations} ===")
        ba_config = pycolmap.BundleAdjustmentConfig()
        for image_id in recon.reg_image_ids():
            ba_config.add_image(image_id)
        summary = solve_bundle_adjustment(
            recon, ba_options, ba_config,
            integrators, preintegrated, variables,
            optimize_imu_from_cam=optimize_imu_from_cam,
        )
        print(f"  {summary.BriefReport()}")

    scale = float(np.exp(variables["log_scale"][0]))
    print(f"\nOptimized scale: exp({variables['log_scale'][0]:.4f}) = {scale:.4f}")
    print(f"Optimized gravity: {variables['gravity']}")
    print(f"Optimized imu_from_cam: {variables['imu_from_cam']}")


def export_tum(
    recon: pycolmap.Reconstruction,
    image_timestamps: dict[int, int],
    out_path: str,
) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for image_id in sorted(recon.images.keys()):
            img = recon.images[image_id]
            ts_ns = image_timestamps[image_id]
            ts_s = ts_ns * 1e-9          # keeps +10000 s offset
            # cam_from_world → world_from_cam
            cfw = img.cam_from_world()
            R = cfw.rotation.matrix().T
            t = -R @ cfw.translation
            # rotation matrix → quaternion (xyzw)
            tr = R[0, 0] + R[1, 1] + R[2, 2]
            if tr > 0:
                s = 0.5 / math.sqrt(tr + 1.0)
                qw = 0.25 / s
                qx, qy, qz = (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s
            elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                s = 2 * math.sqrt(1 + R[0,0] - R[1,1] - R[2,2])
                qw = (R[2,1]-R[1,2])/s; qx = 0.25*s
                qy = (R[0,1]+R[1,0])/s; qz = (R[0,2]+R[2,0])/s
            elif R[1,1] > R[2,2]:
                s = 2 * math.sqrt(1 + R[1,1] - R[0,0] - R[2,2])
                qw = (R[0,2]-R[2,0])/s; qx = (R[0,1]+R[1,0])/s
                qy = 0.25*s; qz = (R[1,2]+R[2,1])/s
            else:
                s = 2 * math.sqrt(1 + R[2,2] - R[0,0] - R[1,1])
                qw = (R[1,0]-R[0,1])/s; qx = (R[0,2]+R[2,0])/s
                qy = (R[1,2]+R[2,1])/s; qz = 0.25*s
            f.write(f"{ts_s:.9f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
                    f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
    print(f"Exported {len(recon.images)} poses → {out_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--colmap",  required=True, help="Path to _colmap/ dir from features_to_colmap.py")
    ap.add_argument("--imu",     required=True, help="IMU numpy array (N,7): [ts_ns ax ay az gx gy gz]")
    ap.add_argument("--calib",   required=True, help="Kalibr imucam YAML (for cam0 T_cam_imu)")
    ap.add_argument("--output",  required=True, help="Output dir (refined COLMAP + TUM)")
    ap.add_argument("--tum-out", default=None,  help="TUM output path (default: output/refined.txt)")
    ap.add_argument("--iterations", type=int, default=3, help="BA+IMU iterations (default: 3)")
    ap.add_argument("--optimize-extrinsic", action="store_true",
                    help="Optimize imu_from_cam during BA (default: frozen at Kalibr value)")
    args = ap.parse_args()

    tum_out = args.tum_out or os.path.join(args.output, "refined.txt")

    print(f"Loading reconstruction: {args.colmap}")
    recon = pycolmap.Reconstruction(args.colmap)
    print(f"  {len(recon.images)} images, {len(recon.points3D)} 3D points")

    ts_path = os.path.join(args.colmap, "image_timestamps.npy")
    image_timestamps: dict[int, int] = np.load(ts_path, allow_pickle=True).item()

    print(f"Loading imu_from_cam from: {args.calib}")
    imu_from_cam = load_imu_from_cam(args.calib)
    print(f"  imu_from_cam: {imu_from_cam}")
    optimize_extrinsic = args.optimize_extrinsic
    print(f"  optimize_imu_from_cam: {optimize_extrinsic}")

    print(f"Loading IMU: {args.imu}")
    imu_calib = hilti_imu_calib()
    imu_measurements = load_imu(args.imu)
    print(f"  {len(imu_measurements)} IMU measurements loaded")

    integrators, preintegrated = preintegrate(
        recon, image_timestamps, imu_measurements, imu_calib)

    variables = init_variables(recon, image_timestamps, imu_from_cam=imu_from_cam)

    run_ba_with_imu(recon, integrators, preintegrated, variables, args.iterations,
                    optimize_imu_from_cam=optimize_extrinsic)

    os.makedirs(args.output, exist_ok=True)
    recon.write(args.output)
    export_tum(recon, image_timestamps, tum_out)
    print(f"\nDone. Refined COLMAP → {args.output}")
    print(f"      TUM trajectory   → {tum_out}")


if __name__ == "__main__":
    main()
