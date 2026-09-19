import os
import glob
import sys
import argparse
import csv
import json
import hashlib
import importlib
import platform
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
import pandas as pd
import numpy as np
from tqdm import tqdm
import trimesh
import tempfile
import uuid
import gc
import re
import math
from scipy.spatial import cKDTree
import multiprocessing as mp  # Isolated worker process for enforceable timeouts
import queue as queue_mod
import traceback
import time
import open3d as o3d

# ==========================================
# 0. Environment and dependency checks
# ==========================================
try:
    from evaluate_step_metrics import (
        StepLoader, TopologyEvaluator
    )
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopAbs import TopAbs_SOLID, TopAbs_IN, TopAbs_ON
    from OCC.Core.gp import gp_Pnt
    from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier
    from OCC.Extend.DataExchange import write_stl_file
    from OCC.Core.BRepCheck import BRepCheck_Analyzer
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib
    HAS_OCC = True
except ImportError:
    HAS_OCC = False
    print("="*60)
    print("[Warning] 未检测到 pythonocc 或 deviation_analysis 模块。")
    print("          STEP 解析与 CAD/B-Rep 拓扑检查将被跳过。")
    print("="*60)

# ==========================================
# 1. Data loading
# ==========================================
class DataLoader:
    @staticmethod
    def load_scan_points(path, estimate_missing_normals=False):
        """Load scan points and available normals for scan-to-CAD evaluation."""
        points, normals = None, None

        def _sanitize_normals(n):
            if n is None:
                return None
            n = np.asarray(n, dtype=float)
            if n.ndim != 2 or n.shape[1] != 3:
                return None

            finite_mask = np.isfinite(n).all(axis=1)
            norm = np.linalg.norm(n, axis=1, keepdims=True)

            valid = finite_mask & (norm[:, 0] > 1e-12)
            if not np.any(valid):
                return None

            n_out = np.zeros_like(n, dtype=float)
            n_out[valid] = n[valid] / norm[valid]
            return n_out

        try:
            ext = os.path.splitext(path)[1].lower()

            # 1. PLY/mesh-like files: prefer Open3D for point-cloud normals
            if ext in [".ply", ".pcd", ".xyz", ".xyzn"]:
                try:
                    pcd = o3d.io.read_point_cloud(path)
                    if len(pcd.points) > 0:
                        points = np.asarray(pcd.points, dtype=float)

                        if len(pcd.normals) == len(pcd.points):
                            normals = _sanitize_normals(np.asarray(pcd.normals, dtype=float))

                        if normals is None and estimate_missing_normals:
                            pcd.estimate_normals(
                                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                                    radius=2.0,
                                    max_nn=30
                                )
                            )
                            pcd.normalize_normals()
                            normals = _sanitize_normals(np.asarray(pcd.normals, dtype=float))

                        return points, normals
                except Exception:
                    pass

            # 2. General mesh-like files: fallback to trimesh
            if ext in [".ply", ".obj", ".stl", ".off"]:
                try:
                    mesh = trimesh.load(path, process=False)
                    if hasattr(mesh, "vertices"):
                        points = np.asarray(mesh.vertices, dtype=float)
                        if hasattr(mesh, "vertex_normals"):
                            vn = np.asarray(mesh.vertex_normals, dtype=float)
                            if vn.shape == points.shape:
                                normals = _sanitize_normals(vn)
                        return points, normals
                except Exception:
                    pass

            # 3. Text files: x y z [nx ny nz]
            try:
                data = np.loadtxt(path)
            except Exception:
                try:
                    data = np.loadtxt(path, delimiter=",")
                except Exception:
                    with open(path, "r") as f:
                        lines = f.readlines()
                        start = 0
                        for i, l in enumerate(lines):
                            try:
                                float(l.split()[0])
                                start = i
                                break
                            except Exception:
                                continue
                    data = np.loadtxt(path, skiprows=start)

            if data is not None:
                if data.ndim == 1:
                    data = data.reshape(1, -1)

                points = data[:, :3].astype(float)

                if data.shape[1] >= 6:
                    normals = _sanitize_normals(data[:, 3:6])

                if normals is None and estimate_missing_normals:
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(points)
                    pcd.estimate_normals(
                        search_param=o3d.geometry.KDTreeSearchParamHybrid(
                            radius=2.0,
                            max_nn=30
                        )
                    )
                    pcd.normalize_normals()
                    normals = _sanitize_normals(np.asarray(pcd.normals, dtype=float))

                return points, normals

        except Exception as e:
            print(f"[load_scan_points] failed to load {path}: {e}")

        return None, None

    @staticmethod
    def load_cad_model(path, deflection=0.05):
        """Load a STEP/B-Rep or mesh input using the established evaluation preprocessing."""
        ext = os.path.splitext(path)[1].lower()
        
        if ext in ['.step', '.stp']:
            if not HAS_OCC: return None, None
            temp = None
            try:
                shape = StepLoader.load(path)
                temp = os.path.join(tempfile.gettempdir(), f"eval_{uuid.uuid4().hex}.stl")
                write_stl_file(shape, temp, mode="binary", linear_deflection=deflection)
                mesh = trimesh.load(temp, force='mesh')
                return shape, mesh
            except Exception:
                return None, None
            finally:
                if temp and os.path.exists(temp):
                    try:
                        os.remove(temp)
                    except Exception:
                        pass
            
        elif ext in ['.ply', '.stl', '.obj', '.off']:
            try: 
                mesh = trimesh.load(path, force='mesh')
                # 尝试修复 Point2CAD 网格
                try:
                    trimesh.repair.fix_normals(mesh)
                    trimesh.repair.fix_inversion(mesh)
                    trimesh.repair.fix_winding(mesh)
                except: pass

                if not mesh.is_watertight:
                    try: trimesh.repair.fill_holes(mesh)
                    except: pass
                
                return None, mesh 
            except: return None, None
            
        return None, None

def compute_solid_watertight_ratio(shape):
    """Return valid-solid count, total-solid count, and their CAD/B-Rep validity ratio."""
    if shape is None:
        return 0, 0, 0.0

    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    total_solids = 0
    watertight_solids = 0

    while explorer.More():
        solid = explorer.Current()
        total_solids += 1

        analyzer = BRepCheck_Analyzer(solid)
        if analyzer.IsValid():
            watertight_solids += 1

        explorer.Next()

    if total_solids == 0:
        return 0, 0, 0.0

    return watertight_solids, total_solids, watertight_solids / total_solids


def is_shape_all_valid_solids(shape):
    """Check whether a STEP shape contains at least one solid and all solids are valid."""
    n_valid, n_total, ratio = compute_solid_watertight_ratio(shape)
    return (n_total > 0) and (n_valid == n_total), n_valid, n_total, ratio


def get_shape_or_mesh_bounds(shape, mesh=None):
    """Return the shape bounding box, falling back to mesh bounds when needed."""
    if shape is not None and HAS_OCC:
        try:
            box = Bnd_Box()
            brepbndlib.Add(shape, box)
            xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
            return np.array([xmin, ymin, zmin], dtype=float), np.array([xmax, ymax, zmax], dtype=float)
        except Exception:
            pass

    if mesh is not None and hasattr(mesh, 'bounds'):
        return np.asarray(mesh.bounds[0], dtype=float), np.asarray(mesh.bounds[1], dtype=float)

    return None, None

# ==========================================
# 2. Evaluation engines
# ==========================================

class CadToCadEvaluator:
    def __init__(self, gt_shape, gt_mesh, pred_shape, pred_mesh, num_samples, iou_samples):
        self.gt_shape = gt_shape
        self.gt_mesh = gt_mesh
        self.pred_shape = pred_shape
        self.pred_mesh = pred_mesh
        self.num_samples = num_samples
        self.iou_samples = iou_samples

    def compute(self, thresholds=[0.01, 0.05]):
        if self.gt_mesh is None or self.pred_mesh is None: return None
        if len(self.pred_mesh.vertices) == 0: return None

        metrics = {}
        metrics['SR_GT'] = self.gt_mesh.is_watertight
        metrics['SR_Pred'] = self.pred_mesh.is_watertight
        metrics['n_faces_pred'] = len(self.pred_mesh.faces)
        metrics['n_verts_pred'] = len(self.pred_mesh.vertices)

        pts_gt, idx_gt = trimesh.sample.sample_surface(self.gt_mesh, self.num_samples)
        normals_gt = self.gt_mesh.face_normals[idx_gt]
        
        pts_pred, idx_pred = trimesh.sample.sample_surface(self.pred_mesh, self.num_samples)
        normals_pred = self.pred_mesh.face_normals[idx_pred]
        
        tree_gt, tree_pred = cKDTree(pts_gt), cKDTree(pts_pred)
        
        d_pred_gt, i_pred_gt = tree_gt.query(pts_pred, k=1) 
        d_gt_pred, _ = tree_pred.query(pts_gt, k=1)
        
        metrics['CD'] = (np.mean(d_pred_gt) + np.mean(d_gt_pred)) / 2
        metrics['HD'] = max(np.max(d_pred_gt), np.max(d_gt_pred))
        
        nearest_gt_normals = normals_gt[i_pred_gt]
        metrics['NC'] = np.mean(np.abs(np.sum(normals_pred * nearest_gt_normals, axis=1)))

        for th in thresholds:
            p = np.mean(d_pred_gt < th)
            r = np.mean(d_gt_pred < th)
            f = 2 * p * r / (p + r + 1e-8)
            metrics[f'F-Score@{th}'] = f

        # Mesh IoU is computed only when both meshes support reliable containment.
        if metrics['SR_GT'] and metrics['SR_Pred']:
            mesh_iou = self._compute_iou(self.iou_samples)
            metrics['Mesh_IoU'] = mesh_iou
            metrics['IoU'] = mesh_iou  # 保留旧字段，兼容后续汇总代码
            metrics['Mesh_IoU_Computed'] = np.isfinite(mesh_iou)
        else:
            metrics['Mesh_IoU'] = np.nan
            metrics['IoU'] = np.nan
            metrics['Mesh_IoU_Computed'] = False

        # Full-set Mesh IoU counts non-computable samples as zero.
        metrics['Mesh_IoU_Strict'] = (
            float(metrics['Mesh_IoU'])
            if metrics['Mesh_IoU_Computed']
            else 0.0
        )

        # CAD/B-Rep solid-validity statistics for reference and prediction outputs.
        if self.gt_shape:
            gt_solid_ok, gt_n_water, gt_n_total, gt_ratio = is_shape_all_valid_solids(self.gt_shape)
            metrics['Solid_Watertight_Ratio_GT'] = gt_ratio
            metrics['Num_Solids_GT'] = gt_n_total
        else:
            gt_solid_ok = False
            metrics['Solid_Watertight_Ratio_GT'] = np.nan
            metrics['Num_Solids_GT'] = 0

        if self.pred_shape:
            pred_solid_ok, pred_n_water, pred_n_total, pred_ratio = is_shape_all_valid_solids(self.pred_shape)
            metrics['Solid_Watertight_Ratio_Pred'] = pred_ratio
            metrics['Solid_Watertight_Ratio'] = pred_ratio
            metrics['Num_Solids_Pred'] = pred_n_total
            metrics['Num_Solids'] = pred_n_total
        else:
            # Mesh-only outputs are not assigned CAD-solid metrics.
            pred_solid_ok = False
            metrics['Solid_Watertight_Ratio_Pred'] = np.nan
            metrics['Solid_Watertight_Ratio'] = np.nan
            metrics['Num_Solids_Pred'] = 0
            metrics['Num_Solids'] = 0

        # Solid IoU uses STEP/B-Rep occupancy when both reference and prediction are valid solids.
        if gt_solid_ok and pred_solid_ok:
            metrics['Solid_IoU'] = self._compute_solid_iou(self.iou_samples)
            metrics['Solid_IoU_Computed'] = np.isfinite(metrics['Solid_IoU'])
        else:
            metrics['Solid_IoU'] = np.nan
            metrics['Solid_IoU_Computed'] = False

        # Full-set Solid IoU counts non-computable samples as zero.
        metrics['Solid_IoU_Strict'] = (
            float(metrics['Solid_IoU'])
            if metrics['Solid_IoU_Computed']
            else 0.0
        )

        if self.pred_shape:
            analyzer = BRepCheck_Analyzer(self.pred_shape)
            metrics['Valid_Topo'] = analyzer.IsValid()
            
            if self.gt_shape:
                topo_gt = TopologyEvaluator(self.gt_shape)
                topo_pred = TopologyEvaluator(self.pred_shape)
                metrics['Face_Diff'] = abs(topo_pred.count_faces() - topo_gt.count_faces())
                
                # Engineering-feature metrics are evaluated from structured identity-preserving records.
        return metrics

    def _compute_iou(self, samples):
        """Mesh-based Monte Carlo IoU：依赖 trimesh.contains，因此要求 mesh watertight。"""
        try:
            bounds = np.vstack((self.gt_mesh.bounds, self.pred_mesh.bounds))
            min_b, max_b = np.min(bounds, axis=0), np.max(bounds, axis=0)
            points = np.random.uniform(min_b, max_b, (samples, 3))
            in_gt = self.gt_mesh.contains(points)
            in_pred = self.pred_mesh.contains(points)
            union = np.sum(in_gt | in_pred)
            return np.sum(in_gt & in_pred) / union if union > 0 else np.nan
        except Exception:
            return np.nan

    def _classify_points_in_solid(self, shape, points, tol=1e-7):
        """
        用 OCC 的 BRepClass3d_SolidClassifier 判断采样点是否在 CAD Solid 内部。
        TopAbs_IN 和 TopAbs_ON 都按占据点处理。
        """
        classifier = BRepClass3d_SolidClassifier()
        classifier.Load(shape)

        inside = np.zeros(len(points), dtype=bool)
        for i, p in enumerate(points):
            classifier.Perform(gp_Pnt(float(p[0]), float(p[1]), float(p[2])), tol)
            state = classifier.State()
            inside[i] = (state == TopAbs_IN) or (state == TopAbs_ON)
        return inside

    def _compute_solid_iou(self, samples):
        """
        B-Rep/Solid-based Monte Carlo Volume IoU。
        只要求 GT/Pred 是有效 Solid，不要求 STEP 转 STL 后的 mesh watertight。
        """
        try:
            gt_min, gt_max = get_shape_or_mesh_bounds(self.gt_shape, self.gt_mesh)
            pr_min, pr_max = get_shape_or_mesh_bounds(self.pred_shape, self.pred_mesh)
            if gt_min is None or pr_min is None:
                return np.nan

            min_b = np.minimum(gt_min, pr_min)
            max_b = np.maximum(gt_max, pr_max)
            span = max_b - min_b
            if np.any(~np.isfinite(span)) or np.any(span <= 0):
                return np.nan

            # Add a small padding to reduce boundary-classification instability.
            pad = max(float(np.max(span)) * 1e-6, 1e-9)
            min_b = min_b - pad
            max_b = max_b + pad

            points = np.random.uniform(min_b, max_b, (samples, 3))
            in_gt = self._classify_points_in_solid(self.gt_shape, points)
            in_pred = self._classify_points_in_solid(self.pred_shape, points)

            union = np.sum(in_gt | in_pred)
            if union == 0:
                return np.nan
            return float(np.sum(in_gt & in_pred) / union)
        except Exception:
            return np.nan


class ScanToCadEvaluator:
    def __init__(self, gt_points, gt_normals, pred_shape, pred_mesh, num_samples=10000):
        self.gt_points = gt_points   
        self.gt_normals = gt_normals 
        self.pred_shape = pred_shape
        self.pred_mesh = pred_mesh
        self.num_samples = num_samples

    def compute(self, thresholds):
        if self.pred_mesh is None or len(self.pred_mesh.vertices) == 0: return None
        if self.gt_points is None or len(self.gt_points) == 0: return None

        metrics = {}
        metrics['SR_Pred'] = self.pred_mesh.is_watertight
        metrics['n_verts_pred'] = len(self.pred_mesh.vertices)
        metrics['n_faces_pred'] = len(self.pred_mesh.faces)

        pred_points, idx_sample = trimesh.sample.sample_surface(self.pred_mesh, self.num_samples)
        pred_normals = self.pred_mesh.face_normals[idx_sample]

        tree_gt = cKDTree(self.gt_points)
        tree_pred = cKDTree(pred_points)

        d_pred_gt, _ = tree_gt.query(pred_points, k=1)
        d_gt_pred, _ = tree_pred.query(self.gt_points, k=1)

        metrics['CD'] = (np.mean(d_pred_gt) + np.mean(d_gt_pred)) / 2
        metrics['HD'] = max(np.max(d_pred_gt), np.max(d_gt_pred))
        metrics['Fitting_RMS'] = np.sqrt(np.mean(d_gt_pred**2))

        for th in thresholds:
            prec = np.mean(d_pred_gt < th)
            rec = np.mean(d_gt_pred < th)
            metrics[f'F-Score@{th}'] = 2 * prec * rec / (prec + rec + 1e-8)

        if self.gt_normals is not None:
            _, idx_gt_pred = tree_pred.query(self.gt_points, k=1)
            nearest_pred_normals = pred_normals[idx_gt_pred]
            metrics['NC'] = np.mean(np.abs(np.sum(self.gt_normals * nearest_pred_normals, axis=1)))
        else:
            metrics['NC'] = np.nan
        
        # CAD/B-Rep solid validity statistics
        if self.pred_shape:
            n_water, n_total, ratio = compute_solid_watertight_ratio(self.pred_shape)
            metrics['Solid_Watertight_Ratio'] = ratio
            metrics['Num_Solids'] = n_total
        else:
            # Mesh-only outputs are not STEP/B-Rep solids.
            metrics['Solid_Watertight_Ratio'] = np.nan
            metrics['Num_Solids'] = 0

        if self.pred_shape:
            analyzer = BRepCheck_Analyzer(self.pred_shape)
            metrics['Valid_Topo'] = analyzer.IsValid()

        return metrics

# ==========================================
# 3. Per-file evaluation
# ==========================================
def process_single_file(gt_path, pred_path, mode, deflection, unit, geo_samples, iou_samples):
    """Evaluate one reference/prediction pair."""
    try:
        res = None
        if mode == 'cad2cad':
            gt_shape, gt_mesh = DataLoader.load_cad_model(gt_path, deflection)
            pred_shape, pred_mesh = DataLoader.load_cad_model(pred_path, deflection)
            
            evaluator = CadToCadEvaluator(
                gt_shape, gt_mesh, pred_shape, pred_mesh, 
                num_samples=geo_samples,
                iou_samples=iou_samples
            )
            th_list = [0.01, 0.05, 0.1, 0.2, 0.5, 1.0] if unit == 'mm' else [0.01, 0.05]
            res = evaluator.compute(thresholds=th_list)

        else: 
            gt_pts, gt_norms = DataLoader.load_scan_points(gt_path)
            if gt_pts is not None and len(gt_pts) > 50000:
                idx = np.random.choice(len(gt_pts), 50000, replace=False)
                gt_pts = gt_pts[idx]
                if gt_norms is not None: gt_norms = gt_norms[idx]
            
            pred_shape, pred_mesh = DataLoader.load_cad_model(pred_path, deflection)
            
            evaluator = ScanToCadEvaluator(gt_pts, gt_norms, pred_shape, pred_mesh, num_samples=geo_samples)
            th = [0.01, 0.05, 0.1, 0.5, 1.0, 2.0] if unit == 'mm' else [0.0005, 0.001, 0.002]
            res = evaluator.compute(thresholds=th)
        
        return res
    except Exception as e:
        return None

# ==========================================
# 4. Timeout-controlled worker execution
# ==========================================
def _process_worker(result_queue, payload):
    """Worker entry point; kept at module scope for multiprocessing spawn compatibility."""
    try:
        (
            gt_path, pred_path, mode, deflection, unit,
            geo_samples, iou_samples, eval_seed
        ) = payload

        # Use a deterministic per-sample seed for reproducible evaluation.
        np.random.seed(int(eval_seed) % (2**32 - 1))
        res = process_single_file(
            gt_path, pred_path, mode, deflection, unit,
            geo_samples, iou_samples
        )
        result_queue.put(("ok", res))
    except BaseException as e:
        result_queue.put(("error", {
            "error": repr(e),
            "traceback": traceback.format_exc()
        }))


def run_process_with_timeout(payload, timeout, mp_ctx):
    """Run one model evaluation in an isolated process with enforceable timeout."""
    result_queue = mp_ctx.Queue(maxsize=1)
    proc = mp_ctx.Process(target=_process_worker, args=(result_queue, payload))
    proc.start()
    proc.join(timeout)

    if proc.is_alive():
        proc.terminate()
        proc.join(5)
        if proc.is_alive():
            try:
                proc.kill()
            except Exception:
                pass
            proc.join(5)
        try:
            result_queue.close()
            result_queue.join_thread()
        except Exception:
            pass
        return None, "Timeout", None

    try:
        # Allow the queue feeder a short interval to flush after worker exit.
        status, data = result_queue.get(timeout=2)
    except queue_mod.Empty:
        exitcode = proc.exitcode
        try:
            result_queue.close()
            result_queue.join_thread()
        except Exception:
            pass
        if exitcode == 0:
            return None, "EmptyResult", None
        return None, f"WorkerExit({exitcode})", None

    try:
        result_queue.close()
        result_queue.join_thread()
    except Exception:
        pass

    if status == "ok":
        return data, None, None
    return None, "WorkerError", data


# ==========================================
# 5. Batch evaluation
# ==========================================
def _as_bool_series(s):
    """Robustly convert a pandas Series to boolean values."""
    if s is None:
        return None
    return s.fillna(False).map(
        lambda x: bool(x) if isinstance(x, (bool, np.bool_))
        else str(x).strip().lower() in {"true", "1", "yes"}
    )


def _num_series(df, col, default=np.nan):
    """Return a numeric Series with the same index as df."""
    if col not in df.columns:
        return pd.Series(default, index=df.index, dtype=float)
    return pd.to_numeric(df[col], errors="coerce")


def _is_solid_output_method(df):
    """
    Determine whether the evaluated method is intended to output STEP/B-Rep solids.
    If at least one valid row has .step/.stp as prediction type, invalid rows are
    also treated as failed solid outputs in full-set S-VR/S-IoU.
    """
    if "Pred_Type" not in df.columns:
        return False
    return df["Pred_Type"].fillna("").str.lower().isin([".step", ".stp"]).any()


def _solid_valid_mask(df):
    """
    Per-sample solid-valid mask.
    A sample is solid-valid only if it contains at least one solid and the predicted
    STEP/B-Rep passes the CAD-kernel validity check.
    """
    ratio = _num_series(df, "Solid_Watertight_Ratio_Pred", default=np.nan)
    if ratio.isna().all():
        ratio = _num_series(df, "Solid_Watertight_Ratio", default=np.nan)

    n_solids = _num_series(df, "Num_Solids_Pred", default=0).fillna(0)
    if "Num_Solids_Pred" not in df.columns and "Num_Solids" in df.columns:
        n_solids = _num_series(df, "Num_Solids", default=0).fillna(0)

    solid_ok = (n_solids > 0) & (ratio >= 1.0)

    if "Valid_Topo" in df.columns:
        valid_topo = _as_bool_series(df["Valid_Topo"])
        solid_ok = solid_ok & valid_topo

    return solid_ok.fillna(False)


def summarize_fullset_metrics(df, mode):
    """
    Summarize evaluation results with both full-set and valid-only statistics.
    Full-set metrics are intended for main-paper tables.
    Valid-only metrics are intended for supplementary conditional-quality reporting.
    """
    total = len(df)
    valid_mask = _as_bool_series(df["Valid"]) if "Valid" in df.columns else pd.Series(False, index=df.index)
    valid_df = df[valid_mask]

    summary = {
        "Total": total,
        "Valid": int(valid_mask.sum()),
        "Invalid": int(total - valid_mask.sum()),
        "IR_all(%)": 100.0 * (1.0 - valid_mask.mean()) if total > 0 else np.nan,
    }

    if total == 0:
        return summary, valid_df

    # -----------------------------
    # Mesh-level watertightness
    # -----------------------------
    if "SR_Pred" in df.columns:
        sr_pred = _as_bool_series(df["SR_Pred"])
        summary["M-WR_all(%)"] = 100.0 * sr_pred.mean()
        summary["M-WR_valid(%)"] = 100.0 * sr_pred[valid_mask].mean() if valid_mask.any() else np.nan
    else:
        summary["M-WR_all(%)"] = np.nan
        summary["M-WR_valid(%)"] = np.nan

    # -----------------------------
    # Solid-level validity/watertightness
    # -----------------------------
    solid_method = _is_solid_output_method(df)
    summary["Solid_Output_Method"] = solid_method

    if solid_method:
        solid_valid = _solid_valid_mask(df)
        summary["S-WR_all(%)"] = 100.0 * solid_valid.mean()
        summary["S-WR_valid(%)"] = 100.0 * solid_valid[valid_mask].mean() if valid_mask.any() else np.nan
    else:
        summary["S-WR_all(%)"] = np.nan
        summary["S-WR_valid(%)"] = np.nan

    # -----------------------------
    # Mesh IoU
    # -----------------------------
    if "IoU" in df.columns:
        mesh_iou = _num_series(df, "IoU", default=np.nan)
        if "Mesh_IoU_Computed" in df.columns:
            mesh_iou_computed = _as_bool_series(df["Mesh_IoU_Computed"])
        else:
            mesh_iou_computed = mesh_iou.notna()

        summary["M-IoU_all"] = mesh_iou.where(mesh_iou_computed, 0.0).fillna(0.0).mean()
        summary["M-IoU_valid"] = mesh_iou[mesh_iou_computed].mean() if mesh_iou_computed.any() else np.nan
        summary["M-IoU_cover_all(%)"] = 100.0 * mesh_iou_computed.mean()
        summary["M-IoU_cover_valid(%)"] = 100.0 * mesh_iou_computed[valid_mask].mean() if valid_mask.any() else np.nan
    else:
        summary["M-IoU_all"] = np.nan
        summary["M-IoU_valid"] = np.nan
        summary["M-IoU_cover_all(%)"] = np.nan
        summary["M-IoU_cover_valid(%)"] = np.nan

    # -----------------------------
    # Solid IoU
    # -----------------------------
    if solid_method and "Solid_IoU" in df.columns:
        solid_iou = _num_series(df, "Solid_IoU", default=np.nan)
        if "Solid_IoU_Computed" in df.columns:
            solid_iou_computed = _as_bool_series(df["Solid_IoU_Computed"])
        else:
            solid_iou_computed = solid_iou.notna()

        summary["S-IoU_all"] = solid_iou.where(solid_iou_computed, 0.0).fillna(0.0).mean()
        summary["S-IoU_valid"] = solid_iou[solid_iou_computed].mean() if solid_iou_computed.any() else np.nan
        summary["S-IoU_cover_all(%)"] = 100.0 * solid_iou_computed.mean()
        summary["S-IoU_cover_valid(%)"] = 100.0 * solid_iou_computed[valid_mask].mean() if valid_mask.any() else np.nan
    else:
        summary["S-IoU_all"] = np.nan
        summary["S-IoU_valid"] = np.nan
        summary["S-IoU_cover_all(%)"] = np.nan
        summary["S-IoU_cover_valid(%)"] = np.nan

    # -----------------------------
    # Conditional geometric fidelity summary
    # -----------------------------
    for col in ["CD", "HD", "NC"]:
        if col in valid_df.columns:
            vals = pd.to_numeric(valid_df[col], errors="coerce")
            vals = vals[np.isfinite(vals)]
            summary[f"{col}_valid_mean"] = vals.mean() if len(vals) else np.nan
            summary[f"{col}_valid_median"] = vals.median() if len(vals) else np.nan
        else:
            summary[f"{col}_valid_mean"] = np.nan
            summary[f"{col}_valid_median"] = np.nan

    for col in sorted(c for c in valid_df.columns if str(c).startswith("F-Score@")):
        vals = pd.to_numeric(valid_df[col], errors="coerce")
        vals = vals[np.isfinite(vals)]
        summary[f"{col}_valid_mean"] = vals.mean() if len(vals) else np.nan

    # S-VR aliases are retained alongside historical S-WR keys for compatibility.
    summary["S-VR_all(%)"] = summary.get("S-WR_all(%)", np.nan)
    summary["S-VR_valid(%)"] = summary.get("S-WR_valid(%)", np.nan)

    # Feature-instance metrics are computed from structured identity-preserving records.

    return summary, valid_df


# ==========================================
# 6. Engineering-feature evaluation
# ==========================================
UNIFIED_PROTOCOL = "RbRM-UnifiedEvaluation-1.0"
DEFAULT_EV2_MANIFEST_SCHEMA = "RbRM-EV2-GTManifest-1.4"


def _stable_sample_seed(base_seed, sample_id):
    """Derive a deterministic uint32 seed from base_seed + sample_id."""
    payload = f"{int(base_seed)}::{sample_id}".encode("utf-8")
    digest = hashlib.sha256(payload).digest()
    return int.from_bytes(digest[:4], byteorder="little", signed=False)


def _sha256_file(path):
    path = Path(path)
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _jsonable(value):
    """Convert numpy/pandas/path values into strict JSON-compatible values."""
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        value = float(value)
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if pd.isna(value) if not isinstance(value, (str, bytes)) else False:
        return None
    return value


def _write_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            _jsonable(payload),
            ensure_ascii=True,
            indent=2,
            allow_nan=False,
        ),
        encoding="ascii",
    )


def _default_ev2_module_dir():
    # Expected repository layout:
    #   repo/deviation_analysis/batch_evaluate_sm4.py
    #   repo/evaluation_v2/*.py
    return Path(__file__).resolve().parent.parent / "evaluation_v2"


def _load_ev2_modules(module_dir):
    """Load the feature-evaluation modules from an explicit repository directory."""
    module_dir = Path(module_dir).resolve()
    required = {
        "metric_definitions": module_dir / "metric_definitions.py",
        "prediction_adapters": module_dir / "prediction_adapters.py",
        "join_feature_instances": module_dir / "join_feature_instances.py",
        "evaluate_feature_instances": module_dir / "evaluate_feature_instances.py",
    }

    missing = [str(p) for p in required.values() if not p.is_file()]
    if missing:
        raise FileNotFoundError(
            "EV2 module files are missing:\n  - " + "\n  - ".join(missing)
        )

    module_dir_str = str(module_dir)
    if module_dir_str not in sys.path:
        sys.path.insert(0, module_dir_str)

    loaded = {}
    # Load metric_definitions first because evaluate_feature_instances imports it by module name.
    for name in [
        "metric_definitions",
        "prediction_adapters",
        "join_feature_instances",
        "evaluate_feature_instances",
    ]:
        if name in sys.modules:
            existing_file = getattr(sys.modules[name], "__file__", None)
            if existing_file is not None:
                existing_parent = Path(existing_file).resolve().parent
                if existing_parent != module_dir:
                    del sys.modules[name]
        loaded[name] = importlib.import_module(name)

    return loaded, required


def _read_gt_manifest_inventory(gt_manifest, dataset, expected_schema=None):
    gt_manifest = Path(gt_manifest).resolve()
    if not gt_manifest.is_file():
        raise FileNotFoundError(f"EV2 GT manifest not found: {gt_manifest}")

    with gt_manifest.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise RuntimeError(f"EV2 GT manifest has no header: {gt_manifest}")
        rows = [dict(r) for r in reader]

    rows = [r for r in rows if str(r.get("dataset", "")).strip() == dataset]
    if not rows:
        raise RuntimeError(
            f"EV2 GT manifest contains no rows for dataset={dataset}: {gt_manifest}"
        )

    schemas = {str(r.get("schema_version", "")).strip() for r in rows}
    if expected_schema and schemas != {expected_schema}:
        raise RuntimeError(
            "Unexpected EV2 GT manifest schema. "
            f"expected={expected_schema}, observed={sorted(schemas)}"
        )

    uids = [str(r.get("instance_uid", "")).strip() for r in rows]
    if any(not uid for uid in uids):
        raise RuntimeError("EV2 GT manifest contains empty instance_uid.")
    if len(uids) != len(set(uids)):
        raise RuntimeError("EV2 GT manifest contains duplicate instance_uid.")

    sample_ids = sorted({str(r.get("sample_id", "")).strip() for r in rows})
    if any(not s for s in sample_ids):
        raise RuntimeError("EV2 GT manifest contains empty sample_id.")

    return gt_manifest, rows, sample_ids, sorted(schemas)


def _candidate_priority(path, xlsx_root, sample_id, policy="prefer-root"):
    """Deterministic priority for known historical XLSX layouts."""
    path = Path(path).resolve()
    xlsx_root = Path(xlsx_root).resolve()

    sample_root = (xlsx_root / sample_id / f"{sample_id}.xlsx").resolve()
    sample_output = (
        xlsx_root / sample_id / "output" / f"{sample_id}.xlsx"
    ).resolve()
    run_root = (xlsx_root / f"{sample_id}.xlsx").resolve()

    if policy in {"prefer-root", "root-only"}:
        # Canonical production order: sample-root first, then output fallback.
        preferred = [sample_root, sample_output, run_root]
    else:
        # Explicit diagnostic/legacy output-first modes.
        preferred = [sample_output, sample_root, run_root]

    for rank, p in enumerate(preferred):
        if path == p:
            return rank
    return 100


def _find_ev2_xlsx_candidates(
    xlsx_root,
    sample_id,
    policy="prefer-root",
):
    """
    Find exact-name XLSX candidates for one sample.

    Supported layouts:
      <run>/<sample>/<sample>.xlsx
      <run>/<sample>/output/<sample>.xlsx
      <run>/<sample>.xlsx

    Policies:
      prefer-root
          Default production rule. Prefer <sample>/<sample>.xlsx whenever it
          exists. If root/output copies conflict, keep the conflict in the audit
          inventory but use the sample-root workbook. If the root workbook is
          absent, fall back to output/<sample>.xlsx and then other exact-name
          candidates.
      strict-equivalent
          Diagnostic mode: inspect all candidates and fail if their canonical EV2
          records differ.
      prefer-output
          Prefer <sample>/output/<sample>.xlsx, with the same audited behavior.
      root-only / output-only
          Accept only the explicitly selected canonical layout.
    """
    valid_policies = {
        "strict-equivalent",
        "prefer-root",
        "prefer-output",
        "root-only",
        "output-only",
    }
    if policy not in valid_policies:
        raise ValueError(
            f"Unknown EV2 XLSX policy: {policy}. "
            f"Expected one of {sorted(valid_policies)}"
        )

    xlsx_root = Path(xlsx_root).resolve()
    sample_dir = xlsx_root / sample_id
    sample_root = xlsx_root / sample_id / f"{sample_id}.xlsx"
    sample_output = xlsx_root / sample_id / "output" / f"{sample_id}.xlsx"
    run_root = xlsx_root / f"{sample_id}.xlsx"

    if policy == "root-only":
        return [sample_root.resolve()] if sample_root.is_file() else []
    if policy == "output-only":
        return [sample_output.resolve()] if sample_output.is_file() else []

    candidates = set()
    for p in [sample_root, sample_output, run_root]:
        if p.is_file():
            candidates.add(p.resolve())

    if sample_dir.is_dir():
        for p in sample_dir.rglob(f"{sample_id}.xlsx"):
            if p.is_file():
                candidates.add(p.resolve())

    return sorted(
        candidates,
        key=lambda p: (
            _candidate_priority(p, xlsx_root, sample_id, policy),
            str(p),
        ),
    )


def _records_digest(records):
    """Semantic digest of canonical EV2 prediction records."""
    payload = [asdict(r) for r in records]
    payload.sort(key=lambda x: str(x.get("instance_uid", "")))
    text = json.dumps(
        payload,
        ensure_ascii=True,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    return hashlib.sha256(text.encode("ascii")).hexdigest()


def _looks_like_dataset_sample_id(candidate, gt_sample_ids):
    """
    Return True only when ``candidate`` follows the sample-ID syntax already
    present in the selected GT manifest.

    This prevents administrative directories/files such as
    ``predictions_for_eval/predictions_for_eval.xlsx`` from being mistaken for
    out-of-manifest samples while still catching genuinely stray CADParser or
    DeepCAD sample outputs.
    """
    candidate = str(candidate).strip()
    gt_ids = [str(x).strip() for x in gt_sample_ids if str(x).strip()]
    if not candidate or not gt_ids:
        return False

    if candidate in set(gt_ids):
        return True

    # CADParser sample IDs, e.g. 00003_index_1.
    if all(re.fullmatch(r"\d+_index_\d+", x) for x in gt_ids):
        return re.fullmatch(r"\d+_index_\d+", candidate) is not None

    # DeepCAD sample IDs are numeric strings.
    if all(re.fullmatch(r"\d+", x) for x in gt_ids):
        return re.fullmatch(r"\d+", candidate) is not None

    # Unknown naming schemes are not interpreted as structured sample IDs.
    return False


def _detect_unknown_structured_samples(xlsx_root, gt_sample_ids):
    """
    Detect structured sample outputs outside the selected GT sample set.

    Only paths whose names follow the GT sample-ID naming convention are
    considered candidate sample outputs. Administrative artifacts such as
    ``predictions_for_eval/predictions_for_eval.xlsx`` are intentionally
    ignored.
    """
    xlsx_root = Path(xlsx_root).resolve()
    gt_set = set(gt_sample_ids)
    unknown = []

    if not xlsx_root.is_dir():
        return unknown

    # Root-level <sample>.xlsx candidates.
    for p in xlsx_root.glob("*.xlsx"):
        if p.name.startswith("~$"):
            continue
        sid = p.stem
        if (
            sid not in gt_set
            and _looks_like_dataset_sample_id(sid, gt_sample_ids)
        ):
            unknown.append(str(p.resolve()))

    # Per-sample directory candidates using the dataset sample-ID convention.
    for child in xlsx_root.iterdir():
        if not child.is_dir() or child.name in gt_set:
            continue

        sid = child.name
        if not _looks_like_dataset_sample_id(sid, gt_sample_ids):
            continue

        probes = [
            child / f"{sid}.xlsx",
            child / "output" / f"{sid}.xlsx",
        ]
        if any(p.is_file() for p in probes):
            unknown.extend(str(p.resolve()) for p in probes if p.is_file())

    return sorted(set(unknown))


def _resolve_and_adapt_sample_xlsx(
    pa,
    dataset,
    xlsx_root,
    sample_id,
    sheet_name,
    strict,
    xlsx_policy="prefer-root",
):
    """
    Resolve one sample XLSX and adapt it to canonical EV2 records.

    Production rule: sample-root XLSX is canonical when present.

    Resolution order:
      1. <run>/<sample>/<sample>.xlsx
      2. <run>/<sample>/output/<sample>.xlsx
      3. <run>/<sample>.xlsx
      4. other exact-name recursive candidates

    If both sample-root and output XLSX exist and are non-equivalent, the
    sample-root workbook is selected and the conflict is retained in the EV2
    audit inventory. If only one candidate exists, that candidate is used.
    Alternative policies remain available for diagnostics, but ``prefer-root``
    is the default used by the common evaluator.
    """
    candidates = _find_ev2_xlsx_candidates(
        xlsx_root,
        sample_id,
        policy=xlsx_policy,
    )
    sample_dir = Path(xlsx_root).resolve() / sample_id

    if not candidates:
        if strict and sample_dir.is_dir():
            other_xlsx = [
                p for p in sample_dir.rglob("*.xlsx")
                if p.is_file() and not p.name.startswith("~$")
            ]
            if other_xlsx:
                raise RuntimeError(
                    f"Sample {sample_id}: XLSX files exist but none has the expected "
                    f"name {sample_id}.xlsx. Candidates: "
                    + "; ".join(str(p) for p in other_xlsx)
                )
        return {
            "sample_id": sample_id,
            "status": "MISSING_XLSX",
            "chosen_path": None,
            "candidate_paths": [],
            "candidate_digests": [],
            "xlsx_policy": xlsx_policy,
            "headers": [],
            "records": [],
        }

    adapted = []
    for p in candidates:
        headers, records = pa.adapt_prediction_xlsx(
            dataset=dataset,
            xlsx_path=p,
            sample_id=sample_id,
            sheet_name=sheet_name,
        )
        adapted.append((p, headers, records, _records_digest(records)))

    unique_digests = {item[3] for item in adapted}
    non_equivalent = len(unique_digests) > 1
    if non_equivalent and xlsx_policy == "strict-equivalent":
        details = "\n".join(
            f"  {p} -> {digest}"
            for p, _, _, digest in adapted
        )
        raise RuntimeError(
            f"Sample {sample_id}: multiple non-equivalent EV2 XLSX records found. "
            "Refusing to choose silently under --ev2_xlsx_policy "
            "strict-equivalent.\n" + details
        )

    # Candidates are ordered by the selected resolution policy.
    chosen_path, headers, records, digest = adapted[0]
    if len(candidates) == 1:
        status = "OK"
    elif not non_equivalent:
        status = "MULTIPLE_EQUIVALENT_XLSX"
    else:
        status = (
            "MULTIPLE_NON_EQUIVALENT_ROOT_SELECTED"
            if xlsx_policy == "prefer-root"
            else "MULTIPLE_NON_EQUIVALENT_OUTPUT_SELECTED"
        )

    return {
        "sample_id": sample_id,
        "status": status,
        "chosen_path": chosen_path,
        "candidate_paths": [x[0] for x in adapted],
        "candidate_digests": [x[3] for x in adapted],
        "xlsx_policy": xlsx_policy,
        "headers": headers,
        "records": records,
        "semantic_digest": digest,
    }


def run_ev2_feature_evaluation(args):
    """Run structured feature evaluation using the shared adapter, join, and metric modules."""
    if args.mode != "cad2cad":
        raise RuntimeError("EV2 feature metrics are only supported in cad2cad mode.")
    if not args.ev2_gt_manifest:
        raise RuntimeError("--ev2_gt_manifest is required when --enable_ev2 is used.")

    module_dir = (
        Path(args.ev2_module_dir).resolve()
        if args.ev2_module_dir
        else _default_ev2_module_dir().resolve()
    )
    modules, module_files = _load_ev2_modules(module_dir)
    md = modules["metric_definitions"]
    pa = modules["prediction_adapters"]
    jf = modules["join_feature_instances"]
    efe = modules["evaluate_feature_instances"]

    xlsx_root = (
        Path(args.ev2_xlsx_root).resolve()
        if args.ev2_xlsx_root
        else (
            Path(args.pred_dir).resolve().parent
            if Path(args.pred_dir).resolve().name.lower() == "predictions_for_eval"
            else Path(args.pred_dir).resolve()
        )
    )
    if not xlsx_root.is_dir():
        raise FileNotFoundError(f"EV2 XLSX root not found: {xlsx_root}")

    ev2_out = (
        Path(args.ev2_out_dir).resolve()
        if args.ev2_out_dir
        else Path(args.pred_dir).resolve() / "ev2"
    )
    ev2_out.mkdir(parents=True, exist_ok=True)

    gt_manifest, gt_rows_raw, sample_ids, manifest_schemas = _read_gt_manifest_inventory(
        gt_manifest=args.ev2_gt_manifest,
        dataset=args.ev2_dataset,
        expected_schema=args.ev2_manifest_schema,
    )

    if args.ev2_expected_samples is not None:
        if len(sample_ids) != int(args.ev2_expected_samples):
            raise RuntimeError(
                f"EV2 sample count mismatch: observed={len(sample_ids)}, "
                f"expected={args.ev2_expected_samples}"
            )
    if args.ev2_expected_features is not None:
        if len(gt_rows_raw) != int(args.ev2_expected_features):
            raise RuntimeError(
                f"EV2 feature count mismatch: observed={len(gt_rows_raw)}, "
                f"expected={args.ev2_expected_features}"
            )

    unknown_structured_samples = _detect_unknown_structured_samples(
        xlsx_root, sample_ids
    )
    if unknown_structured_samples and args.ev2_strict:
        raise RuntimeError(
            "Structured XLSX outputs were found for samples outside the selected "
            "GT manifest:\n  - " + "\n  - ".join(unknown_structured_samples)
        )

    gt_feature_count_by_sample = {}
    for row in gt_rows_raw:
        sid = str(row.get("sample_id", "")).strip()
        gt_feature_count_by_sample[sid] = gt_feature_count_by_sample.get(sid, 0) + 1

    all_records = []
    inventory = []
    adapter_samples_dir = ev2_out / "adapter_samples"

    print("\n" + "=" * 90)
    print("EV2 STRUCTURED FEATURE EVALUATION | exact UID, no geometric matching")
    print("=" * 90)
    print(f"Dataset       : {args.ev2_dataset}")
    print(f"GT manifest   : {gt_manifest}")
    print(f"XLSX root     : {xlsx_root}")
    print(f"XLSX policy   : {args.ev2_xlsx_policy}")
    print(f"EV2 out       : {ev2_out}")
    print(f"GT samples    : {len(sample_ids)}")
    print(f"GT features   : {len(gt_rows_raw)}")
    print(f"Manifest      : {manifest_schemas}")

    for idx, sample_id in enumerate(sample_ids, start=1):
        resolved = _resolve_and_adapt_sample_xlsx(
            pa=pa,
            dataset=args.ev2_dataset,
            xlsx_root=xlsx_root,
            sample_id=sample_id,
            sheet_name=args.ev2_sheet,
            strict=args.ev2_strict,
            xlsx_policy=args.ev2_xlsx_policy,
        )

        records = resolved["records"]
        invalid_records = [r for r in records if not r.adapter_valid]
        all_records.extend(records)

        sample_out = adapter_samples_dir / sample_id
        sample_out.mkdir(parents=True, exist_ok=True)
        pa.write_csv(sample_out / "prediction_records.csv", records)

        if records:
            adapter_summary = pa.make_summary(
                headers=resolved["headers"],
                records=records,
            )
        else:
            adapter_summary = {
                "protocol": getattr(pa, "ADAPTER_PROTOCOL", ""),
                "headers": [],
                "record_count": 0,
                "type_counts": {},
                "role_counts": {},
                "adapter_valid_count": 0,
                "adapter_invalid_count": 0,
                "top_axis_checked_count": 0,
                "top_axis_error_max": None,
                "top_axis_error_mean": None,
                "top_axis_error_over_height_max": None,
                "top_axis_error_over_height_mean": None,
            }

        sample_payload = {
            "protocol": getattr(pa, "ADAPTER_PROTOCOL", ""),
            "dataset": args.ev2_dataset,
            "sample_id": sample_id,
            "status": resolved["status"],
            "source_xlsx": (
                str(resolved["chosen_path"])
                if resolved["chosen_path"] is not None
                else None
            ),
            "candidate_xlsx": [str(p) for p in resolved["candidate_paths"]],
            "candidate_semantic_digests": resolved.get("candidate_digests", []),
            "xlsx_policy": resolved.get("xlsx_policy", args.ev2_xlsx_policy),
            "semantic_digest": resolved.get("semantic_digest"),
            "summary": adapter_summary,
            "records": [asdict(r) for r in records],
        }
        pa.write_json(sample_out / "prediction_records.json", sample_payload)

        inventory.append({
            "sample_id": sample_id,
            "gt_feature_count": gt_feature_count_by_sample.get(sample_id, 0),
            "xlsx_status": resolved["status"],
            "xlsx_candidate_count": len(resolved["candidate_paths"]),
            "chosen_xlsx": (
                str(resolved["chosen_path"])
                if resolved["chosen_path"] is not None
                else ""
            ),
            "xlsx_candidates": " | ".join(str(p) for p in resolved["candidate_paths"]),
            "xlsx_candidate_digests": " | ".join(resolved.get("candidate_digests", [])),
            "xlsx_policy": resolved.get("xlsx_policy", args.ev2_xlsx_policy),
            "semantic_digest": resolved.get("semantic_digest", ""),
            "prediction_record_count": len(records),
            "adapter_valid_count": len(records) - len(invalid_records),
            "adapter_invalid_count": len(invalid_records),
        })

        # print(
        #     f"[{idx:02d}/{len(sample_ids):02d}] {sample_id} | "
        #     f"{resolved['status']} | pred_features={len(records)} | "
        #     f"invalid={len(invalid_records)}"
        # )

    missing_xlsx_samples = [
        r["sample_id"] for r in inventory if r["xlsx_status"] == "MISSING_XLSX"
    ]
    if missing_xlsx_samples and args.ev2_fail_on_missing_xlsx:
        raise RuntimeError(
            "EV2 structured XLSX missing for samples:\n  - "
            + "\n  - ".join(missing_xlsx_samples)
        )

    # ------------------------------------------------------------------
    # Dataset-level canonical prediction records.
    # ------------------------------------------------------------------
    prediction_all_csv = ev2_out / "prediction_records_all.csv"
    pa.write_csv(prediction_all_csv, all_records)

    # ------------------------------------------------------------------
    # Exact identity join.
    # ------------------------------------------------------------------
    gt_fields, gt_all_rows = jf.read_csv_rows(gt_manifest)
    gt_rows = jf.filter_gt_rows(
        gt_all_rows,
        dataset=args.ev2_dataset,
        sample_id=None,
    )
    pred_fields, pred_rows = jf.read_csv_rows(prediction_all_csv)
    pred_rows = jf.filter_pred_rows(
        pred_rows,
        dataset=args.ev2_dataset,
        sample_id=None,
    )

    joined_rows, unknown_rows, join_summary = jf.join_rows(
        gt_rows=gt_rows,
        pred_rows=pred_rows,
        gt_fields=gt_fields,
        pred_fields=pred_fields,
    )

    join_dir = ev2_out / "identity_join"
    join_dir.mkdir(parents=True, exist_ok=True)
    join_csv = join_dir / "identity_join.csv"
    join_report = join_dir / "identity_join_report.json"
    unknown_csv = join_dir / "unknown_prediction_records.csv"

    join_base_fields = [
        "protocol",
        "instance_uid",
        "dataset",
        "sample_id",
        "instance_id",
        "join_status",
        "prediction_present",
        "prediction_adapter_valid",
        "feature_type_match",
        "boolean_role_match",
    ]
    join_fields = (
        join_base_fields
        + [f"gt_{x}" for x in gt_fields]
        + [f"pred_{x}" for x in pred_fields]
    )
    jf.write_csv(join_csv, joined_rows, join_fields)
    jf.write_csv(unknown_csv, unknown_rows, pred_fields)
    jf.write_json_ascii(join_report, {
        "protocol": getattr(jf, "PROTOCOL", ""),
        "gt_manifest": str(gt_manifest),
        "prediction_records": str(prediction_all_csv),
        "dataset_filter": args.ev2_dataset,
        "sample_id_filter": None,
        "summary": join_summary,
    })

    # Strict identity checks; missing predictions remain valid FR failures.
    fatal_join_keys = [
        "unknown_prediction_count",
        "type_mismatch_count",
        "uid_component_mismatch_count",
    ]
    if args.ev2_strict:
        bad = {
            key: int(join_summary.get(key, 0))
            for key in fatal_join_keys
            if int(join_summary.get(key, 0)) > 0
        }
        if bad:
            raise RuntimeError(f"EV2 strict identity-join gate failed: {bad}")

    # ------------------------------------------------------------------
    # Feature-instance metric evaluator.
    # ------------------------------------------------------------------
    identity_errors = efe.validate_identity_join_rows(
        rows=joined_rows,
        expected_dataset=args.ev2_dataset,
    )
    per_instance, dataset_summary, evaluation_audit = efe.evaluate_rows(joined_rows)
    if identity_errors:
        evaluation_audit["errors"] = identity_errors + list(
            evaluation_audit.get("errors", [])
        )
        evaluation_audit["result"] = "FAIL"

    per_instance_path = ev2_out / "per_instance_metrics.csv"
    dataset_summary_path = ev2_out / "dataset_summary.json"
    audit_path = ev2_out / "evaluation_audit.json"

    per_instance_fields = list(per_instance[0].keys()) if per_instance else []
    efe.write_csv(per_instance_path, per_instance, per_instance_fields)
    efe.write_json_ascii(dataset_summary_path, dataset_summary)
    efe.write_json_ascii(audit_path, evaluation_audit)

    if args.ev2_strict and evaluation_audit.get("result") != "PASS":
        raise RuntimeError(
            "EV2 evaluator audit failed: "
            + "; ".join(evaluation_audit.get("errors", []))
        )

    # Add exact-join outcomes to the sample inventory.
    joined_by_sample = {}
    for row in joined_rows:
        sid = str(row.get("sample_id", ""))
        stat = joined_by_sample.setdefault(sid, {
            "joined_feature_count": 0,
            "missing_feature_count": 0,
            "type_mismatch_count": 0,
            "role_mismatch_count": 0,
            "invalid_prediction_count": 0,
        })
        status = str(row.get("join_status", ""))
        if status == "MISSING_PREDICTION":
            stat["missing_feature_count"] += 1
        else:
            stat["joined_feature_count"] += 1
        if str(row.get("feature_type_match", "")).lower() == "false":
            stat["type_mismatch_count"] += 1
        if str(row.get("boolean_role_match", "")).lower() == "false":
            stat["role_mismatch_count"] += 1
        if (
            str(row.get("prediction_present", "")).lower() == "true"
            and str(row.get("prediction_adapter_valid", "")).lower() != "true"
        ):
            stat["invalid_prediction_count"] += 1

    for item in inventory:
        item.update(joined_by_sample.get(item["sample_id"], {}))

    inventory_csv = ev2_out / "run_inventory.csv"
    pd.DataFrame(inventory).to_csv(inventory_csv, index=False)

    # Feature-evaluation run provenance.
    ev2_manifest = {
        "unified_protocol": UNIFIED_PROTOCOL,
        "dataset": args.ev2_dataset,
        "gt_manifest": str(gt_manifest),
        "gt_manifest_sha256": _sha256_file(gt_manifest),
        "manifest_schema": manifest_schemas,
        "xlsx_root": str(xlsx_root),
        "xlsx_policy": args.ev2_xlsx_policy,
        "sample_count": len(sample_ids),
        "feature_count": len(gt_rows_raw),
        "missing_xlsx_sample_count": len(missing_xlsx_samples),
        "missing_xlsx_samples": missing_xlsx_samples,
        "unknown_structured_samples": unknown_structured_samples,
        "adapter_protocol": getattr(pa, "ADAPTER_PROTOCOL", ""),
        "join_protocol": getattr(jf, "PROTOCOL", ""),
        "evaluator_protocol": getattr(efe, "EVALUATOR_PROTOCOL", ""),
        "metric_protocol": getattr(md, "PROTOCOL_VERSION", ""),
        "join_summary": join_summary,
        "evaluation_audit": evaluation_audit,
        "module_sha256": {
            name: _sha256_file(path)
            for name, path in module_files.items()
        },
        "outputs": {
            "prediction_records_all": str(prediction_all_csv),
            "identity_join": str(join_csv),
            "per_instance_metrics": str(per_instance_path),
            "dataset_summary": str(dataset_summary_path),
            "evaluation_audit": str(audit_path),
            "run_inventory": str(inventory_csv),
        },
    }
    ev2_run_manifest_path = ev2_out / "ev2_run_manifest.json"
    _write_json(ev2_run_manifest_path, ev2_manifest)

    print("-" * 90)
    print("EV2 JOIN SUMMARY")
    print(f"  GT instances       : {join_summary['gt_instance_count']}")
    print(f"  Prediction records : {join_summary['prediction_record_count']}")
    print(f"  Joined             : {join_summary['joined_count']}")
    print(f"  Missing prediction : {join_summary['missing_prediction_count']}")
    print(f"  Unknown prediction : {join_summary['unknown_prediction_count']}")
    print(f"  Type mismatch      : {join_summary['type_mismatch_count']}")
    print(f"  Role mismatch      : {join_summary['role_mismatch_count']}")
    print(f"  Invalid prediction : {join_summary['invalid_prediction_count']}")

    fr = dataset_summary["feature_reconstruction"]
    metrics = dataset_summary["metrics"]
    print("-" * 90)
    print("EV2 DATASET METRICS")
    print(
        f"  FR   : {fr['FR_percent']}% "
        f"({fr['recovered_count']}/{fr['gt_eligible_count']})"
    )
    for name in ["AAE", "RRE", "ECD", "nECD"]:
        m = metrics[name]
        print(
            f"  {name:<4}: mean={m['mean']} | "
            f"evaluated={m['evaluated_count']}/{m['gt_eligible_count']}"
        )
    print(f"  Audit: {evaluation_audit['result']}")
    print("=" * 90)

    return {
        "out_dir": str(ev2_out),
        "dataset_summary": dataset_summary,
        "evaluation_audit": evaluation_audit,
        "join_summary": join_summary,
        "ev2_run_manifest": str(ev2_run_manifest_path),
        "module_files": {k: str(v) for k, v in module_files.items()},
        "gt_manifest": str(gt_manifest),
        "xlsx_root": str(xlsx_root),
    }


def _flatten_ev2_summary(ev2_result):
    if not ev2_result:
        return {}
    ds = ev2_result["dataset_summary"]
    fr = ds.get("feature_reconstruction", {})
    metrics = ds.get("metrics", {})
    out = {
        "EV2_Dataset": ds.get("dataset"),
        "EV2_Samples": ds.get("sample_count"),
        "EV2_Instances": ds.get("instance_count"),
        "EV2_FR_percent": fr.get("FR_percent"),
        "EV2_FR_recovered": fr.get("recovered_count"),
        "EV2_FR_eligible": fr.get("gt_eligible_count"),
        "EV2_Audit": ev2_result.get("evaluation_audit", {}).get("result"),
    }
    for metric in ["AAE", "RRE", "ECD", "nECD"]:
        m = metrics.get(metric, {})
        out[f"EV2_{metric}_mean"] = m.get("mean")
        out[f"EV2_{metric}_eligible"] = m.get("gt_eligible_count")
        out[f"EV2_{metric}_evaluated"] = m.get("evaluated_count")
        out[f"EV2_{metric}_failed"] = m.get("skipped_or_failed_count")
    return out


def write_unified_run_outputs(args, output_summary, ev2_result):
    """Write one-row comparison summary + auditable run manifest."""
    pred_dir = Path(args.pred_dir).resolve()
    experiment_id = args.experiment_id or pred_dir.parent.name or pred_dir.name

    flat = {
        "Experiment_ID": experiment_id,
        "Unified_Protocol": UNIFIED_PROTOCOL,
        "Seed": int(args.seed),
        **output_summary,
        **_flatten_ev2_summary(ev2_result),
    }

    unified_csv = pred_dir / "evaluation_summary.csv"
    pd.DataFrame([flat]).to_csv(unified_csv, index=False)

    unified_json = pred_dir / "evaluation_summary.json"
    _write_json(unified_json, {
        "protocol": UNIFIED_PROTOCOL,
        "experiment_id": experiment_id,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "seed": int(args.seed),
        "output_level": output_summary,
        "ev2_feature_level": (
            ev2_result["dataset_summary"] if ev2_result else None
        ),
    })

    source_files = {"batch_evaluate": Path(__file__).resolve()}
    eval_step_path = Path(__file__).resolve().parent / "evaluate_step_metrics.py"
    if eval_step_path.is_file():
        source_files["evaluate_step_metrics"] = eval_step_path
    if ev2_result:
        for name, path in ev2_result.get("module_files", {}).items():
            source_files[f"ev2/{name}"] = Path(path)

    run_manifest = {
        "protocol": UNIFIED_PROTOCOL,
        "experiment_id": experiment_id,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "command": [str(x) for x in sys.argv],
        "python": sys.version,
        "platform": platform.platform(),
        "seed": int(args.seed),
        "mode": args.mode,
        "unit": args.unit,
        "geo_samples": int(args.geo_samples),
        "iou_samples": int(args.iou_samples),
        "deflection": float(args.deflection),
        "timeout": int(args.timeout),
        "gt_dir": str(Path(args.gt_dir).resolve()),
        "pred_dir": str(pred_dir),
        "ev2_enabled": bool(args.enable_ev2),
        "ev2_gt_manifest": (
            ev2_result.get("gt_manifest") if ev2_result else None
        ),
        "ev2_xlsx_root": (
            ev2_result.get("xlsx_root") if ev2_result else None
        ),
        "ev2_xlsx_policy": (
            ev2_result.get("xlsx_policy") if ev2_result else None
        ),
        "source_sha256": {
            name: _sha256_file(path)
            for name, path in source_files.items()
            if Path(path).is_file()
        },
        "outputs": {
            "evaluation_summary_csv": str(unified_csv),
            "evaluation_summary_json": str(unified_json),
            "ev2_run_manifest": (
                ev2_result.get("ev2_run_manifest") if ev2_result else None
            ),
        },
    }
    run_manifest_path = pred_dir / "run_manifest.json"
    _write_json(run_manifest_path, run_manifest)

    return {
        "evaluation_summary_csv": str(unified_csv),
        "evaluation_summary_json": str(unified_json),
        "run_manifest": str(run_manifest_path),
    }

def batch_process(args):
    np.random.seed(int(args.seed))

    # Optional feature-only regression mode reuses an existing output-level summary when available.
    if args.ev2_only:
        if not args.enable_ev2:
            raise RuntimeError("--ev2_only requires --enable_ev2.")

        ev2_result = run_ev2_feature_evaluation(args)
        old_summary_path = Path(args.pred_dir).resolve() / f"eval_summary_{args.mode}.csv"
        if old_summary_path.is_file():
            old_df = pd.read_csv(old_summary_path)
            output_summary = (
                old_df.iloc[0].to_dict()
                if len(old_df) > 0
                else {"OutputLevelSkipped": True}
            )
            output_summary["OutputLevelRecomputed"] = False
        else:
            output_summary = {
                "Mode": args.mode,
                "OutputLevelSkipped": True,
                "OutputLevelRecomputed": False,
            }

        unified_outputs = write_unified_run_outputs(
            args=args,
            output_summary=output_summary,
            ev2_result=ev2_result,
        )
        print("\n" + "=" * 78)
        print("EV2-ONLY REGRESSION COMPLETE")
        print(f"Unified summary CSV : {unified_outputs['evaluation_summary_csv']}")
        print(f"Unified summary JSON: {unified_outputs['evaluation_summary_json']}")
        print(f"Run manifest        : {unified_outputs['run_manifest']}")
        print("=" * 78)
        return
    
    # 1. Discover reference files
    if args.mode == 'cad2cad':
        gt_files = glob.glob(os.path.join(args.gt_dir, "*.step")) + glob.glob(os.path.join(args.gt_dir, "*.stp"))
    else:
        gt_files = []
        for e in ["*.txt", "*.asc", "*.ply", "*.obj", "*.xyz"]:
            gt_files.extend(glob.glob(os.path.join(args.gt_dir, e)))
    
    gt_files.sort()
    print(f"[{args.mode.upper()}] Dataset: {len(gt_files)} files")
    
    records = []
    invalid_count = 0
    timeout_count = 0
    eval_csv_dir = os.path.join(args.pred_dir, f"eval_final_{args.mode}.csv")

    # Use spawn on Windows; fork may be selected explicitly on Linux.
    mp_ctx = mp.get_context(args.mp_start_method)

    pbar = tqdm(gt_files, ncols=110, unit="file")
    for gt_path in pbar:
        file_id = os.path.splitext(os.path.basename(gt_path))[0]
        eval_seed = _stable_sample_seed(args.seed, file_id)
        row = {
            "Model_ID": file_id,
            "Valid": False,
            "Eval_Seed": eval_seed,
        }
        pred_path = None
        
        # 2. Locate prediction output
        for ext in ['.step', '.stp', '.obj', '.ply', '.stl']:
            p = os.path.join(args.pred_dir, file_id + ext)
            if os.path.exists(p): 
                pred_path = p
                row["Pred_Type"] = ext 
                break
        
        if not pred_path:
            invalid_count += 1
            row["Error"] = "MissingPrediction"
            row["Elapsed_s"] = 0.0
            records.append(row)
            if args.save_each:
                pd.DataFrame(records).to_csv(eval_csv_dir, index=False)
            continue

        # ===========================================
        # Evaluate each sample in an isolated process so a timeout can terminate the worker.
        # ===========================================
        start_t = time.time()
        payload = (
            gt_path, pred_path, args.mode,
            args.deflection, args.unit, args.geo_samples, args.iou_samples,
            eval_seed
        )
        res, error_type, error_detail = run_process_with_timeout(payload, args.timeout, mp_ctx)
        row['Elapsed_s'] = round(time.time() - start_t, 3)

        if error_type == 'Timeout':
            print(f"\n[Timeout] Skipping {file_id} (>{args.timeout}s, worker killed)")
            timeout_count += 1
            row['Valid'] = False
            row['Error'] = 'Timeout'
        elif error_type is not None:
            invalid_count += 1
            row['Valid'] = False
            row['Error'] = error_type
            if args.verbose_error and error_detail:
                print(f"\n[Error] {file_id}: {error_type} | {error_detail}")
        elif res:
            row.update(res)
            row['Valid'] = True
        else:
            invalid_count += 1
            row['Valid'] = False
            row['Error'] = 'NoResult'

        records.append(row)
        if args.save_each:
            pd.DataFrame(records).to_csv(eval_csv_dir, index=False)
        gc.collect()

    # 3. Aggregate results
    df = pd.DataFrame(records)
    if len(df) == 0:
        return

    eval_csv_dir = os.path.join(args.pred_dir, f"eval_final_{args.mode}.csv")
    summary_csv_dir = os.path.join(args.pred_dir, f"eval_summary_{args.mode}.csv")

    df.to_csv(eval_csv_dir, index=False)

    summary, valid_df = summarize_fullset_metrics(df, args.mode)
    pd.DataFrame([summary]).to_csv(summary_csv_dir, index=False)

    ev2_result = None
    if args.enable_ev2:
        ev2_result = run_ev2_feature_evaluation(args)

    unified_outputs = write_unified_run_outputs(
        args=args,
        output_summary=summary,
        ev2_result=ev2_result,
    )

    print("\n" + "=" * 78)
    print(f"FINAL EVALUATION REPORT | Mode: {args.mode} | Unit: {args.unit}")
    print("=" * 78)
    print(f"Total: {summary['Total']} | Valid: {summary['Valid']} | Invalid: {summary['Invalid']}")
    print(f"IR_all: {summary['IR_all(%)']:.2f}%")
    print(f"Raw CSV:     {eval_csv_dir}")
    print(f"Summary CSV: {summary_csv_dir}")

    if len(valid_df) > 0:
        print("-" * 45)
        print("[Surface Geometric Fidelity | conditional over evaluable outputs]")
        print(f"  CD Mean:          {valid_df['CD'].mean():.4f}")
        print(f"  CD Median:        {valid_df['CD'].median():.4f}")
        if 'HD' in valid_df:
            print(f"  HD Mean:          {valid_df['HD'].mean():.4f}")
            print(f"  HD Median:        {valid_df['HD'].median():.4f}")
        if 'NC' in valid_df and valid_df['NC'].notna().any():
            print(f"  NC Mean:          {valid_df['NC'].mean():.4f}")

        print("[F-Score Statistics | conditional over evaluable outputs]")
        for col in sorted(valid_df.columns):
            if 'F-Score@' in col:
                print(f"  {col:<16}: {valid_df[col].mean():.4f}")

        if args.mode == 'cad2cad':
            print("-" * 45)
            print("[Output Validity / CAD-kernel Deliverability]")
            print("  Main-table full-set metrics:")
            print(f"    M-WR_all:       {summary['M-WR_all(%)']:.2f}%")
            if not pd.isna(summary["S-WR_all(%)"]):
                print(f"    S-VR_all:       {summary['S-VR_all(%)']:.2f}%")
            else:
                print("    S-VR_all:       N/A (mesh-only output)")
            print(f"    M-IoU_all:      {summary['M-IoU_all']:.4f}")
            if not pd.isna(summary["S-IoU_all"]):
                print(f"    S-IoU_all:      {summary['S-IoU_all']:.4f}")
            else:
                print("    S-IoU_all:      N/A (mesh-only output)")

            print("  Conditional valid/computable metrics:")
            print(f"    M-WR_valid:     {summary['M-WR_valid(%)']:.2f}%")
            if not pd.isna(summary["S-WR_valid(%)"]):
                print(f"    S-VR_valid:     {summary['S-VR_valid(%)']:.2f}%")
            else:
                print("    S-VR_valid:     N/A")
            if not pd.isna(summary["M-IoU_valid"]):
                print(f"    M-IoU_valid:    {summary['M-IoU_valid']:.4f} "
                      f"(coverage all={summary['M-IoU_cover_all(%)']:.1f}%, "
                      f"valid={summary['M-IoU_cover_valid(%)']:.1f}%)")
            else:
                print("    M-IoU_valid:    N/A")
            if not pd.isna(summary["S-IoU_valid"]):
                print(f"    S-IoU_valid:    {summary['S-IoU_valid']:.4f} "
                      f"(coverage all={summary['S-IoU_cover_all(%)']:.1f}%, "
                      f"valid={summary['S-IoU_cover_valid(%)']:.1f}%)")
            else:
                print("    S-IoU_valid:    N/A")

            print("-" * 45)
            print("[Topology diagnostics]")
            if 'Face_Diff' in valid_df:
                print(f"  Face Diff:        {valid_df['Face_Diff'].mean():.2f}")
            if 'Valid_Topo' in valid_df:
                valid_topo = _as_bool_series(valid_df['Valid_Topo'])
                print(f"  Valid STEP Topo valid-only: {valid_topo.mean() * 100:.2f}%")
            if args.enable_ev2:
                print("  Engineering-feature AAE/RRE/ECD/nECD/FR: see pred_dir/ev2/")
            else:
                print("  Engineering-feature metrics: N/A (EV2 not enabled)")

        if args.mode == 'scan2cad':
            print("-" * 45)
            print("[Output Validity / CAD-kernel Deliverability]")
            print(f"  M-WR_all:         {summary['M-WR_all(%)']:.2f}%")
            print(f"  M-WR_valid:       {summary['M-WR_valid(%)']:.2f}%")
            if not pd.isna(summary["S-WR_all(%)"]):
                print(f"  S-VR_all:         {summary['S-VR_all(%)']:.2f}%")
                print(f"  S-VR_valid:       {summary['S-VR_valid(%)']:.2f}%")
            else:
                print("  S-VR_all:         N/A (mesh-only output)")
            if 'Valid_Topo' in valid_df:
                valid_topo = _as_bool_series(valid_df['Valid_Topo'])
                print(f"  Valid STEP Topo valid-only: {valid_topo.mean() * 100:.2f}%")

    else:
        print("[Warning] No valid/evaluable outputs. Only full-set invalid ratio is available.")

    print("-" * 45)
    print(f"Unified summary CSV : {unified_outputs['evaluation_summary_csv']}")
    print(f"Unified summary JSON: {unified_outputs['evaluation_summary_json']}")
    print(f"Run manifest        : {unified_outputs['run_manifest']}")
    print("=" * 78)

if __name__ == "__main__":
    # Multiprocessing-safe entry point for Windows.
    mp.freeze_support()
    parser = argparse.ArgumentParser(
        description=(
            "Unified RbRM output-level evaluator with optional frozen "
            "Evaluation V2 engineering-feature metrics."
        )
    )
    parser.add_argument('--gt_dir', type=str, required=True)
    parser.add_argument('--pred_dir', type=str, required=True)
    parser.add_argument('--geo_samples', type=int, default=30000, help="Points for CD/HD/NC/F-Score")
    parser.add_argument('--iou_samples', type=int, default=10000, help="Points for Monte Carlo IoU (Cad2Cad only)")
    parser.add_argument('--mode', type=str, choices=['cad2cad', 'scan2cad'], required=True)
    parser.add_argument('--unit', type=str, default='mm', choices=['mm', 'm'])
    parser.add_argument('--deflection', type=float, default=0.01)
    parser.add_argument('--timeout', type=int, default=300, help="Timeout in seconds for single model eval")
    parser.add_argument('--mp_start_method', type=str, default='spawn', choices=['spawn', 'fork', 'forkserver'],
                        help="Multiprocessing start method. Use spawn on Windows; fork can be faster on Linux.")
    parser.add_argument('--save_each', action='store_true', default=True,
                        help="Save partial CSV after each model, useful when some CAD kernels crash/hang.")
    parser.add_argument('--verbose_error', action='store_true',
                        help="Print worker exception details for debugging invalid samples.")

    # Reproducibility and experiment identity
    parser.add_argument('--seed', type=int, default=2026,
                        help="Deterministic base seed. Each sample derives a stable seed from this value.")
    parser.add_argument('--experiment_id', type=str, default=None,
                        help="Optional stable experiment name written to evaluation_summary/run_manifest.")

    # Engineering-feature metrics
    parser.add_argument('--enable_ev2', action='store_true',
                        help="Enable frozen EV2 exact-UID engineering-feature evaluation.")
    parser.add_argument('--ev2_only', action='store_true',
                        help="Only recompute EV2 feature metrics; skip output-level geometry/IoU evaluation.")
    parser.add_argument('--ev2_dataset', type=str, choices=['CADParser', 'DeepCAD'], default='CADParser')
    parser.add_argument('--ev2_gt_manifest', type=str, default=None,
                        help="Frozen GT engineering-feature manifest CSV, e.g. CADParser_full_v1.4/gt_feature_manifest.csv.")
    parser.add_argument('--ev2_xlsx_root', type=str, default=None,
                        help="Run root containing per-sample PartFeatures XLSX. Default: parent of predictions_for_eval.")
    parser.add_argument('--ev2_xlsx_policy', type=str, default='prefer-root',
                        choices=['strict-equivalent', 'prefer-root', 'prefer-output', 'root-only', 'output-only'],
                        help=("How to resolve duplicate per-sample XLSX files. "
                              "Default prefer-root: use <sample>/<sample>.xlsx when present; "
                              "if absent, use output/<sample>.xlsx or the only available "
                              "exact-name candidate. Conflicts are recorded in the EV2 audit."))
    parser.add_argument('--ev2_out_dir', type=str, default=None,
                        help="EV2 output directory. Default: <pred_dir>/ev2.")
    parser.add_argument('--ev2_module_dir', type=str, default=None,
                        help="Directory containing prediction_adapters.py, join_feature_instances.py, evaluate_feature_instances.py, metric_definitions.py.")
    parser.add_argument('--ev2_sheet', type=str, default='PartFeatures')
    parser.add_argument('--ev2_manifest_schema', type=str, default=DEFAULT_EV2_MANIFEST_SCHEMA)
    parser.add_argument('--ev2_expected_samples', type=int, default=None)
    parser.add_argument('--ev2_expected_features', type=int, default=None)
    parser.add_argument('--ev2_strict', dest='ev2_strict', action='store_true', default=True,
                        help="Enable strict EV2 integrity gates (default).")
    parser.add_argument('--no_ev2_strict', dest='ev2_strict', action='store_false',
                        help="Disable nonessential EV2 strict integrity gates; not recommended for paper results.")
    parser.add_argument('--ev2_fail_on_missing_xlsx', action='store_true',
                        help="Treat a missing per-sample structured XLSX as a pipeline error instead of a legitimate FR failure.")

    args = parser.parse_args()
    if args.enable_ev2 and not args.ev2_gt_manifest:
        parser.error('--ev2_gt_manifest is required when --enable_ev2 is used.')

    batch_process(args)