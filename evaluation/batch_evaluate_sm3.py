import os
import glob
import sys
import argparse
import pandas as pd
import numpy as np
from tqdm import tqdm
import trimesh
import tempfile
import uuid
import gc
import math
from scipy.spatial import cKDTree
import multiprocessing as mp  # [修正] 用独立进程实现可强制终止的超时控制
import queue as queue_mod
import traceback
import time
import open3d as o3d

# ==========================================
# 0. 环境与依赖检查
# ==========================================
try:
    from evaluate_step_metrics import (
        StepLoader, TopologyEvaluator, FeatureParamEvaluator
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
    print("          STEP 解析、Feature Recall、拓扑检查将被跳过。")
    print("="*60)

# ==========================================
# 1. 数据加载器
# ==========================================
class DataLoader:
    @staticmethod
    def load_scan_points(path, estimate_missing_normals=False):
        """
        [Scan Mode Only] 加载点云并尝试提取法向。
        支持：
        - txt/xyz/asc: x y z nx ny nz
        - ply/obj/stl/off: 优先用 Open3D 读取 point cloud normals，失败再用 trimesh
        """
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
        """[Both Modes] 加载 STEP/Mesh (含 Mesh 修复)"""
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
    """
    计算 Shape 中有效 Solid 的比例。
    注意：这里的 "watertight" 更准确地说是 CAD/B-Rep 层面的有效实体比例，
    与 trimesh.is_watertight 的三角网格水密性不是同一概念。

    返回: (有效 Solid 数量, 总 Solid 数量, 比例)
    """
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
    """
    判断一个 STEP Shape 是否至少包含 1 个 Solid，且所有 Solid 都是有效实体。
    用于决定是否可以计算 B-Rep/Solid 体积 IoU。
    """
    n_valid, n_total, ratio = compute_solid_watertight_ratio(shape)
    return (n_total > 0) and (n_valid == n_total), n_valid, n_total, ratio


def get_shape_or_mesh_bounds(shape, mesh=None):
    """
    获取 CAD/mesh 联合包围盒。优先使用 OCC 的 Shape 包围盒，失败时回退到 mesh.bounds。
    返回: min_b, max_b，shape 为 (3,) 的 numpy 数组。
    """
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
# 2. 评估引擎
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

        # Mesh IoU：沿用原始逻辑，只有 GT/Pred 三角网格都水密时才可信。
        # 非水密时不再强行记为 0，而是记为 NaN，避免把“不可计算”误认为“完全无交集”。
        if metrics['SR_GT'] and metrics['SR_Pred']:
            mesh_iou = self._compute_iou(self.iou_samples)
            metrics['Mesh_IoU'] = mesh_iou
            metrics['IoU'] = mesh_iou  # 保留旧字段，兼容后续汇总代码
            metrics['Mesh_IoU_Computed'] = np.isfinite(mesh_iou)
        else:
            metrics['Mesh_IoU'] = np.nan
            metrics['IoU'] = np.nan
            metrics['Mesh_IoU_Computed'] = False

        # Strict Mesh IoU：不可计算样本按 0 计入，用于全样本公平统计。
        metrics['Mesh_IoU_Strict'] = (
            float(metrics['Mesh_IoU'])
            if metrics['Mesh_IoU_Computed']
            else 0.0
        )

        # CAD/B-Rep Solid 层面的有效实体比例。
        # 新增 GT 和 Pred 两侧统计，原 Solid_Watertight_Ratio 字段继续保留为 Pred 侧，避免影响旧汇总。
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
            # Mesh-only outputs are not STEP/B-Rep solids.
            # They should be marked as N/A for S-WR/S-IoU rather than counted as solid-valid.
            pred_solid_ok = False
            metrics['Solid_Watertight_Ratio_Pred'] = np.nan
            metrics['Solid_Watertight_Ratio'] = np.nan
            metrics['Num_Solids_Pred'] = 0
            metrics['Num_Solids'] = 0

        # Solid IoU：当 GT 和 Pred 都是有效 Solid 时，直接基于 STEP/B-Rep 做体积占据判断。
        # 这不依赖 STL 三角网格是否 watertight，适合 STEP-compatible CAD solid 的体积 IoU 评价。
        if gt_solid_ok and pred_solid_ok:
            metrics['Solid_IoU'] = self._compute_solid_iou(self.iou_samples)
            metrics['Solid_IoU_Computed'] = np.isfinite(metrics['Solid_IoU'])
        else:
            metrics['Solid_IoU'] = np.nan
            metrics['Solid_IoU_Computed'] = False

        # Strict Solid IoU：不可计算样本按 0 计入，用于全样本公平统计。
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
                
                param_eval = FeatureParamEvaluator(self.gt_shape, self.pred_shape)
                p_res = param_eval.compute_metrics()
                metrics['AAE'] = p_res.get('AAE', np.nan)
                metrics['RRE'] = p_res.get('RRE', np.nan)
                metrics['ECD'] = p_res.get('ECD', np.nan)
                if p_res['GT_Count'] > 0:
                    metrics['Feature_Recall'] = p_res['Match_Count'] / p_res['GT_Count']
                else:
                    metrics['Feature_Recall'] = np.nan
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

            # 加一个极小 padding，避免采样盒边界刚好压在面上造成分类不稳定。
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
        
        # [新增] 计算实体级水密比例
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
# 3. 单文件处理逻辑 (封装供线程调用)
# ==========================================
def process_single_file(gt_path, pred_path, mode, deflection, unit, geo_samples, iou_samples):
    """
    单个文件的完整评估流程
    """
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
        # print(f"DEBUG: Exception in worker: {e}")
        return None

# ==========================================
# 4. 真正可终止的单文件超时执行器
# ==========================================
def _process_worker(result_queue, payload):
    """
    子进程入口。
    注意：必须保持在模块顶层，Windows 的 spawn 启动方式才能正常 pickle。
    """
    try:
        gt_path, pred_path, mode, deflection, unit, geo_samples, iou_samples = payload
        # 每个子进程单独设置随机种子，避免 Monte Carlo IoU 完全重复。
        seed = (os.getpid() + int(time.time() * 1000)) % (2**32 - 1)
        np.random.seed(seed)
        res = process_single_file(gt_path, pred_path, mode, deflection, unit, geo_samples, iou_samples)
        result_queue.put(("ok", res))
    except BaseException as e:
        result_queue.put(("error", {
            "error": repr(e),
            "traceback": traceback.format_exc()
        }))


def run_process_with_timeout(payload, timeout, mp_ctx):
    """
    使用独立进程执行单个模型评估。
    与 ThreadPoolExecutor.future.result(timeout=...) 不同，超时后会 terminate/kill 子进程，
    因此 OCC / trimesh / rtree 等 C++ 扩展库卡死时不会继续阻塞后续样本。

    返回:
        res, error_type, error_detail
    """
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
        # 子进程刚退出时，Queue 的 feeder 线程可能还在刷新，稍等一下更稳。
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
# 5. 主流程 (含超时控制)
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
    also treated as failed solid outputs in full-set S-WR/S-IoU.
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
    # Feature Recall: full-set and valid-only
    # -----------------------------
    if "Feature_Recall" in df.columns:
        fr = _num_series(df, "Feature_Recall", default=np.nan)
        summary["FR_all(%)"] = 100.0 * fr.fillna(0.0).mean()
        summary["FR_valid(%)"] = 100.0 * fr[fr.notna()].mean() if fr.notna().any() else np.nan
    else:
        summary["FR_all(%)"] = np.nan
        summary["FR_valid(%)"] = np.nan

    return summary, valid_df

def batch_process(args):
    np.random.seed(2026) 
    
    # 1. GT 搜索
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

    # Windows 下必须使用 spawn；Linux 下如需更快可手动指定 fork。
    mp_ctx = mp.get_context(args.mp_start_method)

    pbar = tqdm(gt_files, ncols=110, unit="file")
    for gt_path in pbar:
        file_id = os.path.splitext(os.path.basename(gt_path))[0]
        row = {"Model_ID": file_id, "Valid": False}
        pred_path = None
        
        # 2. Pred 搜索
        for ext in ['.step', '.stp', '.obj', '.ply', '.stl']:
            p = os.path.join(args.pred_dir, file_id + ext)
            if os.path.exists(p): 
                pred_path = p
                row["Pred_Type"] = ext 
                break
        
        if not pred_path:
            invalid_count += 1
            records.append(row); continue

        # ===========================================
        # [修正] 超时控制逻辑：每个样本放到独立进程中执行，超时后强制终止
        # ===========================================
        start_t = time.time()
        payload = (
            gt_path, pred_path, args.mode,
            args.deflection, args.unit, args.geo_samples, args.iou_samples
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

    # 3. 结果汇总
    df = pd.DataFrame(records)
    if len(df) == 0:
        return

    eval_csv_dir = os.path.join(args.pred_dir, f"eval_final_{args.mode}.csv")
    summary_csv_dir = os.path.join(args.pred_dir, f"eval_summary_{args.mode}.csv")

    df.to_csv(eval_csv_dir, index=False)

    summary, valid_df = summarize_fullset_metrics(df, args.mode)
    pd.DataFrame([summary]).to_csv(summary_csv_dir, index=False)

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
                print(f"    S-WR_all:       {summary['S-WR_all(%)']:.2f}%")
            else:
                print("    S-WR_all:       N/A (mesh-only output)")
            print(f"    M-IoU_all:      {summary['M-IoU_all']:.4f}")
            if not pd.isna(summary["S-IoU_all"]):
                print(f"    S-IoU_all:      {summary['S-IoU_all']:.4f}")
            else:
                print("    S-IoU_all:      N/A (mesh-only output)")

            print("  Conditional valid/computable metrics:")
            print(f"    M-WR_valid:     {summary['M-WR_valid(%)']:.2f}%")
            if not pd.isna(summary["S-WR_valid(%)"]):
                print(f"    S-WR_valid:     {summary['S-WR_valid(%)']:.2f}%")
            else:
                print("    S-WR_valid:     N/A")
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
            print("[Feature-level and Parametric Consistency]")
            if 'Face_Diff' in valid_df:
                print(f"  Face Diff:        {valid_df['Face_Diff'].mean():.2f}")
            if 'AAE' in valid_df:
                print(f"  AAE (Angle):      {valid_df['AAE'].mean():.4f}")
            if 'RRE' in valid_df:
                print(f"  RRE (Radius):     {valid_df['RRE'].mean():.4f}")
            if 'ECD' in valid_df:
                print(f"  ECD (Center):     {valid_df['ECD'].mean():.4f}")
            if not pd.isna(summary["FR_all(%)"]):
                print(f"  Feature Recall all:   {summary['FR_all(%)']:.2f}%")
                print(f"  Feature Recall valid: {summary['FR_valid(%)']:.2f}%")
            if 'Valid_Topo' in valid_df:
                valid_topo = _as_bool_series(valid_df['Valid_Topo'])
                print(f"  Valid STEP Topo valid-only: {valid_topo.mean() * 100:.2f}%")

        if args.mode == 'scan2cad':
            print("-" * 45)
            print("[Output Validity / CAD-kernel Deliverability]")
            print(f"  M-WR_all:         {summary['M-WR_all(%)']:.2f}%")
            print(f"  M-WR_valid:       {summary['M-WR_valid(%)']:.2f}%")
            if not pd.isna(summary["S-WR_all(%)"]):
                print(f"  S-WR_all:         {summary['S-WR_all(%)']:.2f}%")
                print(f"  S-WR_valid:       {summary['S-WR_valid(%)']:.2f}%")
            else:
                print("  S-WR_all:         N/A (mesh-only output)")
            if 'Valid_Topo' in valid_df:
                valid_topo = _as_bool_series(valid_df['Valid_Topo'])
                print(f"  Valid STEP Topo valid-only: {valid_topo.mean() * 100:.2f}%")

    else:
        print("[Warning] No valid/evaluable outputs. Only full-set invalid ratio is available.")

    print("=" * 78)

if __name__ == "__main__":
    # Windows 多进程安全入口。
    mp.freeze_support()
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt_dir', type=str, required=True)
    parser.add_argument('--pred_dir', type=str, required=True)
    parser.add_argument('--geo_samples', type=int, default=30000, help="Points for CD/HD/NC/F-Score")
    parser.add_argument('--iou_samples', type=int, default=10000, help="Points for Monte Carlo IoU (Cad2Cad only)")
    parser.add_argument('--mode', type=str, choices=['cad2cad', 'scan2cad'], required=True)
    parser.add_argument('--unit', type=str, default='mm', choices=['mm', 'm'])
    parser.add_argument('--deflection', type=float, default=0.01)
    # [修正] 超时参数：超时后会杀掉子进程，而不是仅停止等待。
    parser.add_argument('--timeout', type=int, default=300, help="Timeout in seconds for single model eval")
    parser.add_argument('--mp_start_method', type=str, default='spawn', choices=['spawn', 'fork', 'forkserver'],
                        help="Multiprocessing start method. Use spawn on Windows; fork can be faster on Linux.")
    parser.add_argument('--save_each', action='store_true', default=True,
                        help="Save partial CSV after each model, useful when some CAD kernels crash/hang.")
    parser.add_argument('--verbose_error', action='store_true',
                        help="Print worker exception details for debugging invalid samples.")
    args = parser.parse_args()
    batch_process(args)